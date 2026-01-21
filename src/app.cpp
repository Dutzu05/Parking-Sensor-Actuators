#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

volatile uint8_t ms = 0;
volatile uint16_t distance_cm = 0;

volatile uint16_t icr_start = 0;
volatile uint16_t icr_end = 0;
volatile uint8_t capture_state = 0;

volatile uint16_t buzzer_ms = 0;

/* Buttons: PC0 (A0) = CW, PC1 (A1) = CCW  (active LOW, pull-up)
   Motor pins: PB1 = CW, PB2 = CCW */

#define BTN_CW_MASK     0b00000001   // PC0
#define BTN_CCW_MASK    0b00000010   // PC1

#define MOTOR_CW_MASK   0b00000010   // PB1
#define MOTOR_CCW_MASK  0b00000100   // PB2

static inline void motor_stop(void)
{
    PORTB &= (uint8_t)~(MOTOR_CW_MASK | MOTOR_CCW_MASK);
}

static inline void motor_cw(void)
{
    PORTB |= MOTOR_CW_MASK;
    PORTB &= (uint8_t)~MOTOR_CCW_MASK;
}

static inline void motor_ccw(void)
{
    PORTB |= MOTOR_CCW_MASK;
    PORTB &= (uint8_t)~MOTOR_CW_MASK;
}

static inline void motor_buttons_update(void)
{
    uint8_t buttons = PINC;

    uint8_t cw_pressed  = ((buttons & BTN_CW_MASK) == 0);
    uint8_t ccw_pressed = ((buttons & BTN_CCW_MASK) == 0);

    if (cw_pressed && !ccw_pressed && distance_cm > 10)
        motor_cw();
    else if (ccw_pressed && !cw_pressed && distance_cm > 10)
        motor_ccw();
    else
        motor_stop();
}

/* PD2 = Green (always ON)
   PD4 = Red   (<10 cm)
   PD7 = Yellow (<20 cm) */

static inline void leds_update(uint16_t dist)
{
    // Green always ON
    PORTD |= 0b00000100; // PD2

    // Clear PD4 and PD7 (do NOT touch PD3)
    PORTD &= 0b01101111; // keep PD3 intact

    if (dist == 0)
        return;

    if (dist < 20)
        PORTD |= 0b10000000; // PD7

    if (dist < 10)
        PORTD |= 0b00010000; // PD4
}

static inline void buzzer_set(uint8_t volume)
{
    OCR2B = volume; // 0 = OFF
}

static inline void ultrasonic_trigger(void)
{
    PORTB &= 0b11110111; // PB3 LOW
    _delay_us(2);

    PORTB |= 0b00001000; // PB3 HIGH
    _delay_us(10);

    PORTB &= 0b11110111; // PB3 LOW
}

ISR(TIMER1_CAPT_vect)
{
    if (capture_state == 0)
    {
        icr_start = ICR1;
        TCCR1B &= 0b10111111; // falling edge
        capture_state = 1;
    }
    else
    {
        icr_end = ICR1;
        TCCR1B |= 0b01000000; // rising edge
        capture_state = 0;

        uint16_t ticks = icr_end - icr_start;
        distance_cm = ticks / 116;
    }
}

ISR(TIMER0_COMPA_vect)
{
    if (++ms >= 50)
    {
        ultrasonic_trigger();
        ms = 0;
    }

    leds_update(distance_cm);

    // motor direction control from buttons
    motor_buttons_update();

    uint16_t interval;

    if (distance_cm == 0)
    {
        buzzer_set(0);
        buzzer_ms = 0;
        return;
    }

    if (distance_cm > 20)
        interval = 600;
    else if (distance_cm >= 10)
        interval = 250;
    else
        interval = 100;

    if (++buzzer_ms >= interval)
    {
        buzzer_ms = 0;

        if (OCR2B)
            buzzer_set(0);
        else
            buzzer_set(80); // quiet volume
    }
}

void init_gpio(void)
{
    // PD2, PD4, PD7 outputs (LEDs)
    DDRD |= 0b10010100;

    // PD3 output (buzzer, OC2B)
    DDRD |= 0b00001000;

    // Start LEDs and buzzer OFF
    PORTD &= 0b01100011;

    // PB3 output (TRIG)
    DDRB |= 0b00001000;

    // PB0 input (ECHO ICP1)
    DDRB &= 0b11111110;

    // motor pins PB1, PB2 outputs
    DDRB |= (MOTOR_CW_MASK | MOTOR_CCW_MASK);
    motor_stop();

    // buttons PC0, PC1 inputs + pull-ups enabled
    DDRC &= (uint8_t)~(BTN_CW_MASK | BTN_CCW_MASK);
    PORTC |= (BTN_CW_MASK | BTN_CCW_MASK);
}

void init_timer0(void)
{
    TCCR0A = 0b00000010; // CTC
    TCCR0B = 0b00000011; // /64
    OCR0A = 249;         // 1 ms
    TIMSK0 = 0b00000010;
}

void init_timer1_icp(void)
{
    TCCR1A = 0b00000000;
    TCCR1B = 0b01000010; // rising edge + /8
    TIMSK1 = 0b00100000;
}

void init_timer2_pwm(void)
{
    // Fast PWM, TOP = OCR2A, non-inverting on OC2B (PD3)
    TCCR2A = 0b00100011; // COM2B1=1, WGM21=1, WGM20=1
    TCCR2B = 0b00001100; // WGM22=1, prescaler /32

    OCR2A = 249; // ~2 kHz
    OCR2B = 0;
}

// Timer2 PWM for buzzer on PD3 (OC2B):
// - Fast PWM mode with TOP=OCR2A (WGM=7) => OCR2A sets frequency (tone)
// - Non-inverting on OC2B (COM2B1=1)     => PD3 HIGH then LOW at OCR2B (duty cycle)
// - Prescaler /32                       => timer clock = F_CPU/32
// - OCR2A=249 => ~2 kHz tone, OCR2B controls volume (0=silent)

int main(void)
{
    init_gpio();
    init_timer0();
    init_timer1_icp();
    init_timer2_pwm();

    sei();

    while (1)
    {
    }
}
