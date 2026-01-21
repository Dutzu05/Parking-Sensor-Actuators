# Parking Sensor with Actuators

A bare-metal embedded parking sensor system using **direct register manipulation** on the ATmega328P microcontroller.  The system measures distance using an ultrasonic sensor (HC-SR04) and provides visual (LEDs) and audible (buzzer) feedback with variable warning intensity based on proximity.

## 🎯 Project Overview

This project implements a real-time distance measurement system with multi-peripheral coordination: 
- **Ultrasonic distance sensing** using Timer1 Input Capture
- **PWM-driven buzzer** with frequency and duty cycle control via Timer2
- **Multi-LED warning system** with distance-based thresholds
- **Millisecond-precision scheduler** using Timer0 CTC mode

## 🔧 Hardware Components

| Component | Pin | Function |
|-----------|-----|----------|
| HC-SR04 Trigger | PB3 | Ultrasonic pulse generation |
| HC-SR04 Echo | PB0 (ICP1) | Input Capture for pulse measurement |
| Green LED | PD2 | System active indicator |
| Yellow LED | PD7 | Warning (<20 cm) |
| Red LED | PD4 | Critical warning (<10 cm) |
| Buzzer | PD3 (OC2B) | PWM audio feedback |

## ⚙️ Technical Architecture

### Timer Configuration

#### Timer0 - System Scheduler (CTC Mode)
```c
TCCR0A = 0b00000010; // CTC mode
TCCR0B = 0b00000011; // Prescaler /64
OCR0A = 249;         // 1ms tick at 16MHz
```
**Purpose**: Generates precise 1ms interrupts for task scheduling (sensor polling every 50ms, LED updates, buzzer timing)

#### Timer1 - Input Capture (Rising/Falling Edge Detection)
```c
TCCR1B = 0b01000010; // Rising edge trigger + prescaler /8
TIMSK1 = 0b00100000; // Enable Input Capture interrupt
```
**Purpose**: Hardware-based pulse width measurement of ultrasonic echo signal with 0.5µs resolution

#### Timer2 - Fast PWM for Buzzer (Phase-Correct PWM)
```c
TCCR2A = 0b00100011; // Fast PWM, TOP=OCR2A, non-inverting OC2B
TCCR2B = 0b00001100; // Prescaler /32
OCR2A = 249;         // ~2kHz frequency
OCR2B = 80;          // Duty cycle (volume control)
```
**Purpose**: Generates 2kHz audio tone with dynamic volume control (0-255 duty cycle range)

### Distance Calculation

```c
uint16_t ticks = icr_end - icr_start;
distance_cm = ticks / 116;
```
**Formula**: With Timer1 prescaler /8 at 16MHz → 2MHz tick rate → 0.5µs per tick  
Sound travels at ~343 m/s → 29.1µs/cm round trip → 116 ticks/cm

## 🚀 Why Register-Level Programming?

### 1. **Hardware Control Precision**
Direct register access provides **bit-level control** over peripheral behavior.  For example, switching Timer1 edge detection in real-time: 
```c
TCCR1B &= 0b10111111; // Switch to falling edge (clear ICES1 bit)
TCCR1B |= 0b01000000; // Switch to rising edge (set ICES1 bit)
```
This enables dynamic edge detection for accurate pulse width measurement **within the same ISR** — impossible with abstraction layers.

### 2. **Deterministic Timing**
Register operations execute in **1-2 CPU cycles**, critical for: 
- **10µs ultrasonic trigger pulse** generation with cycle-accurate timing
- **ISR latency minimization** (<5µs entry time)
- **Jitter-free PWM** generation without library overhead

### 3. **Resource Optimization**
- **Flash**:  ~1-2KB vs. 10-15KB with Arduino libraries
- **RAM**: No hidden buffer allocations or vtables
- **No runtime overhead**: Compile-time register addresses, zero abstraction penalty

### 4. **Multi-Timer Coordination**
Configuring **three simultaneous timer modes** (CTC, Input Capture, Fast PWM) requires understanding hardware capabilities that high-level APIs obscure: 
```c
init_timer0();      // 1ms system tick
init_timer1_icp();  // Pulse width measurement
init_timer2_pwm();  // Audio generation
```
Each timer operates independently with custom prescalers, modes, and interrupt sources.

### 5. **Educational Value**
Understanding **datasheet-to-code translation**:
- Interpreting timing diagrams
- Calculating prescaler values for exact frequencies
- Debugging with oscilloscope/logic analyzer correlation
- Prerequisite knowledge for **RTOS development** and **professional embedded systems**

## 📊 System Behavior

| Distance | Yellow LED | Red LED | Buzzer Interval |
|----------|------------|---------|-----------------|
| >20 cm   | OFF | OFF | 600ms (slow beep) |
| 10-20 cm | ON | OFF | 250ms (medium beep) |
| <10 cm   | ON | ON | 100ms (fast beep) |
|
Green LED remains **always ON** as a power/system active indicator.

## 🛠️ Build & Flash

```bash
# Using PlatformIO
pio run --target upload

# Using avr-gcc directly
avr-gcc -mmcu=atmega328p -DF_CPU=16000000UL -Os -o app.elf src/app.cpp
avr-objcopy -O ihex app.elf app.hex
avrdude -c arduino -p m328p -P /dev/ttyUSB0 -U flash:w:app.hex
```

## 📚 Key Takeaways

This project demonstrates: 
- **Interrupt-driven architecture** without blocking delays
- **Mixed-mode timer operation** (CTC + Input Capture + Fast PWM)
- **Fixed-point arithmetic** for efficient distance calculation
- **Volatile semantics** for ISR-main thread communication
- **Bitwise operations** for GPIO manipulation without read-modify-write hazards

---

**Platform**: ATmega328P @ 16MHz  
**Toolchain**: AVR-GCC + PlatformIO  
**Language**: C with AVR libc  
**Programming Style**: Bare-metal register-level manipulation