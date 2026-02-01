# MCU-Based 100W LED Driver Control System

## Overview
This project implements a **microcontroller-based control system for a 100W LED driver** using an STM32 microcontroller.  
The system supports **multiple user inputs** (UART, potentiometer via ADC, and a push button) to control LED brightness from **0–100%**, with **smooth and gradual transitions** between brightness levels.

A **priority-based input handling mechanism** ensures that the **most recent control input** determines the LED behavior.

---

## Features
- LED brightness control from **0% to 100%**
- Smooth PWM-based brightness transitions
- Multiple input interfaces:
  - **UART commands**
  - **Potentiometer (ADC)**
  - **Push button (GPIO + EXTI)**
- Input priority handling (latest input wins)
- Bare-metal register-level STM32 programming
- Modular driver structure

---

## Hardware Components
- **Microcontroller**: STM32 (STM32F4 series)
- **LED Driver**: External 100W LED driver (PWM controlled)
- **Potentiometer**: Analog brightness input
- **Push Button**: ON/OFF toggle
- **UART Interface**: Serial command interface
- **MOSFET / Driver Stage**: LED power switching

---

## Software Architecture

The firmware is organized into **three logical layers**:

### Application Layer
- Input priority management
- System state control
- Brightness control logic

### Middleware Layer
- UART command processing
- ADC value processing
- Button handling
- Brightness ramp controller

### Hardware Abstraction Layer (HAL-like)
- PWM driver (Timer)
- ADC driver (interrupt-based)
- UART driver
- GPIO / EXTI driver

---

## Input Priority Logic
When multiple inputs are available, the system follows this priority rule:

1. **UART** (highest priority)
2. **ADC (Potentiometer)**
3. **Button**

The **most recent input source** always overrides previous inputs.

---

## Brightness Control Model

Two brightness variables are used:

- `targetBrightness`  
  → Desired brightness set by user input

- `currentBrightness`  
  → Actual applied PWM duty cycle

The system gradually adjusts `currentBrightness` toward `targetBrightness` to ensure **smooth LED transitions**.

---

## UART Command Interface

| Command | Action |
|------|------|
| `0` | Turn LED OFF |
| `1` | Turn LED ON (100% brightness) |
| `0–100` | Set LED brightness directly (percentage) |
| `P` | Print current brightness |

---

## ADC Behavior
- ADC reads potentiometer value (0–4095)
- Value is scaled to **0–100%**
- Updates `targetBrightness`
- Used when UART input is not active

---

## Button Behavior
- Toggles LED ON/OFF
- Does not change stored brightness
- LED ramps back to last brightness when turned ON

---

## PWM Configuration
- PWM generated using STM32 Timer peripheral
- Duty cycle range: **0–100**
- PWM output controls LED driver gate signal

