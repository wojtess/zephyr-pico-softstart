# Softstart Current Control - Feature Specification

## Overview
Precision current regulation with softstart ramp-up for RP2040 embedded system.

## Hardware Setup
```
        ┌─────────┐     ┌──────┐     ┌─────────┐
Setpoint→│  PID    │───→──│  PWM │───→──│ MOSFET  │───→ I_obciążenie
         │ (ster.) │     │ (buj)│     │ (klucz) │      ↑
         └─────────┘     └──────┘     └─────────┘      │
                                                           │
                                              ┌─────────────────────┘
                                              │
                                              ↓
                                        ┌───────────┐
                                        │ Shunt +   │
                                        │ OpAmp →   │───→ Measured (ADC)
                                        │ Filtr     │
                                        └───────────┘
```

## Requirements

### 1. Softstart Mode (Option B)
- **Target current** (Setpoint) set via GUI (e.g., 2.0A)
- **Smooth ramp** from 0A → Setpoint over configurable time (e.g., 1s)
- **1% precision** on final current value

### 2. Mode Switching
- **Manual Mode**: PWM set directly from GUI (current behavior)
- **PID Mode**: Automatic current control with softstart
- **Bumpless transfer**: No PWM jumps when switching modes

### 3. PID Background Operation
- PID thread runs continuously in firmware (independent of GUI)
- GUI can be closed - PID continues working
- Future: Physical knob interface (very distant future)

### 4. Data Visualization
- Real-time plot showing:
  - **Setpoint** (target current) - e.g., red line
  - **Measured** (actual ADC current) - e.g., blue line
  - Future: P, I, D component values

## Protocol Extensions (TBD)
- `0x06`: SET_PID_MODE (manual=0, pid=1)
- `0x07`: SET_PID_SETPOINT (float - target current in Amps)
- `0x08`: SET_PID_PARAMETERS (Kp, Ki, Kd)
- `0x09`: SET_SOFTSTART_TIME (ramp duration in ms)
- `0x0A`: GET_PID_STATUS (returns mode, setpoint, measured current, PID output)

## Firmware Architecture

### Thread Priorities (Zephyr RTOS)
```
PID Thread:     priority 5-7  (high - deterministic control)
ADC Processing: priority 10  (medium)
GUI Comm:       priority 15  (low)
```

### Bumpless Transfer Implementation
```c
// During Manual mode - PID calculates but doesn't control
// Integrator "tracks" manual output to be ready for smooth transition
if (mode == MANUAL) {
    integrator = manual_pwm;  // Track for bumpless transfer
    // PWM NOT updated by PID
} else {
    // PID controls normally
    pwm = pid_calculate(setpoint, measured);
}
```

## Current Status
- [x] ADC streaming (100ms intervals)
- [x] Manual PWM control (0-100%)
- [x] Real-time plot with toggle-able channels
- [ ] PID module implementation
- [ ] Softstart ramp logic
- [ ] Mode switching (Manual ↔ PID)
- [ ] Protocol extensions
- [ ] GUI PID controls
- [ ] GUI dual-line plot (setpoint + measured)

## Open Questions
- What is the current range? (0 to ? Amps)
- What is the typical setpoint range?
- What softstart ramp time is typical? (100ms? 1s? 10s?)
- ADC voltage-to-current conversion formula?
