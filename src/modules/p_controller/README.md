# P-Controller Module

## Overview

P-controller module for precision current regulation on RP2040 (Raspberry Pi Pico) using Zephyr RTOS.

## Control Formula

```
error = setpoint - measured
correction = (error * gain) / 100
pwm = clamp(feed_forward + correction, 0, 100)
```

## Features

- **1 kHz control loop** (1ms interval)
- **Two operation modes**:
  - MANUAL: Direct PWM control via SET_PWM command
  - AUTO: Automatic P-control based on ADC feedback
- **Configurable parameters**:
  - `setpoint`: Target ADC value (0-4095)
  - `gain`: Proportional gain (0-1000, represents 0.0-10.0)
  - `feed_forward`: Base PWM bias (0-100%)
- **Streaming support**: Send control data at configurable rates
- **Thread-safe**: Atomic operations for all parameters accessed in ISR

## File Structure

```
src/modules/p_controller/
├── p_controller.h        # Public API
├── p_controller.c        # Implementation
├── CMakeLists.txt        # Build configuration
├── Kconfig              # Zephyr config options
├── INTEGRATION.md       # Integration example
└── README.md            # This file
```

## API Reference

### Initialization

```c
int p_ctrl_init(struct p_ctrl_ctx *ctx);
```

Initialize P-controller context. Call once during system initialization.

### Mode Control

```c
void p_ctrl_set_mode(struct p_ctrl_ctx *ctx, uint8_t mode);
```

- `P_CTRL_MODE_MANUAL`: Direct PWM control, timer stopped
- `P_CTRL_MODE_AUTO`: P-control active, timer runs at 1kHz

### Parameter Setting

```c
void p_ctrl_set_setpoint(struct p_ctrl_ctx *ctx, uint16_t setpoint);
void p_ctrl_set_gain(struct p_ctrl_ctx *ctx, uint16_t gain);
void p_ctrl_set_feed_forward(struct p_ctrl_ctx *ctx, uint8_t ff);
```

All values are automatically clamped to valid ranges.

### Callbacks

```c
void p_ctrl_set_callbacks(struct p_ctrl_ctx *ctx,
                         void (*pwm_set)(uint8_t),
                         uint16_t (*adc_read)(void),
                         void (*stream_data)(uint16_t, uint16_t, uint8_t));
```

- `pwm_set`: Called to set PWM duty cycle (0-100)
- `adc_read`: Called to read current ADC value
- `stream_data`: Called to send streaming data

### Streaming

```c
int p_ctrl_start_stream(struct p_ctrl_ctx *ctx, uint32_t rate_hz);
int p_ctrl_stop_stream(struct p_ctrl_ctx *ctx);
```

Streaming at up to 1000 Hz. Uses decimation for lower rates.

### Status

```c
uint8_t p_ctrl_get_pwm(struct p_ctrl_ctx *ctx);
uint16_t p_ctrl_get_measured(struct p_ctrl_ctx *ctx);
```

## Integration Steps

1. **Add include to main.c**:
   ```c
   #include "modules/p_controller/p_controller.h"
   ```

2. **Declare context**:
   ```c
   static struct p_ctrl_ctx p_ctrl_ctx;
   ```

3. **Initialize**:
   ```c
   p_ctrl_init(&p_ctrl_ctx);
   p_ctrl_set_callbacks(&p_ctrl_ctx, led_pwm_set_duty, adc_ctrl_read_channel0, stream_cb);
   ```

4. **Add protocol handlers** for mode, setpoint, gain, feed_forward

5. **Update CMakeLists.txt** to add `src/modules/p_controller/p_controller.c`

See `INTEGRATION.md` for complete integration example.

## Technical Details

### Timer/Work Queue Architecture

- Timer callback (ISR context) submits work to system work queue
- Work handler (thread context) performs P-control calculation
- This keeps ISR short and allows use of kernel APIs in work handler

### Atomic Operations

Parameters read in timer ISR (`mode`, `setpoint`, `gain`, `feed_forward`) use `atomic_t` to ensure thread-safe access when modified by protocol handlers.

### Precision

- Control loop: 1 kHz (±1ms jitter acceptable for this application)
- Gain resolution: 0.01 (stored as integer 0-1000, divided by 100)
- PWM resolution: 1% (0-100)

## Testing

Recommended test procedure:

1. Start with MANUAL mode, verify PWM control works
2. Switch to AUTO mode with gain=0, verify feed_forward works
3. Set setpoint near current ADC reading, verify small gain
4. Increase gain gradually, observe response
5. Test streaming at various rates

## Future Enhancements

- PI/PID control (add integral/derivative terms)
- ADC filtering (moving average or exponential)
- Adaptive gain
- Anti-windup protection
- Ramp rate limiting
