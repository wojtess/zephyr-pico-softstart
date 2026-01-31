# ADC Continuous Streaming - Specification

## Version: 1.0
## Date: 2026-01-31

---

## 1. Overview

This specification defines a continuous ADC streaming feature that allows the RP2040 firmware to periodically sample ADC and stream data to the host independently of other protocol commands.

### Key Features
- **Configurable sampling interval**: 1-65535 ms (configurable at runtime)
- **Independent operation**: Stream continues while processing other commands
- **Thread-safe TX**: UART transmission synchronized between timer ISR and main loop
- **Real-time visualization**: GUI displays continuous ADC data on plot

---

## 2. Protocol Specification

### 2.1 New Commands

| Command Byte | Name | Direction | Frame Format |
|--------------|------|-----------|--------------|
| `0x04` | START_ADC_STREAM | Host → Device | `[0x04][INT_L][INT_H][CRC8]` |
| `0x05` | STOP_ADC_STREAM | Host → Device | `[0x05][CRC8]` |

### 2.2 START_ADC_STREAM Request

```
Frame: [0x04][INTerval_L][INTerval_H][CRC8]

Bytes:
  [0x04]         - Command byte
  [INTerval_L]   - Interval low byte (0-255)
  [INTerval_H]   - Interval high byte (0-255)
  [CRC8]         - CRC-8 of first 3 bytes

Interval: 16-bit value in milliseconds (1-65535 ms)
  - Min: 1 ms (1000 samples/sec)
  - Max: 65535 ms (~0.015 samples/sec)

Response:
  - ACK (0xFF) - Streaming started
  - NACK (0xFE) + error code - Failed to start
```

### 2.3 STOP_ADC_STREAM Request

```
Frame: [0x05][CRC8]

Bytes:
  [0x05]  - Command byte
  [CRC8]  - CRC-8 of first byte

Response:
  - ACK (0xFF) - Streaming stopped
  - NACK (0xFE) + error code - Failed to stop
```

### 2.4 Stream Data Format

Each ADC sample transmitted as stream frame:

```
Frame: [0x03][ADC_H][ADC_L][CRC8]

Bytes:
  [0x03]     - Command byte (same as CMD_READ_ADC)
  [ADC_H]    - ADC value high byte (0-15 for 12-bit ADC)
  [ADC_L]    - ADC value low byte (0-255)
  [CRC8]     - CRC-8 of first 3 bytes

Identical to single ADC read response for protocol compatibility.
```

---

## 3. Firmware Implementation (RP2040)

### 3.1 File Structure (Modular Architecture)

Current single-file (`src/main.c`, ~660 lines) will be split into modular components:

```
src/
├── main.c                 # Application entry point, initialization
├── protocol/
│   ├── protocol.c         # Protocol state machine, CRC, frame parsing
│   ├── protocol.h         # Protocol constants, enums, function prototypes
│   ├── tx_buffer.c        # TX ring buffer implementation
│   └── tx_buffer.h        # TX buffer API, spinlock interface
├── drivers/
│   ├── adc_ctrl.c         # ADC read functions
│   ├── adc_ctrl.h         # ADC driver interface
│   ├── led_pwm.c          # LED and PWM control functions
│   └── led_pwm.h          # LED/PWM driver interface
├── threads/
│   ├── rx_thread.c        # UART RX processing thread
│   ├── rx_thread.h        # RX thread interface
│   ├── tx_thread.c        # UART TX processing thread
│   └── tx_thread.h        # TX thread interface
│   ├── adc_stream.c       # ADC streaming thread
│   └── adc_stream.h       # Stream control interface
└── uart_isr.c             # UART interrupt handlers (ISR context)
```

**Rationale for modularization:**
- **Separation of concerns**: Each module handles specific functionality
- **Testability**: Individual modules can be unit tested
- **Maintainability**: Easier to locate and fix bugs
- **Scalability**: New features can be added without touching existing code
- **RTOS compliance**: Thread-safe boundaries clearly defined

### 3.2 RTOS Thread Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                        Zephyr RTOS Threads                      │
│  ─────────────────────────────────────────────────────────────  │
│                                                                 │
│  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐ │
│  │   Main Thread   │  │   TX Thread     │  │  RX Thread      │ │
│  │   (lowest)      │  │   (high)        │  │   (high)        │ │
│  │                 │  │                 │  │                 │ │
│  │ - Init          │  │ - Process TX    │  │ - Process RX    │ │
│  │ - Idle monitor  │  │   ring buffer   │  │   ring buffer   │ │
│  │ - Health check  │  │ - UART TX       │  │ - Protocol      │ │
│  │                 │  │                 │  │   state machine │ │
│  └─────────────────┘  └─────────────────┘  └─────────────────┘ │
│           │                    │                    │           │
│           └────────────────────┼────────────────────┘           │
│                                ▼                                │
│                     ┌─────────────────────┐                    │
│                     │   Shared Memory     │                    │
│                     │  (protected by)     │                    │
│                     │  - spinlocks        │                    │
│                     │  - mutexes          │                    │
│                     │  - atomic vars      │                    │
│                     └─────────────────────┘                    │
│                                │                                │
│                                ▼                                │
│  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐ │
│  │  k_timer ISR    │  │  UART ISR       │  │  Work Queue     │ │
│  │  (ADC stream)   │  │  (RX callback)  │  │  (deferred TX)  │ │
│  └─────────────────┘  └─────────────────┘  └─────────────────┘ │
└─────────────────────────────────────────────────────────────────┘
```

#### Thread Definitions

| Thread Name | Priority | Stack Size | Purpose |
|-------------|----------|------------|---------|
| `main_thread` | 0 (lowest) | 2048 bytes | Initialization, monitoring |
| `tx_thread` | -2 (high) | 1024 bytes | TX buffer processing |
| `rx_thread` | -2 (high) | 1024 bytes | RX buffer processing, protocol |

#### Thread Configuration (Kconfig)

```dts
# CONFIGURATION (prj.conf or Kconfig)

# Enable threads
CONFIG_THREADS=y

# Thread stack sizes
CONFIG_MAIN_STACK_SIZE=2048
CONFIG_SYSTEM_WORKQUEUE_PRIORITY=0

# Semaphore/Mutex support
CONFIG_MUTEX=y

# Timer support
CONFIG_TIMER=y
CONFIG_SYSTEM_WORKQUEUE=y
```

#### Thread Communication

**Synchronization Primitives:**
- **Spinlocks**: For ISR-safe data access (TX/RX ring buffers)
- **Mutexes**: For thread-safe resource access (ADC, LED)
- **Atomic variables**: For simple state flags (streaming active)
- **Semaphores**: For thread signaling (data ready)
- **Message Queues**: For inter-thread command passing

**Shared Data Structures:**
```c
/* TX Ring Buffer (protected by spinlock) */
struct tx_buffer {
    uint8_t buffer[TX_BUF_SIZE];
    volatile size_t head;
    volatile size_t tail;
    struct k_spinlock lock;
    struct k_sem data_ready;  /* Signal TX thread */
};

/* RX Ring Buffer (protected by spinlock) */
struct rx_buffer {
    uint8_t buffer[RX_BUF_SIZE];
    volatile size_t head;
    volatile size_t tail;
    struct k_spinlock lock;
    struct k_sem data_ready;  /* Signal RX thread */
};

/* Stream Control (atomic) */
struct stream_ctrl {
    atomic_t active;          /* Streaming enabled */
    atomic_t interval_ms;     /* Sample interval */
    struct k_timer timer;     /* Sampling timer */
};
```

#### Thread Lifecycle

```
┌─────────────────────────────────────────────────────────────┐
│  Main Thread (main.c)                                       │
│  ─────────────────────────────────────────────────────────  │
│                                                             │
│  1. Hardware init (UART, ADC, PWM, GPIO)                    │
│  2. Create TX thread                                        │
│  3. Create RX thread                                        │
│  4. Start UART RX interrupt                                 │
│  5. Enter monitoring loop                                   │
│     - Watch dog                                            │
│     - Health check                                          │
│     - Error recovery                                        │
└─────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────┐
│  RX Thread (rx_thread.c)                                    │
│  ─────────────────────────────────────────────────────────  │
│                                                             │
│  while (running) {                                          │
│      k_sem_take(&rx_data_ready, K_FOREVER);  /* Wait */     │
│      process_ring_buffer();                   /* Parse */    │
│      execute_protocol_commands();              /* Handle */  │
│  }                                                         │
└─────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────┐
│  TX Thread (tx_thread.c)                                    │
│  ─────────────────────────────────────────────────────────  │
│                                                             │
│  while (running) {                                          │
│      k_sem_take(&tx_data_ready, K_FOREVER);  /* Wait */     │
│      tx_process_chunk();                       /* Send */    │
│  }                                                         │
└─────────────────────────────────────────────────────────────┘
```

### 3.3 New State Machine States

```c
enum proto_state {
    STATE_WAIT_CMD,        // Waiting for command byte
    STATE_WAIT_VALUE,      // Waiting for value byte
    STATE_WAIT_CRC,        // Waiting for CRC (3-byte frame)
    STATE_WAIT_CRC_ADC,    // Waiting for CRC (2-byte frame)
    STATE_WAIT_INT_L,      // NEW: Waiting for interval low byte
    STATE_WAIT_INT_H,      // NEW: Waiting for interval high byte
};
```

### 3.3 New Protocol Constants

```c
/* protocol.h - Protocol constants */

#define CMD_SET_LED         0x01
#define CMD_SET_PWM         0x02
#define CMD_READ_ADC        0x03
#define CMD_START_STREAM    0x04    /* NEW: Start ADC streaming */
#define CMD_STOP_STREAM     0x05    /* NEW: Stop ADC streaming */

#define RESP_ACK            0xFF
#define RESP_NACK           0xFE

/* Error codes */
#define ERR_CRC             0x01
#define ERR_INVALID_CMD     0x02
#define ERR_INVALID_VAL     0x03
#define ERR_STREAM_ACTIVE   0x04    /* NEW: Already streaming */
#define ERR_NOT_STREAMING   0x05    /* NEW: Not streaming */
#define ERR_INVALID_INTERVAL 0x06   /* NEW: Invalid interval */

/* Buffer sizes */
#define RX_BUF_SIZE         32
#define TX_BUF_SIZE         128
```

### 3.4 Module: TX Buffer (tx_buffer.c/h)

**tx_buffer.h - Interface:**
```c
/**
 * @file tx_buffer.h
 * @brief Thread-safe TX ring buffer for UART transmission
 */

#ifndef TX_BUFFER_H
#define TX_BUFFER_H

#include <zephyr/kernel.h>
#include <stddef.h>

/** @brief TX buffer instance */
struct tx_buffer {
    uint8_t *buffer;          /* Data buffer */
    size_t size;              /* Buffer size */
    volatile size_t head;     /* Write position */
    volatile size_t tail;     /* Read position */
    struct k_spinlock lock;   /* ISR-safe lock */
    struct k_sem data_ready;  /* Data available signal */
};

/**
 * @brief Initialize TX buffer
 *
 * @param buf TX buffer instance
 * @param data Underlying data buffer
 * @param size Buffer size
 * @return 0 on success, negative on error
 */
int tx_buffer_init(struct tx_buffer *buf, uint8_t *data, size_t size);

/**
 * @brief Put data to TX buffer (ISR-safe, thread-safe)
 *
 * @param buf TX buffer instance
 * @param data Data to write
 * @param len Length of data
 * @return Number of bytes written, or -1 if buffer full
 */
int tx_buffer_put(struct tx_buffer *buf, const uint8_t *data, size_t len);

/**
 * @brief Get data from TX buffer (thread-safe)
 *
 * @param buf TX buffer instance
 * @param data Output buffer
 * @param len Maximum length to read
 * @return Number of bytes read
 */
size_t tx_buffer_get(struct tx_buffer *buf, uint8_t *data, size_t len);

/**
 * @brief Get available data count
 *
 * @param buf TX buffer instance
 * @return Number of bytes available to read
 */
size_t tx_buffer_available(const struct tx_buffer *buf);

/**
 * @brief Clear TX buffer
 *
 * @param buf TX buffer instance
 */
void tx_buffer_clear(struct tx_buffer *buf);

#endif /* TX_BUFFER_H */
```

**tx_buffer.c - Implementation:**
```c
/**
 * @file tx_buffer.c
 * @brief Thread-safe TX ring buffer implementation
 */

#include "tx_buffer.h"
#include <zephyr/sys/__assert.h>

int tx_buffer_init(struct tx_buffer *buf, uint8_t *data, size_t size)
{
    if (!buf || !data || size == 0) {
        return -EINVAL;
    }

    buf->buffer = data;
    buf->size = size;
    buf->head = 0;
    buf->tail = 0;
    k_spin_lock_init(&buf->lock);

    /* Initialize semaphore with initial count 0 */
    k_sem_init(&buf->data_ready, 0, 1);

    return 0;
}

int tx_buffer_put(struct tx_buffer *buf, const uint8_t *data, size_t len)
{
    k_spinlock_key_t key;
    size_t next_head;
    size_t available;
    size_t copied = 0;

    if (!buf || !data || len == 0) {
        return -EINVAL;
    }

    key = k_spin_lock(&buf->lock);

    /* Calculate available space */
    if (buf->head >= buf->tail) {
        available = buf->size - (buf->head - buf->tail) - 1;
    } else {
        available = buf->tail - buf->head - 1;
    }

    if (len > available) {
        k_spin_unlock(&buf->lock, key);
        return -ENOBUFS;  /* Buffer full */
    }

    /* Copy data to ring buffer */
    for (size_t i = 0; i < len; i++) {
        buf->buffer[buf->head] = data[i];
        buf->head = (buf->head + 1) % buf->size;
        copied++;
    }

    k_spin_unlock(&buf->lock, key);

    /* Signal data available */
    k_sem_give(&buf->data_ready);

    return copied;
}

size_t tx_buffer_get(struct tx_buffer *buf, uint8_t *data, size_t len)
{
    k_spinlock_key_t key;
    size_t available;
    size_t to_copy;
    size_t chunk_len;

    if (!buf || !data || len == 0) {
        return 0;
    }

    key = k_spin_lock(&buf->lock);

    /* Calculate available data */
    if (buf->head >= buf->tail) {
        available = buf->head - buf->tail;
    } else {
        available = buf->size - buf->tail;
    }

    to_copy = (len < available) ? len : available;

    if (to_copy == 0) {
        k_spin_unlock(&buf->lock, key);
        return 0;
    }

    /* Copy chunk */
    for (size_t i = 0; i < to_copy; i++) {
        data[i] = buf->buffer[buf->tail];
        buf->tail = (buf->tail + 1) % buf->size;
    }

    k_spin_unlock(&buf->lock, key);

    return to_copy;
}

size_t tx_buffer_available(const struct tx_buffer *buf)
{
    size_t available;

    if (!buf) {
        return 0;
    }

    /* Non-locking read for stats (may be slightly inaccurate) */
    if (buf->head >= buf->tail) {
        available = buf->head - buf->tail;
    } else {
        available = buf->size - buf->tail;
    }

    return available;
}

void tx_buffer_clear(struct tx_buffer *buf)
{
    k_spinlock_key_t key;

    if (!buf) {
        return;
    }

    key = k_spin_lock(&buf->lock);
    buf->head = 0;
    buf->tail = 0;
    k_spin_unlock(&buf->lock, key);
}
```

### 3.5 Module: Protocol (protocol.c/h)

**protocol.h - Interface:**
```c
/**
 * @file protocol.h
 * @brief Protocol state machine and frame handling
 */

#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>
#include <stddef.h>

/** @brief Protocol state machine states */
enum proto_state {
    STATE_WAIT_CMD,        /* Waiting for command byte */
    STATE_WAIT_VALUE,      /* Waiting for value byte */
    STATE_WAIT_CRC,        /* Waiting for CRC (3-byte frame) */
    STATE_WAIT_CRC_ADC,    /* Waiting for CRC (2-byte frame) */
    STATE_WAIT_INT_L,      /* Waiting for interval low byte */
    STATE_WAIT_INT_H,      /* Waiting for interval high byte */
};

/** @brief Protocol command context */
struct proto_context {
    enum proto_state state;
    uint8_t frame_buf[4];  /* Frame buffer: CMD + VALUE + INT_H */
    uint8_t frame_len;     /* Current frame length */

    /* Callbacks for command execution */
    void (*on_led_set)(uint8_t state);
    void (*on_pwm_set)(uint8_t duty);
    int (*on_adc_read)(void);
    void (*on_stream_start)(uint16_t interval_ms);
    void (*on_stream_stop)(void);
};

/**
 * @brief Initialize protocol context
 *
 * @param ctx Protocol context
 */
void proto_init(struct proto_context *ctx);

/**
 * @brief Process received byte through state machine
 *
 * @param ctx Protocol context
 * @param byte Received byte
 */
void proto_process_byte(struct proto_context *ctx, uint8_t byte);

/**
 * @brief Calculate CRC-8 checksum
 *
 * @param data Input data
 * @param len Length
 * @return CRC-8 value
 */
uint8_t crc8(const uint8_t *data, size_t len);

#endif /* PROTOCOL_H */
```

**protocol.c - Implementation (excerpt):**
```c
/**
 * @file protocol.c
 * @brief Protocol state machine implementation
 */

#include "protocol.h"
#include "tx_buffer.h"
#include <zephyr/sys/printk.h>

/* External references */
extern struct tx_buffer g_tx_buf;
extern struct device *uart_dev;

void proto_init(struct proto_context *ctx)
{
    ctx->state = STATE_WAIT_CMD;
    ctx->frame_len = 0;
    ctx->on_led_set = NULL;
    ctx->on_pwm_set = NULL;
    ctx->on_adc_read = NULL;
    ctx->on_stream_start = NULL;
    ctx->on_stream_stop = NULL;
}

/* Helper: Send response via TX buffer */
static void send_response(uint8_t resp)
{
    tx_buffer_put(&g_tx_buf, &resp, 1);
}

/* Helper: Send ADC response */
static void send_adc_response(uint16_t adc_value)
{
    uint8_t response[4];
    response[0] = CMD_READ_ADC;
    response[1] = (adc_value >> 8) & 0xFF;
    response[2] = adc_value & 0xFF;
    response[3] = crc8(response, 3);
    tx_buffer_put(&g_tx_buf, response, sizeof(response));
}

void proto_process_byte(struct proto_context *ctx, uint8_t byte)
{
    switch (ctx->state) {
    case STATE_WAIT_CMD:
        if (byte == CMD_SET_LED || byte == CMD_SET_PWM) {
            ctx->frame_buf[0] = byte;
            ctx->state = STATE_WAIT_VALUE;
        }
        else if (byte == CMD_READ_ADC) {
            ctx->frame_buf[0] = byte;
            ctx->state = STATE_WAIT_CRC_ADC;
        }
        else if (byte == CMD_START_STREAM) {
            ctx->frame_buf[0] = byte;
            ctx->state = STATE_WAIT_INT_L;
        }
        else if (byte == CMD_STOP_STREAM) {
            ctx->frame_buf[0] = byte;
            ctx->state = STATE_WAIT_CRC_ADC;
        }
        break;

    case STATE_WAIT_VALUE:
        ctx->frame_buf[1] = byte;
        ctx->state = STATE_WAIT_CRC;
        break;

    case STATE_WAIT_INT_L:
        ctx->frame_buf[1] = byte;  /* Interval low byte */
        ctx->state = STATE_WAIT_INT_H;
        break;

    case STATE_WAIT_INT_H:
        ctx->frame_buf[2] = byte;  /* Interval high byte */
        ctx->frame_len = 3;
        ctx->state = STATE_WAIT_CRC;
        break;

    case STATE_WAIT_CRC:
    {
        uint8_t calculated_crc = crc8(ctx->frame_buf, ctx->frame_len);
        if (byte != calculated_crc) {
            send_response(RESP_NACK);
            send_response(ERR_CRC);
            ctx->state = STATE_WAIT_CMD;
            break;
        }

        /* Execute command */
        if (ctx->frame_buf[0] == CMD_SET_LED && ctx->on_led_set) {
            ctx->on_led_set(ctx->frame_buf[1]);
            send_response(RESP_ACK);
        }
        else if (ctx->frame_buf[0] == CMD_SET_PWM && ctx->on_pwm_set) {
            if (ctx->frame_buf[1] <= 100) {
                ctx->on_pwm_set(ctx->frame_buf[1]);
                send_response(RESP_ACK);
            } else {
                send_response(RESP_NACK);
                send_response(ERR_INVALID_VAL);
            }
        }
        else if (ctx->frame_buf[0] == CMD_START_STREAM && ctx->on_stream_start) {
            uint16_t interval = (ctx->frame_buf[2] << 8) | ctx->frame_buf[1];
            ctx->on_stream_start(interval);
            send_response(RESP_ACK);
        }

        ctx->state = STATE_WAIT_CMD;
        break;
    }

    case STATE_WAIT_CRC_ADC:
    {
        uint8_t calculated_crc = crc8(&ctx->frame_buf[0], 1);
        if (byte != calculated_crc) {
            send_response(RESP_NACK);
            send_response(ERR_CRC);
            ctx->state = STATE_WAIT_CMD;
            break;
        }

        if (ctx->frame_buf[0] == CMD_READ_ADC && ctx->on_adc_read) {
            int adc_val = ctx->on_adc_read();
            if (adc_val >= 0) {
                send_adc_response(adc_val);
            } else {
                send_response(RESP_NACK);
                send_response(ERR_INVALID_CMD);
            }
        }
        else if (ctx->frame_buf[0] == CMD_STOP_STREAM && ctx->on_stream_stop) {
            ctx->on_stream_stop();
            send_response(RESP_ACK);
        }

        ctx->state = STATE_WAIT_CMD;
        break;
    }

    default:
        ctx->state = STATE_WAIT_CMD;
        break;
    }
}

uint8_t crc8(const uint8_t *data, size_t len)
{
    uint8_t crc = 0x00;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x07;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}
```

### 3.6 Module: ADC Stream (adc_stream.c/h)

**adc_stream.h - Interface:**
```c
/**
 * @file adc_stream.h
 * @brief ADC streaming control
 */

#ifndef ADC_STREAM_H
#define ADC_STREAM_H

#include <zephyr/kernel.h>
#include <stdbool.h>

/**
 * @brief Initialize ADC streaming module
 *
 * @return 0 on success, negative on error
 */
int adc_stream_init(void);

/**
 * @brief Start ADC streaming
 *
 * @param interval_ms Sampling interval in milliseconds (1-65535)
 * @return 0 on success, negative on error
 */
int adc_stream_start(uint16_t interval_ms);

/**
 * @brief Stop ADC streaming
 *
 * @return 0 on success, negative on error
 */
int adc_stream_stop(void);

/**
 * @brief Check if streaming is active
 *
 * @return true if streaming, false otherwise
 */
bool adc_stream_is_active(void);

/**
 * @brief Get current stream interval
 *
 * @return Interval in milliseconds, or 0 if not streaming
 */
uint16_t adc_stream_get_interval(void);

#endif /* ADC_STREAM_H */
```

**adc_stream.c - Implementation:**
```c
/**
 * @file adc_stream.c
 * @brief ADC streaming implementation with timer
 */

#include "adc_stream.h"
#include "adc_ctrl.h"
#include "tx_buffer.h"
#include <zephyr/sys/atomic.h>
#include <zephyr/drivers/adc.h>

/* External references */
extern struct tx_buffer g_tx_buf;
extern const struct device *adc_dev;

/* Stream control */
static atomic_t stream_active = ATOMIC_INIT(0);
static atomic_t stream_interval = ATOMIC_INIT(0);

/* Timer */
static struct k_timer adc_stream_timer;

/* Forward declarations */
static void adc_stream_timer_callback(struct k_timer *timer_id);

int adc_stream_init(void)
{
    /* Initialize timer */
    k_timer_init(&adc_stream_timer, adc_stream_timer_callback, NULL);
    return 0;
}

static void adc_stream_timer_callback(struct k_timer *timer_id)
{
    /* ISR context - minimal work only! */

    if (!atomic_get(&stream_active)) {
        return;
    }

    /* Read ADC */
    int adc_value = adc_read_channel0();
    if (adc_value < 0) {
        return;  /* Skip on error */
    }

    /* Build frame */
    uint8_t frame[4];
    frame[0] = CMD_READ_ADC;
    frame[1] = (adc_value >> 8) & 0xFF;
    frame[2] = adc_value & 0xFF;
    frame[3] = crc8(frame, 3);

    /* Queue to TX buffer (non-blocking) */
    tx_buffer_put(&g_tx_buf, frame, sizeof(frame));
}

int adc_stream_start(uint16_t interval_ms)
{
    if (interval_ms == 0 || interval_ms > 65535) {
        return -EINVAL;
    }

    if (atomic_get(&stream_active)) {
        return -EALREADY;  /* Already streaming */
    }

    atomic_set(&stream_interval, interval_ms);
    atomic_set(&stream_active, 1);

    /* Start periodic timer */
    k_timer_start(&adc_stream_timer, K_MSEC(interval_ms), K_MSEC(interval_ms));

    return 0;
}

int adc_stream_stop(void)
{
    if (!atomic_get(&stream_active)) {
        return -EINVAL;  /* Not streaming */
    }

    atomic_set(&stream_active, 0);
    k_timer_stop(&adc_stream_timer);

    return 0;
}

bool adc_stream_is_active(void)
{
    return atomic_get(&stream_active) != 0;
}

uint16_t adc_stream_get_interval(void)
{
    return atomic_get(&stream_interval);
}
```

### 3.7 Module: TX Thread (tx_thread.c/h)

**tx_thread.c - Implementation:**
```c
/**
 * @file tx_thread.c
 * @brief TX processing thread - sends data from TX buffer to UART
 */

#include "tx_thread.h"
#include "tx_buffer.h"
#include <zephyr/kernel.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(tx_thread, LOG_LEVEL_INF);

/* External references */
extern struct tx_buffer g_tx_buf;
extern const struct device *uart_dev;

/* TX thread stack */
static K_THREAD_DEFINE(tx_thread_tid, tx_thread_entry,
                       1024, NULL, NULL, NULL,
                       -2, 0, 0);

/* Thread synchronization */
static struct k_sem tx_thread_stop;

void tx_thread_entry(void *p1, void *p2, void *p3)
{
    uint8_t chunk[32];
    size_t chunk_len;

    LOG_INF("TX thread started");

    while (true) {
        /* Wait for data to be available */
        k_sem_take(&g_tx_buf.data_ready, K_FOREVER);

        /* Process all available data */
        while (tx_buffer_available(&g_tx_buf) > 0) {
            chunk_len = tx_buffer_get(&g_tx_buf, chunk, sizeof(chunk));
            if (chunk_len == 0) {
                break;
            }

            /* Send via UART */
            int sent = uart_fifo_fill(uart_dev, chunk, chunk_len);
            if (sent < chunk_len) {
                LOG_WRN("Partial TX: %d/%d bytes", sent, chunk_len);
            }
        }
    }
}

int tx_thread_init(void)
{
    k_sem_init(&tx_thread_stop, 0, 1);
    return 0;
}
```

### 3.8 Module: RX Thread (rx_thread.c/h)

**rx_thread.c - Implementation:**
```c
/**
 * @file rx_thread.c
 * @brief RX processing thread - processes UART RX data
 */

#include "rx_thread.h"
#include "protocol.h"
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(rx_thread, LOG_LEVEL_INF);

/* External references */
extern struct rx_buffer g_rx_buf;
extern struct proto_context g_proto_ctx;

/* RX thread stack */
static K_THREAD_DEFINE(rx_thread_tid, rx_thread_entry,
                       1024, NULL, NULL, NULL,
                       -2, 0, 0);

void rx_thread_entry(void *p1, void *p2, void *p3)
{
    uint8_t byte;

    LOG_INF("RX thread started");

    while (true) {
        /* Wait for data to be available */
        k_sem_take(&g_rx_buf.data_ready, K_FOREVER);

        /* Process all available data */
        while (rx_buffer_available(&g_rx_buf) > 0) {
            if (rx_buffer_get(&g_rx_buf, &byte, 1) == 1) {
                proto_process_byte(&g_proto_ctx, byte);
            }
        }
    }
}

int rx_thread_init(void)
{
    return 0;
}
```

### 3.9 Updated main() (main.c)

```c
/**
 * @file main.c
 * @brief Application entry point - modular architecture
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/sys/printk.h>

/* Module headers */
#include "tx_buffer.h"
#include "protocol.h"
#include "adc_ctrl.h"
#include "led_pwm.h"
#include "adc_stream.h"
#include "tx_thread.h"
#include "rx_thread.h"

/* Device tree nodes */
#define UART_NODE DT_NODELABEL(cdc_acm_uart0)
#define LED_NODE DT_NODELABEL(led0)
#define PWM_DEV_NODE DT_NODELABEL(pwm)
#define ADC_NODE DT_NODELABEL(adc)

/* Global instances */
struct tx_buffer g_tx_buf;
struct rx_buffer g_rx_buf;
struct proto_context g_proto_ctx;

static uint8_t tx_data[TX_BUF_SIZE];
static uint8_t rx_data[RX_BUF_SIZE];

/* Forward declarations */
static void uart_rx_handler(const struct device *dev, void *user_data);

/* Command callbacks */
static void on_led_set(uint8_t state);
static void on_pwm_set(uint8_t duty);
static int on_adc_read(void);
static void on_stream_start(uint16_t interval_ms);
static void on_stream_stop(void);

int main(void)
{
    uint32_t dtr = 0;
    const struct device *uart_dev;
    const struct device *pwm_dev;
    const struct device *adc_dev;
    const struct device *led_dev;

    /* Get device pointers */
    uart_dev = DEVICE_DT_GET(UART_NODE);
    pwm_dev = DEVICE_DT_GET(PWM_DEV_NODE);
    adc_dev = DEVICE_DT_GET(ADC_NODE);
    led_dev = DEVICE_DT_GET(LED_NODE);

    /* Verify devices */
    if (!device_is_ready(uart_dev)) {
        printk("ERROR: UART not ready\n");
        return 0;
    }
    if (!device_is_ready(pwm_dev)) {
        printk("ERROR: PWM not ready\n");
        return 0;
    }
    if (!device_is_ready(adc_dev)) {
        printk("ERROR: ADC not ready\n");
        return 0;
    }
    if (!gpio_is_ready_dt(&(struct gpio_dt_spec)GPIO_DT_SPEC_GET(LED_NODE, gpios))) {
        printk("ERROR: LED not ready\n");
        return 0;
    }

    /* Initialize modules */
    tx_buffer_init(&g_tx_buf, tx_data, TX_BUF_SIZE);
    rx_buffer_init(&g_rx_buf, rx_data, RX_BUF_SIZE);
    adc_ctrl_init(adc_dev);
    led_pwm_init(pwm_dev, led_dev);
    adc_stream_init();

    /* Initialize protocol with callbacks */
    proto_init(&g_proto_ctx);
    g_proto_ctx.on_led_set = on_led_set;
    g_proto_ctx.on_pwm_set = on_pwm_set;
    g_proto_ctx.on_adc_read = on_adc_read;
    g_proto_ctx.on_stream_start = on_stream_start;
    g_proto_ctx.on_stream_stop = on_stream_stop;

    /* Wait for USB DTR */
    printk("Waiting for USB connection...\n");
    while (!dtr) {
        uart_line_ctrl_get(uart_dev, UART_LINE_CTRL_DTR, &dtr);
        k_sleep(K_MSEC(100));
    }

    printk("===================================\n");
    printk("RP2040 LED + ADC Control (Modular)\n");
    printk("===================================\n\n");

    /* Setup UART RX interrupt */
    uart_irq_callback_set(uart_dev, uart_rx_handler);
    uart_irq_rx_enable(uart_dev);

    /* Enter main monitoring loop */
    while (true) {
        /* Health monitoring, watchdog, etc. */
        k_sleep(K_MSEC(1000));
    }

    return 0;
}

/* UART RX interrupt handler */
static void uart_rx_handler(const struct device *dev, void *user_data)
{
    uint8_t buf[16];
    uint32_t len;

    while (uart_irq_update(dev) && uart_irq_rx_ready(dev)) {
        len = uart_fifo_read(dev, buf, sizeof(buf));
        for (uint32_t i = 0; i < len; i++) {
            rx_buffer_put(&g_rx_buf, &buf[i], 1);
        }
    }
}

/* Command callbacks */
static void on_led_set(uint8_t state)
{
    led_pwm_set(state > 0);
}

static void on_pwm_set(uint8_t duty)
{
    led_pwm_set_duty(duty);
}

static int on_adc_read(void)
{
    return adc_read_channel0();
}

static void on_stream_start(uint16_t interval_ms)
{
    int ret = adc_stream_start(interval_ms);
    if (ret < 0) {
        printk("Failed to start stream: %d\n", ret);
    }
}

static void on_stream_stop(void)
{
    adc_stream_stop();
}
```

### 3.10 Configuration Updates (prj.conf)

```c
/**
 * @brief Put data to TX ring buffer (ISR-safe, thread-safe)
 *
 * @param data Data buffer to send
 * @param len Length of data
 * @return Number of bytes queued, or -1 if buffer full
 */
static int tx_buf_put(const uint8_t *data, size_t len)
{
    k_spinlock_key_t key;
    size_t next_head;
    size_t available;

    key = k_spin_lock(&tx_buf_lock);

    /* Calculate available space */
    if (tx_head >= tx_tail) {
        available = TX_BUF_SIZE - (tx_head - tx_tail) - 1;
    } else {
        available = tx_tail - tx_head - 1;
    }

    if (len > available) {
        k_spin_unlock(&tx_buf_lock, key);
        return -1;  /* Buffer full */
    }

    /* Copy data to ring buffer */
    for (size_t i = 0; i < len; i++) {
        tx_buf[tx_head] = data[i];
        tx_head = (tx_head + 1) % TX_BUF_SIZE;
    }

    k_spin_unlock(&tx_buf_lock, key);

    /* Trigger TX work */
    k_work_submit(&tx_work);

    return len;
}

/**
 * @brief TX work handler (thread context)
 *
 * @details Reads from TX ring buffer and sends via UART
 */
static void tx_work_handler(struct k_work *work)
{
    uint8_t chunk[32];
    size_t chunk_len;

    while (true) {
        k_spinlock_key_t key = k_spin_lock(&tx_buf_lock);

        /* Calculate available data */
        if (tx_head >= tx_tail) {
            chunk_len = tx_head - tx_tail;
        } else {
            chunk_len = TX_BUF_SIZE - tx_tail;
        }

        if (chunk_len > sizeof(chunk)) {
            chunk_len = sizeof(chunk);
        }

        if (chunk_len == 0) {
            k_spin_unlock(&tx_buf_lock, key);
            break;  /* No more data */
        }

        /* Copy chunk */
        for (size_t i = 0; i < chunk_len; i++) {
            chunk[i] = tx_buf[tx_tail];
            tx_tail = (tx_tail + 1) % TX_BUF_SIZE;
        }

        k_spin_unlock(&tx_buf_lock, key);

        /* Send via UART (outside lock!) */
        uart_fifo_fill(uart_dev, chunk, chunk_len);
    }
}
```

### 3.5 Timer Callback (ISR Context)

```c
/**
 * @brief ADC timer callback (ISR CONTEXT!)
 *
 * @details Runs in interrupt context - minimal work, non-blocking only!
 *          Reads ADC and queues response to TX buffer.
 */
static void adc_timer_callback(struct k_timer *timer_id)
{
    if (!atomic_get(&adc_streaming)) {
        return;
    }

    /* Read ADC (fast, non-blocking) */
    int adc_value = read_adc_channel0();
    if (adc_value < 0) {
        return;  /* Error, skip this sample */
    }

    /* Build frame */
    uint8_t frame[4];
    frame[0] = CMD_READ_ADC;
    frame[1] = (adc_value >> 8) & 0xFF;
    frame[2] = adc_value & 0xFF;
    frame[3] = crc8(frame, 3);

    /* Queue to TX buffer (non-blocking, ISR-safe) */
    tx_buf_put(frame, sizeof(frame));
}
```

### 3.6 Updated Response Functions

```c
/* All response functions now use TX buffer */
static void send_response(uint8_t resp)
{
    uint8_t data = resp;
    tx_buf_put(&data, 1);
}

static void send_adc_response(uint16_t adc_value)
{
    uint8_t response[4];
    response[0] = CMD_READ_ADC;
    response[1] = (adc_value >> 8) & 0xFF;
    response[2] = adc_value & 0xFF;
    response[3] = crc8(response, 3);
    tx_buf_put(response, sizeof(response));
}
```

### 3.7 Protocol State Machine Updates

```c
/* In STATE_WAIT_CMD case: */
if (byte == CMD_START_STREAM) {
    frame_buf[0] = byte;
    proto_state = STATE_WAIT_INT_L;
}
else if (byte == CMD_STOP_STREAM) {
    frame_buf[0] = byte;
    proto_state = STATE_WAIT_CRC_ADC;  /* 2-byte frame */

/* New case: STATE_WAIT_INT_L */
case STATE_WAIT_INT_L:
    frame_buf[1] = byte;  /* Interval low byte */
    proto_state = STATE_WAIT_INT_H;
    break;

/* New case: STATE_WAIT_INT_H */
case STATE_WAIT_INT_H:
{
    /* Verify CRC for 3-byte frame [CMD][INT_L][INT_H][CRC] */
    frame_buf[2] = byte;  /* Interval high byte */
    uint8_t calculated_crc = crc8(frame_buf, 3);

    proto_state = STATE_WAIT_CRC;  /* Wait for CRC */
    break;
}

/* In STATE_WAIT_CRC (after CMD_START_STREAM confirmed): */
if (frame_buf[0] == CMD_START_STREAM) {
    uint16_t interval = (frame_buf[2] << 8) | frame_buf[1];

    /* Validate interval */
    if (interval == 0 || interval > 65535) {
        send_response(RESP_NACK);
        send_response(ERR_INVALID_INTERVAL);
        proto_state = STATE_WAIT_CMD;
        break;
    }

    /* Check if already streaming */
    if (atomic_get(&adc_streaming)) {
        send_response(RESP_NACK);
        send_response(ERR_STREAM_ACTIVE);
        proto_state = STATE_WAIT_CMD;
        break;
    }

    /* Start streaming */
    atomic_set(&adc_stream_interval, interval);
    atomic_set(&adc_streaming, 1);
    k_timer_start(&adc_timer, K_MSEC(interval), K_MSEC(interval));

    send_response(RESP_ACK);
    proto_state = STATE_WAIT_CMD;
    break;
}

/* In STATE_WAIT_CRC_ADC (after CMD_STOP_STREAM confirmed): */
if (frame_buf[0] == CMD_STOP_STREAM) {
    /* Stop streaming */
    atomic_set(&adc_streaming, 0);
    k_timer_stop(&adc_timer);

    send_response(RESP_ACK);
    proto_state = STATE_WAIT_CMD;
    break;
}
```

### 3.8 Initialization in main()

```c
int main(void)
{
    /* ... existing initialization ... */

    /* Initialize TX work queue */
    k_work_init(&tx_work, tx_work_handler);

    /* Initialize ADC timer */
    k_timer_init(&adc_timer, adc_timer_callback, NULL);

    /* ... rest of main loop ... */
}
```

---

## 4. Performance Analysis

### 4.1 Timing Breakdown

| Operation | Time | Notes |
|-----------|------|-------|
| ADC read | ~5-10 µs | RP2040 ADC conversion |
| CRC-8 calculation | ~1-2 µs | 3 bytes @ 133MHz |
| UART TX (4 bytes) | ~347 µs | @ 115200 baud |
| **Total per sample** | **~360 µs** | Excluding timer overhead |

### 4.2 Recommended Intervals

| Interval | Samples/s | UART Usage | Use Case |
|----------|-----------|------------|----------|
| 2 ms | 500 | 1.7% | Fast monitoring |
| 10 ms | 100 | 0.35% | Standard |
| 50 ms | 20 | 0.07% | Slow logging |
| 100 ms | 10 | 0.035% | Periodic check |
| 1000 ms | 1 | 0.0035% | Very slow |

**Minimum practical interval: 2 ms** (500 samples/s)
**Recommended range: 10-100 ms** (100-10 samples/s)

### 4.3 UART Bandwidth Analysis

```
UART @ 115200 baud:
  Raw bandwidth: 11,520 bytes/sec
  ADC frame: 4 bytes
  Max theoretical: 2,880 samples/s

At 100 samples/s (10 ms interval):
  400 bytes/sec = 3.5% bandwidth
  Plenty of headroom for commands
```

---

## 5. Python GUI Implementation

### 5.1 New Data Structures

```python
# constants.py

class SerialCommand(Enum):
    # ... existing ...
    START_ADC_STREAM = "start_adc_stream"
    STOP_ADC_STREAM = "stop_adc_stream"

@dataclass
class ADCStreamPoint:
    """Single ADC stream data point."""
    timestamp: float    # time.time() for sequence
    raw_value: int      # 0-4095
    voltage: float      # 0-3.3V

@dataclass
class SerialTask:
    # ... existing ...
    adc_interval: Optional[int] = None  # For START_STREAM command
```

### 5.2 App State Variables

```python
# app.py - in __init__

# ADC stream state
self._adc_streaming: bool = False

# Stream queues and buffers
self._adc_stream_queue: queue.Queue = queue.Queue(maxsize=500)  # Bounded
self._adc_stream_lock = threading.Lock()
self._adc_stream_buffer: deque[ADCStreamPoint] = deque(maxlen=1000)

# Plot throttling
self._last_plot_update: float = 0.0
self._plot_update_interval: float = 0.033  # 30 FPS
```

### 5.3 Modified Worker Loop

```python
def _serial_worker(self) -> None:
    """Modified worker loop with stream support."""
    while self._running:
        task = None

        # Step 1: Check task_queue with short timeout (non-blocking)
        try:
            task = self._task_queue.get(timeout=0.01)  # 10ms
        except queue.Empty:
            pass

        # Step 2: Execute task if available (priority)
        if task:
            if task.command == SerialCommand.QUIT:
                break

            elif task.command == SerialCommand.START_ADC_STREAM:
                result = self._handle_start_adc_stream(task.adc_interval)

            elif task.command == SerialCommand.STOP_ADC_STREAM:
                result = self._handle_stop_adc_stream()

            # ... existing command handling ...

            # Send result
            if task.result_queue and result:
                task.result_queue.put(result)

            self._task_queue.task_done()

        # Step 3: Receive stream data if idle and streaming active
        elif self._adc_streaming:
            self._receive_adc_stream_data()
```

### 5.4 Stream Receive Handler

```python
def _receive_adc_stream_data(self) -> None:
    """Receive available ADC stream data (non-blocking)."""
    try:
        with self._lock:
            if not self._serial_connection or not self._serial_connection.is_open:
                return

        # Read all available frames
        while self._serial_connection.in_waiting >= 4:
            point = self._read_adc_stream_frame()
            if point:
                self._enqueue_stream_point(point)
    except Exception as e:
        logger.error(f"Stream receive error: {e}")

def _read_adc_stream_frame(self) -> Optional[ADCStreamPoint]:
    """Read single ADC stream frame [CMD][ADC_H][ADC_L][CRC]."""
    try:
        resp = self._serial_connection.read(4)
        if not resp or len(resp) < 4:
            return None

        # Verify it's ADC data
        if resp[0] != CMD_READ_ADC:
            return None

        # Parse with existing function
        adc_value, err_code = parse_adc_response(resp)
        if adc_value >= 0:
            voltage = (adc_value / 4095.0) * 3.3
            return ADCStreamPoint(
                timestamp=time.time(),
                raw_value=adc_value,
                voltage=voltage
            )
        return None
    except Exception as e:
        logger.error(f"Stream frame read error: {e}")
        return None

def _enqueue_stream_point(self, point: ADCStreamPoint) -> None:
    """Thread-safe enqueue with drop-oldest on overflow."""
    try:
        self._adc_stream_queue.put_nowait(point)
    except queue.Full:
        # Queue full - drop oldest and add new
        try:
            self._adc_stream_queue.get_nowait()
            self._adc_stream_queue.put_nowait(point)
        except queue.Empty:
            pass
```

### 5.5 Start/Stop Stream Handlers

```python
def _handle_start_adc_stream(self, interval: int) -> SerialResult:
    """Send START_ADC_STREAM command to device."""
    try:
        with self._lock:
            if not self._serial_connection or not self._serial_connection.is_open:
                return SerialResult(
                    SerialCommand.START_ADC_STREAM,
                    False,
                    "Not connected",
                    error="No connection"
                )

        # Build frame: [CMD][INT_L][INT_H][CRC]
        int_l = interval & 0xFF
        int_h = (interval >> 8) & 0xFF

        frame = bytearray([CMD_START_STREAM, int_l, int_h])
        frame.append(crc8(frame, 3))

        with self._lock:
            self._serial_connection.write(frame)
            self._serial_connection.flush()

            # Read response
            resp = self._serial_connection.read(1)
            if not resp:
                return SerialResult(
                    SerialCommand.START_ADC_STREAM,
                    False,
                    "No response",
                    error="Timeout"
                )

            if resp[0] == RESP_ACK:
                self._adc_streaming = True
                logger.info(f"ADC stream started at {interval}ms interval")
                return SerialResult(
                    SerialCommand.START_ADC_STREAM,
                    True,
                    f"Stream started ({interval}ms interval)"
                )
            else:  # NACK
                err_byte = self._serial_connection.read(1)
                err_code = err_byte[0] if err_byte else 0
                error_name = get_error_name(err_code)
                return SerialResult(
                    SerialCommand.START_ADC_STREAM,
                    False,
                    f"Failed: {error_name}",
                    error=error_name
                )

    except Exception as e:
        logger.error(f"Start stream error: {e}")
        return SerialResult(
            SerialCommand.START_ADC_STREAM,
            False,
            "Unexpected error",
            error=str(e)
        )

def _handle_stop_adc_stream(self) -> SerialResult:
    """Send STOP_ADC_STREAM command to device."""
    try:
        with self._lock:
            if not self._serial_connection or not self._serial_connection.is_open:
                return SerialResult(
                    SerialCommand.STOP_ADC_STREAM,
                    False,
                    "Not connected",
                    error="No connection"
                )

        # Build frame: [CMD][CRC]
        frame = bytearray([CMD_STOP_STREAM])
        frame.append(crc8(frame, 1))

        with self._lock:
            self._serial_connection.write(frame)
            self._serial_connection.flush()

            # Read response
            resp = self._serial_connection.read(1)
            if not resp:
                return SerialResult(
                    SerialCommand.STOP_ADC_STREAM,
                    False,
                    "No response",
                    error="Timeout"
                )

            if resp[0] == RESP_ACK:
                self._adc_streaming = False

                # Clear stream queue
                while not self._adc_stream_queue.empty():
                    try:
                        self._adc_stream_queue.get_nowait()
                    except queue.Empty:
                        break

                logger.info("ADC stream stopped")
                return SerialResult(
                    SerialCommand.STOP_ADC_STREAM,
                    True,
                    "Stream stopped"
                )
            else:  # NACK
                err_byte = self._serial_connection.read(1)
                err_code = err_byte[0] if err_byte else 0
                error_name = get_error_name(err_code)
                return SerialResult(
                    SerialCommand.STOP_ADC_STREAM,
                    False,
                    f"Failed: {error_name}",
                    error=error_name
                )

    except Exception as e:
        logger.error(f"Stop stream error: {e}")
        return SerialResult(
            SerialCommand.STOP_ADC_STREAM,
            False,
            "Unexpected error",
            error=str(e)
        )
```

### 5.6 Frame Callback Update

```python
def on_frame_callback(sender, app_data, user_data: LEDControllerApp) -> None:
    """Called every frame - process stream and update plot."""

    # --- Health check (existing) ---
    results = user_data.check_results()
    for result in results:
        if result.command == SerialCommand.CHECK_HEALTH:
            if not result.success:
                handle_disconnect_state(user_data)
                update_status(f"Connection lost: {result.message}", [255, 150, 50])
                continue

            # Schedule next health check
            import time
            current_time = time.time()
            if current_time - user_data._last_health_check >= user_data._health_check_interval:
                user_data._last_health_check = current_time
                user_data.send_task(SerialTask(command=SerialCommand.CHECK_HEALTH))

    # --- ADC Stream batch processing (NEW) ---
    if user_data._adc_streaming:
        # Get ALL new points from queue (batch)
        new_points = []
        try:
            while True:
                point = user_data._adc_stream_queue.get_nowait()
                new_points.append(point)
        except queue.Empty:
            pass

        # Add to buffer (thread-safe)
        if new_points:
            with user_data._adc_stream_lock:
                user_data._adc_stream_buffer.extend(new_points)

            # Update plot with throttling (max 30 FPS)
            current_time = time.time()
            if current_time - user_data._last_plot_update >= user_data._plot_update_interval:
                _update_adc_plot(user_data)
                user_data._last_plot_update = current_time

    # --- Queue display (existing) ---
    update_queue_display(user_data)
```

### 5.7 Plot Update Helper

```python
def _update_adc_plot(app: LEDControllerApp) -> None:
    """Update DearPyGui plot with current stream buffer."""
    with app._adc_stream_lock:
        if not app._adc_stream_buffer:
            return

        # Convert deque to lists for plot
        points = list(app._adc_stream_buffer)
        x_axis = list(range(len(points)))  # Sequential indices
        y_raw = [p.raw_value for p in points]
        y_voltage = [p.voltage for p in points]

    # Update plot series
    if dpg.does_item_exist(TAGS["adc_series_raw"]):
        dpg.set_value(TAGS["adc_series_raw"], [x_axis, y_raw])
    if dpg.does_item_exist(TAGS["adc_series_voltage"]):
        dpg.set_value(TAGS["adc_series_voltage"], [x_axis, y_voltage])
```

### 5.8 GUI Widgets (widgets.py)

```python
def create_adc_stream_widgets():
    """Create ADC streaming control widgets."""

    # Stream controls
    dpg.add_separator()
    dpg.add_text("ADC Streaming:", color=[200, 200, 200])

    with dpg.group(horizontal=True):
        dpg.add_text("Interval (ms):", color=[180, 180, 180])
        dpg.add_input_int(
            tag=TAGS["adc_interval_input"],
            default_value=100,
            min_value=2,
            max_value=10000,
            width=100
        )

    with dpg.group(horizontal=True):
        dpg.add_checkbox(
            tag=TAGS["adc_stream_enable"],
            label="Enable Streaming",
            callback=on_adc_stream_toggled
        )

    # Stream status
    dpg.add_text(
        tag=TAGS["adc_stream_status"],
        default_value="Stream: OFF",
        color=[150, 150, 150]
    )
```

### 5.9 Stream Toggle Callback

```python
def on_adc_stream_toggled(sender, app_data, user_data: LEDControllerApp) -> None:
    """Callback for ADC stream enable/disable checkbox."""
    is_checked = app_data

    # Get interval from input
    interval = dpg.get_value(TAGS["adc_interval_input"])

    if is_checked:
        # Validate interval
        if interval < 2 or interval > 10000:
            update_status("Invalid interval (2-10000 ms)", [255, 100, 100])
            dpg.set_value(sender, False)
            return

        task = SerialTask(
            command=SerialCommand.START_ADC_STREAM,
            adc_interval=interval
        )
        update_status(f"Starting ADC stream ({interval}ms)...", [200, 200, 100])
    else:
        task = SerialTask(command=SerialCommand.STOP_ADC_STREAM)
        update_status("Stopping ADC stream...", [200, 200, 100])

    result_queue = user_data.send_task(task)

    def check_result():
        try:
            result = result_queue.get(timeout=2)

            if result.success:
                update_status(result.message, [100, 200, 100])

                # Update status display
                if is_checked:
                    dpg.set_value(TAGS["adc_stream_status"], f"Stream: ON ({interval}ms)")
                    dpg.configure_item(TAGS["adc_stream_status"], color=[100, 255, 100])
                else:
                    dpg.set_value(TAGS["adc_stream_status"], "Stream: OFF")
                    dpg.configure_item(TAGS["adc_stream_status"], color=[150, 150, 150])
            else:
                update_status(f"Stream error: {result.message}", [255, 100, 100])
                # Revert checkbox
                if dpg.does_item_exist(sender):
                    dpg.set_value(sender, not is_checked)

        except queue.Empty:
            update_status("Stream command timeout", [255, 150, 50])
            if dpg.does_item_exist(sender):
                dpg.set_value(sender, not is_checked)

    threading.Thread(target=check_result, daemon=True).start()
```

---

## 6. Protocol Module Updates (protocol.py)

```python
# New command constants
CMD_START_STREAM = 0x04
CMD_STOP_STREAM = 0x05

# New error codes
ERR_STREAM_ACTIVE = 0x04
ERR_NOT_STREAMING = 0x05
ERR_INVALID_INTERVAL = 0x06

def get_error_name(error_code: int) -> str:
    """Get human-readable error name."""
    errors = {
        0x01: "CRC Mismatch",
        0x02: "Invalid Command",
        0x03: "Invalid Value",
        0x04: "Already Streaming",
        0x05: "Not Streaming",
        0x06: "Invalid Interval",
    }
    return errors.get(error_code, f"Unknown Error (0x{error_code:02X})")

def build_stream_start_frame(interval: int) -> bytes:
    """Build START_ADC_STREAM frame."""
    int_l = interval & 0xFF
    int_h = (interval >> 8) & 0xFF
    frame = bytes([CMD_START_STREAM, int_l, int_h])
    crc = crc8_bytes(frame, 3)
    return frame + bytes([crc])

def build_stream_stop_frame() -> bytes:
    """Build STOP_ADC_STREAM frame."""
    frame = bytes([CMD_STOP_STREAM])
    crc = crc8_bytes(frame, 1)
    return frame + bytes([crc])
```

---

## 7. Configuration Updates (prj.conf)

### 7.1 RTOS and Thread Configuration

```conf
# =========================================================================
# RTOS KERNEL CONFIGURATION
# =========================================================================

# Enable multi-threading
CONFIG_THREADS=y

# Thread priorities (negative = higher priority)
CONFIG_MAIN_THREAD_PRIORITY=0
CONFIG_NUM_PREEMPT_PRIORITIES=16

# Thread stack sizes
CONFIG_MAIN_STACK_SIZE=2048
CONFIG_IDLE_STACK_SIZE=512
CONFIG_SYSTEM_WORKQUEUE_STACK_SIZE=1024

# Enable mutex for thread synchronization
CONFIG_MUTEX=y

# Enable semaphores
CONFIG_SEMAPHORE=y

# Enable spinlocks (for ISR-safe locking)
CONFIG_SPINLOCK=y

# =========================================================================
# TIMER CONFIGURATION
# =========================================================================

# Enable system timer
CONFIG_TIMER=y

# Enable work queues for deferred work
CONFIG_SYSTEM_WORKQUEUE=y

# Timer precision
CONFIG_SYSTEM_WORKQUEUE_PRIORITY=0

# =========================================================================
# ADC STREAMING CONFIGURATION
# =========================================================================

# ADC stream buffer size
# Must be power of 2 for efficiency
CONFIG_ADC_STREAM_TX_BUF_SIZE=128

# ADC stream interval range (milliseconds)
CONFIG_ADC_STREAM_INTERVAL_MIN=1
CONFIG_ADC_STREAM_INTERVAL_MAX=65535

# Default ADC stream interval
CONFIG_ADC_STREAM_INTERVAL_DEFAULT=100

# =========================================================================
# LOGGING (optional, for debugging)
# =========================================================================

# Enable logging
CONFIG_LOG=y
CONFIG_LOG_MODE_IMMEDIATE=n
CONFIG_LOG_BUFFER_SIZE=1024

# Log levels
CONFIG_LOG_DEFAULT_LEVEL=2  # Info level

# Module-specific log levels
CONFIG_ADC_STREAM_LOG_LEVEL=2
CONFIG_TX_THREAD_LOG_LEVEL=2
CONFIG_RX_THREAD_LOG_LEVEL=2
```

### 7.2 Full Updated prj.conf

```conf
# --- USB CDC ACM ---
CONFIG_USB_DEVICE_STACK=y
CONFIG_USB_DEVICE_PRODUCT="RP2040 LED Ctrl"
CONFIG_USB_DEVICE_VID=0x2E8A
CONFIG_USB_DEVICE_PID=0x0001
CONFIG_USB_DEVICE_INITIALIZE_AT_BOOT=y
CONFIG_SERIAL=y
CONFIG_UART_LINE_CTRL=y
CONFIG_USB_CDC_ACM=y

# --- Console/Logging ---
# Disable console on UART to prevent log messages in binary protocol
CONFIG_UART_CONSOLE=n

# --- GPIO (for built-in LED) ---
CONFIG_GPIO=y

# --- PWM (for LED brightness control) ---
CONFIG_PWM=y

# --- ADC (for analog input reading) ---
CONFIG_ADC=y

# --- RTOS Kernel (NEW for ADC streaming) ---
CONFIG_THREADS=y
CONFIG_MUTEX=y
CONFIG_SEMAPHORE=y
CONFIG_SPINLOCK=y

# --- Timers and Work Queues (NEW) ---
CONFIG_TIMER=y
CONFIG_SYSTEM_WORKQUEUE=y

# --- Logging (optional) ---
CONFIG_LOG=y
CONFIG_LOG_BUFFER_SIZE=1024
```

---

## 8. Testing Plan

### 8.1 Unit Tests (Firmware)

1. **Timer callback test**: Verify timer fires at correct interval
2. **TX buffer test**: Verify concurrent access from ISR and main loop
3. **Protocol parsing test**: Verify START/STOP stream commands

### 8.2 Integration Tests

1. **Start stream at 10ms**: Verify 100 samples/s for 10 seconds
2. **Start stream while streaming**: Verify NACK + ERR_STREAM_ACTIVE
3. **Stop without start**: Verify NACK + ERR_NOT_STREAMING
4. **Invalid interval**: Verify NACK + ERR_INVALID_INTERVAL

### 8.3 GUI Tests

1. **Stream visualization**: Verify plot updates in real-time
2. **Queue overflow**: Test at 1ms interval - verify drop-oldest
3. **Disconnect while streaming**: Verify cleanup
4. **Command during stream**: Verify LED/PWM commands work

---

## 9. Rollout Plan

### Phase 1: File Structure Refactor (Foundation)
1. Create `src/protocol/` directory
2. Create `src/drivers/` directory
3. Create `src/threads/` directory
4. Move existing code to modules:
   - `protocol.{c,h}` - CRC, protocol constants
   - `tx_buffer.{c,h}` - TX ring buffer
   - `rx_buffer.{c,h}` - RX ring buffer
   - `led_pwm.{c,h}` - LED/PWM control
   - `adc_ctrl.{c,h}` - ADC read functions
5. Update `CMakeLists.txt` to include new source files
6. Verify build succeeds

### Phase 2: RTOS Thread Implementation
1. Implement `tx_thread.c` - TX processing thread
2. Implement `rx_thread.c` - RX processing thread
3. Update `main.c` to create threads
4. Add thread synchronization (semaphores, mutexes)
5. Test thread communication

### Phase 3: ADC Streaming Feature
1. Implement `adc_stream.{c,h}` - Stream control
2. Add timer callback for periodic sampling
3. Update protocol state machine (STREAM_START/STOP)
4. Test streaming at various intervals

### Phase 4: Protocol Module Updates
1. Add new command constants to `protocol.h`
2. Implement frame building functions
3. Update error codes
4. Add protocol documentation

### Phase 5: Python GUI Integration
1. Update `protocol.py` with new commands
2. Implement stream receive handler in GUI
3. Add stream control widgets
4. Test real-time visualization

### Phase 1: Firmware (Priority)
1. Implement TX ring buffer + work queue
2. Add timer callback for ADC streaming
3. Update protocol state machine
4. Test with hardware

### Phase 2: Protocol Module
1. Add new command constants
2. Add frame building functions
3. Update error codes

### Phase 3: GUI
1. Add stream state variables
2. Implement stream receive handler
3. Add GUI widgets
4. Test integration

---

## 10. Backward Compatibility

- Existing commands (LED, PWM, single ADC read) unchanged
- Stream frames identical to single ADC response
- Existing GUI functionality unaffected

---

## 11. Limitations

1. **TX buffer overflow**: At very high streaming rates (>500 samples/s), TX buffer may fill
2. **Main loop priority**: Commands take priority over stream (expected behavior)
3. **Timer precision**: Zephyr timers have ~1ms precision

---

## 12. Future Enhancements

1. **Adaptive interval**: Auto-adjust based on available bandwidth
2. **Multi-channel streaming**: Stream multiple ADC channels
3. **Data compression**: Delta encoding for better efficiency
4. **Burst mode**: Send N samples per frame
