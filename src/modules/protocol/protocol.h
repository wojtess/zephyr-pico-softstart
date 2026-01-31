/**
 * @file protocol.h
 * @brief Binary protocol module for UART communication
 *
 * @details Implements 3-byte frame format [CMD][VALUE][CRC8] and 2-byte ADC read.
 *          Supports LED ON/OFF, PWM duty cycle control, and ADC reading.
 */

#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stddef.h>
#include <stdint.h>

/* =========================================================================
 * PROTOCOL CONSTANTS
 * ========================================================================= */

/** @brief Command byte for LED ON/OFF control (legacy) */
#define PROTO_CMD_SET_LED      0x01

/** @brief Command byte for PWM duty cycle control */
#define PROTO_CMD_SET_PWM      0x02

/** @brief Command byte for ADC read request */
#define PROTO_CMD_READ_ADC     0x03

/** @brief Command byte for start ADC streaming */
#define PROTO_CMD_START_STREAM 0x04

/** @brief Command byte for stop ADC streaming */
#define PROTO_CMD_STOP_STREAM  0x05

/** @brief Positive response (ACK) */
#define PROTO_RESP_ACK         0xFF

/** @brief Negative response (NACK) */
#define PROTO_RESP_NACK        0xFE

/** @brief Debug response marker */
#define PROTO_RESP_DEBUG       0xFD

/** @brief Error code: CRC mismatch */
#define PROTO_ERR_CRC          0x01

/** @brief Error code: Invalid command */
#define PROTO_ERR_INVALID_CMD  0x02

/** @brief Error code: Invalid value */
#define PROTO_ERR_INVALID_VAL  0x03

/* =========================================================================
 * PROTOCOL STATE MACHINE
 * ========================================================================= */

/**
 * @brief Protocol state machine states
 */
enum proto_state {
	PROTO_STATE_WAIT_CMD,               /**< Waiting for command byte */
	PROTO_STATE_WAIT_VALUE,             /**< Waiting for value byte */
	PROTO_STATE_WAIT_CRC,               /**< Waiting for CRC byte (3-byte frame) */
	PROTO_STATE_WAIT_CRC_ADC,           /**< Waiting for CRC byte (2-byte frame for ADC) */
	PROTO_STATE_WAIT_STREAM_INT_H,      /**< Waiting for interval high byte (stream) */
	PROTO_STATE_WAIT_STREAM_INT_L,      /**< Waiting for interval low byte (stream) */
	PROTO_STATE_WAIT_STREAM_CRC,        /**< Waiting for CRC byte (START_STREAM) */
	PROTO_STATE_WAIT_STOP_STREAM_CRC,   /**< Waiting for CRC byte (STOP_STREAM) */
};

/* =========================================================================
 * CALLBACK TYPES
 * ========================================================================= */

/**
 * @brief Callback type for LED set command
 *
 * @param state LED state (0=OFF, 1+=ON)
 */
typedef void (*proto_led_set_cb)(uint8_t state);

/**
 * @brief Callback type for PWM duty set command
 *
 * @param duty Duty cycle (0-100)
 * @return 0 on success, -1 on invalid value
 */
typedef int (*proto_pwm_set_cb)(uint8_t duty);

/**
 * @brief Callback type for ADC read command
 *
 * @return ADC value (0-4095), or negative on error
 */
typedef int (*proto_adc_read_cb)(void);

/**
 * @brief Callback type for start ADC streaming
 *
 * @param interval_ms Sampling interval in milliseconds
 * @return 0 on success, negative on failure
 */
typedef int (*proto_stream_start_cb)(uint32_t interval_ms);

/**
 * @brief Callback type for stop ADC streaming
 *
 * @return 0 on success, negative on failure
 */
typedef int (*proto_stream_stop_cb)(void);

/**
 * @brief Callback type for sending response byte
 *
 * @param resp Response byte (PROTO_RESP_ACK or PROTO_RESP_NACK)
 */
typedef void (*proto_send_resp_cb)(uint8_t resp);

/**
 * @brief Callback type for sending ADC response
 *
 * @param cmd Command byte (PROTO_CMD_READ_ADC)
 * @param adc_h ADC high byte
 * @param adc_l ADC low byte
 * @param crc CRC byte
 */
typedef void (*proto_send_adc_resp_cb)(uint8_t cmd, uint8_t adc_h, uint8_t adc_l, uint8_t crc);

/**
 * @brief Callback type for sending debug packet
 *
 * @param debug_code Debug code (1-255)
 */
typedef void (*proto_send_debug_cb)(uint8_t debug_code);

/* =========================================================================
 * PROTOCOL CONTEXT
 * ========================================================================= */

/**
 * @brief Protocol context structure
 *
 * @details Holds protocol state and callback pointers for command handling.
 *          Callbacks must be registered before calling proto_process_byte().
 */
struct proto_ctx {
	/** Current state machine state */
	enum proto_state state;

	/** Frame buffer for CMD + VALUE + extra (for stream commands) */
	uint8_t frame_buf[3];

	/** Callback: LED set command */
	proto_led_set_cb on_led_set;

	/** Callback: PWM duty set command */
	proto_pwm_set_cb on_pwm_set;

	/** Callback: ADC read command */
	proto_adc_read_cb on_adc_read;

	/** Callback: Start ADC streaming */
	proto_stream_start_cb on_stream_start;

	/** Callback: Stop ADC streaming */
	proto_stream_stop_cb on_stream_stop;

	/** Callback: Send response byte */
	proto_send_resp_cb send_resp;

	/** Callback: Send ADC response */
	proto_send_adc_resp_cb send_adc_resp;

	/** Callback: Send debug packet */
	proto_send_debug_cb send_debug;
};

/* =========================================================================
 * FUNCTION DECLARATIONS
 * ========================================================================= */

/**
 * @brief Initialize protocol context
 *
 * @details Resets state machine to WAIT_CMD and clears frame buffer.
 *          Callbacks must be set after initialization.
 *
 * @param ctx Protocol context to initialize
 */
void proto_init(struct proto_ctx *ctx);

/**
 * @brief Process received byte through protocol state machine
 *
 * @details Implements protocol parser with support for:
 *          - 3-byte commands: [CMD][VALUE][CRC8] (SET_LED, SET_PWM)
 *          - 2-byte commands: [CMD][CRC8] (READ_ADC)
 *
 *          Calls registered callbacks on valid commands.
 *
 * @param ctx Protocol context
 * @param byte Received byte from UART
 */
void proto_process_byte(struct proto_ctx *ctx, uint8_t byte);

/**
 * @brief Calculate CRC-8 checksum
 *
 * @details Uses polynomial 0x07 with initial value 0x00.
 *          This is used to verify integrity of protocol frames.
 *
 * @param data Input data buffer
 * @param len Length of data in bytes
 * @return CRC-8 checksum (0-255)
 */
uint8_t proto_crc8(const uint8_t *data, size_t len);

/**
 * @brief Send debug packet via protocol
 *
 * @details Sends 2-byte debug packet: [DEBUG_MARKER][CODE]
 *
 * @param ctx Protocol context
 * @param debug_code Debug code (1-255)
 */
void proto_send_debug(struct proto_ctx *ctx, uint8_t debug_code);

#endif /* PROTOCOL_H */
