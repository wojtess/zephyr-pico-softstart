/**
 * @file adc_stream.h
 * @brief ADC streaming module with periodic sampling
 *
 * @details Timer-based periodic ADC sampling with configurable interval.
 *          ADC values are queued to TX buffer for transmission.
 */

#ifndef ADC_STREAM_H
#define ADC_STREAM_H

#include <zephyr/kernel.h>
#include <stdint.h>

/* =========================================================================
 * CONSTANTS
 * ========================================================================= */

/** @brief ADC stream command: start streaming */
#define ADC_STREAM_CMD_START  0x04

/** @brief ADC stream command: stop streaming */
#define ADC_STREAM_CMD_STOP   0x05

/** @brief Minimum streaming interval (ms) */
#define ADC_STREAM_MIN_INTERVAL_MS  10

/** @brief Maximum streaming interval (ms) */
#define ADC_STREAM_MAX_INTERVAL_MS  60000

/** @brief Default streaming interval (ms) */
#define ADC_STREAM_DEFAULT_INTERVAL_MS  100

/* =========================================================================
 * CONTEXT STRUCTURE
 * ========================================================================= */

/**
 * @brief ADC stream context
 *
 * @details Holds timer, state, and ADC/TX buffer references.
 *          Must be initialized with adc_stream_init() before use.
 */
struct adc_stream_ctx {
	/** Timer for periodic sampling */
	struct k_timer timer;

	/** ADC driver context */
	struct adc_ctrl_ctx *adc;

	/** TX buffer for sending ADC values */
	struct tx_buffer *tx_buf;

	/** Streaming active flag */
	volatile bool active;

	/** Sampling interval (ms) */
	uint32_t interval_ms;

	/** Current ADC value (cached) */
	uint16_t last_value;
};

/* =========================================================================
 * FUNCTION DECLARATIONS
 * ========================================================================= */

/**
 * @brief Initialize ADC stream module
 *
 * @details Initializes timer and sets up context.
 *
 * @param ctx ADC stream context to initialize
 * @param adc ADC driver context
 * @param tx_buf TX buffer for sending ADC values
 * @return 0 on success, negative errno on failure
 */
int adc_stream_init(struct adc_stream_ctx *ctx,
		    struct adc_ctrl_ctx *adc,
		    struct tx_buffer *tx_buf);

/**
 * @brief Start ADC streaming
 *
 * @details Starts periodic ADC sampling at specified interval.
 *
 * @param ctx ADC stream context
 * @param interval_ms Sampling interval in milliseconds (10-60000)
 * @return 0 on success, negative errno on failure
 */
int adc_stream_start(struct adc_stream_ctx *ctx, uint32_t interval_ms);

/**
 * @brief Stop ADC streaming
 *
 * @details Stops periodic ADC sampling.
 *
 * @param ctx ADC stream context
 * @return 0 on success, negative errno on failure
 */
int adc_stream_stop(struct adc_stream_ctx *ctx);

/**
 * @brief Check if streaming is active
 *
 * @param ctx ADC stream context
 * @return true if streaming, false otherwise
 */
bool adc_stream_is_active(const struct adc_stream_ctx *ctx);

/**
 * @brief Get current streaming interval
 *
 * @param ctx ADC stream context
 * @return Interval in milliseconds
 */
uint32_t adc_stream_get_interval(const struct adc_stream_ctx *ctx);

#endif /* ADC_STREAM_H */
