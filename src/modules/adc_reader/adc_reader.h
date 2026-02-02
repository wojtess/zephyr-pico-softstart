/**
 * @file adc_reader.h
 * @brief Shared ADC reader module for periodic sampling
 *
 * @details Provides a single source of ADC readings for multiple modules.
 *          Uses timer-based periodic sampling to avoid ADC race conditions.
 *          Thread-safe: last value can be read from any context.
 */

#ifndef ADC_READER_H
#define ADC_READER_H

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================================
 * CONSTANTS
 * ========================================================================= */

/** @brief Default ADC read interval (50µs = 20kHz) */
#define ADC_READER_DEFAULT_INTERVAL_US  50

/** @brief Maximum ADC read interval (µs) - 1 second */
#define ADC_READER_MAX_INTERVAL_US     1000000

/** @brief Minimum ADC read interval (µs) - 10kHz for safety */
#define ADC_READER_MIN_INTERVAL_US     10

/** @brief IIR LPF alpha coefficient (fixed-point: 1/256 for ~12Hz cutoff at 20kHz)
 *
 * @details α (alpha) determines filter response speed:
 *          - Formula: y[n] = α·x[n] + (1-α)·y[n-1]
 *          - Small α = strong filtering, slow response
 *          - Large α = weak filtering, fast response
 *          - Cutoff freq: fc = α × fs / (2π)
 *          - For α=1/256, fs=20kHz: fc ≈ 12Hz
 */
#define ADC_IIR_ALPHA_NUMERATOR   1
#define ADC_IIR_ALPHA_DENOMINATOR 256
#define ADC_IIR_ALPHA_SHIFT       8  /* log2(256) for efficient fixed-point */

/* =========================================================================
 * CONTEXT STRUCTURE
 * ========================================================================= */

/**
 * @brief ADC reader context
 *
 * @details Holds timer, work item, state, and shared ADC value.
 *          Must be initialized with adc_reader_init() before use.
 */
struct adc_reader_ctx {
	/** Timer for periodic ADC sampling */
	struct k_timer timer;

	/** Work item for ADC read (must be in thread context) */
	struct k_work work;

	/** ADC device pointer */
	const struct device *adc_dev;

	/** Shared ADC value (atomic access) - FILTERED value */
	atomic_t last_adc_value;

	/** Flag indicating new data is available */
	volatile bool new_data_ready;

	/** ADC read interval in microseconds */
	uint32_t interval_us;

	/** Active flag */
	volatile bool active;

	/** Initialization flag */
	bool initialized;

	/** IIR filter state (previous output y[n-1]) */
	uint32_t filter_state;
};

/* =========================================================================
 * FUNCTION DECLARATIONS
 * ========================================================================= */

/**
 * @brief Initialize ADC reader module
 *
 * @details Initializes timer, work item, and context structure.
 *          Does NOT start ADC sampling - use adc_reader_start() after init.
 *
 * @param ctx ADC reader context to initialize
 * @param adc_dev ADC device pointer
 * @param interval_us ADC read interval in microseconds (10-1000000)
 * @return 0 on success, negative errno on failure
 */
int adc_reader_init(struct adc_reader_ctx *ctx,
		    const struct device *adc_dev,
		    uint32_t interval_us);

/**
 * @brief Start periodic ADC reading
 *
 * @details Starts timer-based ADC sampling at the configured interval.
 *          New ADC values are written to shared atomic variable.
 *
 * @param ctx ADC reader context
 * @return 0 on success, negative errno on failure
 */
int adc_reader_start(struct adc_reader_ctx *ctx);

/**
 * @brief Stop periodic ADC reading
 *
 * @details Stops timer-based ADC sampling.
 *
 * @param ctx ADC reader context
 * @return 0 on success, negative errno on failure
 */
int adc_reader_stop(struct adc_reader_ctx *ctx);

/**
 * @brief Get last ADC value (atomic read)
 *
 * @details Returns the most recent ADC value from shared atomic variable.
 *          This is a non-blocking, thread-safe read.
 *
 * @param ctx ADC reader context
 * @return ADC value (0-4095), or 0 if not initialized
 */
uint16_t adc_reader_get_last(struct adc_reader_ctx *ctx);

/**
 * @brief Check if new data is available
 *
 * @details Returns true if new ADC value has been read since last check.
 *          Note: This flag is NOT cleared automatically.
 *
 * @param ctx ADC reader context
 * @return true if new data available, false otherwise
 */
bool adc_reader_has_new_data(const struct adc_reader_ctx *ctx);

/**
 * @brief Clear new data flag
 *
 * @details Manually clears the new_data_ready flag.
 *
 * @param ctx ADC reader context
 */
void adc_reader_clear_new_data(struct adc_reader_ctx *ctx);

/**
 * @brief Check if ADC reader is active
 *
 * @param ctx ADC reader context
 * @return true if active, false otherwise
 */
bool adc_reader_is_active(const struct adc_reader_ctx *ctx);

/**
 * @brief Get current read interval
 *
 * @param ctx ADC reader context
 * @return Interval in microseconds
 */
uint32_t adc_reader_get_interval(const struct adc_reader_ctx *ctx);

/**
 * @brief Update read interval
 *
 * @details Updates the ADC read interval. If reader is active,
 *          it will be restarted with the new interval.
 *
 * @param ctx ADC reader context
 * @param interval_us New interval in microseconds (10-1000000)
 * @return 0 on success, negative errno on failure
 */
int adc_reader_set_interval(struct adc_reader_ctx *ctx, uint32_t interval_us);

#ifdef __cplusplus
}
#endif

#endif /* ADC_READER_H */
