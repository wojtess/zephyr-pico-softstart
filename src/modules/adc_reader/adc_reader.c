/**
 * @file adc_reader.c
 * @brief Single-source ADC reader implementation
 *
 * @details Provides periodic ADC sampling at a fixed rate.
 *          ADC values are stored in a shared atomic variable for
 *          consumption by P-controller and ADC streaming modules.
 *
 * @author Project Team
 * @version 0.1.0
 * @date 2026-02-01
 */

#include "adc_reader.h"
#include <zephyr/drivers/adc.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/__assert.h>
#include <string.h>

/* =========================================================================
 * CONSTANTS
 * ========================================================================= */

/** @brief ADC channel for GPIO26 (ADC0) */
#define ADC_CHANNEL_0  0

/** @brief ADC resolution (12-bit = 0-4095) */
#define ADC_RESOLUTION 12

/* =========================================================================
 * INTERNAL FUNCTIONS
 * ========================================================================= */

/**
 * @brief Apply IIR 1st order Low Pass Filter to raw ADC value
 *
 * @details Implements a single-pole IIR low-pass filter using fixed-point arithmetic.
 *          Transfer function: H(z) = α / (1 - (1-α)z^(-1))
 *          Difference equation: y[n] = α*x[n] + (1-α)*y[n-1]
 *
 *          Fixed-point implementation with runtime alpha:
 *          y[n] = (alpha_num * x[n] + (alpha_den - alpha_num) * y[n-1]) / alpha_den
 *
 *          For power-of-2 denominator, uses bit shift for efficiency.
 *
 *          α (alpha) determines filter response:
 *          - Small α = strong filtering, slow response
 *          - Large α = weak filtering, fast response
 *          - Cutoff frequency: fc = α × fs / (2π)
 *
 *          Filter characteristics (α = 1/256, fs = 20kHz):
 *          - Cutoff frequency: ~12Hz (-3dB point)
 *          - Attenuation at 1kHz PWM: ~-38dB (16× better than old 8-sample MA)
 *          - Group delay: ~12.5ms at low frequencies
 *          - Step response (10-90%): ~28ms, settling (99%): ~64ms
 *          - Excellent PWM noise rejection for softstart control
 *
 * @param ctx ADC reader context
 * @param raw_value Raw ADC value (0-4095)
 * @return Filtered ADC value
 */
static uint16_t apply_iir_lpf(struct adc_reader_ctx *ctx, uint16_t raw_value)
{
	uint32_t y_new;

	/* Use runtime alpha parameters */
	uint8_t alpha_num = ctx->alpha_num;
	uint8_t alpha_den = ctx->alpha_den;

	if (ctx->alpha_shift > 0) {
		/* Power-of-2 denominator: use bit shift for efficiency
		 * y[n] = (alpha_num * x[n] + (alpha_den - alpha_num) * y[n-1]) >> alpha_shift */
		y_new = (alpha_num * raw_value + (alpha_den - alpha_num) * ctx->filter_state) >> ctx->alpha_shift;
	} else {
		/* Non-power-of-2 denominator: use division
		 * y[n] = (alpha_num * x[n] + (alpha_den - alpha_num) * y[n-1]) / alpha_den */
		y_new = (alpha_num * raw_value + (alpha_den - alpha_num) * ctx->filter_state) / alpha_den;
	}

	/* Update filter state for next iteration */
	ctx->filter_state = y_new;

	return (uint16_t)y_new;
}

/**
 * @brief Perform oversampling and return averaged value
 *
 * @details Reads ADC multiple times and returns the average value.
 *          Oversampling improves effective resolution:
 *          - 16x  = +2 bits (12-bit -> 14-bit)
 *          - 64x  = +3 bits (12-bit -> 15-bit)
 *          Uses sum + shift for efficient averaging.
 *
 * @param ctx ADC reader context
 * @return Averaged ADC value (0-4095), or 0 on error
 */
static uint16_t perform_oversample(struct adc_reader_ctx *ctx)
{
	uint32_t sum = 0;
	int16_t sample_buffer;  /* Use int16_t for signed ADC reads */
	int ret;

	/* Setup ADC sequence for single channel */
	struct adc_sequence sequence = {
		.channels = BIT(ADC_CHANNEL_0),
		.buffer = &sample_buffer,
		.buffer_size = sizeof(sample_buffer),
		.resolution = ADC_RESOLUTION,
	};

	/* Read ADC multiple times and accumulate */
	for (int i = 0; i < ADC_OVERSAMPLE_COUNT; i++) {
		/* Reset buffer before each read */
		sample_buffer = 0;

		ret = adc_read(ctx->adc_dev, &sequence);
		if (ret == 0) {
			sum += (uint16_t)sample_buffer;
		}

		/* Small delay between samples for ADC settling (~1µs)
		 * This ensures samples are independent and ADC has time to settle */
		k_busy_wait(1);
	}

	/* Average by shifting (divide by ADC_OVERSAMPLE_COUNT)
	 * For power-of-2 counts: 8->>3, 16->>4, 32->>5, 64->>6 */
#if (ADC_OVERSAMPLE_COUNT == 8)
	return (uint16_t)(sum >> 3);
#elif (ADC_OVERSAMPLE_COUNT == 16)
	return (uint16_t)(sum >> 4);
#elif (ADC_OVERSAMPLE_COUNT == 32)
	return (uint16_t)(sum >> 5);
#elif (ADC_OVERSAMPLE_COUNT == 64)
	return (uint16_t)(sum >> 6);
#else
	/* Fallback for non-power-of-2 (slow division) */
	return (uint16_t)(sum / ADC_OVERSAMPLE_COUNT);
#endif
}

/* =========================================================================
 * FORWARD DECLARATIONS
 * ========================================================================= */

/**
 * @brief Timer expiration callback (ISR context)
 *
 * @details Called when timer expires. Submits work to system work queue
 *          for actual ADC read (which must happen in thread context).
 *
 * @param timer Timer structure
 */
static void adc_reader_timer_expiry(struct k_timer *timer);

/**
 * @brief Work queue handler for ADC read (thread context)
 *
 * @details Reads ADC value and updates shared atomic variable.
 *          Called from system work queue context.
 *
 * @param work Work item
 */
static void adc_reader_work_handler(struct k_work *work);

/* =========================================================================
 * INTERNAL FUNCTIONS - IMPLEMENTATION
 * ========================================================================= */

/**
 * @brief Reset IIR filter state
 *
 * @details Resets the IIR filter state to zero.
 *          Should be called when starting or restarting acquisition.
 *
 * @param ctx ADC reader context
 */
static void adc_reader_reset_filter(struct adc_reader_ctx *ctx)
{
	ctx->filter_state = 0;
}

/* =========================================================================
 * WORK QUEUE HANDLER
 * ========================================================================= */

/**
 * @brief Work item for ADC read (thread context only)
 */
static void adc_reader_work_handler(struct k_work *work)
{
	struct adc_reader_ctx *ctx =
		CONTAINER_OF(work, struct adc_reader_ctx, work);

	uint16_t filtered_value = 0;

	if (!ctx->adc_dev) {
		return;
	}

	/* Verify ADC device is ready */
	if (!device_is_ready(ctx->adc_dev)) {
		printk("ERROR: ADC device not ready\n");
		return;
	}

	/* Apply filtering based on current mode */
	switch (ctx->filter_mode) {
	case ADC_FILTER_MODE_IIR_ONLY:
		/* Current behavior: single read + IIR */
		{
			int16_t sample_buffer;
			struct adc_sequence sequence = {
				.channels = BIT(ADC_CHANNEL_0),
				.buffer = &sample_buffer,
				.buffer_size = sizeof(sample_buffer),
				.resolution = ADC_RESOLUTION,
			};
			int ret = adc_read(ctx->adc_dev, &sequence);
			if (ret == 0) {
				filtered_value = apply_iir_lpf(ctx, (uint16_t)sample_buffer);
			} else {
				/* ADC error - don't update shared value */
				return;
			}
		}
		break;

	case ADC_FILTER_MODE_OVERSAMPLE_ONLY:
		/* 16x oversampling, no IIR */
		filtered_value = perform_oversample(ctx);
		break;

	case ADC_FILTER_MODE_OVERSAMPLE_IIR:
		/* 16x oversampling + IIR */
		{
			uint16_t oversampled = perform_oversample(ctx);
			filtered_value = apply_iir_lpf(ctx, oversampled);
		}
		break;

	default:
		printk("ERROR: Invalid filter mode %d\n", ctx->filter_mode);
		return;
	}

	/* Update shared value with FILTERED value (atomic write) */
	atomic_set(&ctx->last_adc_value, filtered_value);
	ctx->new_data_ready = true;
}

/* =========================================================================
 * TIMER CALLBACK
 * ========================================================================= */

/**
 * @brief Timer expiration callback
 *
 * @details Called periodically, submits work queue item for ADC read.
 *          ADC read must happen in thread context, not ISR.
 *
 * @param timer Timer structure
 */
static void adc_reader_timer_expiry(struct k_timer *timer)
{
	struct adc_reader_ctx *ctx =
		CONTAINER_OF(timer, struct adc_reader_ctx, timer);

	/* Submit work to system work queue (ADC read must be in thread context) */
	int ret = k_work_submit(&ctx->work);
	if (ret < 0) {
		/* Work queue full - log error but continue
		 * In production, could increment error counter for monitoring */
		/* Note: We can't use printk here safely in ISR context */
		(void)ret;  /* Suppress unused variable warning */
	}
}

/* =========================================================================
 * PUBLIC API
 * ========================================================================= */

/**
 * @brief Initialize ADC reader module
 */
int adc_reader_init(struct adc_reader_ctx *ctx,
		    const struct device *adc_dev,
		    uint32_t interval_us)
{
	if (!ctx || !adc_dev) {
		return -EINVAL;
	}

	if (!device_is_ready(adc_dev)) {
		return -ENODEV;
	}

	/* Validate interval */
	if (interval_us < ADC_READER_MIN_INTERVAL_US ||
	    interval_us > ADC_READER_MAX_INTERVAL_US) {
		return -EINVAL;
	}

	/* Initialize all fields to zero/safe values */
	memset(ctx, 0, sizeof(struct adc_reader_ctx));

	/* Set up context */
	ctx->adc_dev = adc_dev;
	ctx->interval_us = interval_us;
	ctx->active = false;
	ctx->new_data_ready = false;
	ctx->initialized = false;

	/* Initialize atomic value */
	atomic_set(&ctx->last_adc_value, 0);

	/* Initialize filter state (explicitly zero buffer) */
	adc_reader_reset_filter(ctx);

	/* Set default alpha values (1/256 for ~12Hz cutoff at 20kHz) */
	ctx->alpha_num = ADC_IIR_ALPHA_NUMERATOR;
	ctx->alpha_den = ADC_IIR_ALPHA_DENOMINATOR;
	ctx->alpha_shift = ADC_IIR_ALPHA_SHIFT;

	/* Set default filter mode (IIR only - current behavior) */
	ctx->filter_mode = ADC_FILTER_MODE_IIR_ONLY;

	/* Initialize timer with user data */
	k_timer_init(&ctx->timer, adc_reader_timer_expiry, NULL);

	/* Initialize work item */
	k_work_init(&ctx->work, adc_reader_work_handler);

	ctx->initialized = true;

	return 0;
}

/**
 * @brief Start periodic ADC reading
 */
int adc_reader_start(struct adc_reader_ctx *ctx)
{
	if (!ctx || !ctx->initialized) {
		return -EINVAL;
	}

	if (ctx->active) {
		/* Already running */
		return 0;
	}

	/* Reset filter state for clean start */
	adc_reader_reset_filter(ctx);

	/* Start periodic timer
	 * Note: k_timer_start() returns void in Zephyr, so we can't check for errors
	 * The active flag is set optimistically - if timer fails, no work will be submitted
	 */
	k_timer_start(&ctx->timer,
		      K_USEC(ctx->interval_us),
		      K_USEC(ctx->interval_us));

	ctx->active = true;

	return 0;
}

/**
 * @brief Stop periodic ADC reading
 */
int adc_reader_stop(struct adc_reader_ctx *ctx)
{
	if (!ctx || !ctx->initialized) {
		return -EINVAL;
	}

	if (!ctx->active) {
		/* Already stopped */
		return 0;
	}

	k_timer_stop(&ctx->timer);
	ctx->active = false;

	return 0;
}

/**
 * @brief Get last ADC value (atomic read)
 */
uint16_t adc_reader_get_last(struct adc_reader_ctx *ctx)
{
	if (!ctx || !ctx->initialized) {
		return 0;
	}

	return (uint16_t)atomic_get(&ctx->last_adc_value);
}

/**
 * @brief Check if new data is available
 */
bool adc_reader_has_new_data(const struct adc_reader_ctx *ctx)
{
	if (!ctx || !ctx->initialized) {
		return false;
	}

	return ctx->new_data_ready;
}

/**
 * @brief Clear new data flag
 */
void adc_reader_clear_new_data(struct adc_reader_ctx *ctx)
{
	if (!ctx || !ctx->initialized) {
		return;
	}

	ctx->new_data_ready = false;
}

/**
 * @brief Check if ADC reader is active
 */
bool adc_reader_is_active(const struct adc_reader_ctx *ctx)
{
	if (!ctx || !ctx->initialized) {
		return false;
	}

	return ctx->active;
}

/**
 * @brief Get current read interval
 */
uint32_t adc_reader_get_interval(const struct adc_reader_ctx *ctx)
{
	if (!ctx || !ctx->initialized) {
		return 0;
	}

	return ctx->interval_us;
}

/**
 * @brief Update read interval
 */
int adc_reader_set_interval(struct adc_reader_ctx *ctx, uint32_t interval_us)
{
	if (!ctx || !ctx->initialized) {
		return -EINVAL;
	}

	/* Validate interval */
	if (interval_us < ADC_READER_MIN_INTERVAL_US ||
	    interval_us > ADC_READER_MAX_INTERVAL_US) {
		return -EINVAL;
	}

	bool was_active = ctx->active;

	/* Stop if running */
	if (was_active) {
		k_timer_stop(&ctx->timer);
	}

	/* Update interval */
	ctx->interval_us = interval_us;

	/* Reset filter state when changing interval
	 * (different sampling rate = different filter characteristics) */
	adc_reader_reset_filter(ctx);

	/* Restart if was running */
	if (was_active) {
		k_timer_start(&ctx->timer,
			      K_USEC(ctx->interval_us),
			      K_USEC(ctx->interval_us));
	}

	return 0;
}

/**
 * @brief Set IIR filter alpha coefficient at runtime
 */
int adc_reader_set_alpha(struct adc_reader_ctx *ctx, uint8_t numerator, uint8_t denominator)
{
	if (!ctx || !ctx->initialized) {
		return -EINVAL;
	}

	/* Validate inputs */
	if (numerator == 0 || denominator == 0 || numerator > denominator) {
		return -EINVAL;
	}

	/* Check if denominator is power of 2 */
	uint8_t shift = 0;
	uint8_t temp = denominator;
	while (temp > 1) {
		if (temp & 1) {
			/* Not a power of 2 */
			shift = 0;
			break;
		}
		temp >>= 1;
		shift++;
	}

	/* Update alpha parameters */
	ctx->alpha_num = numerator;
	ctx->alpha_den = denominator;
	ctx->alpha_shift = shift;

	/* Reset filter state when changing alpha */
	adc_reader_reset_filter(ctx);

	return 0;
}

/**
 * @brief Set ADC filter mode
 */
int adc_reader_set_filter_mode(struct adc_reader_ctx *ctx, uint8_t mode)
{
	if (!ctx || !ctx->initialized) {
		return -EINVAL;
	}

	/* Validate mode */
	if (mode > ADC_FILTER_MODE_OVERSAMPLE_IIR) {
		return -EINVAL;
	}

	bool was_active = ctx->active;

	/* Stop if running (for clean transition) */
	if (was_active) {
		k_timer_stop(&ctx->timer);
	}

	/* Update mode */
	ctx->filter_mode = mode;

	/* NOTE: We do NOT reset filter state when switching modes!
	 * This preserves the IIR filter state and prevents the "cold filter" problem
	 * where output starts at 0 and takes time to converge. */

	/* Restart if was running */
	if (was_active) {
		k_timer_start(&ctx->timer,
			      K_USEC(ctx->interval_us),
			      K_USEC(ctx->interval_us));
	}

	return 0;
}

/**
 * @brief Get current ADC filter mode
 */
int adc_reader_get_filter_mode(struct adc_reader_ctx *ctx)
{
	if (!ctx || !ctx->initialized) {
		return -1;
	}
	return (int)ctx->filter_mode;
}
