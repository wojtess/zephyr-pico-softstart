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
 *          Fixed-point implementation (α = 1/256):
 *          y[n] = (x[n] + 255*y[n-1]) / 256
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
	/* Fixed-point IIR filter: y[n] = (x[n] + 255 * y[n-1]) / 256
	 * Using 32-bit arithmetic to avoid overflow during multiplication
	 * Filter state holds y[n-1] from previous sample */
	uint32_t y_new = (raw_value + 255U * ctx->filter_state) >> ADC_IIR_ALPHA_SHIFT;

	/* Update filter state for next iteration */
	ctx->filter_state = y_new;

	return (uint16_t)y_new;
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

	int ret;
	uint16_t adc_value = 0;
	int16_t sample_buffer;

	if (!ctx->adc_dev) {
		return;
	}

	/* Verify ADC device is ready */
	if (!device_is_ready(ctx->adc_dev)) {
		printk("ERROR: ADC device not ready\n");
		return;
	}

	/* Setup ADC sequence for single channel */
	struct adc_sequence sequence = {
		.channels = BIT(ADC_CHANNEL_0),
		.buffer = &sample_buffer,
		.buffer_size = sizeof(sample_buffer),
		.resolution = ADC_RESOLUTION,  /* 12-bit (0-4095) */
	};

	/* Read ADC */
	ret = adc_read(ctx->adc_dev, &sequence);
	if (ret != 0) {
		/* ADC error - don't update shared value */
		return;
	}

	adc_value = (uint16_t)sample_buffer;

	/* Apply IIR low-pass filter to remove PWM noise */
	uint16_t filtered_value = apply_iir_lpf(ctx, adc_value);

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
