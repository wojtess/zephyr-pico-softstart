/**
 * @file adc_ctrl.c
 * @brief ADC driver implementation for RP2040
 */

#include "adc_ctrl.h"
#include <zephyr/drivers/adc.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/__assert.h>

/* =========================================================================
 * PUBLIC API
 * ========================================================================= */

/**
 * @brief Initialize ADC driver
 *
 * @details Initializes ADC device. Must be called before any ADC operations.
 *
 * @param ctx ADC context to initialize
 * @param adc_dev ADC device pointer
 * @return 0 on success, negative errno on failure
 */
int adc_ctrl_init(struct adc_ctrl_ctx *ctx, const struct device *adc_dev)
{
	if (!ctx || !adc_dev) {
		return -EINVAL;
	}

	if (!device_is_ready(adc_dev)) {
		return -ENODEV;
	}

	ctx->adc_dev = adc_dev;
	return 0;
}

/**
 * @brief Read ADC value from channel 0 (GPIO26)
 *
 * @details Reads 12-bit ADC value (0-4095) from ADC channel 0.
 *          RP2040 ADC: 0V = 0, 3.3V = 4095
 *
 * @param ctx ADC context
 * @return ADC value (0-4095), or negative on error
 */
int adc_ctrl_read_channel0(struct adc_ctrl_ctx *ctx)
{
	int ret;
	uint16_t adc_value = 0;
	int16_t sample_buffer;

	if (!ctx) {
		return -EINVAL;
	}

	/* Verify ADC device is ready */
	if (!device_is_ready(ctx->adc_dev)) {
		printk("ERROR: ADC device not ready\n");
		return -ENODEV;
	}

	/* Setup ADC sequence for single channel */
	struct adc_sequence sequence = {
		.channels = BIT(ADC_CTRL_CHANNEL_0),
		.buffer = &sample_buffer,
		.buffer_size = sizeof(sample_buffer),
		.resolution = ADC_CTRL_RESOLUTION,  /* 12-bit (0-4095) */
	};

	/* Read ADC */
	ret = adc_read(ctx->adc_dev, &sequence);
	if (ret != 0) {
		return ret;
	}

	adc_value = sample_buffer;
	return adc_value;
}
