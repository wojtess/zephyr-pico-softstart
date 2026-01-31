/**
 * @file adc_ctrl.h
 * @brief ADC driver module for RP2040
 *
 * @details Controls ADC on GPIO26 (ADC channel 0, 12-bit, 0-3.3V).
 */

#ifndef ADC_CTRL_H
#define ADC_CTRL_H

#include <zephyr/device.h>
#include <stdint.h>

/* =========================================================================
 * CONSTANTS
 * ========================================================================= */

/** @brief ADC channel for GPIO26 (ADC0) */
#define ADC_CTRL_CHANNEL_0  0

/** @brief ADC resolution (12-bit = 0-4095) */
#define ADC_CTRL_RESOLUTION 12

/* =========================================================================
 * CONTEXT STRUCTURE
 * ========================================================================= */

/**
 * @brief ADC driver context
 *
 * @details Holds device pointer for ADC control.
 *          Must be initialized with adc_ctrl_init() before use.
 */
struct adc_ctrl_ctx {
	/** ADC device pointer */
	const struct device *adc_dev;
};

/* =========================================================================
 * FUNCTION DECLARATIONS
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
int adc_ctrl_init(struct adc_ctrl_ctx *ctx, const struct device *adc_dev);

/**
 * @brief Read ADC value from channel 0 (GPIO26)
 *
 * @details Reads 12-bit ADC value (0-4095) from ADC channel 0.
 *          RP2040 ADC: 0V = 0, 3.3V = 4095
 *
 * @param ctx ADC context
 * @return ADC value (0-4095), or negative on error
 */
int adc_ctrl_read_channel0(struct adc_ctrl_ctx *ctx);

#endif /* ADC_CTRL_H */
