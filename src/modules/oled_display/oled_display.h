/**
 * @file oled_display.h
 * @brief OLED display module for showing setpoint and ADC values
 *
 * @details Displays current setpoint and measured ADC value on SSD1306 OLED.
 *          Updates autonomously at 4 Hz (250ms interval) using timer + work queue.
 *          Uses Zephyr Character Framebuffer (CFB) for text rendering.
 *
 * Display layout (128x64, 8x8 font):
 *   Line 1: "SET: 4095" (setpoint value)
 *   Line 2: "ADC: 2048" (measured ADC value)
 */

#ifndef OLED_DISPLAY_H
#define OLED_DISPLAY_H

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations */
struct p_ctrl_ctx;
struct adc_reader_ctx;

/* =========================================================================
 * CONSTANTS
 * ========================================================================= */

/** @brief Default display update interval (4 Hz = 250ms) */
#define OLED_DISPLAY_UPDATE_INTERVAL_MS  250

/** @brief Maximum value for display (12-bit ADC) */
#define OLED_DISPLAY_MAX_VALUE  4095

/* =========================================================================
 * CONTEXT STRUCTURE
 * ========================================================================= */

/**
 * @brief OLED display context structure
 *
 * @details Holds display device, timer, work item, and module references.
 *          Must be initialized with oled_display_init() before use.
 *
 * @note IMPORTANT: Set all module references (p_ctrl, adc_reader) BEFORE
 *       calling oled_display_start(). Do not change references while active.
 *       This is safe in practice because main.c sets references before starting.
 */
struct oled_display_ctx {
    /** Display device pointer */
    const struct device *display_dev;

    /** Timer for periodic display updates */
    struct k_timer timer;

    /** Work item for display update (must be in thread context) */
    struct k_work work;

    /** PI-controller reference (for setpoint) - volatile for thread safety */
    volatile struct p_ctrl_ctx *p_ctrl;

    /** ADC reader reference (for ADC value) - volatile for thread safety */
    volatile struct adc_reader_ctx *adc_reader;

    /** Initialization flag */
    bool initialized;

    /** Active flag (atomic access) */
    atomic_t active;
};

/* =========================================================================
 * FUNCTION DECLARATIONS
 * ========================================================================= */

/**
 * @brief Initialize OLED display module
 *
 * @details Initializes display device, character framebuffer, and context structure.
 *          Does NOT start display updates - use oled_display_start() after init.
 *
 * @param ctx OLED display context to initialize
 * @return 0 on success, negative errno on failure
 */
int oled_display_init(struct oled_display_ctx *ctx);

/**
 * @brief Start periodic display updates
 *
 * @details Starts timer-based display updates at the configured interval.
 *          Display will show current setpoint and ADC values.
 *
 * @param ctx OLED display context
 * @return 0 on success, negative errno on failure
 */
int oled_display_start(struct oled_display_ctx *ctx);

/**
 * @brief Stop periodic display updates
 *
 * @details Stops timer-based display updates. Display content is frozen.
 *
 * @param ctx OLED display context
 * @return 0 on success, negative errno on failure
 */
int oled_display_stop(struct oled_display_ctx *ctx);

/**
 * @brief Set PI-controller reference
 *
 * @details Sets the PI-controller reference for reading setpoint values.
 *
 * @param ctx OLED display context
 * @param p_ctrl Pointer to PI-controller context
 */
void oled_display_set_p_controller(struct oled_display_ctx *ctx,
                                    struct p_ctrl_ctx *p_ctrl);

/**
 * @brief Set ADC reader reference
 *
 * @details Sets the ADC reader reference for reading measured values.
 *
 * @param ctx OLED display context
 * @param adc_reader Pointer to ADC reader context
 */
void oled_display_set_adc_reader(struct oled_display_ctx *ctx,
                                  struct adc_reader_ctx *adc_reader);

#ifdef __cplusplus
}
#endif

#endif /* OLED_DISPLAY_H */
