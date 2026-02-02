/**
 * @file oled_display.c
 * @brief OLED display module implementation
 *
 * @details Implements autonomous display updates for setpoint and measured current.
 *          Converts ADC values to current (amperes) using hardware constants:
 *          - Shunt resistor: 0.05 ohm
 *          - Op-amp gain: 56.6x
 *          - Vref: 3.3V
 *          - Current range: 0-1.17A
 *          Uses Zephyr's Character Framebuffer (CFB) for text rendering.
 *          Update rate: 4 Hz (250ms) to balance readability and CPU load.
 *
 * Display layout (128x64, Font 1: 15x24):
 *   Line 1: "S:0.85A" (setpoint current)
 *   Line 2: "M:0.42A" (measured current)
 */

#include "oled_display.h"
#include "../p_controller/p_controller.h"
#include "../adc_reader/adc_reader.h"
#include <zephyr/drivers/display.h>
#include <zephyr/display/cfb.h>
#include <zephyr/sys/printk.h>
#include <string.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(oled_display, LOG_LEVEL_INF);

/* =========================================================================
 * INTERNAL CONSTANTS
 * ========================================================================= */

/** @brief First line Y position */
#define DISPLAY_LINE1_Y  0

/** @brief Second line Y position (calculated as font_height from context) */

/** @brief Hardware constants for ADC to current conversion */
#define VREF 3.3f              ///< ADC reference voltage [V]
#define ADC_MAX 4095.0f        ///< 12-bit ADC max value
#define SHUNT_R 0.05f         ///< Shunt resistor [ohms]
#define OPAMP_GAIN 56.6f      ///< Op-amp gain

/** @brief ADC to current conversion factor [A per LSB]
 *
 * Formula: I = (ADC/4095 * VREF) / (GAIN * R_shunt)
 * I = ADC * (VREF / 4095 / GAIN / R_shunt)
 * I = ADC * ~0.000285 A/LSB
 */
#define AMPS_PER_ADC ((VREF / ADC_MAX) / OPAMP_GAIN / SHUNT_R)  // ~0.000285

/** @brief Scale factor for display (100 for centiamps) */
#define CURRENT_SCALE 100  ///< Display in centiamps (0.01A precision)

/* =========================================================================
 * INTERNAL FUNCTIONS
 * ========================================================================= */

/**
 * @brief Convert ADC value to current in centiamps
 *
 * @details Converts raw ADC value (0-4095) to current in centiamps (0-117).
 *          Uses integer math to avoid floating point overhead.
 *
 * Formula: I = (ADC * VREF) / (GAIN * R_shunt)
 *         I_cA = (ADC * VREF * 1000) / (GAIN * R_shunt * 10)
 *
 * @param adc_raw Raw ADC value (0-4095)
 * @return Current in centiamps (0-117, representing 0.00-1.17A)
 */
static int32_t adc_to_centiamps(uint16_t adc_raw)
{
    /* Calculate using integer math:
     * I[mV] = (ADC * 3300mV) / 4095
     * I[cA] = (I[mV] * 100) / (GAIN * R_shunt * 10)
     *      = (ADC * 3300 * 100) / (4095 * 56.6 * 0.05 * 10)
     *      = (ADC * 33000000) / 11559245 ≈ (ADC * 33000000) / 11559245
     * Simplified: (ADC * 3300) / (11559 * 10) ≈ ADC / 350
     */
    uint32_t numerator = (uint32_t)adc_raw * 3300;
    uint32_t denominator = 11559 * 10;  // 56.6 * 0.05 * 4095 / 10 ≈ 11559
    return (int32_t)(numerator / denominator);
}

/* =========================================================================
 * FORWARD DECLARATIONS
 * ========================================================================= */

static void oled_display_timer_expiry(struct k_timer *timer);
static void oled_display_work_handler(struct k_work *work);

/* =========================================================================
 * INTERNAL FUNCTIONS
 * ========================================================================= */

/**
 * @brief Work queue handler for display update
 *
 * @details Reads setpoint and ADC values, formats text, and updates display.
 *          Called from system work queue context (not interrupt context).
 *
 * @param work Work item
 */
static void oled_display_work_handler(struct k_work *work)
{
    struct oled_display_ctx *ctx =
        CONTAINER_OF(work, struct oled_display_ctx, work);

    if (!ctx->display_dev || !device_is_ready(ctx->display_dev)) {
        LOG_WRN("Display device not ready");
        return;
    }

    /* Read values from modules */
    uint16_t setpoint_adc = 0;
    uint16_t adc_value = 0;

    if (ctx->p_ctrl != NULL) {
        setpoint_adc = p_ctrl_get_setpoint(ctx->p_ctrl);
    }

    if (ctx->adc_reader != NULL) {
        adc_value = adc_reader_get_last(ctx->adc_reader);
    }

    /* Convert ADC to current in centiamps (0.01A precision) */
    int32_t setpoint_ca = adc_to_centiamps(setpoint_adc);
    int32_t measured_ca = adc_to_centiamps(adc_value);

    /* Format display strings - show current in Amps with 2 decimal places
     * "SET:0.85A" = 9 chars × 15px = 135px - might be tight but fits
     */
    char line1[16];
    char line2[16];

    snprintf(line1, sizeof(line1), "S:%d.%02dA",
             setpoint_ca / 100, setpoint_ca % 100);
    snprintf(line2, sizeof(line2), "M:%d.%02dA",
             measured_ca / 100, measured_ca % 100);

    /* Calculate line positions using actual font height */
    const uint8_t line1_y = DISPLAY_LINE1_Y;
    const uint8_t line2_y = ctx->font_height;  /* Use actual font height! */

    /* Clear and update display */
    cfb_framebuffer_clear(ctx->display_dev, false);

    if (cfb_print(ctx->display_dev, line1, 0, line1_y) != 0) {
        LOG_WRN("Failed to print line 1");
    }

    if (cfb_print(ctx->display_dev, line2, 0, line2_y) != 0) {
        LOG_WRN("Failed to print line 2");
    }

    cfb_framebuffer_finalize(ctx->display_dev);
}

/**
 * @brief Timer expiration callback
 *
 * @details Called periodically, submits work queue item for display update.
 *          Display operations must happen in thread context, not ISR.
 *
 * @param timer Timer structure
 */
static void oled_display_timer_expiry(struct k_timer *timer)
{
    struct oled_display_ctx *ctx =
        CONTAINER_OF(timer, struct oled_display_ctx, timer);

    /* Submit work to system work queue */
    k_work_submit(&ctx->work);
}

/* =========================================================================
 * PUBLIC API
 * ========================================================================= */

int oled_display_init(struct oled_display_ctx *ctx)
{
    if (!ctx) {
        return -EINVAL;
    }

    /* Initialize all fields to zero/safe values */
    memset(ctx, 0, sizeof(struct oled_display_ctx));

    /* Get display device from device tree */
    ctx->display_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_display));
    if (!device_is_ready(ctx->display_dev)) {
        LOG_ERR("Display device not ready");
        return -ENODEV;
    }

    /* IMPORTANT: Set pixel format BEFORE initializing CFB!
     * For OLED: try MONO10 first (1=white on black), fallback to MONO01 */
    if (display_set_pixel_format(ctx->display_dev, PIXEL_FORMAT_MONO10) != 0) {
        LOG_WRN("MONO10 failed, trying MONO01");
        if (display_set_pixel_format(ctx->display_dev, PIXEL_FORMAT_MONO01) != 0) {
            LOG_ERR("Failed to set pixel format");
            return -EIO;
        }
    }

    /* Initialize character framebuffer AFTER setting pixel format */
    if (cfb_framebuffer_init(ctx->display_dev) != 0) {
        LOG_ERR("Framebuffer initialization failed");
        return -EIO;
    }

    /* Clear display */
    cfb_framebuffer_clear(ctx->display_dev, true);

    /* Turn off display blanking */
    display_blanking_off(ctx->display_dev);

    /* Query and set available fonts */
    uint8_t font_count = cfb_get_numof_fonts(ctx->display_dev);

    for (uint8_t i = 0; i < font_count; ++i) {
        uint8_t w, h;
        cfb_get_font_size(ctx->display_dev, i, &w, &h);
    }

    /* Set font 1 (15x24) - good compromise: readable and fits 8 chars on 128px width
     * 8 chars × 15px = 120px < 128px (fits!)
     * 2 lines × 24px = 48px < 64px (fits!) */
    const uint8_t selected_font = 1;
    if (cfb_framebuffer_set_font(ctx->display_dev, selected_font) != 0) {
        LOG_WRN("Failed to set font %d, trying font 0", selected_font);
        cfb_framebuffer_set_font(ctx->display_dev, 0);
        cfb_get_font_size(ctx->display_dev, 0, &ctx->font_width, &ctx->font_height);
    } else {
        cfb_get_font_size(ctx->display_dev, selected_font, &ctx->font_width, &ctx->font_height);
    }
    LOG_INF("Selected font: %dx%d", ctx->font_width, ctx->font_height);

    /* Get display parameters */
    uint16_t x_res = cfb_get_display_parameter(ctx->display_dev, CFB_DISPLAY_WIDTH);
    uint16_t y_res = cfb_get_display_parameter(ctx->display_dev, CFB_DISPLAY_HEIGHT);
    uint8_t ppt = cfb_get_display_parameter(ctx->display_dev, CFB_DISPLAY_PPT);
    uint8_t rows = cfb_get_display_parameter(ctx->display_dev, CFB_DISPLAY_ROWS);
    uint16_t cols = cfb_get_display_parameter(ctx->display_dev, CFB_DISPLAY_COLS);

    LOG_INF("Display initialized: %dx%d, ppt=%d, rows=%d, cols=%d",
            x_res, y_res, ppt, rows, cols);

    /* Initialize timer with user data */
    k_timer_init(&ctx->timer, oled_display_timer_expiry, NULL);
    k_timer_user_data_set(&ctx->timer, ctx);

    /* Initialize work item */
    k_work_init(&ctx->work, oled_display_work_handler);

    /* Set initial state */
    ctx->p_ctrl = NULL;
    ctx->adc_reader = NULL;
    atomic_set(&ctx->active, 0);
    ctx->initialized = true;

    return 0;
}

int oled_display_start(struct oled_display_ctx *ctx)
{
    if (!ctx || !ctx->initialized) {
        return -EINVAL;
    }

    if (atomic_get(&ctx->active) != 0) {
        /* Already running */
        return 0;
    }

    /* Start periodic timer */
    k_timer_start(&ctx->timer, K_MSEC(OLED_DISPLAY_UPDATE_INTERVAL_MS),
                  K_MSEC(OLED_DISPLAY_UPDATE_INTERVAL_MS));

    atomic_set(&ctx->active, 1);

    LOG_INF("Display updates started (interval: %d ms)", OLED_DISPLAY_UPDATE_INTERVAL_MS);

    return 0;
}

int oled_display_stop(struct oled_display_ctx *ctx)
{
    if (!ctx || !ctx->initialized) {
        return -EINVAL;
    }

    if (atomic_get(&ctx->active) == 0) {
        /* Already stopped */
        return 0;
    }

    k_timer_stop(&ctx->timer);
    atomic_set(&ctx->active, 0);

    LOG_INF("Display updates stopped");

    return 0;
}

void oled_display_set_p_controller(struct oled_display_ctx *ctx,
                                    struct p_ctrl_ctx *p_ctrl)
{
    if (!ctx || !ctx->initialized) {
        return;
    }

    ctx->p_ctrl = p_ctrl;
}

void oled_display_set_adc_reader(struct oled_display_ctx *ctx,
                                  struct adc_reader_ctx *adc_reader)
{
    if (!ctx || !ctx->initialized) {
        return;
    }

    ctx->adc_reader = adc_reader;
}
