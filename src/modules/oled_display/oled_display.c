/**
 * @file oled_display.c
 * @brief OLED display module implementation
 *
 * @details Implements autonomous display updates for setpoint and ADC values.
 *          Uses Zephyr's Character Framebuffer (CFB) for text rendering.
 *          Update rate: 4 Hz (250ms) to balance readability and CPU load.
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

/** @brief Font height for CFB (default 8 pixels) */
#define DISPLAY_FONT_HEIGHT  8

/** @brief Display text positions (calculated from font height) */
#define DISPLAY_LINE1_Y  0   /**< First line Y position */
#define DISPLAY_LINE2_Y  DISPLAY_FONT_HEIGHT  /**< Second line Y position */

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
    uint16_t setpoint = 0;
    uint16_t adc_value = 0;

    if (ctx->p_ctrl != NULL) {
        setpoint = p_ctrl_get_setpoint(ctx->p_ctrl);
    }

    if (ctx->adc_reader != NULL) {
        adc_value = adc_reader_get_last(ctx->adc_reader);
    }

    /* Format display strings */
    char line1[16];
    char line2[16];

    snprintf(line1, sizeof(line1), "SET: %4u", setpoint);
    snprintf(line2, sizeof(line2), "ADC: %4u", adc_value);

    /* Clear and update display */
    cfb_framebuffer_clear(ctx->display_dev, false);

    if (cfb_print(ctx->display_dev, line1, 0, DISPLAY_LINE1_Y) != 0) {
        LOG_WRN("Failed to print line 1");
    }

    if (cfb_print(ctx->display_dev, line2, 0, DISPLAY_LINE2_Y) != 0) {
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

    /* Initialize character framebuffer */
    if (cfb_framebuffer_init(ctx->display_dev) != 0) {
        LOG_ERR("Framebuffer initialization failed");
        return -EIO;
    }

    /* Set pixel format (try MONO10 first, fallback to MONO01) */
    if (display_set_pixel_format(ctx->display_dev, PIXEL_FORMAT_MONO10) != 0) {
        if (display_set_pixel_format(ctx->display_dev, PIXEL_FORMAT_MONO01) != 0) {
            LOG_ERR("Failed to set pixel format");
            return -EIO;
        }
    }

    /* Clear display */
    cfb_framebuffer_clear(ctx->display_dev, true);

    /* Turn off display blanking */
    display_blanking_off(ctx->display_dev);

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
