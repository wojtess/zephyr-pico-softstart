/**
 * @file p_controller.c
 * @brief P-controller implementation for current regulation
 *
 * Formula: PWM = feed_forward + (Kp * (setpoint - measured)) / 100
 *
 * Control loop runs at 1 kHz (1ms interval) using Zephyr timer + work queue.
 */

#include "p_controller.h"
#include "../adc_reader/adc_reader.h"
#include <zephyr/kernel.h>
#include <zephyr/sys/util_macro.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(p_controller, LOG_LEVEL_INF);

/* Constants */
#define CONTROL_INTERVAL_US  K_USEC(1000)  /* 1ms = 1kHz control loop */

/**
 * @brief Work queue handler for P-control calculation
 *
 * Called from system work queue context (not interrupt context).
 * Performs the actual P-control calculation and PWM output.
 */
static void p_ctrl_work_handler(struct k_work *work)
{
    struct p_ctrl_ctx *ctx = CONTAINER_OF(work, struct p_ctrl_ctx, work);

    /* Read current state atomically */
    uint8_t mode = atomic_get(&ctx->mode);
    uint16_t setpoint = atomic_get(&ctx->setpoint);
    uint16_t gain = atomic_get(&ctx->gain);
    uint8_t feed_forward = atomic_get(&ctx->feed_forward);

    uint8_t pwm_output = 0;
    uint16_t measured = 0;

    if (mode == P_CTRL_MODE_AUTO) {
        /* AUTO mode: Calculate P-control */
        if (ctx->adc_reader != NULL && adc_reader_is_active(ctx->adc_reader)) {
            measured = adc_reader_get_last(ctx->adc_reader);

            /* Handle first read (value = 0, no valid data yet) */
            if (measured == 0 && !adc_reader_has_new_data(ctx->adc_reader)) {
                /* No valid ADC data yet - use feed forward only */
                LOG_DBG("No valid ADC data yet, using feed-forward only");
                pwm_output = feed_forward;
                goto set_pwm;
            }

            ctx->last_measured = measured;

            /* Calculate error and correction
             * Formula: correction = (error * gain) / 100
             * where gain is scaled by 100 (0-1000 represents 0.0-10.0)
             */
            int16_t error = (int16_t)setpoint - (int16_t)measured;

            /* Calculate correction with proper scaling
             * gain range: 0-1000 (represents 0.0-10.0)
             * divide by 100 to get actual gain multiplier
             */
            int32_t correction = ((int32_t)error * (int32_t)gain) / 100;

            /* Calculate PWM: feed_forward + correction */
            int32_t pwm_calc = (int32_t)feed_forward + correction;

            /* Clamp to valid range */
            if (pwm_calc < P_CTRL_PWM_MIN) {
                pwm_calc = P_CTRL_PWM_MIN;
            } else if (pwm_calc > P_CTRL_PWM_MAX) {
                pwm_calc = P_CTRL_PWM_MAX;
            }

            pwm_output = (uint8_t)pwm_calc;
        } else {
            LOG_WRN("ADC reader not set or not active, using feed-forward only");
            pwm_output = feed_forward;
        }
    } else {
        /* MANUAL mode: Don't update PWM */
        return;
    }

set_pwm:
    /* Update stored PWM value */
    ctx->last_pwm = pwm_output;

    /* Apply PWM output */
    if (ctx->on_pwm_set != NULL) {
        ctx->on_pwm_set(pwm_output);
    }

    /* Handle streaming */
    if (ctx->streaming && ctx->on_stream_data != NULL) {
        ctx->stream_counter++;

        if (ctx->stream_counter >= ctx->stream_interval_ticks) {
            ctx->stream_counter = 0;

            /* Send streaming data: setpoint, measured, pwm */
            ctx->on_stream_data(setpoint, measured, pwm_output);
        }
    }
}

/**
 * @brief Timer expiration callback
 *
 * Called in interrupt context. Submits work to system work queue.
 */
static void p_ctrl_timer_expiry(struct k_timer *timer)
{
    struct p_ctrl_ctx *ctx = k_timer_user_data_get(timer);

    /* Submit work to system work queue */
    k_work_submit(&ctx->work);
}

int p_ctrl_init(struct p_ctrl_ctx *ctx)
{
    if (ctx == NULL) {
        return -EINVAL;
    }

    /* Initialize all fields to zero/safe values */
    memset(ctx, 0, sizeof(struct p_ctrl_ctx));

    /* Initialize atomic variables */
    atomic_set(&ctx->mode, P_CTRL_MODE_MANUAL);
    atomic_set(&ctx->setpoint, 0);
    atomic_set(&ctx->gain, 0);
    atomic_set(&ctx->feed_forward, 0);

    /* Initialize timer with user data */
    k_timer_init(&ctx->timer, p_ctrl_timer_expiry, NULL);
    k_timer_user_data_set(&ctx->timer, ctx);

    /* Initialize work item */
    k_work_init(&ctx->work, p_ctrl_work_handler);

    /* Initialize state */
    ctx->last_measured = 0;
    ctx->last_pwm = 0;
    ctx->streaming = false;
    ctx->stream_interval_ticks = P_CTRL_CONTROL_HZ / P_CTRL_STREAM_HZ_DEFAULT;
    ctx->stream_counter = 0;
    ctx->initialized = true;

    LOG_INF("P-controller initialized");

    return 0;
}

void p_ctrl_set_mode(struct p_ctrl_ctx *ctx, uint8_t mode)
{
    if (ctx == NULL || !ctx->initialized) {
        return;
    }

    uint8_t old_mode = atomic_get(&ctx->mode);

    /* Clamp mode to valid range */
    if (mode > P_CTRL_MODE_AUTO) {
        mode = P_CTRL_MODE_AUTO;
    }

    atomic_set(&ctx->mode, mode);

    /* Handle timer state based on mode */
    if (mode == P_CTRL_MODE_AUTO && old_mode == P_CTRL_MODE_MANUAL) {
        /* Transition to AUTO: Start timer */
        k_timer_start(&ctx->timer, CONTROL_INTERVAL_US, CONTROL_INTERVAL_US);
        LOG_INF("P-controller: MANUAL -> AUTO (timer started)");
    } else if (mode == P_CTRL_MODE_MANUAL && old_mode == P_CTRL_MODE_AUTO) {
        /* Transition to MANUAL: Stop timer */
        k_timer_stop(&ctx->timer);
        LOG_INF("P-controller: AUTO -> MANUAL (timer stopped)");
    }
}

void p_ctrl_set_setpoint(struct p_ctrl_ctx *ctx, uint16_t setpoint)
{
    if (ctx == NULL || !ctx->initialized) {
        return;
    }

    /* Clamp to valid range */
    if (setpoint > P_CTRL_SETPOINT_MAX) {
        setpoint = P_CTRL_SETPOINT_MAX;
    }

    atomic_set(&ctx->setpoint, setpoint);
}

void p_ctrl_set_gain(struct p_ctrl_ctx *ctx, uint16_t gain)
{
    if (ctx == NULL || !ctx->initialized) {
        return;
    }

    /* Clamp to valid range */
    if (gain > P_CTRL_GAIN_MAX) {
        gain = P_CTRL_GAIN_MAX;
    }

    atomic_set(&ctx->gain, gain);
}

void p_ctrl_set_feed_forward(struct p_ctrl_ctx *ctx, uint8_t ff)
{
    if (ctx == NULL || !ctx->initialized) {
        return;
    }

    /* Clamp to valid range */
    if (ff > P_CTRL_FF_MAX) {
        ff = P_CTRL_FF_MAX;
    }

    atomic_set(&ctx->feed_forward, ff);
}

void p_ctrl_set_callbacks(struct p_ctrl_ctx *ctx,
                         void (*pwm_set)(uint8_t),
                         void (*stream_data)(uint16_t, uint16_t, uint8_t))
{
    if (ctx == NULL || !ctx->initialized) {
        return;
    }

    ctx->on_pwm_set = pwm_set;
    ctx->on_stream_data = stream_data;
}

void p_ctrl_set_adc_reader(struct p_ctrl_ctx *ctx, struct adc_reader_ctx *adc_reader)
{
    if (ctx == NULL || !ctx->initialized) {
        return;
    }

    ctx->adc_reader = adc_reader;
}

int p_ctrl_start_stream(struct p_ctrl_ctx *ctx, uint32_t rate_hz)
{
    if (ctx == NULL || !ctx->initialized) {
        return -EINVAL;
    }

    if (rate_hz == 0 || rate_hz > P_CTRL_CONTROL_HZ) {
        LOG_ERR("Invalid streaming rate: %u Hz (max: %u Hz)", rate_hz, P_CTRL_CONTROL_HZ);
        return -EINVAL;
    }

    /* Calculate decimation interval */
    ctx->stream_interval_ticks = P_CTRL_CONTROL_HZ / rate_hz;
    ctx->stream_counter = 0;
    ctx->streaming = true;

    LOG_INF("Streaming started: %u Hz (decimation: %u)", rate_hz, ctx->stream_interval_ticks);

    return 0;
}

int p_ctrl_stop_stream(struct p_ctrl_ctx *ctx)
{
    if (ctx == NULL || !ctx->initialized) {
        return -EINVAL;
    }

    ctx->streaming = false;
    ctx->stream_counter = 0;

    LOG_INF("Streaming stopped");

    return 0;
}

uint8_t p_ctrl_get_pwm(struct p_ctrl_ctx *ctx)
{
    if (ctx == NULL || !ctx->initialized) {
        return 0;
    }

    return ctx->last_pwm;
}

uint16_t p_ctrl_get_measured(struct p_ctrl_ctx *ctx)
{
    if (ctx == NULL || !ctx->initialized) {
        return 0;
    }

    return ctx->last_measured;
}
