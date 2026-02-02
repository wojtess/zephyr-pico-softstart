/**
 * @file p_controller.h
 * @brief PI-controller for current regulation
 *
 * Formula: PWM = feed_forward + (Kp * error) / 100 + (Ki * integral) / 100
 *
 * The PI-controller operates at 1 kHz (1ms interval) and supports two modes:
 * - MANUAL: PWM is controlled directly via SET_PWM command
 * - AUTO: PI-controller calculates PWM automatically based on ADC feedback
 * - Anti-windup: integral sum is clamped and back-calculated on output saturation
 */

#pragma once

#include <zephyr/kernel.h>
#include <stdint.h>

/* Forward declaration */
struct adc_reader_ctx;

#ifdef __cplusplus
extern "C" {
#endif

/** @brief PI-controller operation modes */
#define P_CTRL_MODE_MANUAL     0  /**< Manual PWM control */
#define P_CTRL_MODE_AUTO       1  /**< Automatic PI-control */

/** @brief PI-controller parameter ranges */
#define P_CTRL_SETPOINT_MIN   0     /**< Minimum setpoint (ADC) */
#define P_CTRL_SETPOINT_MAX   4095  /**< Maximum setpoint (ADC) */
#define P_CTRL_GAIN_MIN       0     /**< Minimum proportional gain (Kp) */
#define P_CTRL_GAIN_MAX       1000  /**< Maximum proportional gain (Kp, represents 10.0) */
#define P_CTRL_KI_MIN         0     /**< Minimum integral gain (Ki) */
#define P_CTRL_KI_MAX         1000  /**< Maximum integral gain (Ki, represents 10.0) */
#define P_CTRL_FF_MIN         0     /**< Minimum feed-forward */
#define P_CTRL_FF_MAX         100   /**< Maximum feed-forward (PWM %) */
#define P_CTRL_PWM_MIN        0     /**< Minimum PWM output */
#define P_CTRL_PWM_MAX        100   /**< Maximum PWM output */

/** @brief Anti-windup limits for integral term */
#define P_CTRL_INTEGRAL_LIMIT 100000 /**< Maximum absolute integral sum */

/** @brief Streaming configuration */
#define P_CTRL_STREAM_HZ_DEFAULT  100  /**< Default streaming rate (Hz) */
#define P_CTRL_CONTROL_HZ         1000 /**< Control loop frequency (Hz) */

/**
 * @brief PI-controller context structure
 *
 * Contains all state, parameters, and callbacks for the PI-controller.
 */
struct p_ctrl_ctx {
    /** @brief Timer for 1kHz control loop */
    struct k_timer timer;

    /** @brief Work item for control calculations */
    struct k_work work;

    /** @brief Current operation mode (atomic access required) */
    atomic_t mode;

    /** @brief Target ADC value (atomic access required) */
    atomic_t setpoint;

    /** @brief Proportional gain (atomic access required) */
    atomic_t gain;

    /** @brief Integral gain (atomic access required) */
    atomic_t ki;

    /** @brief Integral sum (non-atomic, work queue only) */
    int32_t integral_sum;

    /** @brief Base PWM bias (atomic access required) */
    atomic_t feed_forward;

    /** @brief Last measured ADC value */
    uint16_t last_measured;

    /** @brief Last calculated PWM output */
    uint8_t last_pwm;

    /** @brief Is streaming active */
    bool streaming;

    /** @brief Streaming interval in control ticks (1 tick = 1ms) */
    uint16_t stream_interval_ticks;

    /** @decimation counter for streaming */
    uint16_t stream_counter;

    /** @brief Callback to set PWM duty cycle */
    void (*on_pwm_set)(uint8_t duty);

    /** @brief Callback to send streaming data */
    void (*on_stream_data)(uint16_t setpoint, uint16_t measured, uint8_t pwm);

    /** @brief ADC reader reference (shared ADC source) */
    struct adc_reader_ctx *adc_reader;

    /** @brief Initialization flag */
    bool initialized;
};

/**
 * @brief Initialize PI-controller
 *
 * @param ctx Pointer to PI-controller context
 * @return 0 on success, negative errno on failure
 */
int p_ctrl_init(struct p_ctrl_ctx *ctx);

/**
 * @brief Set PI-controller operation mode
 *
 * @param ctx Pointer to PI-controller context
 * @param mode P_CTRL_MODE_MANUAL or P_CTRL_MODE_AUTO
 */
void p_ctrl_set_mode(struct p_ctrl_ctx *ctx, uint8_t mode);

/**
 * @brief Set target ADC value (setpoint)
 *
 * @param ctx Pointer to PI-controller context
 * @param setpoint Target value (0-4095)
 */
void p_ctrl_set_setpoint(struct p_ctrl_ctx *ctx, uint16_t setpoint);

/**
 * @brief Set proportional gain
 *
 * @param ctx Pointer to PI-controller context
 * @param gain Proportional gain value (0-1000, represents 0.0-10.0)
 */
void p_ctrl_set_gain(struct p_ctrl_ctx *ctx, uint16_t gain);

/**
 * @brief Set integral gain
 *
 * @param ctx Pointer to PI-controller context
 * @param ki Integral gain value (0-1000, represents 0.0-10.0)
 */
void p_ctrl_set_ki(struct p_ctrl_ctx *ctx, uint16_t ki);

/**
 * @brief Reset integral sum (anti-windup reset)
 *
 * @param ctx Pointer to PI-controller context
 */
void p_ctrl_reset_integral(struct p_ctrl_ctx *ctx);

/**
 * @brief Set feed-forward value
 *
 * @param ctx Pointer to PI-controller context
 * @param ff Feed-forward PWM (0-100)
 */
void p_ctrl_set_feed_forward(struct p_ctrl_ctx *ctx, uint8_t ff);

/**
 * @brief Register callbacks for PWM and streaming
 *
 * @param ctx Pointer to PI-controller context
 * @param pwm_set Callback to set PWM duty cycle
 * @param stream_data Callback to send streaming data
 */
void p_ctrl_set_callbacks(struct p_ctrl_ctx *ctx,
                         void (*pwm_set)(uint8_t),
                         void (*stream_data)(uint16_t, uint16_t, uint8_t));

/**
 * @brief Set ADC reader reference
 *
 * @param ctx Pointer to PI-controller context
 * @param adc_reader Pointer to ADC reader context
 */
void p_ctrl_set_adc_reader(struct p_ctrl_ctx *ctx, struct adc_reader_ctx *adc_reader);

/**
 * @brief Start streaming control data
 *
 * @param ctx Pointer to PI-controller context
 * @param rate_hz Streaming rate in Hz (max 1000)
 * @return 0 on success, negative errno on failure
 */
int p_ctrl_start_stream(struct p_ctrl_ctx *ctx, uint32_t rate_hz);

/**
 * @brief Stop streaming control data
 *
 * @param ctx Pointer to PI-controller context
 * @return 0 on success, negative errno on failure
 */
int p_ctrl_stop_stream(struct p_ctrl_ctx *ctx);

/**
 * @brief Get current PWM output
 *
 * @param ctx Pointer to PI-controller context
 * @return Current PWM duty cycle (0-100)
 */
uint8_t p_ctrl_get_pwm(struct p_ctrl_ctx *ctx);

/**
 * @brief Get last measured ADC value
 *
 * @param ctx Pointer to PI-controller context
 * @return Last ADC reading
 */
uint16_t p_ctrl_get_measured(struct p_ctrl_ctx *ctx);

/**
 * @brief Get current setpoint value
 *
 * @param ctx Pointer to PI-controller context
 * @return Current setpoint (0-4095)
 */
uint16_t p_ctrl_get_setpoint(struct p_ctrl_ctx *ctx);

#ifdef __cplusplus
}
#endif
