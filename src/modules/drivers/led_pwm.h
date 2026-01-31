/**
 * @file led_pwm.h
 * @brief LED and PWM driver module
 *
 * @details Controls built-in LED (GPIO25) and PWM output (GPIO16).
 *          Independent controls - LED state does not affect PWM.
 */

#ifndef LED_PWM_H
#define LED_PWM_H

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <stdint.h>

/* =========================================================================
 * CONSTANTS
 * ========================================================================= */

/** @brief PWM frequency in Hz */
#define LED_PWM_FREQ         1000

/** @brief PWM channel for external output (GPIO16 = PWM0 Channel A) */
#define LED_PWM_CHANNEL_EXT  0

/* =========================================================================
 * CONTEXT STRUCTURE
 * ========================================================================= */

/**
 * @brief LED/PWM driver context
 *
 * @details Holds device pointers and state for LED and PWM control.
 *          Must be initialized with led_pwm_init() before use.
 */
struct led_pwm_ctx {
	/** PWM device pointer */
	const struct device *pwm_dev;

	/** GPIO pin specification for LED from device tree */
	const struct gpio_dt_spec *led;

	/** Current LED state (cached) */
	uint8_t led_state;

	/** Current PWM duty cycle (cached, 0-100) */
	uint8_t pwm_duty;
};

/* =========================================================================
 * FUNCTION DECLARATIONS
 * ========================================================================= */

/**
 * @brief Initialize LED/PWM driver
 *
 * @details Initializes GPIO and PWM devices. LED is set to OFF, PWM to 0%.
 *
 * @param ctx LED/PWM context to initialize
 * @param pwm_dev PWM device pointer
 * @param led GPIO DT spec for LED
 * @return 0 on success, negative errno on failure
 */
int led_pwm_init(struct led_pwm_ctx *ctx,
		 const struct device *pwm_dev,
		 const struct gpio_dt_spec *led);

/**
 * @brief Set LED state using GPIO (legacy)
 *
 * @details Turns built-in LED ON (state > 0) or OFF (state == 0).
 *          Does NOT affect PWM output - independent controls.
 *
 * @param ctx LED/PWM context
 * @param state LED state (0=OFF, 1+=ON)
 */
void led_pwm_set_led(struct led_pwm_ctx *ctx, uint8_t state);

/**
 * @brief Set PWM duty cycle
 *
 * @details Sets PWM duty cycle (0-100%).
 *          Duty cycle 0 = OFF, 100 = full brightness.
 *          Does NOT affect LED state - independent controls.
 *
 * @param ctx LED/PWM context
 * @param duty Duty cycle (0-100)
 * @return 0 on success, -1 on invalid value
 */
int led_pwm_set_duty(struct led_pwm_ctx *ctx, uint8_t duty);

/**
 * @brief Get current LED state
 *
 * @param ctx LED/PWM context
 * @return LED state (0=OFF, 1=ON)
 */
uint8_t led_pwm_get_led_state(const struct led_pwm_ctx *ctx);

/**
 * @brief Get current PWM duty cycle
 *
 * @param ctx LED/PWM context
 * @return PWM duty cycle (0-100)
 */
uint8_t led_pwm_get_duty(const struct led_pwm_ctx *ctx);

#endif /* LED_PWM_H */
