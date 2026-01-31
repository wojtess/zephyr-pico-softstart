/**
 * @file led_pwm.c
 * @brief LED and PWM driver implementation
 */

#include "led_pwm.h"
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/drivers/gpio.h>

/* =========================================================================
 * INTERNAL HELPERS
 * ========================================================================= */

/**
 * @brief Set PWM using microseconds (helper for Zephyr API)
 *
 * @param ctx LED/PWM context
 * @param pulse_usec Pulse width in microseconds
 * @param period_usec Period in microseconds
 * @return 0 on success, negative on error
 */
static int pwm_set_usec(struct led_pwm_ctx *ctx, uint32_t pulse_usec, uint32_t period_usec)
{
	uint64_t cycles_per_sec;
	int ret;

	/* Get clock frequency */
	ret = pwm_get_cycles_per_sec(ctx->pwm_dev, LED_PWM_CHANNEL_EXT, &cycles_per_sec);
	if (ret != 0) {
		return ret;
	}

	/* Convert microseconds to cycles */
	uint64_t period_cycles = (period_usec * cycles_per_sec) / 1000000ULL;
	uint64_t pulse_cycles = (pulse_usec * cycles_per_sec) / 1000000ULL;

	return pwm_set_cycles(ctx->pwm_dev, LED_PWM_CHANNEL_EXT, period_cycles, pulse_cycles, 0);
}

/* =========================================================================
 * PUBLIC API
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
		 const struct gpio_dt_spec *led)
{
	if (!ctx || !pwm_dev || !led) {
		return -EINVAL;
	}

	if (!device_is_ready(pwm_dev)) {
		return -ENODEV;
	}

	if (!gpio_is_ready_dt(led)) {
		return -ENODEV;
	}

	ctx->pwm_dev = pwm_dev;
	ctx->led = led;
	ctx->led_state = 0;
	ctx->pwm_duty = 0;

	/* Configure LED GPIO as output */
	gpio_pin_configure_dt(led, GPIO_OUTPUT);

	/* Initialize: LED OFF, PWM 0% */
	gpio_pin_set_dt(led, 0);
	pwm_set_usec(ctx, 0, 1000000 / LED_PWM_FREQ);

	return 0;
}

/**
 * @brief Set LED state using GPIO (legacy)
 *
 * @details Turns built-in LED ON (state > 0) or OFF (state == 0).
 *          Does NOT affect PWM output - independent controls.
 *
 * @param ctx LED/PWM context
 * @param state LED state (0=OFF, 1+=ON)
 */
void led_pwm_set_led(struct led_pwm_ctx *ctx, uint8_t state)
{
	if (!ctx) {
		return;
	}

	int value = (state > 0) ? 1 : 0;

	gpio_pin_set_dt(ctx->led, value);
	ctx->led_state = value;
}

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
int led_pwm_set_duty(struct led_pwm_ctx *ctx, uint8_t duty)
{
	if (!ctx) {
		return -EINVAL;
	}

	if (duty > 100) {
		return -1;  /* Invalid value */
	}

	/* Duty cycle 0 = OFF */
	if (duty == 0) {
		pwm_set_usec(ctx, 0, 1000000 / LED_PWM_FREQ);
		ctx->pwm_duty = 0;
		return 0;
	}

	/* Calculate pulse period in microseconds */
	uint32_t period = 1000000 / LED_PWM_FREQ;  // 1000us for 1kHz

	/* Calculate pulse width for duty cycle */
	uint32_t pulse_width = (period * duty) / 100;

	/* Set PWM */
	int ret = pwm_set_usec(ctx, pulse_width, period);

	if (ret == 0) {
		ctx->pwm_duty = duty;
	}

	return ret;
}

/**
 * @brief Get current LED state
 *
 * @param ctx LED/PWM context
 * @return LED state (0=OFF, 1=ON)
 */
uint8_t led_pwm_get_led_state(const struct led_pwm_ctx *ctx)
{
	if (!ctx) {
		return 0;
	}
	return ctx->led_state;
}

/**
 * @brief Get current PWM duty cycle
 *
 * @param ctx LED/PWM context
 * @return PWM duty cycle (0-100)
 */
uint8_t led_pwm_get_duty(const struct led_pwm_ctx *ctx)
{
	if (!ctx) {
		return 0;
	}
	return ctx->pwm_duty;
}
