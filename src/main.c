/**
 * @file main.c
 * @brief RP2040 PWM LED Control via UART ACM Binary Protocol
 *
 * @details This firmware implements a PWM-based LED control system for RP2040.
 *          Future versions will extend this to softstart current control.
 *
 * **Hardware Platform:**
 *   - MCU: RP2040 (Raspberry Pi Pico)
 *   - LED: GPIO25 (built-in LED, PWM capable)
 *   - ADC Input: GPIO26 (ADC channel 0) - for analog sensor reading
 *   - Communication: USB CDC ACM (virtual serial port)
 *
 * **Binary Protocol Specification:**
 *   - Frame Format: [CMD][VALUE][CRC8] (3 bytes)
 *   - CMD 0x01: SET LED (0=OFF, 1+=ON) - Legacy ON/OFF
 *   - CMD 0x02: SET PWM DUTY (0-100, 0%=OFF, 100%=full brightness)
 *   - CMD 0x03: READ ADC - Request ADC reading, response: [0x03][ADC_H][ADC_L][CRC8]
 *     - ADC_H: High byte of ADC value (0-3.3V maps to 0-4095, 12-bit)
 *     - ADC_L: Low byte of ADC value
 *   - Response: ACK 0xFF or NACK 0xFE + error code
 *   - CRC-8: Polynomial 0x07, Initial 0x00
 *
 * **Project Roadmap:**
 *   1. LED ON/OFF control (legacy)
 *   2. PWM duty cycle control (current implementation)
 *   3. ADC analog input reading (current implementation)
 *   4. Softstart with current regulation (1% accuracy)
 *
 * @author Project Team
 * @version 0.3.0
 * @date 2026-01-30
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/spinlock.h>
#include <zephyr/sys/__assert.h>
#include "modules/ringbuf/ringbuf.h"
#include "modules/protocol/protocol.h"

/* =========================================================================
 * CONSTANTS AND DEFINITIONS
 * ========================================================================= */

/** @brief UART device node from device tree */
#define UART_NODE DT_NODELABEL(cdc_acm_uart0)

/** @brief LED device node from device tree */
#define LED_NODE DT_NODELABEL(led0)

/** @brief PWM device (all channels) */
#define PWM_DEV_NODE DT_NODELABEL(pwm)

/** @brief PWM channel for external output (GPIO16 = PWM0 Channel A) */
#define PWM_CHANNEL_EXT 0

/** @brief PWM frequency in Hz */
#define PWM_FREQ 1000

/** @brief ADC device node from device tree */
#define ADC_NODE DT_NODELABEL(adc)

/** @brief ADC channel for GPIO26 (ADC0) */
#define ADC_CHANNEL_0 0

/* -------------------------------------------------------------------------
 * Buffer Sizes
 * ------------------------------------------------------------------------- */
/** @brief UART RX ring buffer size in bytes */
#define RX_BUF_SIZE  32

/* =========================================================================
 * GLOBAL VARIABLES
 * ========================================================================= */

/** @brief Current LED state (atomic for thread safety) */
static atomic_val_t led_state = ATOMIC_INIT(0);

/** @brief Current PWM duty cycle (0-100, atomic for thread safety) */
static atomic_val_t pwm_duty = ATOMIC_INIT(0);

/** @brief Cached UART device pointer */
static const struct device *uart_dev;

/** @brief Cached PWM device pointer */
static const struct device *pwm_dev;

/** @brief Cached ADC device pointer */
static const struct device *adc_dev;

/** @brief GPIO pin specification for LED from device tree */
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED_NODE, gpios);

/* =========================================================================
 * FUNCTION PROTOTYPES
 * ========================================================================= */

/** @brief UART RX ring buffer data */
static uint8_t rx_buf_data[RX_BUF_SIZE];

/** @brief RX ring buffer instance */
static struct ringbuf g_rx_buf;

/** @brief Protocol context with state machine and callbacks */
static struct proto_ctx g_proto;

/* =========================================================================
 * FUNCTION PROTOTYPES
 * ========================================================================= */

/* Forward declarations for callbacks */
static void set_led(uint8_t state);
static int set_pwm_duty(uint8_t duty);
static int read_adc_channel0(void);

/**
 * @brief Send response byte via UART (callback for protocol module)
 *
 * @param resp Response byte (PROTO_RESP_ACK or PROTO_RESP_NACK)
 */
static void send_response_cb(uint8_t resp)
{
	uart_fifo_fill(uart_dev, &resp, 1);
}

/**
 * @brief Send ADC response frame (callback for protocol module)
 *
 * @details Sends 4-byte response: [CMD][ADC_H][ADC_L][CRC8]
 *
 * @param cmd Command byte (PROTO_CMD_READ_ADC)
 * @param adc_h ADC high byte
 * @param adc_l ADC low byte
 * @param crc CRC byte
 */
static void send_adc_response_cb(uint8_t cmd, uint8_t adc_h, uint8_t adc_l, uint8_t crc)
{
	uint8_t response[4];

	response[0] = cmd;
	response[1] = adc_h;
	response[2] = adc_l;
	response[3] = crc;

	uart_fifo_fill(uart_dev, response, sizeof(response));
}

/**
 * @brief LED set callback for protocol module
 *
 * @param state LED state (0=OFF, 1+=ON)
 */
static void led_set_cb(uint8_t state)
{
	set_led(state);
}

/**
 * @brief PWM duty set callback for protocol module
 *
 * @param duty Duty cycle (0-100)
 * @return 0 on success, -1 on invalid value
 */
static int pwm_set_cb(uint8_t duty)
{
	return set_pwm_duty(duty);
}

/**
 * @brief ADC read callback for protocol module
 *
 * @return ADC value (0-4095), or negative on error
 */
static int adc_read_cb(void)
{
	return read_adc_channel0();
}

/**
 * @brief Set PWM using microseconds (helper for new API)
 *
 * @param channel PWM channel
 * @param pulse_usec Pulse width in microseconds
 * @param period_usec Period in microseconds
 * @return 0 on success, negative on error
 */
static int pwm_set_usec(uint32_t channel, uint32_t pulse_usec, uint32_t period_usec)
{
	uint64_t cycles_per_sec;
	int ret;

	/* Get clock frequency */
	ret = pwm_get_cycles_per_sec(pwm_dev, channel, &cycles_per_sec);
	if (ret != 0) {
		return ret;
	}

	/* Convert microseconds to cycles */
	uint64_t period_cycles = (period_usec * cycles_per_sec) / 1000000ULL;
	uint64_t pulse_cycles = (pulse_usec * cycles_per_sec) / 1000000ULL;

	return pwm_set_cycles(pwm_dev, channel, period_cycles, pulse_cycles, 0);
}

/* =========================================================================
 * LED/PWM CONTROL FUNCTIONS
 * ========================================================================= */

/**
 * @brief Set LED state using GPIO (legacy)
 *
 * @details Turns built-in LED ON (state > 0) or OFF (state == 0).
 *          LED is on GPIO25, PWM output is on GPIO16 - independent controls.
 *
 * @param state LED state (0=OFF, 1+=ON)
 */
static void set_led(uint8_t state)
{
	int value = (state > 0) ? 1 : 0;

	gpio_pin_set_dt(&led, value);
	atomic_set(&led_state, value);
}

/**
 * @brief Set PWM duty cycle for LED
 *
 * @details Sets PWM duty cycle (0-100%).
 *          Duty cycle 0 = LED OFF, 100 = LED full brightness.
 *
 * @param duty Duty cycle (0-100)
 * @return 0 on success, -1 on invalid value
 */
static int set_pwm_duty(uint8_t duty)
{
	if (duty > 100) {
		return -1;  /* Invalid value */
	}

	/* Duty cycle 0 = OFF */
	if (duty == 0) {
		pwm_set_usec(PWM_CHANNEL_EXT, 0, 1000000 / PWM_FREQ);
		gpio_pin_set_dt(&led, 0);
		atomic_set(&pwm_duty, 0);
		atomic_set(&led_state, 0);
		return 0;
	}

	/* Calculate pulse period in microseconds */
	uint32_t period = 1000000 / PWM_FREQ;  // 1000us for 1kHz

	/* Calculate pulse width for duty cycle */
	uint32_t pulse_width = (period * duty) / 100;

	/* Set PWM */
	int ret = pwm_set_usec(PWM_CHANNEL_EXT, pulse_width, period);

	if (ret == 0) {
		atomic_set(&pwm_duty, duty);
		atomic_set(&led_state, duty > 0 ? 1 : 0);
	}

	return ret;
}

/* =========================================================================
 * ADC READ FUNCTION
 * ========================================================================= */

/**
 * @brief Read ADC value from GPIO26 (ADC channel 0)
 *
 * @details Reads 12-bit ADC value (0-4095) from ADC channel 0.
 *          RP2040 ADC: 0V = 0, 3.3V = 4095
 *
 * @return ADC value (0-4095), or negative on error
 */
static int read_adc_channel0(void)
{
	int ret;
	uint16_t adc_value = 0;
	int16_t sample_buffer;

	/* Verify ADC device is ready */
	if (!device_is_ready(adc_dev)) {
		printk("ERROR: ADC device not ready\n");
		return -1;
	}

	/* Setup ADC sequence for single channel */
	struct adc_sequence sequence = {
		.channels = BIT(ADC_CHANNEL_0),
		.buffer = &sample_buffer,
		.buffer_size = sizeof(sample_buffer),
		.resolution = 12,  /* 12-bit resolution (0-4095) */
	};

	/* Read ADC */
	ret = adc_read(adc_dev, &sequence);
	if (ret != 0) {
		return ret;
	}

	adc_value = sample_buffer;
	return adc_value;
}

/**
 * @brief Send ADC reading as response
 *
 * @details Sends [CMD][ADC_H][ADC_L][CRC8] response
 *
 * @param adc_value 12-bit ADC value (0-4095)
 */
/**
 * @brief Clear ring buffer by resetting head and tail pointers
 *
 * @details Thread-safe with spinlock protection
 */
static void ring_buf_clear(void)
{
	ringbuf_clear(&g_rx_buf);
}

/* =========================================================================
 * RING BUFFER FUNCTIONS (ISR-safe)
 * ========================================================================= */

/**
 * @brief Put byte into ring buffer (ISR-safe, thread-safe)
 *
 * @param byte Byte to add
 * @return 0 on success, -ENOBUFS if buffer full
 */
static int ring_buf_put_byte(uint8_t byte)
{
	return ringbuf_put(&g_rx_buf, byte);
}

/**
 * @brief Get byte from ring buffer (thread-safe)
 *
 * @param byte Pointer to store retrieved byte
 * @return 0 on success, -ENOMSG if buffer empty
 */
static int ring_buf_get_byte(uint8_t *byte)
{
	return ringbuf_get(&g_rx_buf, byte);
}

/* =========================================================================
 * UART INTERRUPT HANDLERS
 * ========================================================================= */

/**
 * @brief UART RX interrupt handler (ISR context)
 *
 * @details Called by UART interrupt to read bytes into ring buffer.
 *          Must be ISR-safe (no blocking operations).
 *
 * @param dev UART device structure
 * @param user_data User data (unused)
 */
static void uart_rx_handler(const struct device *dev, void *user_data)
{
	uint8_t buf[16];
	uint32_t len;

	while (uart_irq_update(dev) && uart_irq_rx_ready(dev)) {
		len = uart_fifo_read(dev, buf, sizeof(buf));

		for (uint32_t i = 0; i < len; i++) {
			ring_buf_put_byte(buf[i]);
		}
	}
}

/**
 * @brief Process ring buffer contents (main loop context)
 *
 * @details Reads bytes from ring buffer and processes them
 *          through the protocol state machine.
 */
static void process_ring_buffer(void)
{
	uint8_t byte;

	while (ring_buf_get_byte(&byte) == 0) {
		proto_process_byte(&g_proto, byte);
	}
}

/* =========================================================================
 * MAIN
 * ========================================================================= */

/**
 * @brief Main application entry point
 *
 * @details Initializes UART, PWM, GPIO, and interrupt handlers.
 *          Waits for USB DTR signal before entering main loop.
 *          Main loop processes incoming protocol frames at 10ms intervals.
 *
 * @return 0 on success (never reached in normal operation)
 */
int main(void)
{
	uint32_t dtr = 0;

	/* Get device pointers */
	uart_dev = DEVICE_DT_GET(UART_NODE);
	pwm_dev = DEVICE_DT_GET(PWM_DEV_NODE);
	adc_dev = DEVICE_DT_GET(ADC_NODE);

	/* Verify UART is ready */
	if (!device_is_ready(uart_dev)) {
		printk("ERROR: UART device not ready\n");
		return 0;
	}

	/* Verify PWM is ready */
	if (!device_is_ready(pwm_dev)) {
		printk("ERROR: PWM device not ready\n");
		return 0;
	}

	/* Verify ADC is ready */
	if (!device_is_ready(adc_dev)) {
		printk("ERROR: ADC device not ready\n");
		return 0;
	}
	printk("ADC initialized on GPIO26 (ADC0)\n");

	/* Configure LED GPIO */
	if (!gpio_is_ready_dt(&led)) {
		printk("ERROR: LED device not ready\n");
		return 0;
	}

	gpio_pin_configure_dt(&led, GPIO_OUTPUT);

	/* Initialize RX ring buffer */
	if (ringbuf_init(&g_rx_buf, rx_buf_data, RX_BUF_SIZE) != 0) {
		printk("ERROR: Failed to initialize RX ring buffer\n");
		return 0;
	}

	/* Initialize protocol context */
	proto_init(&g_proto);

	/* Register protocol callbacks */
	g_proto.on_led_set = led_set_cb;
	g_proto.on_pwm_set = pwm_set_cb;
	g_proto.on_adc_read = adc_read_cb;
	g_proto.send_resp = send_response_cb;
	g_proto.send_adc_resp = send_adc_response_cb;

	/* Wait for DTR */
	printk("Waiting for USB connection...\n");
	while (!dtr) {
		uart_line_ctrl_get(uart_dev, UART_LINE_CTRL_DTR, &dtr);
		k_sleep(K_MSEC(100));
	}

	printk("===================================\n");
	printk("RP2040 PWM LED + ADC Control\n");
	printk("LED: GPIO25 (built-in)\n");
	printk("PWM: 1kHz frequency\n");
	printk("ADC: GPIO26 (12-bit, 0-3.3V)\n");
	printk("Protocol: [CMD][VALUE][CRC8]\n");
	printk("  0x01 <val> <crc>  -> Set LED (0=OFF, 1+=ON)\n");
	printk("  0x02 <0-100> <crc> -> Set PWM duty (0=OFF, 100=full)\n");
	printk("  0x03 <crc>        -> Read ADC (returns [0x03][ADC_H][ADC_L][CRC])\n");
	printk("  Response: ACK=0xFF, NACK=0xFE\n");
	printk("===================================\n\n");

	/* Setup UART RX interrupt */
	uart_irq_callback_set(uart_dev, uart_rx_handler);
	uart_irq_rx_enable(uart_dev);

	/* Initialize: LED OFF, PWM 0% */
	set_led(0);
	printk("LED initialized: OFF (PWM 0%%)\n");

	/* Main loop */
	while (true) {
		process_ring_buffer();
		k_sleep(K_MSEC(10));
	}

	return 0;
}
