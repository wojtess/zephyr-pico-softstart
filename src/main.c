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
#include "modules/drivers/led_pwm.h"
#include "modules/drivers/adc_ctrl.h"
#include "modules/tx_buffer/tx_buffer.h"
#include "modules/threads/tx_thread.h"

/* =========================================================================
 * CONSTANTS AND DEFINITIONS
 * ========================================================================= */

/** @brief UART device node from device tree */
#define UART_NODE DT_NODELABEL(cdc_acm_uart0)

/** @brief LED device node from device tree */
#define LED_NODE DT_NODELABEL(led0)

/** @brief PWM device (all channels) */
#define PWM_DEV_NODE DT_NODELABEL(pwm)

/** @brief ADC device node from device tree */
#define ADC_NODE DT_NODELABEL(adc)

/* -------------------------------------------------------------------------
 * Buffer Sizes
 * ------------------------------------------------------------------------- */
/** @brief UART RX ring buffer size in bytes */
#define RX_BUF_SIZE  32

/** @brief UART TX ring buffer size in bytes */
#define TX_BUF_SIZE  32

/* =========================================================================
 * GLOBAL VARIABLES
 * ========================================================================= */

/** @brief Cached UART device pointer */
static const struct device *uart_dev;

/** @brief Cached PWM device pointer */
static const struct device *pwm_dev;

/** @brief LED GPIO DT spec (for led_pwm module) */
static const struct gpio_dt_spec led_dt_spec = GPIO_DT_SPEC_GET(LED_NODE, gpios);

/** @brief LED/PWM driver context */
static struct led_pwm_ctx g_led_pwm;

/** @brief ADC driver context */
static struct adc_ctrl_ctx g_adc;

/* =========================================================================
 * FUNCTION PROTOTYPES
 * ========================================================================= */

/** @brief UART RX ring buffer data */
static uint8_t rx_buf_data[RX_BUF_SIZE];

/** @brief RX ring buffer instance */
static struct ringbuf g_rx_buf;

/** @brief UART TX ring buffer data */
static uint8_t tx_buf_data[TX_BUF_SIZE];

/** @brief TX ring buffer instance */
static struct tx_buffer g_tx_buf;

/** @brief TX thread context */
static struct tx_thread_ctx g_tx_thread;

/** @brief Protocol context with state machine and callbacks */
static struct proto_ctx g_proto;

/* =========================================================================
 * FUNCTION PROTOTYPES
 * ========================================================================= */

/**
 * @brief Send response byte via UART (callback for protocol module)
 *
 * @param resp Response byte (PROTO_RESP_ACK or PROTO_RESP_NACK)
 */
static void send_response_cb(uint8_t resp)
{
	tx_buffer_put(&g_tx_buf, &resp, 1);
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

	tx_buffer_put(&g_tx_buf, response, sizeof(response));
}

/**
 * @brief LED set callback for protocol module
 *
 * @param state LED state (0=OFF, 1+=ON)
 */
static void led_set_cb(uint8_t state)
{
	led_pwm_set_led(&g_led_pwm, state);
}

/**
 * @brief PWM duty set callback for protocol module
 *
 * @param duty Duty cycle (0-100)
 * @return 0 on success, -1 on invalid value
 */
static int pwm_set_cb(uint8_t duty)
{
	return led_pwm_set_duty(&g_led_pwm, duty);
}

/**
 * @brief ADC read callback for protocol module
 *
 * @return ADC value (0-4095), or negative on error
 */
static int adc_read_cb(void)
{
	return adc_ctrl_read_channel0(&g_adc);
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
	const struct device *adc_dev = DEVICE_DT_GET(ADC_NODE);

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

	/* Initialize LED/PWM driver */
	if (led_pwm_init(&g_led_pwm, pwm_dev, &led_dt_spec) != 0) {
		printk("ERROR: Failed to initialize LED/PWM driver\n");
		return 0;
	}
	printk("LED/PWM initialized: LED OFF (PWM 0%%)\n");

	/* Initialize ADC driver */
	if (adc_ctrl_init(&g_adc, adc_dev) != 0) {
		printk("ERROR: Failed to initialize ADC driver\n");
		return 0;
	}
	printk("ADC initialized on GPIO26 (ADC0)\n");

	/* Initialize RX ring buffer */
	if (ringbuf_init(&g_rx_buf, rx_buf_data, RX_BUF_SIZE) != 0) {
		printk("ERROR: Failed to initialize RX ring buffer\n");
		return 0;
	}

	/* Initialize TX ring buffer */
	if (tx_buffer_init(&g_tx_buf, tx_buf_data, TX_BUF_SIZE) != 0) {
		printk("ERROR: Failed to initialize TX ring buffer\n");
		return 0;
	}

	/* Start TX thread */
	if (tx_thread_start(&g_tx_thread, &g_tx_buf, uart_dev) != 0) {
		printk("ERROR: Failed to start TX thread\n");
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

	/* Main loop */
	while (true) {
		process_ring_buffer();
		k_sleep(K_MSEC(10));
	}

	return 0;
}
