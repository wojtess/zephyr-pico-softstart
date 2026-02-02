/**
 * @file main.c
 * @brief RP2040 PWM LED Control with P-Controller via UART ACM Binary Protocol
 *
 * @details This firmware implements a PWM-based LED control system for RP2040
 *          with PI-controller for automatic current regulation.
 *
 * **Hardware Platform:**
 *   - MCU: RP2040 (Raspberry Pi Pico)
 *   - LED: GPIO25 (built-in LED, PWM capable)
 *   - ADC Input: GPIO26 (ADC channel 0) - for analog sensor reading
 *   - Communication: USB CDC ACM (virtual serial port)
 *
 * **Binary Protocol Specification:**
 *   - Frame Format: [CMD][VALUE][CRC8] (3 bytes) or [CMD][H][L][CRC8] (4 bytes)
 *   - CMD 0x01: SET LED (0=OFF, 1+=ON) - Legacy ON/OFF
 *   - CMD 0x02: SET PWM DUTY (0-100, 0%=OFF, 100%=full brightness)
 *   - CMD 0x03: READ ADC - Request ADC reading, response: [0x03][ADC_H][ADC_L][CRC8]
 *   - CMD 0x06: SET MODE (0=MANUAL, 1=AUTO)
 *   - CMD 0x07: SET P SETPOINT (H/L bytes, 0-4095 ADC = 0-1.17A)
 *   - CMD 0x08: SET P GAIN (H/L bytes, 0-1000, represents 0.0-10.0)
 *   - CMD 0x09: START P STREAM (rate H/L in Hz)
 *   - CMD 0x0A: STOP P STREAM
 *   - CMD 0x0C: SET FEED FORWARD (0-100)
 *   - Response: ACK 0xFF or NACK 0xFE + error code
 *   - CRC-8: Polynomial 0x07, Initial 0x00
 *
 * **P-Controller Operation:**
 *   - Mode 0 (MANUAL): PWM controlled by GUI via SET_PWM (0x02)
 *   - Mode 1 (AUTO): PI-controller calculates PWM automatically
 *   - Formula: PWM = feed_forward + (Kp * (setpoint - measured)) / 100
 *   - Control loop runs at 1kHz (1ms interval)
 *
 * @author Project Team
 * @version 0.4.0
 * @date 2026-02-01
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
#include "modules/adc_reader/adc_reader.h"
#include "modules/tx_buffer/tx_buffer.h"
#include "modules/threads/tx_thread.h"
#include "modules/threads/rx_thread.h"
#include "modules/adc_stream/adc_stream.h"
#include "modules/p_controller/p_controller.h"
#include "modules/oled_display/oled_display.h"

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

/** @brief ADC reader context (shared ADC source) */
static struct adc_reader_ctx g_adc_reader;

/** @brief PI-controller context */
static struct p_ctrl_ctx g_p_ctrl;

/** @brief OLED display context */
static struct oled_display_ctx g_oled_display;

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

/** @brief RX thread context */
static struct rx_thread_ctx g_rx_thread;

/** @brief ADC stream context */
static struct adc_stream_ctx g_adc_stream;

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
	/* Return the latest value from shared ADC reader */
	return (int)adc_reader_get_last(&g_adc_reader);
}

/**
 * @brief Start ADC streaming callback for protocol module
 *
 * @param interval_ms Sampling interval in milliseconds
 * @return 0 on success, negative on failure
 */
static int stream_start_cb(uint32_t interval_ms)
{
	return adc_stream_start(&g_adc_stream, interval_ms);
}

/**
 * @brief Stop ADC streaming callback for protocol module
 *
 * @return 0 on success, negative on failure
 */
static int stream_stop_cb(void)
{
	return adc_stream_stop(&g_adc_stream);
}

/* =========================================================================
 * P-CONTROLLER WRAPPER FUNCTIONS
 * ========================================================================= */

/**
 * @brief Wrapper for PWM set (matches PI-controller callback signature)
 *
 * @param duty PWM duty cycle (0-100)
 */
static void led_pwm_set_duty_wrapper(uint8_t duty)
{
	led_pwm_set_duty(&g_led_pwm, duty);
}

/**
 * @brief Wrapper for sending P-stream data via protocol
 *
 * @param setpoint Target ADC value
 * @param measured Measured ADC value
 * @param pwm PWM output
 */
static void proto_send_p_stream_data(uint16_t setpoint, uint16_t measured, uint8_t pwm)
{
	uint8_t frame[7];
	proto_build_p_stream_frame(frame, setpoint, measured, pwm);
	tx_buffer_put(&g_tx_buf, frame, sizeof(frame));
}

/**
 * @brief Set PI-controller mode callback
 *
 * @param mode 0=MANUAL, 1=AUTO
 */
static void p_ctrl_set_mode_cb(uint8_t mode)
{
	p_ctrl_set_mode(&g_p_ctrl, mode);
}

/**
 * @brief Set PI-controller setpoint callback
 *
 * @param setpoint Target ADC value (0-4095)
 */
static void p_ctrl_set_setpoint_cb(uint16_t setpoint)
{
	p_ctrl_set_setpoint(&g_p_ctrl, setpoint);
}

/**
 * @brief Set PI-controller gain callback
 *
 * @param gain Proportional gain value (0-1000, represents 0.0-10.0)
 */
static void p_ctrl_set_gain_cb(uint16_t gain)
{
	p_ctrl_set_gain(&g_p_ctrl, gain);
}

/**
 * @brief Set PI-controller Ki (integral gain) callback
 *
 * @param ki Integral gain value (0-1000, represents 0.0-10.0)
 */
static void p_ctrl_set_ki_cb(uint16_t ki)
{
	p_ctrl_set_ki(&g_p_ctrl, ki);
}

/**
 * @brief Set PI-controller feed-forward callback
 *
 * @param ff Feed-forward PWM (0-100)
 */
static void p_ctrl_set_feed_forward_cb(uint8_t ff)
{
	p_ctrl_set_feed_forward(&g_p_ctrl, ff);
}

/**
 * @brief Start PI-controller streaming callback
 *
 * @param rate_hz Streaming rate in Hz (max 1000)
 * @return 0 on success, negative on failure
 */
static int p_ctrl_start_stream_cb(uint32_t rate_hz)
{
	return p_ctrl_start_stream(&g_p_ctrl, rate_hz);
}

/**
 * @brief Stop PI-controller streaming callback
 *
 * @return 0 on success, negative on failure
 */
static int p_ctrl_stop_stream_cb(void)
{
	return p_ctrl_stop_stream(&g_p_ctrl);
}

/**
 * @brief Get PI-controller status callback
 *
 * @return 0 on success (sends response via proto_send_p_stream_data)
 */
static int p_ctrl_get_status_cb(void)
{
	uint8_t frame[7];
	uint16_t setpoint = (uint16_t)atomic_get(&g_p_ctrl.setpoint);
	uint16_t measured = g_p_ctrl.last_measured;
	uint8_t pwm = g_p_ctrl.last_pwm;
	proto_build_p_stream_frame(frame, setpoint, measured, pwm);
	tx_buffer_put(&g_tx_buf, frame, sizeof(frame));
	return 0;
}

/**
 * @brief Set IIR filter alpha callback
 *
 * @param num Alpha numerator (1-255)
 * @param den Alpha denominator (1-255)
 */
static void filter_alpha_set_cb(uint8_t num, uint8_t den)
{
	adc_reader_set_alpha(&g_adc_reader, num, den);
}

/**
 * @brief Set ADC filter mode callback for protocol module
 *
 * @details Called when SET_FILTER_MODE command is received.
 *          Mode: 0=IIR only, 1=Oversample only, 2=Oversample+IIR
 *
 * @param mode Filter mode (0-2)
 */
static void filter_mode_set_cb(uint8_t mode)
{
	int ret = adc_reader_set_filter_mode(&g_adc_reader, mode);
	if (ret != 0) {
		printk("Failed to set filter mode: %d\n", ret);
	}
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
			ringbuf_put(&g_rx_buf, buf[i]);
		}
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

	/* Initialize ADC reader (shared ADC source at 20kHz = 50us interval) */
	if (adc_reader_init(&g_adc_reader, adc_dev, ADC_READER_DEFAULT_INTERVAL_US) != 0) {
		printk("ERROR: Failed to initialize ADC reader\n");
		return 0;
	}

	/* Start ADC reader */
	if (adc_reader_start(&g_adc_reader) != 0) {
		printk("ERROR: Failed to start ADC reader\n");
		return 0;
	}

	/* Verify ADC reader is actually running */
	if (!adc_reader_is_active(&g_adc_reader)) {
		printk("ERROR: ADC reader failed to start (timer not running)\n");
		return 0;
	}
	printk("ADC reader started (20kHz sampling with moving average filter)\n");

	/* Initialize ADC stream module */
	if (adc_stream_init(&g_adc_stream, &g_tx_buf, &g_led_pwm) != 0) {
		printk("ERROR: Failed to initialize ADC stream module\n");
		return 0;
	}

	/* Set ADC reader for streaming module */
	adc_stream_set_adc_reader(&g_adc_stream, &g_adc_reader);

	/* Initialize PI-controller */
	if (p_ctrl_init(&g_p_ctrl) != 0) {
		printk("ERROR: Failed to initialize PI-controller\n");
		return 0;
	}
	printk("PI-controller initialized (MANUAL mode)\n");

	/* Register PI-controller callbacks (PWM and stream data only) */
	p_ctrl_set_callbacks(&g_p_ctrl,
			     led_pwm_set_duty_wrapper,
			     proto_send_p_stream_data);

	/* Set ADC reader for PI-controller */
	p_ctrl_set_adc_reader(&g_p_ctrl, &g_adc_reader);

	/* Initialize OLED display (non-fatal) */
	if (oled_display_init(&g_oled_display) == 0) {
		printk("OLED display initialized\n");
		/* Set module references */
		oled_display_set_p_controller(&g_oled_display, &g_p_ctrl);
		oled_display_set_adc_reader(&g_oled_display, &g_adc_reader);

		/* Start display updates */
		if (oled_display_start(&g_oled_display) == 0) {
			printk("OLED display updates started (4 Hz)\n");
		} else {
			printk("WARNING: Failed to start OLED display updates\n");
		}
	} else {
		printk("WARNING: Failed to initialize OLED display - continuing without display\n");
	}

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

	/* Start RX thread */
	if (rx_thread_start(&g_rx_thread, &g_rx_buf, &g_proto) != 0) {
		printk("ERROR: Failed to start RX thread\n");
		return 0;
	}

	/* Initialize protocol context */
	proto_init(&g_proto);

	/* Register protocol callbacks */
	g_proto.on_led_set = led_set_cb;
	g_proto.on_pwm_set = pwm_set_cb;
	g_proto.on_adc_read = adc_read_cb;
	g_proto.on_stream_start = stream_start_cb;
	g_proto.on_stream_stop = stream_stop_cb;
	/* PI-controller callbacks */
	g_proto.on_set_mode = p_ctrl_set_mode_cb;
	g_proto.on_set_p_setpoint = p_ctrl_set_setpoint_cb;
	g_proto.on_set_p_gain = p_ctrl_set_gain_cb;
	g_proto.on_set_p_ki = p_ctrl_set_ki_cb;
	g_proto.on_set_feed_forward = p_ctrl_set_feed_forward_cb;
	g_proto.on_start_p_stream = p_ctrl_start_stream_cb;
	g_proto.on_stop_p_stream = p_ctrl_stop_stream_cb;
	g_proto.on_get_p_status = p_ctrl_get_status_cb;
	g_proto.on_filter_alpha_set = filter_alpha_set_cb;
	g_proto.on_filter_mode_set = filter_mode_set_cb;
	/* Response callbacks */
	g_proto.send_resp = send_response_cb;
	g_proto.send_adc_resp = send_adc_response_cb;

	/* Wait for DTR */
	printk("Waiting for USB connection...\n");
	while (!dtr) {
		uart_line_ctrl_get(uart_dev, UART_LINE_CTRL_DTR, &dtr);
		k_sleep(K_MSEC(100));
	}

	printk("===================================\n");
	printk("RP2040 PWM LED + ADC + P-Controller\n");
	printk("LED: GPIO25 (built-in)\n");
	printk("PWM: 25kHz frequency\n");
	printk("ADC: GPIO26 (12-bit, 0-3.3V)\n");
	printk("P-Controller: 1kHz control loop\n");
	printk("Protocol: [CMD][VALUE][CRC8]\n");
	printk("  0x01 <val> <crc>  -> Set LED (0=OFF, 1+=ON)\n");
	printk("  0x02 <0-100> <crc> -> Set PWM duty (0=OFF, 100=full)\n");
	printk("  0x03 <crc>        -> Read ADC (returns [0x03][ADC_H][ADC_L][CRC])\n");
	printk("  0x06 <mode> <crc> -> Set mode (0=MANUAL, 1=AUTO)\n");
	printk("  0x07 <H><L><crc>  -> Set setpoint (0-4095 ADC = 0-1.17A)\n");
	printk("  0x08 <H><L><crc>  -> Set gain (0-1000, Kp=0.0-10.0)\n");
	printk("  0x0C <ff> <crc>   -> Set feed-forward (0-100)\n");
	printk("  0x09 <H><L><crc>  -> Start P-stream (rate Hz)\n");
	printk("  0x0A <num><den><crc> -> Set filter alpha (num/den)\n");
	printk("  Response: ACK=0xFF, NACK=0xFE\n");
	printk("===================================\n\n");

	/* Setup UART RX interrupt */
	uart_irq_callback_set(uart_dev, uart_rx_handler);
	uart_irq_rx_enable(uart_dev);

	/* Main loop - idle (all work done in threads) */
	while (true) {
		k_sleep(K_MSEC(1000));
	}

	return 0;
}
