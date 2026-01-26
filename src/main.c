/**
 * @file main.c
 * @brief RP2040 PWM LED Control via UART ACM Binary Protocol
 *
 * @details This firmware implements a PWM-based LED control system for RP2040.
 *          Future versions will extend this to softstart current control.
 *
 * **Hardware Platform:**
 *   - MCU: RP2040 (Raspberry Pi Pico)
 *   - LED: GPIO25 (built-in LED)
 *   - Communication: USB CDC ACM (virtual serial port)
 *
 * **Binary Protocol Specification:**
 *   - Frame Format: [CMD][VALUE][CRC8] (3 bytes)
 *   - CMD 0x01: SET LED (0=OFF, 1+=ON)
 *   - Response: ACK 0xFF or NACK 0xFE + error code
 *   - CRC-8: Polynomial 0x07, Initial 0x00
 *
 * **Project Roadmap:**
 *   1. LED ON/OFF control (current implementation)
 *   2. PWM duty cycle control
 *   3. Softstart with current regulation (1% accuracy)
 *
 * @author Project Team
 * @version 0.1.0
 * @date 2026-01-26
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/atomic.h>

/* =========================================================================
 * CONSTANTS AND DEFINITIONS
 * ========================================================================= */

/** @brief UART device node from device tree */
#define UART_NODE DT_NODELABEL(cdc_acm_uart0)

/** @brief LED device node from device tree */
#define LED_NODE DT_NODELABEL(led0)

/* -------------------------------------------------------------------------
 * Protocol Constants
 * ------------------------------------------------------------------------- */
/** @brief Command byte for LED control */
#define CMD_SET_LED      0x01

/** @brief Positive response (ACK) */
#define RESP_ACK         0xFF

/** @brief Negative response (NACK) */
#define RESP_NACK        0xFE

/** @brief Error code: CRC mismatch */
#define ERR_CRC          0x01

/** @brief Error code: Invalid command */
#define ERR_INVALID_CMD  0x02

/** @brief Error code: Invalid value */
#define ERR_INVALID_VAL  0x03

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

/** @brief Cached UART device pointer */
static const struct device *uart_dev;

/** @brief GPIO pin specification for LED from device tree */
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED_NODE, gpios);

/** @brief Current protocol state machine state */
static enum proto_state proto_state = STATE_WAIT_CMD;

/** @brief Frame buffer for storing CMD + VALUE */
static uint8_t frame_buf[2];

/** @brief UART RX ring buffer */
static uint8_t rx_buf[RX_BUF_SIZE];

/** @brief Ring buffer head pointer (write position) */
static volatile size_t rx_head = 0;

/** @brief Ring buffer tail pointer (read position) */
static volatile size_t rx_tail = 0;

/**
 * @brief Calculate CRC-8 checksum
 *
 * @details Uses polynomial 0x07 with initial value 0x00.
 *          This is used to verify integrity of protocol frames.
 *
 * @param data Input data buffer
 * @param len Length of data in bytes
 * @return CRC-8 checksum (0-255)
 */
static uint8_t crc8(const uint8_t *data, size_t len)
{
	uint8_t crc = 0x00;

	for (size_t i = 0; i < len; i++) {
		crc ^= data[i];
		for (uint8_t bit = 0; bit < 8; bit++) {
			if (crc & 0x80) {
				crc = (crc << 1) ^ 0x07;
			} else {
				crc <<= 1;
			}
		}
	}
	return crc;
}

/**
 * @brief Send response byte via UART
 *
 * @param resp Response byte (RESP_ACK or RESP_NACK)
 */
static void send_response(uint8_t resp)
{
	uart_fifo_fill(uart_dev, &resp, 1);
}

/**
 * @brief Set LED state
 *
 * @details Turns LED ON (state > 0) or OFF (state == 0).
 *          Uses atomic operations for thread safety.
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
 * @brief Protocol state machine states
 */
enum proto_state {
	STATE_WAIT_CMD,    /**< Waiting for command byte */
	STATE_WAIT_VALUE,  /**< Waiting for value byte */
	STATE_WAIT_CRC,    /**< Waiting for CRC byte */
};

static enum proto_state proto_state = STATE_WAIT_CMD;
static uint8_t frame_buf[2];  /* CMD + VALUE */

/**
 * @brief Process received byte through protocol state machine
 *
 * @details Implements 3-state protocol parser:
 *          WAIT_CMD -> WAIT_VALUE -> WAIT_CRC -> execute command
 *
 * @param byte Received byte from UART
 */
static void process_byte(uint8_t byte)
{
	switch (proto_state) {
	case STATE_WAIT_CMD:
		if (byte == CMD_SET_LED) {
			frame_buf[0] = byte;
			proto_state = STATE_WAIT_VALUE;
		}
		/* Unknown command - ignore */
		break;

	case STATE_WAIT_VALUE:
		frame_buf[1] = byte;
		proto_state = STATE_WAIT_CRC;
		break;

	case STATE_WAIT_CRC:
	{
		/* Verify CRC */
		uint8_t calculated_crc = crc8(frame_buf, 2);
		if (byte != calculated_crc) {
			send_response(RESP_NACK);
			send_response(ERR_CRC);
			proto_state = STATE_WAIT_CMD;
			break;
		}

		/* Execute command */
		if (frame_buf[0] == CMD_SET_LED) {
			set_led(frame_buf[1]);
			send_response(RESP_ACK);
		}

		proto_state = STATE_WAIT_CMD;
		break;
	}

	default:
		proto_state = STATE_WAIT_CMD;
		break;
	}
}

/* Simple ring buffer for UART RX */
#define RX_BUF_SIZE  32
static uint8_t rx_buf[RX_BUF_SIZE];
static volatile size_t rx_head = 0;
static volatile size_t rx_tail = 0;

/* =========================================================================
 * RING BUFFER FUNCTIONS (ISR-safe)
 * ========================================================================= */

/**
 * @brief Put byte into ring buffer (ISR-safe)
 *
 * @param byte Byte to add
 * @return true on success, false if buffer full
 */
static bool ring_buf_put_byte(uint8_t byte)
{
	size_t next_head = (rx_head + 1) % RX_BUF_SIZE;

	if (next_head == rx_tail) {
		return false;  /* Buffer full */
	}

	rx_buf[rx_head] = byte;
	rx_head = next_head;
	return true;
}

/**
 * @brief Get byte from ring buffer
 *
 * @param byte Pointer to store retrieved byte
 * @return true on success, false if buffer empty
 */
static bool ring_buf_get_byte(uint8_t *byte)
{
	if (rx_head == rx_tail) {
		return false;  /* Buffer empty */
	}

	*byte = rx_buf[rx_tail];
	rx_tail = (rx_tail + 1) % RX_BUF_SIZE;
	return true;
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

	while (ring_buf_get_byte(&byte)) {
		process_byte(byte);
	}
}

/**
 * @brief Main application entry point
 *
 * @details Initializes UART, GPIO, and interrupt handlers.
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

	/* Verify UART is ready */
	if (!device_is_ready(uart_dev)) {
		printk("ERROR: UART device not ready\n");
		return 0;
	}

	/* Configure LED GPIO */
	if (!gpio_is_ready_dt(&led)) {
		printk("ERROR: LED device not ready\n");
		return 0;
	}

	gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);

	/* Wait for DTR */
	printk("Waiting for USB connection...\n");
	while (!dtr) {
		uart_line_ctrl_get(uart_dev, UART_LINE_CTRL_DTR, &dtr);
		k_sleep(K_MSEC(100));
	}

	printk("===================================\n");
	printk("RP2040 LED Control\n");
	printk("LED: GPIO25 (built-in)\n");
	printk("Protocol: [CMD][VALUE][CRC8]\n");
	printk("  0x01 <val> <crc>  -> Set LED (0=OFF, 1+=ON)\n");
	printk("  Response: 0xFF=OK, 0xFE=ERR\n");
	printk("===================================\n\n");

	/* Setup UART RX interrupt */
	uart_irq_callback_set(uart_dev, uart_rx_handler);
	uart_irq_rx_enable(uart_dev);

	/* Set initial LED state to OFF */
	set_led(0);
	printk("LED initialized: OFF\n");

	/* Main loop */
	while (true) {
		process_ring_buffer();
		k_sleep(K_MSEC(10));
	}

	return 0;
}
