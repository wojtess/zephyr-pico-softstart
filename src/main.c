/**
 * @file main.c
 * @brief RP2040 Built-in LED Control via UART ACM Binary Protocol
 *
 * Hardware: RP2040 (Raspberry Pi Pico)
 * - LED: GPIO25 (built-in LED)
 * - UART: USB CDC ACM (virtual serial)
 *
 * Binary Protocol:
 *   Frame: [CMD][VALUE][CRC8] - 3 bytes
 *   CMD 0x01 = SET LED (0=OFF, 1+=ON)
 *   Response: ACK 0xFF or NACK 0xFE + error code
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/atomic.h>

/* Device Tree Nodes */
#define UART_NODE DT_NODELABEL(cdc_acm_uart0)
#define LED_NODE DT_NODELABEL(led0)

/* Protocol Definitions */
#define CMD_SET_LED      0x01
#define RESP_ACK         0xFF
#define RESP_NACK        0xFE
#define ERR_CRC          0x01
#define ERR_INVALID_CMD  0x02
#define ERR_INVALID_VAL  0x03

/* Global State */
static atomic_val_t led_state = ATOMIC_INIT(0);

/* Cached device pointers */
static const struct device *uart_dev;

/* GPIO pin number from devicetree */
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED_NODE, gpios);

/* CRC-8 (Polynomial 0x07, Initial 0x00) */
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

/* Send response byte via UART */
static void send_response(uint8_t resp)
{
	uart_fifo_fill(uart_dev, &resp, 1);
}

/* Set LED state (0=OFF, 1+=ON) */
static void set_led(uint8_t state)
{
	int value = (state > 0) ? 1 : 0;

	gpio_pin_set_dt(&led, value);
	atomic_set(&led_state, value);
}

/* Protocol state machine states */
enum proto_state {
	STATE_WAIT_CMD,
	STATE_WAIT_VALUE,
	STATE_WAIT_CRC,
};

static enum proto_state proto_state = STATE_WAIT_CMD;
static uint8_t frame_buf[2];  /* CMD + VALUE */

/* Process received byte through state machine */
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

/* Ring buffer helpers (ISR-safe) */
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

static bool ring_buf_get_byte(uint8_t *byte)
{
	if (rx_head == rx_tail) {
		return false;  /* Buffer empty */
	}

	*byte = rx_buf[rx_tail];
	rx_tail = (rx_tail + 1) % RX_BUF_SIZE;
	return true;
}

/* UART RX interrupt handler - ISR safe */
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

/* Process ring buffer (called from main loop) */
static void process_ring_buffer(void)
{
	uint8_t byte;

	while (ring_buf_get_byte(&byte)) {
		process_byte(byte);
	}
}

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
