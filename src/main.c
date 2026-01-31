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
 * Protocol Constants
 * ------------------------------------------------------------------------- */
/** @brief Command byte for LED ON/OFF control (legacy) */
#define CMD_SET_LED      0x01

/** @brief Command byte for PWM duty cycle control */
#define CMD_SET_PWM      0x02

/** @brief Command byte for ADC read request */
#define CMD_READ_ADC     0x03

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

/* -------------------------------------------------------------------------
 * Protocol State Machine
 * ------------------------------------------------------------------------- */
/**
 * @brief Protocol state machine states
 */
enum proto_state {
	STATE_WAIT_CMD,     /**< Waiting for command byte */
	STATE_WAIT_VALUE,   /**< Waiting for value byte */
	STATE_WAIT_CRC,     /**< Waiting for CRC byte (3-byte frame) */
	STATE_WAIT_CRC_ADC, /**< Waiting for CRC byte (2-byte frame for ADC) */
};

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

/** @brief Current protocol state machine state */
static enum proto_state proto_state = STATE_WAIT_CMD;

/** @brief Frame buffer for storing CMD + VALUE */
static uint8_t frame_buf[2];

/* =========================================================================
 * FUNCTION PROTOTYPES
 * ========================================================================= */

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
 * @details Turns LED ON (state > 0) or OFF (state == 0).
 *          Disables PWM when using GPIO control.
 *
 * @param state LED state (0=OFF, 1+=ON)
 */
static void set_led(uint8_t state)
{
	int value = (state > 0) ? 1 : 0;

	/* Disable PWM and switch to GPIO */
	pwm_set_usec(PWM_CHANNEL_EXT, 0, 1000000 / PWM_FREQ);
	gpio_pin_set_dt(&led, value);

	atomic_set(&pwm_duty, 0);
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

/**
 * @brief Send ADC response frame
 *
 * @details Sends 4-byte response: [CMD_READ_ADC][ADC_H][ADC_L][CRC8]
 *
 * @param adc_value 12-bit ADC value (0-4095)
 */
static void send_adc_response(uint16_t adc_value)
{
	uint8_t response[4];

	response[0] = CMD_READ_ADC;
	response[1] = (adc_value >> 8) & 0xFF;  /* High byte */
	response[2] = adc_value & 0xFF;         /* Low byte */
	response[3] = crc8(response, 3);

	uart_fifo_fill(uart_dev, response, sizeof(response));
}

/* =========================================================================
 * PROTOCOL STATE MACHINE
 * ========================================================================= */

/**
 * @brief Process received byte through protocol state machine
 *
 * @details Implements protocol parser with support for:
 *          - 3-byte commands: [CMD][VALUE][CRC8] (SET_LED, SET_PWM)
 *          - 2-byte commands: [CMD][CRC8] (READ_ADC)
 *
 * @param byte Received byte from UART
 */
static void process_byte(uint8_t byte)
{
	switch (proto_state) {
	case STATE_WAIT_CMD:
		if (byte == CMD_SET_LED || byte == CMD_SET_PWM) {
			frame_buf[0] = byte;
			proto_state = STATE_WAIT_VALUE;
		}
		else if (byte == CMD_READ_ADC) {
			frame_buf[0] = byte;
			proto_state = STATE_WAIT_CRC_ADC;  /* Different state for ADC */
		}
		/* Unknown command - ignore */
		break;

	case STATE_WAIT_VALUE:
		frame_buf[1] = byte;
		proto_state = STATE_WAIT_CRC;
		break;

	case STATE_WAIT_CRC:
	{
		/* Verify CRC for 3-byte frame [CMD][VALUE][CRC] */
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
		else if (frame_buf[0] == CMD_SET_PWM) {
			if (set_pwm_duty(frame_buf[1]) == 0) {
				send_response(RESP_ACK);
			} else {
				send_response(RESP_NACK);
				send_response(ERR_INVALID_VAL);
			}
		}

		proto_state = STATE_WAIT_CMD;
		break;
	}

	case STATE_WAIT_CRC_ADC:
	{
		/* Verify CRC for 2-byte frame [CMD][CRC] */
		uint8_t calculated_crc = crc8(&frame_buf[0], 1);
		if (byte != calculated_crc) {
			send_response(RESP_NACK);
			send_response(ERR_CRC);
			proto_state = STATE_WAIT_CMD;
			break;
		}

		/* Execute ADC read command */
		int adc_value = read_adc_channel0();
		if (adc_value >= 0) {
			send_adc_response(adc_value);
		} else {
			send_response(RESP_NACK);
			send_response(ERR_INVALID_CMD);  /* ADC error */
		}

		/* Small delay to ensure ADC response transmission completes */
		k_sleep(K_MSEC(2));

		proto_state = STATE_WAIT_CMD;
		break;
	}

	default:
		proto_state = STATE_WAIT_CMD;
		break;
	}
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
		process_byte(byte);
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
	printk("  Response: 0xFF=OK, 0xFE=ERR\n");
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
