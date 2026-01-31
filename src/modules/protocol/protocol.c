/**
 * @file protocol.c
 * @brief Binary protocol implementation for UART communication
 */

#include "protocol.h"

/* =========================================================================
 * CRC-8 CALCULATION
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
uint8_t proto_crc8(const uint8_t *data, size_t len)
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

/* =========================================================================
 * PROTOCOL CONTEXT INITIALIZATION
 * ========================================================================= */

/**
 * @brief Initialize protocol context
 *
 * @details Resets state machine to WAIT_CMD and clears frame buffer.
 *          Callbacks must be set after initialization.
 *
 * @param ctx Protocol context to initialize
 */
void proto_init(struct proto_ctx *ctx)
{
	if (!ctx) {
		return;
	}

	ctx->state = PROTO_STATE_WAIT_CMD;
	ctx->frame_buf[0] = 0;
	ctx->frame_buf[1] = 0;

	/* Callbacks must be set explicitly by caller */
	ctx->on_led_set = NULL;
	ctx->on_pwm_set = NULL;
	ctx->on_adc_read = NULL;
	ctx->send_resp = NULL;
	ctx->send_adc_resp = NULL;
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
 *          Calls registered callbacks on valid commands.
 *
 * @param ctx Protocol context
 * @param byte Received byte from UART
 */
void proto_process_byte(struct proto_ctx *ctx, uint8_t byte)
{
	if (!ctx) {
		return;
	}

	switch (ctx->state) {
	case PROTO_STATE_WAIT_CMD:
		if (byte == PROTO_CMD_SET_LED || byte == PROTO_CMD_SET_PWM) {
			ctx->frame_buf[0] = byte;
			ctx->state = PROTO_STATE_WAIT_VALUE;
		}
		else if (byte == PROTO_CMD_READ_ADC) {
			ctx->frame_buf[0] = byte;
			ctx->state = PROTO_STATE_WAIT_CRC_ADC;
		}
		/* Unknown command - ignore */
		break;

	case PROTO_STATE_WAIT_VALUE:
		ctx->frame_buf[1] = byte;
		ctx->state = PROTO_STATE_WAIT_CRC;
		break;

	case PROTO_STATE_WAIT_CRC:
	{
		/* Verify CRC for 3-byte frame [CMD][VALUE][CRC] */
		uint8_t calculated_crc = proto_crc8(ctx->frame_buf, 2);
		if (byte != calculated_crc) {
			if (ctx->send_resp) {
				ctx->send_resp(PROTO_RESP_NACK);
				ctx->send_resp(PROTO_ERR_CRC);
			}
			ctx->state = PROTO_STATE_WAIT_CMD;
			break;
		}

		/* Execute command */
		if (ctx->frame_buf[0] == PROTO_CMD_SET_LED) {
			if (ctx->on_led_set) {
				ctx->on_led_set(ctx->frame_buf[1]);
			}
			if (ctx->send_resp) {
				ctx->send_resp(PROTO_RESP_ACK);
			}
		}
		else if (ctx->frame_buf[0] == PROTO_CMD_SET_PWM) {
			int ret = 0;
			if (ctx->on_pwm_set) {
				ret = ctx->on_pwm_set(ctx->frame_buf[1]);
			}
			if (ctx->send_resp) {
				if (ret == 0) {
					ctx->send_resp(PROTO_RESP_ACK);
				} else {
					ctx->send_resp(PROTO_RESP_NACK);
					ctx->send_resp(PROTO_ERR_INVALID_VAL);
				}
			}
		}

		ctx->state = PROTO_STATE_WAIT_CMD;
		break;
	}

	case PROTO_STATE_WAIT_CRC_ADC:
	{
		/* Verify CRC for 2-byte frame [CMD][CRC] */
		uint8_t calculated_crc = proto_crc8(&ctx->frame_buf[0], 1);
		if (byte != calculated_crc) {
			if (ctx->send_resp) {
				ctx->send_resp(PROTO_RESP_NACK);
				ctx->send_resp(PROTO_ERR_CRC);
			}
			ctx->state = PROTO_STATE_WAIT_CMD;
			break;
		}

		/* Execute ADC read command */
		int adc_value = 0;
		if (ctx->on_adc_read) {
			adc_value = ctx->on_adc_read();
		}

		if (adc_value >= 0) {
			if (ctx->send_adc_resp) {
				uint8_t adc_h = (adc_value >> 8) & 0xFF;
				uint8_t adc_l = adc_value & 0xFF;
				uint8_t crc = proto_crc8(ctx->frame_buf, 1);
				ctx->send_adc_resp(PROTO_CMD_READ_ADC, adc_h, adc_l, crc);
			}
		} else {
			if (ctx->send_resp) {
				ctx->send_resp(PROTO_RESP_NACK);
				ctx->send_resp(PROTO_ERR_INVALID_CMD);  /* ADC error */
			}
		}

		ctx->state = PROTO_STATE_WAIT_CMD;
		break;
	}

	default:
		ctx->state = PROTO_STATE_WAIT_CMD;
		break;
	}
}
