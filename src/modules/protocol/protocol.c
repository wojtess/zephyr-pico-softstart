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
				crc = (crc << 1) & 0xFF;
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
	ctx->on_stream_start = NULL;
	ctx->on_stream_stop = NULL;
	ctx->on_set_mode = NULL;
	ctx->on_set_p_setpoint = NULL;
	ctx->on_set_p_gain = NULL;
	ctx->on_set_p_ki = NULL;
	ctx->on_set_feed_forward = NULL;
	ctx->on_start_p_stream = NULL;
	ctx->on_stop_p_stream = NULL;
	ctx->on_get_p_status = NULL;
	ctx->on_filter_alpha_set = NULL;
	ctx->send_resp = NULL;
	ctx->send_adc_resp = NULL;
	ctx->send_debug = NULL;
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
		else if (byte == PROTO_CMD_START_STREAM) {
			ctx->frame_buf[0] = byte;
			ctx->state = PROTO_STATE_WAIT_STREAM_INT_H;
		}
		else if (byte == PROTO_CMD_STOP_STREAM) {
			ctx->frame_buf[0] = byte;
			ctx->state = PROTO_STATE_WAIT_STOP_STREAM_CRC;
		}
		/* PI-controller commands */
		else if (byte == PROTO_CMD_SET_MODE) {
			ctx->frame_buf[0] = byte;
			ctx->state = PROTO_STATE_WAIT_VALUE;
		}
		else if (byte == PROTO_CMD_SET_P_SETPOINT) {
			ctx->frame_buf[0] = byte;
			ctx->state = PROTO_STATE_WAIT_P_SETPOINT_H;
		}
		else if (byte == PROTO_CMD_SET_P_GAIN) {
			ctx->frame_buf[0] = byte;
			ctx->state = PROTO_STATE_WAIT_P_GAIN_H;
		}
		else if (byte == PROTO_CMD_SET_P_KI) {
			ctx->frame_buf[0] = byte;
			ctx->state = PROTO_STATE_WAIT_P_KI_H;
		}
		else if (byte == PROTO_CMD_SET_FEED_FORWARD) {
			ctx->frame_buf[0] = byte;
			ctx->state = PROTO_STATE_WAIT_VALUE;
		}
		else if (byte == PROTO_CMD_START_P_STREAM) {
			ctx->frame_buf[0] = byte;
			ctx->state = PROTO_STATE_WAIT_P_STREAM_RATE_H;
		}
		else if (byte == PROTO_CMD_STOP_P_STREAM) {
			ctx->frame_buf[0] = byte;
			ctx->state = PROTO_STATE_WAIT_STOP_P_STREAM_CRC;
		}
		else if (byte == PROTO_CMD_GET_P_STATUS) {
			ctx->frame_buf[0] = byte;
			ctx->state = PROTO_STATE_WAIT_GET_P_STATUS_CRC;
		}
		else if (byte == PROTO_CMD_SET_FILTER_ALPHA) {
			ctx->frame_buf[0] = byte;
			ctx->state = PROTO_STATE_WAIT_FILTER_ALPHA_NUM;
		}
		else {
			/* Unknown command - reset state to WAIT_CMD */
			ctx->state = PROTO_STATE_WAIT_CMD;
		}
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
			/* Validate PWM duty cycle range (0-100) */
			if (ctx->frame_buf[1] > 100) {
				if (ctx->send_resp) {
					ctx->send_resp(PROTO_RESP_NACK);
					ctx->send_resp(PROTO_ERR_INVALID_VAL);
				}
			} else {
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
		}
		else if (ctx->frame_buf[0] == PROTO_CMD_SET_MODE) {
			/* Validate mode (0=MANUAL, 1=AUTO) */
			if (ctx->frame_buf[1] > 1) {
				if (ctx->send_resp) {
					ctx->send_resp(PROTO_RESP_NACK);
					ctx->send_resp(PROTO_ERR_INVALID_MODE);
				}
			} else {
				if (ctx->on_set_mode) {
					ctx->on_set_mode(ctx->frame_buf[1]);
				}
				if (ctx->send_resp) {
					ctx->send_resp(PROTO_RESP_ACK);
				}
			}
		}
		else if (ctx->frame_buf[0] == PROTO_CMD_SET_FEED_FORWARD) {
			/* Validate feed-forward (0-100) */
			if (ctx->frame_buf[1] > 100) {
				if (ctx->send_resp) {
					ctx->send_resp(PROTO_RESP_NACK);
					ctx->send_resp(PROTO_ERR_INVALID_FF);
				}
			} else {
				if (ctx->on_set_feed_forward) {
					ctx->on_set_feed_forward(ctx->frame_buf[1]);
				}
				if (ctx->send_resp) {
					ctx->send_resp(PROTO_RESP_ACK);
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
				/* CRC calculated from [CMD][ADC_H][ADC_L] */
				uint8_t data[3] = {PROTO_CMD_READ_ADC, adc_h, adc_l};
				uint8_t crc = proto_crc8(data, 3);
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

	case PROTO_STATE_WAIT_STREAM_INT_H:
		/* Store interval high byte */
		ctx->frame_buf[1] = byte;
		ctx->state = PROTO_STATE_WAIT_STREAM_INT_L;
		break;

	case PROTO_STATE_WAIT_STREAM_INT_L:
		/* Store interval low byte */
		ctx->frame_buf[2] = byte;
		ctx->state = PROTO_STATE_WAIT_STREAM_CRC;
		break;

	case PROTO_STATE_WAIT_STREAM_CRC:
	{
		/* Verify CRC for 4-byte frame [CMD][INT_H][INT_L][CRC] */
		uint8_t calculated_crc = proto_crc8(ctx->frame_buf, 3);
		if (byte != calculated_crc) {
			if (ctx->send_resp) {
				ctx->send_resp(PROTO_RESP_NACK);
				ctx->send_resp(PROTO_ERR_CRC);
			}
			ctx->state = PROTO_STATE_WAIT_CMD;
			break;
		}

		/* Calculate interval from 2 bytes */
		uint32_t interval = (ctx->frame_buf[1] << 8) | ctx->frame_buf[2];

		/* Execute start stream command */
		int ret = 0;
		if (ctx->on_stream_start) {
			ret = ctx->on_stream_start(interval);
		}

		if (ctx->send_resp) {
			if (ret == 0) {
				ctx->send_resp(PROTO_RESP_ACK);
			} else {
				ctx->send_resp(PROTO_RESP_NACK);
				ctx->send_resp(PROTO_ERR_INVALID_VAL);
			}
		}

		ctx->state = PROTO_STATE_WAIT_CMD;
		break;
	}

	case PROTO_STATE_WAIT_STOP_STREAM_CRC:
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

		/* Execute stop stream command */
		int ret = 0;
		if (ctx->on_stream_stop) {
			ret = ctx->on_stream_stop();
		}

		if (ctx->send_resp) {
			if (ret == 0) {
				ctx->send_resp(PROTO_RESP_ACK);
			} else {
				ctx->send_resp(PROTO_RESP_NACK);
				ctx->send_resp(PROTO_ERR_INVALID_CMD);
			}
		}

		ctx->state = PROTO_STATE_WAIT_CMD;
		break;
	}

	/* PI-controller states */
	case PROTO_STATE_WAIT_P_SETPOINT_H:
		/* Store setpoint high byte */
		ctx->frame_buf[1] = byte;
		ctx->state = PROTO_STATE_WAIT_P_SETPOINT_L;
		break;

	case PROTO_STATE_WAIT_P_SETPOINT_L:
		/* Store setpoint low byte */
		ctx->frame_buf[2] = byte;
		ctx->state = PROTO_STATE_WAIT_P_SETPOINT_CRC;
		break;

	case PROTO_STATE_WAIT_P_SETPOINT_CRC:
	{
		/* Verify CRC for 4-byte frame [CMD][SP_H][SP_L][CRC] */
		uint8_t calculated_crc = proto_crc8(ctx->frame_buf, 3);
		if (byte != calculated_crc) {
			if (ctx->send_resp) {
				ctx->send_resp(PROTO_RESP_NACK);
				ctx->send_resp(PROTO_ERR_CRC);
			}
			ctx->state = PROTO_STATE_WAIT_CMD;
			break;
		}

		/* Calculate setpoint from 2 bytes */
		uint16_t setpoint = (ctx->frame_buf[1] << 8) | ctx->frame_buf[2];

		/* Execute set setpoint command */
		if (ctx->on_set_p_setpoint) {
			ctx->on_set_p_setpoint(setpoint);
		}

		if (ctx->send_resp) {
			ctx->send_resp(PROTO_RESP_ACK);
		}

		ctx->state = PROTO_STATE_WAIT_CMD;
		break;
	}

	case PROTO_STATE_WAIT_P_GAIN_H:
		/* Store gain high byte */
		ctx->frame_buf[1] = byte;
		ctx->state = PROTO_STATE_WAIT_P_GAIN_L;
		break;

	case PROTO_STATE_WAIT_P_GAIN_L:
		/* Store gain low byte */
		ctx->frame_buf[2] = byte;
		ctx->state = PROTO_STATE_WAIT_P_GAIN_CRC;
		break;

	case PROTO_STATE_WAIT_P_KI_H:
		/* Store Ki high byte */
		ctx->frame_buf[1] = byte;
		ctx->state = PROTO_STATE_WAIT_P_KI_L;
		break;

	case PROTO_STATE_WAIT_P_KI_L:
		/* Store Ki low byte */
		ctx->frame_buf[2] = byte;
		ctx->state = PROTO_STATE_WAIT_P_KI_CRC;
		break;

	case PROTO_STATE_WAIT_P_KI_CRC:
	{
		/* Verify CRC for 4-byte frame [CMD][KI_H][KI_L][CRC] */
		uint8_t calculated_crc = proto_crc8(ctx->frame_buf, 3);
		if (byte != calculated_crc) {
			if (ctx->send_resp) {
				ctx->send_resp(PROTO_RESP_NACK);
				ctx->send_resp(PROTO_ERR_CRC);
			}
			ctx->state = PROTO_STATE_WAIT_CMD;
			break;
		}

		/* Calculate Ki from 2 bytes */
		uint16_t ki = (ctx->frame_buf[1] << 8) | ctx->frame_buf[2];

		/* Validate Ki range (0-1000) */
		if (ki > 1000) {
			if (ctx->send_resp) {
				ctx->send_resp(PROTO_RESP_NACK);
				ctx->send_resp(PROTO_ERR_INVALID_KI);
			}
		} else {
			/* Execute set Ki command */
			if (ctx->on_set_p_ki) {
				ctx->on_set_p_ki(ki);
			}

			if (ctx->send_resp) {
				ctx->send_resp(PROTO_RESP_ACK);
			}
		}

		ctx->state = PROTO_STATE_WAIT_CMD;
		break;
	}
		/* Store gain low byte */
		ctx->frame_buf[2] = byte;
		ctx->state = PROTO_STATE_WAIT_P_GAIN_CRC;
		break;

	case PROTO_STATE_WAIT_P_GAIN_CRC:
	{
		/* Verify CRC for 4-byte frame [CMD][GAIN_H][GAIN_L][CRC] */
		uint8_t calculated_crc = proto_crc8(ctx->frame_buf, 3);
		if (byte != calculated_crc) {
			if (ctx->send_resp) {
				ctx->send_resp(PROTO_RESP_NACK);
				ctx->send_resp(PROTO_ERR_CRC);
			}
			ctx->state = PROTO_STATE_WAIT_CMD;
			break;
		}

		/* Calculate gain from 2 bytes */
		uint16_t gain = (ctx->frame_buf[1] << 8) | ctx->frame_buf[2];

		/* Validate gain range (0-1000) */
		if (gain > 1000) {
			if (ctx->send_resp) {
				ctx->send_resp(PROTO_RESP_NACK);
				ctx->send_resp(PROTO_ERR_INVALID_GAIN);
			}
		} else {
			/* Execute set gain command */
			if (ctx->on_set_p_gain) {
				ctx->on_set_p_gain(gain);
			}

			if (ctx->send_resp) {
				ctx->send_resp(PROTO_RESP_ACK);
			}
		}

		ctx->state = PROTO_STATE_WAIT_CMD;
		break;
	}

	case PROTO_STATE_WAIT_P_STREAM_RATE_H:
		/* Store stream rate high byte */
		ctx->frame_buf[1] = byte;
		ctx->state = PROTO_STATE_WAIT_P_STREAM_RATE_L;
		break;

	case PROTO_STATE_WAIT_P_STREAM_RATE_L:
		/* Store stream rate low byte */
		ctx->frame_buf[2] = byte;
		ctx->state = PROTO_STATE_WAIT_P_STREAM_CRC;
		break;

	case PROTO_STATE_WAIT_P_STREAM_CRC:
	{
		/* Verify CRC for 4-byte frame [CMD][RATE_H][RATE_L][CRC] */
		uint8_t calculated_crc = proto_crc8(ctx->frame_buf, 3);
		if (byte != calculated_crc) {
			if (ctx->send_resp) {
				ctx->send_resp(PROTO_RESP_NACK);
				ctx->send_resp(PROTO_ERR_CRC);
			}
			ctx->state = PROTO_STATE_WAIT_CMD;
			break;
		}

		/* Calculate rate from 2 bytes */
		uint32_t rate = (ctx->frame_buf[1] << 8) | ctx->frame_buf[2];

		/* Execute start P-stream command */
		int ret = 0;
		if (ctx->on_start_p_stream) {
			ret = ctx->on_start_p_stream(rate);
		}

		if (ctx->send_resp) {
			if (ret == 0) {
				ctx->send_resp(PROTO_RESP_ACK);
			} else {
				ctx->send_resp(PROTO_RESP_NACK);
				ctx->send_resp(PROTO_ERR_INVALID_VAL);
			}
		}

		ctx->state = PROTO_STATE_WAIT_CMD;
		break;
	}

	case PROTO_STATE_WAIT_STOP_P_STREAM_CRC:
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

		/* Execute stop P-stream command */
		int ret = 0;
		if (ctx->on_stop_p_stream) {
			ret = ctx->on_stop_p_stream();
		}

		if (ctx->send_resp) {
			if (ret == 0) {
				ctx->send_resp(PROTO_RESP_ACK);
			} else {
				ctx->send_resp(PROTO_RESP_NACK);
				ctx->send_resp(PROTO_ERR_INVALID_CMD);
			}
		}

		ctx->state = PROTO_STATE_WAIT_CMD;
		break;
	}

	case PROTO_STATE_WAIT_GET_P_STATUS_CRC:
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

		/* Execute get P status command */
		int ret = 0;
		if (ctx->on_get_p_status) {
			ret = ctx->on_get_p_status();
		}

		if (ctx->send_resp) {
			if (ret == 0) {
				/* Status response is sent via callback */
				ctx->send_resp(PROTO_RESP_ACK);
			} else {
				ctx->send_resp(PROTO_RESP_NACK);
				ctx->send_resp(PROTO_ERR_INVALID_CMD);
			}
		}

		ctx->state = PROTO_STATE_WAIT_CMD;
		break;
	}

	case PROTO_STATE_WAIT_FILTER_ALPHA_NUM:
		/* Store alpha numerator */
		ctx->frame_buf[1] = byte;
		ctx->state = PROTO_STATE_WAIT_FILTER_ALPHA_DEN;
		break;

	case PROTO_STATE_WAIT_FILTER_ALPHA_DEN:
		/* Store alpha denominator */
		ctx->frame_buf[2] = byte;
		ctx->state = PROTO_STATE_WAIT_FILTER_ALPHA_CRC;
		break;

	case PROTO_STATE_WAIT_FILTER_ALPHA_CRC:
	{
		/* Verify CRC for 4-byte frame [CMD][NUM][DEN][CRC] */
		uint8_t calculated_crc = proto_crc8(ctx->frame_buf, 3);
		if (byte != calculated_crc) {
			if (ctx->send_resp) {
				ctx->send_resp(PROTO_RESP_NACK);
				ctx->send_resp(PROTO_ERR_CRC);
			}
			ctx->state = PROTO_STATE_WAIT_CMD;
			break;
		}

		/* Validate alpha numerator and denominator (1-255) */
		uint8_t alpha_num = ctx->frame_buf[1];
		uint8_t alpha_den = ctx->frame_buf[2];

		if (alpha_num == 0 || alpha_den == 0) {
			if (ctx->send_resp) {
				ctx->send_resp(PROTO_RESP_NACK);
				ctx->send_resp(PROTO_ERR_INVALID_VAL);
			}
			ctx->state = PROTO_STATE_WAIT_CMD;
			break;
		}

		/* Execute set filter alpha command */
		if (ctx->on_filter_alpha_set) {
			ctx->on_filter_alpha_set(alpha_num, alpha_den);
		}

		if (ctx->send_resp) {
			ctx->send_resp(PROTO_RESP_ACK);
		}

		ctx->state = PROTO_STATE_WAIT_CMD;
		break;
	}

	default:
		ctx->state = PROTO_STATE_WAIT_CMD;
		break;
	}
}

/* =========================================================================
 * DEBUG PACKET HELPERS
 * ========================================================================= */

/**
 * @brief Send debug packet via protocol
 *
 * @details Sends 2-byte debug packet: [DEBUG_MARKER][CODE]
 *
 * @param ctx Protocol context
 * @param debug_code Debug code (1-255)
 */
void proto_send_debug(struct proto_ctx *ctx, uint8_t debug_code)
{
	if (!ctx || !ctx->send_debug) {
		return;
	}
	ctx->send_debug(debug_code);
}

/* =========================================================================
 * P-STREAM FRAME BUILDER
 * ========================================================================= */

/**
 * @brief Build P-stream frame
 *
 * @details Builds 7-byte P-stream frame: [RESP_P_STREAM][SET_H][SET_L][MEAS_H][MEAS_L][PWM][CRC8]
 *
 * @param frame Output buffer (must be at least 7 bytes)
 * @param setpoint Setpoint value (0-4095)
 * @param measured Measured ADC value (0-4095)
 * @param pwm PWM output (0-100)
 */
void proto_build_p_stream_frame(uint8_t *frame, uint16_t setpoint, uint16_t measured, uint8_t pwm)
{
	if (!frame) {
		return;
	}

	frame[0] = PROTO_RESP_P_STREAM;
	frame[1] = (setpoint >> 8) & 0xFF;
	frame[2] = setpoint & 0xFF;
	frame[3] = (measured >> 8) & 0xFF;
	frame[4] = measured & 0xFF;
	frame[5] = pwm;

	/* Calculate CRC for first 6 bytes */
	uint8_t crc = proto_crc8(frame, 6);
	frame[6] = crc;
}
