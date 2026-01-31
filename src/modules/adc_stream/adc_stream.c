/**
 * @file adc_stream.c
 * @brief ADC streaming implementation
 */

#include "adc_stream.h"
#include "../drivers/led_pwm.h"
#include "../drivers/adc_ctrl.h"
#include "../tx_buffer/tx_buffer.h"
#include "../protocol/protocol.h"

/* Debug flag - ustaw na 1 aby włączyć pakiety debug */
#define ADC_STREAM_DEBUG_ENABLED 1

/* Debug codes */
#define DEBUG_TIMER_START  1
#define DEBUG_TIMER_CB     2
#define DEBUG_ADC_READ     3
#define DEBUG_ADC_ERROR    4
#define DEBUG_TX_PUT       5

/* =========================================================================
 * WORK QUEUE (dla ADC read z thread context)
 * ========================================================================= */

/**
 * @brief Send debug packet to TX buffer
 */
static void send_debug(struct adc_stream_ctx *ctx, uint8_t code)
{
	uint8_t debug_pkt[2] = {0xFD, code};  /* DEBUG marker + code */
	tx_buffer_put_isr(ctx->tx_buf, debug_pkt, sizeof(debug_pkt));
}

/**
 * @brief Work item dla ADC read (tylko work może wywołać adc_read)
 */
static void adc_stream_work_handler(struct k_work *work)
{
	struct adc_stream_ctx *ctx =
		CONTAINER_OF(work, struct adc_stream_ctx, work);

	int adc_value;
	uint8_t response[4];
	uint8_t crc;

	/* Debug: work handler entered */
	send_debug(ctx, DEBUG_ADC_READ);

	/* Read ADC (teraz w thread context, nie ISR) */
	adc_value = adc_ctrl_read_channel0(ctx->adc);
	if (adc_value < 0) {
		/* ADC error */
		send_debug(ctx, DEBUG_ADC_ERROR);
		return;
	}

	ctx->last_value = adc_value;

	/* Build ADC response frame: [CMD][ADC_H][ADC_L][CRC] */
	response[0] = ADC_STREAM_CMD_START;  /* Use START as stream data marker */
	response[1] = (adc_value >> 8) & 0xFF;  /* High byte */
	response[2] = adc_value & 0xFF;         /* Low byte */

	/* Calculate CRC from [CMD][ADC_H][ADC_L] */
	crc = proto_crc8(response, 3);
	response[3] = crc;

	/* Send to TX buffer (ISR-safe version) */
	tx_buffer_put_isr(ctx->tx_buf, response, sizeof(response));
	send_debug(ctx, DEBUG_TX_PUT);
}

/* =========================================================================
 * TIMER CALLBACK
 * ========================================================================= */

/**
 * @brief Timer expiration callback
 *
 * @details Called periodically, submits work queue item for ADC read.
 *          ADC read must happen in thread context, not ISR.
 *
 * @param timer Timer structure
 */
static void adc_stream_timer_callback(struct k_timer *timer)
{
	struct adc_stream_ctx *ctx =
		CONTAINER_OF(timer, struct adc_stream_ctx, timer);

	/* Debug: timer callback entered */
	send_debug(ctx, DEBUG_TIMER_CB);

	/* Increment debug counter */
	ctx->debug_count++;

	/* Submit work to system work queue (ADC read musi być w thread context) */
	k_work_submit(&ctx->work);
}

/* =========================================================================
 * PUBLIC API
 * ========================================================================= */

/**
 * @brief Initialize ADC stream module
 *
 * @details Initializes timer and sets up context.
 *
 * @param ctx ADC stream context to initialize
 * @param adc ADC driver context
 * @param tx_buf TX buffer for sending ADC values
 * @param led LED/PWM driver context (for debug signaling)
 * @return 0 on success, negative errno on failure
 */
int adc_stream_init(struct adc_stream_ctx *ctx,
		    struct adc_ctrl_ctx *adc,
		    struct tx_buffer *tx_buf,
		    struct led_pwm_ctx *led)
{
	if (!ctx || !adc || !tx_buf) {
		return -EINVAL;
	}

	ctx->adc = adc;
	ctx->tx_buf = tx_buf;
	ctx->led = led;
	ctx->active = false;
	ctx->interval_ms = ADC_STREAM_DEFAULT_INTERVAL_MS;
	ctx->last_value = 0;
	ctx->debug_count = 0;

	/* Initialize timer */
	k_timer_init(&ctx->timer, adc_stream_timer_callback, NULL);

	/* Initialize work item */
	k_work_init(&ctx->work, adc_stream_work_handler);

	return 0;
}

/**
 * @brief Start ADC streaming
 *
 * @details Starts periodic ADC sampling at specified interval.
 *
 * @param ctx ADC stream context
 * @param interval_ms Sampling interval in milliseconds (10-60000)
 * @return 0 on success, negative errno on failure
 */
int adc_stream_start(struct adc_stream_ctx *ctx, uint32_t interval_ms)
{
	if (!ctx) {
		return -EINVAL;
	}

	if (interval_ms < ADC_STREAM_MIN_INTERVAL_MS ||
	    interval_ms > ADC_STREAM_MAX_INTERVAL_MS) {
		return -EINVAL;
	}

	if (ctx->active) {
		/* Already streaming - stop first */
		adc_stream_stop(ctx);
	}

	ctx->interval_ms = interval_ms;
	ctx->active = true;
	ctx->debug_count = 0;

	/* Start periodic timer */
	k_timer_start(&ctx->timer, K_MSEC(interval_ms), K_MSEC(interval_ms));

	/* Debug: timer started (wysyłamy PO timer start, żeby ACK przyszło pierwsze) */
	send_debug(ctx, DEBUG_TIMER_START);

	return 0;
}

/**
 * @brief Stop ADC streaming
 *
 * @details Stops periodic ADC sampling.
 *
 * @param ctx ADC stream context
 * @return 0 on success, negative errno on failure
 */
int adc_stream_stop(struct adc_stream_ctx *ctx)
{
	if (!ctx) {
		return -EINVAL;
	}

	if (!ctx->active) {
		return 0;  /* Already stopped */
	}

	k_timer_stop(&ctx->timer);
	ctx->active = false;

	return 0;
}

/**
 * @brief Check if streaming is active
 *
 * @param ctx ADC stream context
 * @return true if streaming, false otherwise
 */
bool adc_stream_is_active(const struct adc_stream_ctx *ctx)
{
	if (!ctx) {
		return false;
	}
	return ctx->active;
}

/**
 * @brief Get current streaming interval
 *
 * @param ctx ADC stream context
 * @return Interval in milliseconds
 */
uint32_t adc_stream_get_interval(const struct adc_stream_ctx *ctx)
{
	if (!ctx) {
		return 0;
	}
	return ctx->interval_ms;
}
