/**
 * @file adc_stream.c
 * @brief ADC streaming implementation
 */

#include "adc_stream.h"
#include "../drivers/led_pwm.h"
#include "../adc_reader/adc_reader.h"
#include "../tx_buffer/tx_buffer.h"
#include "../protocol/protocol.h"

/* Debug flag - ustaw na 1 aby włączyć pakiety debug */
#define ADC_STREAM_DEBUG_ENABLED 0

/* Debug codes */
#define DEBUG_TIMER_START  1
#define DEBUG_TIMER_CB     2
#define DEBUG_ADC_READ     3
#define DEBUG_ADC_ERROR    4
#define DEBUG_TX_PUT       5

/* =========================================================================
 * WORK QUEUE (dla ADC read z thread context)
 * ========================================================================= */

#if ADC_STREAM_DEBUG_ENABLED
/**
 * @brief Send debug packet to TX buffer
 */
static void send_debug(struct adc_stream_ctx *ctx, uint8_t code)
{
	uint8_t debug_pkt[2] = {0xFD, code};  /* DEBUG marker + code */
	tx_buffer_put_isr(ctx->tx_buf, debug_pkt, sizeof(debug_pkt));
}
#endif

/**
 * @brief Work item for sending ADC stream data
 *
 * @details Reads the latest ADC value from shared ADC reader and
 *          sends it via TX buffer. Runs in thread context.
 */
static void adc_stream_work_handler(struct k_work *work)
{
	struct adc_stream_ctx *ctx =
		CONTAINER_OF(work, struct adc_stream_ctx, work);

	uint16_t adc_value;
	uint8_t response[4];
	uint8_t crc;

#if ADC_STREAM_DEBUG_ENABLED
	/* Debug: work handler entered */
	send_debug(ctx, DEBUG_ADC_READ);
#endif

	/* Get latest ADC value from shared reader */
	if (ctx->adc_reader != NULL && adc_reader_is_active(ctx->adc_reader)) {
		adc_value = adc_reader_get_last(ctx->adc_reader);
	} else {
		/* No ADC reader configured or not active - skip this cycle */
#if ADC_STREAM_DEBUG_ENABLED
		send_debug(ctx, DEBUG_ADC_ERROR);
#endif
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
#if ADC_STREAM_DEBUG_ENABLED
	send_debug(ctx, DEBUG_TX_PUT);
#endif
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

#if ADC_STREAM_DEBUG_ENABLED
	/* Debug: timer callback entered */
	send_debug(ctx, DEBUG_TIMER_CB);
#endif

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
 * @param tx_buf TX buffer for sending ADC values
 * @param led LED/PWM driver context (for debug signaling)
 * @return 0 on success, negative errno on failure
 */
int adc_stream_init(struct adc_stream_ctx *ctx,
		    struct tx_buffer *tx_buf,
		    struct led_pwm_ctx *led)
{
	if (!ctx || !tx_buf) {
		return -EINVAL;
	}

	ctx->adc_reader = NULL;
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
 * @brief Set ADC reader for streaming module
 *
 * @details Sets the shared ADC reader context.
 *          Must be called before starting streaming.
 *
 * @param ctx ADC stream context
 * @param adc_reader ADC reader context (shared source)
 */
void adc_stream_set_adc_reader(struct adc_stream_ctx *ctx,
			       struct adc_reader_ctx *adc_reader)
{
	if (ctx) {
		ctx->adc_reader = adc_reader;
	}
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

#if ADC_STREAM_DEBUG_ENABLED
	/* Debug: timer started (wysyłamy PO timer start, żeby ACK przyszło pierwsze) */
	send_debug(ctx, DEBUG_TIMER_START);
#endif

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
