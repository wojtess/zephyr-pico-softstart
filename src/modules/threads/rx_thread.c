/**
 * @file rx_thread.c
 * @brief RX processing thread implementation
 */

#include "rx_thread.h"
#include "../ringbuf/ringbuf.h"
#include "../protocol/protocol.h"
#include <zephyr/sys/printk.h>

/* =========================================================================
 * INTERNAL STATE
 * ========================================================================= */

/** @brief RX thread stack */
K_THREAD_STACK_DEFINE(rx_thread_stack, RX_THREAD_STACK_SIZE);

/** @brief RX thread data */
static struct k_thread rx_thread_data;

/** @brief Global RX thread context (for thread entry) */
static struct rx_thread_ctx *g_rx_ctx;

/* =========================================================================
 * THREAD ENTRY POINT
 * ========================================================================= */

/**
 * @brief RX thread entry point
 *
 * @details Reads from RX buffer and processes through protocol state machine.
 *          Waits on semaphore for data availability.
 *
 * @param p1 Unused (required by Zephyr thread API)
 * @param p2 Unused
 * @param p3 Unused
 */
static void rx_thread_entry(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	uint8_t byte;
	int ret;

	printk("RX thread started\n");

	while (true) {
		/* Wait for data to be available (INT32_MAX = forever) */
		ret = ringbuf_wait_data(g_rx_ctx->rx_buf, INT32_MAX);
		if (ret != 0) {
			/* Timeout or error - should not happen with INT32_MAX */
			continue;
		}

		/* Process one byte at a time */
		ret = ringbuf_get(g_rx_ctx->rx_buf, &byte);
		if (ret == 0) {
			proto_process_byte(g_rx_ctx->proto, byte);
		}
	}
}

/* =========================================================================
 * PUBLIC API
 * ========================================================================= */

/**
 * @brief Start RX thread
 *
 * @details Creates and starts RX processing thread.
 *          Thread runs until system shutdown.
 *
 * @param ctx RX thread context to initialize
 * @param rx_buf RX ring buffer to read from
 * @param proto Protocol context for processing
 * @return 0 on success, negative errno on failure
 */
int rx_thread_start(struct rx_thread_ctx *ctx,
		    struct ringbuf *rx_buf,
		    struct proto_ctx *proto)
{
	if (!ctx || !rx_buf || !proto) {
		return -EINVAL;
	}

	ctx->rx_buf = rx_buf;
	ctx->proto = proto;

	/* Store global context for thread entry */
	g_rx_ctx = ctx;

	/* Create and start RX thread */
	k_thread_create(&rx_thread_data,
		       rx_thread_stack,
		       RX_THREAD_STACK_SIZE,
		       rx_thread_entry,
		       NULL, NULL, NULL,
		       RX_THREAD_PRIORITY, 0, K_NO_WAIT);

	/* Give thread a name for debugging */
	k_thread_name_set(&rx_thread_data, "rx_thread");

	return 0;
}
