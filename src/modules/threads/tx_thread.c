/**
 * @file tx_thread.c
 * @brief TX processing thread implementation
 */

#include "tx_thread.h"
#include "../tx_buffer/tx_buffer.h"
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/printk.h>

/* =========================================================================
 * INTERNAL STATE
 * ========================================================================= */

/** @brief TX thread stack */
K_THREAD_STACK_DEFINE(tx_thread_stack, TX_THREAD_STACK_SIZE);

/** @brief TX thread data */
static struct k_thread tx_thread_data;

/** @brief Global TX thread context (for thread entry) */
static struct tx_thread_ctx *g_tx_ctx;

/* =========================================================================
 * THREAD ENTRY POINT
 * ========================================================================= */

/**
 * @brief TX thread entry point
 *
 * @details Drains TX buffer to UART in a loop.
 *          Waits on semaphore for data availability.
 *
 * @param p1 Unused (required by Zephyr thread API)
 * @param p2 Unused
 * @param p3 Unused
 */
static void tx_thread_entry(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	uint8_t data[16];
	size_t len;

	printk("TX thread started\n");

	while (true) {
		/* Check for data availability with short timeout (10ms) */
		/* This allows TX thread to poll for data from ISR context */
		int ret = tx_buffer_wait_data(g_tx_ctx->tx_buf, 10);
		if (ret != 0) {
			/* Timeout - check if data available anyway (for ISR-sent data) */
			if (tx_buffer_available(g_tx_ctx->tx_buf) == 0) {
				continue;
			}
		}

		/* Drain available data */
		len = tx_buffer_get(g_tx_ctx->tx_buf, data, sizeof(data));
		if (len > 0) {
			uart_fifo_fill(g_tx_ctx->uart_dev, data, len);
		}
	}
}

/* =========================================================================
 * PUBLIC API
 * ========================================================================= */

/**
 * @brief Start TX thread
 *
 * @details Creates and starts TX processing thread.
 *          Thread runs until system shutdown.
 *
 * @param ctx TX thread context to initialize
 * @param tx_buf TX buffer to drain
 * @param uart_dev UART device for transmission
 * @return 0 on success, negative errno on failure
 */
int tx_thread_start(struct tx_thread_ctx *ctx,
		    struct tx_buffer *tx_buf,
		    const struct device *uart_dev)
{
	if (!ctx || !tx_buf || !uart_dev) {
		return -EINVAL;
	}

	ctx->tx_buf = tx_buf;
	ctx->uart_dev = uart_dev;

	/* Store global context for thread entry */
	g_tx_ctx = ctx;

	/* Create and start TX thread */
	k_thread_create(&tx_thread_data,
		       tx_thread_stack,
		       TX_THREAD_STACK_SIZE,
		       tx_thread_entry,
		       NULL, NULL, NULL,
		       TX_THREAD_PRIORITY, 0, K_NO_WAIT);

	/* Give thread a name for debugging */
	k_thread_name_set(&tx_thread_data, "tx_thread");

	return 0;
}
