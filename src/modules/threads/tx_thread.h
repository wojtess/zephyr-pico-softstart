/**
 * @file tx_thread.h
 * @brief TX processing thread for UART transmission
 *
 * @details Dedicated thread that drains TX ring buffer to UART.
 *          Waits on semaphore for data availability.
 */

#ifndef TX_THREAD_H
#define TX_THREAD_H

#include <zephyr/kernel.h>

/* =========================================================================
 * CONFIGURATION
 * ========================================================================= */

/** @brief TX thread priority (-2 = high priority) */
#define TX_THREAD_PRIORITY -2

/** @brief TX thread stack size */
#define TX_THREAD_STACK_SIZE 1024

/* =========================================================================
 * CONTEXT STRUCTURE
 * ========================================================================= */

/**
 * @brief TX thread context
 *
 * @details Holds TX buffer reference and thread data.
 *          Must be initialized with tx_thread_start() before use.
 */
struct tx_thread_ctx {
	/** TX buffer to drain */
	struct tx_buffer *tx_buf;

	/** UART device for transmission */
	const struct device *uart_dev;
};

/* =========================================================================
 * FUNCTION DECLARATIONS
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
		    const struct device *uart_dev);

#endif /* TX_THREAD_H */
