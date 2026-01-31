/**
 * @file rx_thread.h
 * @brief RX processing thread for protocol handling
 *
 * @details Dedicated thread that processes RX ring buffer through protocol state machine.
 *          Waits on semaphore for data availability.
 */

#ifndef RX_THREAD_H
#define RX_THREAD_H

#include <zephyr/kernel.h>

/* =========================================================================
 * CONFIGURATION
 * ========================================================================= */

/** @brief RX thread priority (-2 = high priority) */
#define RX_THREAD_PRIORITY -2

/** @brief RX thread stack size */
#define RX_THREAD_STACK_SIZE 1024

/* =========================================================================
 * CONTEXT STRUCTURE
 * ========================================================================= */

/**
 * @brief RX thread context
 *
 * @details Holds RX buffer and protocol context references.
 *          Must be initialized with rx_thread_start() before use.
 */
struct rx_thread_ctx {
	/** RX ring buffer to read from */
	struct ringbuf *rx_buf;

	/** Protocol context for processing */
	struct proto_ctx *proto;
};

/* =========================================================================
 * FUNCTION DECLARATIONS
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
		    struct proto_ctx *proto);

#endif /* RX_THREAD_H */
