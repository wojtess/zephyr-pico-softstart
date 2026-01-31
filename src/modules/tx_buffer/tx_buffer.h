/**
 * @file tx_buffer.h
 * @brief Thread-safe TX ring buffer for UART transmission
 *
 * @details Ring buffer with semaphore signaling for thread-safe UART TX.
 *          Designed for multi-threaded environments where producer threads
 *          queue data and a consumer thread drains to UART.
 */

#ifndef TX_BUFFER_H
#define TX_BUFFER_H

#include <zephyr/kernel.h>
#include <stddef.h>
#include <stdint.h>

/* =========================================================================
 * CONTEXT STRUCTURE
 * ========================================================================= */

/**
 * @brief TX buffer context
 *
 * @details Thread-safe ring buffer with semaphore signaling.
 *          Must be initialized with tx_buffer_init() before use.
 */
struct tx_buffer {
	/** Underlying data buffer */
	uint8_t *buffer;

	/** Buffer size */
	size_t size;

	/** Write position */
	volatile size_t head;

	/** Read position */
	volatile size_t tail;

	/** Spinlock for thread safety */
	struct k_spinlock lock;

	/** Semaphore for signaling data available */
	struct k_sem sem_data;
};

/* =========================================================================
 * FUNCTION DECLARATIONS
 * ========================================================================= */

/**
 * @brief Initialize TX buffer
 *
 * @details Initializes ring buffer and semaphore.
 *
 * @param tb TX buffer context
 * @param data Underlying data buffer
 * @param size Buffer size
 * @return 0 on success, -EINVAL on invalid parameters
 */
int tx_buffer_init(struct tx_buffer *tb, uint8_t *data, size_t size);

/**
 * @brief Put data into TX buffer (thread-safe, blocking on space)
 *
 * @details Copies data to buffer. Returns error if buffer full.
 *
 * @param tb TX buffer context
 * @param data Data to write
 * @param len Length of data
 * @return Number of bytes written, or -ENOBUFS if buffer full
 */
int tx_buffer_put(struct tx_buffer *tb, const uint8_t *data, size_t len);

/**
 * @brief Get data from TX buffer (thread-safe)
 *
 * @details Copies data from buffer to output.
 *
 * @param tb TX buffer context
 * @param data Output buffer
 * @param len Maximum length to read
 * @return Number of bytes read
 */
size_t tx_buffer_get(struct tx_buffer *tb, uint8_t *data, size_t len);

/**
 * @brief Get number of bytes available for reading
 *
 * @details Non-locking read (may be slightly inaccurate)
 *
 * @param tb TX buffer context
 * @return Number of bytes available
 */
size_t tx_buffer_available(const struct tx_buffer *tb);

/**
 * @brief Wait for data to be available
 *
 * @details Blocks until data is available or timeout.
 *
 * @param tb TX buffer context
 * @param timeout_ms Timeout in milliseconds, or K_FOREVER to wait forever
 * @return 0 if data available, -EINPROGRESS if timeout
 */
int tx_buffer_wait_data(struct tx_buffer *tb, int32_t timeout_ms);

#endif /* TX_BUFFER_H */
