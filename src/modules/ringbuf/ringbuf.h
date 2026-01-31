/**
 * @file ringbuf.h
 * @brief ISR-safe ring buffer for UART RX
 *
 * @details Thread-safe and ISR-safe ring buffer implementation using spinlocks.
 *          Designed for UART RX interrupt handler to main loop data transfer.
 */

#ifndef RINGBUF_H
#define RINGBUF_H

#include <zephyr/kernel.h>
#include <stddef.h>
#include <stdint.h>

/**
 * @brief Ring buffer structure
 */
struct ringbuf {
	uint8_t *buffer;              /**< Underlying data buffer */
	size_t size;                  /**< Buffer size */
	volatile size_t head;          /**< Write position (ISR-modified) */
	volatile size_t tail;          /**< Read position */
	struct k_spinlock lock;       /**< Spinlock for ISR safety */
	struct k_sem sem_data;        /**< Semaphore for signaling data available */
};

/**
 * @brief Initialize ring buffer
 *
 * @param rb Ring buffer instance
 * @param data Underlying data buffer
 * @param size Buffer size (must be power of 2 for efficiency)
 * @return 0 on success, -EINVAL on invalid parameters
 */
int ringbuf_init(struct ringbuf *rb, uint8_t *data, size_t size);

/**
 * @brief Put single byte into ring buffer (ISR-safe, thread-safe)
 *
 * @param rb Ring buffer instance
 * @param byte Byte to add
 * @return 0 on success, -ENOBUFS if buffer full
 */
int ringbuf_put(struct ringbuf *rb, uint8_t byte);

/**
 * @brief Get single byte from ring buffer (thread-safe)
 *
 * @param rb Ring buffer instance
 * @param byte Pointer to store retrieved byte
 * @return 0 on success, -ENOMSG if buffer empty
 */
int ringbuf_get(struct ringbuf *rb, uint8_t *byte);

/**
 * @brief Clear ring buffer by resetting head and tail
 *
 * @details Thread-safe with spinlock protection.
 *          Physically clears all bytes to prevent data leakage.
 *
 * @param rb Ring buffer instance
 */
void ringbuf_clear(struct ringbuf *rb);

/**
 * @brief Get number of bytes available for reading
 *
 * @details Non-locking read (may be slightly inaccurate in concurrent access)
 *
 * @param rb Ring buffer instance
 * @return Number of bytes available
 */
size_t ringbuf_available(const struct ringbuf *rb);

/**
 * @brief Check if buffer is empty
 *
 * @details Non-locking read
 *
 * @param rb Ring buffer instance
 * @return true if empty, false otherwise
 */
static inline bool ringbuf_is_empty(const struct ringbuf *rb)
{
	return rb->head == rb->tail;
}

/**
 * @brief Check if buffer is full
 *
 * @details Thread-safe read with spinlock protection
 *
 * @param rb Ring buffer instance
 * @return true if full, false otherwise
 */
bool ringbuf_is_full(struct ringbuf *rb);

/**
 * @brief Wait for data to be available
 *
 * @details Blocks until data is available or timeout.
 *
 * @param rb Ring buffer instance
 * @param timeout_ms Timeout in milliseconds, or INT32_MAX to wait forever
 * @return 0 if data available, -EINPROGRESS if timeout
 */
int ringbuf_wait_data(struct ringbuf *rb, int32_t timeout_ms);

#endif /* RINGBUF_H */
