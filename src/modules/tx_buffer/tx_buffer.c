/**
 * @file tx_buffer.c
 * @brief Thread-safe TX ring buffer implementation
 */

#include "tx_buffer.h"
#include <zephyr/kernel.h>
#include <zephyr/sys/__assert.h>

/* =========================================================================
 * PUBLIC API
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
int tx_buffer_init(struct tx_buffer *tb, uint8_t *data, size_t size)
{
	if (!tb || !data || size == 0) {
		return -EINVAL;
	}

	tb->buffer = data;
	tb->size = size;
	tb->head = 0;
	tb->tail = 0;

	k_sem_init(&tb->sem_data, 0, size);

	return 0;
}

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
int tx_buffer_put(struct tx_buffer *tb, const uint8_t *data, size_t len)
{
	k_spinlock_key_t key;
	size_t available;
	size_t to_write;

	if (!tb || !data) {
		return -EINVAL;
	}

	if (len == 0) {
		return 0;
	}

	key = k_spin_lock(&tb->lock);

	/* Calculate available space */
	if (tb->head >= tb->tail) {
		available = tb->size - (tb->head - tb->tail);
	} else {
		available = tb->tail - tb->head;
	}

	if (len > available - 1) {  /* -1 for full check */
		k_spin_unlock(&tb->lock, key);
		return -ENOBUFS;
	}

	/* Write data */
	to_write = len;
	for (size_t i = 0; i < to_write; i++) {
		tb->buffer[tb->head] = data[i];
		tb->head = (tb->head + 1) % tb->size;
	}

	k_spin_unlock(&tb->lock, key);

	/* Signal data available */
	k_sem_give(&tb->sem_data);

	return to_write;
}

/**
 * @brief Put data into TX buffer (ISR-safe version)
 *
 * @details ISR-safe variant that doesn't use semaphore.
 *          Caller must manually wake TX thread if needed.
 *
 * @param tb TX buffer context
 * @param data Data to write
 * @param len Length of data
 * @return Number of bytes written, or -ENOBUFS if buffer full
 */
int tx_buffer_put_isr(struct tx_buffer *tb, const uint8_t *data, size_t len)
{
	k_spinlock_key_t key;
	size_t available;
	size_t to_write;

	if (!tb || !data) {
		return -EINVAL;
	}

	if (len == 0) {
		return 0;
	}

	key = k_spin_lock(&tb->lock);

	/* Calculate available space */
	if (tb->head >= tb->tail) {
		available = tb->size - (tb->head - tb->tail);
	} else {
		available = tb->tail - tb->head;
	}

	if (len > available - 1) {  /* -1 for full check */
		k_spin_unlock(&tb->lock, key);
		return -ENOBUFS;
	}

	/* Write data */
	to_write = len;
	for (size_t i = 0; i < to_write; i++) {
		tb->buffer[tb->head] = data[i];
		tb->head = (tb->head + 1) % tb->size;
	}

	k_spin_unlock(&tb->lock, key);

	/* Don't signal semaphore - not ISR-safe.
	 * TX thread polls periodically, so data will be picked up. */

	return to_write;
}

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
size_t tx_buffer_get(struct tx_buffer *tb, uint8_t *data, size_t len)
{
	k_spinlock_key_t key;
	size_t to_read;
	size_t count = 0;

	if (!tb || !data) {
		return 0;
	}

	if (len == 0) {
		return 0;
	}

	key = k_spin_lock(&tb->lock);

	/* Calculate available data */
	if (tb->head >= tb->tail) {
		to_read = tb->head - tb->tail;
	} else {
		to_read = tb->size - tb->tail;
	}

	if (to_read > len) {
		to_read = len;
	}

	/* Read data */
	for (size_t i = 0; i < to_read; i++) {
		data[i] = tb->buffer[tb->tail];
		tb->tail = (tb->tail + 1) % tb->size;
		count++;
	}

	k_spin_unlock(&tb->lock, key);

	return count;
}

/**
 * @brief Get number of bytes available for reading
 *
 * @details Non-locking read (may be slightly inaccurate)
 *
 * @param tb TX buffer context
 * @return Number of bytes available
 */
size_t tx_buffer_available(const struct tx_buffer *tb)
{
	size_t available;

	if (!tb) {
		return 0;
	}

	/* Non-locking read for stats (may be slightly inaccurate) */
	if (tb->head >= tb->tail) {
		available = tb->head - tb->tail;
	} else {
		available = tb->size - tb->tail;
	}

	return available;
}

/**
 * @brief Wait for data to be available
 *
 * @details Blocks until data is available or timeout.
 *
 * @param tb TX buffer context
 * @param timeout_ms Timeout in milliseconds, or K_FOREVER to wait forever
 * @return 0 if data available, -EINPROGRESS if timeout
 */
int tx_buffer_wait_data(struct tx_buffer *tb, int32_t timeout_ms)
{
	if (!tb) {
		return -EINVAL;
	}

	return k_sem_take(&tb->sem_data, K_MSEC(timeout_ms));
}
