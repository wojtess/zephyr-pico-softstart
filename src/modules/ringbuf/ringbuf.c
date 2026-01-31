/**
 * @file ringbuf.c
 * @brief ISR-safe ring buffer implementation
 */

#include "ringbuf.h"
#include <zephyr/sys/__assert.h>

int ringbuf_init(struct ringbuf *rb, uint8_t *data, size_t size)
{
	if (!rb || !data || size == 0) {
		return -EINVAL;
	}

	rb->buffer = data;
	rb->size = size;
	rb->head = 0;
	rb->tail = 0;

	k_sem_init(&rb->sem_data, 0, size);

	/* Spinlock initialized implicitly in Zephyr 4.x */

	return 0;
}

int ringbuf_put(struct ringbuf *rb, uint8_t byte)
{
	k_spinlock_key_t key;
	size_t next_head;

	if (!rb) {
		return -EINVAL;
	}

	key = k_spin_lock(&rb->lock);

	next_head = (rb->head + 1) % rb->size;

	if (next_head == rb->tail) {
		k_spin_unlock(&rb->lock, key);
		return -ENOBUFS;  /* Buffer full */
	}

	rb->buffer[rb->head] = byte;
	rb->head = next_head;

	k_spin_unlock(&rb->lock, key);

	/* Signal data available */
	k_sem_give(&rb->sem_data);

	return 0;
}

int ringbuf_get(struct ringbuf *rb, uint8_t *byte)
{
	k_spinlock_key_t key;

	if (!rb || !byte) {
		return -EINVAL;
	}

	key = k_spin_lock(&rb->lock);

	if (rb->head == rb->tail) {
		k_spin_unlock(&rb->lock, key);
		return -ENOMSG;  /* Buffer empty */
	}

	*byte = rb->buffer[rb->tail];
	rb->tail = (rb->tail + 1) % rb->size;

	k_spin_unlock(&rb->lock, key);

	return 0;
}

void ringbuf_clear(struct ringbuf *rb)
{
	k_spinlock_key_t key;

	if (!rb) {
		return;
	}

	key = k_spin_lock(&rb->lock);

	/* Physically clear all bytes in the buffer to prevent data leakage */
	for (size_t i = 0; i < rb->size; i++) {
		rb->buffer[i] = 0;
	}

	/* Reset pointers */
	rb->head = 0;
	rb->tail = 0;

	k_spin_unlock(&rb->lock, key);
}

size_t ringbuf_available(const struct ringbuf *rb)
{
	size_t available;

	if (!rb) {
		return 0;
	}

	/* Non-locking read for stats (may be slightly inaccurate) */
	if (rb->head >= rb->tail) {
		available = rb->head - rb->tail;
	} else {
		available = rb->size - rb->tail;
	}

	return available;
}

int ringbuf_wait_data(struct ringbuf *rb, int32_t timeout_ms)
{
	if (!rb) {
		return -EINVAL;
	}

	return k_sem_take(&rb->sem_data, K_MSEC(timeout_ms));
}
