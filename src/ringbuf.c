/* ringbuf.c - see ringbuf.h */
#include "ringbuf.h"

#include <string.h>

/* Compiler barrier. On a single-core Cortex-M this is sufficient: it stops the
 * compiler reordering the data writes past the index update. If you ever move
 * this to a multi-core part you need real memory barriers (DMB) instead. */
#if defined(__GNUC__) || defined(__clang__)
#define RB_BARRIER() __asm__ __volatile__("" ::: "memory")
#else
#define RB_BARRIER() ((void)0)
#endif

bool rb_init(ringbuf_t *rb, uint8_t *storage, size_t size)
{
    if (rb == NULL || storage == NULL || size == 0u || (size & (size - 1u)) != 0u) {
        return false;
    }
    rb->buf = storage;
    rb->size = size;
    rb->mask = size - 1u;
    rb->head = 0u;
    rb->tail = 0u;
    return true;
}

void rb_reset(ringbuf_t *rb)
{
    rb->head = 0u;
    rb->tail = 0u;
}

size_t rb_count(const ringbuf_t *rb)
{
    return (rb->head - rb->tail) & rb->mask;
}

size_t rb_space(const ringbuf_t *rb)
{
    /* One slot is kept empty so head == tail always means "empty". */
    return rb->mask - rb_count(rb);
}

size_t rb_write(ringbuf_t *rb, const uint8_t *src, size_t len)
{
    size_t space = rb_space(rb);
    if (len > space) {
        len = space;
    }
    if (len == 0u) {
        return 0u;
    }

    size_t h = rb->head;
    size_t first = rb->size - h;
    if (first > len) {
        first = len;
    }
    memcpy(&rb->buf[h], src, first);
    if (len > first) {
        memcpy(&rb->buf[0], src + first, len - first);
    }

    RB_BARRIER(); /* data must land before the consumer can see the new head */
    rb->head = (h + len) & rb->mask;
    return len;
}

bool rb_write_all(ringbuf_t *rb, const uint8_t *src, size_t len)
{
    if (rb_space(rb) < len) {
        return false;
    }
    (void)rb_write(rb, src, len);
    return true;
}

size_t rb_read(ringbuf_t *rb, uint8_t *dst, size_t len)
{
    size_t avail = rb_count(rb);
    if (len > avail) {
        len = avail;
    }
    if (len == 0u) {
        return 0u;
    }

    size_t t = rb->tail;
    size_t first = rb->size - t;
    if (first > len) {
        first = len;
    }
    memcpy(dst, &rb->buf[t], first);
    if (len > first) {
        memcpy(dst + first, &rb->buf[0], len - first);
    }

    RB_BARRIER(); /* finish reading before the producer can reuse the space */
    rb->tail = (t + len) & rb->mask;
    return len;
}

const uint8_t *rb_peek_contiguous(const ringbuf_t *rb, size_t *out_len)
{
    size_t avail = rb_count(rb);
    size_t t = rb->tail;
    size_t to_end = rb->size - t;
    *out_len = (avail < to_end) ? avail : to_end;
    return &rb->buf[t];
}

void rb_consume(ringbuf_t *rb, size_t len)
{
    size_t avail = rb_count(rb);
    if (len > avail) {
        len = avail;
    }
    RB_BARRIER();
    rb->tail = (rb->tail + len) & rb->mask;
}
