/* ringbuf.h - single-producer / single-consumer byte ring buffer.
 *
 * Portable C99, no allocation, no locks. Verified by the native test in test/,
 * including an interleaved producer/consumer stress test.
 *
 * The discipline this relies on:
 *   - Exactly ONE writer. In this project that is your block-processing context.
 *   - Exactly ONE reader. In this project that is your UART TX path.
 *   - head is written only by the producer; tail only by the consumer.
 * Under those rules no lock is needed on a single-core Cortex-M. If you break
 * either rule - say by writing from two different interrupt priorities - this
 * is no longer safe, and that is a design decision you own.
 *
 * On backpressure: rb_write() writes what fits and returns the count. It never
 * blocks and never overwrites unread data. A short write means the consumer is
 * behind; count those bytes as dropped and keep going. Stalling the audio path
 * to wait for the UART is the wrong answer.
 */
#ifndef RINGBUF_H
#define RINGBUF_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Capacity must be a power of two. Usable capacity is size - 1 bytes. */
typedef struct {
    uint8_t *buf;
    size_t   size;          /* power of two */
    size_t   mask;          /* size - 1 */
    volatile size_t head;   /* producer writes here */
    volatile size_t tail;   /* consumer reads here */
} ringbuf_t;

/* Returns false if size is zero or not a power of two. */
bool   rb_init(ringbuf_t *rb, uint8_t *storage, size_t size);
void   rb_reset(ringbuf_t *rb);

size_t rb_count(const ringbuf_t *rb);  /* bytes available to read */
size_t rb_space(const ringbuf_t *rb);  /* bytes available to write */

/* Producer. Writes min(len, rb_space()) bytes; returns how many. */
size_t rb_write(ringbuf_t *rb, const uint8_t *src, size_t len);

/* All-or-nothing write. Returns false and writes nothing if it does not fit.
 * Preferred for frames: half a frame in the stream is worse than no frame. */
bool   rb_write_all(ringbuf_t *rb, const uint8_t *src, size_t len);

/* Consumer. Reads min(len, rb_count()) bytes; returns how many. */
size_t rb_read(ringbuf_t *rb, uint8_t *dst, size_t len);

/* Zero-copy consumer helper: hands back a pointer to the largest contiguous
 * readable run, for handing straight to a DMA transfer. Call rb_consume() with
 * the number of bytes actually sent once the transfer completes. */
const uint8_t *rb_peek_contiguous(const ringbuf_t *rb, size_t *out_len);
void   rb_consume(ringbuf_t *rb, size_t len);

#endif /* RINGBUF_H */
