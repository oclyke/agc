/* test_starter.c - native tests for the portable parts of the starter repo.
 *
 * Build and run:   make test
 *
 * Also emits stream.bin, which the Python round-trip test decodes with the same
 * receiver the candidate will use. That is the check that matters: it proves the
 * C encoder and the Python decoder agree byte for byte.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "framing.h"
#include "instrument.h"
#include "ringbuf.h"

static int failures = 0;

#define CHECK(cond, msg)                                                       \
    do {                                                                       \
        if (!(cond)) {                                                         \
            printf("  FAIL  %s  (%s:%d)\n", (msg), __FILE__, __LINE__);        \
            failures++;                                                        \
        } else {                                                               \
            printf("  ok    %s\n", (msg));                                     \
        }                                                                      \
    } while (0)

static void test_crc(void)
{
    printf("CRC-16/CCITT-FALSE\n");
    /* The canonical check value for this variant: CRC("123456789") == 0x29B1 */
    CHECK(crc16_ccitt((const uint8_t *)"123456789", 9) == 0x29B1u,
          "check value 0x29B1");
    CHECK(crc16_ccitt((const uint8_t *)"", 0) == 0xFFFFu, "empty input is init value");

    uint8_t a[4] = {0xDE, 0xAD, 0xBE, 0xEF};
    uint8_t b[4] = {0xDE, 0xAD, 0xBE, 0xEE};
    CHECK(crc16_ccitt(a, 4) != crc16_ccitt(b, 4), "single bit change alters CRC");
}

static void test_frame_shape(void)
{
    printf("Frame construction\n");
    int16_t in[BLOCK_SAMPLES], out[BLOCK_SAMPLES];
    for (int i = 0; i < BLOCK_SAMPLES; i++) {
        in[i] = (int16_t)(i * 37);
        out[i] = (int16_t)(-i * 11);
    }

    uint8_t buf[AUDIO_FRAME_BYTES];
    size_t n = frame_audio(buf, sizeof buf, 0x12345678u, in, out);
    CHECK(n == AUDIO_FRAME_BYTES, "audio frame is 267 bytes");
    CHECK(buf[0] == 0xA5 && buf[1] == 0x5A, "sync word");
    CHECK(buf[2] == FRAME_TYPE_AUDIO, "type byte");
    CHECK(buf[3] == (AUDIO_PAYLOAD_BYTES & 0xFF) && buf[4] == (AUDIO_PAYLOAD_BYTES >> 8),
          "length is little-endian 260");
    CHECK(buf[5] == 0x78 && buf[6] == 0x56 && buf[7] == 0x34 && buf[8] == 0x12,
          "sequence number is little-endian");

    uint16_t want = crc16_ccitt(&buf[2], AUDIO_PAYLOAD_BYTES + 3);
    uint16_t got = (uint16_t)(buf[5 + AUDIO_PAYLOAD_BYTES] |
                              (buf[6 + AUDIO_PAYLOAD_BYTES] << 8));
    CHECK(want == got, "CRC trailer matches recomputation");

    CHECK(frame_audio(buf, AUDIO_FRAME_BYTES - 1, 0, in, out) == 0,
          "refuses to write past a short buffer");

    telem_t t = {0};
    uint8_t tb[TELEM_FRAME_BYTES];
    CHECK(frame_telem(tb, sizeof tb, &t) == TELEM_FRAME_BYTES,
          "telemetry frame is 43 bytes");
    CHECK(frame_telem(tb, TELEM_FRAME_BYTES - 1, &t) == 0,
          "telemetry refuses a short buffer");
}

static void test_stack_paint(void)
{
    printf("Stack painting\n");
    /* A standalone region standing in for a stack: paint it, dirty part of it,
     * and confirm the high-water calculation. */
    static uint32_t region[256];
    for (size_t i = 0; i < 256; i++) region[i] = 0u;

    stack_paint(region, region + 256);
    CHECK(region[0] == STACK_PAINT_VALUE, "low end painted");
    CHECK(region[200] == STACK_PAINT_VALUE, "far end painted");
    CHECK(stack_free_bytes(region, region + 256) == 256u * 4u,
          "all bytes free before anything is used");

    /* The stack grows DOWNWARD from the high address, so usage consumes the
     * high end and the untouched paint survives at the low end. Simulate 40
     * words of usage by dirtying the top of the region. */
    for (int i = 216; i < 256; i++) region[i] = 0x12345678u;
    CHECK(stack_free_bytes(region, region + 256) == 216u * 4u,
          "high-water reflects 40 words used from the top");

    /* One word of deeper usage must be visible. */
    region[215] = 0x99u;
    CHECK(stack_free_bytes(region, region + 256) == 215u * 4u,
          "one more word of depth is detected");

    CHECK(stack_free_bytes(NULL, NULL) == 0u, "null bounds are safe");
    CHECK(stack_free_bytes(region + 10, region) == 0u, "inverted bounds are safe");
    stack_paint(NULL, NULL); /* must not crash */
    CHECK(1, "painting null bounds does not crash");
}

static void test_ringbuf(void)
{
    printf("Ring buffer\n");
    uint8_t storage[256];
    ringbuf_t rb;

    CHECK(rb_init(&rb, storage, 256) == true, "init with power-of-two size");
    CHECK(rb_init(&rb, storage, 100) == false, "rejects non-power-of-two size");
    (void)rb_init(&rb, storage, 256);

    CHECK(rb_count(&rb) == 0, "starts empty");
    CHECK(rb_space(&rb) == 255, "usable capacity is size-1");

    uint8_t src[64], dst[64];
    for (int i = 0; i < 64; i++) src[i] = (uint8_t)i;

    CHECK(rb_write(&rb, src, 64) == 64, "write 64");
    CHECK(rb_count(&rb) == 64, "count reflects write");
    CHECK(rb_read(&rb, dst, 64) == 64, "read 64");
    CHECK(memcmp(src, dst, 64) == 0, "data round-trips intact");
    CHECK(rb_count(&rb) == 0, "empty again");

    /* Overfill: must write only what fits, never overwrite unread data. */
    rb_reset(&rb);
    uint8_t big[300];
    for (int i = 0; i < 300; i++) big[i] = (uint8_t)(i & 0xFF);
    size_t w = rb_write(&rb, big, 300);
    CHECK(w == 255, "short write on overfill, no overwrite");
    CHECK(rb_space(&rb) == 0, "full");
    CHECK(rb_write(&rb, big, 1) == 0, "write to full buffer returns 0");

    rb_reset(&rb);
    CHECK(rb_write_all(&rb, big, 300) == false, "write_all refuses when it will not fit");
    CHECK(rb_count(&rb) == 0, "write_all wrote nothing on refusal");
    CHECK(rb_write_all(&rb, big, 200) == true, "write_all succeeds when it fits");

    /* Wraparound stress: interleaved partial writes and reads, odd sizes, many
     * laps around the buffer. This is where naive implementations break. */
    rb_reset(&rb);
    uint8_t expect_w = 0, expect_r = 0;
    int bad = 0;
    for (int iter = 0; iter < 20000; iter++) {
        size_t wn = (size_t)(rand() % 70);
        uint8_t tmp[70];
        for (size_t i = 0; i < wn; i++) tmp[i] = expect_w++;
        size_t did = rb_write(&rb, tmp, wn);
        expect_w = (uint8_t)(expect_w - (wn - did)); /* rewind unwritten */

        size_t rn = (size_t)(rand() % 70);
        uint8_t got[70];
        size_t gotn = rb_read(&rb, got, rn);
        for (size_t i = 0; i < gotn; i++) {
            if (got[i] != expect_r++) bad++;
        }
    }
    CHECK(bad == 0, "20000 interleaved read/write laps preserve byte order");

    /* Zero-copy peek path used for DMA transmit. */
    rb_reset(&rb);
    (void)rb_write(&rb, src, 64);
    size_t clen = 0;
    const uint8_t *cp = rb_peek_contiguous(&rb, &clen);
    CHECK(clen == 64 && cp[0] == 0, "peek exposes contiguous run");
    rb_consume(&rb, 32);
    CHECK(rb_count(&rb) == 32, "consume advances tail");
    rb_peek_contiguous(&rb, &clen);
    CHECK(clen == 32, "peek reflects consumption");
}

/* Emit a stream for the Python round-trip test. */
static void emit_stream(const char *path)
{
    FILE *f = fopen(path, "wb");
    if (!f) { printf("  FAIL  cannot open %s\n", path); failures++; return; }

    uint8_t buf[AUDIO_FRAME_BYTES];
    int16_t in[BLOCK_SAMPLES], out[BLOCK_SAMPLES];

    for (uint32_t seq = 0; seq < 50; seq++) {
        for (int i = 0; i < BLOCK_SAMPLES; i++) {
            in[i]  = (int16_t)((seq * BLOCK_SAMPLES + i) * 7 - 20000);
            out[i] = (int16_t)(-(int)(seq * 3 + i));
        }
        size_t n = frame_audio(buf, sizeof buf, seq, in, out);
        fwrite(buf, 1, n, f);

        if (seq % 10 == 5) {
            telem_t t;
            t.timestamp_ms       = seq * 8u;
            t.gain_db            = 12.5f + (float)seq;
            t.rms_in_dbfs        = -33.25f;
            t.rms_out_dbfs       = -20.0f;
            t.worst_block_cycles = 412345u + seq;
            t.dma_overruns       = 0u;
            t.adc_overruns       = 0u;
            t.stream_drops       = seq;
            t.blocks_processed   = seq * 6u;
            uint8_t tb[TELEM_FRAME_BYTES];
            size_t tn = frame_telem(tb, sizeof tb, &t);
            fwrite(tb, 1, tn, f);
        }
    }
    fclose(f);
    printf("  ok    wrote %s for the Python round-trip test\n", path);
}

int main(void)
{
    srand(12345);
    test_crc();
    test_frame_shape();
    test_ringbuf();
    test_stack_paint();
    printf("Stream emission\n");
    emit_stream("stream.bin");

    printf("\n%s\n", failures == 0 ? "ALL C TESTS PASSED" : "SOME C TESTS FAILED");
    return failures == 0 ? 0 : 1;
}
