/* arm_smoke.c - forces every inline helper in instrument.h to be instantiated
 * and type-checked by the cross-compiler. Compiled by `make -f arm_check.mk`,
 * never linked or run. It is not part of the native test suite.
 */
#include "framing.h"
#include "instrument.h"
#include "ringbuf.h"

static uint8_t   rb_storage[1024];
static ringbuf_t rb;
static int16_t   in_blk[BLOCK_SAMPLES], out_blk[BLOCK_SAMPLES];
static uint8_t   frame[AUDIO_FRAME_BYTES];

volatile bool     g_dwt_ok;
volatile uint32_t g_sink;

void arm_smoke(void)
{
    g_dwt_ok = dwt_init();
    rb_init(&rb, rb_storage, sizeof rb_storage);

    BLOCK_TIMER_START();
    for (int i = 0; i < BLOCK_SAMPLES; i++) {
        out_blk[i] = (int16_t)(in_blk[i] >> 1);
    }
    BLOCK_TIMER_END();

    size_t n = frame_audio(frame, sizeof frame, g_counters.blocks, in_blk, out_blk);
    if (!rb_write_all(&rb, frame, n)) {
        g_counters.stream_drops++;
    }

    size_t clen = 0;
    const uint8_t *cp = rb_peek_contiguous(&rb, &clen);
    g_sink = (clen > 0u) ? cp[0] : 0u;
    rb_consume(&rb, clen);

    telem_t t;
    t.timestamp_ms       = 0u;
    t.gain_db            = 0.0f;
    t.rms_in_dbfs        = -20.0f;
    t.rms_out_dbfs       = -20.0f;
    t.worst_block_cycles = g_counters.worst_cycles;
    t.dma_overruns       = g_counters.dma_overruns;
    t.adc_overruns       = g_counters.adc_overruns;
    t.stream_drops       = g_counters.stream_drops;
    t.blocks_processed   = g_counters.blocks;
    uint8_t tb[TELEM_FRAME_BYTES];
    g_sink += (uint32_t)frame_telem(tb, sizeof tb, &t);

    static uint32_t fake_stack[64];
    stack_paint(fake_stack, fake_stack + 64);
    g_sink += stack_free_bytes(fake_stack, fake_stack + 64);
}
