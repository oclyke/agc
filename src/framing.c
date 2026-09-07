/* framing.c - see framing.h */
#include "framing.h"

#include <string.h>

uint16_t crc16_ccitt(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFFu;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)((uint16_t)data[i] << 8);
        for (int b = 0; b < 8; b++) {
            crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u)
                                  : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

/* Little-endian writers. Explicit byte writes rather than memcpy of a struct,
 * so the wire format does not depend on the compiler's padding or the host's
 * endianness. */
static size_t put_u16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFFu);
    p[1] = (uint8_t)(v >> 8);
    return 2;
}

static size_t put_u32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v & 0xFFu);
    p[1] = (uint8_t)((v >> 8) & 0xFFu);
    p[2] = (uint8_t)((v >> 16) & 0xFFu);
    p[3] = (uint8_t)((v >> 24) & 0xFFu);
    return 4;
}

static size_t put_f32(uint8_t *p, float f)
{
    uint32_t bits;
    memcpy(&bits, &f, sizeof bits); /* type-pun without violating strict aliasing */
    return put_u32(p, bits);
}

static size_t put_i16(uint8_t *p, int16_t v)
{
    return put_u16(p, (uint16_t)v);
}

/* Common header + trailer. Payload must already be at out_buf[5]. */
static size_t finish_frame(uint8_t *out_buf, uint8_t type, uint16_t payload_len)
{
    out_buf[0] = FRAME_SYNC0;
    out_buf[1] = FRAME_SYNC1;
    out_buf[2] = type;
    put_u16(&out_buf[3], payload_len);
    /* CRC covers type, length and payload: bytes [2 .. 4+len] */
    uint16_t crc = crc16_ccitt(&out_buf[2], (size_t)payload_len + 3u);
    put_u16(&out_buf[5 + payload_len], crc);
    return (size_t)payload_len + FRAME_OVERHEAD_BYTES;
}

size_t frame_audio(uint8_t *out_buf, size_t out_cap, uint32_t seq,
                   const int16_t *in, const int16_t *out)
{
    if (out_buf == NULL || out_cap < AUDIO_FRAME_BYTES) {
        return 0;
    }
    uint8_t *p = &out_buf[5];
    p += put_u32(p, seq);
    for (int i = 0; i < BLOCK_SAMPLES; i++) {
        p += put_i16(p, in[i]);
        p += put_i16(p, out[i]);
    }
    return finish_frame(out_buf, FRAME_TYPE_AUDIO, AUDIO_PAYLOAD_BYTES);
}

size_t frame_telem(uint8_t *out_buf, size_t out_cap, const telem_t *t)
{
    if (out_buf == NULL || t == NULL || out_cap < TELEM_FRAME_BYTES) {
        return 0;
    }
    uint8_t *p = &out_buf[5];
    p += put_u32(p, t->timestamp_ms);
    p += put_f32(p, t->gain_db);
    p += put_f32(p, t->rms_in_dbfs);
    p += put_f32(p, t->rms_out_dbfs);
    p += put_u32(p, t->worst_block_cycles);
    p += put_u32(p, t->dma_overruns);
    p += put_u32(p, t->adc_overruns);
    p += put_u32(p, t->stream_drops);
    p += put_u32(p, t->blocks_processed);
    return finish_frame(out_buf, FRAME_TYPE_TELEM, TELEM_PAYLOAD_BYTES);
}
