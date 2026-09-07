/* framing.h - wire framing for the AGC challenge.
 *
 * Portable C99. No STM32 dependencies, no dynamic allocation, no I/O.
 * This code is verified by the native test in test/ against the Python host
 * receiver, so you should not need to debug the protocol.
 *
 * See PROTOCOL.md for the byte layout.
 */
#ifndef FRAMING_H
#define FRAMING_H

#include <stddef.h>
#include <stdint.h>

#define FRAME_SYNC0 0xA5u
#define FRAME_SYNC1 0x5Au

#define FRAME_TYPE_AUDIO 0x01u
#define FRAME_TYPE_TELEM 0x02u

#define BLOCK_SAMPLES 64
#define AUDIO_PAYLOAD_BYTES 260 /* 4 seq + 64 stereo pairs * 2 ch * 2 bytes */
#define TELEM_PAYLOAD_BYTES 36
#define FRAME_OVERHEAD_BYTES 7 /* 2 sync + 1 type + 2 len + 2 crc */

#define AUDIO_FRAME_BYTES (AUDIO_PAYLOAD_BYTES + FRAME_OVERHEAD_BYTES) /* 267 */
#define TELEM_FRAME_BYTES (TELEM_PAYLOAD_BYTES + FRAME_OVERHEAD_BYTES) /*  43 */

/* Telemetry record. Field order and sizes are fixed by PROTOCOL.md.
 * Written field-by-field into the frame, so struct padding does not matter. */
typedef struct {
    uint32_t timestamp_ms;
    float    gain_db;
    float    rms_in_dbfs;
    float    rms_out_dbfs;
    uint32_t worst_block_cycles;
    uint32_t dma_overruns;
    uint32_t adc_overruns;
    uint32_t stream_drops;
    uint32_t blocks_processed;
} telem_t;

/* CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, no reflection, no final xor. */
uint16_t crc16_ccitt(const uint8_t *data, size_t len);

/* Build an audio frame into out[].
 *   in, out : BLOCK_SAMPLES samples each, interleaved as in0,out0,in1,out1,...
 *   returns : bytes written (AUDIO_FRAME_BYTES), or 0 if out_cap is too small.
 */
size_t frame_audio(uint8_t *out_buf, size_t out_cap, uint32_t seq,
                   const int16_t *in, const int16_t *out);

/* Build a telemetry frame into out_buf[].
 *   returns : bytes written (TELEM_FRAME_BYTES), or 0 if out_cap is too small. */
size_t frame_telem(uint8_t *out_buf, size_t out_cap, const telem_t *t);

#endif /* FRAMING_H */
