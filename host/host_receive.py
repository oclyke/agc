#!/usr/bin/env python3
"""
Host-side receiver for the AGC take-home.

Reads framed data from the board's virtual COM port and writes:
  stereo_capture.wav  channel 1 = input, channel 2 = AGC output
  telemetry.csv       one row per telemetry frame

Also measures the true sample rate by timestamping frame arrivals against the host
clock. (Timing the board's samples with the board's own timer measures its clock
against itself and always reads exactly nominal.)

Usage:
    python host_receive.py --port /dev/ttyACM0 --seconds 30
    python host_receive.py --port COM4 --seconds 60 --prefix run2

Requires: pyserial, numpy
"""

import argparse
import csv
import struct
import sys
import time
import wave

import numpy as np

SYNC = b"\xA5\x5A"
TYPE_AUDIO = 0x01
TYPE_TELEM = 0x02
AUDIO_PAYLOAD = 260
TELEM_PAYLOAD = 36
SAMPLES_PER_BLOCK = 64
NOMINAL_FS = 8000

TELEM_FIELDS = [
    "timestamp_ms", "gain_db", "rms_in_dbfs", "rms_out_dbfs",
    "worst_block_cycles", "dma_overruns", "adc_overruns",
    "stream_drops", "blocks_processed",
]


def crc16_ccitt(data: bytes) -> int:
    """CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, no reflection, no final xor."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


class Deframer:
    """Incremental byte-stream deframer. Resynchronises after corruption."""

    def __init__(self):
        self.buf = bytearray()
        self.crc_errors = 0
        self.resyncs = 0

    def feed(self, chunk: bytes):
        self.buf.extend(chunk)
        while True:
            frame = self._try_parse()
            if frame is None:
                return
            yield frame

    def _try_parse(self):
        idx = self.buf.find(SYNC)
        if idx < 0:
            # Keep one byte in case a sync word straddles the chunk boundary.
            if len(self.buf) > 1:
                del self.buf[:-1]
            return None
        if idx > 0:
            self.resyncs += 1
            del self.buf[:idx]

        if len(self.buf) < 5:
            return None

        ftype = self.buf[2]
        length = struct.unpack_from("<H", self.buf, 3)[0]

        if length > 1024:  # implausible; treat this sync as spurious
            del self.buf[:2]
            self.resyncs += 1
            return None

        total = 5 + length + 2
        if len(self.buf) < total:
            return None

        payload = bytes(self.buf[5:5 + length])
        want = struct.unpack_from("<H", self.buf, 5 + length)[0]
        got = crc16_ccitt(bytes(self.buf[2:5 + length]))

        if want != got:
            self.crc_errors += 1
            del self.buf[:2]
            return None

        del self.buf[:total]
        return ftype, payload


def monitor(args):
    """Live input level, for setting the microphone trim pot.

    Play the opening tone of your test track and adjust the pot until the peak
    sits a few dB below full scale - loud, but never touching 0 dBFS. Then leave
    it alone for the rest of the exercise.
    """
    import serial

    print(f"Monitoring {args.port} at {args.baud}. Ctrl-C to stop.\n")
    print("Play the calibration tone at the start of your track and adjust the")
    print("trim pot until peak sits around -3 dBFS. Never let it reach 0.\n")

    d = Deframer()
    buf = []
    t_last = time.monotonic()
    with serial.Serial(args.port, args.baud, timeout=0.1) as ser:
        ser.reset_input_buffer()
        try:
            while True:
                chunk = ser.read(8192)
                if chunk:
                    for ftype, payload in d.feed(chunk):
                        if ftype == TYPE_AUDIO and len(payload) == AUDIO_PAYLOAD:
                            buf.append(np.frombuffer(payload, dtype="<i2", offset=4))
                now = time.monotonic()
                if now - t_last >= 0.5 and buf:
                    a = np.concatenate(buf).reshape(-1, 2)[:, 0].astype(np.float64)
                    buf = []
                    peak = np.max(np.abs(a)) / 32768.0
                    rms = np.sqrt(np.mean(a ** 2)) / 32768.0
                    pk_db = 20 * np.log10(peak + 1e-9)
                    rms_db = 20 * np.log10(rms + 1e-9)
                    bars = int(max(0, min(40, (pk_db + 60) * 40 / 60)))
                    flag = "  CLIPPING" if peak >= 0.999 else ""
                    print(f"\rpeak {pk_db:6.1f} dBFS   rms {rms_db:6.1f} dBFS  "
                          f"|{'#' * bars}{' ' * (40 - bars)}|{flag}   ", end="")
                    t_last = now
        except KeyboardInterrupt:
            print("\n\nStopped.")
    return 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True, help="serial port, e.g. /dev/ttyACM0 or COM4")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--seconds", type=float, default=30.0)
    ap.add_argument("--prefix", default="")
    ap.add_argument("--monitor", action="store_true",
                    help="print the live input level and stop; use this to set "
                         "the microphone trim pot. Writes no files.")
    args = ap.parse_args()

    p = (args.prefix + "_") if args.prefix else ""
    wav_path = f"{p}stereo_capture.wav"
    csv_path = f"{p}telemetry.csv"

    audio = []          # list of (128,) int16 arrays
    telemetry = []
    seq_expected = None
    seq_gaps = 0
    blocks_missing = 0

    deframer = Deframer()
    first_audio_t = None
    last_audio_t = None

    # Imported here, not at module scope, so the Deframer can be unit-tested
    # without pyserial installed.
    import serial

    if args.monitor:
        return monitor(args)

    print(f"Opening {args.port} at {args.baud}...")
    with serial.Serial(args.port, args.baud, timeout=0.1) as ser:
        ser.reset_input_buffer()
        t_end = time.monotonic() + args.seconds
        try:
            while time.monotonic() < t_end:
                chunk = ser.read(8192)
                if not chunk:
                    continue
                now = time.monotonic()
                for ftype, payload in deframer.feed(chunk):
                    if ftype == TYPE_AUDIO and len(payload) == AUDIO_PAYLOAD:
                        seq = struct.unpack_from("<I", payload, 0)[0]
                        if seq_expected is not None and seq != seq_expected:
                            seq_gaps += 1
                            blocks_missing += (seq - seq_expected) & 0xFFFFFFFF
                        seq_expected = (seq + 1) & 0xFFFFFFFF
                        audio.append(np.frombuffer(payload, dtype="<i2", offset=4))
                        if first_audio_t is None:
                            first_audio_t = now
                        last_audio_t = now
                    elif ftype == TYPE_TELEM and len(payload) == TELEM_PAYLOAD:
                        vals = struct.unpack("<IfffIIIII", payload)
                        telemetry.append(dict(zip(TELEM_FIELDS, vals)))
        except KeyboardInterrupt:
            print("\nInterrupted, writing what we have.")

    if not audio and not telemetry:
        print("Nothing received. Check baud rate, port, and framing.")
        return 1

    if not audio:
        # Telemetry-only is the normal case; audio streaming is a stretch item.
        with open(csv_path, "w", newline="") as f:
            wr = csv.DictWriter(f, fieldnames=TELEM_FIELDS)
            wr.writeheader()
            wr.writerows(telemetry)
        print(f"\nWrote {csv_path}  ({len(telemetry)} rows)")
        span = (telemetry[-1]["timestamp_ms"] - telemetry[0]["timestamp_ms"]) / 1000.0
        rate = (len(telemetry) - 1) / span if span > 0 else 0.0
        print(f"  duration    : {span:.1f} s")
        print(f"  telemetry   : {rate:.0f} Hz")
        print(f"  CRC errors  : {deframer.crc_errors}")
        last = telemetry[-1]
        print("  final counters:")
        for k in ("dma_overruns", "adc_overruns", "stream_drops",
                  "worst_block_cycles", "blocks_processed"):
            print(f"    {k:<20}: {int(last[k])}")
        if last["worst_block_cycles"] == 0:
            print("    WARNING: worst_block_cycles is 0 - your DWT counter is "
                  "not running. Check dwt_init() returned true.")
        return 0

    stereo = np.concatenate(audio).reshape(-1, 2)

    # True sample rate, board crystal vs host clock.
    n_blocks = len(audio)
    fs_measured = float("nan")
    if n_blocks > 1 and last_audio_t > first_audio_t:
        elapsed = last_audio_t - first_audio_t
        fs_measured = (n_blocks - 1) * SAMPLES_PER_BLOCK / elapsed

    with wave.open(wav_path, "wb") as w:
        w.setnchannels(2)
        w.setsampwidth(2)
        w.setframerate(NOMINAL_FS)
        w.writeframes(stereo.astype("<i2").tobytes())

    if telemetry:
        with open(csv_path, "w", newline="") as f:
            wr = csv.DictWriter(f, fieldnames=TELEM_FIELDS)
            wr.writeheader()
            wr.writerows(telemetry)

    dur = len(stereo) / NOMINAL_FS
    print(f"\nWrote {wav_path}  ({len(stereo)} frames, {dur:.1f} s)")
    print(f"Wrote {csv_path}  ({len(telemetry)} rows)")
    print()
    print(f"  audio blocks received : {n_blocks}")
    print(f"  sequence gaps         : {seq_gaps}  ({blocks_missing} blocks missing)")
    print(f"  CRC errors            : {deframer.crc_errors}")
    print(f"  resyncs               : {deframer.resyncs}")
    print(f"  measured sample rate  : {fs_measured:.2f} Hz "
          f"({(fs_measured / NOMINAL_FS - 1) * 1e6:+.0f} ppm vs nominal)")

    if telemetry:
        last = telemetry[-1]
        print()
        print("  final counters:")
        for k in ("dma_overruns", "adc_overruns", "stream_drops",
                  "worst_block_cycles", "blocks_processed"):
            print(f"    {k:<20}: {last[k]}")
        budget = 170e6 / (NOMINAL_FS / SAMPLES_PER_BLOCK)
        print(f"    block budget @170MHz: {budget:.0f} cycles "
              f"({last['worst_block_cycles'] / budget * 100:.1f}% used)")

    if seq_gaps or deframer.crc_errors:
        print("\n  NOTE: gaps or CRC errors present. Sequence gaps mean the board "
              "dropped blocks; CRC errors usually mean the link is the problem.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
