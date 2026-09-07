#!/usr/bin/env python3
"""Round-trip test: decode the C-generated stream with the real host receiver.

This is the check that matters. It proves the firmware's frame encoder and the
host's decoder agree byte for byte, so neither you nor the candidate has to
debug the protocol.

    make test        (runs this automatically)
    python test/test_roundtrip.py stream.bin
"""
import struct
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "host"))
from host_receive import Deframer, TYPE_AUDIO, TYPE_TELEM, crc16_ccitt  # noqa: E402

fails = []


def check(cond, msg):
    print(f"  {'ok   ' if cond else 'FAIL '} {msg}")
    if not cond:
        fails.append(msg)


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else "stream.bin"
    data = open(path, "rb").read()
    print(f"Python round-trip against {path} ({len(data)} bytes)")

    # Python's CRC must agree with the C one on the canonical vector.
    check(crc16_ccitt(b"123456789") == 0x29B1, "Python CRC check value matches C")

    d = Deframer()
    audio, telem = [], []
    # Feed in awkward chunk sizes so frames straddle boundaries, as they do on a
    # real serial port.
    pos, n = 0, 0
    for chunk_size in (1, 7, 263, 64, 4096):
        while pos < len(data):
            chunk = data[pos:pos + chunk_size]
            pos += len(chunk)
            for ftype, payload in d.feed(chunk):
                n += 1
                if ftype == TYPE_AUDIO:
                    audio.append(payload)
                elif ftype == TYPE_TELEM:
                    telem.append(payload)
            if pos >= len(data):
                break

    check(len(audio) == 50, f"decoded 50 audio frames (got {len(audio)})")
    check(len(telem) == 5, f"decoded 5 telemetry frames (got {len(telem)})")
    check(d.crc_errors == 0, f"no CRC errors (got {d.crc_errors})")
    check(d.resyncs == 0, f"no resyncs needed (got {d.resyncs})")

    # Sequence numbers must be contiguous 0..49.
    seqs = [struct.unpack_from("<I", p, 0)[0] for p in audio]
    check(seqs == list(range(50)), "sequence numbers are 0..49 in order")

    # Sample values must match what the C side generated.
    ok = True
    for seq, p in enumerate(audio):
        s = struct.unpack_from("<128h", p, 4)
        for i in range(64):
            want_in = ((seq * 64 + i) * 7 - 20000) & 0xFFFF
            want_in = want_in - 0x10000 if want_in > 0x7FFF else want_in
            want_out = -(seq * 3 + i)
            want_out = ((want_out + 0x10000) & 0xFFFF)
            want_out = want_out - 0x10000 if want_out > 0x7FFF else want_out
            if s[2 * i] != want_in or s[2 * i + 1] != want_out:
                ok = False
                break
        if not ok:
            break
    check(ok, "every interleaved sample matches the C-side values")

    # Telemetry fields, including the float encoding.
    t0 = struct.unpack("<IfffIIIII", telem[0])
    check(t0[0] == 5 * 8, "telemetry uint32 field decodes")
    check(abs(t0[1] - (12.5 + 5)) < 1e-6, "telemetry float32 gain decodes")
    check(abs(t0[2] + 33.25) < 1e-6, "negative float decodes")
    check(t0[4] == 412345 + 5, "worst_block_cycles decodes")
    check(t0[7] == 5, "stream_drops decodes")

    # Corruption must be caught, not silently accepted.
    bad = bytearray(data)
    bad[400] ^= 0xFF
    d2 = Deframer()
    got = sum(1 for _ in d2.feed(bytes(bad)))
    check(d2.crc_errors > 0, "a flipped byte is caught by CRC")
    check(got < 55, "corrupted frame is rejected, not passed through")

    # Garbage before the first frame must not prevent syncing.
    d3 = Deframer()
    got3 = list(d3.feed(b"\x00\xFF\xA5garbage\x5A\x01" + data))
    check(len(got3) >= 54, "recovers sync after leading garbage")

    print()
    if fails:
        print("ROUND-TRIP FAILED:")
        for f in fails:
            print("  -", f)
        return 1
    print("ROUND-TRIP PASSED — C encoder and Python decoder agree")
    return 0


if __name__ == "__main__":
    sys.exit(main())
