AGC challenge - provided library
================================

Everything here is written and tested. You should not need to change any of it.

  make test        run the tests (needs gcc and python3 with numpy)

Both suites should pass in a few seconds. That confirms the frame encoder here
agrees with the host script, so the link is one less thing to worry about.

Layout
------
  inc/, src/   framing.c    packs samples into frames with a checksum
               ringbuf.c    holds frames while the UART drains them
               instrument.h cycle counter and dropped-data counters
  host/        host_receive.py   run this on your laptop
  test/        the test suites

No main.c, no project files, no peripheral setup - that part is yours. Add inc/
to your include path and src/*.c to your build. There are no dependencies: not
CMSIS, not the HAL, not a device header. Define USE_DWT_CYCCNT when building for
the board to enable the cycle counter.

Sending a block
---------------
  #include "framing.h"
  #include "ringbuf.h"

  static uint8_t   storage[8192];        /* power of two, your choice of size */
  static ringbuf_t rb;

  rb_init(&rb, storage, sizeof storage);          /* once at startup */

  uint8_t frame[AUDIO_FRAME_BYTES];               /* once per block */
  size_t n = frame_audio(frame, sizeof frame, seq++, in_block, out_block);
  if (!rb_write_all(&rb, frame, n)) {
      g_counters.stream_drops++;   /* host is behind: drop and count, never stall */
  }

Draining is yours. rb_read() copies out; rb_peek_contiguous() plus rb_consume()
hands a pointer straight to a DMA transfer without copying. Read the comment at
the top of ringbuf.h first - the single-writer/single-reader rule it relies on
is a real constraint.

Timing a block
--------------
  BLOCK_TIMER_START();
  process_block(...);
  BLOCK_TIMER_END();      /* updates g_counters.worst_cycles and .blocks */

Receiving on your laptop
------------------------
  pip install pyserial numpy
  python host/host_receive.py --port /dev/ttyACM0 --seconds 130

Writes stereo_capture.wav and telemetry.csv, and prints your measured sample
rate, any gaps, and your timing numbers. The port is usually /dev/ttyACM0 on
Linux, /dev/cu.usbmodem* on macOS, COMn on Windows.

Wire format
-----------
921600 baud, 8N1. Frames are 2-byte sync, type, length, payload, CRC-16.
framing.c does all of it; you should not need the details. If you want them,
read framing.h.

What is tested and what is not
------------------------------
Tested: framing.c and ringbuf.c, including CRC against the standard check value,
buffer wraparound over 20000 interleaved operations, backpressure, stack
painting, and full round-trip decoding by the host script. Also cross-compiled
clean for cortex-m4.

Not tested on hardware: dwt_init() returns false if the cycle counter does not
tick - check that return value. On some parts the counter only runs with a
debugger attached, and a dead counter would silently zero every timing number
you report.
