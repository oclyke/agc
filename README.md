# agc

Real-time automatic gain control on a NUCLEO-G474RE with a MAX4466 microphone
module. See `README.txt` for the provided library and `docs/` for the brief.

## Status

Capture and stream. The ADC runs at 8 kHz under a hardware timer with DMA into
a ping-pong buffer; each block is framed by the provided library and sent to
the host over the ST-LINK virtual com port at 921600 baud, with a status record
20 times a second. `host/host_receive.py` works against it as-is.

Capture, filter, level and stream. The whole chain is in place: channel 1 of
`stereo_capture.wav` is the raw capture and channel 2 is the bandpassed signal
with the AGC's gain on it, delay-matched so the two line up sample for sample.
The loop holds the output at -20 dBFS RMS, drops the gain the instant a block
asks it to and takes 50 ms to bring it back, and clamps what reaches the
samples to -20 .. +40 dB. Every field in the telemetry record is now real, and
`worst_block_cycles` covers the filter and the loop together.

## Hardware

MAX4466 module | NUCLEO-G474RE
---|---
VCC | 3V3
GND | GND
OUT | A0 (PA0)

The MAX4466 idles at about VCC/2. That offset is removed in hardware by the
ADC's offset unit rather than in software - see `src/main.c`.

Pin map, and `inc/board.h`:

name | pin | function
---|---|---
MIC | PA0 | MAX4466 OUT, ADC1_IN1, Arduino A0
LED | PA5 | LD2, capture heartbeat at 0.5 Hz, Arduino D13
SCOPE | PB5 | sample clock, one edge per sample, Arduino D4
VCP TX | PA2 | LPUART1_TX to the ST-LINK
VCP RX | PA3 | LPUART1_RX from the ST-LINK

## Receiving

```sh
pip install pyserial numpy
python host/host_receive.py --port /dev/cu.usbmodem* --seconds 30
```

921600 baud is the host script's default. If the ST-LINK will not hold it, drop
`VCP_BAUDRATE` in `inc/board.h` and pass `--baud` to match.

## Building

The STM32CubeG4 package is a submodule and keeps its drivers in nested
submodules of its own. `make` checks them out on first use; to do it by hand:

```sh
git submodule update --init third-party/github/STM32CubeG4
git -C third-party/github/STM32CubeG4 submodule update --init \
  Drivers/STM32G4xx_HAL_Driver Drivers/CMSIS/Device/ST/STM32G4xx
```

Then, with `arm-none-eabi-gcc` 13 or later on the path:

```sh
make            # build/firmware/firmware.elf and .bin
make flash      # program it with openocd
make test       # native tests: the provided ones, the filter's, the loop's
make clean
```

`make -f arm_check.mk armcheck` cross-compiles the portable sources on their
own - the provided library as it did before, and the filter and the AGC
alongside it - with no CMSIS, no HAL and no include path beyond `inc/`.

## Layout

```
inc/   framing.h ringbuf.h instrument.h   provided library
       fir_bandpass.h                     bandpass filter
       agc.h                              the gain stage and the control loop
       board.h                            pin map
       assert_custom.h                    trap macros
       stm32g4xx_hal_conf.h               HAL module selection
src/   framing.c ringbuf.c instrument.c   provided library
       fir_bandpass.c                     bandpass filter
       agc.c                              the gain stage and the control loop
       main.c                             clocks, peripherals, capture loop
       interrupt.c                        vector handlers
       syscalls.c                         the two libc hooks the startup needs
       linker.ld
test/  test_starter.c                     provided native tests
       test_fir_bandpass.c                filter response and streaming tests
       test_agc.c                         loop dynamics, clamps and timing
host/  host_receive.py                    provided receiver
```

## Ground rules held to

- Bare metal, no RTOS.
- No dynamic allocation. There is no `_sbrk`, and the firmware does not link
  libnosys, so anything reaching for `malloc` fails at link time rather than
  quietly acquiring a heap. `arm-none-eabi-nm build/firmware/firmware.elf`
  shows no allocator in the image. The filter needs `libm` for its design
  step, the AGC for its setup and the telemetry for its logarithms, all of
  which were checked the same way: newlib's math brings no heap with it.
- Peripheral setup uses the HAL; the DMA and timing paths are written directly.

## Notes

- **Clock.** HSI16 / 4 x 85 / 2 = 170 MHz, the part's maximum, with the
  regulator in boost mode as required above 150 MHz. The board ships without a
  crystal populated for HSE, so the internal oscillator is the source; its
  ~1% tolerance shows up directly as sample rate error, which
  `host_receive.py` will measure against the laptop clock.
- **Sample rate.** APB1 timer clock is 170 MHz, so TIM6 divides by exactly
  21250 for 8 kHz with no rounding error. `main.c` asserts the division is
  exact rather than assuming it.
- **Instrumentation.** No debug console. Counters and block timing live in
  `g_counters` (`inc/instrument.h`) and reach the host in the telemetry record
  that `inc/framing.h` defines, which `host_receive.py` writes to
  `telemetry.csv`. If a check trips, the firmware parks with `trap_file` and
  `trap_line` set - read those first under a debugger.

  The three levels in that record all describe the same block, the one the AGC
  last emitted. `rms_in_dbfs` is measured at the loop's own measurement point -
  after the filter, before the gain - rather than on channel 1, which still
  carries the rumble, the alias and the residual DC; measured there,
  `rms_out_dbfs - rms_in_dbfs` is `gain_db` in the steady state. The three
  separate when the loop is moving, because the block was scaled by a ramp
  rather than by a constant and `gain_db` is only where that ramp finished.
- **Sample clock on a pin.** PB5 toggles in the TIM6 update interrupt, so a
  scope on it shows a 4 kHz square wave whose edges are the sample instants
  (the conversion starts a few ADC clocks after the edge and takes ~6 us).
  This is the only thing in the firmware that costs a CPU interrupt per
  sample - 8000 a second against the 250 the DMA capture path needs, about
  0.2% of the CPU. Comment out `SCOPE_SAMPLE_CLOCK` in `src/main.c` for runs
  where the timing numbers have to be untainted.
- **Link budget.** 125 audio frames a second at 267 bytes plus 20 status
  records at 43 is 34.2 kB/s. 921600 baud 8N1 carries 92.2 kB/s, so the stream
  sits at 37% of the link.
- **Bandpass filter.** `inc/fir_bandpass.h` is a linear-phase FIR designed at
  run time: the tap count is fixed at compile time, the band is not, so one
  build can carry several filters or be retuned without a reflash. Nothing in
  it allocates. For the project's band - 300 Hz to 3000 Hz at 8 kHz, 150 Hz
  skirts, 60 dB down - it takes 217 taps and measures a flat passband to
  within 0.01 dB with the worst stopband point at -65.5 dB.

  The textbook Kaiser tap count for that spec is 195, and 195 taps measures
  -58.8 dB, not -60: Kaiser's estimate is derived for a lowpass, and a
  bandpass has two error terms that can add. The library designs for 6 dB more
  than asked to cover that, which is where 217 comes from.

- **AGC.** `inc/agc.h`, block-rate, driven by the RMS of each block of the
  filtered signal. It holds the output at -20 dBFS with the gain that reaches
  the samples clamped to -20 .. +40 dB, both from the brief.

  Three things about the shape of it are worth stating.

  *It holds a block back.* The gain that a block's ramp ends on is the gain the
  block AFTER it asked for, so the loop has 8 ms of lookahead: a reduction has
  finished ramping by the time the loud audio that caused it arrives. That is
  the extra 64 samples in the delay line above.

  *Down at once, up over 50 ms.* The brief suggests something like 10 ms down
  and 150 ms up. Reductions here take no time at all - with the lookahead there
  is nothing to be gained by smoothing them - and recoveries follow
  `g += a*(target - g)`. `a` is a per-sample coefficient, `1 - exp(-1/(tau*fs))`,
  which is 0.002496 for 50 ms at 8 kHz. The gain is a per-block quantity, so
  applying that number once a block would give a time constant 64 times too
  long, 3.2 seconds; `agc_init()` folds a whole block of the recursion into one
  coefficient, `1 - (1-a)^64 = 0.1478`, instead. `test_agc.c` does not take
  that on trust - it recovers the time constant from the measured gain
  trajectory and gets 50.1 ms, and separately compares the whole run against a
  reference that steps the recursion one sample at a time.

  *The gain never steps.* Every sample gets its own gain, one linear
  interpolation step apart, and the line is continuous across block boundaries
  because each block starts from the value the last one ended on. The position
  along the line is computed per sample rather than accumulated: accumulating
  is one instruction cheaper and drifts, and on a ramp starting high and ending
  low the drift reached a few parts in ten thousand of the endpoint, which is a
  step in the gain at every block boundary. Measured, and now asserted at a
  single rounding.

  What the clamp does not do is bound the gain STATE - only what multiplies the
  samples. On silence the state releases up past the ceiling while the applied
  gain sits pinned on it. That needs no anti-windup because the attack is
  immediate: the first block of real audio asks for less gain than the state
  holds, so the state lands on the new value in one block from wherever it had
  drifted to, rather than having to unwind.

  The one place the lookahead costs rather than pays is a sudden drop in level:
  the block being emitted is still the loud one, and its ramp is aimed at the
  much higher gain the quiet block behind it asked for. It is bounded - one
  release step, so at most 14.8% of the way to the new gain - and it lasts
  exactly one block. On a 39 dB instantaneous step, which no microphone will
  produce, one 8 ms block comes out at -1.3 dBFS. It saturates rather than
  wrapping.

- **Delay matching.** Two delays have to be made up on the raw channel, not
  one. The filter delays every frequency alike by 108 samples, 13.5 ms, and the
  AGC holds a block back so its ramp can be aimed by the block that follows,
  which is another 64. `main.c` runs the raw channel through a delay line of
  exactly 172 samples before framing it, so the two channels of
  `stereo_capture.wav` describe the same instant - otherwise comparing them, by
  ear or by the host or by a level detector, compares two different moments.

  Both halves are measured rather than assumed. `test_fir_bandpass.c` sweeps
  the shift and checks that 108 is where an in-band signal reconstructs to
  -78 dB and that 107 and 109 are both far worse; `test_agc.c` checks that what
  comes out of the loop is the previous block's samples and not this one's. An
  off-by-one in either cannot pass quietly.

- **Clipping.** `main.c` saturates the output channel rather than letting it
  wrap, so an overload sounds like clipping instead of like noise, and counts
  the samples it had to hold at a rail in `output_clips`. There is no field for
  that count in the telemetry record - `PROTOCOL.md` fixes the layout - so read
  it under a debugger, or look for the flat tops in channel 2.

  Two things can push a sample past the rail. The filter is unity gain to a
  tone in the passband, but its worst-case gain to an arbitrary waveform is the
  sum of the magnitudes of its coefficients, which for this band is 2.92, about
  9.3 dB above full scale; real audio does not go near that. The AGC is the
  likelier source, because it aims its gain at an RMS level and any crest
  factor at all puts peaks above the target.

- **Aliasing.** Worth flagging: at 8 kHz Nyquist is 4 kHz, and there is no
  analogue anti-alias filter between the MAX4466 and the ADC. The high tone in
  the test track folds straight into the passband - 15 kHz lands at 1 kHz - and
  no digital filter can undo that. The fix is an RC low-pass on the ADC input,
  or ADC hardware oversampling, or both. Not addressed yet.
