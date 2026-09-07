# agc

Real-time automatic gain control on a NUCLEO-G474RE with a MAX4466 microphone
module. See `README.txt` for the provided library and `docs/` for the brief.

## Status

Capture and stream. The ADC runs at 8 kHz under a hardware timer with DMA into
a ping-pong buffer; each block is framed by the provided library and sent to
the host over the ST-LINK virtual com port at 921600 baud, with a status record
20 times a second. `host/host_receive.py` works against it as-is.

The signal is not processed yet: the filter, the gain stage and the AGC loop
are not written, so both channels of the capture carry the same raw samples and
`gain_db` / `rms_in_dbfs` / `rms_out_dbfs` report zero. The counters are real.

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
make test       # the provided native tests, unchanged
make clean
```

`make -f arm_check.mk armcheck` still cross-compiles the provided library on its
own, as it did before.

## Layout

```
inc/   framing.h ringbuf.h instrument.h   provided library
       assert_custom.h                    trap macros
       stm32g4xx_hal_conf.h               HAL module selection
src/   framing.c ringbuf.c instrument.c   provided library
       main.c                             clocks, peripherals, capture loop
       interrupt.c                        vector handlers
       syscalls.c                         the two libc hooks the startup needs
       linker.ld
host/  host_receive.py                    provided receiver
```

## Ground rules held to

- Bare metal, no RTOS.
- No dynamic allocation. There is no `_sbrk`, and the firmware does not link
  libnosys, so anything reaching for `malloc` fails at link time rather than
  quietly acquiring a heap. `arm-none-eabi-nm build/firmware/firmware.elf`
  shows no allocator in the image.
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
- **Aliasing.** Worth flagging: at 8 kHz Nyquist is 4 kHz, and there is no
  analogue anti-alias filter between the MAX4466 and the ADC. The high tone in
  the test track folds straight into the passband - 15 kHz lands at 1 kHz - and
  no digital filter can undo that. The fix is an RC low-pass on the ADC input,
  or ADC hardware oversampling, or both. Not addressed yet.
