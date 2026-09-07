# AGC firmware for the NUCLEO-G474RE, plus the native tests for the portable
# parts of the provided library.
#
#   make            build the firmware image
#   make flash      build and flash it with openocd
#   make test       run the native tests (needs gcc and python3 with numpy)
#   make clean      remove the build directory

.PHONY: all firmware flash test clean

all: firmware

# ------------------------------------------------------------------ firmware

CROSS      ?= arm-none-eabi-
FW_CC      := $(CROSS)gcc
FW_OBJCOPY := $(CROSS)objcopy
FW_SIZE    := $(CROSS)size

CUBE  := third-party/github/STM32CubeG4
CMSIS := $(CUBE)/Drivers/CMSIS
HAL   := $(CUBE)/Drivers/STM32G4xx_HAL_Driver

FW_BUILD := build/firmware
FW_ELF   := $(FW_BUILD)/firmware.elf
FW_BIN   := $(FW_BUILD)/firmware.bin
FW_LD    := src/linker.ld

# Ours. Built with warnings as errors.
FW_SRC := src/main.c src/interrupt.c src/syscalls.c \
          src/framing.c src/ringbuf.c src/instrument.c \
          src/fir_bandpass.c src/agc.c

# Third-party. Does not compile clean under -Werror, so it is built without it.
FW_TP_SRC := \
  $(CMSIS)/Device/ST/STM32G4xx/Source/Templates/system_stm32g4xx.c \
  $(HAL)/Src/stm32g4xx_hal.c \
  $(HAL)/Src/stm32g4xx_hal_adc.c \
  $(HAL)/Src/stm32g4xx_hal_adc_ex.c \
  $(HAL)/Src/stm32g4xx_hal_cortex.c \
  $(HAL)/Src/stm32g4xx_hal_dma.c \
  $(HAL)/Src/stm32g4xx_hal_dma_ex.c \
  $(HAL)/Src/stm32g4xx_hal_gpio.c \
  $(HAL)/Src/stm32g4xx_hal_pwr_ex.c \
  $(HAL)/Src/stm32g4xx_hal_rcc.c \
  $(HAL)/Src/stm32g4xx_hal_rcc_ex.c \
  $(HAL)/Src/stm32g4xx_hal_tim.c \
  $(HAL)/Src/stm32g4xx_hal_tim_ex.c \
  $(HAL)/Src/stm32g4xx_hal_uart.c \
  $(HAL)/Src/stm32g4xx_hal_uart_ex.c

FW_ASM := $(CMSIS)/Device/ST/STM32G4xx/Source/Templates/gcc/startup_stm32g474xx.s

FW_MCU  := -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard
FW_DEFS := -DSTM32G474xx -DUSE_HAL_DRIVER -DUSE_FULL_ASSERT -DUSE_DWT_CYCCNT
FW_INC  := -Iinc -I$(HAL)/Inc -I$(HAL)/Inc/Legacy \
           -I$(CMSIS)/Device/ST/STM32G4xx/Include -I$(CMSIS)/Include

FW_OPT   ?= -O2
FW_WARN  := -Wall -Wextra -Wpedantic -Werror
# -std=c2x for __VA_ARGS__ with no arguments in DEBUG_PRINTF. Requires GCC 13 or later.
FW_CFLAGS := -std=c2x $(FW_MCU) $(FW_DEFS) $(FW_INC) $(FW_OPT) -g3 \
             -ffunction-sections -fdata-sections --specs=nano.specs -MMD -MP

# No libnosys: the firmware makes no system calls, so anything reaching for
# one - malloc included - fails to link rather than quietly getting a heap.
# --gc-sections needs -ffunction-sections and -fdata-sections to be effective.
FW_LDFLAGS := $(FW_MCU) $(FW_OPT) -T $(FW_LD) \
              --specs=nano.specs -nostartfiles \
              -Wl,--gc-sections -Wl,-Map=$(FW_BUILD)/firmware.map

# The filter design needs sin, cos and pow, the AGC setup needs pow and exp, and
# the telemetry needs log10. Newlib's libm neither allocates nor makes system
# calls, so this does not undo the no-heap property above; check it with nm if
# you change it. Must follow the objects on the link line.
FW_LIBS := -lm

FW_OBJ    := $(FW_SRC:%.c=$(FW_BUILD)/%.o) $(FW_ASM:%.s=$(FW_BUILD)/%.o)
FW_TP_OBJ := $(FW_TP_SRC:%.c=$(FW_BUILD)/%.o)

firmware: $(FW_BIN)

$(FW_BIN): $(FW_ELF)
	$(FW_OBJCOPY) -O binary $< $@
	@$(FW_SIZE) $(FW_ELF)

$(FW_ELF): $(FW_OBJ) $(FW_TP_OBJ) $(FW_LD)
	@mkdir -p $(dir $@)
	$(FW_CC) $(FW_LDFLAGS) $(FW_OBJ) $(FW_TP_OBJ) $(FW_LIBS) -o $@

$(FW_TP_OBJ): $(FW_BUILD)/%.o: %.c $(HAL)/Src
	@mkdir -p $(dir $@)
	$(FW_CC) $(FW_CFLAGS) -c $< -o $@

$(FW_BUILD)/%.o: %.c
	@mkdir -p $(dir $@)
	$(FW_CC) $(FW_CFLAGS) $(FW_WARN) -c $< -o $@

$(FW_BUILD)/%.o: %.s
	@mkdir -p $(dir $@)
	$(FW_CC) $(FW_MCU) -c $< -o $@

# The Cube package keeps its drivers in nested submodules of its own.
$(HAL)/Src:
	git -C $(CUBE) submodule update --init \
	  Drivers/STM32G4xx_HAL_Driver Drivers/CMSIS/Device/ST/STM32G4xx

OPENOCD     ?= openocd
OPENOCD_CFG ?= board/st_nucleo_g4.cfg

flash: $(FW_ELF)
	$(OPENOCD) -f $(OPENOCD_CFG) -c "program $(FW_ELF) verify reset exit"

-include $(FW_OBJ:.o=.d) $(FW_TP_OBJ:.o=.d)

# --------------------------------------------------------------- native tests

CC      ?= gcc
CFLAGS  ?= -std=c99 -Wall -Wextra -Wpedantic -Werror -O2 -Iinc
SRC      = src/framing.c src/ringbuf.c src/instrument.c
TEST_SRC = test/test_starter.c

test: build/test_starter build/test_fir_bandpass build/test_agc
	@cd build && ./test_starter
	@python3 test/test_roundtrip.py build/stream.bin
	@./build/test_fir_bandpass
	@./build/test_agc

build/test_starter: $(SRC) $(TEST_SRC) | build
	$(CC) $(CFLAGS) $(SRC) $(TEST_SRC) -o $@

build/test_fir_bandpass: src/fir_bandpass.c test/test_fir_bandpass.c | build
	$(CC) $(CFLAGS) src/fir_bandpass.c test/test_fir_bandpass.c -o $@ -lm

build/test_agc: src/agc.c test/test_agc.c | build
	$(CC) $(CFLAGS) src/agc.c test/test_agc.c -o $@ -lm

build:
	@mkdir -p build

clean:
	@rm -rf build
