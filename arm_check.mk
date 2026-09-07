# Cross-compile check: proves every source builds clean for the target.
# Compiles only - does not link or run.
#
#   make -f arm_check.mk armcheck
#
# No CMSIS, no HAL, no device header, no include path beyond inc/.
# Requires only arm-none-eabi-gcc.

CROSS    ?= arm-none-eabi-
ARMCC    := $(CROSS)gcc
MCU      := -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard
ARMFLAGS := -std=c99 -Wall -Wextra -Wpedantic -Werror -O2 -ffunction-sections \
            -DUSE_DWT_CYCCNT -Iinc
ARM_SRC  := src/framing.c src/ringbuf.c src/instrument.c src/fir_bandpass.c \
            test/arm_smoke.c

.PHONY: armcheck
armcheck:
	@mkdir -p build/arm
	@for f in $(ARM_SRC); do \
	  echo "  CC(arm) $$f"; \
	  $(ARMCC) $(MCU) $(ARMFLAGS) -c $$f -o build/arm/$$(basename $$f .c).o || exit 1; \
	done
	@echo "ARM CROSS-COMPILE CLEAN"
	@$(CROSS)size build/arm/*.o
