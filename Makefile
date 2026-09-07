# Native tests for the portable parts of the starter repo.
# The firmware itself is built with your STM32 toolchain, not this Makefile.

CC      ?= gcc
CFLAGS  ?= -std=c99 -Wall -Wextra -Wpedantic -Werror -O2 -Iinc
SRC      = src/framing.c src/ringbuf.c src/instrument.c
TEST_SRC = test/test_starter.c

.PHONY: test clean

test: build/test_starter
	@cd build && ./test_starter
	@python3 test/test_roundtrip.py build/stream.bin

build/test_starter: $(SRC) $(TEST_SRC) | build
	$(CC) $(CFLAGS) $(SRC) $(TEST_SRC) -o $@

build:
	@mkdir -p build

clean:
	@rm -rf build
