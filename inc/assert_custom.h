/**
 * @file assert_custom.h
 *
 * @brief AGC firmware assertion macros.
 *
 *  A failed check parks the CPU with the file and line recorded in globals,
 *  so a debugger says where without the firmware carrying a formatter.
 *  Counters and timing are reported over the wire instead, in the telemetry
 *  record that framing.h defines.
 */

#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

[[noreturn]] void trap_error(const char* file, uint32_t line);

void assert_failed(uint8_t* file, uint32_t line); // stm32 HAL assert

#ifdef USE_FULL_ASSERT
  #define ASSERT(expr) ((expr) ? (void)0U : trap_error(__FILE__, __LINE__))
#else
  #define ASSERT(expr) ((void)0U)
#endif

#define ERROR_CHECK(x) { \
  if (0 != (x)) { \
    trap_error(__FILE__, __LINE__); \
  } \
}

#ifdef __cplusplus
}
#endif
