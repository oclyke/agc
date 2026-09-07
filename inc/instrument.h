/* instrument.h - measurement helpers. This is your lab equipment.
 *
 * DEPENDENCIES: none. Not CMSIS, not the STM32 HAL, not a device header. The
 * cycle counter is reached through its ARMv7-M architectural addresses, which
 * are identical on every Cortex-M3, M4 and M7. Drop this in any project and it
 * compiles.
 *
 * VERIFICATION: compiles clean for cortex-m4 under -Wall -Wextra -Wpedantic
 * -Werror with no include path beyond this directory, and the portable parts
 * are exercised by the native test suite. It has NOT been run on hardware -
 * see dwt_init() below.
 */
#ifndef INSTRUMENT_H
#define INSTRUMENT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* ---------------------------------------------------------------- counters */

typedef struct {
    uint32_t dma_overruns;   /* block arrived while the previous was still busy */
    uint32_t adc_overruns;   /* ADC peripheral overrun flag seen                */
    uint32_t stream_drops;   /* frame dropped because the ring buffer was full  */
    uint32_t blocks;         /* blocks processed                                */
    uint32_t worst_cycles;   /* worst-case block processing time                */
} counters_t;

extern volatile counters_t g_counters;

/* --------------------------------------------------------- cycle counting */
/* Enable with -DUSE_DWT_CYCCNT when building for the target. Without it these
 * become stubs, so the same sources also build natively for the tests. */
#ifdef USE_DWT_CYCCNT

/* ARMv7-M architectural addresses - same on M3/M4/M7, so no device header is
 * needed. Note that the DWT software lock register is NOT part of CMSIS's
 * DWT_Type on Cortex-M4: writing DWT->LAR is a compile error, which is why it
 * is addressed directly here. */
#define DWT_CTRL_REG   (*(volatile uint32_t *)0xE0001000u)
#define DWT_CYCCNT_REG (*(volatile uint32_t *)0xE0001004u)
#define DWT_LAR_REG    (*(volatile uint32_t *)0xE0001FB0u)
#define SCB_DEMCR_REG  (*(volatile uint32_t *)0xE000EDFCu)

#define DEMCR_TRCENA_BIT   (1uL << 24)
#define DWT_CTRL_CYCCNTENA (1uL << 0)

/* Call once at startup.
 *
 * Returns false if the counter does not tick. CHECK THIS. On some parts CYCCNT
 * only runs with a debugger attached; a silently dead counter reads zero
 * forever and would make every timing number you report a zero. */
static inline bool dwt_init(void)
{
    SCB_DEMCR_REG |= DEMCR_TRCENA_BIT;
    DWT_LAR_REG = 0xC5ACCE55uL;   /* harmless where the lock is not implemented */
    DWT_CYCCNT_REG = 0uL;
    DWT_CTRL_REG |= DWT_CTRL_CYCCNTENA;

    uint32_t t0 = DWT_CYCCNT_REG;
    for (volatile int i = 0; i < 16; i++) { /* burn a few cycles */ }
    return DWT_CYCCNT_REG != t0;
}

static inline uint32_t dwt_now(void) { return DWT_CYCCNT_REG; }

#else /* native build */
static inline bool dwt_init(void) { return true; }
static inline uint32_t dwt_now(void) { return 0u; }
#endif

/* Unsigned subtraction is correct across the 32-bit wrap, so this needs no
 * special handling at the ~25 s wrap interval at 170 MHz. */
static inline uint32_t dwt_elapsed(uint32_t start) { return dwt_now() - start; }

#define BLOCK_TIMER_START() uint32_t _blk_t0 = dwt_now()
#define BLOCK_TIMER_END()                                                      \
    do {                                                                       \
        uint32_t _dt = dwt_elapsed(_blk_t0);                                   \
        if (_dt > g_counters.worst_cycles) g_counters.worst_cycles = _dt;       \
        g_counters.blocks++;                                                   \
    } while (0)

/* ------------------------------------------------------- stack high-water */

#define STACK_PAINT_VALUE 0xC0FFEEC0uL

/* Bounds are passed in rather than hard-coded, because linker symbol names vary
 * between toolchains and projects. Look in your .ld file - CubeIDE usually
 * gives you _estack, and the bottom of the stack region is often _sstack or
 * similar. Then:
 *
 *     extern uint32_t _sstack, _estack;
 *     stack_paint(&_sstack, &_estack);
 *
 * MUST be called at the very top of main(), before anything else uses the
 * stack, or you will overwrite live stack frames and crash.
 *
 * Direction matters: the stack grows DOWNWARD from stack_hi, so untouched paint
 * survives at the low end. stack_free_bytes() counts up from stack_lo until it
 * hits a word that has been written.
 */
void stack_paint(uint32_t *stack_lo, uint32_t *stack_hi);

/* Bytes still holding the paint value, i.e. never touched since stack_paint(). */
uint32_t stack_free_bytes(const uint32_t *stack_lo, const uint32_t *stack_hi);

#endif /* INSTRUMENT_H */
