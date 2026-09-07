/* instrument.c - see instrument.h. No external dependencies. */
#include "instrument.h"

volatile counters_t g_counters = {0, 0, 0, 0, 0};

void stack_paint(uint32_t *stack_lo, uint32_t *stack_hi)
{
    if (stack_lo == NULL || stack_hi == NULL || stack_lo >= stack_hi) {
        return;
    }
    /* Paint from the low end up to just below the current stack pointer. Taking
     * the address of a local is a portable way to find roughly where we are;
     * the margin keeps us clear of this function's own frame. */
    uint32_t here;
    /* Integer arithmetic, not pointer arithmetic: (&here - 32) is undefined
     * behaviour because `here` is a scalar, and GCC rejects it under
     * -Werror=array-bounds. The 128-byte margin keeps us clear of this
     * function's own frame. */
    uintptr_t sp = (uintptr_t)&here;
    uintptr_t lo = (uintptr_t)stack_lo;
    uintptr_t hi = (uintptr_t)stack_hi;
    uintptr_t limit = (sp > lo + 128u) ? (sp - 128u) : lo;
    if (limit > hi) {
        limit = hi;
    }
    for (uintptr_t a = lo; a + sizeof(uint32_t) <= limit; a += sizeof(uint32_t)) {
        *(uint32_t *)a = STACK_PAINT_VALUE;
    }
    return;
}

uint32_t stack_free_bytes(const uint32_t *stack_lo, const uint32_t *stack_hi)
{
    if (stack_lo == NULL || stack_hi == NULL || stack_lo >= stack_hi) {
        return 0u;
    }
    const uint32_t *p = stack_lo;
    while (p < stack_hi && *p == STACK_PAINT_VALUE) {
        p++;
    }
    return (uint32_t)((size_t)(p - stack_lo) * sizeof(uint32_t));
}
