/**
 * @file syscalls.c
 * @brief C library integration for the AGC firmware.
 *
 *  All the firmware needs from the C library is memset() and the init array
 *  hook the startup file calls. Nothing here, and nothing the firmware links
 *  against, allocates: there is no _sbrk() and no libnosys, so a heap cannot
 *  appear by accident. Anything that reached for malloc would fail to link.
 */

void _init(void) {} // __libc_init_array() is called from the startup file
void _fini(void) {} // no finalization needed
