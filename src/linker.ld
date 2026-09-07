/**
 * file: linker.ld
 * brief: Linker script for the AGC firmware (STM32G474RE).
 *
 *  Nothing exotic here. The G4 has no cache and no tightly
 *  coupled memories worth splitting the image across, so the
 *  whole 128 KiB of SRAM is treated as one region and the DMA
 *  capture buffer lives in .bss like everything else.
 *
 *  There is no heap: the application allocates nothing
 *  dynamically. Only the stack is reserved.
 *
 *  Recommended linker flags:
 *    -Wl,--gc-sections
 */

/* Stack reservation. There is no heap - see syscalls.c. */
_Reserved_Stack_Size = 0x1000;

ENTRY(Reset_Handler)
PROVIDE(_estack = ORIGIN(SRAM) + LENGTH(SRAM));

MEMORY {
  FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 0x00080000 /* 512 KiB internal flash */
  SRAM (xrw) : ORIGIN = 0x20000000, LENGTH = 0x00020000 /* 128 KiB: SRAM1 + SRAM2 + CCM SRAM, contiguous */
}

SECTIONS {
  .isr_vector : {
    . = ALIGN(4);
    KEEP(*(.isr_vector))
    . = ALIGN(4);
  } >FLASH

  .text : {
    . = ALIGN(4);
    *(.text)
    *(.text*)
    *(.glue_7) /* glue arm to thumb code */
    *(.glue_7t) /* glue thumb to arm code */

    KEEP (*(.init))
    KEEP (*(.fini))

    . = ALIGN(4);
    _etext = .;
  } >FLASH

  .rodata : {
    . = ALIGN(4);
    *(.rodata)
    *(.rodata*)
    . = ALIGN(4);
  } >FLASH

  .ARM.extab : {
    . = ALIGN(4);
    *(.ARM.extab* .gnu.linkonce.armextab.*)
    . = ALIGN(4);
  } >FLASH

  .ARM : {
    . = ALIGN(4);
    __exidx_start = .;
    *(.ARM.exidx*)
    __exidx_end = .;
    . = ALIGN(4);
  } >FLASH

  .preinit_array (READONLY) : {
    . = ALIGN(4);
    PROVIDE_HIDDEN (__preinit_array_start = .);
    KEEP (*(.preinit_array*))
    PROVIDE_HIDDEN (__preinit_array_end = .);
    . = ALIGN(4);
  } >FLASH

  .init_array (READONLY) : {
    . = ALIGN(4);
    PROVIDE_HIDDEN (__init_array_start = .);
    KEEP (*(SORT(.init_array.*)))
    KEEP (*(.init_array*))
    PROVIDE_HIDDEN (__init_array_end = .);
    . = ALIGN(4);
  } >FLASH

  .fini_array (READONLY) : {
    . = ALIGN(4);
    PROVIDE_HIDDEN (__fini_array_start = .);
    KEEP (*(SORT(.fini_array.*)))
    KEEP (*(.fini_array*))
    PROVIDE_HIDDEN (__fini_array_end = .);
    . = ALIGN(4);
  } >FLASH

  _sidata = LOADADDR(.data);

  .data : {
    . = ALIGN(4);
    _sdata = .;
    *(.data)
    *(.data*)
    . = ALIGN(4);
    _edata = .;
  } >SRAM AT> FLASH

  .bss : {
    . = ALIGN(4);
    _sbss = .;
    __bss_start__ = _sbss;
    *(.bss)
    *(.bss*)
    *(COMMON)
    . = ALIGN(4);
    _ebss = .;
    __bss_end__ = _ebss;
  } >SRAM

  /* Reserve the stack so that a build which cannot fit it fails here at link
   * time rather than at run time. */
  ._stack (NOLOAD) : {
    . = ALIGN(8);
    PROVIDE (end = .);
    PROVIDE (_end = .);
    . = . + _Reserved_Stack_Size;
    . = ALIGN(8);
  } >SRAM

  .ARM.attributes 0 : {
    *(.ARM.attributes)
  }
}
