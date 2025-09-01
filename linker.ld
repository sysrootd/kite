/* STM32F401RBT6 linker script (C-only bare-metal) */

ENTRY(Reset_Handler)

MEMORY
{
  FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 128K
  SRAM  (rwx): ORIGIN = 0x20000000, LENGTH = 64K
}

_estack = ORIGIN(SRAM) + LENGTH(SRAM);

SECTIONS
{
  /* Code and read-only data in FLASH */
  .text :
  {
    KEEP(*(.isr_vector))       /* Interrupt vector table */
    *(.text)                   /* Program code */
    *(.text.*)
    *(.rodata)                 /* Read-only data */
    *(.rodata.*)
    *(.init)
    *(.fini)
    KEEP(*(.init_array))       /* for C++ global constructors if any */
    . = ALIGN(4);
    _etext = .;                /* End of text */
  } > FLASH

  /* Initialized data copied from FLASH to SRAM */
  _sidata = LOADADDR(.data);

  .data : AT(_sidata)
  {
    _sdata = .;                /* Start of data in RAM */
    *(.data)
    *(.data.*)
    . = ALIGN(4);
    _edata = .;                /* End of data */
  } > SRAM

  /* Uninitialized data (BSS) */
  .bss (NOLOAD) :
  {
    _sbss = .;                 /* Start of BSS */
    *(.bss)
    *(.bss.*)
    *(COMMON)
    . = ALIGN(4);
    _ebss = .;                 /* End of BSS */
  } > SRAM

  /* Provide symbols for end of memory */
  PROVIDE(end = _ebss);
  PROVIDE(__end__ = _ebss);
}
