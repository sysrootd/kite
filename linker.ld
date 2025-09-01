_estack = ORIGIN(SRAM) + LENGTH(SRAM);
ENTRY(Reset_Handler)

MEMORY
{
  FLASH(rx) : ORIGIN = 0x08000000, LENGTH = 128K
  SRAM(rwx) : ORIGIN = 0x20000000, LENGTH = 64K
}

SECTIONS
{
  /* Code and read-only data go to FLASH */
  .text :
  {
    KEEP(*(.isr_vector))        /* Interrupt vectors */
    *(.text)                    /* Code */
    *(.text.*)
    *(.init)
    *(.fini)
    *(.rodata)                  /* Read-only data */
    *(.rodata.*)
    KEEP(*(.init_array))        /* C++ global constructors */
    . = ALIGN(4);
    _etext = .;                 /* End of text section */
  } > FLASH

  /* Initialized data copied from FLASH to SRAM */
  _la_data = LOADADDR(.data);

  .data : AT(_la_data)
  {
    _sdata = .;                 /* Start of data section in RAM */
    *(.data)
    *(.data.*)
    . = ALIGN(4);
    _edata = .;                 /* End of data section */
  } > SRAM

  /* Uninitialized data (zero-initialized) */
  .bss (NOLOAD) :
  {
    _sbss = .;
    __bss_start__ = _sbss;
    *(.bss)
    *(.bss.*)
    *(COMMON)
    . = ALIGN(4);
    _ebss = .;
    __bss_end__ = _ebss;
  } > SRAM

  /* Heap (optional) */
  .heap (NOLOAD) :
  {
    _sheap = .;
    . = . + 0x1000;
    _eheap = .;
  } > SRAM

  /* Exception tables (optional) */
  .ARM.exidx :
  {
    *(.ARM.exidx* .gnu.linkonce.armexidx.*)
  } > FLASH

  /* Provide symbols for end of memory */
  PROVIDE(end = _ebss);
  PROVIDE(__end__ = _ebss);
}
