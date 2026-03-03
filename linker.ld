/* Entry point for execution */
ENTRY(Reset_Handler)

/* Highest address of the user mode stack */
/* _estack = top of RAM (calculated below) */

/* SRAM memory symbols */
_sram_start = ORIGIN(RAM);      /* Start of RAM */
_sram_size  = LENGTH(RAM);      /* Total RAM size */
_estack     = ORIGIN(RAM) + LENGTH(RAM); /* End of RAM (top of stack) */

/* Heap and stack size requirements */
_Min_Heap_Size  = 0x200;  /* Minimum heap size */
_Min_Stack_Size = 0x400;  /* Minimum stack size */

/* Memory layout for STM32F401RBTx */
/* RAM:  0x20000000 - 0x2000FFFF (64KB) */
/* FLASH: 0x08000000 - 0x0801FFFF (128KB) */
MEMORY
{
  RAM    (xrw)    : ORIGIN = 0x20000000,   LENGTH = 64K
  FLASH    (rx)    : ORIGIN = 0x8000000,   LENGTH = 128K
}

/* Output sections */
SECTIONS
{

  /* Startup code into FLASH */
  .isr_vector :
  {
    . = ALIGN(4);
    KEEP(*(.isr_vector))  /* Startup code with vector table */
    . = ALIGN(4);
  } >FLASH

  /* Program code and data into FLASH */
  .text :
  {
    . = ALIGN(4);
    *(.text)           /* .text sections (code) */
    *(.text*)          /* .text* sections (code) */
    *(.glue_7)         /* glue arm to thumb code */
    *(.glue_7t)        /* glue thumb to arm code */
    *(.eh_frame)

    KEEP (*(.init))
    KEEP (*(.fini))

    . = ALIGN(4);
    _etext = .;        /* Define a global symbol at end of code */
  } >FLASH

  /* Constant data into FLASH */
  .rodata :
  {
    . = ALIGN(4);
    *(.rodata)         /* .rodata sections (constants, strings, etc.) */
    *(.rodata*)        /* .rodata* sections (constants, strings, etc.) */
    . = ALIGN(4);
  } >FLASH

  .ARM.extab (READONLY) : /* READONLY keyword is GCC11+ */
  {
    . = ALIGN(4);
    *(.ARM.extab* .gnu.linkonce.armextab.*)
    . = ALIGN(4);
  } >FLASH

  .ARM (READONLY) : /* READONLY keyword is GCC11+ */
  {
    . = ALIGN(4);
    __exidx_start = .;
    *(.ARM.exidx*)
    __exidx_end = .;
    . = ALIGN(4);
  } >FLASH

  .preinit_array (READONLY) : /* READONLY keyword is GCC11+ */
  {
    . = ALIGN(4);
    PROVIDE_HIDDEN (__preinit_array_start = .);
    KEEP (*(.preinit_array*))
    PROVIDE_HIDDEN (__preinit_array_end = .);
    . = ALIGN(4);
  } >FLASH

  .init_array (READONLY) : /* READONLY keyword is GCC11+ */
  {
    . = ALIGN(4);
    PROVIDE_HIDDEN (__init_array_start = .);
    KEEP (*(SORT(.init_array.*)))
    KEEP (*(.init_array*))
    PROVIDE_HIDDEN (__init_array_end = .);
    . = ALIGN(4);
  } >FLASH

  .fini_array (READONLY) : /* READONLY keyword is GCC11+ */
  {
    . = ALIGN(4);
    PROVIDE_HIDDEN (__fini_array_start = .);
    KEEP (*(SORT(.fini_array.*)))
    KEEP (*(.fini_array*))
    PROVIDE_HIDDEN (__fini_array_end = .);
    . = ALIGN(4);
  } >FLASH

  /* Used by startup to initialize data section */
  _sidata = LOADADDR(.data);

  /* Initialized data sections into RAM */
  .data :
  {
    . = ALIGN(4);
    _sdata = .;        /* Create a global symbol at data start */
    *(.data)           /* .data sections */
    *(.data*)          /* .data* sections */
    *(.RamFunc)        /* .RamFunc sections */
    *(.RamFunc*)       /* .RamFunc* sections */

    . = ALIGN(4);
    _edata = .;        /* Define a global symbol at data end */

  } >RAM AT> FLASH

  /* Uninitialized data section into RAM */
  . = ALIGN(4);
  .bss :
  {
    /* Used by startup to initialize .bss section */
    _sbss = .;         /* Define a global symbol at bss start */
    __bss_start__ = _sbss;
    *(.bss)
    *(.bss*)
    *(COMMON)

    . = ALIGN(4);
    _ebss = .;         /* Define a global symbol at bss end */
    __bss_end__ = _ebss;
  } >RAM

  /* User heap and stack section */
  ._user_heap_stack :
  {
    . = ALIGN(8);
    PROVIDE ( end = . );
    PROVIDE ( _end = . );
    . = . + _Min_Heap_Size;
    . = . + _Min_Stack_Size;
    . = ALIGN(8);
  } >RAM

  /* Remove unnecessary compiler libraries */
  /DISCARD/ :
  {
    libc.a ( * )
    libm.a ( * )
    libgcc.a ( * )
  }

  .ARM.attributes 0 : { *(.ARM.attributes) }
}