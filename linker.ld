/* STM32F401RBT6 linker script */

/* Memory regions */
MEMORY
{
    FLASH (rx)  : ORIGIN = 0x08000000, LENGTH = 128K
    RAM   (xrw) : ORIGIN = 0x20000000, LENGTH = 64K
}

/* Entry point */
ENTRY(Reset_Handler)

SECTIONS
{
    /* Vector table and code */
    .text :
    {
        KEEP(*(.isr_vector))    /* interrupt vector table */
        *(.text*)               /* program code */
        *(.rodata*)             /* read-only data */
        KEEP(*(.init))
        KEEP(*(.fini))
        . = ALIGN(4);
        _etext = .;             /* end of code */
    } > FLASH

    /* Initialized data (will be copied to RAM) */
    .data : AT (ADDR(.text) + SIZEOF(.text))
    {
        _sidata = LOADADDR(.data); /* start of init values in flash */

        _sdata = .;               /* start of data in RAM */
        *(.data*)
        . = ALIGN(4);
        _edata = .;               /* end of data in RAM */
    } > RAM

    /* Uninitialized data (BSS) */
    .bss (NOLOAD) :
    {
        _sbss = .;                /* start of bss */
        *(.bss*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = .;                /* end of bss */
    } > RAM

    /* End of RAM used by program */
    _end = .;

    /* Top of stack (used in startup) */
    PROVIDE(_estack = ORIGIN(RAM) + LENGTH(RAM));
}
