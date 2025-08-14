/* ENTRY POINT: where execution starts */
ENTRY(Reset_Handler)

/* map the memory */
MEMORY
{
    FLASH(rx):ORIGIN = 0x08000000,LENGTH =256K
    SRAM1(rwx):ORIGIN = 0x20000000,LENGTH =48K
    SRAM2(rwx):ORIGIN = 0x10000000,LENGTH =16K
}


/* map the code, constants, globals/statics, 
   zero-initialised globals/statics to physical addresses */

SECTIONS
{
    /* --- VECTOR TABLE + CODE + CONSTANTS --- */
    .text : {
        KEEP(*(.isr_vector)) /* Interrupt vector table */
        *(.text*)            /* All .text (functions) */
        *(.rodata*)          /* Read-only data */
    } > FLASH

    /* Used by the startup to initialize data */
    _sidata = LOADADDR(.data);

    /* --- INITIALISED DATA ---  */
    .data : {
        . = ALIGN(4);
        _sdata = .;          /* start of .data section in RAM */
        *(.data)
        *(.data*)            /* Initialised global/static variables */
        . = ALIGN(4);
        _edata = .;          /* End of .data section in RAM */
    } > SRAM1 AT> FLASH

    /* --- UNINITIALISED DATA --- */
    .bss : {
        . = ALIGN(4);
        _sbss = .;           /* start of .bss section */
        *(.bss)
        *(.bss*)             /* uninitialised global/static vars */
        *(COMMON)
        . = ALIGN(4);
        _ebss = .;           /* End of .bss */
    } > SRAM1

    /* --- STACK (TOP OF RAM, GROWS DOWN) ---  */
    .stack (NOLOAD): {
        . = ALIGN(8);
        _sstack = ORIGIN(SRAM1) + LENGTH(SRAM1) - 0x1000; /* allocate 4KB for stack */
        . = _sstack;
        _estack = ORIGIN(SRAM1) + LENGTH(SRAM1);
    } > SRAM1

    _end = .;

}