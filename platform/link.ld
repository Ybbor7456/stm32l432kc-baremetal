ENTRY(_reset); 
/* tells linker of entry point, aid for a debugger since debugger does not know about vector table so uses elf header */ 

/* tells linker of two emory regions in the address space and sizes */ 
MEMORY {
  flash (rx)  : ORIGIN = 0x08000000, LENGTH = 256K
  sram  (rwx) : ORIGIN = 0x20000000, LENGTH = 48K
}

/* tells linker to create estack symbol, with value at end of RAM */ 
_estack     = ORIGIN(sram) + LENGTH(sram);    
/* stack points to end of SRAM */


/* These lines tell the linker to put vectors table on flash first, followed by .text section (firmware code), followed by the read only data .rodata. */ 
SECTIONS {
  .vectors  : { KEEP(*(.vectors)) }   > flash
  .text     : { *(.text*) }           > flash
  .rodata   : { *(.rodata*) }         > flash


/*
At runtime, .data lives in SRAM (> sram) because it’s writable.
But its initial values must be stored somewhere non-volatile, so the linker also allocates its “load image” in Flash (AT > flash).
*/

  .data : {
    _sdata = .;   /* .data section start */
    *(.first_data)
    *(.data SORT(.data.*))
    _edata = .;  /* .data section end */
  } > sram AT > flash
  _sidata = LOADADDR(.data);



  /*
.bss is for globals/statics that start as zero (or have no explicit initializer).

It lives in SRAM.

It does not take space in Flash for initial contents, because startup code will just zero it.
  
  */
  .bss : {
    _sbss = .;              /* .bss section start */
    *(.bss SORT(.bss.*) COMMON)
    _ebss = .;              /* .bss section end */
  } > sram


  /*Aligns the “current location” to an 8-byte boundary (often useful for ABI / stack alignment) */ 
  . = ALIGN(8);
  _end = .;     /* for cmsis_gcc.h  */
}

/*
    BIG PICTURE
    Flash gets: .vectors, .text, .rodata, and the initial image of .data

    SRAM gets: runtime .data, .bss, plus space remaining for stack/heap

*/