baremetal/
  Makefile
  README.md

  app/
    main.c

  bsp/
    board.h
    board.c          (optional but nice)

  drivers/
    hal.h            (rename later; see note below)
    gpio.c/.h        (optional split)
    uart.c/.h        (optional split)
    i2c.c/.h         (later)

  platform/
    startup.c
    syscalls.c
    link.ld

  docs/
    pinmap.md
    week1-notes.md

  references/
    datasheets/
    reference-manuals/
    app-notes/
  
  build/
    firmware.elf
    firmware.bin
    firmware.elf.map
    main.o
