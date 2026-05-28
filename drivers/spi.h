#pragma once
#include <inttypes.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <math.h> 
#include <stdint.h>

#include "drivers/util.h"
#include "drivers/stm32l4_regs.h"
#include "bsp/board.h"

//Register Addresses as per Datasheet 
#define ADXL345_REG_DEVID 0x00
#define BW_RATE 0x2C
#define ADXL345_REG_POWER_CTL 0x2D
#define DATA_FORMAT 0x31
#define DATAX0 0x32
#define DATAX1 0x33
#define DATAY0 0x34
#define DATAY1 0x35
#define DATAZ0 0x36
#define DATAZ1 0x37
#define POWER_CTL 0x2D


//SPI Protocol Bits
#define ADXL345_SPI_READ 0x80
#define ADXL345_SPI_WRITE 0x00
#define ADXL345_SPI_MB 0x40
#define ADXL345_MEASURE 0x08


/*
ADXL345 uses I^2C pin names, it is an I^2C && SPI device. 
CS = CS (B0)
SCL = SCK A5
SDO = MISO A6 
SDA = MOSI A7
*/

void spi1_gpio_init(uint16_t sck, uint16_t miso, uint16_t mosi, uint16_t cs);

void spi1_master_config(void);

/*
Make sure SPI is enabled
Wait until the transmit side is ready for a new data unit.
Write one byte into the SPI data register.
Let the peripheral clock that byte out on MOSI.
Wait until the byte has progressed far enough that the transmit path is empty again.
*/


uint8_t spi1_byte_txrx(uint8_t byte);

static inline void spi1_wait_idle(void) {
    while (SPI1->SR & BIT(7)) {   // BSY
    }
}

static inline void spi_cs_low(uint16_t pin){
    gpio_write(pin, false); 
}

static inline void spi_cs_high(uint16_t pin){
    gpio_write(pin, true);
}

/*
https://patorjk.com/software/taag/#p=display&f=RubiFont&t=Shift+Register+Helpers&x=rainbow3&v=4&h=4&w=80&we=false
rubifont
 ▗▄▄▖▗▖ ▗▖▗▄▄▄▖▗▄▄▄▖▗▄▄▄▖    ▗▄▄▖ ▗▄▄▄▖ ▗▄▄▖▗▄▄▄▖ ▗▄▄▖▗▄▄▄▖▗▄▄▄▖▗▄▄▖     ▗▖ ▗▖▗▄▄▄▖▗▖   ▗▄▄▖ ▗▄▄▄▖▗▄▄▖  ▗▄▄▖
▐▌   ▐▌ ▐▌  █  ▐▌     █      ▐▌ ▐▌▐▌   ▐▌     █  ▐▌     █  ▐▌   ▐▌ ▐▌    ▐▌ ▐▌▐▌   ▐▌   ▐▌ ▐▌▐▌   ▐▌ ▐▌▐▌   
 ▝▀▚▖▐▛▀▜▌  █  ▐▛▀▀▘  █      ▐▛▀▚▖▐▛▀▀▘▐▌▝▜▌  █   ▝▀▚▖  █  ▐▛▀▀▘▐▛▀▚▖    ▐▛▀▜▌▐▛▀▀▘▐▌   ▐▛▀▘ ▐▛▀▀▘▐▛▀▚▖ ▝▀▚▖
▗▄▄▞▘▐▌ ▐▌▗▄█▄▖▐▌     █      ▐▌ ▐▌▐▙▄▄▖▝▚▄▞▘▗▄█▄▖▗▄▄▞▘  █  ▐▙▄▄▖▐▌ ▐▌    ▐▌ ▐▌▐▙▄▄▖▐▙▄▄▖▐▌   ▐▙▄▄▖▐▌ ▐▌▗▄▄▞▘                 
*/

static inline void shiftreg_latch_pulse(uint16_t latch_pin) {
    gpio_write(latch_pin, true);
    gpio_write(latch_pin, false);
}

static inline void shiftreg_write(uint8_t value, uint16_t latch_pin) {
    spi1_byte_txrx(value);     // shift 8 bits in
    spi1_wait_idle();          // wait until SPI fully finishes
    shiftreg_latch_pulse(latch_pin);   // move shifted bits to outputs
}

