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
#define ADXL345_REG_DATAX0 0x32
#define DATAX1 0x33
#define DATAY0 0x34
#define DATAY1 0x35
#define DATAZ0 0x36
#define DATAZ1 0x37

//SPI Protocol Bits
#define ADXL345_SPI_READ 0x80
#define ADXL345_SPI_MB 0x40

/*
0. select SPI device to use, GPIO pins for miso, mosi, ssel, and sck. 74HC595
- use 74HC595 8bit shift reg first to make it easy by omitting miso, move on to 
1. bring up clocks
2. GPIO functionality. 
3. 
*/

/*
use the shift register, latch pin, datapin, clock pin. 
tie OE to GND so outputs stay enabled
PA5 → 74HC595 SHCP
PA7 → 74HC595 SER
PB0 → 74HC595 STCP

74HC595 OE → GND
74HC595 MR → 3.3V
74HC595 VCC → 3.3V
74HC595 GND → GND
74HC595 HRCL -> 3.3

Q0–Q7 → resistor → LED → GND
*/

static inline void spi1_gpio_init(uint16_t sck, uint16_t miso, uint16_t mosi, uint16_t cs){
    /* use these if not rewiring/have a fixed spi pinout. 
    sck = PIN('A', 5);
    miso = PIN('A',6 ); can be unused for shift register
    mosi = PIN('A',7 ); 
    ssel = PIN('B',0 );
    */
    // gpio set mode also configures the gpio PIN/BANK clocks
    uint8_t af = 5; 
    RCC->APB2ENR |= BIT(12); 
    gpio_set_mode(sck, GPIO_MODE_AF); 
    gpio_set_af(sck, af); 
    gpio_set_mode(miso, GPIO_MODE_AF); 
    gpio_set_af(miso, af); 
    gpio_set_mode(mosi, GPIO_MODE_AF); 
    gpio_set_af(mosi, af); 
    gpio_set_mode(cs, GPIO_MODE_OUTPUT); 
    gpio_write(cs, false); 
    //gpio_set_af(ssel, af); 
    // needs pull-up/pulldown 
}

static inline void spi1_master_config(void){
    SPI1->CR1 |= BIT(2); 
    // set 8-bit
    SPI1->CR2 &= ~(0xFU << 8); // clear CR2 bits 8-11
    SPI1->CR2 |= (7U << 8); // set 8bit (0111)

    //Keep RXONLY bit clear when bidirectional mode is active.
    SPI1->CR1 &= ~BIT(10); // set to full-duplex
    SPI1->CR1 |= BIT(8);   // SSI
    SPI1->CR1 |= BIT(9);   // SSM
    SPI1->CR1 &= ~BIT(7); // MSB
    // SPI1->CR1 |= BIT(7); lsb first 
    SPI1->CR1 &= ~(7U << 3); // clear baud 
    SPI1->CR1 |= (4U << 3); // set baud to Fpclk/32
    //SPI1->CR1 &= ~(BIT(1) | BIT(0)); // CPOL and CPHA = 0
    SPI1->CR1 |= (BIT(1) | BIT(0)); // cpol cpha = 1
    // SPI1->CR1 |= BIT(0); // cpha = 1
    // SPI1->CR1 |= BIT(1); // cpol = 1
    SPI1->CR2 |= BIT(12); // RXNE 8-bit
    SPI1->CR1 |= BIT(6); // spi enable
}

/*
Make sure SPI is enabled
Wait until the transmit side is ready for a new data unit.
Write one byte into the SPI data register.
Let the peripheral clock that byte out on MOSI.
Wait until the byte has progressed far enough that the transmit path is empty again.
*/


static inline uint8_t spi1_byte_txrx(uint8_t byte){
    if(SPI1->CR1 & BIT(6)){ // checks if SPI enabled
        // Wait until transmit buffer is empty (ready for new data)
        while ((SPI1->SR & BIT(1)) == 0) {
        }
        *(volatile uint8_t *)&SPI1->DR = byte;
        // Wait until a byte has been received
        while ((SPI1->SR & BIT(0)) == 0) {
        } 
    }
    // Read and return the received byte
    return *(volatile uint8_t *)&SPI1->DR;
}

static inline void spi1_wait_idle(void) {
    while (SPI1->SR & BIT(7)) {   // BSY
    }
}


/*
The ADXL345 needs normal SPI transaction framing:

CS low

send command/address bytes

transfer data

CS high
*/


static inline void spi_cs_low(uint16_t pin){
    gpio_write(pin, false); 
}

static inline void spi_cs_high(uint16_t pin){
    gpio_write(pin, true; )
}

/*
pull CS low
send a command byte
send a dummy byte
capture the returned data byte
wait until SPI is idle
pull CS high
return the data byte
*/

// move to adxl.c
static inline uint8_t adxl345_read_reg(uint16_t cs_pin, uint8_t reg){
    //ADXL345 -> 10111001, 0x9A4819, needs 1 byte to read, bit 7: read bit 6: multi byte bit 0-5: reg address
    uint8_t cmd = ADXL345_SPI_READ | reg; 
    gpio_write(cs_pin, false); // drive low
    // send cmd over SPI
        // check/wait status TX reg
        // move cmd to DR
        // wait for RX to finish
        // dummy/blank data to clear flag
        // pull CS high

    while ((SPI1->SR & BIT(1)) == 0) {} // wait for TX
    // send cmd to DR
    gpio_write(cs_pin, true); 
}

static inline void adxl345_write_reg(uint16_t cs_pin, uint8_t reg, uint8_t value){

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

