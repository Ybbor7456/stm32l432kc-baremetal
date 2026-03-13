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

Q0–Q7 → resistor → LED → GND
*/

static inline void spi1_gpio_init(uint16_t sck, uint16_t miso, uint16_t mosi, uint16_t ssel, uint16_t latch){
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
    gpio_set_mode(ssel, GPIO_MODE_OUTPUT); 
    gpio_set_af(ssel, af); 
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
    SPI1->CR1 &= ~(BIT(1) | BIT(0)); // CPOL and CPHA = 0 
    // SPI1->CR1 |= BIT(0); // cpha = 1
    // SPI1->CR1 |= BIT(1); // cpol = 1
    SPI1->CR2 |= BIT(12); // RXNE 8-bit
    SPI1->CR1 |= BIT(6); // spi enable
}