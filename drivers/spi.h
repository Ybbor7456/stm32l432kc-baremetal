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
0. select SPI device to use, GPIO pins for miso, mosi, ssel, and sck. 
- use 8bit shift reg first to make it easy by omitting miso, move on to 
1. bring up clocks
2. GPIO functionality. 
3. 
*/

static inline void spi1_gpio_init(uint16_t sck, uint16_t miso, uint16_t mosi, uint16_t ssel){
    /* use these if not rewiring/have a fixed spi pinout. 
    sck = PIN('A', 5);
    miso = PIN('A',6 );
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