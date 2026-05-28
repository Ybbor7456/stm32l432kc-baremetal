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

static inline void rng_init(uint8_t clk){
    if((clk >= 4) || (clk == 1) || (clk == 0)) {clk = 3;} 
    RCC->AHB2ENR |= BIT(18); // set RNG bus clock
    RCC->CCIPR &= ~(3u << 26); // clear
    RCC->CCIPR |= (clk << 26); // set kernel clock src for USB, RNG, SDMMC
    RNG->CR |= BIT(2); // set RNGEN
}

uint32_t rng_read();