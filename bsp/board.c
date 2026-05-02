#include "board.h"
#include "../drivers/stm32l4_regs.h"


/*
#define FLASH_ACR (*(volatile uint32_t *)0x40022000u)

void rcc_init_pll_80mhz(void){
    RCC->CR |= (1u << 8); 
    while(!(RCC->CR & (1u << 10))); 
    FLASH_ACR = (4u << 0) | (1u << 8) | (1u << 9) | (1u <<10); 
    while((FLASH_ACR & 0x7u) != 4u); 

    RCC->PLLCFGR = (2u << 0) | (0u << 4) | (10u << 8) | (0u << 25) | (1u << 24); 

    RCC->CR |= (1u << 24); 
    while(!(RCC->CR & (1u << 25)));

    RCC->CFGR = (RCC->CFGR * ~(3u << 0)) | (3u << 0);
    while((RCC->CFGR & (3u << 2)) != (3u << 2));  
} */