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


static inline void can_gpio_init(uint16_t can_rx, uint16_t can_tx){
    RCC->APB1ENR1 |= BIT(25); // can peripheral clock
    RCC->AHB2ENR  |= BIT(0); // gpio port clock 
    uint8_t af = 9; 
    gpio_set_mode(can_rx, GPIO_MODE_AF);
    gpio_set_af(can_rx, af); 
    gpio_set_mode(can_tx, GPIO_MODE_AF); // find enum names in gpio.h
    gpio_set_af(can_tx, af); 

    gpio_set_otype(can_rx, 0);
    gpio_set_otype(can_tx, 0); // push pull

    gpio_set_speed(can_rx); // default to 11 (very high speed)
    gpio_set_speed(can_tx); \

    gpio_set_pupd(can_rx, NO_PULL); // enum names in gpio.h
    gpio_set_pupd(can_tx, NO_PULL);  
}

static inline void can_init_mode(){
    CAN_CORE->MCR |= BIT(0);
    while(!(CAN_CORE->MSR & BIT(0))){} // wait for init acknowledgement  
}

static inline void can_filter_init(){
    CAN_FILTER->FMR |= BIT(0); // initialize mode
}