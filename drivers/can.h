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


static inline void can_gpio_init(uint16_t can_rx, uint16_t can_tx, ){
    RCC->APB1ENR1 |= BIT(25); // can peripheral clock
    gpio_enable(can_rx); 
    gpio_enable(can_tx); // redundant but harmless
    uint8_t af = 9; 
    gpio_set_mode(can_rx, GPIO_MODE_AF);
    gpio_set_af(can_rx, af); 
    gpio_set_mode(can_tx, GPIO_MODE_AF); // 2nd arg enum names in gpio.h
    gpio_set_af(can_tx, af); 

    gpio_set_otype(can_rx, 0);
    gpio_set_otype(can_tx, 0); // push pull

    //gpio_set_speed(can_rx); // default to 11 (very high speed), alter in gpio.h to set enum values. 
    //gpio_set_speed(can_tx); 

    gpio_set_select_speed(can_rx,GPIO_VHIGH); 
    gpio_set_select_speed(can_tx, GPIO_VHIGH); 

    gpio_set_pupd(can_rx, NO_PULL); // enum names in gpio.h
    gpio_set_pupd(can_tx, NO_PULL);  
}

static inline void can_enter_init_mode(void){
    CAN_CORE->MCR |= BIT(0); // set      
    while (!(CAN_CORE->MSR & BIT(0))) {} // wait for init acknowledgement
}

static inline void can_leave_init_mode(void){
    CAN_CORE->MCR &= ~BIT(0); // clear
    while (CAN_CORE->MSR & BIT(0)) {} // wait for INAK to clear
}

static inline void can_btr_config(void){
    const uint32_t brp = 4;   // 4 + 1 = 5
    const uint32_t ts1 = 11;  // ts1 = 12 tq
    const uint32_t ts2 = 1;   // ts2 = 2 tq
    const uint32_t sjw = 1;   // sjw = 2 tq
    //can_enter_init_mode();
    CAN_CORE->BTR =
        ((sjw & 0x3u)  << 24) | // # of time quantum of a bit change during resynchronization to compensate for phase errors
        ((ts2 & 0x7u)  << 20) | // duration from sample point to end of bit, a time buffer.
        ((ts1 & 0xFu)  << 16) | // 1100 << 16, duration from start of bit to sample point
        ((brp & 0x3FFu) << 0);  // baud rate prescaler = 5, divides peripheral clock to set time quantum for CAN
    //can_leave_init_mode();
}

static inline void can_filter_bank0_init(){
    CAN_FILTER->FMR |= BIT(0); // initialize mode
    
    //CAN_FILTER->FM1R |= BIT(0); // identifier list mode on filter bank 0. 
    CAN_FILTER->FM1R &= ~BIT(0); // identifier mask mode on filter bank 0 

    CAN_FILTER->FS1R |= BIT(0); // 32 bit scale config
    //CAN_FILTER->FFA1R |= BIT(0); //filter assigned to FIFO1
    CAN_FILTER->FFAIR &= ~BIT(0); // filter assigned to FIFO0

    CAN1_FILTERBK[0].FR1 = 0;   // write FR1
    CAN1_FILTERBK[0].FR2 = 0;   // write FR2

    CAN_FILTER->FA1R |= BIT(0); // filter is on
    CAN_FILTER->FMR &= ~BIT(0); // leave filter init mode
}

static inline void can_init(void){
    can_gpio_init(); // does not need MCR bit 0 (INRW) set
    can_enter_init_mode();
    can_btr_config();
    can_filter_bank0_init(); 
    can_leave_init_mode(); 
}