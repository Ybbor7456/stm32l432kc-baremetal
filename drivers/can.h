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

void can_gpio_init(uint16_t can_rx, uint16_t can_tx);

static inline void can_enter_init_mode(void){
    CAN_CORE->MCR &= ~BIT(1); // cleaer sleep 
    CAN_CORE->MCR |= BIT(0); // set      
    while (!(CAN_CORE->MSR & BIT(0))) {} // wait for init acknowledgement
}

static inline void can_leave_init_mode(void){
    CAN_CORE->MCR &= ~BIT(0); // clear
    while (CAN_CORE->MSR & BIT(0)) {} // wait for INAK to clear
    while(CAN_CORE->MSR & BIT(1)){}
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
        ((brp & 0x3FFu) << 0); // baud rate prescaler = 5, divides peripheral clock to set time quantum for CAN
        //BIT(30); // loopback
    //can_leave_init_mode();

}

static inline void can_filter_bank0_init(){
    CAN_FILTER->FMR |= BIT(0); // initialize mode
    
    //CAN_FILTER->FM1R |= BIT(0); // identifier list mode on filter bank 0. 
    CAN_FILTER->FM1R &= ~BIT(0); // identifier mask mode on filter bank 0 

    CAN_FILTER->FS1R |= BIT(0); // 32 bit scale config
    //CAN_FILTER->FFA1R |= BIT(0); //filter assigned to FIFO1
    CAN_FILTER->FFA1R &= ~BIT(0); // filter assigned to FIFO0 at bank 0

    CAN1_FILTERBK[0].FR1 = 0;   // write FR1
    CAN1_FILTERBK[0].FR2 = 0;   // write FR2

    CAN_FILTER->FA1R |= BIT(0); // filter is on, bank 0
    CAN_FILTER->FMR &= ~BIT(0); // leave filter init mode
}

static inline void can_init(uint16_t can_rx, uint16_t can_tx){
    can_gpio_init(can_rx, can_tx); // does not need MCR bit 0 (INRW) set
    can_enter_init_mode();
    CAN_CORE->MCR |= BIT(6);   // ABOM = auto bus-off management
    can_btr_config();
    can_filter_bank0_init(); 
    can_leave_init_mode(); 
}

static inline int8_t can_TX_get_empty(void){ // check if a TX mailbox is empty
    if(CAN_CORE->TSR & BIT(26)){return 0;} 
    if(CAN_CORE->TSR & BIT(27)){return 1;} 
    if(CAN_CORE->TSR & BIT(28)){return 2;} 
    else return -1; 
}

typedef struct {
    uint32_t id;
    uint8_t dlc;
    uint8_t data[8];
} can_msg_t;

void can_send_std(can_msg_t *msg);

static inline bool can_fifo0_pending(void){
    printf("testing can_fifo0 pending \r\n");
    return(CAN_CORE->RF0R & 0x3U) != 0; 
}

void can_read_fifo0(can_msg_t *msg);