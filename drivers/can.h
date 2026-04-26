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

static inline void can_send_std(can_msg_t *msg){
    int8_t mb = can_TX_get_empty();
    if (mb == -1) return;

    uint8_t dlc = msg->dlc;
    if (dlc > 8) dlc = 8;

    // set mailbox index based on availability
    CAN_TXMBOX[mb].TIR  = 0;
    CAN_TXMBOX[mb].TDTR = 0;
    CAN_TXMBOX[mb].TDLR = 0;
    CAN_TXMBOX[mb].TDHR = 0;

    CAN_TXMBOX[mb].TIR |= ((uint32_t)(msg->id & 0x7FFu) << 21); // STID[10:0] -> bits 31:21
    CAN_TXMBOX[mb].TIR &= ~BIT(2); // IDE = 0, standard frame
    CAN_TXMBOX[mb].TIR &= ~BIT(1);  // RTR = 0, data frame

    CAN_TXMBOX[mb].TDTR |= (msg->dlc & 0xFu); // DLC bits 3:0
    
    if (dlc > 0) CAN_TXMBOX[mb].TDLR |= ((uint32_t)msg->data[0] << 0);
    if (dlc > 1) CAN_TXMBOX[mb].TDLR |= ((uint32_t)msg->data[1] << 8);
    if (dlc > 2) CAN_TXMBOX[mb].TDLR |= ((uint32_t)msg->data[2] << 16);
    if (dlc > 3) CAN_TXMBOX[mb].TDLR |= ((uint32_t)msg->data[3] << 24);

    if (dlc > 4) CAN_TXMBOX[mb].TDHR |= ((uint32_t)msg->data[4] << 0);
    if (dlc > 5) CAN_TXMBOX[mb].TDHR |= ((uint32_t)msg->data[5] << 8);
    if (dlc > 6) CAN_TXMBOX[mb].TDHR |= ((uint32_t)msg->data[6] << 16);
    if (dlc > 7) CAN_TXMBOX[mb].TDHR |= ((uint32_t)msg->data[7] << 24);

    CAN_TXMBOX[mb].TIR |= BIT(0); // TXRQ
}

static inline bool can_fifo0_pending(void){
    printf("testing can_fifo0 pending \r\n");
    return(CAN_CORE->RF0R & 0x3U) != 0; 
}

static inline void can_read_fifo0(can_msg_t *msg) {
    printf("testing can_read_fifo0 \r\n"); 
    // Check if a message is actually there
    if(!can_fifo0_pending()){
        return; 
    }
    // Extract Identifier 
    if ((CAN_RXMBOX->RIR & BIT(2)) == 0) {
        msg->id = (CAN_RXMBOX->RIR >> 21) & 0x7FFu;
    } else {
        msg->id = (CAN_RXMBOX->RIR >> 3) & 0x1FFFFFFFu;
    }
    //Extract Data Length Code (DLC is bits 3:0)
    msg->dlc = (uint8_t)(CAN_RXMBOX->RDTR & 0xFu);

    uint32_t low = CAN_RXMBOX->RDLR;
    msg->data[0] = (uint8_t)(low >> 0);
    msg->data[1] = (uint8_t)(low >> 8);
    msg->data[2] = (uint8_t)(low >> 16);
    msg->data[3] = (uint8_t)(low >> 24);

    uint32_t high = CAN_RXMBOX->RDHR;
    msg->data[4] = (uint8_t)(high >> 0);
    msg->data[5] = (uint8_t)(high >> 8);
    msg->data[6] = (uint8_t)(high >> 16);
    msg->data[7] = (uint8_t)(high >> 24);

    // release the FIFO
    CAN_CORE->RF0R |= BIT(5); 
}