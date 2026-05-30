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

void can_enter_init_mode(void);

void can_leave_init_mode(void);

void can_btr_config(void);

void can_filter_bank0_init();

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