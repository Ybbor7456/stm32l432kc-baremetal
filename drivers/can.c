#include "drivers/i2c.h"

typedef struct {
    uint32_t id;
    uint8_t dlc;
    uint8_t data[8];
} can_msg_t;


void can_gpio_init(uint16_t can_rx, uint16_t can_tx){
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

void can_send_std(can_msg_t *msg){
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

void can_read_fifo0(can_msg_t *msg) {
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