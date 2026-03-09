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


static inline void uart_init(struct usart *usart, unsigned long baud) {
  // 
  uint8_t af = 0;           // Alternate function
  uint16_t rx = 0, tx = 0;  // pins

  if (usart == USART1) RCC->APB2ENR |= BIT(14); // clock enable bit usart1
  if (usart == USART2) RCC->APB1ENR1 |= BIT(17); // clock enable bit usart 2
  //if (usart == USART3) RCC->APB1ENR1 |= BIT(18); //clock enable bit usart3 "usart3 clock enable" ref manual
  if (usart == LPUART1) RCC->APB1ENR2 |= BIT(0); 
  // set af = 7 or 8 (for LPUART)
  if (usart == USART1) af = 7, tx = PIN('A', 9), rx = PIN('A', 10); // design choice, open ended as long as uart tx and rx pins
  if (usart == USART2) af = 7, tx = PIN('A', 2), rx = PIN('A', 3);
  // if (usart == USART3) tx = PIN('D', 8), rx = PIN('D', 9); 
  if (usart == LPUART1) af = 8, tx = PIN('A', 2), rx = PIN('A', 3); 

  gpio_set_mode(tx, GPIO_MODE_AF);
  gpio_set_af(tx, af);
  gpio_set_mode(rx, GPIO_MODE_AF);
  gpio_set_af(rx, af);
  usart->CR1 = 0;                           // Disable this UART
  usart->BRR = USART2_FCK / baud;                 // FREQ is a UART bus frequency
  usart->CR1 |= BIT(0) | BIT(2) | BIT(3);  // Set UE, RE, TE
  if (tx == 0 && rx == 0) return; // safeguard 
}

static inline void uart_write_byte(struct usart *usart, uint8_t byte) {
  while ((usart->ISR & BIT(7)) == 0) (void)0;  // TXE: TDR empty
  usart->TDR = byte;
}

static inline void uart_write_buf(struct usart *usart, char *buf, size_t len) {
  while (len-- > 0) uart_write_byte(usart, *(uint8_t *) buf++);
}

static inline int uart_read_ready(struct usart *usart) {
  return usart->ISR & BIT(5);  // If RXNE bit is set, data is ready, SR is now ISR due to new IP, bit 5 is still RXNE
}

static inline uint8_t uart_read_byte(struct usart *usart) {
  return (uint8_t) (usart->RDR & 255); // DR is now RDR
}