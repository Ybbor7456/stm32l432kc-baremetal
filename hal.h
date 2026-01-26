


#pragma once

#include <inttypes.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>

#define USART2_FCK 4000000UL  // must match USART2 kernel clock (fCK)
#define BIT(x) (1UL << (x))
#define PIN(bank, num) ((((bank) - 'A') << 8) | (num))
#define PINNO(pin) (pin & 255)
#define PINBANK(pin) (pin >> 8)

static inline void spin(volatile uint32_t count) {
  while (count--) {
    __asm volatile ("nop");
  }
}

struct systick {
  volatile uint32_t CTRL, LOAD, VAL, CALIB;
};
#define SYSTICK ((struct systick *) 0xe000e010) // arm core systck base address

struct rcc {
    volatile uint32_t CR;          // 0x00
    volatile uint32_t ICSCR;       // 0x04
    volatile uint32_t CFGR;        // 0x08
    volatile uint32_t PLLCFGR;     // 0x0C
    volatile uint32_t PLLSAI1CFGR; // 0x10
    uint32_t RESERVED0;            // 0x14

    volatile uint32_t CIER;        // 0x18
    volatile uint32_t CIFR;        // 0x1C
    volatile uint32_t CICR;        // 0x20
    uint32_t RESERVED1;            // 0x24

    volatile uint32_t AHB1RSTR;    // 0x28
    volatile uint32_t AHB2RSTR;    // 0x2C
    volatile uint32_t AHB3RSTR;    // 0x30
    uint32_t RESERVED2;            // 0x34

    volatile uint32_t APB1RSTR1;   // 0x38
    volatile uint32_t APB1RSTR2;   // 0x3C
    volatile uint32_t APB2RSTR;    // 0x40
    uint32_t RESERVED3;            // 0x44

    volatile uint32_t AHB1ENR;     // 0x48
    volatile uint32_t AHB2ENR;     // 0x4C
    volatile uint32_t AHB3ENR;     // 0x50
    uint32_t RESERVED4;            // 0x54

    volatile uint32_t APB1ENR1;    // 0x58
    volatile uint32_t APB1ENR2;    // 0x5C
    volatile uint32_t APB2ENR;     // 0x60
    uint32_t RESERVED5;            // 0x64

    volatile uint32_t AHB1SMENR;   // 0x68
    volatile uint32_t AHB2SMENR;   // 0x6C
    volatile uint32_t AHB3SMENR;   // 0x70
    uint32_t RESERVED6;            // 0x74

    volatile uint32_t APB1SMENR1;  // 0x78
    volatile uint32_t APB1SMENR2;  // 0x7C
    volatile uint32_t APB2SMENR;   // 0x80
    uint32_t RESERVED7;            // 0x84

    volatile uint32_t CCIPR;       // 0x88
    uint32_t RESERVED8;            // 0x8C
    volatile uint32_t BDCR;        // 0x90
    volatile uint32_t CSR;         // 0x94
    volatile uint32_t CRRCR;       // 0x98
    volatile uint32_t CCIPR2;      // 0x9C
}; 
#define RCC ((struct rcc *)0x40021000)

static inline void systick_init(uint32_t ticks) { // stm32l4 runs at 4MHz
  if ((ticks - 1) > 0xffffff) return;  // Systick timer is 24 bit
  SYSTICK->LOAD = ticks - 1;
  SYSTICK->VAL = 0;
  SYSTICK->CTRL = BIT(0) | BIT(1) | BIT(2);  // Enable systick
  RCC->APB2ENR |= BIT(0);                   // Enable SYSCFG, bit 0 for smt32l4
}

struct gpio {
  volatile uint32_t MODER, OTYPER, OSPEEDR, PUPDR, IDR, ODR, BSRR, LCKR, AFR[2];
    // MODER - GPIO functioninality, GPIO -> MDOER
    // OTYPER - output type register, determines if a pin functions as push-pull or open drain output
    // OSPEEDR - controls rise and fall times/speed of output or alternate function pins. 
    // PUPDR - Pulllup and pulldown register, used to contorl if there are itnernal resistors active in pins
    // IDR - input datat register, used to determine logic high or logic low on pins
    // ODR - output data register, sets logical output (HIGH/LOW)
    // BSRR - bit set/reset register, allows to reset or set pins without corruption 
    // LCKR - GPIO port configuration LoCK Register, used to reset configs until next reset
    // AFR[] - GPIO Alternate Function Registers, used to map peripheral such as UART/SPI pin to GPIO 
};
#define GPIO(bank) ((struct gpio *)(0x48000000u + 0x400u * (bank)))// 0x48e6 is stm32L4 addy 

// Enum values are per datasheet: 0, 1, 2, 3
enum {GPIO_MODE_INPUT, GPIO_MODE_OUTPUT, GPIO_MODE_AF, GPIO_MODE_ANALOG };

static inline void gpio_set_mode(uint16_t pin, uint8_t mode) {
  struct gpio *gpio = GPIO(PINBANK(pin));  // GPIO bank
  int n = PINNO(pin);                      // Pin number
  RCC->AHB2ENR |= BIT(PINBANK(pin));
  gpio->MODER &= ~(3U << (n * 2));         // Clear existing setting
  gpio->MODER |= (mode & 3) << (n * 2);    // Set new mode
}

static inline void gpio_set_af(uint16_t pin, uint8_t af_num) {
  struct gpio *gpio = GPIO(PINBANK(pin));  // GPIO bank
  int n = PINNO(pin);                      // Pin number
  gpio->AFR[n >> 3] &= ~(15UL << ((n & 7) * 4)); // clears
  gpio->AFR[n >> 3] |= ((uint32_t) af_num) << ((n & 7) * 4); // sets 
}

static inline void gpio_write(uint16_t pin, bool val) {
  struct gpio *gpio = GPIO(PINBANK(pin));
  gpio->BSRR = (1U << PINNO(pin)) << (val ? 0 : 16);
}

struct usart{
  volatile uint32_t CR1, CR2, CR3, BRR, GTPR, RTOR, RQR, ISR, ICR, RDR, TDR; 
};
// control register 1-3, baud rate reg, guard time prescaler, receive timeout reg, request reg, interrupt and statis reg
// interrupt flag clear reg, receive data reg, transmit data reg. 

#define USART1 ((struct usart *) 0x40013800u)
#define USART2 ((struct usart *) 0x40004400u)
#define USART3 ((struct usart *) 0x40004800u)
#define LPUART1 ((struct usart *) 0x40008000u) // low power UART, asyn only, deep sleep

 // CPU frequency, 4 Mhz
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

// t: expiration time, prd: period, now: current time. Return true if expired
bool timer_expired(uint32_t *t, uint32_t prd, uint32_t now) {
  if (now + prd < *t) *t = 0;                    // Time wrapped? Reset timer
  if (*t == 0) *t = now + prd;                   // First poll? Set expiration
  if (*t > now) return false;                    // Not expired yet, return
  *t = (now - *t) > prd ? now + prd : *t + prd;  // Next expiration time
  return true;                                   // Expired, return true
}