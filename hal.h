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

struct i2c{
  volatile uint32_t CR1, CR2, OAR1, OAR2, TIMINGR, TIMEOUTR, ISR, ICR, PECR, RXDR, TXDR; 
}; 
/* control reg 1-3, Own address reg 1&2, timing register, timeout register,interrupt and status
  interrupt clear, packet error checking register, receive data, transmit data. 
*/

#define USART1 ((struct usart *) 0x40013800u)
#define USART2 ((struct usart *) 0x40004400u)
#define USART3 ((struct usart *) 0x40004800u)
#define LPUART1 ((struct usart *) 0x40008000u) // low power UART, asyn only, deep sleep

#define I2C1   ((struct i2c *) 0x40005400u)
// stm32l432kc does not have I2C2, I2C3 is for low power, having clock to wakeup from deep sleep. 
#define I2C3   ((struct i2c *) 0x40005C00u)

static inline void gpio_set_pullup(uint16_t pin) {
  struct gpio *gpio = GPIO(PINBANK(pin));
  int n = PINNO(pin);
  gpio->PUPDR &= ~(3U << (n * 2));      // clear
  gpio->PUPDR |=  (1U << (n * 2));      // 01 pullup, 00 no PU or PD, 10 ulldown, 11 resserved
}

static inline void gpio_set_speed(uint16_t pin) {
  struct gpio *gpio = GPIO(PINBANK(pin));
  int n = PINNO(pin);
  gpio->OSPEEDR &= ~(3U << (n * 2));    // clear
  gpio->OSPEEDR |=  (3U << (n * 2));    // 11 = (very) high, 10 is high, 01 medium, 00 slow
}

static inline void gpio_set_open_drain(uint16_t pin) {
  struct gpio *g = GPIO(PINBANK(pin));
  int n = PINNO(pin);
  g->OTYPER |= BIT(n);              // 1 = open-drain
}

static inline void i2c_gpio_init(uint16_t scl, uint16_t sda, uint8_t af) {
  gpio_set_mode(scl, GPIO_MODE_AF);
  gpio_set_af(scl, af);
  gpio_set_open_drain(scl);
  //gpio_set_pullup(scl);            //uses external pull-ups
  gpio_set_speed(scl);

  gpio_set_mode(sda, GPIO_MODE_AF);
  gpio_set_af(sda, af);
  gpio_set_open_drain(sda);
  //gpio_set_pullup(sda);            //external pull-ups
  gpio_set_speed(sda);
}

static inline void rcc_i2c_select_hsi(struct i2c *i2c) {
  // HSI ison as a peripheral clock
  RCC->CR |= BIT(8);  // bit 8 HSION 

  if (i2c == I2C1) {
    RCC->CCIPR = (RCC->CCIPR & ~(3U << 12)) | (2U << 12); // target, clear, and set bits 12 and 13 to set 10 to target HSI as lcock
  } else if (i2c == I2C3) {
    RCC->CCIPR = (RCC->CCIPR & ~(3U << 16)) | (2U << 16); 
  }
}

static inline int i2c_write(struct i2c *i2c, uint8_t addr7, const uint8_t *buf, size_t len) {
  // checks if BUS is busy
  if (i2c->ISR & BIT(15)) return I2C_ERR_BUSY;

  i2c->ICR = BIT(5) | BIT(4) | BIT(8) | BIT(9); // writing 1 to these clears error flags 

  // For writes longer than 255 bytes, use RELOAD and wait for TCR between chunks.
  // TCR - Transfer Control Reload, register that works specifically with the RELOAD bit to handle data transfers larger than 255 bytes
  while (len > 0) {
    uint32_t chunk = (len > 255) ? 255 : (uint32_t)len;
    uint32_t last  = (len <= 255);

    // Build CR2: 7-bit address goes into SADD field shifted by 1 on STM32 I2C v2.
    //SADD holds the address in bits [7:1]; bit 0 is 0 for 7-bit addressing
    uint32_t cr2 = 0;
    cr2 |= ((uint32_t)(addr7 << 1) << 0);   
    cr2 |= (chunk << 16);
    // RD_WRN = 0 for write
    if (!last) cr2 |= BIT(24);          // more bytes to come
    if (last)  cr2 |= BIT(25);         // auto STOP after last byte
    cr2 |= BIT(13);                      // generate START
    // Write CR2
    i2c->CR2 = cr2;
    // Send this chunk
    for (uint32_t i = 0; i < chunk; i++) {
      // Wait until TXDR is ready (TXIS) or error
      uint32_t timeout = 1000000;
      while (((i2c->ISR & BIT(1)) == 0)) {
        uint32_t isr = i2c->ISR;

        if (isr & BIT(4)) {
          i2c->ICR = BIT(4);
          i2c->CR2 |= BIT(14);
          return I2C_ERR_NACK;
        }
        if (isr & BIT(8)) {
          i2c->ICR = BIT(8);
          i2c->CR2 |= BIT(14);
          return I2C_ERR_BUS;
        }
        if (isr & BIT(9)) {
          i2c->ICR = BIT(9); 
          return I2C_ERR_ARLO;
        }
        if (timeout-- == 0) {
          i2c->CR2 |= BIT(14); // bit(14) cr2_STOP
          return I2C_ERR_TIMEOUT;
        }
      }
      // Write next byte
      i2c->TXDR = *buf++;
  }
    if (!last) {
      // Wait for TCR
      int rc = i2c_wait_flag_set(&i2c->ISR, BIT(7), 1000000);
      if (rc != I2C_OK) {
        i2c->CR2 |= BIT(14);
        return rc;
        }
      } 
      else {  
      // Wait for STOPF
      int rc = i2c_wait_flag_set(&i2c->ISR, BIT(5), 1000000);
      if (rc != I2C_OK) {
        i2c->CR2 |= BIT(14);
        return rc;
      }
      // Clear STOPF
      i2c->ICR = BIT(5);
    }
    len -= chunk;
  }
  return I2C_OK;
}

static inline int i2c_read(struct i2c *i2c, uint8_t addr7, uint8_t *buf, size_t len ){
  // checks if BUS is busy and clear flags
  if (i2c->ISR & BIT(15)) return I2C_ERR_BUSY;
  i2c->ICR = BIT(5) | BIT(4) | BIT(8) | BIT(9); // writing 1 to these clears error flags 

  while (len > 0) {
    uint32_t chunk = (len > 255) ? 255 : (uint32_t)len;
    uint32_t last  = (len <= 255);
    //2. set CR2 (SADD, RD_WRN=1, NBYTES, START, AUTOEND/RELOAD
    
    uint32_t cr2 = 0;
    cr2 |= ((uint32_t)(addr7 << 1));   // set SADD
    cr2 |= (chunk << 16);                   // NBYTES = chunk
    cr2 |= BIT(10);               // set RD_WRN
    if (!last) cr2 |= BIT(24);         
    if (last)  cr2 |= BIT(25);         // auto STOP after last byte
    cr2 |= BIT(13); // set START
    i2c->CR2 = cr2;
    for (uint32_t i = 0; i < chunk; i++) {
      uint32_t timeout = 1000000; 
      while (((i2c->ISR & BIT(2)) == 0)) { // BIT(1) is TXIS, change from  to TXIS to BIT(2) RXNE
        uint32_t isr = i2c->ISR;
        if (isr & BIT(4)) {
          i2c->ICR = BIT(4);
          i2c->CR2 |= BIT(14);
          return I2C_ERR_NACK;
        }
        if (isr & BIT(8)) {
          i2c->ICR = BIT(8);
          i2c->CR2 |= BIT(14);
          return I2C_ERR_BUS;
        }
        if (isr & BIT(9)) {
          i2c->ICR = BIT(9); 
          return I2C_ERR_ARLO;
        }
        if (timeout-- == 0) {
          i2c->CR2 |= BIT(14); // bit(14) cr2_STOP
          return I2C_ERR_TIMEOUT;
        }
      }
    // read next byte
    *buf++ = (uint8_t)i2c->RXDR;
    }
    if (!last) {
      // Wait for transfer complete reload
      int rc = i2c_wait_flag_set(&i2c->ISR, BIT(7), 1000000);
      if (rc != I2C_OK) {
        i2c->CR2 |= BIT(14);
        return rc;
      }
    } 
    else {  
      // Wait for STOPF (AUTOEND generates STOP automatically)
      int rc = i2c_wait_flag_set(&i2c->ISR, BIT(5), 1000000);
      if (rc != I2C_OK) {
        i2c->CR2 |= BIT(14);
        return rc;
      }
      // Clear STOPF
      i2c->ICR = BIT(5);
    }
    len -= chunk;
  }
  return I2C_OK;
}


static inline void i2c_init(struct i2c *i2c){
  uint af = 0; 
  uint16_t scl = 0, sda =0; 

  if (i2c == I2C1) RCC -> APB1ENR1 |= BIT(21); 
  if (i2c == I2C3) RCC -> APB1ENR1 |= BIT(23); 

  if (i2c == I2C1) af = 4, sda = PIN('B', 7), scl = PIN('B', 6); 
  if (i2c == I2C3) af = 4, sda = PIN('B', 4), scl = PIN('A', 7); 

  i2c_gpio_init(scl, sda, af); 
  rcc_i2c_select_hsi(i2c);
  i2c->CR1 &= ~BIT(0); 
  i2c->TIMINGR = 0x00503D58; // taken from CubemX, TIMINGR determines speed, not source
  i2c -> CR1 |= BIT(0); // peripheral disabled, PE = 0
}


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
static inline bool timer_expired(uint32_t *t, uint32_t prd, uint32_t now) {
  if (now + prd < *t) *t = 0;                    // Time wrapped? Reset timer
  if (*t == 0) *t = now + prd;                   // First poll? Set expiration
  if (*t > now) return false;                    // Not expired yet, return
  *t = (now - *t) > prd ? now + prd : *t + prd;  // Next expiration time
  return true;                                   // Expired, return true
}
