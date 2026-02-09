#pragma once

#include <inttypes.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <math.h> 

#include "drivers/util.h"
#include "drivers/stm32l4_regs.h"
#include "bsp/board.h"

static inline void systick_init(uint32_t ticks) { // stm32l4 runs at 4MHz
  if ((ticks - 1) > 0xffffff) return;  // Systick timer is 24 bit
  SYSTICK->LOAD = ticks - 1;
  SYSTICK->VAL = 0;
  SYSTICK->CTRL = BIT(0) | BIT(1) | BIT(2);  // Enable systick
  RCC->APB2ENR |= BIT(0);                   // Enable SYSCFG, bit 0 for smt32l4
}

// Enum values are per datasheet: 0, 1, 2, 3
enum {GPIO_MODE_INPUT, GPIO_MODE_OUTPUT, GPIO_MODE_AF, GPIO_MODE_ANALOG };

enum { // debug values 
  I2C_OK = 0, I2C_ERR_BUSY = -1, I2C_ERR_NACK = -2, I2C_ERR_BUS = -3, I2C_ERR_ARLO = -4, I2C_ERR_TIMEOUT = -5
};

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

static inline bool gpio_read_pin(uint16_t pin) {
  struct gpio *gpio = GPIO(PINBANK(pin));
  return (gpio->IDR & BIT(PINNO(pin))) != 0;
}

static inline bool i2c_bus_idle(uint16_t scl, uint16_t sda) {
  return gpio_read_pin(scl) && gpio_read_pin(sda);
}

static inline int i2c_wait_flag_set(volatile uint32_t *reg, uint32_t mask, uint32_t timeout) {
  while (((*reg) & mask) == 0) {
    if (timeout-- == 0) return I2C_ERR_TIMEOUT;
  }
  return I2C_OK;
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
    // (SADD holds the address in bits [7:1]; bit0 is 0 for 7-bit addressing)
    uint32_t cr2 = 0;
    cr2 |= ((uint32_t)(addr7 << 1) << 0);   
    cr2 |= (chunk << 16);
    // RD_WRN = 0 for write
    if (!last) cr2 |= BIT(24);          // more bytes to come
    if (last)  cr2 |= BIT(25);         // auto STOP after last byte
    cr2 |= BIT(13);                      // generate START
    // Write CR2 (note: avoid accidentally leaving STOP/START bits set from earlier use)
    i2c->CR2 = cr2;
    // Send this chunk
    for (uint32_t i = 0; i < chunk; i++) {
      // Wait until TXDR is ready (TXIS), or error occurs
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
   // End-of-chunk handling
    if (!last) {
      // Wait for TCR (Transfer Complete Reload) before programming next chunk
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

static inline int i2c_read(struct i2c *i2c, uint8_t addr7, uint8_t *buf, size_t len ){
  // checks if BUS is busy
  // 1. Clear flags
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
    if (!last) cr2 |= BIT(24);          // more bytes to come
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
      // Wait for TCR (Transfer Complete Reload) before programming next chunk
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

//split into phase A and B
/*
The difference is that
an I2C Read command initiates a read from the current address pointer location
in the slave device's memory, while an I2C Read Register command first performs 
a write operation to set the internal register pointer to a specific address before performing the read operation

Phase A - set register pointer, device address + write, send register address byte, wait for TC

Phase B - read the payload. 
STOPF vs TC. 
TC means transfer of bytes complete, don't stop , STOPF -> action completed/AUTOEND

Phase A (write pointer): -> START -> 0x68 + W -> 0x0F ← this is the register address byte -> (no STOP)
Phase B (read data): -> repeated START -> 0x68 + R -> byte0 ← goes into buf[0] -> byte1 ← goes into buf[1] -> STOP
*/

static inline int i2c_read_register(struct i2c *i2c, uint8_t addr7, uint8_t *buf, size_t len, uint8_t reg ){
  //busy check
  if (i2c->ISR & BIT(15)) return I2C_ERR_BUSY;
  i2c->ICR = BIT(5) | BIT(4) | BIT(8) | BIT(9); 
  uint32_t cr2 = 0;
  cr2 |= ((uint32_t)(addr7 & 0x7F) << 1); 
  cr2 |= (1U << 16); 
  cr2 |= BIT(13);  // START
  i2c->CR2 = cr2;
  uint32_t timeout = 1000000;
  while ((i2c->ISR & BIT(1)) == 0) { // TXIS wait on register address byte 
    uint32_t isr = i2c->ISR;
    if (isr & BIT(4)) {  // NACKF received flag
      i2c->ICR = BIT(4);  // NACKF flag cleared
      i2c->CR2 |= BIT(14); // STOP
      return I2C_ERR_NACK;
    }
    if (isr & BIT(8)) { // BERR bus error
      i2c->ICR = BIT(8); // bus error flag clear
      i2c->CR2 |= BIT(14); // stop 
      return I2C_ERR_BUS;
    }
    if (isr & BIT(9)) {  // ARLO
      i2c->ICR = BIT(9); // clears arbitration flag
      return I2C_ERR_ARLO;
    }
    if (timeout-- == 0) {
      i2c->CR2 |= BIT(14); // STOP
      return I2C_ERR_TIMEOUT;
    }
  }
  // wait TC (transfer complete, no STOPF) so there is a repeated START
  i2c->TXDR = reg; // writes register address byte into TXDR
  int rc = i2c_wait_flag_set(&i2c->ISR, BIT(6), 1000000); // transfer complete condition 
  if (rc != I2C_OK) { // if not TC, stop condition generation 
      i2c->CR2 |= BIT(14);
      return rc;
  }
  //Phase B: repeated START + read len bytes
  //Phase B is the actual data coming from the device starting at that register.
  while (len > 0) {
    uint32_t chunk = (len > 255) ? 255 : (uint32_t)len;
    uint32_t last  = (len <= 255);

    cr2 = 0;
    cr2 |= ((uint32_t)(addr7 & 0x7F) << 1); // SADD
    cr2 |= (chunk << 16);                   // NBYTES
    cr2 |= BIT(10);                         // RD_WRN = 1, read
    if (!last) cr2 |= BIT(24);              // RELOAD
    if (last)  cr2 |= BIT(25);              // AUTOEND
    cr2 |= BIT(13);                         // STARTx2
    i2c->CR2 = cr2;
    // read chunk bytes
    for (uint32_t i = 0; i < chunk; i++) {
      timeout = 1000000;
      while ((i2c->ISR & BIT(2)) == 0) {    // RXNE
        uint32_t isr = i2c->ISR;
        if (isr & BIT(4)) {                 // NACKF
          i2c->ICR = BIT(4);
          i2c->CR2 |= BIT(14);
          return I2C_ERR_NACK;
        }
        if (isr & BIT(8)) {                 // BERR
          i2c->ICR = BIT(8);
          i2c->CR2 |= BIT(14);
          return I2C_ERR_BUS;
        }
        if (isr & BIT(9)) {                 // ARLO
          i2c->ICR = BIT(9);
          return I2C_ERR_ARLO;
        }
        if (timeout-- == 0) {
          i2c->CR2 |= BIT(14);
          return I2C_ERR_TIMEOUT;
        }
      }
      *buf++ = (uint8_t)i2c->RXDR;
    }
        // end-of-chunk
    if (!last) {
      rc = i2c_wait_flag_set(&i2c->ISR, BIT(7), 1000000); // TCR
      if (rc != I2C_OK) {
        i2c->CR2 |= BIT(14);
        return rc;
      }
    } else {
      rc = i2c_wait_flag_set(&i2c->ISR, BIT(5), 1000000); // STOPF
      if (rc != I2C_OK) {
        i2c->CR2 |= BIT(14);
        return rc;
      }
      i2c->ICR = BIT(5); // clear STOPF
    }
    len -= chunk;
  }
  return I2C_OK;
}

/*
START
device address + Write
register address byte (the pointer)
data byte(s)
STOP
*/
// helper
static inline int i2c_wait_txis_or_err(struct i2c *i2c, uint32_t timeout) {
  while ((i2c->ISR & BIT(1)) == 0) {            // TXIS
    uint32_t isr = i2c->ISR;

    if (isr & BIT(4)) {                         // NACKF
      i2c->ICR = BIT(4);
      i2c->CR2 |= BIT(14);                      // STOP
      return I2C_ERR_NACK;
    }
    if (isr & BIT(8)) {                         // BERR
      i2c->ICR = BIT(8);
      i2c->CR2 |= BIT(14);
      return I2C_ERR_BUS;
    }
    if (isr & BIT(9)) {                         // ARLO
      i2c->ICR = BIT(9);
      return I2C_ERR_ARLO;
    }
    if (timeout-- == 0) {
      i2c->CR2 |= BIT(14);
      return I2C_ERR_TIMEOUT;
    }
  }
  return I2C_OK;
}

static inline int i2c_write_register(struct i2c *i2c, uint8_t dev_addr7, const uint8_t *buf, size_t len, uint8_t reg){
  if (i2c->ISR & BIT(15)) return I2C_ERR_BUSY;

  // 1) Clear sticky flags (STOPF, NACKF, BERR, ARLO)
  i2c->ICR = BIT(5) | BIT(4) | BIT(8) | BIT(9);

  bool first = true;

  while (len > 0) {
    // payload bytes to send in this segment
    uint32_t payload_chunk = (uint32_t)len;
    uint32_t max_payload   = first ? 254u : 255u;      // reserve 1 byte for reg on first segment
    if (payload_chunk > max_payload) payload_chunk = max_payload;

    // total bytes this segment = reg (first only) + payload
    uint32_t nbytes = payload_chunk + (first ? 1u : 0u);

    // more payload remains after this segment?
    bool more = (len > payload_chunk);
    bool last = !more;

    // 2) Program CR2 for this WRITE segment
    uint32_t cr2 = 0;
    cr2 |= ((uint32_t)(dev_addr7 & 0x7F) << 1);        // SADD (7-bit)
    cr2 |= (nbytes << 16);                              // NBYTES
    // RD_WRN = 0 (write) by default
    if (more) cr2 |= BIT(24);                           // RELOAD
    else      cr2 |= BIT(25);                           // AUTOEND
    cr2 |= BIT(13);                                     // START
    i2c->CR2 = cr2;
    // 3) Send register pointer once (first segment only)
    if (first) {
      int rc = i2c_wait_txis_or_err(i2c, 1000000);
      if (rc != I2C_OK) return rc;
      i2c->TXDR = reg;
      first = false;
    }
    // 4) Send payload bytes
    for (uint32_t i = 0; i < payload_chunk; i++) {
      int rc = i2c_wait_txis_or_err(i2c, 1000000);
      if (rc != I2C_OK) return rc;
      i2c->TXDR = *buf++;
    }
    // 5) End-of-segment handling
    if (!last) {
      // wait TCR before programming next segment
      int rc = i2c_wait_flag_set(&i2c->ISR, BIT(7), 1000000); // TCR
      if (rc != I2C_OK) {
        i2c->CR2 |= BIT(14);
        return rc;
      }
    } else {
      // wait STOPF (AUTOEND)
      int rc = i2c_wait_flag_set(&i2c->ISR, BIT(5), 1000000); // STOPF
      if (rc != I2C_OK) {
        i2c->CR2 |= BIT(14);
        return rc;
      }
      i2c->ICR = BIT(5);                                     // clear STOPF
    }

    len -= payload_chunk;                                    // payload remaining
  }

  return I2C_OK;
}

static inline void i2c_init(struct i2c *i2c){
  uint8_t af = 0; 
  uint16_t scl = 0, sda =0; 

  if (i2c == I2C1) RCC -> APB1ENR1 |= BIT(21); // enable i2c1 clock
  if (i2c == I2C3) RCC -> APB1ENR1 |= BIT(23); 

  if (i2c == I2C1) af = 4, sda = PIN('B', 7), scl = PIN('B', 6); 
  if (i2c == I2C3) af = 4, sda = PIN('B', 4), scl = PIN('A', 7); 

  i2c_gpio_init(scl, sda, af); 
  rcc_i2c_select_hsi(i2c);
  i2c->CR1 &= ~BIT(0); //PE = 0
  i2c->TIMINGR = 0x00503D58; // taken from CubemX, TIMINGR determines speed, not source
  i2c -> CR1 |= BIT(0); // peripheral enabled, PE = 1
}


/* 
0. macros, structs
1 enable peripheral clocks, instead of BIT(#), use BIT(RCC_APB1ENR1_I2C1EN_Pos) for readability 
2. configure mode, AF, output type and output speed
3. fully reset i2c through CR
4. set peripheral clock frequency (CR)
5. set SCL frequency CCR, divide T_high or T_low by T_PCLK1 to get CCR. T-high/low = PCLK * CCR
6. set max rise time for CL, TRISE register
7. enable/disable I2C
8. BUS sanity checks (check IDR of GPIO ports assigned to I2C and checking status flags BUSY and RXNE)
9. primitive transactions, i2c_start_write(addr7, nbytes), i2c_start_read(addr7, nbytes), i2c_write_bytes(...)i2c_read_bytes(...) ,i2c_stop()
10. OR rely on AUTOEND instead of i2c_stop() , which stop i2c after designated byte numbers sent/read are finished
11. register write and reads
12. Error handling
13. device specific drivers 
*/


