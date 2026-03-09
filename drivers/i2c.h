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

enum { // debug values 
  I2C_OK = 0, I2C_ERR_BUSY = -1, I2C_ERR_NACK = -2, I2C_ERR_BUS = -3, I2C_ERR_ARLO = -4, I2C_ERR_TIMEOUT = -5
};

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