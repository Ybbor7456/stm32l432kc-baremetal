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

static inline bool i2c_bus_idle(uint16_t scl, uint16_t sda) {
  return gpio_read_pin(scl) && gpio_read_pin(sda);
}

void i2c_gpio_init(uint16_t scl, uint16_t sda, uint8_t af);
void rcc_i2c_select_hsi(struct i2c *i2c); 
int i2c_wait_flag_set(volatile uint32_t *reg, uint32_t mask, uint32_t timeout); 
int i2c_write(struct i2c *i2c, uint8_t addr7, const uint8_t *buf, size_t len); 
int i2c_read(struct i2c *i2c, uint8_t addr7, uint8_t *buf, size_t len); 
int i2c_read_register(struct i2c *i2c, uint8_t addr7, uint8_t *buf, size_t len, uint8_t reg);
int i2c_write_register(struct i2c *i2c, uint8_t dev_addr7, const uint8_t *buf, size_t len, uint8_t reg); 
void i2c_init(struct i2c *i2c);