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


#include "gpio.h"
#include "uart.h"
#include "i2c.h"
#include "interrupt.h"
// #include "exti.h" combined with itnerrupt.h
#include "adc_dma.h"
#include "spi.h"

// uint32_t hal_millis(void);
static volatile uint32_t s_ticks;
extern volatile uint8_t  g_btn_event;
static volatile uint32_t g_btn_last_ms;

static inline void systick_init(uint32_t ticks) { // stm32l4 runs at 4MHz
  if ((ticks - 1) > 0xffffff) return;  // Systick timer is 24 bit
  SYSTICK->LOAD = ticks - 1;
  SYSTICK->VAL = 0;
  SYSTICK->CTRL = BIT(0) | BIT(2);  // Enable systick (remove BIT(1))
  RCC->APB2ENR |= BIT(0);                   // Enable SYSCFG, bit 0 for smt32l4
}

enum{
  sixbit = 3, eightbit = 2, tenbit = 1, twelvebit = 0
};

// t: expiration time, prd: period, now: current time. Return true if expired
static inline bool timer_expired(uint32_t *t, uint32_t prd, uint32_t now) {
  if (now + prd < *t) *t = 0;                    // Time wrapped? Reset timer
  if (*t == 0) *t = now + prd;                   // First poll? Set expiration
  if (*t > now) return false;                    // Not expired yet, return
  *t = (now - *t) > prd ? now + prd : *t + prd;  // Next expiration time
  return true;                                   // Expired, return true
}

static inline uint32_t hal_millis(void) {
  return s_ticks;
}

static inline void delay_ms(uint32_t ms) {
  uint32_t start = hal_millis();
  while ((hal_millis() - start) < ms) {
  }
}

void EXTI9_5_IRQHandler(void);

