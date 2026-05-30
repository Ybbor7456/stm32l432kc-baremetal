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

static inline void adc_gpio_init(uint16_t pin, uint8_t conf /*, uint8_t af_num */){ 
  gpio_enable(pin); 
  gpio_set_mode(pin, GPIO_MODE_ANALOG); 
  // gpio_set_af(pin, af_num); 
  gpio_set_pupd(pin, conf);
}

static inline void dma_adc_clock_enable(){
  RCC->AHB1ENR |= BIT(0); // sets 1 to bit 0
  RCC->AHB2ENR |= BIT(13);  
}

static inline void dma1_ch1_disable(void) {
  DMA1_CH1->CCR &= ~BIT(0);           // EN = 0
}

static inline void dma1_ch1_enable(void) {
  //printf("channel enable start \r\n");
  DMA1_CH1->CCR |= BIT(0);            // EN = 1
  //printf("channel enable stop \r\n");
}

void dma_ch1_setup(uint8_t request_id, volatile uint16_t *dst);

void adc_stable_calibration_state();

void adc_set_configs(uint8_t res_bit, bool left_align, bool conv_mode, uint16_t pin);

void adc_set_sequence(uint8_t len, uint8_t ch, uint8_t sample_rate);

void start_adc();