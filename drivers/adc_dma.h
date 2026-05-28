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

static inline void adc_set_sequence(uint8_t len, uint8_t ch, uint8_t sample_rate){
  //printf("set seq. start \r\n"); 
   if (ADC1->CR & BIT(2)) {              
    ADC1->CR |= BIT(4);
    while (ADC1->CR & BIT(4)) (void)0;     // wait ADSTP clears
    while (ADC1->CR & BIT(2)) (void)0;     // wait ADSTART clears
  } // stops the conv. 
  // clear and set conv bits
  ADC1->SQR1 = (ADC1->SQR1 & ~((0xFU << 0) | (0x1FU << 6)))
  | ((len & 0xFU) << 0)   // L[3:0]
  | ((ch & 0x1FU) << 6); // SQ1[4:0]
  
  // set sample time (ahrdcodes channel 6)
  ADC1->SMPR1 = (ADC1->SMPR1 & ~(0x7U << 18)) | ((sample_rate & 0x7U)<<18); 
  //printf("set seq. stop \r\n"); 
 // printf("adc_set_sequence args: len=%u ch=%u smp=%u\r\n", len, ch, sample_rate);
}

void start_adc();