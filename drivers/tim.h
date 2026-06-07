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
#include "drivers/gpio.h"


static inline void tim2_init(void){
    RCC->APB1ENR1 |= BIT(0); // enable TIM2 
    // psc = (fck_psc/Fcnt) - 1. stm32l4 4MHz MSI clock, 4/Fcnt - 1 = 79. Fcnt = 1MHz. 
    // ARR [ Fcnt/reload) - 1, 1MHz/1000s -1 = 9999
    TIM2->CR1 = 0;
    TIM2->CNT = 0;
    TIM2->PSC = 3;
    TIM2->ARR = 999;
    TIM2->EGR |= BIT(0); // update gen. 
    TIM2->SR &= ~BIT(0); // clear SR UIF (UIF is automatically set)
    TIM2->CR1 |= BIT(0); // timer enable
}

static inline void tim2_wait(void) {
    while ((TIM2->SR & BIT(0)) == 0) {
    }
    TIM2->SR &= ~BIT(0);
}

static inline void tim2_delay_ms(uint16_t tim2_ms){ // 2^-16) -1 = 65535 ms max, 66(ish) seconds
    while(tim2_ms--){
        tim2_wait();
    }
}

static inline void tim2_test(void){
    uint32_t a = TIM2->CNT;
    printf("Time A: %ld \r\n", a); 
    tim2_delay_ms(100); 
    uint32_t b = TIM2->CNT;
    printf("Time B: %ld \r\n", b); 
    tim2_delay_ms(100); 
}

// start timer, stop timer, reset timer, 
static inline void tim2_start(void){
    TIM2->CR1 |= BIT(0); 
}

static inline void tim2_stop(void){
    TIM2->CR1 &= ~BIT(0); 
}

static inline void tim2_reset(void){
    TIM2->CNT = 0;
    TIM2->SR &= ~BIT(0);
}

static inline uint32_t tim2_get_count(void){
    return TIM2->CNT;
}

static inline void tim2_clear_uif(void){
    TIM2->SR &= ~BIT(0);
}

static inline void tim2_pwm_gpio_init(uint16_t pin){
    gpio_set_mode(pin, GPIO_MODE_AF);
    gpio_set_af(pin, 1);
    gpio_set_otype(pin, 0);
    gpio_set_select_speed(pin, GPIO_VHIGH);
    gpio_set_pupd(pin, NO_PULL); 
}

void tim2_pwm_init(uint16_t pin, uint8_t ch_num);

static inline void tim2_pwm_set_duty(uint32_t duty) {
    if (duty > 100) duty = 100; 
    TIM2->CCR1 = (duty * (TIM2->ARR + 1)) / 100;
}

// add ISR increments to be non-blocking



static inline void tim6_init(void){
    RCC->APB1ENR1 |= BIT(4); // enable TIM6 
    // psc = (fck_psc/Fcnt) - 1. stm32l4 4MHz MSI clock, 4/Fcnt - 1 = 79. Fcnt = 1MHz. 
    // ARR [ Fcnt/reload) - 1, 1MHz/1000s -1 = 9999
    TIM6->CR1 = 0;
    TIM6->CNT = 0;
    TIM6->PSC = 3;
    TIM6->ARR = 999;
    TIM6->EGR |= BIT(0); // update gen. 
    TIM6->SR &= ~BIT(0); // clear SR UIF (UIF is automatically set)
    TIM6->CR1 |= BIT(0); // timer enable
}

static inline void tim6_wait(void) {
    while ((TIM6->SR & BIT(0)) == 0) {
    }
    TIM6->SR &= ~BIT(0);
}

static inline void tim6_delay_ms(uint16_t tim6_ms){ // 2^-16) -1 = 65535 ms max, 66(ish) seconds
    while(tim6_ms--){
        tim6_wait();
    }
}

static inline void tim6_test(void){
    uint32_t a = TIM6->CNT;
    printf("Time A: %ld \r\n", a); 
    tim6_delay_ms(100); 
    uint32_t b = TIM6->CNT;
    printf("Time B: %ld \r\n", b); 
    tim6_delay_ms(100); 
}

// start timer, stop timer, reset timer, 
static inline void tim6_start(void){
    TIM6->CR1 |= BIT(0); 
}

static inline void tim6_stop(void){
    TIM6->CR1 &= ~BIT(0); 
}

static inline void tim6_reset(void){
    TIM6->CNT = 0;
    TIM6->SR &= ~BIT(0);
}

static inline uint32_t tim6_get_count(void){
    return TIM6->CNT;
}

static inline void tim6_clear_uif(void){
    TIM6->SR &= ~BIT(0);
}

static inline void tim6_trgo_init(uint32_t psc, uint32_t arr){
    RCC->APB1ENR1 |= BIT(4);    // TIM6 clock enable
    TIM6->CR1 = 0;              // counter disabled while configuring
    TIM6->CNT = 0;
    TIM6->PSC = psc;
    TIM6->ARR = arr;
    TIM6->CR2 &= ~(0x7u << 4);
    TIM6->CR2 |=  (0x2u << 4);  // MMS = 010: update event as TRGO
    TIM6->EGR = BIT(0);         // load PSC/ARR immediately
    TIM6->SR &= ~BIT(0);        // clear UIF
    TIM6->CR1 |= BIT(0);        // CEN start time
}