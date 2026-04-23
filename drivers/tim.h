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

static inline void tim2_pwm_init(uint16_t pin, uint8_t ch_num){
    tim2_pwm_gpio_init(pin);

    if(ch_num == 1){
        TIM2->CCMR1 &= ~(3u << 0); // CC1s = 00 output
        TIM2->CCMR1 |= (1u << 11); // OC1PE
        TIM2->CCMR1 &= ~(7u << 4); // clear OC1M
        TIM2->CCMR1 |= (6u << 4); // PWM mode 1
        TIM2->CCER |= (1u << 0);  // CC1E enable
    }

    if(ch_num ==2){
        TIM2->CCMR1 &= ~(3u << 8); // CC2s = 00 output
        TIM2->CCMR1 |= (1u << 11); // OC2PE
        TIM2->CCMR1 &= ~(7u << 12); // clear OC2M
        TIM2->CCMR1 |= (6u << 12); // PWM mode 1
        TIM2->CCER |= (1u << 4);  // CC2E enable
    }
    TIM2->EGR |= BIT(0); // update event
}


// add ISR increments to be non-blocking