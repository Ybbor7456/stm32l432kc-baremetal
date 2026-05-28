#include "drivers/tim.c"

void tim2_pwm_init(uint16_t pin, uint8_t ch_num){
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
