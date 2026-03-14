// main.c functions 
#include "fnxs.h"
#include <stdio.h>
#include "drivers/hal.h"
#include "drivers/gpio.h"
#include "drivers/adc_dma.h"
#include "drivers/stm32l4_regs.h"
#include "drivers/spi.h"

void uart_led(uint32_t *timer, uint32_t period, uint32_t ticks, uint16_t led){
    if (timer_expired(timer, period, s_ticks)) {
        static bool on;       // This block is executed
        gpio_write(led, on);  // Every `period` milliseconds
        on = !on;             // Toggle LED state
        printf("LED: %d, tick: %lu\r\n", on, ticks);
    } 
}

void led_on_off(volatile uint8_t *btn_event, uint16_t led, bool *led_on, uint32_t ticks){
    if (*btn_event) {
        *btn_event = 0;
        *led_on = !*led_on;
        gpio_write(led, *led_on);
        printf("Button! LED=%d tick=%lu\r\n", *led_on, (unsigned long)ticks);
    }
}

void adc_read(uint32_t *t, uint32_t ticks, uint16_t volatile *sample){
    if (timer_expired(t, 1000, ticks)) {          // every 100 ms
        //printf("Step A: \r\n");
        dma1_ch1_disable();
        DMA1->IFCR = 0xFU << 0;                        // clear CH1 flags
        DMA1_CH1->CNDTR = 1U;                          // reload count
        dma1_ch1_enable();
        /*
        printf("CR=%08lX ISR=%08lX SQR1=%08lX SMPR1=%08lX CFGR=%08lX DR=%lu\r\n",
        ADC1->CR, ADC1->ISR, ADC1->SQR1, ADC1->SMPR1, ADC1->CFGR, (unsigned long)ADC1->DR); 
         Start ADC conversion (ADC must already be enabled + ready)
        */
        ADC1->CR |= BIT(2);      // ADSTART
        // Wait for DMA transfer complete
        while (!(DMA1->ISR & BIT(1))) (void)0;
        DMA1->IFCR = BIT(1);                    // clear TCIF1
        // printf("ADC1 CR:  0x%08lX\r\n", ADC1->CR);
        // printf("ADC1 ISR:  0x%08lX\r\n", ADC1->ISR);
        printf("ADC Sample: %u\r\n", (unsigned)*sample);
    } 
}

void reg_lights(uint16_t ssel){
    shiftreg_write(0x00, ssel);    // 0000
    delay_ms(250);
    shiftreg_write(0x01, ssel); // 0001
    delay_ms(250); 
    shiftreg_write(0x04, ssel); // 0100
    delay_ms(250);
    shiftreg_write(0x09, ssel); // 1001 
    delay_ms(250); 
    shiftreg_write(0x05, ssel);    // 0101
    delay_ms(250);  
    shiftreg_write(0xFF, ssel);    // 1111
    delay_ms(250); 
    shiftreg_write(0x06, ssel);    // 0110
    delay_ms(250); 
    shiftreg_write(0x0A, ssel);   // 1010
    delay_ms(250); 
    shiftreg_write(0x0B, ssel);   // 1011
    delay_ms(250); 
}
    
