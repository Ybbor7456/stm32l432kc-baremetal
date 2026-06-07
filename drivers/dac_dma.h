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



/*
Part 1: DAC basic DC output
    write one 12-bit value manually to DAC_DHR12R1

Part 2: DAC software-triggered output
    update DAC value in a loop

Part 3: DAC timer-triggered output
    timer controls when DAC updates

Part 4: DAC + DMA circular waveform
    memory table automatically feeds DAC conf
*/

static inline void dac_gpio_init(uint16_t pin, uint8_t conf){
  gpio_enable(pin); 
  gpio_set_mode(pin, GPIO_MODE_ANALOG); 
  gpio_set_pupd(pin, conf);
}

static inline void dac_clock_enable(void){
    //RCC->AHB1ENR |= BIT(1); // DMA2 enable
    RCC->APB1ENR1 |= BIT(29); // DAC1 enable
}

static inline void dac_disable_ch1(void) {
    DAC1->CR &= ~BIT(0);
}

static inline void dac_enable_ch1(void) {
    DAC1->CR |= BIT(0); // EN1
}

static inline void dac_ch1_normal_mode(void){
    // calibration mode and DAC must not be enabled to write bits.
    DAC1->CR &= ~BIT(14); // turn off calibration ch 1
    dac_disable_ch1(); // turn off DAC
    DAC1->MCR &= ~(0x7u << 0); 
    //DAC->MCR |= (0u << 0); //000: DAC channel1 is connected to external pin with Buffer enabled
    //DAC->CR |= BIT(14); re-enable calib mode
}

static inline void dac_ch1_write_12bit(uint16_t value) {
    DAC1->DHR12R1 = value & 0x0FFFu; // if Vref is 3.3, 2048 / 4095 * 3.3V ≈ 1.65V, 0x0FFFu = 4095
    // uint32_t actual = DAC->DOR1; // dont write to DOR1, use as sanity test. DOR1 reflects value of DHR12r1
}   

static inline void dac_init_test(uint16_t pin) {
    dac_clock_enable();
    dac_gpio_init(pin, NO_PULL);
    dac_ch1_normal_mode();
    dac_enable_ch1();
    dac_ch1_write_12bit(2048);
}

static inline void dac_trigger_select_tim6(void){
    DAC1->CR &= ~(0x7u <<3); 
    DAC1->CR |= BIT(2); 
}

static inline void dac_dma_enable_ch1(void){
    DAC1->CR |= BIT(12); // DMAEN1
}

static inline void dac_dma3_setup(uint16_t *buf, uint16_t len){
    RCC->AHB1ENR |= BIT(0); // clk en
    DMA1_CH3->CCR = 0; // disable channel
    *DMA1_CSELR = (*DMA1_CSELR & ~(0xFu << 8)) | (6u << 8); 

    DMA1_CH3->CPAR = (uint32_t) &DAC1->DHR12R1; 
    DMA1_CH3->CMAR = (uint32_t) buf; 
    DMA1_CH3->CNDTR = len; 

    DMA1_CH3->CCR = (1 << 4) | (1u << 5) | (1u << 7) | (1u << 8) | (1u <<10); 
    /*1-4 memory to peripheral
        5, 6circular mode
        7 memory increment
        8, 9 01 16-bit peripheral transfer
        10 - 1 16-bit memory transfer
    */
    DMA1_CH3->CCR |= BIT(0); // enable channel 
}

static inline void dac_dma_ch1_tim6_trigger_init(uint16_t pin, uint16_t *buf, uint16_t len){
    dac_clock_enable();
    dac_gpio_init(pin, NO_PULL);
    dac_ch1_normal_mode();
    dac_trigger_select_tim6();
    dac_dma3_setup(buf, len); 
    dac_dma_enable_ch1(); 
    dac_enable_ch1();
}