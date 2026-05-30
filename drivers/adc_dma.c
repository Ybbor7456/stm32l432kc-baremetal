#include "drivers/adc_dma.h"

void start_adc(){
  RCC->CCIPR = (RCC->CCIPR & ~(3U << 28)) | (3U << 28); // set ADC clock source, RCC->APB2ENR is not enough 
  //printf("begin adc start \r\n"); 
  ADC1->CR &= ~(BIT(29)); // disable deep power down mode
  ADC1-> CR |= BIT(28); // enable ADC V regulator 
  for (volatile int i = 0; i < 200000; i++) (void)0;
  //printf("timer done\r\n");
  if (ADC1->CR & BIT(0)) {           // ADEN
    ADC1->CR |= BIT(1);              // ADDIS
    while (ADC1->CR & BIT(0)) (void)0;
  }
 
  // Make sure no conversion is running
  if (ADC1->CR & BIT(2)) {           // ADSTART
    ADC1->CR |= BIT(4);              // ADSTP
    while (ADC1->CR & BIT(4)) (void)0;
    while (ADC1->CR & BIT(2)) (void)0;
  }
  ADC1->CR &= ~BIT(30);
  ADC1->CR |= BIT(31);               // ADCAL=1
  //printf("cal: started\r\n");
  
  //printf("ADC1 CR before cal:  0x%08lX\r\n", ADC1->CR);
  //printf("ADC1 ISR: 0x%08lX\r\n", ADC1->ISR);
  while (ADC1->CR & BIT(31)) (void)0;
  
  //printf("cal complete \r\n"); 
  ADC1->CR |= BIT(0);
}

void dma_ch1_setup(uint8_t request_id, volatile uint16_t *dst) {
  //printf("DMA ch1setup test begin \r\n"); 
  dma_adc_clock_enable();
  dma1_ch1_disable();                // EN=0 before touching config
  DMA1->IFCR = 0xFU << 0;            // clear CH1: bits 0..3
  DMA1->CSELR = (DMA1->CSELR & ~(0xFU << 0)) | ((request_id & 0xFU) << 0);

  // set dma peripheral address, memory address, number of data items to transfer
  DMA1_CH1->CPAR  = (uint32_t)&ADC1->DR;
  DMA1_CH1->CMAR  = (uint32_t)dst;
  DMA1_CH1->CNDTR = 1U;
  uint32_t ccr = DMA1_CH1->CCR;
  ccr &= ~(BIT(4) | BIT(5) | BIT(6) | BIT(7) | (3U << 8) | (3U << 10));

  ccr |= (1U << 8);
  ccr |= (1U << 10);
  DMA1_CH1->CCR = ccr; 
 // printf(" DMA ch1setup test end \r\n");
}

void adc_stable_calibration_state(){ // getting stuck here in main()
  //printf("test for adc cal state begin \r\n");
  // if a conversion is running, end it
  //printf("stop conversions test begin \r\n");
  if (ADC1->CR & BIT(2)) {              
    ADC1->CR |= BIT(4);
    while (ADC1->CR & BIT(4)) (void)0;  // waiting to avoid race conditons 
    while (ADC1->CR & BIT(2)) (void)0;
  } 
  //printf("stop conversions test end \r\n");
  // if ADC is enabled, disable it
  //printf("stop ADC enable test begin \r\n"); 
  if (ADC1->CR & BIT(0)) {              
    ADC1->CR |= BIT(1);
    while (ADC1->CR & BIT(0)) (void)0;
  }
  // printf("stop ADC enable test end \r\n"); 
  // callibrate 
  ADC1->CR |= BIT(31);
  //while (ADC1->CR & BIT(31)) (void)0;
  //printf("test for adc cal state end \r\n");
}

void adc_set_configs(uint8_t res_bit, bool left_align, bool conv_mode, uint16_t pin){
  // can only set res when adstart and jadstart are 0
 // printf("adc set configs start \r\n"); 
  if (ADC1->CR & BIT(2)) {              
    ADC1->CR |= BIT(4);
    while (ADC1->CR & BIT(4)) (void)0;     // wait ADSTP clears
    while (ADC1->CR & BIT(2)) (void)0;     // wait ADSTART clears
  } // stops the conv. 
  ADC1->CFGR &= ~(3U << 3U); 
  ADC1->CFGR |= ((res_bit & 3) << 3U); 
  if(left_align) ADC1->CFGR |= BIT(5); //align left
  else ADC1->CFGR &= ~BIT(5);
  if(conv_mode) ADC1->CFGR |= BIT(13);
  else ADC1->CFGR &= ~BIT(13); 
  ADC1->CFGR |= BIT(0); // setting DMA enable only (for now)
  ADC1->CFGR &= ~BIT(1); // set single-shot for now
  gpio_set_pupd(pin, NO_PULL); 
  //printf("adc set configs stop \r\n"); 
}

void adc_set_sequence(uint8_t len, uint8_t ch, uint8_t sample_rate){
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