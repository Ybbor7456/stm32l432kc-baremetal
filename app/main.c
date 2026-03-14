#include "drivers/hal.h"
#include "bsp/board.h"
#include "drivers/stm32l4_regs.h"
#include "drivers/util.h"
#include "drivers/logger.h"
#include "drivers/gpio.h"
#include "drivers/interrupt.h"
#include "drivers/i2c.h"
#include "drivers/adc_dma.h"
#include "drivers/uart.h"
#include "drivers/spi.h"


static volatile uint32_t s_ticks; // volatile is important!!
void SysTick_Handler(void) {
  s_ticks++;
}

uint32_t hal_millis(void) {
  return s_ticks;
}
volatile uint8_t g_btn_event = 0;
void EXTI9_5_IRQHandler(void) // use this for button hardware, don't use static inlie
{
  // check which line (5..9) fired, e.g. line 7:
  if (EXTI->PR1 & BIT(7)) {
    EXTI->PR1 = BIT(7);     // clear pending by writing 1
    // button logic to add
    g_btn_event = 1; 
  }
}


static inline void delay_ms(uint32_t ms) {
  uint32_t start = hal_millis();
  while ((hal_millis() - start) < ms) {
  }
}

//  GPIOB port 3 has onboard LED 

int main(void) {
  uint16_t led = PIN('B', 3);            // Green LED, PIN('B' - A  << 8) | Num.... 0x100 | 3 = 0x103 = led
  uint16_t btn = PIN('B', 7);
  uint16_t adc = PIN('A', 1);     // adc PA1 ADC1_6

  uint16_t spi_sck = PIN('A', 5);
  uint16_t miso = PIN('A',6 );
  uint16_t mosi = PIN('A',7 ); 
  uint16_t ssel = PIN('B',0 ); 

  //const uint8_t af_num = 2; 
  const uint16_t request_ID = 0; // ADC1 channel 1, Table 45
  static volatile uint16_t adc_sample;
  uint8_t smp = 5; // 92.5,.... set enum for it later to change/reference it easier 
  uint8_t resolution = 0; // 12-bit
  uint8_t align = 0; // 0 if right
  uint8_t single = 0; // single conversion 
  

  // upper byte stores 01 = B, and lower stores 03 for LED
  //RCC->AHB2ENR |= BIT(PINBANK(led));     // Enable GPIO clock for LED, PINBANK(0x103) >> 8 = 0
  uart_init(USART2, 115200);
  for (volatile int i = 0; i < 100000; i++) (void)0;
 // printf("UART alive\r\n"); 
  systick_init(USART2_FCK / 1000);         // Tick every 1 ms
  exti_init(btn);
  nvic_set_priority(IRQ_EXTI9_5, 0x80); 
  nvic_enable_irq(IRQ_EXTI9_5);
  irq_global_enable(); 
 // printf("UART alive2\r\n"); 
  //EXTI->SWIER1 |= BIT(7); // test 
  gpio_set_mode(led, GPIO_MODE_OUTPUT);  // Set blue LED to output mode
  gpio_set_mode(btn, GPIO_MODE_INPUT); 
  gpio_set_pullup(btn); 
 // printf("UART alive3\r\n"); 
  adc_gpio_init(adc, NO_PULL); 
  
  dma_ch1_setup(request_ID, &adc_sample); 
  adc_stable_calibration_state(); 
  adc_set_configs(resolution, align, single, adc);
  
 
  adc_set_sequence(0, 6, smp); 
  dma1_ch1_enable(); 
  start_adc(); 

  spi1_gpio_init(spi_sck, miso, mosi, ssel);
  spi1_master_config(); 
  //printf("&ADC1->SQR1=%p &ADC1->DR=%p\r\n", &ADC1->SQR1, &ADC1->DR);
  
  //uint32_t timer = 0, period = 500; // remove when removing timer_expired cond. 
  //uint32_t t = 0; 
    for (;;) {
    
     // remove so button toggles LED only
     /*
      if (timer_expired(&timer, period, s_ticks)) {
        static bool on;       // This block is executed
        gpio_write(led, on);  // Every `period` milliseconds
        on = !on;             // Toggle LED state
        printf("LED: %d, tick: %lu\r\n", on, s_ticks);
      } */
      /*
      static bool led_on = false;
      if (g_btn_event) {
      g_btn_event = 0;
      led_on = !led_on;
      gpio_write(led, led_on);
      printf("Button! LED=%d tick=%lu\r\n", led_on, s_ticks);
    } */ 
      //printf("condition not met \n");
      /*
      if (timer_expired(&t, 1000, s_ticks)) {          // every 100 ms
        //printf("Step A: \r\n");
        dma1_ch1_disable();
        DMA1->IFCR = 0xFU << 0;                        // clear CH1 flags
        DMA1_CH1->CNDTR = 1U;                          // reload count
        dma1_ch1_enable();
        //printf("CR=%08lX ISR=%08lX SQR1=%08lX SMPR1=%08lX CFGR=%08lX DR=%lu\r\n",
        //ADC1->CR, ADC1->ISR, ADC1->SQR1, ADC1->SMPR1, ADC1->CFGR, (unsigned long)ADC1->DR); 
        // Start ADC conversion (ADC must already be enabled + ready)
        ADC1->CR |= BIT(2);      // ADSTART
        //printf("ADSTART \r\n"); 
        // Wait for DMA transfer complete
        while (!(DMA1->ISR & BIT(1))) (void)0;
        DMA1->IFCR = BIT(1);                    // clear TCIF1
       // printf("ADC1 CR:  0x%08lX\r\n", ADC1->CR);
       // printf("ADC1 ISR:  0x%08lX\r\n", ADC1->ISR);
        printf("ADC Sample: %u\r\n", (unsigned)adc_sample);
    } */
    shiftreg_write(0x00, ssel);    // 0000
    delay_ms(750); 
    shiftreg_write(0xFF, ssel);    // 1111
    delay_ms(750); 
    shiftreg_write(0x05, ssel);    // 0101
    delay_ms(750); 
    shiftreg_write(0x0A, ssel);   // 1010
    delay_ms(750); 
  }
  return 0;
}