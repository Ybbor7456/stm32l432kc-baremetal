#include "drivers/hal.h"
#include "bsp/board.h"
#include "drivers/stm32l4_regs.h"
#include "drivers/util.h"
#include "drivers/logger.h"


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
//  GPIOB port 3 has onboard LED 

int main(void) {
  uint16_t led = PIN('B', 3);            // Green LED, PIN('B' - A  << 8) | Num.... 0x100 | 3 = 0x103 = led
  uint16_t btn = PIN('B', 7);
  uint16_t adc = PIN('A', 1);     // adc PA1 ADC1_6
  // upper byte stores 01 = B, and lower stores 03 for LED
  //RCC->AHB2ENR |= BIT(PINBANK(led));     // Enable GPIO clock for LED, PINBANK(0x103) >> 8 = 0
  uart_init(USART2, 115200);
  systick_init(USART2_FCK / 1000);         // Tick every 1 ms
  exti_init(btn);
  nvic_set_priority(IRQ_EXTI9_5, 0x80); 
  nvic_enable_irq(IRQ_EXTI9_5);
  irq_global_enable(); 
  //EXTI->SWIER1 |= BIT(7); // test 
  gpio_set_mode(led, GPIO_MODE_OUTPUT);  // Set blue LED to output mode
  gpio_set_mode(btn, GPIO_MODE_INPUT); 
  gpio_set_pullup(btn); 
  //uint32_t timer = 0, period = 500; // remove when removing timer_expired cond. 
    for (;;) {
     // remove so button toggles LED only
     /*
      if (timer_expired(&timer, period, s_ticks)) {
        static bool on;       // This block is executed
        gpio_write(led, on);  // Every `period` milliseconds
        on = !on;             // Toggle LED state
        printf("LED: %d, tick: %lu\r\n", on, s_ticks);
      } */
     
      static bool led_on = false;
      if (g_btn_event) {
      g_btn_event = 0;
      led_on = !led_on;
      gpio_write(led, led_on);
      printf("Button! LED=%d tick=%lu\r\n", led_on, s_ticks);
    } 
    // Here we could perform other activities!
  }
  return 0; 
}