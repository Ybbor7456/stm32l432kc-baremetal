#include "drivers/hal.h"
#include "bsp/board.h"
#include "drivers/stm32l4_regs.h"
#include "drivers/util.h"
#include "drivers/logger.h"


static volatile uint32_t s_ticks; // volatile is important!!
void SysTick_Handler(void) {
  s_ticks++;
}
static inline void irq_global_enable(void){ 
  __asm volatile ("cpsie i");
}


//  GPIOB port 3 has onboard LED 

int main(void) {
  uint16_t led = PIN('B', 3);            // Green LED, PIN('B' - A  << 8) | Num.... 0x100 | 3 = 0x103 = led
  // upper byte stores 01 = B, and lower stores 03 for LED
  //RCC->AHB2ENR |= BIT(PINBANK(led));     // Enable GPIO clock for LED, PINBANK(0x103) >> 8 = 0
  uart_init(USART2, 115200);
  systick_init(USART2_FCK / 1000);         // Tick every 1 ms
  //exti_init(...);
  //nvic_enable_irq(...);
  irq_global_enable(); 
  gpio_set_mode(led, GPIO_MODE_OUTPUT);  // Set blue LED to output mode
  uint32_t timer = 0, period = 500; 
    for (;;) {
      if (timer_expired(&timer, period, s_ticks)) {
        static bool on;       // This block is executed
        gpio_write(led, on);  // Every `period` milliseconds
        on = !on;             // Toggle LED state
        printf("LED: %d, tick: %lu\r\n", on, s_ticks);
      }
    // Here we could perform other activities!
  }
  return 0; 
}
