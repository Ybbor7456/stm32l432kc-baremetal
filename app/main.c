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
#include "fnxs.h"
#include "devices/adxl345.h"
#include "drivers/tim.h"
#include "drivers/iwdg.h"
#include "drivers/can.h"

static volatile uint32_t s_ticks; // volatile is important!!
/* void SysTick_Handler(void) {
  s_ticks++;
} */

volatile uint8_t g_btn_event = 0; // for interrupts S / baud; 
/**/
void EXTI9_5_IRQHandler(void) // use this for button hardware, don't use static inlie
{
  // check which line (5..9) fired, e.g. line 7:
  if (EXTI->PR1 & BIT(7)) {
    EXTI->PR1 = BIT(7);     // clear pending by writing 1
    // button logic to add
    g_btn_event = 1; 
  }
}

int main(void) {
    uint16_t test_pin = PIN('A', 12);
    //uint16_t test_pin = PIN ('A', 11); 
    uart_init(USART2, 115200);
    tim2_init();
    gpio_set_mode(test_pin, GPIO_MODE_OUTPUT);
    for (;;) {
        gpio_write(test_pin, 1);
        //printf("PA11 high \r\n");
        printf("PA12 high\r\n");
        tim2_delay_ms(1000);

        gpio_write(test_pin, 0);
        //printf("PA11 low \r\n");
        printf("PA12 low\r\n");
        tim2_delay_ms(1000);
    }
    return 0;
}


/*
int main(void) {
  
  uint16_t led = PIN('B', 3);            // Green LED, PIN('B' - A  << 8) | Num.... 0x100 | 3 = 0x103 = led
  uint16_t btn = PIN('B', 7);
  uint16_t adc = PIN('A', 1);     // adc PA1 ADC1_6

  uint16_t spi_sck = PIN('A', 5);
  uint16_t miso = PIN('A',6 );
  uint16_t mosi = PIN('A',7 ); 
  uint16_t cs = PIN('B',0 ); 
  uint16_t can_rx = PIN('A', 11);
  uint16_t can_tx = PIN('A', 12); 
  //int16_t x,y,z;
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

 
  systick_init(SYSCLK_HZ  / 1000);         // Tick every 1 ms
 
  exti_init(btn);
  
  nvic_set_priority(IRQ_EXTI9_5, 0x80); 
  nvic_enable_irq(IRQ_EXTI9_5);
  irq_global_enable(); 
  
  
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
  //can_init();

  
  //printf("spi1 before init \r\n");
  spi1_gpio_init(spi_sck, miso, mosi, cs);
  //printf("spi1 init finished \r\n");
  spi1_master_config(); 
  
  //printf("&ADC1->SQR1=%p &ADC1->DR=%p\r\n", &ADC1->SQR1, &ADC1->DR);
  
  //uint32_t timer = 0, period = 500; // remove when removing timer_expired cond. 
  //uint32_t t = 0; static bool led_on = false;
  adxl345_init(cs); 
  tim2_init();
  
  // timeout = ((rlr +1)*prescaler)/LSI, LSI = 32kHz, prescaler = 64, timeout = 2 (roughly)
  //uint32_t pr = 0x4u; // 
  //uint32_t rlr = 0x3E7u;// 999
  
  //iwdg_init(pr, rlr);
  //iwdg_refresh();

 
 can_msg_t msg = { // sender side
    .id = 0x123,
    .dlc = 4,
    .data = {0x11, 0x22, 0x33, 0x44}
  };  

  can_msg_t rx = {0}; 
  can_init(can_rx, can_tx); 
    for (;;) {
    // receive 
      
      tim2_delay_ms(2000); 
      printf("Hi\r\n");
      printf("RF0R=0x%08lx  MSR=0x%08lx\r\n ", CAN_CORE->RF0R, CAN_CORE->MSR);
      printf("RF1R=0x%08lx  ESR=0x%08lx\r\n ", CAN_CORE->RF1R, CAN_CORE->ESR);
      
      if (can_fifo0_pending()) {
        printf("hey \r\n"); 
        can_read_fifo0(&rx);
        
        printf("RX: id=0x%03lx dlc=%u data=%02X %02X %02X %02X\r\n",
        rx.id, rx.dlc, rx.data[0], rx.data[1], rx.data[2], rx.data[3]);
      } 
      
      //transmit 
      
      tim2_delay_ms(2000); 
      printf("Before: TSR=0x%08lx ESR=0x%08lx\r\n", CAN_CORE->TSR, CAN_CORE->ESR);
      can_send_std(&msg); // wait for anothe stm32l4 to come in mail to test receiver side
      printf("AFTER TXREQ: TSR=0x%08lx ESR=0x%08lx\r\n", CAN_CORE->TSR, CAN_CORE->ESR);
      printf("TX: id=0x%03lx dlc=%u data=%02X %02X %02X %02X\r\n",
        msg.id, msg.dlc, msg.data[0], msg.data[1], msg.data[2], msg.data[3]);
      tim2_delay_ms(2000);
      printf("testing loop. \r\n"); 
    } 
  return 0;
}
*/

//loopback
/*
can_send_std(&msg); 
tim2_delay_ms(100);   
  if (can_fifo0_pending()) {
    can_read_fifo0(&rx);
    printf("RX: id=0x%03lx dlc=%u data=%02X %02X %02X %02X\r\n",
    rx.id, rx.dlc, rx.data[0], rx.data[1], rx.data[2], rx.data[3]);
    tim2_delay_ms(1500);
  } */ 