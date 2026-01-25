#include <stdint.h>
#include <stdbool.h>

#define BIT(x) (1UL << (x))
#define PIN(bank, num) ((((bank) - 'A') << 8) | (num))
#define PINNO(pin) (pin & 255)
#define PINBANK(pin) (pin >> 8)

struct gpio {
  volatile uint32_t MODER, OTYPER, OSPEEDR, PUPDR, IDR, ODR, BSRR, LCKR, AFR[2];
};
#define GPIO(bank) ((struct gpio *)(0x48000000u + 0x400u * (bank)))// 0x48e6 is stm32L4 addy 
// have also tried 0x4800_0000u for GPIOA

// Enum values are per datasheet: 0, 1, 2, 3
enum { GPIO_MODE_INPUT, GPIO_MODE_OUTPUT, GPIO_MODE_AF, GPIO_MODE_ANALOG };


struct rcc {
  volatile uint32_t CR;          // 0x00
  volatile uint32_t ICSCR;       // 0x04
  volatile uint32_t CFGR;        // 0x08
  volatile uint32_t PLLCFGR;     // 0x0C
  volatile uint32_t PLLSAI1CFGR; // 0x10
  uint32_t RESERVED0;            // 0x14

  volatile uint32_t CIER;        // 0x18
  volatile uint32_t CIFR;        // 0x1C
  volatile uint32_t CICR;        // 0x20
  uint32_t RESERVED1;            // 0x24

  volatile uint32_t AHB1RSTR;    // 0x28
  volatile uint32_t AHB2RSTR;    // 0x2C
  volatile uint32_t AHB3RSTR;    // 0x30
  uint32_t RESERVED2;            // 0x34

  volatile uint32_t APB1RSTR1;   // 0x38
  volatile uint32_t APB1RSTR2;   // 0x3C
  volatile uint32_t APB2RSTR;    // 0x40
  uint32_t RESERVED3;            // 0x44

  volatile uint32_t AHB1ENR;     // 0x48
  volatile uint32_t AHB2ENR;     // 0x4C
  volatile uint32_t AHB3ENR;     // 0x50
  uint32_t RESERVED4;            // 0x54

  volatile uint32_t APB1ENR1;    // 0x58
  volatile uint32_t APB1ENR2;    // 0x5C
  volatile uint32_t APB2ENR;     // 0x60
  uint32_t RESERVED5;            // 0x64

  volatile uint32_t AHB1SMENR;   // 0x68
  volatile uint32_t AHB2SMENR;   // 0x6C
  volatile uint32_t AHB3SMENR;   // 0x70
  uint32_t RESERVED6;            // 0x74

  volatile uint32_t APB1SMENR1;  // 0x78
  volatile uint32_t APB1SMENR2;  // 0x7C
  volatile uint32_t APB2SMENR;   // 0x80
  uint32_t RESERVED7;            // 0x84

  volatile uint32_t CCIPR;       // 0x88
  uint32_t RESERVED8;            // 0x8C
  volatile uint32_t BDCR;        // 0x90
  volatile uint32_t CSR;         // 0x94
  volatile uint32_t CRRCR;       // 0x98
volatile uint32_t CCIPR2;      // 0x9C
}; 



#define RCC ((struct rcc *)0x40021000)

static inline void gpio_set_mode(uint16_t pin, uint8_t mode) {
  struct gpio *gpio = GPIO(PINBANK(pin));  // GPIO bank
  int n = PINNO(pin);                      // Pin number
  gpio->MODER &= ~(3U << (n * 2));         // Clear existing setting
  gpio->MODER |= (mode & 3) << (n * 2);    // Set new mode
}

static inline void gpio_write(uint16_t pin, bool val) {
  struct gpio *gpio = GPIO(PINBANK(pin));
  gpio->BSRR = (1U << PINNO(pin)) << (val ? 0 : 16);
}

static inline void spin(volatile uint32_t count) {
  while (count--) {
    __asm volatile ("nop");
  }
}

//  GPIOB port 3 has onboard LED 

/*
int main(void) {
  uint16_t led = PIN('B', 3);

  RCC->AHB2ENR |= BIT(PINBANK(led));
  gpio_set_mode(led, GPIO_MODE_OUTPUT);

  gpio_write(led, true);   // should turn LD3 ON (PB3 high)
  for (;;) (void)0;
}
*/ 

int main(void) {
  uint16_t led = PIN('B', 3);            // Blue LED
  RCC->AHB2ENR |= BIT(PINBANK(led));     // Enable GPIO clock for LED
  gpio_set_mode(led, GPIO_MODE_OUTPUT);  // Set blue LED to output mode
    for (;;) {
    gpio_write(led, true);
    spin(200000);
    gpio_write(led, false);
    spin(200000);
  }
  return 0; 
}


// Startup code
__attribute__((naked, noreturn)) void _reset(void) {          //naked=don't generate prologue/epilogue  //noreturn=it will never exit
  // memset .bss to zero, and copy .data section to RAM region
  extern long _sbss, _ebss, _sdata, _edata, _sidata;          //linker symbols declared in the .ld file
  for (long *dst = &_sbss; dst < &_ebss; dst++) *dst = 0;
  for (long *dst = &_sdata, *src = &_sidata; dst < &_edata;) *dst++ = *src++;

  main();             // Call main()
  for (;;) (void) 0;  // Infinite loop in the case if main() returns
}

extern void _estack(void);  // Defined in link.ld

// 16 standard and 91 STM32-specific handlers
__attribute__((section(".vectors"))) void (*const tab[16 + 91])(void) = { // defines the interrupt vector table and dumps it into .vectors via the linker script
    _estack, _reset};