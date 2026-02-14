// startup.c (bare-metal)

#include <stdint.h>

// --------- Reset handler ----------
__attribute__((naked, noreturn)) void _reset(void) {
  // Initialise memory
  extern long _sbss, _ebss, _sdata, _edata, _sidata;
  for (long *dst = &_sbss; dst < &_ebss; dst++) *dst = 0;
  for (long *dst = &_sdata, *src = &_sidata; dst < &_edata;) *dst++ = *src++;

  extern void main(void);
  main();

  for (;;) (void)0;
}

// --------- Default handler ----------
__attribute__((noreturn)) void Default_Handler(void) {
  for (;;) (void)0;
}

// --------- Symbols / handlers ----------
extern void _estack(void);          // from link.ld

// Provide weak defaults so unimplemented IRQs don't crash into address 0
void SysTick_Handler(void)   __attribute__((weak, alias("Default_Handler")));
void EXTI9_5_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));

// If you later add USART2, etc, you can add more weak aliases here.

// --------- Vector table ----------
__attribute__((section(".vectors")))
void (*const tab[16 + 67])(void) = {
  _estack,        // 0: initial stack pointer
  _reset,         // 1: reset
  Default_Handler,// 2: NMI
  Default_Handler,// 3: HardFault
  Default_Handler,// 4: MemManage
  Default_Handler,// 5: BusFault
  Default_Handler,// 6: UsageFault
  0, 0, 0, 0,     // 7-10: reserved
  Default_Handler,// 11: SVCall
  0, 0,           // 12-13: reserved
  Default_Handler,// 14: PendSV
  SysTick_Handler,// 15: SysTick

  // External IRQs start at index 16 (IRQ0)
  [16 + 23] = EXTI9_5_IRQHandler,   // IRQ23 = EXTI9_5 (lines 5..9)
};
