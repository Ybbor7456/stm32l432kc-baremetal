#pragma once
#include <stdint.h>

struct systick { volatile uint32_t CTRL, LOAD, VAL, CALIB; };
#define SYSTICK ((struct systick *) 0xE000E010u)

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

struct gpio {
  volatile uint32_t MODER, OTYPER, OSPEEDR, PUPDR, IDR, ODR, BSRR, LCKR, AFR[2];
    // MODER - GPIO functioninality, GPIO -> MDOER
    // OTYPER - output type register, determines if a pin functions as push-pull or open drain output
    // OSPEEDR - controls rise and fall times/speed of output or alternate function pins. 
    // PUPDR - Pulllup and pulldown register, used to contorl if there are itnernal resistors active in pins
    // IDR - input datat register, used to determine logic high or logic low on pins
    // ODR - output data register, sets logical output (HIGH/LOW)
    // BSRR - bit set/reset register, allows to reset or set pins without corruption 
    // LCKR - GPIO port configuration LoCK Register, used to reset configs until next reset
    // AFR[] - GPIO Alternate Function Registers, used to map peripheral such as UART/SPI pin to GPIO 
};
#define GPIO(bank) ((struct gpio *)(0x48000000u + 0x400u * (bank)))// 0x48e6 is stm32L4 addy 

struct usart { volatile uint32_t CR1, CR2, CR3, BRR, GTPR, RTOR, RQR, ISR, ICR, RDR, TDR; };
#define USART1  ((struct usart *) 0x40013800u)
#define USART2  ((struct usart *) 0x40004400u)
#define USART3  ((struct usart *) 0x40004800u)
#define LPUART1 ((struct usart *) 0x40008000u)

struct i2c { volatile uint32_t CR1, CR2, OAR1, OAR2, TIMINGR, TIMEOUTR, ISR, ICR, PECR, RXDR, TXDR; };
/* control reg 1-3, Own address reg 1&2, timing register, timeout register,interrupt and status
  interrupt clear, packet error checking register, receive data, transmit data. 
*/
#define I2C1 ((struct i2c *) 0x40005400u)
#define I2C3 ((struct i2c *) 0x40005C00u)

struct exti{ volatile uint32_t IMR1, EMR1, RTSR1, FTSR1, SWIER1, PR1, IMR2, EMR2, RTSR2, FTSR2, SWIER2, PR2;}; 
#define EXTI ((struct exti *)0x40010400u)

/*
IMR1 - Interupt Mask 1
EMR1 - Event Mask 1
RTSR1 - Rising Trigger Selection Register 1
FTSR1 - Falling Trigger Selection Register 1
SWIER1 - Software Interrupt Event Register
PR1 - Pending Registers
*/

struct syscfg{volatile uint32_t MEMRMP,CFGR1,EXTICR1, EXTICR2, EXTICR3, EXTICR4, SCSR,CFGR2, SWPR, SKR;}; 
#define SYSCFG ((struct syscfg *)0x40010000u)

/*
memory remap
config register
external interrupt config register
control and status reg
write portection reg
sram2 key register
*/


//These are raw memory-mapped register addresses for the ARM Cortex-M4 NVIC
//ISER = Interrupt Set-Enable Register
// ICER = interrupt clear enable register 
//ISER0/ICER0 covers IRQ 0..31
//ISER1/ICER1 covers IRQ 32..63

#define NVIC_ISER0 (*(volatile uint32_t *)0xE000E100u)
#define NVIC_ISER1 (*(volatile uint32_t *)0xE000E104u)
#define NVIC_ICER0 (*(volatile uint32_t *)0xE000E180u)
#define NVIC_ICER1 (*(volatile uint32_t *)0xE000E184u)
#define NVIC_IPR   ((volatile uint8_t  *)0xE000E400u)   // priority bytes

//This is not a register. It’s just a number telling you which IRQ line in the NVIC corresponds to the “EXTI lines 5..9”
#define IRQ_EXTI9_5 23u