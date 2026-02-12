# Baremetal Programming STM32l432KC

Baremetal programming is writing software that runs directly on the hardware. For example, writing lines of code that directly trigger the individual registers, clocks, and peripherals 
within an MCU. This type of programming peels away the layers of abstraction and bypasses the operating system. Aside from learning the fundamentals of software-hardware interactions, bare-metal programming 
comes with advantages to in shipable products. 
## Advantages ##
1. Absolute hardware control - directly manage every hardware register, clock source, and sleep state without an operating system layer interfering
2. Lack of overhead - Removing an OS/RTOS allows for a smaller memory footprint. If there is limited memory remaining in a product, it might be best to take a bare-metal approach. 
3. Precise timing - In devices where precision timing matters and milliseconds (even microseconds) can be saved, bare-metal programming is an excellent approach. 
## Disadvantages ##
1. Complexity - As projects grow the complexity of bare-metal becomes increasingly difficult. 
2. Dev Time - Though bare-metal may save time in runtime and startup, the time it takes to program is a longer process. 
3. Portability - Periperals and clocks change from MCU to MCU, making writen bare-metal code impossible to transfer to various projects. The Arm-Cortex does have standardized systicks and 
the first few entries of the NVIC (nested vector interrupt table), but overall each MCU is programmed differently. 
## Features ##
- GPIO
- RCC
- UART
- I^2C
- SysTick ms timebase
- EXTI
## Quickstart ##
### Toolchain Prerequisites ###
- ARM GCC Toolchain
- ST-Link flashing tool
### Build ###
- Make Clean
- Make Build
#### Expected Output #####
- firmware.elf
- firmware.bin
- firmware.elf.map
### Flash ###
-Make Flash
### Serial Monitor ###
Finding the serial device/port
- ls /dev/ttyACM* /dev/ttyUSB* 2>/dev/null
Opening with baud
- screen /dev/ttyACM0 115200
## Project Layout ## 
#### app
main.c
#### bsp
board.c
board.h
#### build
firmware.elf
firmware.bin
firmware.elf.map
#### docs 
pinmap.md
#### drivers
hal.h
stm32l4_regs.h
util.h
logger.h
logger.c
#### platform
link.ld
syscalls.c
startup.c
#### references 
datasheets, reference manuals, user manuals, general notes
#### Makefile
## Pinmap ##
[Pin Map](docs/pinmap.md#uart-pins)

## Clocks and buses note
The BUS connects the CPU to the memory and peripherals. Peripherals are mapped to different buses for speed/power/clocking reasons. Each bus has its own clock domain, and RCC can turn on/off clocks to peripherals on that bus.
GPIO is on AHB2 → enable via RCC->AHB2ENR (Advanced High-performance Bus, domain 2) 
USART1 is on APB2 -> enable via RCC->APB2ENR  (Advanced Peripheral Bus 2)
USART2 is on APB1 -> enable via RCC->APB1ENR1 (Advanced Peripheral Bus 1)
LPUART (Low-Power UART on STML4 model) is on APB1, enabled via RCC -> APB1ENR2. 
APB1ENR1 vs. ABP1ENR2. STM32L4 splits APB1 into two enable-register banks to maximize performance. 
Other buses - AHB3, DMA1, DMA2, CCM. 
Clock gating is when the MCU turns off the clock signal to a peripheral block (GPIO, USART, I2C, timers, etc.) until explicitly enabled.
### example
#### Enabling clock to GPIO ports. 
#define BIT(x) (1UL << (x))
#define PINBANK(pin) ((pin) >> 8)
RCC->AHB2ENR |= BIT(PINBANK(pin));
#### Enabling clock to peripherals (USART2) 
RCC->APB1ENR1 |= BIT(17);
## GPIO Modes
GPIO pin modes are configured via the GPIO port mode register GPIOx_MODER (one per port: GPIOA, GPIOB, GPIOC, …).
Some STM32L4 devices provide ports beyond A–C, but the STM32L432KC exposes only a subset (commonly A–C depending on package/board).
GPIOx_MODER is 32 bits: 16 pins per port * 2 bits per pin.
Mode encoding:
- 00 Input
- 01 General-purpose output
- 10 Alternate function (peripheral-controlled; AF selection is in `GPIOx_AFR[0/1]`)
- 11 Analog





