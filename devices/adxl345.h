// adxl functions


#include <stdio.h>
#include "drivers/hal.h"
#include "drivers/gpio.h"
#include "drivers/adc_dma.h"
#include "drivers/stm32l4_regs.h"
#include "drivers/spi.h"

uint8_t adxl345_read_reg(uint16_t cs, uint8_t registers){
    //ADXL345 -> 10111001, 0x9A4819, needs 1 byte to read, bit 7: read bit 6: multi byte bit 0-5: reg address
    // full duplex: send a receive bytes at the same time
    uint8_t cmd = ADXL345_SPI_READ | registers; 
    uint8_t dummy_byte = 0x00; // dummy byte to push clock pulses
    uint8_t trash, data; 
    gpio_write(cs, false); // drive low

    while ((SPI1->SR & BIT(1)) == 0) {} // wait for TX in 
    *(volatile uint8_t *)&SPI1->DR = cmd; // send cmd to DR *8-bit
    while(((SPI1->SR & BIT(0)) == 0)){} // wait for RX = 1
    trash = *(volatile uint8_t *)&SPI1->DR; // clears RX flag
    (void) trash; 
    while((SPI1->SR & BIT(1)) == 0){}
    *(volatile uint8_t *)&SPI1->DR = dummy_byte; 
    while((SPI1->SR & BIT(0)) == 0){}
    data = *(volatile uint8_t *)&SPI1->DR; // fill register value

    spi1_wait_idle(); // bsy flag
    gpio_write(cs, true); // pull CS high
    return data;// retrun 
}


void adxl345_write_reg(uint16_t cs, uint8_t registers, uint8_t val){
    uint8_t cmd = ADXL345_SPI_WRITE | registers; 
    uint8_t trash; 
    gpio_write(cs, false); 
    while((SPI1->SR & BIT(1))== 0){}
    *(volatile uint8_t *)&SPI1->DR = cmd; 
    while((SPI1->SR & BIT(0))== 0){}
    trash = *(volatile uint8_t *)&SPI1->DR; 
    (void) trash; 
    while((SPI1->SR & BIT(1))== 0){}
    *(volatile uint8_t *)&SPI1->DR = val; 
    while((SPI1->SR & BIT(0))== 0){}
    trash = *(volatile uint8_t *)&SPI1->DR;

    spi1_wait_idle();  
    gpio_write(cs, true); 
}

void adxl345_init(uint16_t cs){
    adxl345_write_reg(cs, POWER_CTL ,ADXL345_MEASURE); 
    adxl345_write_reg(cs, DATA_FORMAT, 0x08 | 0x00); // +/- 2g full res
    adxl345_write_reg(cs, BW_RATE, 0x0A); // 100mz output
}
