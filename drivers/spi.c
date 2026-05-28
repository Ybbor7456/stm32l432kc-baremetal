#include "drivers/spi.h"

void spi1_gpio_init(uint16_t sck, uint16_t miso, uint16_t mosi, uint16_t cs){
    /* use these if not rewiring/have a fixed spi pinout. 
    sck = PIN('A', 5);
    miso = PIN('A',6 ); can be unused for shift register
    mosi = PIN('A',7 ); 
    ssel = PIN('B',0 );
    */
    // gpio set mode also configures the gpio PIN/BANK clocks
    uint8_t af = 5; 
    RCC->APB2ENR |= BIT(12); //clock enable
    gpio_set_mode(sck, GPIO_MODE_AF); 
    gpio_set_af(sck, af); 
    gpio_set_mode(miso, GPIO_MODE_AF); 
    gpio_set_af(miso, af); 
    gpio_set_mode(mosi, GPIO_MODE_AF); 
    gpio_set_af(mosi, af); 
    gpio_set_mode(cs, GPIO_MODE_OUTPUT); 
    gpio_write(cs, true); 
    //gpio_set_af(ssel, af); 
    // needs pull-up/pulldown 
}

void spi1_master_config(void){
    //printf("A \r\n");
    SPI1->CR1 |= BIT(2); 
    //printf("B \r\n");
    // set 8-bit
    SPI1->CR2 &= ~(0xFU << 8); // clear CR2 bits 8-11
    //printf("C \r\n");
    SPI1->CR2 |= (7U << 8); // set 8bit (0111)
    
    //Keep RXONLY bit clear when bidirectional mode is active.
    SPI1->CR1 &= ~BIT(10); // set to full-duplex
    SPI1->CR1 |= BIT(8);   // SSI
    SPI1->CR1 |= BIT(9);   // SSM
    SPI1->CR1 &= ~BIT(7); // MSB
    
    // SPI1->CR1 |= BIT(7); lsb first 
    SPI1->CR1 &= ~(7U << 3); // clear baud 
    SPI1->CR1 |= (4U << 3); // set baud to Fpclk/32
    //SPI1->CR1 &= ~(BIT(1) | BIT(0)); // CPOL and CPHA = 0
    SPI1->CR1 |= (BIT(1) | BIT(0)); // cpol cpha = 1
    // SPI1->CR1 |= BIT(0); // cpha = 1
    // SPI1->CR1 |= BIT(1); // cpol = 1
   
    SPI1->CR2 |= BIT(12); // RXNE 8-bit
    SPI1->CR1 |= BIT(6); // spi enable
}

uint8_t spi1_byte_txrx(uint8_t byte){
    if(SPI1->CR1 & BIT(6)){ // checks if SPI enabled
        // Wait until transmit buffer is empty (ready for new data)
        while ((SPI1->SR & BIT(1)) == 0) {
        }
        *(volatile uint8_t *)&SPI1->DR = byte;
        // Wait until a byte has been received
        while ((SPI1->SR & BIT(0)) == 0) {
        } 
    }
    // Read and return the received byte
    return *(volatile uint8_t *)&SPI1->DR;
}

