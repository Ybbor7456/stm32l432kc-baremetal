#include <stdint.h>
#include <stdbool.h>

void adxl345_read_reg(uint16_t cs, uint8_t registers); 
void adxl345_write_reg(uint16_t cs, uint8_t registers, uint8_t val); 
void adxl345_init(uint16_t cs); 