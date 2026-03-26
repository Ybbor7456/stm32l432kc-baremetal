#include <stdint.h>
#include <stdbool.h>

uint8_t adxl345_read_reg(uint16_t cspin, uint8_t reg); 
void adxl345_write_reg(uint16_t cspin, uint8_t reg, uint8_t val); 
void adxl345_init(uint16_t cspin); 
void adxl345_write_multibyte_reg(uint16_t cspin, uint8_t sr,const uint8_t *b, uint8_t length); 
void adxl345_read_multibyte_reg(uint16_t cspin, uint8_t sr, uint8_t *b, uint8_t length); 
void adxl345_read_xyz(uint16_t cspin, int16_t *xaxis, int16_t *yaxis, int16_t *zaxis); 