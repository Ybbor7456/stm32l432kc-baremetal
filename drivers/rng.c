#include "drivers/rng.h"

uint32_t rng_read(){
    while(!(RNG->SR & BIT(0)));
    uint32_t data = RNG->DR;
    return data; 
}