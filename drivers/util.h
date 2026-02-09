#pragma once
#include <stdint.h>
#include <stdbool.h>


#define BIT(x) (1UL << (x))
#define PIN(bank, num) ((((bank) - 'A') << 8) | (num))
#define PINNO(pin)   ((pin) & 0xFFu)
#define PINBANK(pin) ((pin) >> 8)
#define MIN(a,b) (((a) < (b)) ? (a) : (b))

