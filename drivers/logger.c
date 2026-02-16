// logger implementation 
#include "drivers/logger.h"
#include "drivers/hal.h"
#include <stdarg.h>
#include <stdio.h>

uint32_t millis(void) {
  return hal_millis();
}
