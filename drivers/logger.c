// logger implementation 
#include "drivers/logger.h"
#include <stdarg.h>
#include <stdio.h>    // if using vsnprintf

uint32_t millis(void) {
  return ticks;
}

void log_printf(const char *level, const char *fmt, ...) {
    va_list args;
    va_start(args, fmt);

    printf("[%8u] [%s] ", millis(), level);

    // vprintf is the variadic version of printf
    vprintf(fmt, args);
    printf("\n");

    va_end(args);
}

