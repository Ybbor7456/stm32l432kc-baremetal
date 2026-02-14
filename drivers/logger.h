// logger.h
#pragma once
#include <stdint.h>

uint32_t millis(void);
void log_printf(const char *level, const char *fmt,...);

#define LOGI(...) log_printf("INFO",  __VA_ARGS__)
#define LOGW(...) log_printf("WARN",  __VA_ARGS__)
#define LOGE(...) log_printf("ERROR", __VA_ARGS__)