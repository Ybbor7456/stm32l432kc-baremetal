#include <stdint.h>
#include <stdbool.h>

void uart_led(uint32_t *timer, uint32_t period, uint32_t ticks, uint16_t led);
void led_on_off(volatile uint8_t *btn_event, uint16_t led, bool *led_on, uint32_t ticks);
void adc_read(uint32_t *t, uint32_t ticks, volatile uint16_t *sample);
void reg_lights(uint16_t ssel);