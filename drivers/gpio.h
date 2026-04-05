#pragma once
#include <inttypes.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <math.h> 
#include <stdint.h>

#include "drivers/util.h"
#include "drivers/stm32l4_regs.h"
#include "bsp/board.h"

enum {GPIO_MODE_INPUT, GPIO_MODE_OUTPUT, GPIO_MODE_AF, GPIO_MODE_ANALOG };
enum{ // 00, 01, 10, 11 for pullup/pull-down
  NO_PULL, UP, DOWN, RESERVED
}; 
enum {GPIO_LOW, GPIO_MEDIUM, GPIO_HIGH, GPIO_VHIGH}; 

static inline void gpio_enable(uint16_t pin) { // enable GPIO clock port
  RCC->AHB2ENR |= BIT(PINBANK(pin));
  (void)RCC->AHB2ENR; // optional readback to ensure the write completes
}

static inline void gpio_set_mode(uint16_t pin, uint8_t gpio_mode) {
  struct gpio *gpio = GPIO(PINBANK(pin));  // GPIO bank
  gpio_enable(pin);
  int n = PINNO(pin);                      // Pin number
  gpio->MODER &= ~(3U << (n * 2));         // Clear existing setting
  gpio->MODER |= (gpio_mode & 3) << (n * 2);    // Set new mode, uses (gpio_mode & 3) incase bad value passed, preserve bottom 2 bits
}

static inline void gpio_set_af(uint16_t pin, uint8_t af_num) { // check datasheet for AF mapping 
  struct gpio *gpio = GPIO(PINBANK(pin));  // GPIO bank
  gpio_enable(pin);
  int n = PINNO(pin);                      // Pin number
  gpio->AFR[n >> 3] &= ~(15UL << ((n & 7) * 4)); // clears, shifting by 3 is the same as dividing by 8, selecting high or low
  gpio->AFR[n >> 3] |= ((uint32_t) af_num) << ((n & 7) * 4); // sets 
}

static inline void gpio_write(uint16_t pin, bool val) {
  struct gpio *gpio = GPIO(PINBANK(pin));
  gpio->BSRR = (1U << PINNO(pin)) << (val ? 0 : 16); // writing to the BSSR is atomic unlike the ODR
}

static inline bool gpio_read_pin(uint16_t pin) {
  struct gpio *gpio = GPIO(PINBANK(pin));
  return (gpio->IDR & BIT(PINNO(pin))) != 0;
}

static inline void gpio_set_pullup(uint16_t pin) {
  struct gpio *gpio = GPIO(PINBANK(pin));
  int n = PINNO(pin);
  gpio_enable(pin);
  gpio->PUPDR &= ~(3U << (n * 2));      // clear
  gpio->PUPDR |=  (1U << (n * 2));      // 01 pullup, 00 no PU or PD, 10 ulldown, 11 resserved
}

static inline void gpio_set_pupd(uint16_t pin, uint8_t conf) {
  struct gpio *gpio = GPIO(PINBANK(pin));
  int n = PINNO(pin);
  gpio_enable(pin);
  gpio->PUPDR &= ~(3U << (n * 2));      // clear
  gpio->PUPDR |=  ((conf & 3U) << (n * 2));      // 01 pullup, 00 no PU or PD, 10 ulldown, 11 resserved
}

static inline void gpio_set_speed(uint16_t pin) {
  struct gpio *gpio = GPIO(PINBANK(pin));
  gpio_enable(pin);
  int n = PINNO(pin);
  gpio->OSPEEDR &= ~(3U << (n * 2));    // clear
  gpio->OSPEEDR |=  (3U << (n * 2));    // 11 = (very) high, 10 is high, 01 medium, 00 slow
}

static inline void gpio_set_select_speed(uint16_t pin, uint8_t speed) {
  struct gpio *gpio = GPIO(PINBANK(pin));
  gpio_enable(pin);
  int n = PINNO(pin);
  gpio->OSPEEDR &= ~(3U << (n * 2));    // clear 2 bit field
  gpio->OSPEEDR |=  ((speed & 0x03U) << (n * 2)); 
}

static inline void gpio_set_open_drain(uint16_t pin) {
  struct gpio *g = GPIO(PINBANK(pin));
  gpio_enable(pin);
  int n = PINNO(pin);
  g->OTYPER |= BIT(n);              // 1 = open-drain
}

static inline void gpio_set_otype(uint16_t pin, uint8_t otype){
  struct gpio *g = GPIO(PINBANK(pin));
  gpio_enable(pin);
  int n = PINNO(pin);
  g->OTYPER &= ~(1U << n);
  g->OTYPER |= (otype & 1U) << n;
}

// move to hardware.h
static inline void button_gpio_init(uint16_t pin){
  gpio_set_mode(pin, GPIO_MODE_INPUT);
  gpio_set_pupd(pin, NO_PULL);            
}

/*
digitalWrite()-> BSRR / ODR
digitalRead() -> IDR
analogWrite() -> timer PWM peripheral

*/