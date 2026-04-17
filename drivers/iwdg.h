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



static inline void iwdg_unlock(void)  { IWDG->KR = 0x5555;}

static inline void iwdg_start(void)   { IWDG->KR = 0xCCCC;}

static inline void iwdg_refresh(void) { IWDG->KR = 0xAAAA;}

static inline void iwdg_wait_ready(void) {
    while (IWDG->SR & (BIT(0) | BIT(1))) {
    }
}

static inline void iwdg_set_prescaler(uint32_t pr) {
    IWDG->PR = pr;
}

static inline void iwdg_set_reload(uint32_t rlr) {
    IWDG->RLR = rlr & 0x0FFFu;
}

static inline void iwdg_init(uint32_t pr, uint32_t rlr) {
    iwdg_unlock();
    iwdg_set_prescaler(pr);
    iwdg_set_reload(rlr);
    //iwdg_wait_ready(); // hanging
    iwdg_refresh(); 
    iwdg_start();
}