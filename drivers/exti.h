static inline void exti_route(uint16_t pin) { // handles the SYSCFG mapping
  uint32_t line  = PINNO(pin);       // 0..15
  uint32_t port  = PINBANK(pin);     // A=0,B=1,C=2...
  uint32_t idx   = line >> 2;        // 0..3 (EXTICR1..4)
  uint32_t shift = (line & 3u) * 4u; // 0,4,8,12

  RCC->APB2ENR |= BIT(0);            // SYSCFG clock enable
  volatile uint32_t *exticr = &SYSCFG->EXTICR1 + idx;
  *exticr = (*exticr & ~(0xFu << shift)) | ((port & 0xFu) << shift); // clear then set
}

static inline void exti_enable_falling(uint32_t line){
  EXTI->IMR1 |= BIT(line);     // interrupt line 0 = masked, 1 = not masked/enable interrupts
  EXTI->RTSR1 &= ~BIT(line);  // disable rising
  EXTI->FTSR1 |= BIT(line);   // line 1 = FT enabled, Trigger an intterupt when going high to low
  EXTI->PR1 = BIT(line);   // 1: Selected trigger request occurred
}

static inline void exti_enable_rising(uint32_t line){
  EXTI->IMR1 |= BIT(line); 
  EXTI->RTSR1 |= BIT(line); 
  EXTI->FTSR1 &= ~BIT(line); 
  EXTI->PR1 = BIT(line); 
}

static inline void exti_enable_both(uint32_t line){
  EXTI->IMR1 |= BIT(line); 
  EXTI->RTSR1 |= BIT(line); 
  EXTI->FTSR1 |= BIT(line); 
  EXTI->PR1 = BIT(line); 
}

static inline void exti_init(uint16_t pin){
  uint32_t line = PINNO(pin);
  exti_route(pin); 
  exti_enable_falling(line); 
}