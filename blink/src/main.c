#include "stm32f4xx.h"
void ms_delay(int ms) {
  const int inner_iters_per_ms = 2000; 
  while (ms-- > 0) {
    volatile int x = inner_iters_per_ms;
    while (x-- > 0) {
      __asm volatile ("nop");
    }
  }
}
int main(void) {
  // liga clock do GPIOC
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
  // PC13 como saída (01)
  GPIOC->MODER &= ~(3u << (13u * 2u));
  GPIOC->MODER |= (1u << (13u * 2u));
  for (;;) {
    ms_delay(500);           // ~500 ms
    GPIOC->ODR ^= (1u << 13u);     // toggle LED
  }
}
