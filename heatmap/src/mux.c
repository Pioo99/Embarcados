#include "stm32f4xx.h"
#include "mux.h"

/* S0=PA1  S1=PA2  S2=PA3  S3=PA4 */

void mux_init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    /* PA1-PA4: output push-pull, low speed */
    GPIOA->MODER  &= ~((3u<<(1*2))|(3u<<(2*2))|(3u<<(3*2))|(3u<<(4*2)));
    GPIOA->MODER  |=  ((1u<<(1*2))|(1u<<(2*2))|(1u<<(3*2))|(1u<<(4*2)));
    GPIOA->OTYPER &= ~(GPIO_OTYPER_OT1|GPIO_OTYPER_OT2|GPIO_OTYPER_OT3|GPIO_OTYPER_OT4);
    GPIOA->PUPDR  &= ~((3u<<(1*2))|(3u<<(2*2))|(3u<<(3*2))|(3u<<(4*2)));

    /* default: channel 0 (all address bits LOW) */
    GPIOA->BSRR = ((1u<<1)|(1u<<2)|(1u<<3)|(1u<<4)) << 16;
}

void mux_select(uint8_t ch) {
    ch &= 0x0F;
    uint32_t set = 0, clr = 0;

    if (ch & 1u) set |= (1u<<1); else clr |= (1u<<1);   /* S0 = PA1 */
    if (ch & 2u) set |= (1u<<2); else clr |= (1u<<2);   /* S1 = PA2 */
    if (ch & 4u) set |= (1u<<3); else clr |= (1u<<3);   /* S2 = PA3 */
    if (ch & 8u) set |= (1u<<4); else clr |= (1u<<4);   /* S3 = PA4 */

    /* BSRR: high 16 bits = reset, low 16 bits = set */
    GPIOA->BSRR = set | (clr << 16);
}
