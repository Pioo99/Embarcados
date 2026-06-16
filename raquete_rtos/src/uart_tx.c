#include "stm32f4xx.h"
#include "uart_tx.h"

void uart1_init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

    /* PA9 → AF7 (USART1_TX) */
    GPIOA->MODER   &= ~(3u << (9 * 2));
    GPIOA->MODER   |=  (2u << (9 * 2));
    GPIOA->AFR[1]  &= ~(0xFu << ((9 - 8) * 4));
    GPIOA->AFR[1]  |=  (7u   << ((9 - 8) * 4));
    GPIOA->OSPEEDR |=  (2u << (9 * 2));

    /* APB2 = 96 MHz → 115200: mantissa 52, fração 1 (erro 0,04%) */
    USART1->BRR = (52u << 4) | 1u;
    USART1->CR1 = USART_CR1_UE | USART_CR1_TE;
}

#define UART_TIMEOUT 100000u   /* guarda contra USART travada */

void uart1_send(const uint8_t *buf, uint16_t len) {
    for (uint16_t i = 0; i < len; i++) {
        uint32_t t = UART_TIMEOUT;
        while (!(USART1->SR & USART_SR_TXE)) { if (--t == 0) return; }
        USART1->DR = buf[i];
    }
    uint32_t t = UART_TIMEOUT;
    while (!(USART1->SR & USART_SR_TC)) { if (--t == 0) return; }
}
