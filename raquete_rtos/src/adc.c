/*
 * ADC1 + DMA2 Stream0 — aquisição contínua dos 6 piezos.
 *
 * Porte register-level (CMSIS) do caminho que, no firmware HAL
 * original, usava HAL_ADC_Start_DMA. O DMA circular (CONT) mantém
 * adc_buffer sempre atualizado em background; T1 lê com adc_snapshot
 * de forma periódica (sem interrupção, para não monopolizar a CPU).
 *
 * Sequência scan (6 conversões): IN0 IN1 IN2 IN3 IN4 IN6
 * (PA0 PA1 PA2 PA3 PA4 PA6 — ver board.h). ADC1 servido pelo
 * DMA2 Stream0, canal 0.
 */
#include "stm32f4xx.h"
#include "adc.h"

static volatile uint16_t adc_buffer[N_PIEZOS];   /* preenchido pelo DMA */

void adc_dma_init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_DMA2EN;
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

    /* PA0..PA4 e PA6 em modo analógico */
    const uint8_t pins[N_PIEZOS] = {0, 1, 2, 3, 4, 6};
    for (int i = 0; i < N_PIEZOS; i++) {
        GPIOA->MODER |=  (3u << (pins[i] * 2));   /* analógico */
        GPIOA->PUPDR &= ~(3u << (pins[i] * 2));
    }

    /* ── DMA2 Stream0 canal 0 ← ADC1 (circular, sem IRQ) ───────── */
    DMA2_Stream0->CR = 0;
    while (DMA2_Stream0->CR & DMA_SxCR_EN);            /* aguarda parar */
    DMA2_Stream0->PAR  = (uint32_t)&ADC1->DR;
    DMA2_Stream0->M0AR = (uint32_t)adc_buffer;
    DMA2_Stream0->NDTR = N_PIEZOS;
    DMA2_Stream0->CR =
          (0u << DMA_SxCR_CHSEL_Pos)   /* canal 0  */
        | (1u << DMA_SxCR_MSIZE_Pos)   /* mem  16 bits */
        | (1u << DMA_SxCR_PSIZE_Pos)   /* perif 16 bits */
        | DMA_SxCR_MINC                /* incrementa memória */
        | DMA_SxCR_CIRC;               /* circular */
    DMA2_Stream0->CR |= DMA_SxCR_EN;

    /* ── ADC1: scan multi-canal contínuo via DMA ───────────────── */
    ADC->CCR    = (1u << ADC_CCR_ADCPRE_Pos);          /* PCLK2/4 = 24 MHz */
    ADC1->CR1   = ADC_CR1_SCAN;                        /* varre a sequência */
    ADC1->CR2   = ADC_CR2_DMA | ADC_CR2_DDS | ADC_CR2_CONT;
    ADC1->SMPR2 = 0;
    for (int i = 0; i < 6; i++)                         /* 144 ciclos por canal */
        ADC1->SMPR2 |= (6u << (i * 3));

    /* sequência regular: 6 conversões */
    ADC1->SQR1 = ((N_PIEZOS - 1u) << ADC_SQR1_L_Pos);
    ADC1->SQR3 = (0u  << 0)  | (1u << 5)  | (2u << 10)
               | (3u  << 15) | (4u << 20) | (6u << 25);

    ADC1->CR2 |= ADC_CR2_ADON;
}

void adc_dma_start(void) {
    ADC1->CR2 |= ADC_CR2_SWSTART;                      /* dispara scan livre */
}

/* Cópia atômica-por-elemento do último quadro DMA */
void adc_snapshot(uint16_t dst[N_PIEZOS]) {
    for (int i = 0; i < N_PIEZOS; i++) dst[i] = adc_buffer[i];
}
