#include "stm32f4xx.h"
#include "st7789.h"
#include "board.h"
#include "mux.h"
#include "heatmap.h"

/* ── SysTick / delay ─────────────────────────────────────────────── */
static volatile uint32_t g_ms = 0;
void SysTick_Handler(void) { g_ms++; }

void delay_ms(uint32_t ms) {
    uint32_t start = g_ms;
    while ((g_ms - start) < ms) {}
}

static void delay_init(void) {
    SysTick->LOAD = (SystemCoreClock / 1000u) - 1u;
    SysTick->VAL  = 0u;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk
                  | SysTick_CTRL_TICKINT_Msk
                  | SysTick_CTRL_ENABLE_Msk;
}

/* ── Clock: HSI → PLL → 96 MHz ──────────────────────────────────── */
static void SystemClock_Config(void) {
    /* 3 wait states para 96 MHz + prefetch + instruction/data cache */
    FLASH->ACR &= ~FLASH_ACR_LATENCY;
    FLASH->ACR |= (3u << FLASH_ACR_LATENCY_Pos)
               | FLASH_ACR_PRFTEN | FLASH_ACR_ICEN | FLASH_ACR_DCEN;

    /* HSI já é a fonte padrão após SystemInit(); garante ready */
    RCC->CR |= RCC_CR_HSION;
    while (!(RCC->CR & RCC_CR_HSIRDY));

    /* PLL: fonte=HSI, PLLM=8, PLLN=96, PLLP=/2 → 96 MHz; PLLQ=4 */
    RCC->PLLCFGR = (8u  << RCC_PLLCFGR_PLLM_Pos) |
                   (96u << RCC_PLLCFGR_PLLN_Pos)  |
                   (0u  << RCC_PLLCFGR_PLLP_Pos)  | /* 00 = /2 */
                   (4u  << RCC_PLLCFGR_PLLQ_Pos);
    /* PLLSRC bit 22 = 0 → HSI (padrão) */

    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY));

    /* AHB=/1, APB1=/2, APB2=/1 */
    RCC->CFGR = RCC_CFGR_HPRE_DIV1 | RCC_CFGR_PPRE1_DIV2 | RCC_CFGR_PPRE2_DIV1;

    /* Troca SYSCLK para PLL e aguarda confirmação */
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);

    SystemCoreClock = 96000000u;
}

/* ── ADC1: PA0 = IN0, 12 bits, software trigger ──────────────────── */
static void adc1_init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

    /* PA0: modo analógico, sem pull */
    GPIOA->MODER |=  (3u << (0 * 2));
    GPIOA->PUPDR &= ~(3u << (0 * 2));

    /* Prescaler ADC = /4 → 96/4 = 24 MHz (limite é 36 MHz) */
    ADC->CCR = (1u << ADC_CCR_ADCPRE_Pos);

    ADC1->CR1  = 0;           /* resolução 12 bits (bits 25:24 = 00) */
    ADC1->CR2  = 0;           /* trigger software, alinhamento à direita */
    ADC1->SMPR2 = (6u << 0);  /* canal 0: 84 ciclos de amostragem */
    ADC1->SQR1  = 0;          /* 1 conversão na sequência */
    ADC1->SQR3  = 0;          /* 1ª conversão = canal 0 */

    ADC1->CR2 |= ADC_CR2_ADON; /* liga o ADC */
}

static uint32_t adc_read(void) {
    ADC1->CR2 |= ADC_CR2_SWSTART;
    while (!(ADC1->SR & ADC_SR_EOC));
    return ADC1->DR;
}

/* ── Detecção de impacto ─────────────────────────────────────────── */
/* Com pull-down de 1 MΩ nos piezos: τ = 1 MΩ × 30 nF = 30 ms → sinal
   dura vários ciclos de varredura, pico detectável com múltiplas leituras.
   Com pull-down de 10 kΩ nos canais sem piezo: τ = 300 µs → sempre 0 V. */
#define THRESHOLD    700u   /* ~0.56 V — separa impacto de ruído de rede (60 Hz) */
#define DEBOUNCE_MS  200u

static uint8_t  debounce_active[HEAT_NUM_CH];
static uint32_t debounce_ts[HEAT_NUM_CH];

/* Lê o canal N vezes consecutivas sem delay e retorna o valor máximo.
   Captura spikes breves do piezo que decaem em dezenas de ms.
   Não usa confirm_hit (exigir 3 leituras acima rejeita spikes legítimos). */
static uint32_t adc_read_peak(uint8_t n) {
    uint32_t peak = 0;
    for (uint8_t i = 0; i < n; i++) {
        uint32_t v = adc_read();
        if (v > peak) peak = v;
    }
    return peak;
}

/* ── Main ────────────────────────────────────────────────────────── */
int main(void) {
    /* Habilita FPU do Cortex-M4 */
    SCB->CPACR |= ((3UL << 10*2) | (3UL << 11*2));

    SystemClock_Config(); /* configura 96 MHz; atualiza SystemCoreClock */
    delay_init();         /* configura SysTick com base em SystemCoreClock */

    mux_init();
    adc1_init();
    st7789_init();
    heat_init();

    uint32_t last_render = g_ms;

    /* canais físicos do CD4067 onde os piezos estão conectados */
    static const uint8_t MUX_CH[HEAT_NUM_CH] = {0, 5, 10, 15};

    while (1) {
        for (uint8_t ch = 0; ch < HEAT_NUM_CH; ch++) {
            mux_select(MUX_CH[ch]);
            delay_ms(1);               /* estabilização do CD4067 */

            uint32_t val = adc_read_peak(5); /* 5 leituras rápidas, retorna o pico */
            uint32_t now = g_ms;

            if (val > THRESHOLD && !debounce_active[ch]) {
                debounce_active[ch] = 1;
                debounce_ts[ch]     = now;
                heat_add(ch, val);
            }

            if (debounce_active[ch] && (now - debounce_ts[ch] > DEBOUNCE_MS)) {
                debounce_active[ch] = 0;
            }
        }

        if (g_ms - last_render >= 500u) {
            heat_render();
            last_render = g_ms;
        }
    }
}
