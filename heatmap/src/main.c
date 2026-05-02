#include "stm32f4xx_hal.h"
#include "st7789.h"
#include "board.h"
#include "mux.h"
#include "heatmap.h"

/* delay_ms exigido pelo st7789.c — delega ao HAL (que controla o SysTick) */
void delay_ms(uint32_t ms) { HAL_Delay(ms); }

/* ── ADC ─────────────────────────────────────────────────────────── */
static ADC_HandleTypeDef hadc1;

/* ── Clock: HSI → PLL → 96 MHz ──────────────────────────────────── */
static void SystemClock_Config(void) {
    RCC_OscInitTypeDef osc = {0};
    osc.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
    osc.HSIState            = RCC_HSI_ON;
    osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    osc.PLL.PLLState        = RCC_PLL_ON;
    osc.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
    osc.PLL.PLLM            = 8;
    osc.PLL.PLLN            = 96;
    osc.PLL.PLLP            = RCC_PLLP_DIV2;
    osc.PLL.PLLQ            = 4;
    HAL_RCC_OscConfig(&osc);

    RCC_ClkInitTypeDef clk = {0};
    clk.ClockType      = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK
                       | RCC_CLOCKTYPE_PCLK1  | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV2;
    clk.APB2CLKDivider = RCC_HCLK_DIV1;
    HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_3);
}

/* ── ADC1: PA0 = IN0, 12 bits, software trigger ──────────────────── */
static void adc1_init(void) {
    __HAL_RCC_ADC1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};
    gpio.Pin  = GPIO_PIN_0;
    gpio.Mode = GPIO_MODE_ANALOG;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &gpio);

    hadc1.Instance                   = ADC1;
    hadc1.Init.Resolution            = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode          = DISABLE;
    hadc1.Init.ContinuousConvMode    = DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion       = 1;
    hadc1.Init.DMAContinuousRequests = DISABLE;
    hadc1.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;
    HAL_ADC_Init(&hadc1);

    ADC_ChannelConfTypeDef ch = {0};
    ch.Channel      = ADC_CHANNEL_0;
    ch.Rank         = 1;
    ch.SamplingTime = ADC_SAMPLETIME_84CYCLES;
    HAL_ADC_ConfigChannel(&hadc1, &ch);
}

static uint32_t adc_read(void) {
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
    return HAL_ADC_GetValue(&hadc1);
}

/* ── Detecção de impacto ─────────────────────────────────────────── */
#define THRESHOLD    300u   /* ~0.24 V */
#define DEBOUNCE_MS  150u

static uint8_t  debounce_active[HEAT_NUM_CH];
static uint32_t debounce_ts[HEAT_NUM_CH];

/* ── Main ────────────────────────────────────────────────────────── */
int main(void) {
    HAL_Init();
    SystemClock_Config();

    mux_init();
    adc1_init();
    st7789_init();
    heat_init();

    uint32_t last_render = HAL_GetTick();

    while (1) {
        for (uint8_t ch = 0; ch < HEAT_NUM_CH; ch++) {
            mux_select(ch);
            HAL_Delay(1);               /* settle do CD4067 */

            uint32_t val = adc_read();
            uint32_t now = HAL_GetTick();

            if (val > THRESHOLD && !debounce_active[ch]) {
                debounce_active[ch] = 1;
                debounce_ts[ch]     = now;
                heat_add(ch, val);
            }

            if (debounce_active[ch] && (now - debounce_ts[ch] > DEBOUNCE_MS)) {
                debounce_active[ch] = 0;
            }
        }

        if (HAL_GetTick() - last_render >= 500u) {
            heat_render();
            last_render = HAL_GetTick();
        }
    }
}
