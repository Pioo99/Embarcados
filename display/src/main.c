#include "stm32f4xx_hal.h"
#include <stdio.h>
#include <string.h>

UART_HandleTypeDef huart1;
ADC_HandleTypeDef  hadc1;

/* ── Declarações antecipadas ───────────────────────────── */
void SystemClock_Config(void);
void UART1_Init(void);
void ADC1_Init(void);
void uart_print(const char *msg);

/* ── Clock: HSI → PLL → 96 MHz ────────────────────────── */
void SystemClock_Config(void) {
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

/* ── UART1: PA9=TX, PA10=RX, 115200 8N1 ───────────────── */
void UART1_Init(void) {
    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};
    gpio.Pin       = GPIO_PIN_9 | GPIO_PIN_10;
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_NOPULL;
    gpio.Speed     = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &gpio);

    huart1.Instance          = USART1;
    huart1.Init.BaudRate     = 115200;
    huart1.Init.WordLength   = UART_WORDLENGTH_8B;
    huart1.Init.StopBits     = UART_STOPBITS_1;
    huart1.Init.Parity       = UART_PARITY_NONE;
    huart1.Init.Mode         = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    HAL_UART_Init(&huart1);
}

/* ── ADC1: PA0 = canal IN0, 12 bits ───────────────────── */
void ADC1_Init(void) {
    __HAL_RCC_ADC1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};
    gpio.Pin  = GPIO_PIN_0;
    gpio.Mode = GPIO_MODE_ANALOG;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &gpio);

    hadc1.Instance                        = ADC1;
    hadc1.Init.Resolution                 = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode               = DISABLE;
    hadc1.Init.ContinuousConvMode         = DISABLE;
    hadc1.Init.DiscontinuousConvMode      = DISABLE;
    hadc1.Init.ExternalTrigConvEdge       = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.ExternalTrigConv           = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign                  = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion            = 1;
    hadc1.Init.DMAContinuousRequests      = DISABLE;
    hadc1.Init.EOCSelection               = ADC_EOC_SINGLE_CONV;
    HAL_ADC_Init(&hadc1);

    ADC_ChannelConfTypeDef canal = {0};
    canal.Channel      = ADC_CHANNEL_0;
    canal.Rank         = 1;
    canal.SamplingTime = ADC_SAMPLETIME_84CYCLES;
    HAL_ADC_ConfigChannel(&hadc1, &canal);
}

/* ── Envia string pela UART ────────────────────────────── */
void uart_print(const char *msg) {
    HAL_UART_Transmit(&huart1, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
}

/* ── Main ──────────────────────────────────────────────── */
int main(void) {
    HAL_Init();
    SystemClock_Config();
    UART1_Init();
    ADC1_Init();

    char     buf[64];
    uint32_t ultimo_impacto = 0;
    uint8_t  impacto_ativo  = 0;

    #define THRESHOLD    300     /* ~0.24V — ajuste conforme seu piezo  */
    #define DEBOUNCE_MS  150     /* ignora re-trigger por 150 ms        */

    uart_print("=== Leitura piezoeletrico iniciada ===\r\n");

    while (1) {
        /* ── Leitura ADC ── */
        HAL_ADC_Start(&hadc1);
        HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
        uint32_t valor = HAL_ADC_GetValue(&hadc1);

        /* ── Converte para tensão ── */
        float tensao = (valor * 3.3f) / 4095.0f;

        /* ── Envia leitura contínua a cada 100 ms ── */
        snprintf(buf, sizeof(buf), "ADC: %4lu  |  %.3f V\r\n", valor, tensao);
        uart_print(buf);

        /* ── Detecção de impacto com debounce ── */
        uint32_t agora = HAL_GetTick();

        if (valor > THRESHOLD && !impacto_ativo) {
            impacto_ativo  = 1;
            ultimo_impacto = agora;

            snprintf(buf, sizeof(buf),
                     ">>> IMPACTO! ADC: %lu  |  %.3f V\r\n", valor, tensao);
            uart_print(buf);
        }

        if (impacto_ativo && (agora - ultimo_impacto > DEBOUNCE_MS)) {
            impacto_ativo = 0;
        }

        HAL_Delay(100);
    }
}