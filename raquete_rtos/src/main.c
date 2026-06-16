/*
 * Raquete Instrumentada — FreeRTOS — STM32F411 BlackPill
 *
 * Porte register-level (CMSIS) do firmware HAL freertos_raquete.c.
 * 5 tarefas FreeRTOS, envio ao vivo para o PC via USART1.
 *
 * ── Conexões ─────────────────────────────────────────────────────
 *   Piezos: P0→PA0 P1→PA1 P2→PA2 P3→PA3 P4→PA4 P5→PA6 (ADC1+DMA2)
 *   ST7789: PA5=SCK PA7=MOSI PB0=DC PB1=RST PB6=BLK PB10=CS
 *   IMU:    SCL→PB8 SDA→PB9 RST→PB4 INT→PB5 (I2C1, BNO08x 0x4A, SHTP)
 *   PC:     PA9=USART1_TX → adaptador USB-serial (COM7) @115200
 *
 * ── Tarefas ──────────────────────────────────────────────────────
 *   T1 prio 4  aquisição piezos   periódica 5 ms (DMA em background)
 *   T2 prio 4  fusão IMU/Kalman   periódica 5 ms
 *   T3 prio 3  processamento      xQueueImpacto
 *   T4 prio 2  transmissão UART   xQueueTX
 *   T5 prio 1  display heatmap    periódica 50 ms, xQueueDisplay
 */
#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "board.h"
#include "st7789.h"
#include "adc.h"
#include "uart_tx.h"
#include "freertos_objects.h"
#include "tasks.h"

/* ── Delay por DWT (cycle counter) ──────────────────────────────
   SysTick pertence ao FreeRTOS. delay_us é sempre busy-wait;
   delay_ms cede a CPU com vTaskDelay quando o scheduler roda.   */
static void dwt_init(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
}

void delay_us(uint32_t us) {
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * (SystemCoreClock / 1000000u);
    while ((DWT->CYCCNT - start) < ticks) {}
}

void delay_ms(uint32_t ms) {
    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) {
        vTaskDelay(pdMS_TO_TICKS(ms));
    } else {
        while (ms--) delay_us(1000u);
    }
}

/* ── Clock: HSI → PLL → 96 MHz ──────────────────────────────────── */
static void SystemClock_Config(void) {
    FLASH->ACR &= ~FLASH_ACR_LATENCY;
    FLASH->ACR |= (3u << FLASH_ACR_LATENCY_Pos)
               | FLASH_ACR_PRFTEN | FLASH_ACR_ICEN | FLASH_ACR_DCEN;

    RCC->CR |= RCC_CR_HSION;
    while (!(RCC->CR & RCC_CR_HSIRDY));

    RCC->PLLCFGR = (8u  << RCC_PLLCFGR_PLLM_Pos) |
                   (96u << RCC_PLLCFGR_PLLN_Pos)  |
                   (0u  << RCC_PLLCFGR_PLLP_Pos)  |
                   (4u  << RCC_PLLCFGR_PLLQ_Pos);

    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY));

    RCC->CFGR = RCC_CFGR_HPRE_DIV1 | RCC_CFGR_PPRE1_DIV2 | RCC_CFGR_PPRE2_DIV1;
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);

    SystemCoreClock = 96000000u;
}

/* ── Idle hook — low power entre eventos ────────────────────────── */
void vApplicationIdleHook(void) {
    __WFI();
}

/* ── Hooks de falha (DIAGNÓSTICO) — pintam a tela e travam ──────── */
void vApplicationMallocFailedHook(void) {
    st7789_fill_screen(C_MAG);          /* magenta = heap esgotada */
    for (;;);
}
void vApplicationStackOverflowHook(TaskHandle_t t, char *name) {
    (void)t; (void)name;
    st7789_fill_screen(C_ORANGE);       /* laranja = stack overflow */
    for (;;);
}

/* ── Fault handlers (DIAGNÓSTICO) — cada falha pinta uma cor ────── */
void HardFault_Handler(void)  { st7789_fill_screen(C_WHITE); for (;;); }  /* branco  */
void BusFault_Handler(void)   { st7789_fill_screen(C_YELL);  for (;;); }  /* amarelo */
void UsageFault_Handler(void) { st7789_fill_screen(C_CYAN);  for (;;); }  /* ciano   */
void MemManage_Handler(void)  { st7789_fill_screen(C_GRAY);  for (;;); }  /* cinza   */

int main(void) {
    SCB->CPACR |= ((3UL << 10*2) | (3UL << 11*2));   /* FPU */
    /* DIAGNÓSTICO: roteia Bus/Usage/MemManage para handlers próprios */
    SCB->SHCSR |= SCB_SHCSR_USGFAULTENA_Msk | SCB_SHCSR_BUSFAULTENA_Msk
                | SCB_SHCSR_MEMFAULTENA_Msk;
    SystemClock_Config();
    dwt_init();

    uart1_init();
    adc_dma_init();
    st7789_init();
    /* IMU: bno08x_init() roda dentro de T2 (usa delay_ms que cede ao scheduler) */

    st7789_fill_screen(C_BLACK);
    st7789_draw_text_5x7(40, 110, "Iniciando raquete...", C_WHITE, 1, 0, C_BLACK);

    /* objetos FreeRTOS — antes das tarefas e do scheduler */
    freertos_objects_init();

    xTaskCreate(vTaskAquisicaoPiezos, "T1_Piezo", 256, NULL, 4, NULL);
    xTaskCreate(vTaskFusaoIMU,        "T2_IMU",   512, NULL, 4, NULL);
    xTaskCreate(vTaskProcessamento,   "T3_Proc",  512, NULL, 3, NULL);
    xTaskCreate(vTaskTransmissao,     "T4_TX",    512, NULL, 2, NULL);
    xTaskCreate(vTaskDisplay,         "T5_Disp",  512, NULL, 1, NULL);

    vTaskStartScheduler();

    /* marcador vermelho: scheduler RETORNOU (falha ao criar idle/task) */
    st7789_fill_rect(100, 0, 30, 30, C_RED);
    for (;;);
}
