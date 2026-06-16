/*
 * T1 — Aquisição de piezos (prioridade 4, periódica 5 ms)
 *
 * O DMA mantém adc_buffer atualizado em background. A cada 5 ms T1
 * tira um snapshot dos 6 canais, anexa o ângulo calculado por T2 e
 * envia para T3. Periódica (vTaskDelayUntil) para ceder CPU às
 * demais tarefas — sem isso a T5 (display) é faminta.
 */
#include "FreeRTOS.h"
#include "task.h"
#include "freertos_objects.h"
#include "adc.h"
#include <string.h>

#define PIEZO_PERIOD_MS 5u

void vTaskAquisicaoPiezos(void *pv) {
    (void)pv;
    ImpactRaw_t raw;
    memset(&raw, 0, sizeof(raw));

    adc_dma_start();   /* inicia o scan livre por DMA */
    TickType_t xLastWakeTime = xTaskGetTickCount();

    for (;;) {
        raw.timestamp_ms = xTaskGetTickCount();
        raw.source       = 0;

        adc_snapshot(raw.adc_ch);
        raw.angle_deg       = imu_angle_snapshot;
        raw.angular_vel_dps = imu_angvel_snapshot;
        raw.peak_accel_g    = imu_accel_snapshot;

        /* se a fila estiver cheia, descarta a leitura */
        xQueueSend(xQueueImpacto, &raw, 0);

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(PIEZO_PERIOD_MS));
    }
}
