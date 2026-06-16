/*
 * T5 — Display heatmap (prioridade 1, periódica 50 ms)
 *
 * Drena xQueueDisplay (RegionHit_t), acumula a contagem por região,
 * normaliza pela região mais atingida e redesenha o heatmap 3x3.
 */
#include "FreeRTOS.h"
#include "task.h"
#include "freertos_objects.h"
#include "heatmap.h"
#include "st7789.h"
#include "board.h"

void vTaskDisplay(void *pv) {
    (void)pv;
    RegionHit_t hit;
    uint32_t    hit_count[N_REGIOES] = {0};
    uint32_t    max_hits = 1;
    TickType_t  xLastWakeTime = xTaskGetTickCount();

    heat_init();

    for (;;) {
        /* drena todos os itens pendentes sem bloquear */
        while (xQueueReceive(xQueueDisplay, &hit, 0) == pdTRUE) {
            if (hit.region_id < N_REGIOES) {
                hit_count[hit.region_id]++;
                if (hit_count[hit.region_id] > max_hits)
                    max_hits = hit_count[hit.region_id];
            }
        }

        for (int r = 0; r < N_REGIOES; r++) {
            float intensidade = (float)hit_count[r] / (float)max_hits;
            heat_draw_region(r, intensidade);
        }

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(50));
    }
}
