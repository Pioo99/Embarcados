/*
 * T3 — Processamento de impacto (prioridade 3)
 *
 * Consome xQueueImpacto, monta o ImpactPacket_t (região pelos piezos;
 * pico de g e velocidade angular já calculados na origem — T2 BNO08x)
 * e distribui para T4 (xQueueTX) e T5 (xQueueDisplay, sempre o mais
 * recente via xQueueOverwrite).
 */
#include "FreeRTOS.h"
#include "task.h"
#include "freertos_objects.h"
#include "board.h"

#define ACCEL_MAX_G 16.0f

/* Estima a região pelo canal ADC de maior amplitude */
static uint8_t estimar_regiao(uint16_t adc_ch[N_PIEZOS]) {
    /* Canal i → região i (calibrar experimentalmente). */
    static const uint8_t ch_to_region[N_PIEZOS] = {0, 1, 2, 3, 4, 5};

    uint8_t  max_ch  = 0;
    uint16_t max_val = 0;
    for (int i = 0; i < N_PIEZOS; i++) {
        if (adc_ch[i] > max_val) {
            max_val = adc_ch[i];
            max_ch  = i;
        }
    }
    return ch_to_region[max_ch];
}

void vTaskProcessamento(void *pv) {
    (void)pv;
    ImpactRaw_t    raw;
    ImpactPacket_t pkt;
    RegionHit_t    hit;

    for (;;) {
        xQueueReceive(xQueueImpacto, &raw, portMAX_DELAY);

        pkt.timestamp_ms    = raw.timestamp_ms;
        pkt.region_id       = estimar_regiao(raw.adc_ch);
        pkt.peak_accel_g    = raw.peak_accel_g;
        pkt.angular_vel_dps = raw.angular_vel_dps;
        pkt.angle_deg       = raw.angle_deg;
        for (int i = 0; i < N_PIEZOS; i++)
            pkt.adc_peak[i] = raw.adc_ch[i];

        xQueueSend(xQueueTX, &pkt, 0);

        hit.region_id    = pkt.region_id;
        hit.intensity    = pkt.peak_accel_g / ACCEL_MAX_G;
        if (hit.intensity > 1.0f) hit.intensity = 1.0f;
        hit.timestamp_ms = pkt.timestamp_ms;
        xQueueOverwrite(xQueueDisplay, &hit);   /* nunca bloqueia */
    }
}
