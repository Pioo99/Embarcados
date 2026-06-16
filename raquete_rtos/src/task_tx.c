/*
 * T4 — Transmissão UART (prioridade 2)
 *
 * Serializa cada ImpactPacket_t em um frame binário de 32 bytes
 * via USART1 (PA9, 115200 8N1). Decodificação: pc/raquete_receiver.py.
 *
 *   [0xAA | len(1) | ts(4) | region(1) | accel(4) | ang_vel(4) |
 *    angle(4) | adc[6](12) | 0x55]   — tudo little-endian
 */
#include "FreeRTOS.h"
#include "freertos_objects.h"
#include "uart_tx.h"
#include <string.h>

#define FRAME_START 0xAA
#define FRAME_END   0x55
#define FRAME_SIZE  32

void vTaskTransmissao(void *pv) {
    (void)pv;
    ImpactPacket_t pkt;
    uint8_t frame[FRAME_SIZE];

    for (;;) {
        xQueueReceive(xQueueTX, &pkt, portMAX_DELAY);

        uint8_t *p = frame;
        *p++ = FRAME_START;
        *p++ = FRAME_SIZE - 2;              /* payload sem start/end */

        memcpy(p, &pkt.timestamp_ms,    4); p += 4;
        *p++ = pkt.region_id;
        memcpy(p, &pkt.peak_accel_g,    4); p += 4;
        memcpy(p, &pkt.angular_vel_dps, 4); p += 4;
        memcpy(p, &pkt.angle_deg,       4); p += 4;
        memcpy(p, pkt.adc_peak, N_PIEZOS * sizeof(uint16_t)); p += 12;
        *p = FRAME_END;

        uart1_send(frame, FRAME_SIZE);
    }
}
