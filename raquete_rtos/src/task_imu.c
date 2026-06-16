/*
 * T2 — Amostragem inercial / IMU BNO08x (prioridade 4, periódica 5 ms)
 *
 * O BNO085 (GY-BN008X @ 0x4A, SHTP) faz a fusão internamente e entrega
 * ângulo (rotation vector → euler) e velocidade angular (giro calibrado)
 * prontos. T2 NÃO envia frames próprios: apenas atualiza os snapshots
 * imu_angle_snapshot / imu_angvel_snapshot, que T1 (piezo) anexa a cada
 * frame. Assim a IMU não disputa a xQueueImpacto com o piezo (que, em
 * 200/s, enchia a fila e descartava os frames da IMU).
 */
#include "FreeRTOS.h"
#include "task.h"
#include "freertos_objects.h"
#include "bno08x.h"
#include "board.h"
#include <string.h>

#define IMU_PERIOD_MS 5u

void vTaskFusaoIMU(void *pv) {
    (void)pv;

    bno08x_init();                             /* reset HW + SHTP + habilita reports */

    /* persiste entre leituras: cada read traz só 1 report (ângulo OU giro);
       o outro campo mantém o último valor conhecido */
    BNO08X_Data data;
    memset(&data, 0, sizeof(data));

    TickType_t xLastWakeTime = xTaskGetTickCount();

    for (;;) {
        uint8_t got = 0;
        /* drena até 8 pacotes pendentes; guarda o mais recente */
        for (int n = 0; n < 8 && bno08x_read(&data) && data.valid; n++) got = 1;

        if (got) {
            imu_angle_snapshot  = data.pitch;   /* eixo da raquete — trocar p/ roll/yaw se preciso */
            imu_angvel_snapshot = data.speed;   /* °/s, magnitude do giro */
            imu_accel_snapshot  = data.accel_g; /* g, magnitude da aceleração */
        }

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(IMU_PERIOD_MS));
    }
}
