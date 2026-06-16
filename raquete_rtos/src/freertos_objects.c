#include "freertos_objects.h"

QueueHandle_t     xQueueImpacto;
QueueHandle_t     xQueueTX;
QueueHandle_t     xQueueDisplay;

volatile float    imu_angle_snapshot  = 0.0f;
volatile float    imu_angvel_snapshot = 0.0f;
volatile float    imu_accel_snapshot  = 0.0f;

void freertos_objects_init(void) {
    xQueueImpacto = xQueueCreate(8, sizeof(ImpactRaw_t));
    xQueueTX      = xQueueCreate(4, sizeof(ImpactPacket_t));
    xQueueDisplay = xQueueCreate(1, sizeof(RegionHit_t));  /* len 1: xQueueOverwrite */

    configASSERT(xQueueImpacto && xQueueTX && xQueueDisplay);
}
