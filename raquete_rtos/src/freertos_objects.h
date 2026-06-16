#pragma once
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "board.h"

/* ── ImpactRaw_t — T1/T2 → T3 via xQueueImpacto ─────────────────── */
typedef struct {
    uint32_t timestamp_ms;
    uint16_t adc_ch[N_PIEZOS];   /* leituras brutas dos piezos       */
    float    peak_accel_g;       /* pico de aceleração já em g        */
    float    angular_vel_dps;    /* velocidade angular já em °/s      */
    float    angle_deg;          /* ângulo fundido do BNO08x          */
    uint8_t  source;             /* 0 = T1 piezos, 1 = T2 IMU         */
} ImpactRaw_t;

/* ── ImpactPacket_t — T3 → T4 via xQueueTX ──────────────────────── */
typedef struct {
    uint32_t timestamp_ms;
    uint8_t  region_id;          /* 0=centro, 1-4=bordas, 5-8=cantos */
    float    peak_accel_g;
    float    angular_vel_dps;
    float    angle_deg;
    uint16_t adc_peak[N_PIEZOS];
} ImpactPacket_t;

/* ── RegionHit_t — T3 → T5 via xQueueDisplay ────────────────────── */
typedef struct {
    uint8_t  region_id;
    float    intensity;          /* 0.0 – 1.0 normalizado */
    uint32_t timestamp_ms;
} RegionHit_t;

/* ── Handles globais ────────────────────────────────────────────── */
/* snapshots mais recentes da IMU — escritos por T2, lidos por T1 */
extern volatile float imu_angle_snapshot;
extern volatile float imu_angvel_snapshot;
extern volatile float imu_accel_snapshot;
extern QueueHandle_t     xQueueImpacto;
extern QueueHandle_t     xQueueTX;
extern QueueHandle_t     xQueueDisplay;

/* Cria semáforos e filas — chamar em main() antes do scheduler */
void freertos_objects_init(void);
