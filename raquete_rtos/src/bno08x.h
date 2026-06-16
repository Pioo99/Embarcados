#pragma once
#include <stdint.h>

/* ── Endereço I2C ───────────────────────────────────────────────────
   ADDR/PS1 = GND (padrão do GY-BN008X) → 0x4A
   ADDR/PS1 = 3V3                        → 0x4B
   ────────────────────────────────────────────────────────────────── */
#define BNO08X_ADDR   0x4A   /* CORRIGIDO: era 0x4B */

/* ── Pinos de controle ──────────────────────────────────────────── */
#define BNO_RST_PORT  GPIOB
#define BNO_RST_PIN   4
#define BNO_INT_PORT  GPIOB
#define BNO_INT_PIN   5

/* ── IDs de relatório SHTP ──────────────────────────────────────── */
#define SHTP_CHAN_COMMAND   0
#define SHTP_CHAN_EXE       1
#define SHTP_CHAN_CONTROL   2
#define SHTP_CHAN_REPORTS   3

#define SHTP_REPORT_ACCELEROMETER        0x01
#define SHTP_REPORT_ROTATION_VECTOR      0x05
#define SHTP_REPORT_GYROSCOPE_CALIBRATED 0x02
#define SHTP_REPORT_PRODUCT_ID_REQUEST   0xF9
#define SHTP_REPORT_PRODUCT_ID_RESPONSE  0xF8
#define SHTP_REPORT_SET_FEATURE_COMMAND  0xFD

/* ── Dados lidos do sensor ──────────────────────────────────────── */
typedef struct {
    float pitch;      /* graus */
    float roll;       /* graus */
    float yaw;        /* graus */
    float gyro_x;     /* rad/s */
    float gyro_y;     /* rad/s */
    float gyro_z;     /* rad/s */
    float speed;      /* magnitude do giroscópio em graus/s */
    float accel_g;    /* magnitude da aceleração em g (inclui gravidade) */
    uint8_t valid;
} BNO08X_Data;

/* ── API pública ─────────────────────────────────────────────────── */
uint8_t bno08x_init(void);
uint8_t bno08x_read(BNO08X_Data *out);
uint8_t bno08x_data_ready(void);   /* verifica pino INT */

/* DIAGNÓSTICO: contagem de reports parseados por tipo */
extern volatile uint32_t bno_rot_cnt;
extern volatile uint32_t bno_gyro_cnt;
