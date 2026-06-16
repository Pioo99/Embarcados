#include "bno08x.h"
#include "board.h"
#include "stm32f4xx.h"
#include <math.h>
#include <string.h>

extern void delay_ms(uint32_t ms);

#define RX_BUF_SIZE  284
#define TX_BUF_SIZE   21

static uint8_t rx_buf[RX_BUF_SIZE];
static uint8_t seq[6] = {0};

/* DIAGNÓSTICO: quantos reports de cada tipo já foram parseados */
volatile uint32_t bno_rot_cnt  = 0;
volatile uint32_t bno_gyro_cnt = 0;

/* ══════════════════════════════════════════════════════════════════
   Reset hardware do BNO085 via PB4
   ══════════════════════════════════════════════════════════════════ */
static void bno08x_hw_reset(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;

    /* RST (PB4) → output push-pull */
    GPIOB->MODER  &= ~(3u << (BNO_RST_PIN * 2));
    GPIOB->MODER  |=  (1u << (BNO_RST_PIN * 2));
    GPIOB->OTYPER &= ~(1u << BNO_RST_PIN);
    GPIOB->PUPDR  &= ~(3u << (BNO_RST_PIN * 2));

    /* INT (PB5) → input com pull-up (ativo-baixo) */
    GPIOB->MODER  &= ~(3u << (BNO_INT_PIN * 2));
    GPIOB->PUPDR  &= ~(3u << (BNO_INT_PIN * 2));
    GPIOB->PUPDR  |=  (1u << (BNO_INT_PIN * 2));

    /* Pulso de reset: LOW 10ms → HIGH → aguarda boot completo */
    GPIOB->BSRR = (1u << (BNO_RST_PIN + 16)); /* RST = LOW */
    delay_ms(10);
    GPIOB->BSRR = (1u << BNO_RST_PIN);        /* RST = HIGH */
    delay_ms(600);
}

/* ══════════════════════════════════════════════════════════════════
   Verifica se o sensor tem dados prontos (INT ativo-baixo)
   ══════════════════════════════════════════════════════════════════ */
uint8_t bno08x_data_ready(void) {
    return !(GPIOB->IDR & (1u << BNO_INT_PIN));
}

/* ══════════════════════════════════════════════════════════════════
   I2C recovery
   ══════════════════════════════════════════════════════════════════ */
static void i2c_recover(void) {
    GPIOB->MODER  &= ~((3u<<(8*2))|(3u<<(9*2)));
    GPIOB->MODER  |=  ((1u<<(8*2))|(1u<<(9*2)));
    GPIOB->OTYPER |=  (GPIO_OTYPER_OT8|GPIO_OTYPER_OT9);
    GPIOB->PUPDR  &= ~((3u<<(8*2))|(3u<<(9*2)));
    GPIOB->PUPDR  |=  ((1u<<(8*2))|(1u<<(9*2)));
    GPIOB->BSRR = (1u<<8)|(1u<<9); delay_ms(2);
    for (int i = 0; i < 9; i++) {
        GPIOB->BSRR = (1u<<(8+16)); delay_ms(1);
        GPIOB->BSRR = (1u<<8);      delay_ms(1);
        if (GPIOB->IDR & (1u<<9)) break;
    }
    GPIOB->BSRR = (1u<<(9+16)); delay_ms(1);
    GPIOB->BSRR = (1u<<8);      delay_ms(1);
    GPIOB->BSRR = (1u<<9);      delay_ms(5);
}

static void i2c_reinit(void) {
    GPIOB->MODER  &= ~((3u<<(8*2))|(3u<<(9*2)));
    GPIOB->MODER  |=  ((2u<<(8*2))|(2u<<(9*2)));
    GPIOB->AFR[1] &= ~((0xFu<<0)|(0xFu<<4));
    GPIOB->AFR[1] |=  ((4u<<0)|(4u<<4));
    I2C1->CR1 = I2C_CR1_SWRST; delay_ms(5); I2C1->CR1 = 0;
    I2C1->CR2 = 48u; I2C1->TRISE = 15u;
    I2C1->CCR = I2C_CCR_FS | 60u;
    I2C1->CR1 = I2C_CR1_PE; delay_ms(5);
}

static void i2c1_init(void) {
    RCC->AHB1ENR  |= RCC_AHB1ENR_GPIOBEN;
    RCC->APB1ENR  |= RCC_APB1ENR_I2C1EN;
    RCC->APB1RSTR |=  RCC_APB1RSTR_I2C1RST;
    RCC->APB1RSTR &= ~RCC_APB1RSTR_I2C1RST;
    GPIOB->OTYPER  |= (GPIO_OTYPER_OT8|GPIO_OTYPER_OT9);
    GPIOB->OSPEEDR |= ((3u<<(8*2))|(3u<<(9*2)));
    i2c_recover();
    i2c_reinit();
}

static uint8_t i2c_ensure_free(void) {
    uint32_t t = 50000;
    while ((I2C1->SR2 & I2C_SR2_BUSY) && --t);
    if (t) return 1;
    i2c_recover(); i2c_reinit();
    t = 50000;
    while ((I2C1->SR2 & I2C_SR2_BUSY) && --t);
    return (t > 0);
}

static uint8_t i2c_start(void) {
    if (!i2c_ensure_free()) return 0;
    I2C1->CR1 |= I2C_CR1_START;
    uint32_t t = 100000;
    while (!(I2C1->SR1 & I2C_SR1_SB) && --t);
    return (t > 0);
}

static uint8_t i2c_addr(uint8_t addr, uint8_t rw) {
    I2C1->DR = (uint8_t)((addr << 1) | rw);
    uint32_t t = 100000;
    while (!(I2C1->SR1 & (I2C_SR1_ADDR | I2C_SR1_AF)) && --t);
    if (!t || (I2C1->SR1 & I2C_SR1_AF)) {
        I2C1->SR1 &= ~I2C_SR1_AF;
        I2C1->CR1 |= I2C_CR1_STOP;
        return 0;
    }
    (void)I2C1->SR2;
    return 1;
}

static void i2c_stop(void) {
    I2C1->CR1 |= I2C_CR1_STOP;
    uint32_t t = 100000;
    while ((I2C1->CR1 & I2C_CR1_STOP) && --t);
    delay_ms(2);
}

static uint8_t i2c_write(uint8_t b) {
    I2C1->DR = b;
    uint32_t t = 100000;
    while (!(I2C1->SR1 & (I2C_SR1_TXE | I2C_SR1_BTF | I2C_SR1_AF)) && --t);
    if (!t || (I2C1->SR1 & I2C_SR1_AF)) { I2C1->SR1 &= ~I2C_SR1_AF; return 0; }
    return 1;
}

static uint8_t i2c_read(uint8_t ack) {
    if (ack) I2C1->CR1 |=  I2C_CR1_ACK;
    else     I2C1->CR1 &= ~I2C_CR1_ACK;
    uint32_t t = 100000;
    while (!(I2C1->SR1 & I2C_SR1_RXNE) && --t);
    return I2C1->DR;
}

/* ══════════════════════════════════════════════════════════════════
   SHTP
   ══════════════════════════════════════════════════════════════════ */
static uint16_t shtp_read_full(uint8_t *buf, uint16_t buflen) {
    uint16_t total_read = 0;
    uint16_t remaining  = 0;

    if (!i2c_start())               return 0;
    if (!i2c_addr(BNO08X_ADDR, 1)) return 0;

    uint8_t h0 = i2c_read(1);
    uint8_t h1 = i2c_read(1);
    uint8_t h2 = i2c_read(1);
    uint8_t h3 = i2c_read(1);
    (void)h2; (void)h3;

    uint16_t pkt_len = (uint16_t)(((h1 & 0x7F) << 8) | h0);
    uint8_t  is_cont = (h1 & 0x80) ? 1 : 0;

    if (pkt_len < 4) { i2c_stop(); return 0; }

    uint16_t payload = pkt_len - 4;
    uint16_t rd = (payload > buflen) ? buflen : payload;

    for (uint16_t i = 0; i < rd; i++)
        buf[i] = i2c_read(i < rd - 1 ? 1 : 0);
    i2c_stop();

    total_read = rd;
    remaining  = (payload > rd) ? (payload - rd) : 0;

    while (remaining > 0 && !is_cont) {
        delay_ms(1);
        if (!i2c_start())               break;
        if (!i2c_addr(BNO08X_ADDR, 1)) break;

        h0 = i2c_read(1); h1 = i2c_read(1);
        h2 = i2c_read(1); h3 = i2c_read(1);
        (void)h2; (void)h3;

        uint16_t frag_len = (uint16_t)(((h1 & 0x7F) << 8) | h0);
        if (frag_len < 4) { i2c_stop(); break; }

        uint16_t frag_pay = frag_len - 4;
        uint16_t frag_rd  = (frag_pay > (buflen - total_read))
                          ? (buflen - total_read) : frag_pay;

        for (uint16_t i = 0; i < frag_rd; i++)
            buf[total_read + i] = i2c_read(i < frag_rd - 1 ? 1 : 0);
        i2c_stop();

        total_read += frag_rd;
        remaining   = (remaining > frag_rd) ? (remaining - frag_rd) : 0;
    }

    return total_read;
}

static uint8_t shtp_send(uint8_t channel, uint8_t *data, uint16_t len) {
    uint16_t total = len + 4;
    if (!i2c_start())               return 0;
    if (!i2c_addr(BNO08X_ADDR, 0)) return 0;
    if (!i2c_write(total & 0xFF))        { i2c_stop(); return 0; }
    if (!i2c_write((total >> 8) & 0x7F)) { i2c_stop(); return 0; }
    if (!i2c_write(channel))             { i2c_stop(); return 0; }
    if (!i2c_write(seq[channel]++))      { i2c_stop(); return 0; }
    for (uint16_t i = 0; i < len; i++)
        if (!i2c_write(data[i]))         { i2c_stop(); return 0; }
    i2c_stop();
    return 1;
}

static uint8_t enable_report(uint8_t rid, uint32_t us) {
    uint8_t cmd[TX_BUF_SIZE];
    memset(cmd, 0, TX_BUF_SIZE);
    cmd[0] = SHTP_REPORT_SET_FEATURE_COMMAND;
    cmd[1] = rid;
    cmd[5] = (us >>  0) & 0xFF;
    cmd[6] = (us >>  8) & 0xFF;
    cmd[7] = (us >> 16) & 0xFF;
    cmd[8] = (us >> 24) & 0xFF;
    for (int i = 0; i < 5; i++) {
        if (shtp_send(SHTP_CHAN_CONTROL, cmd, TX_BUF_SIZE)) return 1;
        delay_ms(100);
    }
    return 0;
}

static void drain(void) {
    for (int i = 0; i < 15; i++) {
        shtp_read_full(rx_buf, RX_BUF_SIZE);
        delay_ms(30);
    }
}

/* ── Quaternion → Euler ──────────────────────────────────────────── */
static void quat_to_euler(float i, float j, float k, float r,
                           float *roll, float *pitch, float *yaw) {
    float sinr = 2.0f * (r*i + j*k);
    float cosr = 1.0f - 2.0f * (i*i + j*j);
    *roll  = atan2f(sinr, cosr) * 57.2957795f;
    float sinp = 2.0f * (r*j - k*i);
    *pitch = (fabsf(sinp) >= 1.0f) ? copysignf(90.0f, sinp)
                                    : asinf(sinp) * 57.2957795f;
    float siny = 2.0f * (r*k + i*j);
    float cosy = 1.0f - 2.0f * (j*j + k*k);
    *yaw   = atan2f(siny, cosy) * 57.2957795f;
}

/* ══════════════════════════════════════════════════════════════════
   API pública
   ══════════════════════════════════════════════════════════════════ */
uint8_t bno08x_init(void) {
    memset(seq, 0, sizeof(seq));

    /* Reset hardware ANTES do I2C — garante boot limpo do BNO085 */
    bno08x_hw_reset();

    i2c1_init();
    delay_ms(100);
    drain();

    enable_report(SHTP_REPORT_ROTATION_VECTOR,      10000); /* 100Hz */
    delay_ms(200);
    enable_report(SHTP_REPORT_GYROSCOPE_CALIBRATED, 10000); /* 100Hz */
    delay_ms(200);
    enable_report(SHTP_REPORT_ACCELEROMETER,        10000); /* 100Hz */
    delay_ms(200);

    return 1;
}

uint8_t bno08x_read(BNO08X_Data *out) {
    out->valid = 0;

    /* Só lê se o sensor sinalizou dados via INT (ativo-baixo) */
    if (!bno08x_data_ready()) return 0;

    uint16_t len = shtp_read_full(rx_buf, RX_BUF_SIZE);
    if (len < 5) return 0;

    uint8_t rid;
    uint8_t base = 0;

    if (len >= 6 && rx_buf[0] == 0xFB) {
        base = 5;
    }

    if (len < base + 1) return 0;
    rid = rx_buf[base];

    if (rid == SHTP_REPORT_ROTATION_VECTOR && len >= base + 14) {
        float sc = 1.0f / (float)(1 << 14);
        float qi = (int16_t)(rx_buf[base+4] | (rx_buf[base+5]  << 8)) * sc;
        float qj = (int16_t)(rx_buf[base+6] | (rx_buf[base+7]  << 8)) * sc;
        float qk = (int16_t)(rx_buf[base+8] | (rx_buf[base+9]  << 8)) * sc;
        float qr = (int16_t)(rx_buf[base+10]| (rx_buf[base+11] << 8)) * sc;
        quat_to_euler(qi, qj, qk, qr,
                      &out->roll, &out->pitch, &out->yaw);
        bno_rot_cnt++;
        out->valid = 1;
        return 1;
    }

    if (rid == SHTP_REPORT_GYROSCOPE_CALIBRATED && len >= base + 10) {
        float sc = 1.0f / (float)(1 << 9);
        out->gyro_x = (int16_t)(rx_buf[base+4] | (rx_buf[base+5] << 8)) * sc;
        out->gyro_y = (int16_t)(rx_buf[base+6] | (rx_buf[base+7] << 8)) * sc;
        out->gyro_z = (int16_t)(rx_buf[base+8] | (rx_buf[base+9] << 8)) * sc;
        float mag = sqrtf(out->gyro_x*out->gyro_x +
                          out->gyro_y*out->gyro_y +
                          out->gyro_z*out->gyro_z);
        out->speed = mag * 57.2957795f;
        bno_gyro_cnt++;
        out->valid = 1;
        return 1;
    }

    if (rid == SHTP_REPORT_ACCELEROMETER && len >= base + 10) {
        float sc = 1.0f / 256.0f;                 /* Q8 → m/s² */
        float ax = (int16_t)(rx_buf[base+4] | (rx_buf[base+5] << 8)) * sc;
        float ay = (int16_t)(rx_buf[base+6] | (rx_buf[base+7] << 8)) * sc;
        float az = (int16_t)(rx_buf[base+8] | (rx_buf[base+9] << 8)) * sc;
        float mag = sqrtf(ax*ax + ay*ay + az*az);
        out->accel_g = mag / 9.80665f;            /* m/s² → g */
        out->valid = 1;
        return 1;
    }

    return 0;
}
