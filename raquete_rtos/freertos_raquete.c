/* =============================================================================
 * Raquete Instrumentada — Firmware FreeRTOS
 * Plataforma : BlackPill (STM32F411CEU6)
 * Periféricos: ADC1/DMA (piezos), I2C1 (IMU BNO085), USART1 (USB-Serial/HC-12)
 * Tarefas    : T1 Aquisição piezos | T2 Fusão IMU | T3 Processamento
 *              T4 Transmissão UART | T5 Display heatmap
 * ============================================================================= */

#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include <string.h>
#include <math.h>

/* -----------------------------------------------------------------------------
 * Handles de periféricos (gerados pelo CubeMX)
 * ----------------------------------------------------------------------------- */
extern ADC_HandleTypeDef  hadc1;
extern DMA_HandleTypeDef  hdma_adc1;
extern I2C_HandleTypeDef  hi2c1;
extern UART_HandleTypeDef huart1;

/* -----------------------------------------------------------------------------
 * Constantes do sistema
 * ----------------------------------------------------------------------------- */
#define N_PIEZOS        6
#define N_REGIOES       9       /* 0=centro, 1-4=bordas, 5-8=cantos */
#define ACCEL_MAX_G     16.0f
#define FRAME_START     0xAA
#define FRAME_END       0x55
#define FRAME_SIZE      32

/* -----------------------------------------------------------------------------
 * Estruturas de dados trafegadas pelas filas
 * ----------------------------------------------------------------------------- */

/* T1/T2 → T3 */
typedef struct {
    uint32_t timestamp_ms;
    uint16_t adc_ch[N_PIEZOS];  /* leituras brutas dos piezos */
    int16_t  accel_xyz[3];      /* acelerômetro em LSB        */
    int16_t  gyro_xyz[3];       /* giroscópio em LSB          */
    float    angle_deg;         /* ângulo calculado pelo filtro */
    uint8_t  source;            /* 0 = T1 piezos, 1 = T2 IMU  */
} ImpactRaw_t;

/* T3 → T4 */
typedef struct {
    uint32_t timestamp_ms;
    uint8_t  region_id;         /* 0=centro, 1-4=bordas, 5-8=cantos */
    float    peak_accel_g;
    float    angular_vel_dps;
    float    angle_deg;
    uint16_t adc_peak[N_PIEZOS];
} ImpactPacket_t;

/* T3 → T5 */
typedef struct {
    uint8_t  region_id;
    float    intensity;         /* 0.0 – 1.0 normalizado */
    uint32_t timestamp_ms;
} RegionHit_t;

/* -----------------------------------------------------------------------------
 * Objetos FreeRTOS (globais)
 * ----------------------------------------------------------------------------- */
SemaphoreHandle_t semBinPiezo;
SemaphoreHandle_t semBinIMU;
QueueHandle_t     xQueueImpacto;
QueueHandle_t     xQueueTX;
QueueHandle_t     xQueueDisplay;

/* -----------------------------------------------------------------------------
 * Buffers de hardware
 * ----------------------------------------------------------------------------- */
static uint16_t adc_dma_buffer[N_PIEZOS];  /* preenchido pelo DMA */

/* -----------------------------------------------------------------------------
 * Estado do filtro de Kalman (simplificado — 1 eixo por variável)
 * ----------------------------------------------------------------------------- */
typedef struct {
    float angle;
    float bias;
    float P[2][2];
} KalmanState_t;

static KalmanState_t kalman = {
    .angle = 0.0f,
    .bias  = 0.0f,
    .P     = {{1.0f, 0.0f}, {0.0f, 1.0f}}
};

/* snapshot do ângulo mais recente — lido por T1 */
static volatile float imu_angle_snapshot = 0.0f;

/* -----------------------------------------------------------------------------
 * Protótipos internos
 * ----------------------------------------------------------------------------- */
static float    kalman_update(float accel_angle, float gyro_rate, float dt);
static uint8_t  estimar_regiao(uint16_t adc_ch[N_PIEZOS]);
static float    calcular_pico_g(int16_t accel[3]);
static float    calcular_vel_angular(int16_t gyro[3]);
static void     uart_send_frame(ImpactPacket_t *pkt);
static void     imu_read_raw(int16_t accel[3], int16_t gyro[3]);

/* =============================================================================
 * ISRs — apenas sinalizam; nenhum processamento aqui
 * ============================================================================= */

/* Fim de conversão ADC via DMA */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
    if (hadc->Instance != ADC1) return;

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(semBinPiezo, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/* Fim de leitura I2C do IMU */
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c->Instance != I2C1) return;

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(semBinIMU, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/* =============================================================================
 * T1 — Aquisição de piezos (prioridade 4)
 * ============================================================================= */
void vTaskAquisicaoPiezos(void *pvParameters) {
    ImpactRaw_t raw;
    memset(&raw, 0, sizeof(raw));

    /* inicia a primeira conversão DMA contínua */
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_dma_buffer, N_PIEZOS);

    for (;;) {
        /* bloqueia até ISR do DMA sinalizar fim de conversão */
        xSemaphoreTake(semBinPiezo, portMAX_DELAY);

        raw.timestamp_ms = xTaskGetTickCount();
        raw.source       = 0;

        /* copia buffer DMA para a struct — acesso atômico por cópia */
        for (int i = 0; i < N_PIEZOS; i++) {
            raw.adc_ch[i] = adc_dma_buffer[i];
        }

        /* snapshot do ângulo calculado por T2 */
        raw.angle_deg = imu_angle_snapshot;

        /* envia para T3 sem bloquear — se fila cheia, descarta leitura */
        xQueueSend(xQueueImpacto, &raw, 0);
    }
}

/* =============================================================================
 * T2 — Fusão inercial / IMU (prioridade 4)
 * ============================================================================= */
void vTaskFusaoIMU(void *pvParameters) {
    ImpactRaw_t raw;
    memset(&raw, 0, sizeof(raw));

    static int16_t accel[3], gyro[3];
    static uint8_t imu_rx_buf[14]; /* buffer I2C para leitura do IMU */

    /* dispara primeira leitura I2C não-bloqueante */
    HAL_I2C_Master_Receive_IT(&hi2c1, 0x28 << 1, imu_rx_buf, 14);

    for (;;) {
        /* bloqueia até ISR do I2C sinalizar fim de leitura */
        xSemaphoreTake(semBinIMU, portMAX_DELAY);

        raw.timestamp_ms = xTaskGetTickCount();
        raw.source       = 1;

        /* decodifica buffer I2C (formato BNO085 simplificado) */
        accel[0] = (int16_t)((imu_rx_buf[1]  << 8) | imu_rx_buf[0]);
        accel[1] = (int16_t)((imu_rx_buf[3]  << 8) | imu_rx_buf[2]);
        accel[2] = (int16_t)((imu_rx_buf[5]  << 8) | imu_rx_buf[4]);
        gyro[0]  = (int16_t)((imu_rx_buf[7]  << 8) | imu_rx_buf[6]);
        gyro[1]  = (int16_t)((imu_rx_buf[9]  << 8) | imu_rx_buf[8]);
        gyro[2]  = (int16_t)((imu_rx_buf[11] << 8) | imu_rx_buf[10]);

        memcpy(raw.accel_xyz, accel, sizeof(accel));
        memcpy(raw.gyro_xyz,  gyro,  sizeof(gyro));

        /* filtro de Kalman — dt = 5 ms (período de T2) */
        float accel_angle = atan2f((float)accel[0], (float)accel[2]) * 57.2958f;
        float gyro_rate   = (float)gyro[1] / 131.0f; /* LSB/(°/s) para MPU-6050 */
        raw.angle_deg     = kalman_update(accel_angle, gyro_rate, 0.005f);

        /* atualiza snapshot acessado por T1 */
        imu_angle_snapshot = raw.angle_deg;

        xQueueSend(xQueueImpacto, &raw, 0);

        /* dispara próxima leitura I2C não-bloqueante */
        HAL_I2C_Master_Receive_IT(&hi2c1, 0x28 << 1, imu_rx_buf, 14);
    }
}

/* =============================================================================
 * T3 — Processamento de impacto (prioridade 3)
 * ============================================================================= */
void vTaskProcessamento(void *pvParameters) {
    ImpactRaw_t    raw;
    ImpactPacket_t pkt;
    RegionHit_t    hit;

    for (;;) {
        /* bloqueia até T1 ou T2 depositarem dado */
        xQueueReceive(xQueueImpacto, &raw, portMAX_DELAY);

        /* monta pacote processado */
        pkt.timestamp_ms    = raw.timestamp_ms;
        pkt.region_id       = estimar_regiao(raw.adc_ch);
        pkt.peak_accel_g    = calcular_pico_g(raw.accel_xyz);
        pkt.angular_vel_dps = calcular_vel_angular(raw.gyro_xyz);
        pkt.angle_deg       = raw.angle_deg;

        for (int i = 0; i < N_PIEZOS; i++) {
            pkt.adc_peak[i] = raw.adc_ch[i];
        }

        /* distribui para T4 (transmissão) e T5 (display) */
        xQueueSend(xQueueTX, &pkt, 0);

        hit.region_id    = pkt.region_id;
        hit.intensity    = pkt.peak_accel_g / ACCEL_MAX_G;
        if (hit.intensity > 1.0f) hit.intensity = 1.0f;
        hit.timestamp_ms = pkt.timestamp_ms;

        /* xQueueOverwrite: nunca bloqueia — sempre entrega o mais recente */
        xQueueOverwrite(xQueueDisplay, &hit);
    }
}

/* =============================================================================
 * T4 — Transmissão UART / USB-Serial / HC-12 (prioridade 2)
 * ============================================================================= */
void vTaskTransmissao(void *pvParameters) {
    ImpactPacket_t pkt;

    for (;;) {
        xQueueReceive(xQueueTX, &pkt, portMAX_DELAY);
        uart_send_frame(&pkt);
    }
}

/* =============================================================================
 * T5 — Display heatmap (prioridade 1)
 * ============================================================================= */
void vTaskDisplay(void *pvParameters) {
    RegionHit_t hit;
    uint32_t    hit_count[N_REGIOES] = {0};
    uint32_t    max_hits = 1;
    TickType_t  xLastWakeTime = xTaskGetTickCount();

    for (;;) {
        /* drena todos os itens pendentes sem bloquear */
        while (xQueueReceive(xQueueDisplay, &hit, 0) == pdTRUE) {
            if (hit.region_id < N_REGIOES) {
                hit_count[hit.region_id]++;
                if (hit_count[hit.region_id] > max_hits)
                    max_hits = hit_count[hit.region_id];
            }
        }

        /* redesenha heatmap — substitua pelas funções do seu display */
        display_clear();
        for (int r = 0; r < N_REGIOES; r++) {
            float intensidade = (float)hit_count[r] / (float)max_hits;
            display_draw_regiao(r, intensidade);
        }
        display_flush();

        /* período fixo de 50 ms — vTaskDelayUntil garante período absoluto */
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(50));
    }
}

/* =============================================================================
 * Idle hook — low power entre impactos
 * ============================================================================= */
void vApplicationIdleHook(void) {
    __WFI();
}

/* =============================================================================
 * main()
 * ============================================================================= */
int main(void) {
    /* inicialização HAL e periféricos (gerado pelo CubeMX) */
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_ADC1_Init();
    MX_I2C1_Init();
    MX_USART1_UART_Init();
    display_init();

    /* -------------------------------------------------------------------------
     * Criação dos objetos FreeRTOS
     * ---------------------------------------------------------------------- */
    semBinPiezo   = xSemaphoreCreateBinary();
    semBinIMU     = xSemaphoreCreateBinary();
    xQueueImpacto = xQueueCreate(8, sizeof(ImpactRaw_t));
    xQueueTX      = xQueueCreate(4, sizeof(ImpactPacket_t));
    xQueueDisplay = xQueueCreate(2, sizeof(RegionHit_t));

    /* trava em debug se qualquer objeto falhar na criação */
    configASSERT(semBinPiezo   != NULL);
    configASSERT(semBinIMU     != NULL);
    configASSERT(xQueueImpacto != NULL);
    configASSERT(xQueueTX      != NULL);
    configASSERT(xQueueDisplay != NULL);

    /* -------------------------------------------------------------------------
     * Criação das tarefas
     * ---------------------------------------------------------------------- */
    xTaskCreate(vTaskAquisicaoPiezos, "T1_Piezo", 256, NULL, 4, NULL);
    xTaskCreate(vTaskFusaoIMU,        "T2_IMU",   256, NULL, 4, NULL);
    xTaskCreate(vTaskProcessamento,   "T3_Proc",  512, NULL, 3, NULL);
    xTaskCreate(vTaskTransmissao,     "T4_TX",    512, NULL, 2, NULL);
    xTaskCreate(vTaskDisplay,         "T5_Disp",  512, NULL, 1, NULL);

    /* inicia o escalonador — não retorna */
    vTaskStartScheduler();

    for (;;);
}

/* =============================================================================
 * Funções auxiliares
 * ============================================================================= */

/* Filtro de Kalman — 1 eixo */
static float kalman_update(float accel_angle, float gyro_rate, float dt) {
    /* predição */
    kalman.angle += dt * (gyro_rate - kalman.bias);
    kalman.P[0][0] += dt * (dt * kalman.P[1][1] - kalman.P[0][1] - kalman.P[1][0] + 0.001f);
    kalman.P[0][1] -= dt * kalman.P[1][1];
    kalman.P[1][0] -= dt * kalman.P[1][1];
    kalman.P[1][1] += 0.003f * dt;

    /* atualização */
    float S = kalman.P[0][0] + 0.03f;
    float K0 = kalman.P[0][0] / S;
    float K1 = kalman.P[1][0] / S;
    float y  = accel_angle - kalman.angle;

    kalman.angle += K0 * y;
    kalman.bias  += K1 * y;
    kalman.P[0][0] -= K0 * kalman.P[0][0];
    kalman.P[0][1] -= K0 * kalman.P[0][1];
    kalman.P[1][0] -= K1 * kalman.P[0][0];
    kalman.P[1][1] -= K1 * kalman.P[0][1];

    return kalman.angle;
}

/* Estima a região pelo canal ADC de maior amplitude */
static uint8_t estimar_regiao(uint16_t adc_ch[N_PIEZOS]) {
    /*
     * Mapeamento de canais para regiões — calibrar experimentalmente:
     *
     *   Canal 0 → região 0 (centro)
     *   Canal 1 → região 1 (borda superior)
     *   Canal 2 → região 2 (borda direita)
     *   Canal 3 → região 3 (borda inferior)
     *   Canal 4 → região 4 (borda esquerda)
     *   Canal 5 → região 5 (canto)
     *
     * Ajuste a tabela após calibração com impactos em posições conhecidas.
     */
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

/* Pico de aceleração em g (norma do vetor 3D) */
static float calcular_pico_g(int16_t accel[3]) {
    /* escala: 2048 LSB/g para fundo de escala ±16g (ajuste conforme IMU) */
    float ax = accel[0] / 2048.0f;
    float ay = accel[1] / 2048.0f;
    float az = accel[2] / 2048.0f;
    return sqrtf(ax*ax + ay*ay + az*az);
}

/* Velocidade angular em °/s (norma do vetor giroscópio) */
static float calcular_vel_angular(int16_t gyro[3]) {
    /* escala: 131 LSB/(°/s) para fundo de escala ±250°/s */
    float gx = gyro[0] / 131.0f;
    float gy = gyro[1] / 131.0f;
    float gz = gyro[2] / 131.0f;
    return sqrtf(gx*gx + gy*gy + gz*gz);
}

/* Monta e envia frame binário pela UART
 *
 * Protocolo:
 * [ 0xAA | len(1) | timestamp(4) | region(1) | accel(4) | ang_vel(4) |
 *   angle(4) | adc[6](12) | 0x55 ]
 * Total: 32 bytes
 */
static void uart_send_frame(ImpactPacket_t *pkt) {
    uint8_t frame[FRAME_SIZE];
    uint8_t *p = frame;

    *p++ = FRAME_START;
    *p++ = FRAME_SIZE - 2;              /* payload sem start/end */

    memcpy(p, &pkt->timestamp_ms,    4); p += 4;
    *p++ = pkt->region_id;
    memcpy(p, &pkt->peak_accel_g,    4); p += 4;
    memcpy(p, &pkt->angular_vel_dps, 4); p += 4;
    memcpy(p, &pkt->angle_deg,       4); p += 4;
    memcpy(p, pkt->adc_peak, N_PIEZOS * sizeof(uint16_t)); p += 12;

    *p = FRAME_END;

    HAL_UART_Transmit(&huart1, frame, FRAME_SIZE, HAL_MAX_DELAY);
}
