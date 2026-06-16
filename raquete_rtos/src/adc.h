#pragma once
#include <stdint.h>
#include "board.h"

/* ── ADC1 + DMA2 Stream0 — 6 piezos em scan contínuo ─────────────
   adc_dma_init/start iniciam conversão livre (CONT) que preenche um
   buffer interno por DMA circular, sem interrupção. T1 lê o último
   quadro de forma periódica com adc_snapshot.
   ────────────────────────────────────────────────────────────── */
void adc_dma_init(void);
void adc_dma_start(void);
void adc_snapshot(uint16_t dst[N_PIEZOS]);
