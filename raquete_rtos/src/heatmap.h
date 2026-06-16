#pragma once
#include <stdint.h>

/* ── Heatmap 3x3 das 9 regiões da raquete no ST7789 ─────────────
   0=centro, 1-4=bordas, 5-8=cantos. Layout no grid espelha o
   pc/raquete_receiver.py (REGION_GRID).
   ────────────────────────────────────────────────────────────── */
void heat_init(void);                                   /* limpa + bordas */
void heat_draw_region(uint8_t region, float intensity); /* 0.0–1.0 */
