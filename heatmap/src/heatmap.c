#include "heatmap.h"
#include "st7789.h"
#include "board.h"
#include <stdio.h>

/* ── Layout: grade 2×2, células 88×88 px, pitch 96 px (gap 8 px) ──
   Origem (28,25) → grid x=[28..211], y=[25..208].
   Borda: draw_rect(20,17,200,200). Título y=4, hits y=222.
   ─────────────────────────────────────────────────────────────────── */
#define GRID_X0    28
#define GRID_Y0    25
#define CELL_SIZE  88
#define CELL_PITCH 96

/* ── Acumuladores ───────────────────────────────────────────────── */
static uint32_t g_heat[HEAT_NUM_CH];
static uint32_t g_total_hits;

/* ── Helpers de posição ─────────────────────────────────────────── */
static uint16_t cell_x(uint8_t ch) { return (uint16_t)(GRID_X0 + (ch % 2) * CELL_PITCH); }
static uint16_t cell_y(uint8_t ch) { return (uint16_t)(GRID_Y0 + (ch / 2) * CELL_PITCH); }

/* ── Gradiente relativo ao canal mais quente ─────────────────────── */
static uint16_t heat_color(uint32_t val, uint32_t max) {
    if (max == 0u || val == 0u) return C_BLACK;
    uint32_t pct = (val * 100u) / max;
    if (pct <= 20u) return C_BLUE;
    if (pct <= 40u) return C_CYAN;
    if (pct <= 60u) return C_GREEN;
    if (pct <= 80u) return C_YELL;
    return C_RED;
}

/* ── Elipse decorativa (Bresenham) ──────────────────────────────── */
static void draw_ellipse(int cx, int cy, int rx, int ry, uint16_t color) {
    long a2 = (long)rx*rx, b2 = (long)ry*ry;
    long fa2 = 4*a2, fb2 = 4*b2;
    long x, y, sigma;

    for (x = 0, y = ry, sigma = 2*b2 + a2*(1 - 2*ry);
         b2*x <= a2*y; x++) {
        st7789_draw_pixel((uint16_t)(cx+x),(uint16_t)(cy+y),color);
        st7789_draw_pixel((uint16_t)(cx-x),(uint16_t)(cy+y),color);
        st7789_draw_pixel((uint16_t)(cx+x),(uint16_t)(cy-y),color);
        st7789_draw_pixel((uint16_t)(cx-x),(uint16_t)(cy-y),color);
        if (sigma >= 0) { sigma += fa2*(1-y); y--; }
        sigma += b2*(4*x+6);
    }
    for (x = rx, y = 0, sigma = 2*a2 + b2*(1 - 2*rx);
         a2*y <= b2*x; y++) {
        st7789_draw_pixel((uint16_t)(cx+x),(uint16_t)(cy+y),color);
        st7789_draw_pixel((uint16_t)(cx-x),(uint16_t)(cy+y),color);
        st7789_draw_pixel((uint16_t)(cx+x),(uint16_t)(cy-y),color);
        st7789_draw_pixel((uint16_t)(cx-x),(uint16_t)(cy-y),color);
        if (sigma >= 0) { sigma += fb2*(1-x); x--; }
        sigma += a2*(4*y+6);
    }
}

/* ── Label "C0"–"C3" centralizado na célula (scale 2) ──────────── */
static void draw_cell_label(uint8_t ch) {
    char label[3] = {'C', '0' + ch, '\0'};
    /* scale 2: 2 chars × 6px × 2 = 24px wide, 7px × 2 = 14px tall */
    int lx = (int)cell_x(ch) + (CELL_SIZE - 24) / 2;
    int ly = (int)cell_y(ch) + (CELL_SIZE - 14) / 2;
    st7789_draw_text_5x7(lx, ly, label, C_WHITE, 2, 1, C_BLACK);
}

/* ── API pública ─────────────────────────────────────────────────── */

void heat_init(void) {
    for (int i = 0; i < HEAT_NUM_CH; i++) g_heat[i] = 0;
    g_total_hits = 0;

    st7789_fill_screen(C_BLACK);
    st7789_draw_text_5x7(75, 4, "RAQUETE HEATMAP", C_WHITE, 1, 0, C_BLACK);
    st7789_draw_rect(20, 17, 200, 200, C_WHITE);
    st7789_draw_rect(21, 18, 198, 198, C_WHITE);
    draw_ellipse(120, 117, 95, 95, 0x4208);

    for (uint8_t i = 0; i < HEAT_NUM_CH; i++) {
        st7789_fill_rect(cell_x(i), cell_y(i), CELL_SIZE, CELL_SIZE, C_BLACK);
        draw_cell_label(i);
    }

    st7789_draw_text_5x7(76, 222, "Hits: 0        ", C_WHITE, 1, 0, C_BLACK);
}

void heat_add(uint8_t ch, uint32_t energy) {
    if (ch >= HEAT_NUM_CH) return;
    g_heat[ch] += energy;
    g_total_hits++;
}

void heat_render(void) {
    uint32_t max_heat = 0;
    for (int i = 0; i < HEAT_NUM_CH; i++) {
        if (g_heat[i] > max_heat) max_heat = g_heat[i];
    }

    for (uint8_t i = 0; i < HEAT_NUM_CH; i++) {
        uint16_t color = heat_color(g_heat[i], max_heat);
        st7789_fill_rect_dma(cell_x(i), cell_y(i), CELL_SIZE, CELL_SIZE, color);
        draw_cell_label(i);
    }

    char buf[24];
    snprintf(buf, sizeof(buf), "Hits: %u        ", (unsigned)g_total_hits);
    st7789_draw_text_5x7(76, 222, buf, C_WHITE, 1, 1, C_BLACK);
}
