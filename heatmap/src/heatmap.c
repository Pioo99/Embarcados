#include "heatmap.h"
#include "st7789.h"
#include "board.h"
#include <stdio.h>

/* ── Layout ─────────────────────────────────────────────────────────
   Grade 4×4, cada célula 46×46 px, pitch 50 px (gap de 4 px).
   Origem (22,22) → grid ocupa x=[22..217], y=[22..217].
   Borda externa: draw_rect(14, 14, 213, 213) → x=[14..226], y=[14..226].
   Título fora da borda (y=4), contador de hits (y=230).
   ─────────────────────────────────────────────────────────────────── */
#define GRID_X0    22
#define GRID_Y0    22
#define CELL_SIZE  46
#define CELL_PITCH 50

/* ── Dados internos ─────────────────────────────────────────────── */
static uint32_t g_heat[HEAT_NUM_CH];
static uint32_t g_total_hits;

/* ── Helpers de posição ─────────────────────────────────────────── */
static uint16_t cell_x(uint8_t ch) { return (uint16_t)(GRID_X0 + (ch % 4) * CELL_PITCH); }
static uint16_t cell_y(uint8_t ch) { return (uint16_t)(GRID_Y0 + (ch / 4) * CELL_PITCH); }

/* ── Gradiente de cor (6 patamares) ─────────────────────────────── */
static uint16_t heat_color(uint32_t val, uint32_t max) {
    if (max == 0u || val == 0u) return C_BLACK;
    uint32_t pct = (val * 100u) / max;
    if (pct <= 20u) return C_BLUE;   /* 0x001F */
    if (pct <= 40u) return C_CYAN;   /* 0x07FF */
    if (pct <= 60u) return C_GREEN;  /* 0x07E0 */
    if (pct <= 80u) return C_YELL;   /* 0xFFE0 */
    return C_RED;                     /* 0xF800 */
}

/* ── Elipse (Bresenham inteiro, sem float) ──────────────────────── */
static void draw_ellipse(int cx, int cy, int rx, int ry, uint16_t color) {
    long a2 = (long)rx*rx, b2 = (long)ry*ry;
    long fa2 = 4*a2, fb2 = 4*b2;
    long x, y, sigma;

    /* Região 1: slope |m| < 1 */
    for (x = 0, y = ry, sigma = 2*b2 + a2*(1 - 2*ry);
         b2*x <= a2*y; x++) {
        st7789_draw_pixel((uint16_t)(cx + x), (uint16_t)(cy + y), color);
        st7789_draw_pixel((uint16_t)(cx - x), (uint16_t)(cy + y), color);
        st7789_draw_pixel((uint16_t)(cx + x), (uint16_t)(cy - y), color);
        st7789_draw_pixel((uint16_t)(cx - x), (uint16_t)(cy - y), color);
        if (sigma >= 0) { sigma += fa2 * (1 - y); y--; }
        sigma += b2 * (4*x + 6);
    }

    /* Região 2: slope |m| > 1 */
    for (x = rx, y = 0, sigma = 2*a2 + b2*(1 - 2*rx);
         a2*y <= b2*x; y++) {
        st7789_draw_pixel((uint16_t)(cx + x), (uint16_t)(cy + y), color);
        st7789_draw_pixel((uint16_t)(cx - x), (uint16_t)(cy + y), color);
        st7789_draw_pixel((uint16_t)(cx + x), (uint16_t)(cy - y), color);
        st7789_draw_pixel((uint16_t)(cx - x), (uint16_t)(cy - y), color);
        if (sigma >= 0) { sigma += fb2 * (1 - x); x--; }
        sigma += a2 * (4*y + 6);
    }
}

/* ── API pública ─────────────────────────────────────────────────── */

void heat_init(void) {
    for (int i = 0; i < HEAT_NUM_CH; i++) g_heat[i] = 0;
    g_total_hits = 0;

    st7789_fill_screen(C_BLACK);

    /* título acima da borda */
    st7789_draw_text_5x7(75, 4, "RAQUETE HEATMAP", C_WHITE, 1, 0, C_BLACK);

    /* borda da raquete */
    st7789_draw_rect(14, 14, 213, 213, C_WHITE);
    st7789_draw_rect(15, 15, 211, 211, C_WHITE);   /* 2 px de espessura */

    /* elipse decorativa interna — centro (120,120), semi-eixos 99×99 */
    draw_ellipse(120, 120, 99, 99, 0x4208 /* cinza escuro */);

    /* células iniciais (todas pretas) */
    for (uint8_t i = 0; i < HEAT_NUM_CH; i++) {
        st7789_fill_rect(cell_x(i), cell_y(i), CELL_SIZE, CELL_SIZE, C_BLACK);
    }

    /* contador inicial */
    st7789_draw_text_5x7(76, 230, "Hits: 0        ", C_WHITE, 1, 0, C_BLACK);
}

void heat_add(uint8_t ch, uint32_t energy) {
    if (ch >= HEAT_NUM_CH) return;
    g_heat[ch] += energy;
    g_total_hits++;
}

void heat_render(void) {
    /* normaliza pelo canal mais quente */
    uint32_t max_heat = 0;
    for (int i = 0; i < HEAT_NUM_CH; i++) {
        if (g_heat[i] > max_heat) max_heat = g_heat[i];
    }

    /* redesenha células via DMA */
    for (uint8_t i = 0; i < HEAT_NUM_CH; i++) {
        uint16_t color = heat_color(g_heat[i], max_heat);
        st7789_fill_rect_dma(cell_x(i), cell_y(i), CELL_SIZE, CELL_SIZE, color);
    }

    /* atualiza contador */
    char buf[24];
    snprintf(buf, sizeof(buf), "Hits: %u        ", (unsigned)g_total_hits);
    st7789_draw_text_5x7(76, 230, buf, C_WHITE, 1, 1, C_BLACK);
}
