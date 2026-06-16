/*
 * Heatmap 3x3 das 9 regiões da raquete (ST7789 240x240).
 * Cada região é uma célula de 76x76 px; a cor varia do azul (frio)
 * ao vermelho (quente) conforme a intensidade acumulada de impactos.
 */
#include "heatmap.h"
#include "st7789.h"
#include "board.h"

#define CELL   80u    /* passo da grade (76 px de célula + 4 de borda) */
#define PAD    2u

/* região → (linha, coluna) no grid 3x3 — igual ao receiver Python */
static const uint8_t region_grid[N_REGIOES][2] = {
    {1, 1},  /* 0 centro        */
    {0, 1},  /* 1 borda sup     */
    {1, 2},  /* 2 borda dir     */
    {2, 1},  /* 3 borda inf     */
    {1, 0},  /* 4 borda esq     */
    {0, 2},  /* 5 canto sup-dir */
    {0, 0},  /* 6 canto sup-esq */
    {2, 2},  /* 7 canto inf-dir */
    {2, 0},  /* 8 canto inf-esq */
};

static uint16_t heat_color(float t) {
    if (t < 0.01f) return C_BLACK;
    if (t < 0.25f) return C_BLUE;
    if (t < 0.50f) return C_GREEN;
    if (t < 0.70f) return C_YELL;
    if (t < 0.85f) return C_ORANGE;
    return C_RED;
}

void heat_init(void) {
    st7789_fill_screen(C_BLACK);
    for (int r = 0; r < N_REGIOES; r++) {
        uint16_t x = region_grid[r][1] * CELL + PAD;
        uint16_t y = region_grid[r][0] * CELL + PAD;
        st7789_draw_rect(x, y, CELL - 2 * PAD, CELL - 2 * PAD, C_GRAY);
    }
}

void heat_draw_region(uint8_t region, float intensity) {
    if (region >= N_REGIOES) return;
    uint16_t x = region_grid[region][1] * CELL + PAD + 1;
    uint16_t y = region_grid[region][0] * CELL + PAD + 1;
    st7789_fill_rect(x, y, CELL - 2 * PAD - 2, CELL - 2 * PAD - 2,
                     heat_color(intensity));
}
