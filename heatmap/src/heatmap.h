#pragma once
#include <stdint.h>

#define HEAT_NUM_CH 16

void heat_init(void);
void heat_add(uint8_t ch, uint32_t energy);
void heat_render(void);
