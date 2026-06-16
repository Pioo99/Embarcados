#pragma once
#include <stdint.h>

/* USART1 TX — PA9, 115200 8N1 — envio de frames para o PC */
void uart1_init(void);
void uart1_send(const uint8_t *buf, uint16_t len);
