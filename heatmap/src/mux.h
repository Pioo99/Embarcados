#pragma once
#include <stdint.h>

/* CD4067 16:1 analog mux
   S0=PA1  S1=PA2  S2=PA3  S3=PA4
   SIG=PA0 (ADC IN0) */

void mux_init(void);
void mux_select(uint8_t ch);   /* ch: 0-15 */
