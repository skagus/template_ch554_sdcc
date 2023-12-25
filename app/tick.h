#pragma once
#include <stdint.h>

#define MSEC_PER_TICK	(10)	// 10 msec.

void TICK_Init(void);
uint16_t TICK_GetTick(void);
void TICK_Delay(uint16_t delay);
void TICK_ISR(void);
