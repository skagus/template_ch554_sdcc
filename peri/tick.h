#pragma once
#include <stdint.h>

#define MSEC_PER_TICK		(10)	// 10 msec.
#define TICK_MSEC(x)		((x) / MSEC_PER_TICK)
#define TICK_SEC(x)			((x) * 1000 / MSEC_PER_TICK)
void TICK_Init(void);
uint16_t TICK_GetTick(void);
void TICK_Delay(uint16_t delay);
void TICK_ISR(void);
