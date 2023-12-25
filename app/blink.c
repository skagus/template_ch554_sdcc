#include "ch554.h"
#include "tick.h"

#define LED_PIN 0
SBIT(LED, 0xB0, LED_PIN);

uint16_t __xdata gnPrv = 0;
void task_LedBlink()
{
	uint16_t nCur = TICK_GetTick();
	if (nCur - gnPrv >= 100)
	{
		LED = !LED;
		gnPrv = nCur;
	}
}
