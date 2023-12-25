#include "ch554.h"
#include "macro.h"
#include "tick.h"

#define LED_PIN 0
SBIT(LED, 0xB0, LED_PIN);

uint16_t __xdata gnPrv = 0;
void Blink_Run()
{
	uint16_t nCur = TICK_GetTick();
	if (GAP_BTWN_16(nCur, gnPrv) >= TICK_SEC(1))
	{
		LED = !LED;
		gnPrv = nCur;
	}
}

