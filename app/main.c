// Blink an LED connected to pin 1.7

#include <stdint.h>
#include <string.h>

#include "ch554.h"
#include "debug.h"
#include "tick.h"

void Timer0_Interrupt(void) __interrupt(INT_NO_TMR0)
{
	TICK_ISR();
}

void Timer1_Interrupt(void) __interrupt(INT_NO_TMR1)
{
	TICK_ISR();
}

void Timer2_Interrupt(void) __interrupt(INT_NO_TMR2)
{
	TICK_ISR();
}


extern void task_LedBlink();

void main()
{
	CfgFsys();
	TICK_Init();

	EA = 1;

	while (1)
	{
		task_LedBlink();
	}
}
