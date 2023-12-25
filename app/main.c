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

extern void LED_Init();
extern void LED_Run();
extern void Blink_Run();

void main()
{
	CfgFsys();
	TICK_Init();

	LED_Init();
	EA = 1;

	while (1)
	{
		Blink_Run();
		LED_Run();
	}
}
