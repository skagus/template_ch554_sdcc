// Blink an LED connected to pin 1.7

#include <stdint.h>
#include <string.h>

#include "ch554.h"
#include "macro.h"
#include "debug.h"
#include "tick.h"
#include "color_led.h"
#include "key.h"

void Timer0_Interrupt(void) ATTR_IRQ(INT_NO_TMR0)
{
	TICK_ISR();
}

void Timer1_Interrupt(void) ATTR_IRQ(INT_NO_TMR1)
{
	TICK_ISR();
}

void Timer2_Interrupt(void) ATTR_IRQ(INT_NO_TMR2)
{
	TICK_ISR();
}

void GPIO_Interrupt(void) ATTR_IRQ(INT_NO_GPIO)
{
	KEY_ISR();
}


extern void Blink_Run();

void main()
{
	CfgFsys();
	TICK_Init();
	CLED_Init();
	KEY_Init();
	__enable_irq();

	while(1)
	{
		Blink_Run();
		CLED_Run();
		KEY_Run();
	}
}
