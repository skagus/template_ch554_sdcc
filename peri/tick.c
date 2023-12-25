#include <stdint.h>
#include <string.h>
#include "ch554.h"

#include "tick.h"

#define CLOCK_PER_MSEC		((FREQ_SYS / 1000) * (MSEC_PER_TICK))
#define CNT_TMR				(65536 - (CLOCK_PER_MSEC / 12))  // bTMR_CLK

#define TIMER_ID_4TICK		(0)

// Tick count. 
uint16_t __xdata gnTick;
uint16_t __xdata gnTick;

void TICK_Init(void)
{
	gnTick = 0;

#if (TIMER_ID_4TICK == 0)
	uint16_t __xdata nCntVal = CNT_TMR;
	TMOD |= bT0_M0; // 16 bit.
	TL0 = nCntVal & 0xff;
	TH0 = (nCntVal >> 8) & 0xff;
	TR0 = 1;
	ET0 = 1;
#elif (TIMER_ID_4TICK == 1)
	uint16_t __xdata nCntVal = CNT_TMR;
	TMOD |= bT1_M0;	// 16 bit.
	RCLK = 1;		// Timer 2 for UART0 RX clock
	TCLK = 1;		// Timer 3 for UART0 TX clock
	TL1 = nCntVal & 0xff;
	TH1 = (nCntVal >> 8) & 0xff;
	TR1 = 1;
	ET1 = 1;
#elif (TIMER_ID_4TICK == 2)
	uint16_t __xdata nCntVal = CNT_TMR;
	C_T2 = 0;		// Internal clock as Timer2 clock source
	RCLK = 0;		// Timer 1 for UART0 RX clock (or Timer2)
	TCLK = 0;		// Timer 1 for UART0 TX clock (or Timer2)
	CP_RL2 = 0;		// Timer2 auto reload.
	T2MOD |= bTMR_CLK;   // FSYS/4,
	RCAP2L = TL2 = nCntVal & 0xff; //16位自动重载定时器
	RCAP2H = TH2 = (nCntVal >> 8) & 0xff;
	TR2 = 1;
	ET2 = 1;
#else
	#err Select Timer;
#endif

}

static void _IncTick(void)
{
	gnTick = gnTick + 1;
}

uint16_t TICK_GetTick(void)
{
	return gnTick;
}

void TICK_Delay(uint16_t delay)
{
	uint16_t __xdata tickstart = TICK_GetTick();
	uint16_t __xdata wait = delay;

	/* Add a freq to guarantee minimum wait */
	if (wait < 65535)
	{
		wait += 1;
	}

	while ((TICK_GetTick() - tickstart) < wait)
	{
	}
}

void TICK_ISR(void)
{
	gnTick++;
#if (TIMER_ID_4TICK == 0)
	TL0 = CNT_TMR & 0xff;
	TH0 = (CNT_TMR >> 8) & 0xff;
	TF0 = 0;
#elif (TIMER_ID_4TICK == 1)	
	TL1 = nCntVal & 0xff;
	TH1 = (nCntVal >> 8) & 0xff;
	TF1 = 0;
#elif (TIMER_ID_4TICK == 2)
	TF2 = 0;
#endif
}
