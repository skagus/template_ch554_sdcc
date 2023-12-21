#include <stdint.h>
#include <string.h>
#include "ch554.h"

#include "time.h"

uint16_t __xdata gnTick;

void time_MsTick_init(void)
{
	gnTick = 0;
	T2MOD |= (bTMR_CLK | bT2_CLK);
	C_T2 = 0;
	RCLK = 0;
	TCLK = 0;
	CP_RL2 = 0;
	//mTimer_x_ModInit(2, 1);
	mTimer_x_SetData(2, 24000); // 1ms
	TR2 = 1;
	ET2 = 1;
}

static void time_MsTick_Inc(void)
{
	gnTick = gnTick + 1;
}

uint16_t time_MsTick_Get(void)
{
	return gnTick;
}

void time_MsTick_Delay(uint16_t delay)
{
	uint16_t __xdata tickstart = time_MsTick_Get();
	uint16_t __xdata wait = delay;

	/* Add a freq to guarantee minimum wait */
	if (wait < 65535)
	{
		wait += 1;
	}

	while ((time_MsTick_Get() - tickstart) < wait)
	{
	}
}

void time_MsTickInterrupt(void)
{
	gnTick++;
	TF2 = 0;
}

/*******************************************************************************
* Description: CH554 타이머 카운터x모드 설정
* Input: UINT8 mode,Timer 모드 선택
0: 모드 0, 13비트 타이머, TLn의 높은 3비트 무효
1: 모드 1, 16비트 타이머
2: 모드 2, 8비트 자동 재설치 타이머
3: 모드3, 8비트 타이머 Timer0
3: 모드3, Timer1 정지
* * Output : None
* * Return : 0:SUCCESS / 1:FAIL
*******************************************************************************/
uint8_t mTimer_x_ModInit(uint8_t timer_id, uint8_t mode)
{
	if (timer_id == 0)
	{
		TMOD = TMOD & 0xf0 | mode;
	}
	else if (timer_id == 1)
	{
		TMOD = TMOD & 0x0f | (mode << 4);
	}
	else if (timer_id == 2)
	{
		RCLK = 0;
		TCLK = 0;
		CP_RL2 = 0;
	} //16位自动重载定时器
	else
		return 1;
	return 0;
}

/*******************************************************************************
* Function Name  : mTimer_x_SetData(UINT8 x,UINT16 dat)
* Description    : CH554Timer0 TH0和TL0赋值
* Input          : UINT16 dat; 타이머 할당.
*******************************************************************************/
void mTimer_x_SetData(uint8_t timer_id, uint16_t tick_count)
{
	uint16_t __xdata tmp;
	tmp = 65536 - tick_count;
	if (timer_id == 0)
	{
		TL0 = tmp & 0xff;
		TH0 = (tmp >> 8) & 0xff;
	}
	else if (timer_id == 1)
	{
		TL1 = tmp & 0xff;
		TH1 = (tmp >> 8) & 0xff;
	}
	else if (timer_id == 2)
	{
		RCAP2L = TL2 = tmp & 0xff; //16位自动重载定时器
		RCAP2H = TH2 = (tmp >> 8) & 0xff;
	}
}
