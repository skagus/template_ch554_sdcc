

#include "config.h"
#include "macro.h"
#include "gpio.h"
#include "color_led.h"
#include "key.h"



/**
이 파일은 Key와 rotary를 관리한다.
MCU level에서 key와 rotary의 입력방식은 동일하기 때문에 입력방식을 동일하게 유지된다.
둘 모두 bouncing 문제가 있을 수 있기 때문에, bouncing 시간 동안은 유효한 입력으로 처리하지 않는다.

Key와 rotary 의 SW상의 처리는 서로 다르다.

1. Key 관리.
  Key는 short와 long 두가지 event를 생성한다.
  short event는, release시점에서 pressed 상태 지속시간이 threshold 미만인 경우 발생.
  long event는, pressed 상태로 threshold 이상 유지되는 경우 발생하며, 이후 release에서는 event발생이 없다.
  ___++++++++^___________________: short case.
  ___++++++++++++++^+++++++++____: long case.
  현재 상태는 각 key의 pressed/released 상태로 정의된다.
2. Rotary의 관리.
  rotary는 회전 방향에 따라서 inc event와 dec event가 발생된다.
  현재 상태는 A/B의 위상으로 정의된다.
3. Rotary 에 포함된 SW 의 경우,
  Short는 user가 사용할 수 있음.
  long은 장치 설정 mode로 들어가는 용도로 사용.
*/

#define NUM_ALL_IN						(6)
#define TIME_DEBOUNCE					(TICK_MSEC(10))
#define TIME_LONG_PRESS					(TICK_MSEC(1000))

typedef void (*GetPins) (uint8_t* aNewKey);


typedef enum _KeyState
{
	KEY_BOUNCE,
	KEY_RELEASED,
	KEY_PRESSED,
	KEY_AFTER_LONG,
	NUM_KEY_STATE,
} KeyState;

void KEY_ISR() {}

void _SendShortKey(uint8_t nKeyId)
{
	UNUSED(nKeyId);
}
void _SendLongKey(uint8_t nKeyId)
{
	UNUSED(nKeyId);
}
void _SendRotKey(int8_t nChange)
{
	UNUSED(nChange);
}

void KEY_Init()
{
}

void _GetRawPins(uint8_t* aNewKey)
{
	aNewKey[0] = PIN_read(PIN_KEY1);
	aNewKey[1] = PIN_read(PIN_KEY2);
	aNewKey[2] = PIN_read(PIN_KEY3);
	aNewKey[3] = PIN_read(PIN_ENC_SW);
	aNewKey[4] = PIN_read(PIN_ENC_A);
	aNewKey[5] = PIN_read(PIN_ENC_B);
}

void _GetPins(uint8_t* aRet)
{
	static uint8_t gaPrvKey[TOTAL_PIN_IN];
	static uint16_t gaPrvTime[TOTAL_PIN_IN];

	uint8_t aNewKey[TOTAL_PIN_IN];
	_GetRawPins(aNewKey);

	uint16_t nCur = TICK_GetTick();
	for(uint8_t nKey = 0; nKey < TOTAL_PIN_IN; nKey++)
	{
		aRet[nKey] = KEY_BOUNCE;
		if(aNewKey[nKey] == gaPrvKey[nKey])
		{
			if(GAP_BTWN_16(nCur,gaPrvTime[nKey]) >= TIME_DEBOUNCE)
			{
				aRet[nKey] = (aNewKey[nKey] ? KEY_PRESSED : KEY_RELEASED);
			}
		}
		else
		{
			gaPrvKey[nKey] = aNewKey[nKey];
			gaPrvTime[nKey] = nCur;
		}
	}
}

#define KEY_CHANGE(old_val, new_val)	((old_val << 4) | new_val)

void _RunKey(uint8_t* aCur)
{
	static uint8_t gaPrv[NUM_KEY_IN];
	static uint16_t gaChangeTime[NUM_KEY_IN];  ///< 이전 update된 시간.

	for(uint8_t nKey = 0; nKey < NUM_KEY_IN; nKey++)
	{
		uint16_t nCurTick = TICK_GetTick();
		uint8_t bmChange = KEY_CHANGE(gaPrv[nKey],aCur[nKey]);
		switch(bmChange)
		{
			case KEY_CHANGE(KEY_RELEASED,KEY_PRESSED):
			{
				gaPrv[nKey] = KEY_PRESSED;
				gaChangeTime[nKey] = nCurTick;
				break;
			}
			case KEY_CHANGE(KEY_PRESSED,KEY_PRESSED):
			{
				if(GAP_BTWN_16(nCurTick,gaChangeTime[nKey]) > TIME_FOR_LONG)
				{
					_SendLongKey(nKey);
					gaPrv[nKey] = KEY_AFTER_LONG;
				}
				break;
			}
			case KEY_CHANGE(KEY_PRESSED,KEY_RELEASED):
			{
				_SendShortKey(nKey);
				gaPrv[nKey] = KEY_RELEASED;
				gaChangeTime[nKey] = nCurTick;
				break;
			}
			case KEY_CHANGE(KEY_AFTER_LONG,KEY_PRESSED):
			{
				// Never change state.
				break;
			}
			case KEY_CHANGE(KEY_AFTER_LONG,KEY_RELEASED):
			case KEY_CHANGE(KEY_RELEASED,KEY_RELEASED):
			default:
			{
				if(KEY_BOUNCE != aCur[nKey])
				{
					gaPrv[nKey] = aCur[nKey];
				}
				break;
			}
		}
	}
}


typedef enum _EncIdx
{
	IDX_ENC_A,
	IDX_ENC_B,
	NUN_ENC_KEY,
} _EncIdx;

const int8_t gaRotTable[4][4] =
{
	//LL,LH,HL,HH <-A v-B
	{+0,+1,-1,+0,},// LL
	{-1,+0,+0,+1,},// LH
	{+1,+0,+0,-1,},// HL
	{+0,-1,+1,+0,},// HH
};

void _RunEncoder(uint8_t* aCur)
{
	static uint8_t gaPrv[NUN_ENC_KEY];
	static uint16_t gaPrvTime[NUN_ENC_KEY];  ///< 이전 update된 시간.

	if((KEY_BOUNCE != aCur[IDX_ENC_A]) && (KEY_BOUNCE != aCur[IDX_ENC_B]))
	{
		////// Rotate //////
		gaPrv[IDX_ENC_A] = (gaPrv[IDX_ENC_A] << 1) | ((KEY_RELEASED == aCur[IDX_ENC_A]) ? 1 : 0);
		gaPrv[IDX_ENC_B] = (gaPrv[IDX_ENC_B] << 1) | ((KEY_RELEASED == aCur[IDX_ENC_B]) ? 1 : 0);

		int8_t nRotChange = gaRotTable[gaPrv[IDX_ENC_A] & 0x3][gaPrv[IDX_ENC_B] & 0x3];
		if(0 != nRotChange)
		{
			_SendRotKey(nRotChange);
		}
	}
}

void KEY_Run()
{
	uint8_t aPins[TOTAL_PIN_IN];
	_GetPins(aPins);
	_RunKey(aPins);
	_RunEncoder(aPins + NUM_KEY_IN);
}
