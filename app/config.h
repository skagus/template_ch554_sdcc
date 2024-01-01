#include "gpio.h"
#include "tick.h"

#define PIN_KEY1			P16			// pin connected to key 1
#define PIN_KEY2			P17			// pin connected to key 2
#define PIN_KEY3			P11			// pin connected to key 3
#define PIN_ENC_SW			P33			// pin connected to knob switch

#define PIN_ENC_A 			P31			// pin connected to knob outA
#define PIN_ENC_B 			P30			// pin connected to knob outB

#define PIN_NEO				P34			// pin connected to NeoPixel

#define TIME_FOR_LONG		TICK_MSEC(1000)	// Long press 감지 시간.
#define TIME_FOR_REPEAT		TICK_MSEC(500)	// Long press 상태에서, 반복 주기.

