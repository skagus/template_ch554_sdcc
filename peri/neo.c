// ===================================================================================
// NeoPixel (Addressable LED) Functions for CH551, CH552 and CH554            * v1.2 *
// ===================================================================================
//
// Basic control functions for 800kHz addressable LEDs (NeoPixel). A simplified 
// protocol is used which should work with most LEDs.
//
// The following must be defined in config.h:
// PIN_NEO   - pin connected to DATA-IN of the pixel strip (via a ~330 ohms resistor).
// NEO_GRB   - type of pixel: NEO_GRB or NEO_RGB
// NEO_COUNT - total number of pixels
// System clock frequency must be at least 6 MHz.
//
// Further information:     https://github.com/wagiminator/ATtiny13-NeoController
// 2023 by Stefan Wagner:   https://github.com/wagiminator

// ===================================================================================
// Libraries, Variables and Constants
// ===================================================================================
#include "neo.h"

#define NEOPIN PIN_asm(PIN_NEO)             // convert PIN_NEO for inline assembly
__xdata uint8_t NEO_buffer[3 * NEO_COUNT];  // pixel buffer

// ===================================================================================
// Protocol Delays
// ===================================================================================
// There are three essential conditions:
// - T0H (HIGH-time for "0"-bit) must be max.  500ns
// - T1H (HIGH-time for "1"-bit) must be min.  625ns
// - TCT (total clock time) must be      min. 1150ns
// The bit transmission loop takes 11 clock cycles.
#if FREQ_SYS == 24000000       // 24 MHz system clock
#define T1H_DELAY \
    nop             \
    nop             \
    nop             \
    nop             \
    nop             \
    nop             \
    nop             \
    nop             \
    nop             \
    nop             \
    nop                     // 15 - 4 = 11 clock cycles for min 625ns
#define TCT_DELAY \
    nop             \
    nop             \
    nop             \
    nop             \
    nop             \
    nop                     // 28 - 11 - 11 = 6 clock cycles for min 1150ns
#elif FREQ_SYS == 16000000     // 16 MHz system clock
#define T1H_DELAY \
    nop             \
    nop             \
    nop             \
    nop             \
    nop             \
    nop                     // 10 - 4 = 6 clock cycles for min 625ns
#define TCT_DELAY \
    nop             \
    nop                     // 19 - 6 - 11 = 2 clock cycles for min 1150ns
#elif FREQ_SYS == 12000000     // 12 MHz system clock
#define T1H_DELAY \
    nop             \
    nop             \
    nop             \
    nop                     // 8 - 4 = 4 clock cycles for min 625ns
#define TCT_DELAY         // 14 - 4 - 11 < 0 clock cycles for min 1150ns
#elif FREQ_SYS == 6000000      // 13 MHz system clock
#define T1H_DELAY         // 4 - 4 = 0 clock cycles for min 625ns
#define TCT_DELAY         // 7 - 0 - 11 < 0 clock cycles for min 1150ns
#else
#error Unsupported system clock frequency for NeoPixels!
#endif

// ===================================================================================
// Send a Data Byte to the Pixels String
// ===================================================================================
// This is the most time sensitive part. Outside of the function, it must be 
// ensured that interrupts are disabled and that the time between the 
// transmission of the individual bytes is less than the pixel's latch time.
void _SendByte(uint8_t data)
{
	data;					// stop unreferenced argument warning
	__asm
	.even
		mov  r7, #8;		2 CLK - 8 bits to transfer
		xch  a, dpl;		2 CLK - data byte->accu;
	01$:
	rlc  a;				1 CLK - data bit->carry(MSB first)
		setb NEOPIN; 		2 CLK - NEO pin HIGH
		mov  NEOPIN, c;		2 CLK - "0" - bit ? ->NEO pin LOW now
		T1H_DELAY;			x CLK - TH1 delay
		clr  NEOPIN;		2 CLK - "1" - bit ? ->NEO pin LOW a little later
		TCT_DELAY;			y CLK - TCT delay
		djnz r7, 01$;		2 / 4 | 5 | 6 CLK - repeat for all bits;
	__endasm;
}

// ===================================================================================
// Write Buffer to Pixels
// ===================================================================================
void NEO_Update(void)
{
	uint8_t i;
	EA = 0;
	for (i = 0; i < 3 * NEO_COUNT; i++)
	{
		_SendByte(NEO_buffer[i]);
	}
	EA = 1;
	NEO_latch();
}

// ===================================================================================
// Clear all Pixels
// ===================================================================================
void NEO_ClearAll(void)
{
	uint8_t i;
	for (i = 0; i < NEO_COUNT; i++)
	{
		NEO_WriteColor(i, 0, 0, 0);
	}
}

// ===================================================================================
// Write Color to a Single Pixel in Buffer
// ===================================================================================
void NEO_WriteColor(uint8_t pixel, uint8_t r, uint8_t g, uint8_t b)
{
	__xdata uint8_t* pBase = NEO_buffer + (3 * pixel);
#if defined (NEO_GRB)
	pBase[0] = g;
	pBase[1] = r;
	pBase[2] = b;
#else
	pBase[0] = r;
	pBase[1] = g;
	pBase[2] = b;
#endif
}

// ===================================================================================
// Clear Single Pixel in Buffer
// ===================================================================================
void NEO_ClearOne(uint8_t pixel)
{
	NEO_WriteColor(pixel, 0, 0, 0);
}

void NEO_Init()
{
	PIN_low(PIN_NEO);
	PIN_output(PIN_NEO);              // init NeoPixels
}
