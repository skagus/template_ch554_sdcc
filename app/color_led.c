#include "neo.h"
#include "color_led.h"

__xdata uint8_t gaRgbBuf[3 * NEO_COUNT];  // pixel buffer
__xdata uint8_t gbUpdated;
// ===================================================================================
// Clear all Pixels
// ===================================================================================
// ===================================================================================
// Clear Single Pixel in Buffer
// ===================================================================================
void NEO_ClearOne(uint8_t pixel)
{
	NEO_WriteColor(pixel, 0, 0, 0);
}

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
	__xdata uint8_t* pBase = gaRgbBuf + (3 * pixel);
#if defined (NEO_GRB)
	pBase[0] = g;
	pBase[1] = r;
	pBase[2] = b;
#else
	pBase[0] = r;
	pBase[1] = g;
	pBase[2] = b;
#endif
	gbUpdated = 1;
}

void CLED_Init()
{
	NEO_ClearAll();
	gbUpdated = 1;
}

void CLED_Run()
{
	if (gbUpdated)
	{
		NEO_Update(gaRgbBuf, NEO_COUNT);
		gbUpdated = 0;
	}
}

