// ===================================================================================
// Delay Functions for CH551, CH552 and CH554
// ===================================================================================

#include "delay.h"
#include "ch554.h"

// ===================================================================================
// Delay in Units of us
// ===================================================================================
void DLY_us(uint16_t n) {           // delay in us
#if FREQ_SYS <= 6000000
	n >>= 2;
#endif
#if FREQ_SYS <= 3000000
	n >>= 2;
#endif
#if FREQ_SYS <= 750000
	n >>= 4;
#endif

	while (n)                        // total = 12~13 Fsys cycles, 1uS @Fsys=12MHz
	{
		SAFE_MOD++;                     // 2 Fsys cycles, for higher Fsys, add operation here
#if FREQ_SYS >= 14000000
		SAFE_MOD++;
#endif
#if FREQ_SYS >= 16000000
		SAFE_MOD++;
#endif
#if FREQ_SYS >= 18000000
		SAFE_MOD++;
#endif
#if FREQ_SYS >= 20000000
		SAFE_MOD++;
#endif
#if FREQ_SYS >= 22000000
		SAFE_MOD++;
#endif
#if FREQ_SYS >= 24000000
		SAFE_MOD++;
#endif
#if FREQ_SYS >= 26000000
		SAFE_MOD++;
#endif
#if FREQ_SYS >= 28000000
		SAFE_MOD++;
#endif
#if FREQ_SYS >= 30000000
		SAFE_MOD++;
#endif
#if FREQ_SYS >= 32000000
		SAFE_MOD++;
#endif
		n--;
	}
}

// ===================================================================================
// Delay in Units of ms
// ===================================================================================
void DLY_ms(uint16_t n) // delay in ms
{
	while (n)
	{
		DLY_us(1000);
		n--;
	}
}

// ===================================================================================
// Delay 20+4*(n-1) Clock Cycles
// ===================================================================================
#pragma callee_saves _delay_more_cycles
void _delay_more_cycles(uint8_t n) __naked
{
	n;					// stop unreferenced arg warning
	__asm
	.even; 				make predictable cycles for jumps
		push ar7;		2 cycles
		mov  r7, dpl;	2 cycles
		djnz r7, . + 0;	2 / 4 cycles
		pop ar7;		2 cycles
		ret;			4 | 5 cycles;
	__endasm;
}
