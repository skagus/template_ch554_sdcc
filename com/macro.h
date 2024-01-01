#pragma once


#define UNUSED(x)						((void)(x))
#define NOT(x)							(!(x))

#define BIT(shift)						(1 <<(shift))
#define BIT_SET(dst, mask)				((dst) |= (mask))
#define BIT_CLR(dst, mask)				((dst) &= ~(mask))
#define BIT_TST(dst, mask)				((dst) & (mask))

#define GAP_BTWN_16(u16big, u16less)		(((u16big) - (u16less)) & 0x7FFF)


/** For MCS51, SDCC **/
#define ATTR_IRQ(INT_NO)				__interrupt(INT_NO)
#define __enable_irq()					do{EA=1;}while(0)
#define __disable_irq()					do{EA=0;}while(0)
