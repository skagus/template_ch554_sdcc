#pragma once

#define NOT(x)							(!(x))

#define BIT(shift)						(1 <<(shift))
#define BIT_SET(dst, mask)				((dst) |= (mask))
#define BIT_CLR(dst, mask)				((dst) &= ~(mask))
#define BIT_TST(dst, mask)				((dst) & (mask))

#define GAP_BTWN_16(u16big, u16less)		(((u16big) - (u16less)) & 0x7FFF)
