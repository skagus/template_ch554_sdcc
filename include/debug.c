/********************************** (C) COPYRIGHT *******************************
* File Name          : Debug.C
* Author             : WCH
* Version            : V1.0
* Date               : 2017/01/20
* Description        : CH554 DEBUG Interface
                     CH554主频修改、延时函数定义
                     串口0和串口1初始化
                     串口0和串口1的收发子函数
                     看门狗初始化										 
*******************************************************************************/

#include <stdint.h>

#include "ch554.h"
#include "debug.h"

/*******************************************************************************
* Function Name  : CfgFsys( )
* Description    : Setup Clock, default: 6MHz
					CLOCK_CFG calculation：
					Fsys = (Fosc * 4/(CLOCK_CFG & MASK_SYS_CK_SEL);
*******************************************************************************/ 
void	CfgFsys( )  
{
	SAFE_MOD = 0x55;
	SAFE_MOD = 0xAA;

//	CLOCK_CFG |= bOSC_EN_XT;		// enable external osc.
//	CLOCK_CFG &= ~bOSC_EN_INT;	// disable internal osc.

#if FREQ_SYS == 32000000
	CLOCK_CFG = CLOCK_CFG & ~ MASK_SYS_CK_SEL | 0x07;  // 32MHz
#elif FREQ_SYS == 24000000
	CLOCK_CFG = CLOCK_CFG & ~ MASK_SYS_CK_SEL | 0x06;  // 24MHz	
#elif FREQ_SYS == 16000000
	CLOCK_CFG = CLOCK_CFG & ~ MASK_SYS_CK_SEL | 0x05;  // 16MHz	
#elif FREQ_SYS == 12000000
	CLOCK_CFG = CLOCK_CFG & ~ MASK_SYS_CK_SEL | 0x04;  // 12MHz
#elif FREQ_SYS == 6000000
	CLOCK_CFG = CLOCK_CFG & ~ MASK_SYS_CK_SEL | 0x03;  // 6MHz	
#elif FREQ_SYS == 3000000
	CLOCK_CFG = CLOCK_CFG & ~ MASK_SYS_CK_SEL | 0x02;  // 3MHz	
#elif FREQ_SYS == 750000
	CLOCK_CFG = CLOCK_CFG & ~ MASK_SYS_CK_SEL | 0x01;  // 750KHz	
#elif FREQ_SYS == 187500
	CLOCK_CFG = CLOCK_CFG & ~ MASK_SYS_CK_SEL | 0x00;  // 187.5KHz		
#else
	#warning FREQ_SYS invalid or not set
#endif

	SAFE_MOD = 0x00;
}



/*******************************************************************************
* Function Name  : mDelayus(UNIT16 n)
* Description    :
* Input          : UNIT16 n
* Output         : None
* Return         : None
*******************************************************************************/ 
void	mDelayuS( uint16_t n )
{
#ifdef	FREQ_SYS
#if		FREQ_SYS <= 6000000
		n >>= 2;
#endif
#if		FREQ_SYS <= 3000000
		n >>= 2;
#endif
#if		FREQ_SYS <= 750000
		n >>= 4;
#endif
#endif
	while ( n ) {  // total = 12~13 Fsys cycles, 1uS @Fsys=12MHz
		++ SAFE_MOD;  // 2 Fsys cycles, for higher Fsys, add operation here
#ifdef	FREQ_SYS
#if		FREQ_SYS >= 14000000
		++ SAFE_MOD;
#endif
#if		FREQ_SYS >= 16000000
		++ SAFE_MOD;
#endif
#if		FREQ_SYS >= 18000000
		++ SAFE_MOD;
#endif
#if		FREQ_SYS >= 20000000
		++ SAFE_MOD;
#endif
#if		FREQ_SYS >= 22000000
		++ SAFE_MOD;
#endif
#if		FREQ_SYS >= 24000000
		++ SAFE_MOD;
#endif
#if		FREQ_SYS >= 26000000
		++ SAFE_MOD;
#endif
#if		FREQ_SYS >= 28000000
		++ SAFE_MOD;
#endif
#if		FREQ_SYS >= 30000000
		++ SAFE_MOD;
#endif
#if		FREQ_SYS >= 32000000
		++ SAFE_MOD;
#endif
#endif
		-- n;
	}
}

/*******************************************************************************
* Function Name  : mDelayms(UNIT16 n)
* Description    : ms延时函数
* Input          : UNIT16 n
* Output         : None
* Return         : None
*******************************************************************************/
void	mDelaymS( uint16_t n )
{
	while ( n ) {
#ifdef	DELAY_MS_HW
		while ( ( TKEY_CTRL & bTKC_IF ) == 0 );
		while ( TKEY_CTRL & bTKC_IF );
#else
		mDelayuS( 1000 );
#endif
		-- n;
	}
}                                         

/*******************************************************************************
* Function Name  : CH554UART0Alter()
* Description    : Change Pin mapping
*******************************************************************************/
void CH554UART0Alter()
{
    PIN_FUNC |= bUART0_PIN_X;
}


/*******************************************************************************
* Function Name  : mInitSTDIO()
* Description    : CH554 UART0 is initialized,
				T1 is used as the baud rate generator by default, T2 can also be used
*******************************************************************************/
void	mInitSTDIO( )
{
	volatile uint32_t x;
	volatile uint8_t x2;

	SM0 = 0;
	SM1 = 1;
	SM2 = 0;		//UART0 uses mode 1, Timer 1 as baud rate gen.

	RCLK = 0;		// UART0 rx clock.
	TCLK = 0;		// UART0 tx clock.
	PCON |= SMOD;
	// If you change the main frequency, be careful not to overflow the value of x
	x = 10 * FREQ_SYS / UART0_BAUD / 16;	
	x2 = x % 10;
	x /= 10;
	if ( x2 >= 5 ) x ++; // ceiling.

	TMOD = TMOD & ~ bT1_GATE & ~ bT1_CT & ~ MASK_T1_MOD | bT1_M1; // 0X20，Timer1: 8bit, auto reload
	T2MOD = T2MOD | bTMR_CLK | bT1_CLK;		// Timer1 clock selection.
	TH1 = 0-x;			// 12MHz crystal oscillator, buad/12 is the actual baud rate that needs to be set
	TR1 = 1;			// start timer 1
	TI = 1;
	REN = 1;			// UART0 RX enable.
}

/*******************************************************************************
* Function Name  : CH554UART0RcvByte()
* Description    : UART0 rx.
* Return         : SBUF
*******************************************************************************/
uint8_t  CH554UART0RcvByte( )
{
	while(RI == 0);
	RI = 0;
	return SBUF;
}

/*******************************************************************************
* Function Name  : CH554UART0SendByte(uint8_t SendDat)
* Description    : CH554 UART0 TX
* Input          : uint8_t SendDat
*******************************************************************************/
void CH554UART0SendByte(uint8_t SendDat)
{
	SBUF = SendDat;
	while(TI ==0);
	TI = 0;
}

#if SDCC < 370
void putchar(char c)
{
	while (!TI) /* assumes UART is initialized */
	;
	TI = 0;
	SBUF = c;
}

char getchar()
{
	while(!RI); /* assumes UART is initialized */
	RI = 0;
	return SBUF;
}
#else
int putchar(int c)
{
	while (!TI) /* assumes UART is initialized */
	;
	TI = 0;
	SBUF = c & 0xFF;

	return c;
}

int getchar()
{
	while(!RI); /* assumes UART is initialized */
	RI = 0;
	return SBUF;
}
#endif

/*******************************************************************************
* Function Name  : CH554UART1Alter()
* Description    : Set the alternate pin mappings for UART1 (TX on P3.2, RX on P3.4)
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void CH554UART1Alter()
{
	PIN_FUNC |= bUART1_PIN_X;
}

/*******************************************************************************
* Function Name  : UART1Setup()
* Description    : UART1 init.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void	UART1Setup( )
{
	U1SM0 = 0; // UART1 data bit : 8bit
	U1SMOD = 1; // Fast mode.
	U1REN = 1; // RX enable
	SBAUD1 = 256 - FREQ_SYS/16/UART1_BAUD;
}

/*******************************************************************************
* Function Name  : CH554UART1RcvByte()
* Description    : CH554 UART1 RX
* Input          : None
* Output         : None
* Return         : SBUF
*******************************************************************************/
uint8_t  CH554UART1RcvByte( )
{
	while(U1RI == 0);
	U1RI = 0;
	return SBUF1;
}

/*******************************************************************************
* Function Name  : CH554UART1SendByte(uint8_t SendDat)
* Description    : CH554 UART1 TX
* Input          : uint8_t SendDat
* Output         : None
* Return         : None
*******************************************************************************/
void CH554UART1SendByte(uint8_t SendDat)
{
	SBUF1 = SendDat;
	while(U1TI ==0);
	U1TI = 0;
}

/*******************************************************************************
* Function Name  : CH554WDTModeSelect(uint8_t mode)
* Description    : CH554 WDT mode select.
* Input          : uint8_t mode
                   0  timer
                   1  watchDog
* Output         : None
* Return         : None
*******************************************************************************/
void CH554WDTModeSelect(uint8_t mode)
{
	SAFE_MOD = 0x55;
	SAFE_MOD = 0xaa;
	if(mode) GLOBAL_CFG |= bWDOG_EN;
	else GLOBAL_CFG &= ~bWDOG_EN;
	SAFE_MOD = 0x00;
	WDOG_COUNT = 0;
}

/*******************************************************************************
* Function Name  : CH554WDTFeed(uint8_t tim)
* Description    : CH554 WDT feeding.
* Input          : uint8_t tim
                   00H(6MHz)=2.8s
                   80H(6MHz)=1.4s
* Output         : None
* Return         : None
*******************************************************************************/
void CH554WDTFeed(uint8_t tim)
{
	WDOG_COUNT = tim;
}
