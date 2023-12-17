/********************************** (C) COPYRIGHT *******************************
* File Name          : CDCx2.C
* Author             : qianfan Zhao
* Version            : V1.0
* Date               : 2021/05/27
* Description        : CH552 virtual two channel CDC serial port
*******************************************************************************/
#include <ch554.h>
#include <debug.h>
//#include "cdc_x2.h"

void usb_isr_call();
void uart0_isr_call();
void uart1_isr_call();
void CDC_Run();
void CDC_Init();



void usb_isr(void) __interrupt (INT_NO_USB)                       //USB interrupt service routine, using register set 1
{
	usb_isr_call();
}

void uart0_isr(void) __interrupt (INT_NO_UART0)
{
	uart0_isr_call();
}

void uart1_isr(void) __interrupt (INT_NO_UART1)
{
	uart1_isr_call();
}

void main()
{
	CfgFsys( );
	mDelaymS(5);
	mInitSTDIO();
	UART1Setup();
	/* TI(Transmit Interrupt Flag) is seted in mInitSTDIO */
	TI = 0;

	CDC_Init();

	E_DIS = 0;
	EA = 1;		//Allow microcontroller interrupt

	while(1)
	{
		CDC_Run();
	}
}
