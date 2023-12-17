

/********************************** (C) COPYRIGHT *******************************
* Author             : WCH
* Version            : V1.0
* Date               : 2017/07/05
* Description        :
*******************************************************************************/

#include <ch554.h>
#include "spi.h"

/*******************************************************************************
* Description    : SPI mode. (default low = 0 or default high = 3)
* Input          : uint8_t mode						 
*******************************************************************************/
void SPIMasterModeSet(uint8_t mode)
{
	SPI0_SETUP = 0;
	if(mode == 0) // default low.
	{
		SPI0_CTRL = bS0_MOSI_OE | bS0_SCK_OE; // 0x60;
	}			
	else if(mode == 3) // default High.
	{
		SPI0_CTRL = bS0_MOSI_OE | bS0_SCK_OE | bS0_MST_CLK; // 0x68;
	}			
	P1_MOD_OC &= 0x0F;
	P1_DIR_PU |= 0xB0;
	P1_DIR_PU &= 0xBF;

	// Set clock speed
	SPI0_CK_SE = 0x02; // clock divisor.
}

/*******************************************************************************
* Description    : CH554SPI interrupt.
*******************************************************************************/
void CH554SPIInterruptInit()
{
	//IP_EX |= bIP_SPI0;
	SPI0_SETUP |= bS0_IE_FIFO_OV | bS0_IE_BYTE;
	SPI0_CTRL |= bS0_AUTO_IF;
	SPI0_STAT |= 0xff;
#ifdef SPI_Interrupt
	IE_SPI0 = 1;
#endif
}

/*******************************************************************************
* Description    : 
* Input          : uint8_t dat
*******************************************************************************/
void CH554SPIMasterWrite(uint8_t dat)
{
	SPI0_DATA = dat;
	while(S0_FREE == 0);
}

/*******************************************************************************
* Function Name  : CH554SPIMasterRead( )
* Description    :
* Return         : uint8_t read data
*******************************************************************************/
uint8_t CH554SPIMasterRead()
{
	SPI0_DATA = 0xff;
	while(S0_FREE == 0);
	return SPI0_DATA;
}

/*******************************************************************************
* Description    :
*******************************************************************************/
void SPISlvModeSet( )
{
	SPI0_SETUP = bS0_MODE_SLV;
	SPI0_CTRL = bS0_MISO_OE | bS0_AUTO_IF; //0x81;
	P1_MOD_OC &= 0x0F;
	P1_DIR_PU &= 0x0F;
}

/*******************************************************************************
* Description    : 
* Input          : uint8_t dat
*******************************************************************************/
void CH554SPISlvWrite(uint8_t dat)
{
	SPI0_DATA = dat;
	while(S0_IF_BYTE==0);	
	S0_IF_BYTE = 0;		                                                     
}

/*******************************************************************************
* Description    :
* Return         : uint8_t ret   
*******************************************************************************/
uint8_t CH554SPISlvRead()
{
	while(S0_IF_BYTE==0);
	S0_IF_BYTE = 0;	
	return SPI0_DATA;
}

