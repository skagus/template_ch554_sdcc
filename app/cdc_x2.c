#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <ch554.h>
#include <ch554_usb.h>
#include <debug.h>
#include "cdc_x2.h"

#define EN_SNIFF					(0)
#define EN_DBG						(0)

#define EP_SIZE_BULK				(MAX_PACKET_SIZE)
#define EP_SIZE_INT					(DEFAULT_ENDP0_SIZE)
#define UART_FIFO_SIZE				(64)
#define NUM_CDC						(2)

typedef struct _RingBuf
{
	uint8_t		aRing[UART_FIFO_SIZE];
	uint16_t		nWrIdx;
	uint16_t		nRdIdx;
	uint16_t		nCnt;
} RingBuf;

__xdata uint8_t  gaBuf4EP3[2 * EP_SIZE_BULK];	// CDC 1, 
__xdata uint8_t  gaBuf4EP2[2 * EP_SIZE_BULK];	// CDC 0
__xdata uint8_t  gaBuf4EP0[EP_SIZE_INT];	// for control.

static volatile __xdata RingBuf gUartFifoB = {.nWrIdx = 0,.nRdIdx = 0,.nCnt = 0,};
static volatile __xdata RingBuf gUartFifoA = {.nWrIdx = 0,.nRdIdx = 0,.nCnt = 0,};

inline void UEP_DONE_D2H(uint8_t nEP)
{
	if(1 == nEP)
	{
		UEP1_T_LEN = 0;
		UEP1_CTRL = (UEP1_CTRL & (~MASK_UEP_T_RES)) | UEP_T_RES_NAK;
	}
	else if(2 == nEP)
	{
		UEP2_T_LEN = 0;
		UEP2_CTRL = (UEP2_CTRL & (~MASK_UEP_T_RES)) | UEP_T_RES_NAK;
	}
	else if(3 == nEP)
	{
		UEP3_T_LEN = 0;
		UEP3_CTRL = (UEP3_CTRL & (~MASK_UEP_T_RES)) | UEP_T_RES_NAK;
	}
	else if(4 == nEP)
	{
		UEP4_T_LEN = 0;
		UEP4_CTRL = (UEP4_CTRL & (~MASK_UEP_T_RES)) | UEP_T_RES_NAK;
	}
}

inline void UEP_TRIG_D2H(uint8_t nEP, uint8_t nTxLen)
{
	if(1 == nEP)
	{
		UEP1_T_LEN = nTxLen;
		UEP1_CTRL = UEP1_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_ACK;
	}
	else if(2 == nEP)
	{
		UEP2_T_LEN = nTxLen;
		UEP2_CTRL = UEP2_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_ACK;
	}
	else if(3 == nEP)
	{
		UEP3_T_LEN = nTxLen;
		UEP3_CTRL = UEP3_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_ACK;
	}
	else if(4 == nEP)
	{
		UEP4_T_LEN = nTxLen;
		UEP4_CTRL = UEP4_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_ACK;
	}
}

/**
 * Just send response, NOT resource free.
*/
inline void UEP_RESP_H2D(uint8_t nEP)
{
	if(1 == nEP)
	{
		UEP1_CTRL = (UEP1_CTRL & (~MASK_UEP_R_RES)) | UEP_R_RES_NAK;
	}
	else if (2 == nEP)
	{
		UEP2_CTRL = (UEP2_CTRL & (~MASK_UEP_R_RES)) | UEP_R_RES_NAK;
	}
	else if(3 == nEP)
	{
		UEP3_CTRL = (UEP3_CTRL & (~MASK_UEP_R_RES)) | UEP_R_RES_NAK;
	}
	else if(4 == nEP)
	{
		UEP4_CTRL = (UEP4_CTRL & (~MASK_UEP_R_RES)) | UEP_R_RES_NAK;
	}
}

inline void UEP_DONE_H2D(uint8_t nEP)
{
	if(1 == nEP)
	{
		UEP1_CTRL = UEP1_CTRL & ~ MASK_UEP_R_RES | UEP_R_RES_ACK;
	}
	else if(2 == nEP)
	{
		UEP2_CTRL = UEP2_CTRL & ~ MASK_UEP_R_RES | UEP_R_RES_ACK;
	}
	else if(3 == nEP)
	{
		UEP3_CTRL = UEP3_CTRL & ~ MASK_UEP_R_RES | UEP_R_RES_ACK;
	}
	else if(4 == nEP)
	{
		UEP4_CTRL = UEP4_CTRL & ~ MASK_UEP_R_RES | UEP_R_RES_ACK;
	}
}

inline uint16_t uart_fifo_length(RingBuf* pstFifo)
{
	return pstFifo->nCnt;
}

inline uint8_t uart_fifo_is_empty(RingBuf* pstFifo)
{
	return (0 == pstFifo->nCnt);
}

// Called from ISR ONLY.
inline uint8_t uart_fifo_is_full(RingBuf* pstFifo)
{
	return (uart_fifo_length(pstFifo) == UART_FIFO_SIZE);
}

// Called from ISR ONLY.
inline void uart_fifo_put(RingBuf* pstFifo, uint8_t nData)
{
	if (!uart_fifo_is_full(pstFifo))
	{
		pstFifo->aRing[pstFifo->nWrIdx] = nData;
		pstFifo->nWrIdx = (pstFifo->nWrIdx + 1) % UART_FIFO_SIZE;
		pstFifo->nCnt++;
	}
}

inline void uart_fifo_get_without_check(RingBuf* pstFifo, uint8_t* pData)
{
	while(uart_fifo_is_empty(pstFifo));
	EA = 0;
	*pData = pstFifo->aRing[pstFifo->nRdIdx];
	pstFifo->nRdIdx = (pstFifo->nRdIdx + 1) % UART_FIFO_SIZE;
	pstFifo->nCnt--;
	EA = 1;
}


typedef struct _CdcDev
{
	uint8_t		anLineCoding[7];	// CDC configuration information.
	uint8_t		bRunD2H;
	uint8_t		nH2DSize;
	uint8_t		nNext2Uart;			// index pointing char in EP. (to out via UART)
	uint8_t		nEP;
	__xdata	uint8_t*	pInBuf;
	__xdata	uint8_t*	pOutBuf;
	__xdata RingBuf*	pFifo;
} CdcDev;

static __xdata CdcDev gaCdc[NUM_CDC] = 
{
	{ // A,
		.anLineCoding = { 0x00, 0xe1, 0x00, 0x00, 0x00, 0x00, 0x08 },
		.bRunD2H = 0,
		.nH2DSize = 0,
		.nNext2Uart = 0,
		.nEP = 2,
		.pInBuf = gaBuf4EP2,
		.pOutBuf = gaBuf4EP2 + EP_SIZE_BULK,
		.pFifo = &gUartFifoA,
	},
	{ // B,
		.anLineCoding = { 0x00, 0xe1, 0x00, 0x00, 0x00, 0x00, 0x08 },
		.bRunD2H = 0,
		.nH2DSize = 0,
		.nNext2Uart = 0,
		.nEP = 3,
		.pInBuf = gaBuf4EP3,
		.pOutBuf = gaBuf4EP3 + EP_SIZE_BULK,
		.pFifo = &gUartFifoB,
	},
};

/*******************************************************************************
* Description    : USB device mode configuration
*******************************************************************************/
void USB_DeviceCfg()
{
	// default: Full speed 12M mode.
	USB_CTRL =  bUC_DEV_PU_EN | bUC_INT_BUSY | bUC_DMA_EN;		//USB device and internal pull-up enable, automatically return to NAK before interrupt flag is cleared
	USB_DEV_AD = 0x00;						// Device address initialization
	UDEV_CTRL = bUD_PD_DIS | bUD_PORT_EN;	// Disable DP/DM pull-down resistor, Enable physical port
}
/*******************************************************************************
* Description    : USB device mode interrupt initialization
*******************************************************************************/
void USB_DeviceIntCfg()
{
	// Enable suspend, Tx completion, bus reset interrupt.
	USB_INT_EN |= bUIE_SUSPEND | bUIE_TRANSFER | bUIE_BUS_RST;
	USB_INT_FG |= 0x1F;		//Clear interrupt flag
	IE_USB = 1;					//Enable USB interrupt
}
/*******************************************************************************
* Description    : USB device mode endpoint configuration, simulation compatible HID device, in addition to endpoint 0 control transmission, also includes endpoint 2 batch upload
*******************************************************************************/
void USB_DeviceEndPointCfg()
{
	UEP0_DMA = (uint16_t) gaBuf4EP0;	// Endpoint 0 data transfer address
	UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;	// Manual flip, OUT transaction returns ACK, IN transaction returns NAK

	UEP2_DMA = (uint16_t) gaBuf4EP2;	// Endpoint 2, D2H, H2D
	UEP2_CTRL = bUEP_AUTO_TOG | UEP_T_RES_NAK | UEP_R_RES_ACK;	//Endpoint 2 automatically flips the synchronization flag, IN transaction returns NAK, OUT returns ACK

	UEP3_DMA = (uint16_t) gaBuf4EP3;	// Endpoint 3, D2H, H2D
	UEP3_CTRL = bUEP_AUTO_TOG | UEP_T_RES_NAK | UEP_R_RES_ACK;

//	UEP1_DMA = (uint16_t) gaBuf4EP1;	// Endpoint 1 sends data transfer address
	UEP1_CTRL = bUEP_AUTO_TOG | UEP_T_RES_NAK | UEP_R_RES_ACK;	// Endpoint 1 automatically flips the synchronization flag, IN transaction returns NAK

//	UEP4_DMA = (uint16_t) gaBuf4EP4;	// Endpoint 4, D2H, H2D
	UEP4_CTRL = bUEP_AUTO_TOG | UEP_T_RES_NAK;

	UEP2_3_MOD = bUEP3_RX_EN | bUEP3_TX_EN | bUEP2_RX_EN | bUEP2_TX_EN; //0xCC;
	UEP4_1_MOD = bUEP1_TX_EN | bUEP4_TX_EN; // 0x44; // Endpoint 1 TX buffer;

	UEP0_T_LEN = 0;
	UEP1_T_LEN = 0;
	UEP2_T_LEN = 0;
	UEP3_T_LEN = 0;
	UEP4_T_LEN = 0;
}

static volatile uint8_t gnUsbNewAddr = 0;
static volatile uint8_t gnUsbCfg = 0;
static const uint8_t *gpXferAddr = NULL;
static volatile uint16_t gnXferSize = 0;
static volatile USB_SETUP_REQ gstUsbLastSetupReq;

void handleReset(void)
{
	UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
	UEP1_CTRL = bUEP_AUTO_TOG | UEP_T_RES_NAK | UEP_R_RES_ACK;;
	UEP2_CTRL = bUEP_AUTO_TOG | UEP_T_RES_NAK | UEP_R_RES_ACK;
	UEP3_CTRL = bUEP_AUTO_TOG | UEP_T_RES_NAK | UEP_R_RES_ACK;
	UEP4_CTRL = bUEP_AUTO_TOG | UEP_T_RES_NAK;

	USB_DEV_AD = 0x00;
	UIF_SUSPEND = 0;
	UIF_TRANSFER = 0;
	UIF_BUS_RST = 0;

	for(uint8_t nIdx = 0; nIdx < NUM_CDC; nIdx++)
	{
		gaCdc[nIdx].bRunD2H = 0;
		gaCdc[nIdx].nH2DSize = 0;
	}
	gnUsbCfg = 0;
}

void handleSuspend(void)
{
	UIF_SUSPEND = 0;
	if ( USB_MIS_ST & bUMS_SUSPEND )
	{
		while ( XBUS_AUX & bUART0_TX ) {;} // Wait Xfer done.

		SAFE_MOD = 0x55;
		SAFE_MOD = 0xAA;
		WAKE_CTRL = bWAK_BY_USB | bWAK_RXD0_LO | bWAK_RXD1_LO; // Wakeup on USB event, RXD0/1 signal low.
		PCON |= PD; // Sleep.
		SAFE_MOD = 0x55;
		SAFE_MOD = 0xAA;
		WAKE_CTRL = 0x00;
	}
}

/***
 * Handle data sending.
 */
const uint8_t* gaNum2Hex = "0123456789ABCDEF";
void handleD2H(uint8_t nEP, uint8_t nDbgVal)
{
	if(0 == nEP)
	{
		switch(gstUsbLastSetupReq.bRequest)
		{
			case USB_GET_DESCRIPTOR:
			{
				int nThisLen = gnXferSize;

				if (nThisLen == 0 && !gpXferAddr)
				{
					/* nothing need sending, force ending setup transfer */
					UEP0_T_LEN = 0;
					UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
					break;
				}
				else if (nThisLen > EP_SIZE_INT)
				{
					nThisLen = EP_SIZE_INT;
				}

				memcpy(gaBuf4EP0, gpXferAddr, nThisLen);
				gpXferAddr += nThisLen;
				gnXferSize -= nThisLen;
				if (nThisLen < EP_SIZE_INT && gnXferSize == 0) // Final packet.
				{
					gpXferAddr = NULL;
				}

				UEP0_T_LEN = nThisLen;
				UEP0_CTRL ^= bUEP_T_TOG;
				break;
			}
			case USB_SET_ADDRESS:
			{
				USB_DEV_AD = USB_DEV_AD & bUDA_GP_BIT | gnUsbNewAddr;
				UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
				break;
			}
			default:
			{
				UEP0_T_LEN = 0;
				UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
				break;
			}
		}
	}
	else if(2 == nEP)
	{
		UEP_DONE_D2H(nEP);
		gaCdc[0].bRunD2H = 0;
#if (EN_DBG == 1)
		uart_fifo_put(&gUartFifoB, '-');
		uart_fifo_put(&gUartFifoB, gaNum2Hex[nDbgVal / 16]);
		uart_fifo_put(&gUartFifoB, gaNum2Hex[nDbgVal % 16]);
		uart_fifo_put(&gUartFifoB, '\n');
#endif
	}
	else if(3 == nEP)
	{
		UEP_DONE_D2H(nEP);
		gaCdc[1].bRunD2H = 0;
	}
}

uint32_t get_baud_rate(__xdata uint8_t* acm_line_code)
{
	uint32_t baud = 0;

	baud |= acm_line_code[3];
	baud <<= 8;
	baud |= acm_line_code[2];
	baud <<= 8;
	baud |= acm_line_code[1];
	baud <<= 8;
	baud |= acm_line_code[0];
	return baud;
}

/**
 * Handle data receiving.
 **/
void handleH2D(uint8_t nEP)
{
	__xdata CdcDev *pstCdc = NULL;

	if (U_TOG_OK)  /* Out of sync packets will be dropped. */
	{
		if(0 == nEP)
		{
			switch (gstUsbLastSetupReq.bRequest)
			{
				case SET_LINE_CODING:
				{
					if (gstUsbLastSetupReq.wIndexL == 0) /* interface 0 is CDC0 */
					{
						pstCdc = &gaCdc[0];
						uint32_t baud = get_baud_rate(gaBuf4EP0);
						TH1 = 256 - FREQ_SYS / baud / 16;
					}
					else if (gstUsbLastSetupReq.wIndexL == 2) /* interface 2 is CDC1 */
					{
						pstCdc = &gaCdc[1];
						uint32_t baud = get_baud_rate(gaBuf4EP0);
						SBAUD1 = 256 - FREQ_SYS / 16 / baud;
					}
					else if (gstUsbLastSetupReq.wIndexL == 4) /* interface 2 is CDC2 */
					{
						pstCdc = &gaCdc[2];
					}
					if (NULL != pstCdc)
					{
						memcpy(pstCdc->anLineCoding, gaBuf4EP0, USB_RX_LEN);
						UEP0_T_LEN = 0;
						UEP0_CTRL |= UEP_R_RES_ACK | UEP_T_RES_ACK;
					}
					else
					{
						UEP0_T_LEN = 0;
						UEP0_CTRL |= UEP_R_RES_ACK | UEP_T_RES_NAK;
					}
					break;
				}
				default:
				{
					UEP0_T_LEN = 0;
					UEP0_CTRL |= UEP_R_RES_ACK | UEP_T_RES_NAK;
					break;
				}
			}
		}
		else if(2 == nEP)
		{
			gaCdc[0].nH2DSize = USB_RX_LEN;
			UEP_RESP_H2D(nEP);
		}
		else if(3 == nEP)
		{
			gaCdc[1].nH2DSize = USB_RX_LEN;
			UEP_RESP_H2D(nEP);
		}
	}
}

/**
 * Get Descriptor information into gpSetupXferPoint/gnSetupXferSize
 **/
int parepareDesc(uint8_t nDescType, uint8_t nStrIdx)
{
	const uint8_t *pstDest = NULL;
	int nSize = 0;

	switch (nDescType)
	{
		case 1: /* device descriptor */
		{
			pstDest = gDevDesc_CDC_ACM2;
			nSize = sizeof(gDevDesc_CDC_ACM2);
			break;
		}
		case 2: /* config descriptor */
		{
			pstDest = gCfgDesc_CDC_ACM2;
			nSize = sizeof(gCfgDesc_CDC_ACM2);
			break;
		}
		case 3: /* string descriptor */
		{
			switch (nStrIdx)
			{
				case 0:
					pstDest = gLangDesc;
					nSize = sizeof(gLangDesc);
					break;
				case IDX_MANUFACTURER_DESCRIPTOR_TYPE: /* iManufacturer */
					pstDest = gszManuDesc;
					nSize = sizeof(gszManuDesc);
					break;
				case IDX_PRODUCT_DESCRIPTOR_TYPE:  /* iProduct */
					pstDest = gszProdDesc;
					nSize = sizeof(gszProdDesc);
					break;
				case IDX_SERIAL_DESCRIPTOR_TYPE: /* iSerial */
					pstDest = gszSerialDesc;
					nSize = sizeof(gszSerialDesc);
					break;
				default:
					return -1;
			}
			break;
		}
		default:
			return -1;
	}

	// 꼭 필요함...
	if (nSize > gnXferSize)
		nSize = gnXferSize;

	gnXferSize = nSize;
	gpXferAddr = pstDest;

	return 0;
}

int handleSetupStd(PXUSB_SETUP_REQ req)
{
	int nRet = 0;
	switch (req->bRequest)
	{
		case USB_GET_DESCRIPTOR:
			nRet = parepareDesc(req->wValueH, req->wValueL);
			break;
		case USB_SET_ADDRESS:
			/* new address is addressed after device ACK */
			gnUsbNewAddr = req->wValueL;
			gnXferSize = 0;
			gpXferAddr = NULL;
			break;
		case USB_GET_CONFIGURATION:
			gnXferSize = sizeof(gnUsbCfg);
			gpXferAddr = &gnUsbCfg;
			break;
		case USB_SET_CONFIGURATION:
			gnUsbCfg = req->wValueL;
			gnXferSize = 0;
			gpXferAddr = NULL;
			break;
		case USB_GET_INTERFACE:
			break;
		default:
			nRet = -1;
	}

	return nRet;
}

int handleSetupVendor(PXUSB_SETUP_REQ pstReq)
{
	switch (pstReq->bRequest)
	{
		case GET_LINE_CODING:
		{
			if(pstReq->wIndexL <= 4)
			{
				gpXferAddr = gaCdc[pstReq->wIndexL / 2].anLineCoding;
			}
			else
			{
				return -1;
			}
			break;
		}
		case SET_CONTROL_LINE_STATE:
		case SET_LINE_CODING:
			gpXferAddr = NULL;
			gnXferSize = 0;
			/* setting data packet defined in EP0 packet out */
			break;
		default:
			return -1;
	}

	return 0;
}

/**
 * Setup handler called only for EP0
 **/
void handleSetup(uint8_t nEP)
{
	if (nEP == 0)
	{
		PXUSB_SETUP_REQ pstReq = (PXUSB_SETUP_REQ)gaBuf4EP0;
		int bFailed = -1;
	
		if(USB_RX_LEN == (sizeof(USB_SETUP_REQ)))
		{
			gnXferSize = ((uint16_t)pstReq->wLengthH << 8) | pstReq->wLengthL;
			memcpy(&gstUsbLastSetupReq, pstReq, sizeof(gstUsbLastSetupReq));

			if ((pstReq->bRequestType & USB_REQ_TYP_MASK ) == USB_REQ_TYP_STANDARD)
			{
				bFailed = handleSetupStd(pstReq);
			}
			else
			{
				bFailed = handleSetupVendor(pstReq);
			}
		}

		if (bFailed || (gnXferSize > 0 && !gpXferAddr))
		{
			/* STALL request */
			UEP0_CTRL = bUEP_R_TOG | bUEP_T_TOG | UEP_R_RES_STALL | UEP_T_RES_STALL;
			return;
		}

		int n1stSize = (gnXferSize > EP_SIZE_INT) ? EP_SIZE_INT : gnXferSize;

		memcpy(gaBuf4EP0, gpXferAddr, n1stSize);
		/* The last packet of data is less than the maximum length of EP0, no need to add empty packets */
		if (gnXferSize == 0 && n1stSize < EP_SIZE_INT) // Final packet.
		{
			gpXferAddr = NULL;
		}

		gpXferAddr += n1stSize;
		gnXferSize -= n1stSize;

		UEP0_T_LEN = n1stSize;
		UEP0_CTRL = bUEP_R_TOG | bUEP_T_TOG | UEP_R_RES_ACK | UEP_T_RES_ACK;
	}
}

void usb_isr_call(void)
{
	if (UIF_TRANSFER) // Transfer done (partial or full)
	{
		uint8_t nDbg = USB_MIS_ST;
		uint8_t nIntSt = USB_INT_ST;
		uint8_t nEP = nIntSt & MASK_UIS_ENDP;
		switch (nIntSt & MASK_UIS_TOKEN)
		{
			case UIS_TOKEN_IN:
				handleD2H(nEP, nDbg);
				break;
			case UIS_TOKEN_OUT:
				handleH2D(nEP);
				break;
			case UIS_TOKEN_SETUP:
				handleSetup(nEP);
				break;
		}
		UIF_TRANSFER = 0; /* clear interrupt */
	}
	else if (UIF_BUS_RST)
	{
		handleReset();
	}
	else if (UIF_SUSPEND)
	{
		handleSuspend();
	}
	else
	{
		USB_INT_FG = 0xFF;
	}
}

void uart0_isr_call(void)
{
	if (RI)
	{
		uint8_t nRcv = SBUF;
		uart_fifo_put(&gUartFifoA, nRcv);
#if (EN_SNIFF == 1)
		uart_fifo_put(&gUartFifoSniff, nRcv);
#endif
		RI = 0;
	}
}

void uart1_isr_call(void)
{
	if (U1RI)
	{
		uint8_t nRcv = SBUF1;
		uart_fifo_put(&gUartFifoB, nRcv);
		U1RI = 0;
	}
}

void handle_cdc(int nCdcId, __xdata CdcDev* pCdc)
{
	static uint8_t anTimeChk[NUM_CDC] = {0,0};

	int nTxLen;
	__xdata uint8_t* aOutBuf = pCdc->pOutBuf;
	__xdata uint8_t* aInBuf = pCdc->pInBuf;

	nTxLen = uart_fifo_length(pCdc->pFifo);
	if ((nTxLen > 0) && (0 == pCdc->bRunD2H))
	{
		if ((anTimeChk[nCdcId] > 20)
			|| (nTxLen >= (EP_SIZE_BULK / 2)))
		{
			uint8_t nOrgLen = nTxLen;
			if(nTxLen > EP_SIZE_BULK - 1)
			{
				nTxLen = EP_SIZE_BULK - 1;
			}

			for (int i = 0; i < nTxLen; i++)
			{
				uint8_t cRcv;
				uart_fifo_get_without_check(pCdc->pFifo, &cRcv);
				aOutBuf[i] = cRcv;
			}
			pCdc->bRunD2H = 1;
#if 0
			if(1 == pCdc->nEP)
			{
				uart_fifo_put(&gUartFifoSniff, ',');
				uart_fifo_put(&gUartFifoSniff, 'S');
				uart_fifo_put(&gUartFifoSniff, '0' + nTxLen / 10);
				uart_fifo_put(&gUartFifoSniff, '0' + nTxLen % 10);
			}
#endif
#if (EN_DBG == 1) // debug
			if(0 == nCdcId)
			{
				aOutBuf[0] = '#';
				uart_fifo_put(&gUartFifoB, '0' + nTxLen / 10);
				uart_fifo_put(&gUartFifoB, '0' + nTxLen % 10);
			}
#endif
			UEP_TRIG_D2H(pCdc->nEP, nTxLen);
			anTimeChk[nCdcId] = 0;
		}
		else
		{
			anTimeChk[nCdcId]++;
		}
	}

	if (pCdc->nH2DSize > 0)
	{
		uint8_t nData = aInBuf[pCdc->nNext2Uart++];
		if(0 == nCdcId)
		{
			CH554UART0SendByte(nData);
#if (EN_SNIFF == 1)
			uart_fifo_put(&gUartFifoB, nData);
#endif
		}
		else if(1 == nCdcId)
		{
			CH554UART1SendByte(nData);
		}

		if (--pCdc->nH2DSize == 0)
		{
			pCdc->nNext2Uart = 0;
			/* gstCdc0: ready, continue recving */
			UEP_DONE_H2D(pCdc->nEP);
		}
	}	
}

void CDC_Init()
{
	USB_DeviceCfg();
	USB_DeviceEndPointCfg();
	USB_DeviceIntCfg();

	UIF_SUSPEND = 0;
	UIF_TRANSFER = 0;
	UIF_BUS_RST = 0;

	ES = 1;
	PS = 1;
	IE_UART1 = 1;
	IP_EX |= bIP_UART1;
}

void CDC_Run()
{
	if(gnUsbCfg)
	{
//		mDelaymS(1);
		handle_cdc(0, &gaCdc[0]);
		handle_cdc(1, &gaCdc[1]);
	}
}
