
//=================================================================================================
//
//	dm320_usb.h
//
//		Description		:	DM320 USB definitions
//		Author			:	Kwang-Seok Kwak
//		Copyright		:	Ingenient Technology Inc.
//		Created date	:	04/04/2005
//		Date modified	:	04/20/2005 
//
//=================================================================================================



#ifndef 	_LINUX_DD_DM320_USB_H_
#define 	_LINUX_DD_DM320_USB_H_



//-----------------------------------------------------------------------------
// Defines



// Common
#define 	EP_DIR_RX		 		0x00
#define 	EP_DIR_TX				0x20
#define 	EP_BUF_SINGLE       	0x0000
#define 	EP_BUF_DOUBLE    	    0x0001

// Transfer type for each endpoint
#define		CONTROL_EP				0x00				// control transfer		
#define		ISO_EP					0x01				// isochronous transfer
#define		BULK_EP					0x02				// bulk transfer
#define		INTR_EP					0x03				// interrupt transfer

//Power flags
#define		USB_POWER_ENSUS			0x00000001
#define		USB_POWER_SUSPEND		0x00000002
#define 	USB_POWER_RESUME		0x00000004
#define 	USB_POWER_RESET			0x00000008
#define		USB_POWER_VBUSLO		0x00000010
#define 	USB_POWER_VBUSSESS		0x00000020
#define 	USB_POWER_VBUSVAL		0x00000040
#define 	USB_POWER_ISO			0x00000080




// Interrupt flags
#define 	USB_NO_INTERRUPT        0x00000000

#define 	USB_SUSPEND             0x00000001
#define 	USB_RESUME              0x00000002
#define 	USB_RESET               0x00000004			// reset condition detected
#define 	USB_SOF                 0x00000008
#define 	USB_CONNECTED           0x00000010
#define 	USB_DISCONNECTED        0x00000020
#define 	USB_SESSREQ             0x00000040
#define 	USB_VBUSERR             0x00000080

#define 	USB_RXFIFO              0x00000F00
#define 	USB_INT_RXFIFO1         0x00000100			// EP1 has received data
#define 	USB_INT_RXFIFO2         0x00000200			// EP2 has received data
#define 	USB_INT_RXFIFO3         0x00000400
#define 	USB_INT_RXFIFO4         0x00000800

#define 	USB_CONTROL             0x00001000			// EP0(control endpoint) has received data

#define 	USB_TXFIFO              0x0001E000			// EP1 is ready to send data
#define 	USB_INT_TXFIFO1         0x00002000			// EP2 is ready to send data
#define 	USB_INT_TXFIFO2         0x00004000
#define 	USB_INT_TXFIFO3         0x00008000
#define 	USB_INT_TXFIFO4         0x00010000

#define 	USB_EP4_TX              0x10
#define 	USB_EP3_TX              0x08
#define 	USB_EP2_TX              0x04
#define 	USB_EP1_TX              0x02
#define 	USB_EP0                 0x01				// EP0 RX/TX

#define 	USB_EP4_RX              0x10
#define 	USB_EP3_RX              0x08
#define 	USB_EP2_RX              0x04
#define 	USB_EP1_RX              0x02

// Endpoint control register index
#define 	USB_EP0_SELECT          0x00

// DEVCTL register
#define 	USB_DEVCTL_CID          0x80
#define 	USB_DEVCTL_FSDEV        0x40
#define 	USB_DEVCTL_LSDEV        0x20
#define 	USB_DEVCTL_PUCON        0x10
#define 	USB_DEVCTL_PDCON        0x08
#define 	USB_DEVCTL_MODE         0x04
#define 	USB_DEVCTL_HOSTREQ      0x02
#define 	USB_DEVCTL_SESSREQ      0x01

// PER_CSR0 register
#define 	USB_CSR0_CLRSETEND      0x80
#define 	USB_CSR0_CLRRXRDY       0x40
#define 	USB_CSR0_SENDST         0x20
#define 	USB_CSR0_SETEND         0x10
#define 	USB_CSR0_DATAEND        0x08
#define 	USB_CSR0_SENTST         0x04
#define 	USB_CSR0_TXPKTRDY       0x02
#define 	USB_CSR0_RXPKTRDY       0x01

// PER_TXCSR1 register
#define 	USB_TXCSR1_CLRDATTOG    0x40
#define 	USB_TXCSR1_SENTST       0x20
#define 	USB_TXCSR1_SENDST       0x10
#define 	USB_TXCSR1_FLFIFO       0x08
#define 	USB_TXCSR1_UNDERRUN     0x04
#define 	USB_TXCSR1_FIFOEMP      0x02
#define 	USB_TXCSR1_TXPKTRDY     0x01

// CSR02 register
#define 	USB_CSR2_FLFIFO        	0x01

// TXCSR2 register
#define 	USB_TXCSR2_AUTOSET      0x80
#define 	USB_TXCSR2_ISO          0x40
#define 	USB_TXCSR2_MODE_TX      0x20
#define 	USB_TXCSR2_DMAEN        0x10
#define 	USB_TXCSR2_FRDATTOG     0x08
#define 	USB_TXCSR2_DMAMODE1     0x04

// PER_RXCSR1 register
#define 	USB_RXCSR1_CLRDATTOG    0x80
#define 	USB_RXCSR1_SENTST       0x40
#define 	USB_RXCSR1_SENDST       0x20
#define 	USB_RXCSR1_FLFIFO       0x10
#define 	USB_RXCSR1_DATERR       0x08
#define 	USB_RXCSR1_OVERRUN      0x04
#define 	USB_RXCSR1_FIFOFUL      0x02
#define 	USB_RXCSR1_RXPKTRDY     0x01

// PER_RXCSR2 register
#define 	USB_RXCSR2_AUTOCLR      0x80
#define 	USB_RXCSR2_ISO          0x40
#define 	USB_RXCSR2_DMAEN        0x20
#define	 	USB_RXCSR2_DMAMODE1     0x10

// PER TXFIFO2 register
#define		USB_TXFIFO2_SZ_8		0x00
#define		USB_TXFIFO2_SZ_16		0x20
#define		USB_TXFIFO2_SZ_32		0x40
#define		USB_TXFIFO2_SZ_64		0x60
#define		USB_TXFIFO2_SZ_128		0x80
#define		USB_TXFIFO2_SZ_256		0xA0
#define		USB_TXFIFO2_SZ_512		0xC0
#define		USB_TXFIFO2_SZ_1024		0xE0
#define		USB_TXFIFO2_SINGLE_BUF	0x00
#define		USB_TXFIFO2_DOUBLE_BUF	0x10

#define		USBDMA_CNTL_DMAEN		0x01
#define		USBDMA_CNTL_DIR_IN		0x02
#define		USBDMA_CNTL_DMAMODE1	0x04
#define		USBDMA_CNTL_INTREN		0x08


#define		REFCTL_SDMA1_USB		0x4000
#define		REFCTL_SDMA2_USB		0x0800


//-----------------------------------------------------------------------------
// Structures

#if 0
typedef struct usbEndpointInfo
{
	char			fAddr;
	unsigned short	nSize;
	char			fBufMode;
	char			fTransMode;
	
}	USB_EP_INFO;



//-----------------------------------------------------------------------------
// Public function prototypes

void		USB_DMAInit				( unsigned char fDmaCh, unsigned char fEP, unsigned char fDir, unsigned char fMode, 
													uint32_t nAddr, uint32_t nBufSize, uint32_t nMaxTrans ); 
void		USB_EnableDMA			( unsigned char fDmaCh, unsigned char fEnable );
void		USB_ExitDMA				( unsigned char fDmaCh );

uint8_t		USB_GetPeripheralAddr	( void );
void		USB_SetPeripheralAddr	( unsigned char ucNewAddress );
int			USB_CheckConn			( void );
int 		USB_ReadIntrRegs		( void );
int 		USB_GetIntrByPriority	( int iInterruptFlags );

int    		USB_ReadEP				( int iEndpoint, void *pvBuffer, int iSize );
void		USB_SetCSR0Reg			( unsigned char fFlag );
int 		USB_WriteEP				( int iEndpoint, void *pvBuffer, int iSize );

void  		USB_SetAddrReg			( unsigned char ucAddress );
int   		USB_SetTx0End			( void );
void 		USB_FlushFifo			( int iEndpoint );

#endif


#endif












