/******************************************************************
 *                                                                *
 *        Copyright Mentor Graphics Corporation 2004              *
 *                                                                *
 *                All Rights Reserved.                            *
 *                                                                *
 *    THIS WORK CONTAINS TRADE SECRET AND PROPRIETARY INFORMATION *
 *  WHICH IS THE PROPERTY OF MENTOR GRAPHICS CORPORATION OR ITS   *
 *  LICENSORS AND IS SUBJECT TO LICENSE TERMS.                    *
 *                                                                *
 ******************************************************************/

#ifndef __MUSB_HDRDF_H__
#define __MUSB_HDRDF_H__

/*
 * DaVinci-specific definitions
 */

/* Integrated highspeed/otg PHY */
#define	USBPHY_CTL_PADDR	0x01c40034
#define	USBPHY_PHYCLKGD		(1 << 8)
#define	USBPHY_SESNDEN		(1 << 7)	/* v(sess_end) comparator */
#define	USBPHY_VBDTCTEN		(1 << 6)	/* v(bus) comparator */
#define	USBPHY_PHYPLLON		(1 << 4)	/* override pll suspend */
#define	USBPHY_CLK01SEL		(1 << 3)
#define	USBPHY_OSCPDWN		(1 << 2)
#define	USBPHY_PHYSPDWN		(1 << 0)

/* For now include usb OTG module registers here */
#define USB_OTG_VERSION_REG		0x00
#define USB_OTG_CTRL_REG		0x04
#define USB_OTG_STAT_REG		0x08
#define USB_OTG_RNDIS_REG		0x10
#define USB_OTG_AUTOREQ_REG		0x14
#define USB_OTG_INT_SOURCE_REG		0x20
#define USB_OTG_INT_SET_REG		0x24
#define USB_OTG_INT_SRC_CLR_REG		0x28
#define USB_OTG_INT_MASK_REG		0x2c
#define USB_OTG_INT_MASK_SET_REG	0x30
#define USB_OTG_INT_MASK_CLR_REG	0x34
#define USB_OTG_INT_SRC_MASKED_REG	0x38
#define USB_OTG_EOI_REG			0x3c
#define USB_OTG_EOI_INTVEC		0x40

/* CPPI related registers */
#define USB_OTG_TXCPPI_CTRL_REG         0x80
#define USB_OTG_TXCPPI_TEAR_REG        0x84
#define USB_OTG_CPPI_EOI_REG          	    0x88
#define USB_OTG_CPPI_INTVEC_REG         0x8c
#define USB_OTG_TXCPPI_MASKED_REG  0x90
#define USB_OTG_TXCPPI_RAW_REG        0x94
#define USB_OTG_TXCPPI_INTENAB_REG 0x98
#define USB_OTG_TXCPPI_INTCLR_REG    0x9c

#define USB_OTG_RXCPPI_CTRL_REG         0xC0
#define USB_OTG_RXCPPI_MASKED_REG   0xD0
#define USB_OTG_RXCPPI_RAW_REG         0xD4
#define USB_OTG_RXCPPI_INTENAB_REG  0xD8
#define USB_OTG_RXCPPI_INTCLR_REG     0xDC

#define USB_OTG_RXCPPI_BUFCNT0_REG   0xE0
#define USB_OTG_RXCPPI_BUFCNT1_REG   0xE4
#define USB_OTG_RXCPPI_BUFCNT2_REG   0xE8
#define USB_OTG_RXCPPI_BUFCNT3_REG   0xEC

/* CPPI state RAM entries */
#define USB_OTG_CPPI_STATERAM_BASE_OFFSET   0x100

#define TXCPPI_STATERAM_OFFSET(channelNum)   ((u32)(USB_OTG_CPPI_STATERAM_BASE_OFFSET + ((channelNum)* 0x40) ))
#define RXCPPI_STATERAM_OFFSET(channelNum)   ((u32)(USB_OTG_CPPI_STATERAM_BASE_OFFSET + 0x20+((channelNum)* 0x40) ) )

/* CPPI masks */
#define DMA_CTRL_ENABLE   1
#define DMA_CTRL_DISABLE  0

#define DMA_ALL_CHANNELS_ENABLE  0xF
#define DMA_ALL_CHANNELS_DISABLE 0xF

/* REVISIT relying on "volatile" here is wrong ... */

/* define structure overalys for Rx/Tx stateRam entries */
typedef struct _txStateRam {
	volatile u32 headPtr;
	volatile u32 sopDescPtr;
	volatile u32 currDescPtr;
	volatile u32 currBuffPtr;
	volatile u32 flags;
	volatile u32 remLength;
	volatile u32 dummy;
	volatile u32 completionPtr;
} TxCppiStateRam;

typedef struct _rxStateRam {
	volatile u32 buffOffset;
	volatile u32 headPtr;
	volatile u32 sopDescPtr;
	volatile u32 currDescPtr;
	volatile u32 currBuffPtr;
	volatile u32 pktLength;
	volatile u32 byteCount;
	volatile u32 completionPtr;
} RxCppiStateRam;

#define USB_OTG_USBINT_MASK	0x01ff0000	/* 8 Mentor, + DRVVBUS */
#define USB_OTG_TXINT_MASK	0x0000001F
#define USB_OTG_RXINT_MASK	0x00001E00
#define USB_OTG_TX_ENDPTS_MASK	0x1f
#define USB_OTG_RX_ENDPTS_MASK	0x1f

#define USB_OTG_USBINT_SHIFT  16
#define USB_OTG_TXINT_SHIFT    0
#define USB_OTG_RXINT_SHIFT    8

#define MENTOR_BASE_OFFSET 0x400

#endif	/* __MUSB_HDRDF_H__ */
