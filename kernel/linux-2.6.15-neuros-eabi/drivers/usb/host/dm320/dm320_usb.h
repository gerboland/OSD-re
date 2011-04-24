/*
 *
 * Copyright (C) 2005-2006 Ingenient Technologies
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

/* DM320 USB definitions */

#ifndef __DM320_USB_H
#define __DM320_USB_H

/* Common */
#define EP_DIR_RX		0x00
#define EP_DIR_TX		0x20
#define EP_BUF_SINGLE		0x0000
#define EP_BUF_DOUBLE		0x0001

/* Transfer type for each endpoint */
#define CONTROL_EP		0x00	/* control transfer */
#define ISO_EP			0x01	/* isochronous transfer */
#define BULK_EP			0x02	/* bulk transfer */
#define INTR_EP			0x03	/* interrupt transfer */

/* Power flags */
#define USB_POWER_ENSUS		0x00000001
#define USB_POWER_SUSPEND	0x00000002
#define USB_POWER_RESUME	0x00000004
#define USB_POWER_RESET		0x00000008
#define USB_POWER_VBUSLO	0x00000010
#define USB_POWER_VBUSSESS	0x00000020
#define USB_POWER_VBUSVAL	0x00000040
#define USB_POWER_ISO		0x00000080

/* Interrupt flags */
#define USB_NO_INTERRUPT	0x00000000

#define USB_SUSPEND		0x00000001
#define USB_RESUME		0x00000002
#define USB_RESET		0x00000004	/* reset condition detected */
#define USB_SOF			0x00000008
#define USB_CONNECTED		0x00000010
#define USB_DISCONNECTED	0x00000020
#define USB_SESSREQ		0x00000040
#define USB_VBUSERR		0x00000080

#define USB_RXFIFO		0x00000F00
#define USB_INT_RXFIFO1		0x00000100	/* EP1 has received data */
#define USB_INT_RXFIFO2		0x00000200	/* EP2 has received data */
#define USB_INT_RXFIFO3		0x00000400
#define USB_INT_RXFIFO4		0x00000800

#define USB_CONTROL		0x00001000	/* EP0(control endpoint) has received data */

#define USB_TXFIFO		0x0001E000	/* EP1 is ready to send data */
#define USB_INT_TXFIFO1		0x00002000	/* EP2 is ready to send data */
#define USB_INT_TXFIFO2		0x00004000
#define USB_INT_TXFIFO3		0x00008000
#define USB_INT_TXFIFO4		0x00010000

#define USB_EP4_TX		0x10
#define USB_EP3_TX		0x08
#define USB_EP2_TX		0x04
#define USB_EP1_TX		0x02
#define USB_EP0			0x01	/* EP0 RX/TX */

#define USB_EP4_RX		0x10
#define USB_EP3_RX		0x08
#define USB_EP2_RX		0x04
#define USB_EP1_RX		0x02

/* Endpoint control register index */
#define USB_EP0_SELECT		0x00

/* DEVCTL register */
#define USB_DEVCTL_CID		0x80
#define USB_DEVCTL_FSDEV	0x40
#define USB_DEVCTL_LSDEV	0x20
#define USB_DEVCTL_PUCON	0x10
#define USB_DEVCTL_PDCON	0x08
#define USB_DEVCTL_MODE		0x04
#define USB_DEVCTL_HOSTREQ	0x02
#define USB_DEVCTL_SESSREQ	0x01

/* PER_CSR0 register */
#define USB_CSR0_CLRSETEND	0x80
#define USB_CSR0_CLRRXRDY	0x40
#define USB_CSR0_SENDST		0x20
#define USB_CSR0_SETEND		0x10
#define USB_CSR0_DATAEND	0x08
#define USB_CSR0_SENTST		0x04
#define USB_CSR0_TXPKTRDY	0x02
#define USB_CSR0_RXPKTRDY	0x01

/* PER_TXCSR1 register */
#define USB_TXCSR1_CLRDATTOG	0x40
#define USB_TXCSR1_SENTST	0x20
#define USB_TXCSR1_SENDST	0x10
#define USB_TXCSR1_FLFIFO	0x08
#define USB_TXCSR1_UNDERRUN	0x04
#define USB_TXCSR1_FIFOEMP	0x02
#define USB_TXCSR1_TXPKTRDY	0x01

/* CSR02 register */
#define USB_CSR2_FLFIFO		0x01

/* TXCSR2 register */
#define USB_TXCSR2_AUTOSET	0x80
#define USB_TXCSR2_ISO		0x40
#define USB_TXCSR2_MODE_TX	0x20
#define USB_TXCSR2_DMAEN	0x10
#define USB_TXCSR2_FRDATTOG	0x08
#define USB_TXCSR2_DMAMODE1	0x04

/* PER_RXCSR1 register */
#define USB_RXCSR1_CLRDATTOG	0x80
#define USB_RXCSR1_SENTST	0x40
#define USB_RXCSR1_SENDST	0x20
#define USB_RXCSR1_FLFIFO	0x10
#define USB_RXCSR1_DATERR	0x08
#define USB_RXCSR1_OVERRUN	0x04
#define USB_RXCSR1_FIFOFUL	0x02
#define USB_RXCSR1_RXPKTRDY	0x01

/* PER_RXCSR2 register */
#define USB_RXCSR2_AUTOCLR	0x80
#define USB_RXCSR2_ISO		0x40
#define USB_RXCSR2_DMAEN	0x20
#define USB_RXCSR2_DMAMODE1	0x10

/* PER TXFIFO2 register */
#define USB_TXFIFO2_SZ_8	0x00
#define USB_TXFIFO2_SZ_16	0x20
#define USB_TXFIFO2_SZ_32	0x40
#define USB_TXFIFO2_SZ_64	0x60
#define USB_TXFIFO2_SZ_128	0x80
#define USB_TXFIFO2_SZ_256	0xA0
#define USB_TXFIFO2_SZ_512	0xC0
#define USB_TXFIFO2_SZ_1024	0xE0
#define USB_TXFIFO2_SINGLE_BUF	0x00
#define USB_TXFIFO2_DOUBLE_BUF	0x10

#define USBDMA_CNTL_DMAEN	0x01
#define USBDMA_CNTL_DIR_IN	0x02
#define USBDMA_CNTL_DMAMODE1	0x04
#define USBDMA_CNTL_INTREN	0x08


#define REFCTL_SDMA1_USB	0x4000
#define REFCTL_SDMA2_USB	0x0800

/*------------------------------------- */
/*        Host Mode Definitions         */
/*------------------------------------- */
/* HST_CSR0 register */
#define USB_HST_CSR0_NAKTO		0x80
#define USB_HST_CSR0_STPKT		0x40
#define USB_HST_CSR0_REQPKT		0x20
#define USB_HST_CSR0_ERR		0x10
#define USB_HST_CSR0_SETPKT		0x08
#define USB_HST_CSR0_RXSTALL		0x04
#define USB_HST_CSR0_TXPKTRDY		0x02
#define USB_HST_CSR0_RXPKTRDY		0x01

/* HST TXCSR1 register */
#define USB_HST_TXCSR1_NAKTO		0x80
#define USB_HST_TXCSR1_CLRDATTOG	0x40
#define USB_HST_TXCSR1_RXSTALL		0x20
#define USB_HST_TXCSR1_FLFIFO		0x08
#define USB_HST_TXCSR1_ERR		0x04
#define USB_HST_TXCSR1_FIFOEMP		0x02
#define USB_HST_TXCSR1_TXPKTRDY		0x01

/* HST TXCSR2 register */
#define USB_HST_TXCSR2_AUTOSET		0x80
#define USB_HST_TXCSR2_ISO		0x40
#define USB_HST_TXCSR2_MODE_TX		0x20
#define USB_HST_TXCSR2_DMAEN		0x10
#define USB_HST_TXCSR2_FRDATTOG		0x08
#define USB_HST_TXCSR2_DMAMODE		0x04

/* HST RXCSR1 register */
#define USB_HST_RXCSR1_CLRDATTOG	0x80
#define USB_HST_RXCSR1_RXSTALL		0x40
#define USB_HST_RXCSR1_REQPKT		0x20
#define USB_HST_RXCSR1_FLFIFO		0x10
#define USB_HST_RXCSR1_DATERR		0x08
#define USB_HST_RXCSR1_ERR		0x04
#define USB_HST_RXCSR1_FIFOFUL		0x02
#define USB_HST_RXCSR1_RXPKTRDY		0x01

/* HST RXCSR2 register */
#define USB_HST_RXCSR2_DMAMODE1		0x10
#define USB_HST_RXCSR2_DMAEN		0x20
#define USB_HST_RXCSR2_AUTOREQ		0x40
#define USB_HST_RXCSR2_AUTOCLR		0x80


/* HST TXFIFO2 register */
#define USB_HST_TXFIFO2_SZ_8		0x00
#define USB_HST_TXFIFO2_SZ_16		0x20
#define USB_HST_TXFIFO2_SZ_32		0x40
#define USB_HST_TXFIFO2_SZ_64		0x60
#define USB_HST_TXFIFO2_SZ_128		0x80
#define USB_HST_TXFIFO2_SZ_256		0xA0
#define USB_HST_TXFIFO2_SZ_512		0xC0
#define USB_HST_TXFIFO2_SZ_1024		0xE0
#define USB_HST_TXFIFO2_SINGLE_BUF	0x00
#define USB_HST_TXFIFO2_DOUBLE_BUF	0x10


#define CSR_MASK_ACK		0x00000001
#define CSR_MASK_NAK		0x00000002
#define CSR_MASK_STALL		0x00000004
#define CSR_MASK_TMOUT		0x00000008
#define CSR_MASK_OVRRUN		0x00000010
#define CSR_MASK_UNDRRUN	0x00000020
#define CSR_MASK_ERR		0x00000040

#define CSR_TXRXEP0		0x1
#define CSR_RXEPS		0x2
#define CSR_TXEPS		0x3

#define CTRL_EP			0
#define DATA_EP			1

#define USB_MIN_EP0		0
#define USB_MAX_HARDWARE_EP	5   /* EP pairs */
#define USB_MAX_DEVICE_EP	16

#define EP0_FIFO_SIZE		64

#define BULK_TRANS_SIZE		512

struct port_sts {
	unsigned short Connection:1;
	unsigned short Enable:1;
	unsigned short Suspend:1;
	unsigned short OverCurrent:1;
	unsigned short Reset:1;
	unsigned short Reserved1:3;
	unsigned short Power:1;
	unsigned short LowSpeed:1;
	unsigned short HighSpeed:1;
	unsigned short Test:1;
	unsigned short Indicator:1;
	unsigned short Reserved2:3;
};

struct port_cng {
	unsigned short Connection:1;
	unsigned short Enable:1;
	unsigned short Suspend:1;
	unsigned short OverCurrent:1;
	unsigned short Reset:1;
	unsigned short Reserved:11;
};

struct port_t {
	struct port_sts p_sts;
	struct port_cng p_cng;
};
#endif
