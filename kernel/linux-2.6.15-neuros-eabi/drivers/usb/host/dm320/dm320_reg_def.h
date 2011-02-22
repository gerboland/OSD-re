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


//==================================================================================================
//
//	dm320_reg_def.h
//
//		Description		:	DM320 register definitions for USB driver extension
//		Author			:	Kwang-Seok Kwak
//		Copyright		:	Ingenient Technology Inc.
//		Created date	:	04/04/2005
//		Date modified	:	04/20/2005 
//
//		[NOTE]			: 	This header file could be removed 
//										if these definitions are moved into kernel source tree
//							Check <asm/arch/hardware.h> file to avoid collision
//
//=================================================================================================



#ifndef		_LINUX_DD_DM320_REG_DEF_H_
#define		_LINUX_DD_DM320_REG_DEF_H_



//------------------------------------------------------------------------------
// Include files

#include	<asm/arch/hardware.h>



//------------------------------------------------------------------------------
// General definitions

// AHB Bus Controller (AHBBUSC) Register Definition	
// In Linux, AHB_BASE is 0xE0035000 ( see include/asm/arch/hardware.h )
#define		AHB_SDRAMSA			( AHB_BASE + 0xF00 )	// 0x00060F00 : SDRAM start address
#define		AHB_SDRAMEA         ( AHB_BASE + 0xF04 )	// 0x00060F04 : SDRAM end address
#define		AHB_BUSCONTROL      ( AHB_BASE + 0xF08 )	// 0x00060F08 : Bus endianess control
#define		AHB_RSV1            ( AHB_BASE + 0xF0C )	// 0x00060F0C : Reserved
#define		AHB_USBCTL          ( AHB_BASE + 0xF10 )	// 0x00060F10 : USB control register on ES1.1


// USB Controller Register Definition 
// In Linux, USB_BASE is 0xE6000000
//#define USB_BASE  0x80000000
// ( Modify include/asm/arch/hardware.h & arch/arm/mach-itdm320-20/core.c files in linux kernel )
#define		USB_FADDR			( USB_BASE + 0x00 )		// 0x80000000 : Peripheral Address Register 				
#define		USB_POWER			( USB_BASE + 0x01 )		// 0x80000001 : Power Control Register
#define		USB_INTRTX1			( USB_BASE + 0x02 )		// 0x80000002 : Transmit EP Interrupt Status Register #1
#define		USB_RSV1			( USB_BASE + 0x03 )		// 0x80000003 : Reserved
#define		USB_INTRRX1			( USB_BASE + 0x04 )		// 0x80000004 : Receive EP Interrupt Status Register #1
#define		USB_RSV2			( USB_BASE + 0x05 )		// 0x80000005 : Reserved
#define		USB_INTRUSB			( USB_BASE + 0x06 )		// 0x80000006 : USB Interrupt Status Register
#define		USB_INTRTX1E		( USB_BASE + 0x07 )		// 0x80000007 : Transmit EP Interrupt Enable Register #1
#define		USB_RSV3			( USB_BASE + 0x08 )		// 0x80000008 : Reserved
#define		USB_INTRRX1E		( USB_BASE + 0x09 )		// 0x80000009 : Receive EP Interrupt Enable Register #1
#define		USB_RSV4			( USB_BASE + 0x0A )		// 0x8000000A : Reserved
#define		USB_INTRUSBE		( USB_BASE + 0x0B )		// 0x8000000B : USB Interrupt Enable Register
#define		USB_FRAME1			( USB_BASE + 0x0C )		// 0x8000000C : Lower Frame Number Register
#define		USB_FRAME2			( USB_BASE + 0x0D )		// 0x8000000D : Upper Frame Number Register
#define		USB_INDEX			( USB_BASE + 0x0E )		// 0x8000000E : Endpoint index Register
#define		USB_DEVCTL			( USB_BASE + 0x0F )		// 0x8000000F : Device Control Register
#define		USB_TXMAXP			( USB_BASE + 0x10 )		// 0x80000010 : Transmit Maximum Packet Size Register
#define		USB_PER_CSR0		( USB_BASE + 0x11 )		// 0x80000011 : Peripheral EP0 Control Register
#define		USB_PER_TXCSR1		( USB_BASE + 0x11 )		// 0x80000011 : Peripheral Tx EP Control Register #1
#define		USB_CSR2			( USB_BASE + 0x12 )		// 0x80000012 : EP0 Control Register #2
#define		USB_TXCSR2			( USB_BASE + 0x12 )		// 0x80000012 : Tx EP Control Register #2
#define		USB_RXMAXP			( USB_BASE + 0x13 )		// 0x80000013 : Receive Maximum Packet Size Register
#define		USB_PER_RXCSR1		( USB_BASE + 0x14 )		// 0x80000014 : Peripheral Rx EP Control Register #1
#define		USB_PER_RXCSR2		( USB_BASE + 0x15 )		// 0x80000015 : Peripheral Rx EP Control Register #2
#define		USB_COUNT0			( USB_BASE + 0x16 )		// 0x80000016 : Count EP0 Data Bytes Register
#define		USB_RXCOUNT1		( USB_BASE + 0x16 )		// 0x80000016 : Count EP Data bytes Register #1
#define		USB_RXCOUNT2		( USB_BASE + 0x17 )		// 0x80000017 : Count EP Data bytes Register #2
#define		USB_TXTYPE			( USB_BASE + 0x18 )		// 0x80000018 : EP Transmit Type Register
#define		USB_NAKLMT0			( USB_BASE + 0x19 )		// 0x80000019 : EP0 NAK Limit Register
#define		USB_TXINTVL			( USB_BASE + 0x19 )		// 0x80000019 : EP Transmit Interval Register
#define		USB_RXTYPE			( USB_BASE + 0x1A )		// 0x8000001A : EP Receive Type Register
#define		USB_RXINTVL			( USB_BASE + 0x1B )		// 0x8000001B : EP Receive Interval Register
#define		USB_TXFIFO1			( USB_BASE + 0x1C )		// 0x8000001C : EP Transmit FIFO Address Register #1
#define		USB_TXFIFO2			( USB_BASE + 0x1D )		// 0x8000001D : EP Transmit FIFO Address Register #2
#define		USB_RXFIFO1			( USB_BASE + 0x1E )		// 0x8000001E : EP Receive FIFO Address Register #1
#define		USB_RXFIFO2			( USB_BASE + 0x1F )		// 0x8000001F : EP Receive FIFO Address Register #2
#define		USB_HST_CSR0		( USB_BASE + 0x11 )		// 0x80000011 : Host EP0 Control Register
#define		USB_HST_TXCSR1		( USB_BASE + 0x11 )		// 0x80000011 : Host Tx EP Control Register #1
#define		USB_HST_TXCSR2		( USB_BASE + 0x12 )		// 0x80000011 : Host Tx EP Control Register #2
#define		USB_HST_RXCSR1		( USB_BASE + 0x14 )		// 0x80000014 : Host Rx EP Control Register #1
#define		USB_HST_RXCSR2		( USB_BASE + 0x15 )		// 0x80000015 : Host Rx EP Control Register #2
#define		USB_FIFO0			( USB_BASE + 0x20 )		// 0x80000020 : EP0 FIFO Access Register
#define		USB_FIFO1			( USB_BASE + 0x24 )		// 0x80000024 : EP1 FIFO Access Register
#define		USB_FIFO2			( USB_BASE + 0x28 )		// 0x80000028 : EP2 FIFO Access Register
#define		USB_FIFO3			( USB_BASE + 0x2C )		// 0x8000002C : EP3 FIFO Access Register
#define		USB_FIFO4			( USB_BASE + 0x30 )		// 0x80000030 : EP4 FIFO Access Register


// USB DMA (USBDMA) Register Definition	=> Kernel Virtual Address										
#define		USBDMA_INTR	    ( USB_BASE + 0x200 )	// 0x80000200 : Interrupt Status Register   
#define		USBDMA_CNTL1        ( USB_BASE + 0x204 )	// 0x80000204 : DMA Channel1 Control Register
#define		USBDMA_ADDR1        ( USB_BASE + 0x208 )	// 0x80000208 : DMA Channel1 Address Register
#define		USBDMA_COUNT1       ( USB_BASE + 0x20C )	// 0x8000020C : DMA Channel1 Byte Count Register
#define		USBDMA_CNTL2        ( USB_BASE + 0x214 )	// 0x80000214 : DMA Channel2 Control Register
#define		USBDMA_ADDR2        ( USB_BASE + 0x218 )	// 0x80000218 : DMA Channel2 Address Register
#define		USBDMA_COUNT2       ( USB_BASE + 0x21C )	// 0x8000021C : DMA Channel2 Byte Count Register
#define		USBDMA_CNTL3        ( USB_BASE + 0x224 )	// 0x80000224 : DMA Channel3 Control Register
#define		USBDMA_ADDR3        ( USB_BASE + 0x228 )	// 0x80000228 : DMA Channel3 Address Register
#define		USBDMA_COUNT3       ( USB_BASE + 0x22C )	// 0x8000022C : DMA Channel3 Byte Count Register
#define		USBDMA_CNTL4        ( USB_BASE + 0x234 )	// 0x80000234 : DMA Channel4 Control Register
#define		USBDMA_ADDR4        ( USB_BASE + 0x238 )	// 0x80000238 : DMA Channel4 Address Register
#define		USBDMA_COUNT4       ( USB_BASE + 0x23C )	// 0x8000023C : DMA Channel4 Byte Count Register



#endif
















