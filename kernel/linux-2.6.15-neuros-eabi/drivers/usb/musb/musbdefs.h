/******************************************************************
 * Copyright 2005 Mentor Graphics Corporation
 * Copyright (C) 2005 by Texas Instruments
 *
 * This file is part of the Inventra Controller Driver for Linux.
 * 
 * The Inventra Controller Driver for Linux is free software; you 
 * can redistribute it and/or modify it under the terms of the GNU 
 * General Public License version 2 as published by the Free Software 
 * Foundation.
 * 
 * The Inventra Controller Driver for Linux is distributed in 
 * the hope that it will be useful, but WITHOUT ANY WARRANTY; 
 * without even the implied warranty of MERCHANTABILITY or 
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public 
 * License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with The Inventra Controller Driver for Linux ; if not, 
 * write to the Free Software Foundation, Inc., 59 Temple Place, 
 * Suite 330, Boston, MA  02111-1307  USA
 * 
 * ANY DOWNLOAD, USE, REPRODUCTION, MODIFICATION OR DISTRIBUTION 
 * OF THIS DRIVER INDICATES YOUR COMPLETE AND UNCONDITIONAL ACCEPTANCE 
 * OF THOSE TERMS.THIS DRIVER IS PROVIDED "AS IS" AND MENTOR GRAPHICS 
 * MAKES NO WARRANTIES, EXPRESS OR IMPLIED, RELATED TO THIS DRIVER.  
 * MENTOR GRAPHICS SPECIFICALLY DISCLAIMS ALL IMPLIED WARRANTIES 
 * OF MERCHANTABILITY; FITNESS FOR A PARTICULAR PURPOSE AND 
 * NON-INFRINGEMENT.  MENTOR GRAPHICS DOES NOT PROVIDE SUPPORT 
 * SERVICES OR UPDATES FOR THIS DRIVER, EVEN IF YOU ARE A MENTOR 
 * GRAPHICS SUPPORT CUSTOMER. 
 ******************************************************************/

#ifndef __MUSB_MUSBDEFS_H__
#define __MUSB_MUSBDEFS_H__

#include <linux/slab.h>
#include <linux/list.h>
#include <linux/smp_lock.h>
#include <linux/errno.h>
#include <linux/usb_ch9.h>

#include <linux/usb_musb.h>

#define	DMA_ADDR_INVALID	(~(dma_addr_t)0)

#include "debug.h"

/* Board-specific definitions (hard-wired controller locations/IRQs) */
#include "plat_cnf.h"
#include "plat_arc.h"
#include "musbhdrc.h"

#ifndef CONFIG_USB_INVENTRA_FIFO
#include "dma.h"
#define	is_dma_capable()	(1)
#else
#define	is_dma_capable()	(0)
#endif

#ifdef CONFIG_USB_MUSB_OTG
#include "otg.h"
#else
#endif

#ifdef CONFIG_PROC_FS
#include <linux/fs.h>
#define MUSB_CONFIG_PROC_FS
#define MUSB_STATISTICS
#endif

#ifdef CONFIG_USB_GADGET_MUSB_HDRC
#include <linux/usb_gadget.h>
#include "musb_gadget.h"
#define	is_peripheral_capable()	(1)
#else
#define	is_peripheral_capable()	(0)
#endif

#ifdef CONFIG_USB_MUSB_HDRC_HCD
/* for sizeof struct virtual_root */
#include "virthub.h" 
#define	is_host_capable()	(1)
#else
#define	is_host_capable()	(0)
#endif

/* NOTE:  otg and peripheral-only state machines start at B_IDLE.
 * OTG or host-only go to A_IDLE when ID is sensed.
 */
#define is_peripheral_active(m)	(is_peripheral_capable() && !(m)->bIsHost)
#define is_host_active(m)	(is_host_capable() && (m)->bIsHost)

/****************************** DEBUG CONSTANTS ********************************/

#define MGC_PAD_FRONT   0xa5deadfe
#define MGC_PAD_BACK    0xabadcafe
#define MGC_TEST_PACKET_SIZE 53

/****************************** CONSTANTS ********************************/

#if MUSB_DEBUG > 0
#define MUSB_PARANOID
#endif

#ifndef TRUE
#define TRUE 1
#endif
#ifndef FALSE
#define FALSE 0
#endif

#ifndef MUSB_C_NUM_EPS
#define MUSB_C_NUM_EPS ((u8)16)
#endif

#ifndef MUSB_MAX_END0_PACKET
#define MUSB_MAX_END0_PACKET ((u16)MGC_END0_FIFOSIZE)
#endif

/* host side ep0 states */
#define MGC_END0_START  0x0
#define MGC_END0_OUT    0x2
#define MGC_END0_IN     0x4
#define MGC_END0_STATUS 0x8

/* peripheral side ep0 states */
#define MGC_END0_STAGE_SETUP 		0x0
#define MGC_END0_STAGE_TX		0x2
#define MGC_END0_STAGE_RX		0x4
#define MGC_END0_STAGE_STATUSIN		0x8
#define MGC_END0_STAGE_STATUSOUT        0xf
#define MGC_END0_STAGE_STALL_BIT	0x10

/* failure codes */
#define MUSB_ERR_WAITING	1
#define MUSB_ERR_VBUS		-1
#define MUSB_ERR_BABBLE		-2
#define MUSB_ERR_CORRUPTED	-3
#define MUSB_ERR_IRQ		-4
#define MUSB_ERR_SHUTDOWN	-5
#define MUSB_ERR_RESTART	-6

/****************************** FUNCTIONS ********************************/

#define kzalloc(n,f) kcalloc(1,(n),(f))

#ifdef	MUSB_STATISTICS
#define	COUNT(symbol)	do{(symbol)++;}while(0)
#else
#define	COUNT(symbol)	do{}while(0)
#endif

/*************************** REGISTER ACCESS ********************************/

/* indexed vs. flat register model */

#define MUSB_FLAT_REG

#ifdef MUSB_FLAT_REG
#define MGC_SelectEnd(_pBase, _bEnd)	(((void)_pBase),((void)_bEnd))
#define MGC_ReadCsr8(_pBase, _bOffset, _bEnd) \
    MGC_Read8(_pBase, MGC_END_OFFSET(_bEnd, _bOffset))
#define MGC_ReadCsr16(_pBase, _bOffset, _bEnd) \
    MGC_Read16(_pBase, MGC_END_OFFSET(_bEnd, _bOffset))
#define MGC_WriteCsr8(_pBase, _bOffset, _bEnd, _bData) \
    MGC_Write8(_pBase, MGC_END_OFFSET(_bEnd, _bOffset), _bData)
#define MGC_WriteCsr16(_pBase, _bOffset, _bEnd, _bData) \
    MGC_Write16(_pBase, MGC_END_OFFSET(_bEnd, _bOffset), _bData)
#else
#define MGC_SelectEnd(_pBase, _bEnd) \
    MGC_Write8(_pBase, MGC_O_HDRC_INDEX, _bEnd)
#define MGC_ReadCsr8(_pBase, _bOffset, _bEnd) \
    MGC_Read8(_pBase, (_bOffset + 0x10))
#define MGC_ReadCsr16(_pBase, _bOffset, _bEnd) \
    MGC_Read16(_pBase, (_bOffset + 0x10))
#define MGC_WriteCsr8(_pBase, _bOffset, _bEnd, _bData) \
    MGC_Write8(_pBase, (_bOffset + 0x10), _bData)
#define MGC_WriteCsr16(_pBase, _bOffset, _bEnd, _bData) \
    MGC_Write16(_pBase, (_bOffset + 0x10), _bData)
#endif

/****************************** FUNCTIONS ********************************/

#define MUSB_HST_MODE(_pthis) { (_pthis)->bIsHost=TRUE; (_pthis)->bIsDevice=FALSE; \
	(_pthis)->bFailCode=0; }
#define MUSB_DEV_MODE(_pthis) { (_pthis)->bIsHost=FALSE; (_pthis)->bIsDevice=TRUE; \
	(_pthis)->bFailCode=0; }
#define MUSB_OTG_MODE(_pthis) { (_pthis)->bIsHost=FALSE; (_pthis)->bIsDevice=FALSE; \
	(_pthis)->bFailCode=MUSB_ERR_WAITING; }
#define MUSB_ERR_MODE(_pthis, _cause) { (_pthis)->bIsHost=FALSE; (_pthis)->bIsDevice=FALSE; \
	(_pthis)->bFailCode=_cause; }

#define MUSB_IS_ERR(_x) ( (_x)->bFailCode<0 )
#define MUSB_IS_HST(_x) ( !MUSB_IS_ERR(_x) && (_x)->bIsHost && !(_x)->bIsDevice )
#define MUSB_IS_DEV(_x) ( !MUSB_IS_ERR(_x) && !(_x)->bIsHost && (_x)->bIsDevice )
#define MUSB_IS_OTG(_x) ( !MUSB_IS_ERR(_x) && !(_x)->bIsHost && !(_x)->bIsDevice )

#define test_devctl_hst_mode(_x) (MGC_Read8((_x)->pRegs, MGC_O_HDRC_DEVCTL)&MGC_M_DEVCTL_HM)

#define MUSB_MODE(_x) ( MUSB_IS_HST(_x)?"HOST":( MUSB_IS_DEV(_x)?"FUNCTION":(MUSB_IS_OTG(_x)?"OTG":"ERROR")) )

#ifdef CONFIG_USB_TI_CPPI_DMA
#define WANTS_DMA(_pUrb) ((_pUrb)->transfer_dma)
#define DMA_BUFFER(_pUrb) ((_pUrb)->transfer_dma)
#endif

/******************************** DMA TYPES **********************************/

#ifdef CONFIG_USB_INVENTRA_DMA
#include "dma.h"

#ifndef MGC_HSDMA_CHANNELS
#define MGC_HSDMA_CHANNELS 8
#endif

#ifdef MUSB_HAS_DMA_URBS
#define WANTS_DMA(_pUrb) ((_pUrb)->transfer_dma)
#define DMA_BUFFER(_pUrb) ((_pUrb)->transfer_dma)
#else
#define WANTS_DMA(_pUrb) (0)
#define DMA_BUFFER(pUrb) ((void*)0x000666)
#endif

extern MGC_DmaControllerFactory MGC_HdrcDmaControllerFactory;
#endif

/************************** Ep Configuration ********************************/

/** The End point descriptor */
struct MUSB_EpFifoDescriptor {
	u8 bType;		/* 0 for autoconfig, CNTR, ISOC, BULK, INTR */
	u8 bDir;		/* 0 for autoconfig, INOUT, IN, OUT */
	int wSize;		/* 0 for autoconfig, or the size */
};

#define MUSB_EPD_AUTOCONFIG	0

#define MUSB_EPD_T_CNTRL	1
#define MUSB_EPD_T_ISOC		2
#define MUSB_EPD_T_BULK		3
#define MUSB_EPD_T_INTR		4

#define MUSB_EPD_D_INOUT	0
#define MUSB_EPD_D_TX		1
#define MUSB_EPD_D_RX		2

/******************************** TYPES *************************************/

extern const u8 MGC_aTestPacket[MGC_TEST_PACKET_SIZE];

/**
 * struct musb_hw_ep - endpoint hardware (bidirectional)
 * @field Lock spinlock
 * @field pUrb current URB
 * @field urb_list list
 * @field dwOffset current buffer offset
 * @field dwRequestSize how many bytes were last requested to move
 * @field wMaxPacketSizeTx local Tx FIFO size
 * @field wMaxPacketSizeRx local Rx FIFO size
 * @field wPacketSize programmed packet size
 * @field bIsSharedFifo TRUE if FIFO is shared between Tx and Rx
 * @field bAddress programmed bus address
 * @field bEnd programmed remote endpoint address
 * @field bTrafficType programmed traffic type
 * @field bIsClaimed TRUE if claimed
 * @field bIsTx TRUE if current direction is Tx
 * @field bIsReady TRUE if ready (available for new URB)
 */
struct musb_hw_ep {
#if MUSB_DEBUG > 0
	u32 dwPadFront;
#endif
	spinlock_t		Lock;
	struct musb		*musb;

#if 1	/*host*/
	/* host side */
	struct list_head	urb_list;
	u8			bTrafficType;
	u8			bIsClaimed;

	//u8			out_traffic_type;
	//u8			in_traffic_type;
	//struct list_head	out_urb_list;		/* TX */
	//struct list_head	in_urb_list;		/* RX */
#endif

#if 0
	/* peripheral side */
	struct musb_ep		ep_in;			/* TX */
	struct musb_ep		ep_out;			/* RX */
#endif

	/* hardware configuration, possibly dynamic */
	u16			wMaxPacketSizeTx;
	u16			wMaxPacketSizeRx;
	u8			bIsSharedFifo;

#ifdef	MUSB_STATISTICS
	unsigned long		dwTotalTxBytes;
	unsigned long		dwTotalRxBytes;
	unsigned long		dwTotalTxPackets;
	unsigned long		dwTotalRxPackets;
	unsigned long		dwErrorTxPackets;
	unsigned long		dwErrorRxPackets;
	unsigned long		dwMissedTxPackets;
	unsigned long		dwMissedRxPackets;
#endif



	/* REVISIT sort these fields too */

	unsigned int dwOffset;
	unsigned int dwRequestSize;
	unsigned int dwIsoPacket;
	unsigned int dwWaitFrame;

#if defined(CONFIG_USB_INVENTRA_DMA) || defined(CONFIG_USB_TI_CPPI_DMA)
	MGC_DmaChannel *pDmaChannel;
#endif

	u16 wPacketSize;
	u8 bAddress;
	u8 bEnd;
	u8 bIsTx;
	u8 bIsReady;
	u8 bRetries;
	u8 bLocalEnd;

	u8 bDisableDma;		/* not used now! */

	/* added for triggering DMAReqEnable with CPPI */
	u8 bEnableDmaReq;

#if MUSB_DEBUG > 0
	u32 dwPadBack;
#endif
};
typedef struct musb_hw_ep MGC_LinuxLocalEnd;

/** A listener for disconnection */
typedef void (*MGC_pfDisconnectListener) (void *);
/** A handler for the default endpoint interrupt */
typedef void (*MGC_pfDefaultEndHandler) (void *);

/**
 * struct musb - Driver instance data.
 * @field Lock spinlock
 * @field Timer interval timer for various things
 * @field pBus pointer to Linux USBD bus
 * @field RootHub virtual root hub
 * @field PortServices services provided to virtual root hub
 * @field pRootDevice root device pointer, to track connection speed
 * @field nIrq IRQ number (needed by free_irq)
 * @field bIsMultipoint TRUE if multi-point core
 * @field bIsDevice TRUE if peripheral (false if host)
 * @field pRegs pointer to mapped registers
 */
struct musb {
#if MUSB_DEBUG > 0
	u32 dwPadFront;
#endif
	spinlock_t Lock;
	struct timer_list Timer;
	struct usb_bus *pBus;
	char aName[32];

#ifdef CONFIG_USB_MUSB_HDRC_HCD
	struct virtual_root	RootHub;
	struct port_services	PortServices;
	struct usb_device	*pRootDevice;
#endif

#if defined(CONFIG_USB_INVENTRA_DMA) || defined(CONFIG_USB_TI_CPPI_DMA)
	MGC_DmaController *pDmaController;
#endif

	struct device *controller;

	int nIrq;
	int nBabbleCount;
	void __iomem *pRegs;

	MGC_LinuxLocalEnd aLocalEnd[MUSB_C_NUM_EPS];

	u16 wEndMask;
	u8 bEndCount;
	u8 bRootSpeed;
	u8 board_mode;		/* enum musb_mode */

	int bFailCode;		/* one of MUSB_ERR_* failure code */

	u8 bEnd0Stage;		/* end0 stage while in host or device mode */
	u8 bBulkTxEnd;
	u8 bBulkRxEnd;

	unsigned bIsMultipoint:1;
	unsigned bIsDevice:1;
	unsigned bIsHost:1;
	unsigned bIgnoreDisconnect:1;	/* during bus resets, fake disconnects */
	unsigned bBulkSplit;
	unsigned bBulkCombine:1;

#ifdef CONFIG_USB_GADGET_MUSB_HDRC
	unsigned bIsSelfPowered:1;
	unsigned bMayWakeup:1;
	unsigned bSetAddress:1;
	unsigned bTestMode:1;

	u8 bAddress;
	u8 bTestModeValue;

	struct usb_gadget g;	/* the gadget */
	struct usb_gadget_driver *pGadgetDriver;	/* its driver */
#endif

#ifdef CONFIG_USB_MUSB_OTG
	MGC_OtgMachine OtgMachine;
	MGC_OtgServices OtgServices;
	u8 bDelayPortPowerOff;
	u8 bOtgError;
#endif

#if MUSB_DEBUG > 0
	u32 dwPadBack;
#endif

#ifdef MUSB_CONFIG_PROC_FS
	struct proc_dir_entry *pProcEntry;

	/* A couple of hooks to enable HSET */
	MGC_pfDisconnectListener pfDisconnectListener;
	void *pDisconnectListenerParam;
	MGC_pfDefaultEndHandler pfDefaultEndHandler;
	void *pDefaultEndHandlerParam;
#endif

};

#ifdef CONFIG_USB_GADGET_MUSB_HDRC
static inline struct musb *gadget_to_musb(struct usb_gadget *g)
{
	return container_of(g, struct musb, g);
}
#endif

typedef struct musb MGC_LinuxCd;

/***************************** Glue it together *****************************/

extern const char musb_driver_name[];

extern void MGC_HdrcStart(struct musb *pThis);
extern void MGC_HdrcStop(struct musb *pThis);
extern void MGC_HdrcServiceUsb(struct musb *pThis, u8 reg);
extern void MGC_HdrcLoadFifo(const u8 * pBase, u8 bEnd,
			     u16 wCount, const u8 * pSource);
extern void MGC_HdrcUnloadFifo(const u8 * pBase, u8 bEnd,
			       u16 wCount, u8 * pDest);

extern unsigned int MGC_nIndex;
extern void MGC_LinuxSetTimer(struct musb *pThis,
			      void (*pfFunc) (unsigned long),
			      unsigned long pParam, unsigned long millisecs);

extern int queue_length(struct list_head *lh);

/* Conditionally-compiled to update OTG state machine when necessary */
extern void MGC_OtgUpdate(struct musb *pThis, u8 bVbusError, u8 bConnect);

/*-------------------------- ProcFS definitions ---------------------*/

struct MGC_TestProcData;
struct proc_dir_entry;

#ifdef MUSB_CONFIG_PROC_FS
extern struct proc_dir_entry *MGC_LinuxCreateProcFs(char *name,
						    struct musb *data);
extern void MGC_LinuxDeleteProcFs(struct musb *data);

#else
static inline struct proc_dir_entry *MGC_LinuxCreateProcFs(char *name,
							   struct musb *data)
{
}
static inline void MGC_LinuxDeleteProcFs(struct musb *data)
{
}
#endif

/*------------------------------ IOCTLS/PROCFS -----------------------*/

extern void MGC_Zap(struct musb *pThis);
extern void MGC_Session(struct musb *pThis);

/*-------------------------- DEBUG Definitions ---------------------*/

#ifdef MUSB_PARANOID
#define MGC_HDRC_DUMPREGS(_t, _s) \
	MGC_HdrcDumpRegs((_t)->pRegs, MUSB_IS_HST(_t) && _t->bIsMultipoint, _s)
#define MGC_ISCORRUPT(_x)	mgc_is_corrupt((_x), __FUNCTION__,__LINE__)

extern int mgc_is_corrupt(struct musb *, const char *f, int line);

#else
#define MGG_IsCorrupt(_x)	(_x)
#define MGC_HDRC_DUMPREGS(_t, _s)
#endif

/* -------------------------- Gadget Definitions --------------------- */

struct usb_ep;

void *MGC_AllocBufferMemory(struct musb *pThis, size_t bytes, int gfp_flags,
			    dma_addr_t * dma);
void MGC_FreeBufferMemory(struct musb *pThis, size_t bytes, void *address,
			  dma_addr_t dma);

#ifdef CONFIG_USB_GADGET_MUSB_HDRC
extern void *MGC_MallocEp0Buffer(const struct musb *pThis);
extern void MGC_InitGadgetEndPoints(struct musb *pThis);

extern void MGC_HdrcServiceDeviceDefaultEnd(struct musb *pThis);
extern void MGC_HdrcServiceDeviceTxAvail(struct musb *pThis, u8 bEnd);
extern void MGC_HdrcServiceDeviceRxReady(struct musb *pThis, u8 bEnd);

#endif

#endif	/* __MUSB_MUSBDEFS_H__ */
