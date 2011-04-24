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

#include <linux/config.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/usb.h>

#if MUSB_DEBUG > 0
#define dbgPrint(args...) printk(args)
#else
#define dbgPrint(args...)
#endif

#include "../core/hcd.h"

#include "musbdefs.h"
#include "musb_host.h"

/** how much to "scale" response timeouts */
#define MUSB_MAX_RETRIES         8

/*************************** Forwards ***************************/

static void MGC_HdrcProgramEnd(MGC_LinuxCd * pThis, u8 bEnd,
			       struct urb *pUrb, unsigned int nOut,
			       u8 * pBuffer, u32 dwLength);

/* DMA/HCD helpers */
/*STATIC*/ void MGC_LinuxStartNextUrb(MGC_LinuxCd * pThis, u8 bEnd);

/**************************************************************************
 *
 **************************************************************************/

/* KLUDGE: race in usb-core?  */
#define SLOW_DEVICE_KLUDGE() { /* mdelay(200); */	}

/**************************************************************************
 * Glue for virtual root hub
**************************************************************************/

/* Root speed need to be translated (addapted)
 */
static u8 MGC_TranslateVirtualHubSpeed(u8 source)
{
	u8 speed = 2;

	switch (source) {
	case 3:
		speed = 0;
		break;
	case 2:
		speed = 1;
		break;
	}

	return speed;
}				/* end of fucntion MGC_TranslateVirtualHubSpeed */

/*
 * Timer completion callback to turn off reset and get connection speed
 */
static void MGC_HdrcResetOff(unsigned long param)
{
	u8 power;
	MGC_LinuxCd *pThis = (MGC_LinuxCd *) param;
	void *pBase = pThis->pRegs;

	pThis->bIgnoreDisconnect = FALSE;
	DBG(2, "Stopping root port reset...\n");

	power = MGC_Read8(pBase, MGC_O_HDRC_POWER);
	MGC_Write8(pBase, MGC_O_HDRC_POWER, power & ~MGC_M_POWER_RESET);

	/* check for high-speed and set in root device if so */
	power = MGC_Read8(pBase, MGC_O_HDRC_POWER);
	if (power & MGC_M_POWER_HSMODE) {
		DBG(3, "high-speed device connected\n");
		pThis->bRootSpeed = 1;
	}

	MGC_VirtualHubPortResetDone(&(pThis->RootHub), 0,
				    MGC_TranslateVirtualHubSpeed(pThis->
								 bRootSpeed));
}				/* end of function MGC_HdrcResetOff() */

/* see virthub.h */
void MGC_LinuxSetPortPower(void *pPrivateData, u8 bPortIndex, u8 bPower)
{
	unsigned long flags;
	MGC_LinuxCd *pThis = (MGC_LinuxCd *) pPrivateData;

	spin_lock_irqsave(&pThis->Lock, flags);

	if (bPower) {
		DBG(1, "Root port power on\n");
		MGC_HdrcStart(pThis);
	} else {
		DBG(1, "Root port power off\n");
		MGC_HdrcStop(pThis);
	}

	spin_unlock_irqrestore(&pThis->Lock, flags);
}

/* see virthub.h */
void MGC_LinuxSetPortEnable(void *pPrivateData, u8 bPortIndex, u8 bEnable)
{
	DBG(2, "<==\n");
	if (bEnable) {
		DBG(3, "Root port enabled\n");
	} else {
		DBG(3, "Root port disabled\n");
	}
}				/* end of function MGC_LinuxSetPortEnable() */

/* see virthub.h */
void MGC_LinuxSetPortSuspend(void *pPrivateData, u8 bPortIndex, u8 bSuspend)
{
	u8 power;
	unsigned long flags;
	MGC_LinuxCd *pThis = (MGC_LinuxCd *) pPrivateData;
	void *pBase = pThis->pRegs;

	DBG(2, "<==\n");

	spin_lock_irqsave(&pThis->Lock, flags);
	power = MGC_Read8(pBase, MGC_O_HDRC_POWER);

	if (bSuspend) {
		DBG(3, "Root port power suspended\n");
		MGC_Write8(pBase, MGC_O_HDRC_POWER,
			   power | MGC_M_POWER_SUSPENDM);
	} else if (power & MGC_M_POWER_SUSPENDM) {
		DBG(3, "Root port power resumed\n");
		power &= ~(MGC_M_POWER_SUSPENDM | MGC_M_POWER_RESUME);
		MGC_Write8(pBase, MGC_O_HDRC_POWER, power | MGC_M_POWER_RESUME);
		mdelay(10);
		MGC_Write8(pBase, MGC_O_HDRC_POWER, power);
	}

	spin_unlock_irqrestore(&pThis->Lock, flags);
}				/* end of function MGC_LinuxSetPortSuspend() */

/* see virthub.h */
void MGC_LinuxSetPortReset(void *pPrivateData, u8 bPortIndex, u8 bReset)
{
	u8 power;
	unsigned long flags;
	MGC_LinuxCd *pThis = (MGC_LinuxCd *) pPrivateData;
	void *pBase = pThis->pRegs;

#ifdef CONFIG_USB_MUSB_OTG
	u8 devctl = MGC_Read8(pBase, MGC_O_HDRC_DEVCTL);
	if (pThis->bDelayPortPowerOff || !(devctl & MGC_M_DEVCTL_HM)) {
		return;
	}
#endif

	spin_lock_irqsave(&pThis->Lock, flags);
	power = MGC_Read8(pBase, MGC_O_HDRC_POWER) & 0xf0;

	DBG(2, "<==\n");

	if (bReset) {
		pThis->bIgnoreDisconnect = TRUE;
		MGC_Write8(pBase, MGC_O_HDRC_POWER, power | MGC_M_POWER_RESET);
		MGC_LinuxSetTimer(pThis, MGC_HdrcResetOff, (unsigned long)pThis,
				  60);
	} else if (power & MGC_M_POWER_RESET) {
		DBG(2, "root port reset stopped\n");
		MGC_Write8(pBase, MGC_O_HDRC_POWER, power & ~MGC_M_POWER_RESET);
	}

	spin_unlock_irqrestore(&pThis->Lock, flags);
}				/* end of function MGC_LinuxSetPortReset() */


/*
 * Start transmit. Caller is responsible for locking shared resources.
 * pThis must be locked.
 *
 * @param pThis instance pointer
 * @param bEnd local endpoint
 */
void MGC_HdrcStartTx(MGC_LinuxCd * pThis, u8 bEnd)
{
	u16 wCsr;
	u8 *pBase = (u8 *) pThis->pRegs;

	DBG(2, "<==\n");

	/* NOTE: no locks here; caller should lock */
	MGC_SelectEnd(pBase, bEnd);
	if (bEnd) {
		wCsr = MGC_ReadCsr16(pBase, MGC_O_HDRC_TXCSR, bEnd);
		wCsr |= MGC_M_TXCSR_TXPKTRDY;
		DBG(3, "Writing to TxCSR %x in StartTx\n", wCsr);
		MGC_WriteCsr16(pBase, MGC_O_HDRC_TXCSR, bEnd, wCsr);
	} else {
		wCsr = MGC_M_CSR0_H_SETUPPKT | MGC_M_CSR0_TXPKTRDY;
		MGC_WriteCsr16(pBase, MGC_O_HDRC_CSR0, 0, wCsr);
	}

}				/* end of function MGC_HdrcStartTx() */

/*
 *   Enable DmareqEnab in TxCSr
 *
 *   @param pThis instance pointer
 *   @param bEnd  local endpoint
 */
void MGC_HdrcEnableDmaReq(MGC_LinuxCd * pThis, u8 bEnd)
{
	u8 *pBase = (u8 *) pThis->pRegs;
	u16 txCsr;

	DBG(2, "<==\n");

	/* NOTE: no locks here; caller should lock */
	MGC_SelectEnd(pBase, bEnd);
	txCsr = MGC_ReadCsr16(pBase, MGC_O_HDRC_TXCSR, bEnd);
	/*dbgPrint("enabDma writing 0x%x end:%d\n",txCsr |MGC_M_TXCSR_DMAENAB , bEnd ); */
	MGC_WriteCsr16(pBase, MGC_O_HDRC_TXCSR, bEnd,
		       txCsr | MGC_M_TXCSR_DMAENAB);

}				/* end of function MGC_HdrcEnableDmaReq() */

/*
 * Start the current URB on an endpoint; wants ep to be
 * locked and pThis to be locked as well; end must be claimed
 * from the caller.
 *
 * @param pThis instance pointer
 * @param bEnd local endpoint
 */
static void MGC_LinuxStartUrb(MGC_LinuxCd * pThis, u8 bEnd)
{
	u16 wFrame;
	u32 dwLength;
	void *pBuffer;
	u8 *pBase = (u8 *) pThis->pRegs;
	MGC_LinuxLocalEnd *pEnd = &(pThis->aLocalEnd[bEnd]);
	struct urb *pUrb = MGC_GetCurrentUrb(pEnd);
	unsigned int nPipe, nOut;
	u16 wPacketSize;
	u8 bAddress, bRemoteEnd;

	DBG(2, "<==\n");

	/* I should not have called!!! */
	if (!pUrb) {
		ERR("BAD LOGIC! cannot call start urb with no urb\n");
		return;
	}

	nPipe = pUrb->pipe;
	nOut = usb_pipeout(nPipe);
	wPacketSize = usb_maxpacket(pUrb->dev, nPipe, nOut);
	bAddress = (u8) usb_pipedevice(nPipe);
	bRemoteEnd = (u8) usb_pipeendpoint(nPipe);

	/* if no root device, assume this must be it */
	if (!pThis->pRootDevice) {
		pThis->pRootDevice = pUrb->dev;
		switch (pThis->bRootSpeed) {
		case 1:
			pThis->pRootDevice->speed = USB_SPEED_HIGH;
			break;
		case 2:
			pThis->pRootDevice->speed = USB_SPEED_FULL;
			break;
		case 3:
			pThis->pRootDevice->speed = USB_SPEED_LOW;
			break;
		}
	}

	/* remember software state */
	pEnd->dwOffset = 0;
	pEnd->dwRequestSize = 0;
	pEnd->dwIsoPacket = 0;
	pEnd->dwWaitFrame = 0;
	pEnd->bRetries = 0;
	pEnd->wPacketSize = wPacketSize;
	pEnd->bAddress = bAddress;
	pEnd->bEnd = bRemoteEnd;
	pEnd->bTrafficType = (u8) usb_pipetype(nPipe);
	pEnd->bIsTx = (nOut) ? TRUE : FALSE;

	/* pEnd->bIsClaimed=(usb_pipeisoc(nPipe) || usb_pipeint(nPipe)) ?TRUE:FALSE;
	 * end must be claimed from my caller
	 */
	if (usb_pipecontrol(nPipe)) {
		/* control transfers always start with an OUT */
		nOut = 1;
		pThis->bEnd0Stage = MGC_END0_START;
	}

	/* gather right source of data */
	if (usb_pipeisoc(nPipe)) {
		pBuffer =
		    pUrb->transfer_buffer + pUrb->iso_frame_desc[0].offset;
		dwLength = pUrb->iso_frame_desc[0].length;
	} else if (usb_pipecontrol(nPipe)) {
		pBuffer = pUrb->setup_packet;
		dwLength = 8;
	} else {
		/* - */
		pBuffer = pUrb->transfer_buffer;
		dwLength = pUrb->transfer_buffer_length;
	}

	DBG(3, "(%p): ep%d%s, type %d, max %d, addr %d, buffer %p\n",
			pUrb, bRemoteEnd, (nOut) ? "out" : "in",
			usb_pipetype(nPipe), wPacketSize, bAddress, pBuffer);

	/* Configure endpoint */
	MGC_HdrcProgramEnd(pThis, bEnd, pUrb, nOut, pBuffer, dwLength);

	/* if transmit, start it if it is time */
	if (!nOut) {
		return;
	}

	DBG(3, "After Programend.Trigg URB len:%d ptr:%x---> ", dwLength,
		 (u32) pUrb);

	/* TODO: with CPPI DMA, once DMA is setup and DmaReqEnable in TxCSR
	 * is set (which is the case) transfer is initiated. For periodic
	 * transfers support add another field in pEnd struct which will
	 * serve as a flag. If CPPI DMA is programmed for the transfer set
	 * this flag and disable DMAReqEnab while programming TxCSR in
	 * programEnd() Once we reach the appropriate time, enable DMA Req
	 * instead of calling StartTx() function
	 */

	/* determine if the time is right for a periodic transfer */
	if (usb_pipeisoc(nPipe) || usb_pipeint(nPipe)) {
		DBG(3, "check whether there's still time for periodic Tx\n");
		pEnd->dwIsoPacket = 0;
		wFrame = MGC_Read16(pBase, MGC_O_HDRC_FRAME);
		/* FIXME this doesn't implement that scheduling policy ... */
		if ((pUrb->transfer_flags & URB_ISO_ASAP) ||
		    (wFrame >= pUrb->start_frame)) {
			pEnd->dwWaitFrame = 0;
			if (!pEnd->bEnableDmaReq) {
				MGC_HdrcStartTx(pThis, bEnd);
				dbgPrint("started periodic TX imme %d\n", bEnd);
			} else {
				dbgPrint("Call periodic EnableDma on %d\n",
					 bEnd);
				MGC_HdrcEnableDmaReq(pThis, bEnd);
			}
		} else {
			pEnd->dwWaitFrame = pUrb->start_frame;
			/* enable SOF interrupt so we can count down */
#if 1 // ifndef	CONFIG_ARCH_DAVINCI
			MGC_Write8(pBase, MGC_O_HDRC_INTRUSBE, 0xff);
#endif
		}
	} else {
		DBG(3, "starting Tx\n");

		if (!pEnd->bEnableDmaReq) {
			DBG(3, "starting NonP Tx on %d\n", bEnd);
			MGC_HdrcStartTx(pThis, bEnd);
		} else {
			DBG(3, "Enable DMA for NonP Tx on %d\n", bEnd);
			MGC_HdrcEnableDmaReq(pThis, bEnd);
		}
	}

}				/* end of function MGC_LinuxStartUrb() */

/* caller owns no controller locks, irqs are blocked */
static void musb_giveback(struct urb *urb, struct pt_regs *regs, int status)
{
	const struct musb_hw_ep		*hw_ep = urb->hcpriv;

	if ((urb->transfer_flags & URB_SHORT_NOT_OK)
			&& (urb->actual_length < urb->transfer_buffer_length)
			&& status == 0
			&& usb_pipein(urb->pipe))
		status = -EREMOTEIO;

	spin_lock(&urb->lock);
	list_del_init(&urb->urb_list);
	if (urb->status == -EINPROGRESS)
		urb->status = status;
	spin_unlock(&urb->lock);

	DBG((urb->status && urb->status != -ESHUTDOWN) ? 3 : 4,
			"complete %p (%d)\n", urb, status);

	urb->hcpriv = NULL;

	/* teardown DMA mapping, if needed (does dcache sync) */
	if (is_dma_capable()
			&& hw_ep->musb->controller->dma_mask
			&& urb->transfer_buffer_length != 0
			&& !(urb->transfer_flags & URB_NO_TRANSFER_DMA_MAP)) {
		dma_unmap_single(hw_ep->musb->controller, urb->transfer_dma,
				urb->transfer_buffer_length,
				usb_pipein(urb->pipe)
					? DMA_FROM_DEVICE
					: DMA_TO_DEVICE);
	}

	/* completion handler may reenter this hcd; periodic transfers
	 * are normally resubmitted during the callback.
	 *
	 * FIXME: make this use hcd framework giveback, so we don't use
	 * the usbcore-internal wakeup queue...
	 */
	urb->complete(urb, regs);
	atomic_dec(&urb->use_count);
	if (urb->reject)
		wake_up(&usb_kill_urb_queue);
	usb_put_urb(urb);
}


/*
 * Advance this endpoint's queue, completing the URB at list head.
 * Start the next URB on an endpoint. Wants the _endpoint_ to be locked.
 * It might call MGC_LinuxStartUrb pThis needs to be locked as well.
 *
 * @param pThis instance pointer
 * @param bEnd local endpoint
 */
void MGC_LinuxStartNextUrb(struct musb *pThis, u8 bEnd)
{
	struct urb		*urb;
	struct musb_hw_ep	*pEnd = pThis->aLocalEnd + bEnd;

	/* REVISIT give this a more accurate name */

	DBG(3, "advance %d bLocal:%02x\n", (u32) pEnd, pEnd->bLocalEnd);

	urb = MGC_GetCurrentUrb(pEnd);
	if (urb) {
		spin_unlock(&pEnd->Lock);
		musb_giveback(urb, NULL, 0);
		spin_lock(&pEnd->Lock);
	}

	if (!list_empty(&pEnd->urb_list))
		MGC_LinuxStartUrb(pThis, bEnd);
	else
		MGC_HdrcStopEnd(pThis, bEnd);
}


/*
 * Receive a packet (or part of it).
 * @requires pThis->Lock locked
 * @return TRUE if URB is complete
 */
static u8 MGC_LinuxPacketRx(MGC_LinuxCd * pThis, u8 bEnd, u8 bIsochError)
{
	u16 wRxCount;
	u16 wLength;
	u8 *pBuffer;
	u16 wCsr;
	u8 bDone = FALSE;
	u8 *pBase = (u8 *) pThis->pRegs;
	MGC_LinuxLocalEnd *pEnd = &(pThis->aLocalEnd[bEnd]);
	struct urb *pUrb = MGC_GetCurrentUrb(pEnd);
	int nPipe = 0;
	void *buffer = NULL;

	// MGC_SelectEnd(pBase, bEnd);
	wRxCount = MGC_ReadCsr16(pBase, MGC_O_HDRC_RXCOUNT, bEnd);
	DBG(2, "end %d RxCount=%04x\n", bEnd, wRxCount);

#ifdef MUSB_PARANOID
	if (!pUrb || ((pUrb->transfer_buffer_length - pEnd->dwOffset) < 0)) {
		ERR("ERROR during Rx: pUrb=%p, pUrb->transfer_buffer_length=%d pEnd->dwOffset=%d\n",
				pUrb, pUrb->transfer_buffer_length, pEnd->dwOffset);
		return TRUE;
	}
#endif

	nPipe = pUrb->pipe;
	buffer = pUrb->transfer_buffer;

	DBG(3, "flags 0x%x buffer %p len %d offset %d\n",
			pUrb->transfer_flags, pUrb->transfer_buffer,
			pUrb->transfer_buffer_length, pEnd->dwOffset);

	/* unload FIFO */
	if (usb_pipeisoc(nPipe)) {
		/* isoch case */
		pBuffer =
		    buffer + pUrb->iso_frame_desc[pEnd->dwIsoPacket].offset;
		wLength =
		    min((unsigned int)wRxCount,
			pUrb->iso_frame_desc[pEnd->dwIsoPacket].length);
		pUrb->actual_length += wLength;
		/* update actual & status */
		pUrb->iso_frame_desc[pEnd->dwIsoPacket].actual_length = wLength;
		if (bIsochError) {
			pUrb->iso_frame_desc[pEnd->dwIsoPacket].status =
			    -EILSEQ;
			pUrb->error_count++;
		} else {
			pUrb->iso_frame_desc[pEnd->dwIsoPacket].status = 0;
		}

		/* see if we are done */
		bDone = (++pEnd->dwIsoPacket >= pUrb->number_of_packets);
	} else {
		/* non-isoch */
		pBuffer = buffer + pEnd->dwOffset;
		wLength = min((unsigned int)wRxCount,
			      pUrb->transfer_buffer_length - pEnd->dwOffset);
		pUrb->actual_length += wLength;
		pEnd->dwOffset += wLength;

		/* see if we are done */
		bDone = (pEnd->dwOffset >= pUrb->transfer_buffer_length) ||
		    (wRxCount < pEnd->wPacketSize);
	}

	MGC_HdrcUnloadFifo(pBase, bEnd, wLength, pBuffer);

#ifdef MUSB_CONFIG_PROC_FS
	pEnd->dwTotalRxBytes += wLength;
#endif
	COUNT(pEnd->dwTotalRxPackets);

	if (wRxCount <= wLength) {
		wCsr = MGC_ReadCsr16(pBase, MGC_O_HDRC_RXCSR, bEnd);
		MGC_WriteCsr16(pBase, MGC_O_HDRC_RXCSR, bEnd,
			       wCsr & ~MGC_M_RXCSR_RXPKTRDY);
	}

	return bDone;
}				/* end of function MGC_LinuxPacketRx() */


/* FIXME this should be ffs(), but it's actually not the right function
 * to be calling ... '0x14' should map to interval 0x10, not to 0x04!
 */

/*
 * Find first (lowest-order) 1 bit
 * @param nValue value in which to search
 * @return bit position (0 could mean no bit; caller should check)
 */
static u8 MGC_FindFirst1(unsigned int nValue)
{
	unsigned int nWork = nValue;
	u8 bResult;

	for (bResult = 0; bResult < 32; bResult++) {
		if (nWork & 1) {
			return bResult;
		}
		nWork >>= 1;
	}
	return bResult;
}				/* end of function MGC_FindFirst1() */

/*
 * Program an HDRC endpoint as per the given URB
 * @param pThis instance pointer
 * @param bEnd local endpoint
 * @param pURB URB pointer
 * @param nOut zero for Rx; non-zero for Tx
 * @param pBuffer buffer pointer
 * @param dwLength how many bytes to transmit or expect to receive
 */
static void MGC_HdrcProgramEnd(MGC_LinuxCd * pThis, u8 bEnd,
			       struct urb *pUrb, unsigned int nOut,
			       u8 * pBuffer, u32 dwLength)
{
	u16 wCsr, wLoadCount, wIntrTxE;
	struct usb_device *pParent;
#ifndef	CONFIG_USB_INVENTRA_FIFO
	struct dma_controller *pDmaController;
	struct dma_channel *pDmaChannel;
	u8 bDmaOk = FALSE;
#endif
	u8 *pBase = (u8 *) pThis->pRegs;
	unsigned int nPipe = pUrb->pipe;
	u16 wPacketSize = usb_maxpacket(pUrb->dev, nPipe, nOut);
	u8 bIsBulk = usb_pipebulk(nPipe);
	u8 bAddress = (u8) usb_pipedevice(nPipe);
	u8 bRemoteEnd = (u8) usb_pipeendpoint(nPipe);
	u8 bSpeed = (u8) pUrb->dev->speed;
	u8 bInterval = (u8) pUrb->interval;
	MGC_LinuxLocalEnd *pEnd = &(pThis->aLocalEnd[bEnd]);
	u8 bStdType = 0;
	u8 bHubAddr = 0;
	u8 bHubPort = 0;
	u8 reg = 0;
	u16 tempCsr;
	u8 bIsMulti = FALSE;
	u8 bDone = FALSE;
	unsigned long flags;

	DBG(2, "<==(bEnd %d, urb %p, len %d) addr %d\n",
			bEnd, pUrb, bAddress, dwLength);

	if(bEnd)
		DBG(3, "ProgramEnd buff=%p urb>tranfer=%p dma=%x\n",
				pBuffer,pUrb->transfer_buffer,pUrb->transfer_dma);

	/* NOTE: there is always a parent due to the virtual root hub */
	/* parent hub address */
	pParent = pUrb->dev->parent;
	bHubAddr = (u8) pParent->devnum;
	if (bHubAddr == pThis->RootHub.bAddress) {
		/* but not if parent is our virtual root hub */
		bHubAddr = 0;
	}

	/* set up tt info if needed */
	if (pUrb->dev->tt) {
		bHubPort = (u8) pUrb->dev->ttport;
		bIsMulti = (u8) pUrb->dev->tt->multi;
	}

	DBG(3,
	    "end %d, device %d, parent %d, port %d, multi-tt: %d, speed:%d\n",
	    bEnd, pUrb->dev->devnum, bHubAddr, bHubPort, bIsMulti,
	    pUrb->dev->speed);

	/* REVISIT hcd glue layer guarantees periodic intervals are
	 * all sane powers-of-two, in units of (micro)frames.  So this
	 * ffs() stuff isnt needed ... it's wrong too:  it should
	 * use fls() of the rounded-up version.
	 */
	/* prepare endpoint registers according to flags */
	if (usb_pipeisoc(nPipe)) {
		bStdType = 1;
		if (pUrb->interval > 16) {
			bInterval = MGC_FindFirst1(pUrb->interval);	/* correct interval */
		}
		if (bInterval < 1) {
			bInterval = 1;
		}
	} else if (usb_pipeint(nPipe)) {
		bStdType = 3;
		if ((USB_SPEED_HIGH == bSpeed) && (pUrb->interval > 255)) {
			/* correct interval for high-speed */
			bInterval = MGC_FindFirst1(pUrb->interval);
		}
		if (bInterval < 1) {
			bInterval = 1;
		}
	} else if (bIsBulk) {
		bStdType = 2;
		bInterval = 0;	/* ignoring bulk NAK limits for now */
	} else {
		nOut = 1;
		bInterval = 0;
	}

	reg = bStdType << 4;
	/* really NAKlimit in this case */
	if (bInterval < 2) {
		bInterval = 16;
	}

	reg |= (bRemoteEnd & 0xf);
	if (pThis->bIsMultipoint) {
		switch (bSpeed) {
		case USB_SPEED_LOW:
			reg |= 0xc0;
			break;
		case USB_SPEED_FULL:
			reg |= 0x80;
			break;
		default:
			reg |= 0x40;
		}
	}

	if (bIsBulk && pThis->bBulkSplit) {
		wLoadCount = min((u32) pEnd->wMaxPacketSizeTx, dwLength);
	} else {
		wLoadCount = min((u32) wPacketSize, dwLength);
	}

#ifdef CONFIG_USB_INVENTRA_DMA
	pDmaController = pThis->pDmaController;
	pDmaChannel = pEnd->pDmaChannel;
	if (!WANTS_DMA(pUrb) && pDmaChannel) {
		/* release previously-allocated channel */
		pDmaController->pfDmaReleaseChannel(pDmaChannel);
		pEnd->pDmaChannel = NULL;
	} else if (WANTS_DMA(pUrb)) {
		/* candidate for DMA */
		if (pDmaController && !pDmaChannel) {
			pDmaChannel = pEnd->pDmaChannel =
			    pDmaController->
			    pfDmaAllocateChannel(pDmaController->pPrivateData,
						 bEnd, nOut ? TRUE : FALSE,
						 bStdType, wPacketSize);
		}
		if (pDmaChannel) {
			pDmaChannel->dwActualLength = 0L;
			pEnd->dwRequestSize =
			    min(dwLength, pDmaChannel->dwMaxLength);
			bDmaOk =
			    pDmaController->pfDmaProgramChannel(pDmaChannel,
								wPacketSize,
								pDmaChannel->
								bDesiredMode,
								pUrb->
								transfer_dma,
								pEnd->
								dwRequestSize);
			if (bDmaOk) {
				wLoadCount = 0;
			} else {
				pDmaController->
				    pfDmaReleaseChannel(pDmaChannel);
				pEnd->pDmaChannel = NULL;
			}
		}
	}
#endif

	spin_lock_irqsave(&pThis->Lock, flags);
	MGC_SelectEnd(pBase, bEnd);

	wCsr = MGC_ReadCsr16(pBase, MGC_O_HDRC_TXCSR, bEnd);
	/*wCsr |= MGC_M_TXCSR_MODE; */

	/* make sure we clear DMAEnab, autoSet bits from previous run */

	/* transmit or receive? */
	if (nOut) {
		/* transmit */
		/* disable interrupt in case we flush */
		wIntrTxE = MGC_Read16(pBase, MGC_O_HDRC_INTRTXE);
		MGC_Write16(pBase, MGC_O_HDRC_INTRTXE, wIntrTxE & ~(1 << bEnd));
		if (bEnd) {

			/* general endpoint */
			/* if not ready, flush and restore data toggle */
			if (!pEnd->bIsReady && pThis->bIsMultipoint) {
				DBG(3,
				    "EndPt not ready flush/clean ready:%d\n",
				     pEnd->bIsReady);
				pEnd->bIsReady = TRUE;
				/* twice in case of double packet buffering */
				MGC_WriteCsr16(pBase, MGC_O_HDRC_TXCSR, bEnd,
					       wCsr | MGC_M_TXCSR_FLUSHFIFO |
					       MGC_M_TXCSR_CLRDATATOG);
				MGC_WriteCsr16(pBase, MGC_O_HDRC_TXCSR, bEnd,
					       wCsr | MGC_M_TXCSR_FLUSHFIFO |
					       MGC_M_TXCSR_CLRDATATOG);
				/* data toggle */
				tempCsr = wCsr | MGC_M_TXCSR_H_WR_DATATOGGLE;
				if (usb_gettoggle(pUrb->dev, pEnd->bEnd, 1)) {
					tempCsr |= MGC_M_TXCSR_H_DATATOGGLE;
				} else {
					tempCsr &= ~MGC_M_TXCSR_H_DATATOGGLE;
				}
				/*printk("reProgram toggle TxCsr=%x\n",tempCsr); */
				MGC_WriteCsr16(pBase, MGC_O_HDRC_TXCSR, bEnd,
					       tempCsr);
			}
		} else {
			/* endpoint 0: just flush */
			MGC_WriteCsr16(pBase, MGC_O_HDRC_CSR0, bEnd,
				       wCsr | MGC_M_CSR0_FLUSHFIFO);
			MGC_WriteCsr16(pBase, MGC_O_HDRC_CSR0, bEnd,
				       wCsr | MGC_M_CSR0_FLUSHFIFO);
		}

		if (pThis->bIsMultipoint) {
			/* target addr & hub addr/port */
			MGC_Write8(pBase,
				   MGC_BUSCTL_OFFSET(bEnd,
						     MGC_O_HDRC_TXFUNCADDR),
				   bAddress);
			MGC_Write8(pBase,
				   MGC_BUSCTL_OFFSET(bEnd,
						     MGC_O_HDRC_TXHUBADDR),
				   bIsMulti ? 0x80 | bHubAddr : bHubAddr);
			MGC_Write8(pBase,
				   MGC_BUSCTL_OFFSET(bEnd,
						     MGC_O_HDRC_TXHUBPORT),
				   bHubPort);
			/* also, try Rx */
			MGC_Write8(pBase,
				   MGC_BUSCTL_OFFSET(bEnd,
						     MGC_O_HDRC_RXFUNCADDR),
				   bAddress);
			MGC_Write8(pBase,
				   MGC_BUSCTL_OFFSET(bEnd,
						     MGC_O_HDRC_RXHUBADDR),
				   bIsMulti ? 0x80 | bHubAddr : bHubAddr);
			MGC_Write8(pBase,
				   MGC_BUSCTL_OFFSET(bEnd,
						     MGC_O_HDRC_RXHUBPORT),
				   bHubPort);
		} else {
			/* non-multipoint core */
			MGC_Write8(pBase, MGC_O_HDRC_FADDR, bAddress);
		}

		/* protocol/endpoint/interval/NAKlimit */
		if (bEnd) {
			MGC_WriteCsr8(pBase, MGC_O_HDRC_TXTYPE, bEnd, reg);
			if (bIsBulk && pThis->bBulkSplit) {
				MGC_WriteCsr16(pBase, MGC_O_HDRC_TXMAXP, bEnd,
					       wPacketSize |
					       ((pEnd->wMaxPacketSizeTx /
						 wPacketSize) - 1) << 11);
			} else {
				MGC_WriteCsr16(pBase, MGC_O_HDRC_TXMAXP, bEnd,
					       wPacketSize);
			}
			MGC_WriteCsr8(pBase, MGC_O_HDRC_TXINTERVAL, bEnd,
				      bInterval);
		} else {
			MGC_WriteCsr8(pBase, MGC_O_HDRC_NAKLIMIT0, 0,
				      bInterval);
			if (pThis->bIsMultipoint) {
				MGC_WriteCsr8(pBase, MGC_O_HDRC_TYPE0, 0,
					      reg & 0xc0);
			}
		}
		/* With CPPI DMA alone, need to set flag for enabling
		 * DmaReqEnab. By default clear this */
		DBG(3, "default enablereq to 0 \n");
		pEnd->bEnableDmaReq = 0;

#ifdef CONFIG_USB_INVENTRA_DMA
		if (bDmaOk) {
			wCsr |= (MGC_M_TXCSR_AUTOSET | MGC_M_TXCSR_DMAENAB |
				 (pDmaChannel->
				  bDesiredMode ? MGC_M_TXCSR_DMAMODE : 0));
			MGC_WriteCsr16(pBase, MGC_O_HDRC_TXCSR, bEnd, wCsr);
		}
#elif defined(CONFIG_USB_TI_CPPI_DMA)
		wCsr = MGC_ReadCsr16(pBase, MGC_O_HDRC_TXCSR, bEnd);

		/* we will program the endpoint CSRs first and then setup DMA
		 * ensure the TXCSR is programmed before firing DMA */

		/* set the CSR flags appropriately */
		/* DMAMode set */
		wCsr |= (MGC_M_TXCSR_DMAMODE);
		/* AutoSet cleared */
		wCsr &= ~MGC_M_TXCSR_AUTOSET;
		/* defer DmaEnable until appropriate time, set flag for enabling dmaReq */
		wCsr &= ~MGC_M_TXCSR_DMAENAB;
		/* write CSR */
		wCsr |= MGC_M_TXCSR_MODE;

		/* set the CSR for DMA mode assuming CPPI DMA programming will succeed
		 * If the programming fails, we revert to copying of data to FIFO and turn
		 * off DMAreqEnable in CSR */
		dbgPrint("Programming CSr for DMA 0x%x\n",
			 wCsr | MGC_M_TXCSR_MODE);
		MGC_WriteCsr16(pBase, MGC_O_HDRC_TXCSR, bEnd,
			       wCsr | MGC_M_TXCSR_MODE);

		/*if(dwLength >= 0x200)
		   {
		   int tempctr;
		   for(tempctr =0;tempctr<512;tempctr++)
		   printk("%02x ",((u8*)(pUrb->transfer_buffer))[tempctr]);
		   printk("\n");
		   }
		 */

		/* Setup CPPI DMA  */
		pDmaController = pThis->pDmaController;
		pDmaChannel = pEnd->pDmaChannel;

#if 0
		/* explore whether device must advertise that its DMA capable.Always WANTS_DMA is false now */
		if (!WANTS_DMA(pUrb) && pDmaChannel) {
			/* release previously-allocated channel */
			pDmaController->pfDmaReleaseChannel(pDmaChannel);
			pEnd->pDmaChannel = NULL;
		} else
#endif
		if (bEnd != 0 /*WANTS_DMA(pUrb) */ ) {
			/* candidate for DMA */
			if (pDmaController && !pDmaChannel) {
				pDmaChannel = pEnd->pDmaChannel =
				    pDmaController->
				    pfDmaAllocateChannel(pDmaController->
							 pPrivateData, bEnd,
							 nOut ? TRUE : FALSE,
							 bStdType, wPacketSize);
			}
			if (pDmaChannel) {
				pDmaChannel->dwActualLength = 0L;
				pEnd->dwRequestSize = dwLength;
				dbgPrint
				    ("Programchannel buff:0x%x dmaadd:0x%x \n",
				     (u32) pUrb->transfer_buffer,
				     (u32) pUrb->transfer_dma);

				bDmaOk =
				    pDmaController->
				    pfDmaProgramChannel(pDmaChannel,
							wPacketSize,
							pDmaChannel->
							bDesiredMode,
							pUrb->transfer_dma,
							pEnd->dwRequestSize);
				if (bDmaOk) {
					wLoadCount = 0;
					/* set flag for enabling Dmareq */
					pEnd->bEnableDmaReq = 1;
				} else {
					pDmaController->
					    pfDmaReleaseChannel(pDmaChannel);
					pEnd->pDmaChannel = NULL;
				}
			}	/* end of if pDmaChannel conditional block */
		}
		/* end of else if conditional block */
#endif
		if (wLoadCount) {
			/* load FIFO */
			pEnd->dwRequestSize = wLoadCount;
			MGC_HdrcLoadFifo(pThis->pRegs, bEnd, wLoadCount,
					 pBuffer);
			wCsr = MGC_ReadCsr16(pBase, MGC_O_HDRC_TXCSR, bEnd);
			wCsr &=
			    ~(MGC_M_TXCSR_DMAENAB | MGC_M_TXCSR_DMAMODE |
			      MGC_M_TXCSR_AUTOSET);
			/* write CSR */
			wCsr |= MGC_M_TXCSR_MODE;

			if (bEnd)
				MGC_WriteCsr16(pBase, MGC_O_HDRC_TXCSR, bEnd,
					       wCsr);

		}

		/* re-enable interrupt */
		MGC_Write16(pBase, MGC_O_HDRC_INTRTXE, wIntrTxE);

	} else {
		/* receive */
		/* if programmed for Tx, be sure it is ready for re-use */
		if (wCsr & MGC_M_TXCSR_MODE) {
			pEnd->bIsReady = FALSE;
			dbgPrint("set endready to False \n");
			if (wCsr & MGC_M_TXCSR_FIFONOTEMPTY) {
				/* this shouldn't happen */
				MGC_WriteCsr16(pBase, MGC_O_HDRC_TXCSR, bEnd,
					       MGC_M_TXCSR_FRCDATATOG);
				MGC_WriteCsr16(pBase, MGC_O_HDRC_TXCSR, bEnd,
					       MGC_M_TXCSR_FRCDATATOG);
				ERR("switching end %d to Rx but Tx FIFO not empty\n", bEnd);
			}
			/* clear mode (and everything else) to enable Rx */
			MGC_WriteCsr16(pBase, MGC_O_HDRC_TXCSR, bEnd, 0);
		}

		/* end of if wCsr & TXCSR_MODE conditional block */
/*#if 0*/
#ifdef CONFIG_USB_TI_CPPI_DMA
		/* Setup CPPI DMA  */
		pDmaController = pThis->pDmaController;
		pDmaChannel = pEnd->pDmaChannel;

#if 0
		/* explore whether device must advertise that its DMA capable.Always WANTS_DMA is false now */
		if (!WANTS_DMA(pUrb) && pDmaChannel) {
			/* release previously-allocated channel */
			pDmaController->pfDmaReleaseChannel(pDmaChannel);
			pEnd->pDmaChannel = NULL;
		} else
#endif
		if (bEnd != 0 /*WANTS_DMA(pUrb) */ ) {
			/* candidate for DMA */
			if (pDmaController && !pDmaChannel) {
				pDmaChannel = pEnd->pDmaChannel =
				    pDmaController->
				    pfDmaAllocateChannel(pDmaController->
							 pPrivateData, bEnd,
							 nOut ? TRUE : FALSE,
							 bStdType, wPacketSize);
			}
			if (pDmaChannel) {
				u32 dmaAddr;
				pDmaChannel->dwActualLength = 0L;
				pEnd->dwRequestSize = dwLength;
				dbgPrint
				    ("Programchannel buff;0x%x dmaadd:0x%x \n",
				     (u32) pUrb->transfer_buffer,
				     (u32) pUrb->transfer_dma);

				printk("Buff:%x dma:%x Inv:%x Sz:%x\n",
				       (u32) pUrb->transfer_buffer,
				       (u32) pUrb->transfer_dma,
				       (u32) pUrb->transfer_buffer,
				       pEnd->dwRequestSize);

				dmaAddr = (u32) pUrb->transfer_dma;

				bDmaOk = pDmaController-> pfDmaProgramChannel(
						pDmaChannel, wPacketSize,
						pDmaChannel->bDesiredMode,
						dmaAddr, pEnd->dwRequestSize);
				if (!bDmaOk) {
					pDmaController->
					    pfDmaReleaseChannel(pDmaChannel);
					pEnd->pDmaChannel = NULL;
				}
			}	/* end of if pDmaChannel conditional block */
		}

		/* end of else if conditional block */
#endif

#ifndef	CONFIG_USB_INVENTRA_FIFO
		/* grab Rx residual if any */
		if (!bDmaOk) {
			wCsr = MGC_ReadCsr16(pBase, MGC_O_HDRC_RXCSR, bEnd);
			if (wCsr & MGC_M_RXCSR_RXPKTRDY) {
				bDone = MGC_LinuxPacketRx(pThis, bEnd, FALSE);
				dbgPrint("Rx : grab residue bDone=%d \n",
					 bDone);
			}
		}
#endif

		/* address */
		if (pThis->bIsMultipoint) {
			/* target addr & hub addr/port */
			MGC_Write8(pBase,
				   MGC_BUSCTL_OFFSET(bEnd,
						     MGC_O_HDRC_RXFUNCADDR),
				   bAddress);
			MGC_Write8(pBase,
				   MGC_BUSCTL_OFFSET(bEnd,
						     MGC_O_HDRC_RXHUBADDR),
				   bIsMulti ? 0x80 | bHubAddr : bHubAddr);
			MGC_Write8(pBase,
				   MGC_BUSCTL_OFFSET(bEnd,
						     MGC_O_HDRC_RXHUBPORT),
				   bHubPort);
			/* also, try Tx */
			MGC_Write8(pBase,
				   MGC_BUSCTL_OFFSET(bEnd,
						     MGC_O_HDRC_TXFUNCADDR),
				   bAddress);
			MGC_Write8(pBase,
				   MGC_BUSCTL_OFFSET(bEnd,
						     MGC_O_HDRC_TXHUBADDR),
				   bIsMulti ? 0x80 | bHubAddr : bHubAddr);
			MGC_Write8(pBase,
				   MGC_BUSCTL_OFFSET(bEnd,
						     MGC_O_HDRC_TXHUBPORT),
				   bHubPort);
		} else {
			/* non-multipoint core */
			MGC_Write8(pBase, MGC_O_HDRC_FADDR, bAddress);
		}

		/* protocol/endpoint/interval/NAKlimit */
		if (bEnd) {
			MGC_WriteCsr8(pBase, MGC_O_HDRC_RXTYPE, bEnd, reg);
#if 0
			/* doesn't work reliably */
			if (bIsBulk && pThis->bBulkCombine) {
				MGC_WriteCsr16(pBase, MGC_O_HDRC_RXMAXP, bEnd,
					       wPacketSize |
					       ((min
						 (pEnd->wMaxPacketSizeRx,
						  dwLength) / wPacketSize) -
						1) << 11);
			} else {
#endif
				MGC_WriteCsr16(pBase, MGC_O_HDRC_RXMAXP, bEnd,
					       wPacketSize);
#if 0
			}
#endif
			MGC_WriteCsr8(pBase, MGC_O_HDRC_RXINTERVAL, bEnd,
				      bInterval);
		} else if (pThis->bIsMultipoint) {
			MGC_WriteCsr8(pBase, MGC_O_HDRC_TYPE0, 0, reg & 0xc0);
		}

		/* first time or re-program and shared FIFO, flush & clear toggle */
		if (!pEnd->bIsReady && pEnd->bIsSharedFifo) {
			/* twice in case of double packet buffering */
			MGC_WriteCsr16(pBase, MGC_O_HDRC_RXCSR, bEnd,
				       MGC_M_RXCSR_FLUSHFIFO |
				       MGC_M_RXCSR_CLRDATATOG);
			MGC_WriteCsr16(pBase, MGC_O_HDRC_RXCSR, bEnd,
				       MGC_M_RXCSR_FLUSHFIFO |
				       MGC_M_RXCSR_CLRDATATOG);
			pEnd->bIsReady = TRUE;
		}

		/* program data toggle if possibly switching use */
		if (!pEnd->bIsReady && pThis->bIsMultipoint) {
			wCsr = MGC_M_RXCSR_H_WR_DATATOGGLE;
			if (usb_gettoggle(pUrb->dev, pEnd->bEnd, 0)) {
				wCsr |= MGC_M_RXCSR_H_DATATOGGLE;
			}
			MGC_WriteCsr16(pBase, MGC_O_HDRC_RXCSR, bEnd, wCsr);
		}

		/* kick things off */
		if (bEnd && !bDone) {
			wCsr = MGC_M_RXCSR_H_REQPKT;
			if (usb_pipeint(nPipe)) {
				wCsr |= MGC_M_RXCSR_DISNYET;
			}
#ifdef CONFIG_USB_INVENTRA_DMA
			if (bDmaOk) {
				wCsr &= ~MGC_M_RXCSR_H_REQPKT;
				wCsr |= MGC_M_RXCSR_H_AUTOREQ;
				wCsr |=
				    (MGC_M_RXCSR_AUTOCLEAR | MGC_M_RXCSR_DMAENAB
				     | (pDmaChannel->
					bDesiredMode ? MGC_M_RXCSR_DMAMODE :
					0));
			}
#elif defined(CONFIG_USB_TI_CPPI_DMA)
			if (bDmaOk) {
				s8 reProgram = TRUE;
				wCsr =
				    MGC_ReadCsr16(pBase, MGC_O_HDRC_RXCSR,
						  bEnd);

				/* if required bits are already set/cleared dont touch the RxCsr */
				if ((wCsr & MGC_M_RXCSR_DMAENAB)
				    && (!(wCsr & MGC_M_RXCSR_AUTOCLEAR))
				    && (!(wCsr & MGC_M_RXCSR_DMAMODE))) {
					if (pThis->bIsHost) {
						if ((!
						     (wCsr &
						      MGC_M_RXCSR_H_AUTOREQ))
						    && (wCsr &
							MGC_M_RXCSR_H_REQPKT)) {
							/* Host mode and all required bits in Csr set/cleared dont have to reprogram */
							reProgram = FALSE;
						}
					} else {
						/* not Host mode and DMAENAB set,AUTOCLEAR/DMAMODE cleared no need to
						 * * re-program */
						reProgram = FALSE;
					}
				}
				if (reProgram) {
					dbgPrint
					    ("reprogramming CSR for CPPI setup in Rx \n");
					if (pThis->bIsHost) {
						/* clear  autoReq in CSR */
						wCsr &=
						    ~(MGC_M_RXCSR_H_AUTOREQ);
						/* Need to program the autoReq register in wrapper register space?
						 * look at impact of Non-Rndis Mode, need to set ReqPkt bit on every packet */
						/* we need to set ReqPkt initially atleast */
						wCsr |= MGC_M_RXCSR_H_REQPKT;
					}
					/* set DMA ReqEnab */
					wCsr |= (MGC_M_RXCSR_DMAENAB);
					/* clear AUTOCLEAr and DMAMODE */
					wCsr &=
					    ~(MGC_M_RXCSR_AUTOCLEAR |
					      MGC_M_RXCSR_DMAMODE);

					/* write RxCsr */
					/*MGC_WriteCsr16(pBase, MGC_O_HDRC_RXCSR, bEnd, wCsr); */

				}
			}
#endif

			/* to ensure that we accidentally dont clear RxPktRdy, write ro RxCSR with
			   RxPktRdy bit set */
			MGC_WriteCsr16(pBase, MGC_O_HDRC_RXCSR, bEnd,
				       wCsr | MGC_M_RXCSR_RXPKTRDY);
			/*dbgPrint("In mhdrcRx: Rxcsr write=0x%x read=0x%x\n",wCsr, MGC_ReadCsr16(pBase, MGC_O_HDRC_RXCSR, bEnd)); */
		}
	}

	DBG(2, "==>\n");
	spin_unlock_irqrestore(&pThis->Lock, flags);
}				/* end of function MGC_HdrcProgramEnd() */

/*
 * Try to stop traffic on the given local endpoint.
 * @param pThis the controller
 * @param bEnd the endpoint number.
 */
void MGC_HdrcStopEnd(struct musb *pThis, u8 bEnd)
{
	u16 wCsr;
	unsigned long flags;
	u8 *pBase = (u8 *) pThis->pRegs;
	const u8 reg = (bEnd) ? MGC_O_HDRC_RXCSR : MGC_O_HDRC_CSR0;
	struct musb_hw_ep *hw_ep = pThis->aLocalEnd + bEnd;

	spin_lock_irqsave(&hw_ep->Lock, flags);

	/* clear the pending request */
	spin_lock(&pThis->Lock);
	MGC_SelectEnd(pBase, bEnd);
	wCsr = MGC_ReadCsr16(pBase, reg, bEnd);
	wCsr &= (bEnd) ? ~MGC_M_RXCSR_H_REQPKT : ~MGC_M_CSR0_H_REQPKT;
	MGC_WriteCsr16(pBase, reg, bEnd, wCsr);
	spin_unlock(&pThis->Lock);

	while (!list_empty(&hw_ep->urb_list)) {
		struct urb *urb;

		urb = list_entry(hw_ep->urb_list.next, struct urb, urb_list);
		DBG(3, "abort urb %p\n", urb);
		musb_giveback(urb, NULL, -ESHUTDOWN);

		// REVISIT ... usbcore will abort things for us
		// if we hook up to it properly, so that this
		// routine should never see urbs still queued
	}

	switch(hw_ep->bTrafficType) {
	case PIPE_ISOCHRONOUS:
	case PIPE_INTERRUPT:
// REVISIT this is where periodic bandwidth would be de-allocated.
		hw_ep->bIsClaimed = FALSE;
		break;
	}

	spin_unlock_irqrestore(&hw_ep->Lock, flags);
}

/*
 * Service the default endpoint (ep0) as host.
 *
 * @param pThis this
 * @param wCount current byte count in FIFO
 * @param pUrb URB pointer for EP0
 * @return TRUE if more packets are required for this transaction
 */
static u8 MGC_HdrcServiceHostDefault(MGC_LinuxCd * pThis,
				     u16 wCount, struct urb *pUrb)
{
	u8 bMore = FALSE;
	u8 *pFifoDest = NULL;
	u16 wFifoCount = 0;
	u8 *pBase = (u8 *) pThis->pRegs;
	MGC_LinuxLocalEnd *pEnd = &(pThis->aLocalEnd[0]);
	struct usb_ctrlrequest *pRequest =
	    (struct usb_ctrlrequest *)pUrb->setup_packet;

	DBG(2, "<== (wCount=%04x, pUrb=%lx, bStage=%02x)\n",
	    wCount, (unsigned long)pUrb, pThis->bEnd0Stage);

	if (MGC_END0_IN == pThis->bEnd0Stage) {
		/* we are receiving from peripheral */
		pFifoDest = pUrb->transfer_buffer + pUrb->actual_length;
		wFifoCount =
		    min(wCount,
			((u16)
			 (pUrb->transfer_buffer_length - pUrb->actual_length)));

		DBG(3,
		    "Receiving %d bytes in &%p[%d] (pUrb->actual_length=%u)\n",
		    wFifoCount, pUrb->transfer_buffer,
		    (unsigned int)pUrb->actual_length, pUrb->actual_length);

		MGC_HdrcUnloadFifo(pBase, 0, wFifoCount, pFifoDest);

#ifdef MUSB_CONFIG_PROC_FS
		pEnd->dwTotalRxBytes += wFifoCount;
#endif
		COUNT(pEnd->dwTotalRxPackets);

		pUrb->actual_length += wFifoCount;
		if ((pUrb->actual_length < pUrb->transfer_buffer_length) &&
		    (wCount == pEnd->wPacketSize)) {
			bMore = TRUE;
		}
	} else {
		/* we are sending to peripheral */
		if ((MGC_END0_START == pThis->bEnd0Stage) &&
		    (pRequest->bRequestType & USB_DIR_IN)) {
			DBG(3, "just did setup, switching to IN\n");

			/* this means we just did setup; switch to IN */
			pThis->bEnd0Stage = MGC_END0_IN;
			bMore = TRUE;

#ifdef MUSB_CONFIG_PROC_FS
			pEnd->dwTotalTxBytes += 8;
#endif
			COUNT(pEnd->dwTotalTxPackets);

		} else if (pRequest->wLength
			   && (MGC_END0_START == pThis->bEnd0Stage)) {
			pThis->bEnd0Stage = MGC_END0_OUT;
			pFifoDest =
			    (u8 *) (pUrb->transfer_buffer +
				    pUrb->actual_length);
			wFifoCount =
			    min(pEnd->wPacketSize,
				((u16)
				 (pUrb->transfer_buffer_length -
				  pUrb->actual_length)));
			DBG(3, "Sending %d bytes to %p\n", wFifoCount,
			    pFifoDest);
			MGC_HdrcLoadFifo(pBase, 0, wFifoCount, pFifoDest);

#ifdef MUSB_CONFIG_PROC_FS
			pEnd->dwTotalTxBytes += wFifoCount;
#endif
			COUNT(pEnd->dwTotalTxPackets);

			pEnd->dwRequestSize = wFifoCount;
			pUrb->actual_length += wFifoCount;
			if (pUrb->actual_length < pUrb->transfer_buffer_length) {
				bMore = TRUE;
			}
		}
	}

	return bMore;
}				/* end of function MGC_HdrcServiceHostDefault() */

/*
 * Handle default endpoint interrupt as host. Only called in IRQ time
 * from the LinuxIsr() interrupt service routine.
 *
 * @param pThis this
 */
void MGC_HdrcServiceDefaultEnd(MGC_LinuxCd * pThis)
{
	struct urb *pUrb;
	unsigned long flags;
	u16 wCsrVal, wCount;
	int status = 0;
	u8 *pBase = (u8 *) pThis->pRegs;
	MGC_LinuxLocalEnd *pEnd = &(pThis->aLocalEnd[0]);
	u8 bVal, bOutVal = 0, bComplete = FALSE, bError = FALSE;

	DBG(2, "<==\n");

	spin_lock_irqsave(&pEnd->Lock, flags);
	pUrb = MGC_GetCurrentUrb(pEnd);

	/* check URB */
#ifdef MUSB_PARANOID
	if (pUrb && (pUrb->hcpriv != pEnd)) {
		ERR("corrupt URB %p!!! from now on \"bad things will happen\"\n", pUrb);
	}
#endif

	spin_lock(&pThis->Lock);
	MGC_SelectEnd(pBase, 0);
	wCsrVal = MGC_ReadCsr16(pBase, MGC_O_HDRC_CSR0, 0);
	wCount = MGC_ReadCsr8(pBase, MGC_O_HDRC_COUNT0, 0);
	bVal = (u8) wCsrVal;

	DBG(2, "<== CSR0=%04x, wCount=%04x\n", wCsrVal, wCount);

	/* if we just did status stage, we are done */
	if (MGC_END0_STATUS == pThis->bEnd0Stage) {
		bComplete = TRUE;
	}

	/* prepare status */
	if ((MGC_END0_START == pThis->bEnd0Stage) && !wCount &&
	    (wCsrVal & MGC_M_CSR0_RXPKTRDY)) {
		DBG(2, "missed data\n");

		/* just started and got Rx with no data, so probably missed data */
		status = -EREMOTEIO;
		bError = TRUE;

		//bComplete = TRUE;
		MGC_WriteCsr16(pBase, MGC_O_HDRC_CSR0, 0, MGC_M_CSR0_FLUSHFIFO);
		MGC_WriteCsr16(pBase, MGC_O_HDRC_CSR0, 0, MGC_M_CSR0_FLUSHFIFO);
	}

	if (bVal & MGC_M_CSR0_H_RXSTALL) {
		DBG(2, "STALLING ENDPOINT\n");
		status = -EPIPE;
		bError = TRUE;

		SLOW_DEVICE_KLUDGE();
	} else if (bVal & MGC_M_CSR0_H_ERROR) {
		DEBUG_CODE(2, ERR("no response (error)\n");
			   MGC_HDRC_DUMPREGS(pThis, 0);
		    );

		status = -ETIMEDOUT;
		bError = TRUE;
	} else if (bVal & MGC_M_CSR0_H_NAKTIMEOUT) {
		DBG(2, "NAK timeout pEnd->bRetries=%d\n", pEnd->bRetries);

		if (++pEnd->bRetries < MUSB_MAX_RETRIES) {
			/* cover it up if retries not exhausted */
			MGC_WriteCsr16(pBase, MGC_O_HDRC_CSR0, 0, 0);
		} else {
			DEBUG_CODE(1, WARN("no response (NAK timeout)\n");
				   MGC_HDRC_DUMPREGS(pThis, 0);
			    );
			pEnd->bRetries = 0;
			status = -ETIMEDOUT;
			bError = TRUE;
		}
	}

	if (-ETIMEDOUT == status) {
		DBG(2, "aborting\n");

		/* use the proper sequence to abort the transfer */
		if (bVal & MGC_M_CSR0_H_REQPKT) {
			bVal &= ~MGC_M_CSR0_H_REQPKT;
			MGC_WriteCsr16(pBase, MGC_O_HDRC_CSR0, 0, bVal);
			bVal &= ~MGC_M_CSR0_H_NAKTIMEOUT;
			MGC_WriteCsr16(pBase, MGC_O_HDRC_CSR0, 0, bVal);
		} else {
			bVal |= MGC_M_CSR0_FLUSHFIFO;
			MGC_WriteCsr16(pBase, MGC_O_HDRC_CSR0, 0, bVal);
			MGC_WriteCsr16(pBase, MGC_O_HDRC_CSR0, 0, bVal);
			bVal &= ~MGC_M_CSR0_H_NAKTIMEOUT;
			MGC_WriteCsr16(pBase, MGC_O_HDRC_CSR0, 0, bVal);
		}

		MGC_WriteCsr8(pBase, MGC_O_HDRC_NAKLIMIT0, 0, 0);
	}

	if (bError) {
		DBG(3, "handling error\n");

		/* clear it */
		MGC_WriteCsr16(pBase, MGC_O_HDRC_CSR0, 0, 0);

		switch (pThis->bEnd0Stage) {
		case MGC_END0_START:
		case MGC_END0_OUT:
			COUNT(pEnd->dwErrorTxPackets);
			break;
		case MGC_END0_IN:
			COUNT(pEnd->dwErrorRxPackets);
			break;
		}
	}

	if (!pUrb) {
		/* stop endpoint since we have no place for its data, this
		 * SHOULD NEVER HAPPEN! */
		DBG(1, "no URB for end 0\n");

		MGC_WriteCsr16(pBase, MGC_O_HDRC_CSR0, 0, MGC_M_CSR0_FLUSHFIFO);
		MGC_WriteCsr16(pBase, MGC_O_HDRC_CSR0, 0, MGC_M_CSR0_FLUSHFIFO);
		MGC_WriteCsr16(pBase, MGC_O_HDRC_CSR0, 0, 0);

		/* start next URB that might be queued for it */
		spin_unlock_irqrestore(&pThis->Lock, flags);
		MGC_LinuxStartNextUrb(pThis, 0);
		spin_unlock_irq(&pEnd->Lock);

		return;
	}

	if (!bComplete && !bError) {

		/* call common logic and prepare response */
		if (MGC_HdrcServiceHostDefault(pThis, wCount, pUrb)) {
			/* more packets required */
			bOutVal = (MGC_END0_IN == pThis->bEnd0Stage) ?
			    MGC_M_CSR0_H_REQPKT : MGC_M_CSR0_TXPKTRDY;
			DBG(3, "Need more bytes bOutVal=%04x\n", bOutVal);
		} else {
			/* data transfer complete; perform status phase */
			bOutVal = MGC_M_CSR0_H_STATUSPKT |
			    (usb_pipeout(pUrb->pipe) ? MGC_M_CSR0_H_REQPKT :
			     MGC_M_CSR0_TXPKTRDY);
			/* flag status stage */
			pThis->bEnd0Stage = MGC_END0_STATUS;

			DBG(3,
			    "Data transfer complete, status phase bOutVal=%04x\n",
			    bOutVal);

		}
	}

	/* write CSR0 if needed */
	if (bOutVal) {
		MGC_WriteCsr16(pBase, MGC_O_HDRC_CSR0, 0, bOutVal);
	}

	/* call completion handler if done */
	if (bComplete || bError) {
		DBG(2, "completing cntrl URB %p, status=%d, len=%x\n",
		    pUrb, status, pUrb->actual_length);

/* FIXME this locking looks pretty strange; two locks, when one suffices?
 * Better to just have a single whole-controller lock.
 */

		pUrb->status = status;
		spin_unlock(&pThis->Lock);
		MGC_LinuxStartNextUrb(pThis, 0);
	} else {
		spin_unlock(&pThis->Lock);
	}

	spin_unlock_irqrestore(&pEnd->Lock, flags);

	DBG(2, "<==\n");
}				/* end of function MGC_HdrcServiceDefaultEnd() */

/*
 * Service a Tx-Available interrupt for the given endpoint
 * @param pThis instance pointer
 * @param bEnd local endpoint
 */
void MGC_HdrcServiceTxAvail(MGC_LinuxCd * pThis, u8 bEnd)
{
	int nPipe;
	u8 bDone = FALSE;
	u16 wTxCsrVal, wLength, wVal = 0;
	u8 *pBuffer;
	struct urb *pUrb;
	MGC_LinuxLocalEnd *pEnd = &(pThis->aLocalEnd[bEnd]);
	u32 status = 0;
	u8 *pBase = (u8 *) pThis->pRegs;
	unsigned long flags;

	DBG(2, "<==\n");

	spin_lock_irqsave(&pEnd->Lock, flags);
	pUrb = MGC_GetCurrentUrb(pEnd);

	spin_lock(&pThis->Lock);
	MGC_SelectEnd(pBase, bEnd);
	wVal = wTxCsrVal = MGC_ReadCsr16(pBase, MGC_O_HDRC_TXCSR, bEnd);
	spin_unlock(&pThis->Lock);

	if (bEnd)
		DBG(3, "TxAvail interrupt end=%d TxCSR=%x \n", bEnd,
			 wTxCsrVal);

#if MUSB_DEBUG > 0
	/* check URB */
	if (pUrb && (pUrb->hcpriv != pEnd)) {
		ERR("end %d has corrupt URB %lx!\n", bEnd, (unsigned long)pUrb);
		spin_unlock_irq(&pEnd->Lock);
		return;
	}
#endif

	nPipe = pUrb ? pUrb->pipe : 0;
	DBG(3, "end %d wTxCsrVal=%04x\n", bEnd, wTxCsrVal);

	/* check for errors */
	if (wTxCsrVal & MGC_M_TXCSR_H_RXSTALL) {
		DBG(3, "TX end %d stall\n", bEnd);

		/* stall; record URB status */
		status = -EPIPE;

		/* KLUDGE: race in usb-core?  */
		SLOW_DEVICE_KLUDGE();
	} else if (wTxCsrVal & MGC_M_TXCSR_H_ERROR) {
		WARN("TX data error on ep=%d\n", bEnd);

		status = -ETIMEDOUT;

		/* do the proper sequence to abort the transfer */
		wVal &= ~MGC_M_TXCSR_FIFONOTEMPTY;
		wVal |= MGC_M_TXCSR_FLUSHFIFO;

		spin_lock(&pThis->Lock);
		MGC_SelectEnd(pBase, bEnd);
		MGC_WriteCsr16(pBase, MGC_O_HDRC_TXCSR, bEnd, wVal);
		MGC_WriteCsr16(pBase, MGC_O_HDRC_TXCSR, bEnd, wVal);
		spin_unlock(&pThis->Lock);

		COUNT(pEnd->dwErrorTxPackets);
	} else if (wTxCsrVal & MGC_M_TXCSR_H_NAKTIMEOUT) {
		status = -ETIMEDOUT;

		if (++pEnd->bRetries < MUSB_MAX_RETRIES) {
			/* cover it up if retries not exhausted */
			spin_lock(&pThis->Lock);
			MGC_SelectEnd(pBase, bEnd);
			MGC_WriteCsr16(pBase, MGC_O_HDRC_CSR0, 0,
				       MGC_M_TXCSR_TXPKTRDY);
			spin_unlock(&pThis->Lock);
			spin_unlock_irqrestore(&pEnd->Lock, flags);
			return;
		}

		/* do the proper sequence to abort the transfer */
		wVal &= ~MGC_M_TXCSR_FIFONOTEMPTY;
		wVal |= MGC_M_TXCSR_FLUSHFIFO;
		spin_lock(&pThis->Lock);
		MGC_SelectEnd(pBase, bEnd);
		MGC_WriteCsr16(pBase, MGC_O_HDRC_TXCSR, bEnd, wVal);
		MGC_WriteCsr16(pBase, MGC_O_HDRC_TXCSR, bEnd, wVal);
		MGC_WriteCsr8(pBase, MGC_O_HDRC_TXINTERVAL, bEnd, 0);
		spin_unlock(&pThis->Lock);

		COUNT(pEnd->dwErrorTxPackets);
		pEnd->bRetries = 0;

		WARN("Device not responding on ep=%d\n", bEnd);
	}

	if (status) {

		/* reset error bits */
		wVal &= ~(MGC_M_TXCSR_H_ERROR | MGC_M_TXCSR_H_RXSTALL |
			  MGC_M_TXCSR_H_NAKTIMEOUT);
		wVal |= MGC_M_TXCSR_FRCDATATOG;

		spin_lock(&pThis->Lock);
		MGC_SelectEnd(pBase, bEnd);
		MGC_WriteCsr16(pBase, MGC_O_HDRC_TXCSR, bEnd, wVal);
		spin_unlock(&pThis->Lock);

		bDone = TRUE;
	} else if (pUrb->status == -EINPROGRESS) {
		/* pUrb->status!=-EINPROGRESS means request has been
		 * faulted, so we abort this transfer
		 */

#ifdef MUSB_CONFIG_PROC_FS
		pEnd->dwTotalTxBytes += pEnd->dwRequestSize;
#endif
		COUNT(pEnd->dwTotalTxPackets);

		/* see if more transactions are needed */
#ifndef	CONFIG_USB_INVENTRA_FIFO
		if (pEnd->pDmaChannel) {
			if (MGC_DMA_STATUS_FREE ==
			    pThis->pDmaController->pfDmaGetChannelStatus(pEnd->
									 pDmaChannel)
			    ) {
				pEnd->dwOffset +=
				    pEnd->pDmaChannel->dwActualLength;
			}
		} else {
			pEnd->dwOffset += pEnd->dwRequestSize;
		}
#else
		pEnd->dwOffset += pEnd->dwRequestSize;
#endif

		if (usb_pipeisoc(nPipe)) {
			/* isoch case */
			pUrb->iso_frame_desc[pEnd->dwIsoPacket].actual_length =
			    pEnd->dwRequestSize;
			if (++pEnd->dwIsoPacket >= pUrb->number_of_packets) {
				bDone = TRUE;
			} else {
				/* more to do */
				pBuffer = pUrb->transfer_buffer +
				    pUrb->iso_frame_desc[pEnd->dwIsoPacket].
				    offset;
				wLength =
				    pUrb->iso_frame_desc[pEnd->dwIsoPacket].
				    length;
				/* assume any DMA controller can move a maximum-size request */
				/* load next packet */
				MGC_HdrcLoadFifo(pBase, bEnd, wLength, pBuffer);
				pEnd->dwRequestSize = wLength;
			}
		} else {
			/* non-isoch */
			pBuffer = pUrb->transfer_buffer + pEnd->dwOffset;
			wLength = min(pEnd->wPacketSize,
				      (u16) (pUrb->transfer_buffer_length -
					     pEnd->dwOffset));
			if (pEnd->dwOffset >= pUrb->transfer_buffer_length) {
				/* sent everything; see if we need to send a null */
				if (!
				    ((pEnd->dwRequestSize == pEnd->wPacketSize)
				     && (pUrb->
					 transfer_flags & URB_ZERO_PACKET))) {
					bDone = TRUE;
				}
			} else {
				/* assume any DMA controller can move a maximum-size request */
				/* load next packet */
				MGC_HdrcLoadFifo(pBase, bEnd, wLength, pBuffer);
				pEnd->dwRequestSize = wLength;
			}
		}
	} else {
		/* unlinked */
		bDone = TRUE;
		status = pUrb->status;
	}

	if (bDone) {
		/* set status */
		pUrb->status = status;
		pUrb->actual_length = pEnd->dwOffset;

		if (usb_pipebulk(nPipe)) {
			/* we re-use bulk, so re-programming required */
			DBG(3, "Set pend_>ready to FALSE in Tx Avail \n");
			pEnd->bIsReady = FALSE;

			/* release claim if borrowed */
			if ((bEnd != pThis->bBulkTxEnd) &&
			    (pThis->bBulkTxEnd != pThis->bBulkRxEnd)) {
				pEnd->bIsClaimed = FALSE;
			}

			/* save data toggle */
			usb_settoggle(pUrb->dev, pEnd->bEnd, 1,
				      (wVal & MGC_M_TXCSR_H_DATATOGGLE) ? 1 :
				      0);
		}

		DBG(3, "completing Tx URB=%p, status=%d, len=%x\n",
		    pUrb, pUrb->status, pUrb->actual_length);

		/* complete current URB & start next */
		MGC_LinuxStartNextUrb(pThis, bEnd);
	} else {
		/* start next transaction */
		spin_lock(&pThis->Lock);
		MGC_SelectEnd(pBase, bEnd);
		MGC_WriteCsr16(pBase, MGC_O_HDRC_TXCSR, bEnd,
			       MGC_M_TXCSR_TXPKTRDY);
		spin_unlock(&pThis->Lock);
	}

	spin_unlock_irqrestore(&pEnd->Lock, flags);
	DBG(2, "==>\n");
}				/* end of function MGC_HdrcServiceTxAvail() */

/*
 * Service an Rx-Ready interrupt for the given endpoint; see section 18.2.1
 * of the manual for details.
 *
 * @param pThis instance pointer
 * @param bEnd local endpoint
 */
void MGC_HdrcServiceRxReady(MGC_LinuxCd * pThis, u8 bEnd)
{
	int nPipe;
	struct urb *pUrb;
	u16 wRxCsrVal, wVal = 0;
	u8 bIsochError = FALSE;
	u8 bDone = FALSE;
	MGC_LinuxLocalEnd *pEnd = &(pThis->aLocalEnd[bEnd]);
	u32 status = 0;
	u8 *pBase = (u8 *) pThis->pRegs;
	unsigned long flags;

	DBG(6, "<== locking end %d\n", bEnd);

	spin_lock_irqsave(&pEnd->Lock, flags);
	pUrb = MGC_GetCurrentUrb(pEnd);
	nPipe = pUrb ? pUrb->pipe : 0;

#ifdef MUSB_PARANOID
	/* check URB */
	if (pUrb && (pUrb->hcpriv != pEnd)) {
		ERR("end %d has corrupt URB %lx (hcpriv=%lx)!\n", bEnd,
		    (unsigned long)pUrb, (unsigned long)pUrb->hcpriv);
		/* about the urb? */
		spin_unlock_irqrestore(&pEnd->Lock, flags);
		return;
	}
#endif

	spin_lock(&pThis->Lock);
	MGC_SelectEnd(pBase, bEnd);
	wVal = wRxCsrVal = MGC_ReadCsr16(pBase, MGC_O_HDRC_RXCSR, bEnd);
	spin_unlock(&pThis->Lock);

	DBG(5, "<== end %d wRxCsrVal=%04x, pUrb->actual_length=%d\n", bEnd,
	    wRxCsrVal, pUrb->actual_length);

	/* check for errors, concurrent stall & unlink is not really
	 * handled yet! */
	if (wRxCsrVal & MGC_M_RXCSR_H_RXSTALL) {
		DBG(1, "RX end %d STALL\n", bEnd);

		/* stall; record URB status */
		status = -EPIPE;

	} else if (wRxCsrVal & MGC_M_RXCSR_H_ERROR) {
		DBG(1, "end %d Rx error\n", bEnd);
		DEBUG_CODE(4, MGC_HDRC_DUMPREGS(pThis, bEnd);
		    );

		status = -ETIMEDOUT;

		/* do the proper sequence to abort the transfer */
		spin_lock(&pThis->Lock);
		MGC_SelectEnd(pBase, bEnd);
		wVal &= ~MGC_M_RXCSR_H_REQPKT;
		MGC_WriteCsr16(pBase, MGC_O_HDRC_RXCSR, bEnd, wVal);
		MGC_WriteCsr8(pBase, MGC_O_HDRC_RXINTERVAL, bEnd, 0);
		spin_unlock(&pThis->Lock);

		COUNT(pEnd->dwErrorRxPackets);
	} else if (wRxCsrVal & MGC_M_RXCSR_DATAERROR) {
		status = -ETIMEDOUT;

		if (PIPE_BULK == pEnd->bTrafficType) {
			// REVISIT:  losing a NAK is an error, report it.
			// But nyet being ready (IN or OUT) is fine...

			/* cover it up if retries not exhausted, slow devices might
			 * not answer quickly enough: I was expecting a packet but the
			 * packet didn't come. The interrupt is generated after 3 failed
			 * attempts, it make MUSB_MAX_RETRIESx3 attempts total.
			 */
			if (++pEnd->bRetries < MUSB_MAX_RETRIES) {
				spin_lock(&pThis->Lock);
				MGC_SelectEnd(pBase, bEnd);
				MGC_WriteCsr16(pBase, MGC_O_HDRC_RXCSR, bEnd,
					       MGC_M_RXCSR_H_REQPKT);
				spin_unlock(&pThis->Lock);
				spin_unlock_irqrestore(&pEnd->Lock, flags);
				return;
			}

			wVal &= ~MGC_M_RXCSR_H_REQPKT;
			spin_lock(&pThis->Lock);
			MGC_SelectEnd(pBase, bEnd);
			MGC_WriteCsr16(pBase, MGC_O_HDRC_RXCSR, bEnd, wVal);
			MGC_WriteCsr8(pBase, MGC_O_HDRC_RXINTERVAL, bEnd, 0);
			spin_unlock(&pThis->Lock);

			pEnd->bRetries = 0;

			/* do the proper sequence to abort the transfer;
			 * am I dealing with a slow device maybe? */
			DBG(3, "end=%d device not responding\n", bEnd);
		} else if (PIPE_ISOCHRONOUS == pEnd->bTrafficType) {
			DBG(3, "bEnd=%d Isochronous error\n", bEnd);
			bIsochError = TRUE;
		}
		COUNT(pEnd->dwErrorRxPackets);
	}

	if (status) {
		if (-EPIPE != status) {
			DBG(2, "end %d Rx error, status=%d\n", bEnd, status);
			DEBUG_CODE(4, MGC_HDRC_DUMPREGS(pThis, bEnd);
			    );
		}

		/* reset error bits, all of them! */
		spin_lock(&pThis->Lock);
		MGC_SelectEnd(pBase, bEnd);
		wVal &= ~(MGC_M_RXCSR_H_ERROR | MGC_M_RXCSR_DATAERROR |
			  MGC_M_RXCSR_H_RXSTALL | MGC_M_RXCSR_RXPKTRDY);
		MGC_WriteCsr16(pBase, MGC_O_HDRC_RXCSR, bEnd, wVal);
		spin_unlock(&pThis->Lock);
		bDone = TRUE;
	} else if (pUrb->status == -EINPROGRESS) {

#ifdef CONFIG_USB_TI_CPPI_DMA
		/* we should not reach here. With CPPI dma we hit endpoint interrupts only
		   on error conditions */
		dbgPrint
		    ("CPPI Mode: Rx endpoint interrupt withour error ?? \n");
#endif

		/* if no errors, be sure a packet is ready for unloading */
		if (!wRxCsrVal & MGC_M_RXCSR_RXPKTRDY) {
			status = -EPROTO;
			DBG(3, "Rx interrupt with no errors or packet!\n");

			/* do the proper sequence to abort the transfer */
			spin_lock(&pThis->Lock);
			MGC_SelectEnd(pBase, bEnd);
			wVal &= ~MGC_M_RXCSR_H_REQPKT;
			MGC_WriteCsr16(pBase, MGC_O_HDRC_RXCSR, bEnd, wVal);
			spin_unlock(&pThis->Lock);
		}

		if (pUrb) {
			/* we are expecting traffic */
#ifdef CONFIG_USB_INVENTRA_DMA
			if (pEnd->pDmaChannel) {
				if (MGC_DMA_STATUS_FREE ==
				    pThis->pDmaController->
				    pfDmaGetChannelStatus(pEnd->pDmaChannel)) {
					pEnd->dwOffset +=
					    pEnd->pDmaChannel->dwActualLength;
				}
			}
#endif
			bDone = MGC_LinuxPacketRx(pThis, bEnd, bIsochError);
			DBG(4, "read data bDone=%d!\n", bDone);
		} else {
			/* THIS SHOULD NEVER HAPPEN */
			/* stop endpoint since we have no place for its data */
			DBG(1, "no URB on end %d Rx!\n", bEnd);

			spin_lock(&pThis->Lock);
			MGC_SelectEnd(pBase, bEnd);
			wVal |= MGC_M_RXCSR_FLUSHFIFO;
			wVal &= ~MGC_M_RXCSR_H_REQPKT;
			MGC_WriteCsr16(pBase, MGC_O_HDRC_RXCSR, bEnd, wVal);
			MGC_WriteCsr16(pBase, MGC_O_HDRC_RXCSR, bEnd, wVal);

			wVal &= ~(MGC_M_RXCSR_FLUSHFIFO | MGC_M_RXCSR_RXPKTRDY);
			MGC_WriteCsr16(pBase, MGC_O_HDRC_RXCSR, bEnd, wVal);

			/* start next URB that might be queued for it */
			spin_unlock(&pThis->Lock);

			MGC_LinuxStartNextUrb(pThis, bEnd);
			spin_unlock_irqrestore(&pEnd->Lock, flags);
			return;
		}
	} else {
		bDone = TRUE;
		status = pUrb->status;
	}

	if (bDone) {
		/* save data toggle if re-using */
		if (usb_pipebulk(nPipe)) {
			/* we re-use bulk, so re-programming required */
			DBG(3, "Set end %d ready to false\n", bEnd);
			pEnd->bIsReady = FALSE;

			/* release claim if borrowed */
			if ((bEnd != pThis->bBulkRxEnd) &&
			    (pThis->bBulkTxEnd != pThis->bBulkRxEnd)) {
				pEnd->bIsClaimed = FALSE;
			}

			/* save data toggle */
			usb_settoggle(pUrb->dev, pEnd->bEnd, 0,
				      (wVal & MGC_M_RXCSR_H_DATATOGGLE) ? 1 :
				      0);
		}

		/* set status */
		pUrb->status = status;
		DBG(3, "completing Rx URB %lx, end=%d, status=%d, len=%x\n",
		    (unsigned long)pUrb, bEnd, status, pUrb->actual_length);

		if (-EPIPE == status) {
			SLOW_DEVICE_KLUDGE();
		}

		/* this completes the urb as well */
		MGC_LinuxStartNextUrb(pThis, bEnd);
	} else {
		DBG(5, "not done yet, setup next in transaction\n");

		/* continue by clearing RxPktRdy and setting ReqPkt */
		spin_lock(&pThis->Lock);
		MGC_SelectEnd(pBase, bEnd);
		wVal &= ~MGC_M_RXCSR_RXPKTRDY;
		wVal |= MGC_M_RXCSR_H_REQPKT;
		MGC_WriteCsr16(pBase, MGC_O_HDRC_RXCSR, bEnd, wVal);
		spin_unlock(&pThis->Lock);
	}

	spin_unlock_irqrestore(&pEnd->Lock, flags);
}				/* end of function MGC_HdrcServiceRxReady() */

/*
 * Find an endpoint for the given pipe
 *
 * @param pThis instance pointer
 * @param pURB URB pointer
 * @return suitable local endpoint
 * @return -1 if nothing appropriate
 */
static int MGC_LinuxFindEnd(MGC_LinuxCd * pThis, struct urb *pUrb)
{
	MGC_LinuxLocalEnd *pEnd;
	u8 bDirOk, bTrafficOk, bSizeOk, bExact;
	// unsigned long flags;
	int nEnd;
	s32 dwDiff;
	u16 wBestDiff = 0xffff;
	u16 wBestExactDiff = 0xffff;
	int nBestEnd = -1;
	int nBestExactEnd = -1;
	unsigned int nPipe = pUrb->pipe;
	unsigned int nOut = usb_pipeout(nPipe);
	u16 wPacketSize = usb_maxpacket(pUrb->dev, nPipe, nOut);
	u8 bEnd = usb_pipeendpoint(nPipe);
	u8 bIsBulk = usb_pipebulk(nPipe);
	u8 bAddress = (u8) usb_pipedevice(nPipe);

	DBG(2, "<==\n");

	/* control is always EP0, and can always be queued */
	if (usb_pipecontrol(pUrb->pipe)) {
		DBG(3, "is a control pipe use ep0\n");
		return 0;
	}

	/* use a reserved one for bulk if any */
	if (bIsBulk) {
		if (nOut && pThis->bBulkTxEnd) {
			DBG(3, "use the bulk tx end\n");
			return pThis->bBulkTxEnd;
		} else if (!nOut && pThis->bBulkRxEnd) {
			DBG(3, "use the bulk rx end\n");
			return pThis->bBulkRxEnd;
		}
	}

	/* scan, remembering exact match and best match (bulk only) */
	for (nEnd = 1; nEnd < pThis->bEndCount; nEnd++) {
		pEnd = &(pThis->aLocalEnd[nEnd]);
		/* consider only if direction is possible */
		bDirOk = (nOut && pEnd->wMaxPacketSizeTx) ||
		    (!nOut && pEnd->wMaxPacketSizeRx);
		/* consider only if size is possible (in the given direction) */
		bSizeOk = (nOut && (pEnd->wMaxPacketSizeTx >= wPacketSize)) ||
		    (!nOut && (pEnd->wMaxPacketSizeRx >= wPacketSize));
		/* consider only traffic type */
		bTrafficOk = (usb_pipetype(nPipe) == pEnd->bTrafficType);

		if (bDirOk && bSizeOk) {
			/* convenient computations */
			dwDiff = nOut ? (pEnd->wMaxPacketSizeTx - wPacketSize) :
			    (pEnd->wMaxPacketSizeRx - wPacketSize);
			bExact = bTrafficOk && (pEnd->bEnd == bEnd) &&
			    (pEnd->bAddress == bAddress);

			/* bulk: best size match not claimed (we only claim periodic) */
			if (bIsBulk && !pEnd->bIsClaimed
			    && (wBestDiff > dwDiff)) {
				wBestDiff = (u16) dwDiff;
				nBestEnd = nEnd;
				/* prefer end already in right direction (to avoid flush) */
				if ((wBestExactDiff > dwDiff)
				    && (nOut == (int)pEnd->bIsTx)) {
					wBestExactDiff = (u16) dwDiff;
					nBestExactEnd = nEnd;
				}
			} else if (!bIsBulk && (nEnd != pThis->bBulkTxEnd) &&
				   (nEnd != pThis->bBulkRxEnd)) {
				/* periodic: exact match if present; otherwise best unclaimed */
				if (bExact) {
					nBestExactEnd = nEnd;
					break;
				} else if (!pEnd->bIsClaimed
					   && (wBestDiff > dwDiff)) {
					wBestDiff = (u16) dwDiff;
					nBestEnd = nEnd;
				}
			}
		}

	}

	DBG(2, "(out=%d, size=%d, proto=%d, addr=%d, end=%d, urb=%lx) = %d\n",
	    nOut, wPacketSize, usb_pipetype(nPipe),
	    bAddress, bEnd, (unsigned long)pUrb,
	    (nBestExactEnd >= 0) ? nBestExactEnd : nBestEnd);

	return (nBestExactEnd >= 0) ? nBestExactEnd : nBestEnd;
}				/* end of function MGC_LinuxFindEnd() */

/*
 * Submit an URB, either to the virtual root hut or to a real device;
 * it also checks the URB to make sure it's valid.
 * This is called by the Linux USB core. TSubmit Urb lock pThis
 * and the End to use, so make sure the caller releases its locks.
 *
 * also set the hcpriv member to the localEnd
 *
 * @param pUrb URB pointer (urb = USB request block data structure)
 * @return status code (0 succes)
 */
static int MGC_LinuxSubmitUrb(struct urb *pUrb, int iMemFlags)
{
	unsigned long		flags;
	unsigned int		pipe = pUrb->pipe;
	struct musb_hw_ep	*pEnd;
	struct musb		*pThis;
#if MUSB_DEBUG > 0
	struct usb_ctrlrequest	*pRequest;
#endif
	int			nEnd, idle = 0;
	int			status;

	DBG(2, "<== pUrb=%p\n", pUrb);

	/* NOTE: generic HCD framework has better sanity checks */

#ifdef MUSB_PARANOID
	if (!pUrb || !pUrb->dev || !pUrb->dev->bus || !pUrb->dev->bus->hcpriv) {
		DBG(3, "invalid URB\n");
		DEBUG_CODE(3, if (!pUrb)
				   printk(KERN_ERR "pUrb is null\n");
			   else if (!pUrb->dev)
				   printk(KERN_ERR "pUrb->dev is null\n");
			   else if (!pUrb->dev->bus)
				   printk(KERN_ERR "pUrb->dev->bus is null\n");
			   else if (!pUrb->dev->bus->hcpriv)
				   printk(KERN_ERR "null instance ptr\n"););
		return -EINVAL;
	}
#endif

	pThis = (struct musb *) pUrb->dev->bus->hcpriv;

#ifdef MUSB_PARANOID
	if (MGC_ISCORRUPT(pThis)) {
		ERR("pThis corrupted: stopping before submit\n");
		MGC_HdrcStop(pThis);
		MUSB_ERR_MODE(pThis, MUSB_ERR_CORRUPTED);
		return -ENOENT;
	}
#endif

	/* if it is a request to the virtual root hub, delegate */
	if (!pUrb->dev->parent)
		return MGC_VirtualHubSubmitUrb(&(pThis->RootHub), pUrb);

	/* find appropriate local endpoint to do it */
	nEnd = MGC_LinuxFindEnd(pThis, pUrb);
	DBG(3, "pUrb=%p, end=%d, bufsize=%x\n", pUrb,
	    nEnd, pUrb->transfer_buffer_length);
	if (MUSB_IS_ERR(pThis)) {
		ERR("CANNOT SUBMIT AN URB WHILE IN ERROR MODE\n");
		return -ENODEV;
	}
#ifdef MUSB_PARANOID
	if (nEnd < 0) {
		ERR("no endpoint for proto=%d, addr=%d, end=%d\n",
		    usb_pipetype(pipe), usb_pipedevice(pipe),
		    usb_pipeendpoint(pipe));
		return -EIO;
	}
#endif

	DEBUG_CODE(2, if (!nEnd) {
		   pRequest = (struct usb_ctrlrequest *)pUrb->setup_packet;
		   printk(KERN_INFO
			  "%s: bRequestType=%02x, bRequest=%02x, wLength=%04x\n",
			  __FUNCTION__, pRequest->bRequestType,
			  pRequest->bRequest,
			  le16_to_cpu(pRequest->wLength));}) ;

	/* if we are not HOST, error */
	if (!MUSB_IS_HST(pThis)) {
		int ep = usb_pipeendpoint(pipe);
		ERR("not host: aborting urb to dev=%d, ep=%d\n",
		    usb_pipedevice(pipe), ep);
		if (ep) {

		}
		return -ENODEV;
	}

	/* if no root device, assume this must be it */
	if (!pThis->pRootDevice) {
		pThis->pRootDevice = pUrb->dev;
	}

#if 0
	/* reserve new periodic bandwidth, unless this endpoint has
	 * already had its bandwidth reserved.
	 *
	 * FIXME but do it right.  For now we can just rely on the
	 * fact that we don't make enough endpoints available to
	 * overcommit bandwidth except maybe for fullspeed ISO.
	 * Two low-rate interrupt transfers and we're full...
	 */
	if (usb_pipeisoc(pipe) || usb_pipeint(pipe)) {
		int bustime = usb_check_bandwidth(pUrb->dev, pUrb);

		if (bustime < 0) {
			return bustime;
		}

		usb_claim_bandwidth(pUrb->dev, pUrb, bustime,
				    usb_pipeisoc(pipe) ? 1 : 0);

	}
#endif

	DBG(3, "end %d claimed for type=%d, addr=%d, end=%d\n", nEnd,
	    usb_pipetype(pipe), usb_pipedevice(pipe), usb_pipeendpoint(pipe));

	pEnd = &(pThis->aLocalEnd[nEnd]);

	/* increment urb's reference count, we now control it. */
	pUrb = usb_get_urb(pUrb);

#ifdef CONFIG_USB_INVENTRA_FIFO
	if ((pUrb->transfer_buffer == 0) && (pUrb->transfer_buffer_length!=0)) {
		printk("Submit Urb with buffer address of zero\n");
		printk("urb:%p dma=0x%x buff=%p\n",pUrb,
				pUrb->transfer_dma,pUrb->transfer_buffer);
		BUG();
	}
#endif

	/* setup DMA mapping, if needed (does dcache sync) */
	if (is_dma_capable()
			&& pThis->controller->dma_mask
			&& pUrb->transfer_buffer_length != 0
			&& !(pUrb->transfer_flags & URB_NO_TRANSFER_DMA_MAP)) {
		pUrb->transfer_dma = dma_map_single(pThis->controller,
				pUrb->transfer_buffer,
				pUrb-> transfer_buffer_length,
				usb_pipein(pUrb->pipe)
					? DMA_FROM_DEVICE
					: DMA_TO_DEVICE);
	}

	/* queue & start */
	spin_lock_irqsave(&pEnd->Lock, flags);
	idle = list_empty(&pEnd->urb_list);	/* shall I start the urb? */
	pEnd->bIsClaimed = usb_pipeisoc(pUrb->pipe)
			|| usb_pipeint(pUrb->pipe);

	/* assign the URB to the endpoint */
	spin_lock(&pUrb->lock);
	if (unlikely(pUrb->reject)) {
		INIT_LIST_HEAD(&pUrb->urb_list);
		status = -EPERM;
	} else {
		status = 0;
		pUrb->status = -EINPROGRESS;
		pUrb->actual_length = 0;
		pUrb->error_count = 0;
		list_add_tail(&pUrb->urb_list, &pEnd->urb_list);
		pUrb->hcpriv = pEnd;
		atomic_inc(&pUrb->use_count);
	}
	spin_unlock(&pUrb->lock);
	if (status)
		goto done;

	DBG(3, "queued URB %p (current %p) for end %d (device=%p, proto=%d)\n",
	    pUrb, MGC_GetCurrentUrb(pEnd), nEnd, pUrb->dev, usb_pipetype(pipe));
	spin_unlock_irqrestore(&pEnd->Lock, flags);
	if (idle) {
		MGC_LinuxStartUrb(pThis, (u8) nEnd);
	}
#ifdef MUSB_PARANOID
	if (MGC_ISCORRUPT(pThis)) {
		ERR("stopping after submit\n");
		MGC_HdrcStop(pThis);
		MUSB_ERR_MODE(pThis, MUSB_ERR_CORRUPTED);
		return -ENOENT;
	}
#endif

	DEBUG_CODE(5, dump_urb(pUrb);
	    );
done:
	return status;
}

/*
 * Cancel URB.
 * @param pUrb URB pointer
 */
static int MGC_LinuxUnlinkUrb(struct urb *pUrb, int status)
{
	int			pipe;
	struct musb		*pThis;
	struct musb_hw_ep	*pEnd;
	struct urb		*head;
	unsigned long		flags;

	DBG(2, "<== pUrb=%p\n", pUrb);

	/* sanity */
	if (!pUrb || !pUrb->hcpriv) {
		return -EINVAL;
	}

	if (!pUrb->dev || !pUrb->dev->bus) {
		return -ENODEV;
	}

	pThis = pUrb->dev->bus->hcpriv;
	if (!pThis) {
		ERR("pThis is null: stopping before unlink\n");
		return -ENODEV;
	}
#ifdef MUSB_PARANOID
	if (MGC_ISCORRUPT(pThis)) {
		ERR("pThis corrupted: stopping before unlink\n");
		MGC_HdrcStop(pThis);
		MUSB_ERR_MODE(pThis, MUSB_ERR_CORRUPTED);
		return -EINVAL;
	}
#endif
	spin_lock_irqsave(&pUrb->lock, flags);
	if (pUrb->status != -EINPROGRESS)
		status = -EBUSY;
	else {
		pUrb->status = status;
		status = 0;
	}
	spin_unlock_irqrestore(&pUrb->lock, flags);
	if (status)
		return status;

	/* if it is a request to the virtual root hub, delegate */
	/* if (usb_pipedevice (pUrb->pipe) == pThis->RootHub.bAddress) */
	if (!pUrb->dev->parent) {
		return MGC_VirtualHubUnlinkUrb(&(pThis->RootHub), pUrb);
	}

	/* stop local end if valid */
	pEnd = pUrb->hcpriv;
#ifdef MUSB_PARANOID
	if ((pEnd < &(pThis->aLocalEnd[0])) ||
	    (pEnd > &(pThis->aLocalEnd[MUSB_C_NUM_EPS - 1]))) {
		/* somehow, we got passed a dangling URB pointer */
		return -EINVAL;
	}
#endif

	pipe = pUrb->pipe;

	spin_lock_irqsave(&pEnd->Lock, flags);
	head = MGC_GetCurrentUrb(pEnd);
	if (!head) {
		status = -EINVAL;
		goto done;
	}

	/* There's an IRQ pending for the urb at list head, and when
	 * that fires, it scrubs out hardware state as needed.  Else
	 * for anything not locked into DMA queues, unlink is easy.
	 */
	if (head != pUrb) {
		spin_unlock(&pEnd->Lock);
		musb_giveback(pUrb, NULL, 0);
		spin_lock(&pEnd->Lock);
	}

	// FIXME else ... for DMA, the transfer must be canceled.

#ifdef MUSB_PARANOID
	if (MGC_ISCORRUPT(pThis)) {
		ERR("stopping after unlink\n");
		MGC_HdrcStop(pThis);
		MUSB_ERR_MODE(pThis, MUSB_ERR_CORRUPTED);
		status = -EINVAL;
	}
#endif

done:
	spin_unlock_irqrestore(&pEnd->Lock, flags);
	return status;
}				/* end of function MGC_LinuxUnlinkUrb() */


/* disable an endpoint */
static void MGC_LinuxDisable(struct usb_device *udev, int bEndpointAddress)
{
	dev_dbg(&udev->dev, "FIXME, disable ep%d%s\n",
			bEndpointAddress & 0x0f,
			(bEndpointAddress & USB_DIR_IN) ? "in" : "out");
}

static void *MGC_LinuxBufferAlloc(struct usb_bus *pBus, size_t nSize,
				  int iMemFlags, dma_addr_t * pDmaAddress)
{
	/* for now, just kmalloc it */
	MGC_LinuxCd *pThis = (MGC_LinuxCd *) pBus->hcpriv;

#ifdef MUSB_PARANOID
	if (!pThis) {
		ERR("cannot find the controller, cannot allocate the memory\n");
		return 0;
	}
#endif

	DBG(2, "<== allocating memory on bus (%s), %d, pDmaAddress=%p\n",
	    pBus->bus_name, pBus->busnum, pDmaAddress);
	return MGC_AllocBufferMemory(pThis, nSize, iMemFlags, pDmaAddress);
}

static void MGC_LinuxBufferFree(struct usb_bus *pBus, size_t nSize,
				void *address, dma_addr_t dma)
{
	MGC_LinuxCd *pThis = (MGC_LinuxCd *) pBus->hcpriv;

#ifdef MUSB_PARANOID
	if (!pThis) {
		kfree(address);
		ERR("cannot find the controller, cannot free the memory properly\n");
		return;
	}
#endif

	MGC_FreeBufferMemory(pThis, nSize, address, dma);
}

/*
 * Get the current frame number
 * @param usb_dev pointer to USB device
 * @return frame number
 */
static int MGC_LinuxGetFrameNumber(struct usb_device *pDevice)
{
	MGC_LinuxCd *pThis = (MGC_LinuxCd *) pDevice->bus->hcpriv;
	u8 *pBase = (u8 *) pThis->pRegs;

	return (int)MGC_Read16(pBase, MGC_O_HDRC_FRAME);
}

/* FIXME -- switchover to use the hcd glue layer;
 * define root hub support with hub suspend/resume calls
 *
 * latest mentor code has some of that, plus ULPI calls
 *
 * also, kernel.org interfaces now pass a usb_host_endpoint
 * handle around; "struct hcd_dev" is gone.
 */
struct usb_operations MGC_LinuxOperations = {
	.get_frame_number	= MGC_LinuxGetFrameNumber,
	.submit_urb		= MGC_LinuxSubmitUrb,
	.unlink_urb		= MGC_LinuxUnlinkUrb,
	.buffer_alloc		= MGC_LinuxBufferAlloc,
	.buffer_free		= MGC_LinuxBufferFree,
	.disable		= MGC_LinuxDisable
};
