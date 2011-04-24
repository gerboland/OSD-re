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

#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/timer.h>
#include <linux/delay.h>
#include <linux/spinlock.h>
#include <linux/device.h>
#include <linux/init.h>

#include "musbdefs.h"

MGC_GadgetLocalEnd MGC_aGadgetLocalEnd[MUSB_C_NUM_EPS];	/* counters and queues */

/***********************************************************************/

static int MGC_GadgetGetFrame(struct usb_gadget *gadget)
{
	struct musb	*pThis = gadget_to_musb(gadget);

#ifdef MUSB_PARANOID
	if (!pThis) {
		ERR("Gadget not initialized, pThis=NULL\n");
		return -EINVAL;
	}
#endif
	return (int)MGC_Read16(pThis->pRegs, MGC_O_HDRC_FRAME);
}

static int MGC_GadgetWakeup(struct usb_gadget *gadget)
{
	u8 power;
	struct musb	*pThis = gadget_to_musb(gadget);

	if (!pThis->bMayWakeup)
		return -EINVAL;

	// XXX spin_lock 

	power = MGC_Read8(pThis->pRegs, MGC_O_HDRC_POWER);
	power |= MGC_M_POWER_RESUME;
	MGC_Write8(pThis->pRegs, MGC_O_HDRC_POWER, power);

	power -= MGC_M_POWER_RESUME;
	mdelay(10);
	MGC_Write8(pThis->pRegs, MGC_O_HDRC_POWER, power);

	// XXX spin_unlock 

	return 0;
}

static int MGC_GadgetSetSelfPowered(struct usb_gadget *gadget, int is_selfpowered)
{
	struct musb	*pThis = gadget_to_musb(gadget);

	DBG(2, "<==\n");

#ifdef MUSB_PARANOID
	if (!pThis) {
		ERR("Gadget not initialized, pThis=NULL\n");
		return -EINVAL;
	}
#endif

	pThis->bIsSelfPowered = !!is_selfpowered;
	return 0;
}

static void musb_pullup(struct musb *musb, int is_on)
{
	u8 power;

	power = MGC_Read8(musb->pRegs, MGC_O_HDRC_POWER);
	if (is_on)
		power |= MGC_M_POWER_SOFTCONN;
	else
		power &= ~MGC_M_POWER_SOFTCONN;

	DBG(3, "gadget %s %sconnecting to host\n",
	    musb->pGadgetDriver->function, is_on ? "" : "dis");
	MGC_Write8(musb->pRegs, MGC_O_HDRC_POWER, power);
}

static int MGC_GadgetVbusSession(struct usb_gadget *gadget, int is_active)
{
	DBG(2, "<= %s =>\n", __FUNCTION__);

	// FIXME iff driver's softconnect flag is set (as it is during probe,
	// though that can clear it), just musb_pullup().

	return -EINVAL;
}

static int MGC_GadgetVbusDraw(struct usb_gadget *gadget, unsigned mA)
{
	/* FIXME -- delegate to otg_transciever logic */

	DBG(2, "<= vbus_draw %u =>\n", mA);
	return 0;
}

static int MGC_GadgetPullup(struct usb_gadget *gadget, int is_on)
{
	// FIXME don't always pull up.  this changes the
	// softconnect flag, and that only implies pullup
	// if we sense vbus ... 

	// XXX spin_lock 

	musb_pullup(gadget_to_musb(gadget), is_on);

	// XXX spin_unlock 
	return 0;
}

struct usb_gadget_ops MGC_GadgetOperations = {
	.get_frame		= MGC_GadgetGetFrame,
	.wakeup			= MGC_GadgetWakeup,
	.set_selfpowered	= MGC_GadgetSetSelfPowered,
	.vbus_session		= MGC_GadgetVbusSession,
	.vbus_draw		= MGC_GadgetVbusDraw,
	.pullup			= MGC_GadgetPullup,
};

/***********************************************************************/

void MGC_GadgetResume(MGC_LinuxCd * pThis)
{
	DBG(4, "<==\n");
	if (pThis->pGadgetDriver && pThis->pGadgetDriver->resume) {
		pThis->pGadgetDriver->resume(&pThis->g);
	}
}

void MGC_GadgetSuspend(MGC_LinuxCd * pThis)
{
	DBG(4, "<==\n");
	if (pThis->pGadgetDriver && pThis->pGadgetDriver->suspend) {
		pThis->pGadgetDriver->suspend(&pThis->g);
	}
}

void MGC_GadgetDisconnect(MGC_LinuxCd * pThis)
{
	DBG(4, "<==\n");
	if (pThis->pGadgetDriver && pThis->pGadgetDriver->disconnect) {
		pThis->pGadgetDriver->disconnect(&pThis->g);
	}
	pThis->g.speed = USB_SPEED_UNKNOWN;
}

void MGC_GadgetReset(MGC_LinuxCd * pThis)
{
	const u8 *pBase = (u8 *) pThis->pRegs;
	u8 devctl = MGC_Read8(pBase, MGC_O_HDRC_DEVCTL);

#ifdef MUSB_PARANOID
	ASSERT_SPINLOCK_UNLOCKED(&pThis->Lock);
	ASSERT_SPINLOCK_UNLOCKED(&MGC_aGadgetLocalEnd[0]);
#endif

	DBG(3, "<== %s addr=%x driver '%s'\n",
			(devctl & MGC_M_DEVCTL_BDEVICE)
				? "B-Device" : "A-Device",
			MGC_Read8(pBase, MGC_O_HDRC_FADDR),
			pThis->pGadgetDriver
				? pThis->pGadgetDriver->driver.name
				: NULL
			);

	/* HR does NOT clear itself */
	if (devctl & MGC_M_DEVCTL_HR) {
		MGC_Write8(pBase, MGC_O_HDRC_DEVCTL, MGC_M_DEVCTL_SESSION);
	}

	/* start in USB_STATE_DEFAULT */
	pThis->bAddress = 0;
	pThis->bEnd0Stage = MGC_END0_STAGE_SETUP;

	pThis->bMayWakeup = 0;
	pThis->g.b_hnp_enable = 0;
	pThis->g.a_alt_hnp_support = 0;
	pThis->g.a_hnp_support = 0;

	pThis->g.speed = USB_SPEED_UNKNOWN;

	if (pThis->pGadgetDriver) {
		u8 bEnd;
		u8 power = MGC_Read8(pBase, MGC_O_HDRC_POWER);

		pThis->g.speed = (power & MGC_M_POWER_HSMODE)
		    ? USB_SPEED_HIGH : USB_SPEED_FULL;

		for (bEnd = 1; bEnd < pThis->bEndCount; bEnd++) {
			(void)usb_ep_disable(&MGC_aGadgetLocalEnd[bEnd]
					     .end_point);
		}

	}

#ifdef MUSB_MONITOR_DATA
	MGC_DisableDebug();
#endif

	DBG(2, "==>\n");
}

/* --------------------------------------------------------------------
 * Buffer functions; using the buffer functions in plat_uds.c
 *
 * -------------------------------------------------------------------- */

/**
 * Allocate a the memory for buffer allocated for an given 
 * endpoint; free the memeory with MGC_GadgetAllocateBuffer().
 * @param ep the end point the buffer should be freed for
 * @param bytes the bytes to allocate
 * @param dma the dma address
 * @param gfp_flags glafs memory should be allocated with
 */
void *MGC_GadgetAllocBuffer(struct usb_ep *ep, unsigned bytes,
			    dma_addr_t * dma, int gfp_flags)
{
	struct musb_ep *musb_ep = to_musb_ep(ep);

	return MGC_AllocBufferMemory(musb_ep->pThis, bytes, gfp_flags, dma);
}

/**
 * Free a buffer allocated from MGC_GadgetAllocateBuffer()
 * @param epo the end point the buffer should be freed for
 * @param address the address to free
 * @param dma the dma address
 * @param bytes the bytes to free
 */
void MGC_GadgetFreeBuffer(struct usb_ep *ep, void *address, dma_addr_t dma,
			  unsigned bytes)
{
	struct musb_ep *musb_ep = to_musb_ep(ep);

	MGC_FreeBufferMemory(musb_ep->pThis, bytes, address, dma);
}

/**********************************************************************
 * FIFO functions
 *
 */

void MGC_GadgetFifoFlush(struct usb_ep *ep)
{
	struct musb_ep	*musb_ep = to_musb_ep(ep);
	MGC_LinuxCd	*pThis;
	u8		*pBase;
	u8		nEnd;
	unsigned long	flags;
	u16		wCsr, wIntrTxE;

	if (!ep)
		return;

	pThis = musb_ep->pThis;
	pBase = pThis->pRegs;
	nEnd = musb_ep->bEndNumber;

#ifdef MUSB_PARANOID
	if (!pThis) {
		ERR("Gadget not initialized, pThis=NULL\n");
		return;
	}

	ASSERT_SPINLOCK_UNLOCKED(&pThis->Lock);
#endif

	spin_lock_irqsave(&(pThis->Lock), flags);
	MGC_SelectEnd(pBase, (u8) nEnd);

	/* disable interrupts */
	wIntrTxE = MGC_Read16(pBase, MGC_O_HDRC_INTRTXE);
	MGC_Write16(pBase, MGC_O_HDRC_INTRTXE, wIntrTxE & ~(1 << nEnd));

	if (nEnd) {
		if (musb_ep->bIsTx) {
			wCsr = MGC_ReadCsr16(pBase, MGC_O_HDRC_TXCSR, nEnd);
			wCsr |= MGC_M_TXCSR_FLUSHFIFO;
			MGC_WriteCsr16(pBase, MGC_O_HDRC_TXCSR, nEnd, wCsr);
			MGC_WriteCsr16(pBase, MGC_O_HDRC_TXCSR, nEnd, wCsr);
		} else {
			wCsr = MGC_ReadCsr16(pBase, MGC_O_HDRC_RXCSR, nEnd);
			wCsr |= MGC_M_RXCSR_FLUSHFIFO;
			MGC_WriteCsr16(pBase, MGC_O_HDRC_RXCSR, nEnd, wCsr);
			MGC_WriteCsr16(pBase, MGC_O_HDRC_RXCSR, nEnd, wCsr);
		}
	} else {
		MGC_WriteCsr16(pBase, MGC_O_HDRC_CSR0, 0, MGC_M_CSR0_FLUSHFIFO);
	}

	/* re-enable interrupt */
	MGC_Write16(pBase, MGC_O_HDRC_INTRTXE, wIntrTxE);
	spin_unlock_irqrestore(&(pThis->Lock), flags);
}

int MGC_GadgetFifoStatus(struct usb_ep *ep)
{
	struct musb_ep		*musb_ep = to_musb_ep(ep);
	struct musb		*pThis = musb_ep->pThis;
	int			bEnd = musb_ep->bEndNumber;
	unsigned long flags;
	int nResult = 0;
	u8 *pBase = pThis->pRegs;

	DBG(2, "<==\n");

#ifdef MUSB_PARANOID
	ASSERT_SPINLOCK_UNLOCKED(&pThis->Lock);
#endif

	spin_lock_irqsave(&(pThis->Lock), flags);
	MGC_SelectEnd(pBase, bEnd);
	nResult = MGC_ReadCsr16(pBase,
				(bEnd) ? MGC_O_HDRC_RXCOUNT : MGC_O_HDRC_COUNT0,
				bEnd);
	spin_unlock_irqrestore(&(pThis->Lock), flags);

	DBG(2, "==> %d\n", nResult);
	return nResult;
}


/*
 * Initialize the endpoints exposed to peripheral drivers, with backlinks
 * to the rest of the driver state.
 */
void __init MGC_InitGadgetEndPoints(MGC_LinuxCd * pThis)
{
	u8 bEnd;

	/* intialize endpoint list just once */
	INIT_LIST_HEAD(&(pThis->g.ep_list));

	DBG(2, "Initializing %d end points\n", pThis->bEndCount);

	/* NOTE:  this initial version should grow so that we could set
	 * up ** both directions ** for hardware endpoints, each with
	 * nonshared fifo:  "ep1in" and "ep1out" running concurrently.
	 * (An musb_hw_ep could appear as two musb_ep structs...)
	 */
	for (bEnd = 0; bEnd < pThis->bEndCount; bEnd++) {
		struct musb_ep		*ep = &MGC_aGadgetLocalEnd[bEnd];
		struct musb_hw_ep	*hw_ep = &pThis->aLocalEnd[bEnd];

		memset(ep, 0, sizeof *ep);

#if MUSB_DEBUG>0
		ep->wPadFront = MGC_LOCAL_PAD;
#endif

		ep->bEndNumber = bEnd;
		ep->pThis = pThis;
		ep->hw_ep = hw_ep;

		spin_lock_init(&ep->Lock);
		INIT_LIST_HEAD(&ep->req_list);

		sprintf(ep->name, "ep%d%s", bEnd,
				(!bEnd || hw_ep->bIsSharedFifo) ? "" : (
					hw_ep->wMaxPacketSizeTx
					? "in" : "out"));
		ep->end_point.name = ep->name;
		ep->end_point.ops = &MGC_GadgetEndpointOperations;
		INIT_LIST_HEAD(&ep->end_point.ep_list);
		ep->bInactive = MUSB_GADGET_EP_ACTIVE;
		ep->end_point.maxpacket = max(hw_ep->wMaxPacketSizeTx,
						hw_ep->wMaxPacketSizeRx);

		if (bEnd) {
			if (!hw_ep->bIsSharedFifo
					&& hw_ep->wMaxPacketSizeTx
					&& hw_ep->wMaxPacketSizeRx)
				pr_debug("%s periph: only using TX side "
						"of HW endpoint %d\n",
						musb_driver_name, bEnd);

			/* endpoints other than ep0 belong in ep_list */
			list_add_tail(&ep->end_point.
				      ep_list, &(pThis->g.ep_list));
		} else {
			pThis->g.ep0 = &ep->end_point;

			/* REVISIT ep0-specific queue/fifo ops may be
			 * appropriate in this driver.
			 */
		}
		pr_debug("%s periph: %s, maxpacket %d\n",
				musb_driver_name, ep->end_point.name,
				ep->end_point.maxpacket);
	}
}
