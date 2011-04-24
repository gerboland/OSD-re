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
#include <linux/spinlock.h>
#include <linux/init.h>
#include <linux/device.h>

#include "musbdefs.h"

/* FROM: usb_gadget.h
 *
 * Accordingly, the driver's setup() callback must always implement all
 * get_descriptor requests, returning at least a device descriptor and
 * a configuration descriptor.  Drivers must make sure the endpoint
 * descriptors match any hardware constraints. Some hardware also constrains
 * other descriptors.
 *
 * The driver's setup() callback must also implement set_configuration,
 * and should also implement set_interface, get_configuration, and
 * get_interface.  Setting a configuration (or interface) is where
 * endpoints should be activated or (config 0) shut down.
 */

static inline int
forward_to_driver(struct musb *pThis,
		  const struct usb_ctrlrequest *pControlRequest)
{
	if (!pThis->pGadgetDriver)
		return -EOPNOTSUPP;
	return pThis->pGadgetDriver->setup(&pThis->g, pControlRequest);
}

static inline int
service_out_request(struct musb *pThis, struct usb_ctrlrequest *req)
{
#ifdef MUSB_PARANOID
	ASSERT_SPINLOCK_UNLOCKED(&pThis->Lock);
	ASSERT_SPINLOCK_UNLOCKED(&MGC_aGadgetLocalEnd[0]);
#endif
	return forward_to_driver(pThis, req);
}

/* handle a standard GET_STATUS request */
static inline int service_tx_status_request(
	struct musb *pThis,
	const struct usb_ctrlrequest *pControlRequest)
{
	u8 handled = 1;
	u8 bResult[2], bEnd = 0;
	const u8 *pBase = (u8 *) pThis->pRegs;
	const u8 bRecip = pControlRequest->bRequestType & USB_RECIP_MASK;
	u16 wTest;

	/* ack the request */
	DBG(3, "acking request %s\n", decode_csr0(MGC_M_CSR0_P_SVDRXPKTRDY));

	spin_lock(&pThis->Lock);
	MGC_SelectEnd(pBase, 0);
	MGC_WriteCsr16(pBase, MGC_O_HDRC_CSR0, 0, MGC_M_CSR0_P_SVDRXPKTRDY);
	spin_unlock(&pThis->Lock);

	bResult[1] = 0;

	switch (bRecip) {
	case USB_RECIP_DEVICE:
		bResult[0] = pThis->bIsSelfPowered << USB_DEVICE_SELF_POWERED;
		bResult[0] |= pThis->bMayWakeup << USB_DEVICE_REMOTE_WAKEUP;
#ifdef CONFIG_USB_MUSB_OTG
		if (pThis->g.is_otg) {
			bResult[0] |= pThis->g.b_hnp_enable
			    << USB_DEVICE_B_HNP_ENABLE;
			bResult[0] |= pThis->g.a_alt_hnp_support
			    << USB_DEVICE_A_ALT_HNP_SUPPORT;
			bResult[0] |= pThis->g.a_hnp_support
			    << USB_DEVICE_A_HNP_SUPPORT;
		}
#endif
		break;

	case USB_RECIP_INTERFACE:
		bResult[0] = 0;
		break;

	case USB_RECIP_ENDPOINT:
		bEnd = (u8) pControlRequest->wIndex;

		spin_lock(&pThis->Lock);
		MGC_SelectEnd(pBase, bEnd);
		wTest = MGC_ReadCsr16(pBase, MGC_O_HDRC_TXCSR, bEnd);
		MGC_SelectEnd(pBase, 0);
		spin_unlock(&pThis->Lock);

		bResult[0] = (wTest & MGC_M_TXCSR_P_SENDSTALL) ? 1 : 0;
		break;

	default:
		/* class, vendor, etc ... delegate */
		handled = 0;
		break;
	}

	/* send it out! (this will trigger the ep0 completion IRQ)  */
	if (handled > 0) {
		u16	len = le16_to_cpu(pControlRequest->wLength);

		if (len > 2)
			len = 2;
		pThis->bEnd0Stage = MGC_END0_STAGE_STATUSOUT;
		MGC_HdrcLoadFifo(pBase, 0, len, bResult);

		spin_lock(&pThis->Lock);
		MGC_SelectEnd(pBase, bEnd);
		MGC_WriteCsr16(pBase, MGC_O_HDRC_CSR0, 0,
			       MGC_M_CSR0_TXPKTRDY | MGC_M_CSR0_P_DATAEND);
		spin_unlock(&pThis->Lock);
	}
	return handled;
}

/*
 * handle a control-IN request, the end0 buffer contains the current request
 * that is supposed to be a standard control request. Assumes the fifo to
 * be at least 2 bytes long.
 *
 * @return 0 if the request was NOT HANDLED,
 * < 0 when error
 * > 0 when the request is processed 
 */
static int
service_in_request(struct musb *pThis,
		   const struct usb_ctrlrequest *pControlRequest)
{
	int handled = 0;	/* not handled */

#ifdef MUSB_PARANOID
	ASSERT_SPINLOCK_UNLOCKED(&pThis->Lock);
	ASSERT_SPINLOCK_UNLOCKED(&MGC_aGadgetLocalEnd[0].Lock);
#endif

	if ((pControlRequest->bRequestType & USB_TYPE_MASK)
			== USB_TYPE_STANDARD) {
		switch (pControlRequest->bRequest) {
		case USB_REQ_GET_STATUS:
			handled = service_tx_status_request(pThis,
					pControlRequest);
			break;

		/* case USB_REQ_SYNC_FRAME: */

		default:
			break;
		}
	}

	if (!handled)
		handled = forward_to_driver(pThis, pControlRequest);

	/* now tx! */
	return handled;
}

/* Executed @ interrupt time; complete CANNOT sleep.
 * caller may not hold pThis->Lock
 */
static void MGC_CompleteEp0Request(struct musb *pThis, struct usb_request *req)
{
#ifdef MUSB_PARANOID
	ASSERT_SPINLOCK_UNLOCKED(&pThis->Lock);
	ASSERT_SPINLOCK_LOCKED(&MGC_aGadgetLocalEnd[0].Lock);
#endif

	/* REVISIT most of this should be shared with the code
	 * doing the same thing for non-control endpoints.
	 */

	if (!req) {
		spin_lock(&MGC_aGadgetLocalEnd[0].Lock);
		req = MGC_CurrentRequest(&MGC_aGadgetLocalEnd[0]);
		if (req) {
			/* currently happens with zero length data stages */
			list_del(&req->list);
			INIT_LIST_HEAD(&MGC_aGadgetLocalEnd[0].req_list);
		}
		spin_unlock(&MGC_aGadgetLocalEnd[0].Lock);
	}
	pThis->bEnd0Stage = MGC_END0_STAGE_SETUP;

	if (req) {
		if (req->status == -EINPROGRESS)
			req->status = 0;
		if (req->status && req->status != -ESHUTDOWN)
			DBG(3, "complete ep0 req %p stat %d len %u/%u\n",
				req, req->status,
				req->actual, req->length);

		req->complete(&MGC_aGadgetLocalEnd[0].end_point, req);
	}
}

static void ep0_rxstate(struct musb *this);

static void handle_ep0_completion_irq(struct musb *pThis)
{
	const u8 *pBase = (u8 *) pThis->pRegs;

	DBG(4, "post event interrupts ep0stage=%s\n",
	    decode_ep0stage(pThis->bEnd0Stage));

#ifdef MUSB_PARANOID
	ASSERT_SPINLOCK_UNLOCKED(&pThis->Lock);
	ASSERT_SPINLOCK_UNLOCKED(&MGC_aGadgetLocalEnd[0].Lock);
#endif

	switch (pThis->bEnd0Stage) {

	case MGC_END0_STAGE_STATUSIN:
		/* end of sequence #2 (RX state) or #3 (no data) */
		DBG(-1001, "MGC_END0_STAGE_STATUSIN request\n");

		/* update address (if needed) only @ the end of the 
		 * status phase per standard. The guide is WRONG!
		 */
		if (pThis->bSetAddress) {
			pThis->bSetAddress = FALSE;
			MGC_Write8(pBase, MGC_O_HDRC_FADDR, pThis->bAddress);
#ifdef MUSB_MONITOR_DATA
			MGC_EnableDebug();
#endif
		}

		/* enter test mode if needed */
		if (pThis->bTestMode) {
			DBG(-1001, "entering TESTMODE\n");

			if (MGC_M_TEST_PACKET == pThis->bTestModeValue) {
				MGC_HdrcLoadFifo(pBase, 0,
						 sizeof(MGC_aTestPacket),
						 MGC_aTestPacket);
			}

			spin_lock(&pThis->Lock);
			MGC_SelectEnd(pBase, 0);	/* select ep0 */
			MGC_Write8(pBase, MGC_O_HDRC_TESTMODE,
				   pThis->bTestModeValue);
			spin_unlock(&pThis->Lock);
		}

		DBG(-1001, "completing posted request (if any)\n");
		MGC_CompleteEp0Request(pThis, NULL);
		break;

	case MGC_END0_STAGE_STATUSOUT:
		/* sequence #1: write to host (TX state)  */
		MGC_CompleteEp0Request(pThis, NULL);
		break;

	case MGC_END0_STAGE_TX:
		// FIXME call expects endpoint to be locked
		(void)  ep0_txstate(pThis);
		break;
	case MGC_END0_STAGE_RX:
		// FIXME call expects endpoint to be locked
		(void) ep0_rxstate(pThis);
		break;

	default:		/* IT WAS STALLED */
		DBG(-1002, "recovering from stall? ep0stage=%s\n",
		    decode_ep0stage(pThis->bEnd0Stage));
		pThis->bEnd0Stage = MGC_END0_STAGE_SETUP;
		break;
	}

	/* weird core! clear CSR0 */
	//spin_lock(&pThis->Lock);
	//MGC_SelectEnd(pBase, 0);
	//MGC_WriteCsr8(pBase, MGC_O_HDRC_CSR0, 0, 0);
	//spin_unlock(&pThis->Lock);

	DBG(3, "==>\n");
}

/*
 * Handle all control requests with no DATA stage, including standard
 * requests such as:
 * USB_REQ_SET_CONFIGURATION, USB_REQ_SET_INTERFACE, unrecognized
 *	always delegated to the gadget driver
 * USB_REQ_SET_ADDRESS, USB_REQ_CLEAR_FEATURE, USB_REQ_SET_FEATURE
 *	always handled here, except for class/vendor/... features
 */
static int
service_zero_data_request(MGC_LinuxCd *pThis,
		struct usb_ctrlrequest *pControlRequest)
{
	int handled = -EINVAL;
	const u8 *pBase = (u8 *) pThis->pRegs;
	const u8 bRecip = pControlRequest->bRequestType & USB_RECIP_MASK;

#ifdef MUSB_PARANOID
	ASSERT_SPINLOCK_UNLOCKED(&pThis->Lock);
	ASSERT_SPINLOCK_UNLOCKED(&MGC_aGadgetLocalEnd[0].Lock);
#endif

	/* the gadget driver handles everything except what we MUST handle */
	if ((pControlRequest->bRequestType & USB_TYPE_MASK)
			== USB_TYPE_STANDARD) {
		switch (pControlRequest->bRequest) {
		case USB_REQ_SET_ADDRESS:
			/* change it after the status stage */
			pThis->bSetAddress = TRUE;
			pThis->bAddress = (u8) (pControlRequest->wValue & 0x7f);
			handled = 1;
			break;

		case USB_REQ_CLEAR_FEATURE:
			switch (bRecip) {
			case USB_RECIP_DEVICE:
				if (pControlRequest->wValue
						!= USB_DEVICE_REMOTE_WAKEUP)
					break;
				pThis->bMayWakeup = 0;
				handled = 1;
				break;
			case USB_RECIP_INTERFACE:
				break;
			case USB_RECIP_ENDPOINT:{
				const u8 bEnd = (u8)
					pControlRequest->wIndex & 0x7f;
				MGC_GadgetLocalEnd *pEnd =
					&MGC_aGadgetLocalEnd[bEnd];

				if (bEnd == 0 || pControlRequest->wValue
						!= USB_ENDPOINT_HALT)
					break;

				// REVISIT this requires spinlock, yes?
				MGC_GadgetSetHalt(&pEnd->end_point, 0);

				/* select ep0 again */
				MGC_SelectEnd(pBase, 0);
				handled = 1;
				} break;
			default:
				/* class, vendor, etc ... delegate */
				handled = 0;
				break;
			}
			break;

		case USB_REQ_SET_FEATURE:
			switch (bRecip) {
			case USB_RECIP_DEVICE:
				handled = 1;
				switch (pControlRequest->wValue) {
				case USB_DEVICE_REMOTE_WAKEUP:
					pThis->bMayWakeup = 1;
					break;
				case USB_DEVICE_TEST_MODE:
					if (pThis->g.speed != USB_SPEED_HIGH)
						break;
					if (pControlRequest->wIndex & 0xff)
						break;

					switch (pControlRequest->wIndex >> 8) {
					case 1:
						pr_debug("TEST_J\n");
						/* TEST_J */
						pThis->bTestModeValue =
							MGC_M_TEST_J;
						break;
					case 2:
						/* TEST_K */
						pr_debug("TEST_K\n");
						pThis->bTestModeValue =
							MGC_M_TEST_K;
						break;
					case 3:
						/* TEST_SE0_NAK */
						pr_debug("TEST_SE0_NAK\n");
						pThis->bTestModeValue =
							MGC_M_TEST_SE0_NAK;
						break;
					case 4:
						/* TEST_PACKET */
						pr_debug("TEST_PACKET\n");
						pThis->bTestModeValue =
							MGC_M_TEST_PACKET;
						break;
					default:
						handled = -EINVAL;
						break;
					}

					/* enter test mode after irq */
					if (handled > 0)
						pThis->bTestMode = TRUE;
					break;
#ifdef CONFIG_USB_MUSB_OTG
				case USB_DEVICE_B_HNP_ENABLE:
					if (!pThis->g.is_otg)
						goto stall;
					pThis->g.b_hnp_enable = 1;
					goto delegate_otg;
				case USB_DEVICE_A_HNP_SUPPORT:
					if (!pThis->g.is_otg)
						goto stall;
					pThis->g.a_hnp_support = 1;
					goto delegate_otg;
				case USB_DEVICE_A_ALT_HNP_SUPPORT:
					if (!pThis->g.is_otg)
						goto stall;
					pThis->g.a_alt_hnp_support = 1;
delegate_otg:
					MGC_OtgMachineSetFeature(
						&(pThis->OtgMachine),
						pControlRequest->wValue);
					break;
stall:
#endif
				default:
					handled = -EINVAL;
					break;
				}
				break;

			case USB_RECIP_INTERFACE:
				break;

			case USB_RECIP_ENDPOINT:{
				const u8 bEnd = (u8)
					pControlRequest->wIndex & 0x7f;
				MGC_GadgetLocalEnd *pEnd =
					&MGC_aGadgetLocalEnd[bEnd];

				if (bEnd == 0 || pControlRequest->wValue
						!= USB_ENDPOINT_HALT)
					break;

				// REVISIT this requires spinlock, yes?
				MGC_GadgetSetHalt(&pEnd->end_point, 1);

				/* select ep0 again */
				MGC_SelectEnd(pBase, 0);
				handled = 1;
				} break;

			default:
				/* class, vendor, etc ... delegate */
				handled = 0;
				break;
			}
			break;
		default:
			/* delegate SET_CONFIGURATION, etc */
			handled = 0;
		}
	}

	if (!handled)
		handled = forward_to_driver(pThis, pControlRequest);

	return handled;
}

/* we may have an ep0out data packet */
static void ep0_rxstate(struct musb *this)
{
	const u8		*pBase = (u8 *) this->pRegs;
	struct usb_request	*req;
	u16			tmp;

	spin_lock(&this->Lock);
	MGC_SelectEnd(pBase, 0);
	tmp = MGC_ReadCsr16(pBase, MGC_O_HDRC_CSR0, 0);
	if ((tmp & MGC_M_CSR0_RXPKTRDY) == 0) {
		req = NULL;
		goto done;
	}

	// FIXME test requires endpoint to be locked
	req = MGC_CurrentRequest(&(MGC_aGadgetLocalEnd[0]));

	/* read packet and ack; or stall */
	if (req) {
		void		*buf = req->buf + req->actual;
		unsigned	len = req->length - req->actual;

		/* read the buffer */
		tmp = MGC_ReadCsr8(pBase, MGC_O_HDRC_COUNT0, 0);
		if (tmp > len) {
			req->status = -EOVERFLOW;
			tmp = len;
		}
		MGC_HdrcUnloadFifo(pBase, 0, tmp, buf);
		req->actual += tmp;
		tmp = MGC_M_CSR0_P_SVDRXPKTRDY;
		if (tmp < 64
				|| req->actual == req->length
				|| req->status != -EINPROGRESS) {
			this->bEnd0Stage = MGC_END0_STAGE_STATUSOUT;
			list_del(&req->list);
			tmp |= MGC_M_CSR0_P_DATAEND;
		} else
			req = NULL;
	} else
		tmp = MGC_M_CSR0_P_SVDRXPKTRDY | MGC_M_CSR0_P_SENDSTALL;
	MGC_WriteCsr16(pBase, MGC_O_HDRC_CSR0, 0, tmp);
done:
	spin_unlock(&this->Lock);

	/* REVISIT  we "should" hold off reporting DATAEND and going to
	 * STATUSOUT until after the completion handler had a chance  to
	 * issue a stall instead.
	 */
	if (req)
		MGC_CompleteEp0Request(this, req);
}

/*
 * transmitting to the host (IN), this code might be called from IRQ 
 * and from kernel thread.
 *
 * @pre spin_is_locked(&pEnd->Lock)
 */
int ep0_txstate(struct musb *pThis)
{
	unsigned long flags;
	const u8 *pBase = (u8 *) pThis->pRegs;
	MGC_GadgetLocalEnd *pEnd = &(MGC_aGadgetLocalEnd[0]);
		// FIXME test requires endpoint to be locked
	struct usb_request *pRequest = MGC_CurrentRequest(pEnd);
	u16 wCsrVal = MGC_M_CSR0_TXPKTRDY;
	u8 *pFifoSource;
	u8 wFifoCount;

	DBG(-1002, "<==\n");

#ifdef MUSB_PARANOID
	if (!pThis || !pRequest) {
		ERR("pThis=%p, pRequest=%p", pThis, pRequest);
		return -EINVAL;
	}
#endif

#ifdef MUSB_PARANOID
	ASSERT_SPINLOCK_UNLOCKED(&pThis->Lock);
	ASSERT_SPINLOCK_LOCKED(&pEnd->Lock);
#endif

	spin_lock_irqsave(&pThis->Lock, flags);
	MGC_SelectEnd(pBase, 0);

	if (pRequest->actual == 0) {
		/* ack the request first */
		DBG(4, "acking request %s\n",
		    decode_csr0(MGC_M_CSR0_P_SVDRXPKTRDY));
		MGC_WriteCsr16(pBase, MGC_O_HDRC_CSR0, 0,
			       MGC_M_CSR0_P_SVDRXPKTRDY);
	}

	/* load the data */
	pFifoSource = (u8 *) pRequest->buf + pRequest->actual;
	wFifoCount = min((unsigned) MGC_END0_FIFOSIZE,
		pRequest->length - pRequest->actual);
	MGC_HdrcLoadFifo(pBase, 0, wFifoCount, pFifoSource);
	pRequest->actual += wFifoCount;	/* done */

	/* update the flags */
	if (wFifoCount < MUSB_MAX_END0_PACKET) {
		pThis->bEnd0Stage = MGC_END0_STAGE_STATUSIN;
		list_del(&pRequest->list);
		wCsrVal |= MGC_M_CSR0_P_DATAEND;
	}

	/* send it out! (this will trigger the ep0 completion IRQ) 
	 * serviced in interrupt_complete()  */
	DBG(4, "wrote wFifoCount=%d bytes, wCsrVal=%s\n", wFifoCount,
	    decode_csr0(wCsrVal));
	MGC_WriteCsr16(pBase, MGC_O_HDRC_CSR0, 0, wCsrVal);
	spin_unlock_irqrestore(&pThis->Lock, flags);

	/* report completions as soon as the fifo's loaded; there's no
	 * win in waiting till this last packet gets acked.
	 */
	if (wFifoCount < MUSB_MAX_END0_PACKET)
		MGC_CompleteEp0Request(pThis, pRequest);

	return 0;
}

/*
 * Read a SETUP packet (struct usb_ctrlrequest) from the hardware.
 * Fields are left in USB byte-order.
 *
 * @return 0 when the packet is complete, a negative number when an error 
 * occurred, a positive number when still there are bytes to read.
 */
static void
MGC_ReadSetup(struct musb *pThis, struct usb_ctrlrequest *req)
{
	const u8 *pBase = (u8 *) pThis->pRegs;

	spin_lock(&pThis->Lock);
	MGC_SelectEnd(pBase, 0);	/* select ep0 */

	MGC_HdrcUnloadFifo(pBase, 0, sizeof *req, (u8 *)req);

	/* NOTE:  earlier 2.6 versions changed setup packets to host
	 * order, but now USB packets always stay in USB byte order.
	 */
	DBG(2, "SETUP req%02x.%02x v%04x i%04x l%d\n",
		 req->bRequestType,
		 req->bRequest,
		 le16_to_cpu(req->wValue),
		 le16_to_cpu(req->wIndex),
		 le16_to_cpu(req->wLength));

	/* the header will set me in the right stage:
	   pThis->bEnd0Stage=MGC_END0_STAGE_TX
	   pThis->bEnd0Stage=MGC_END0_STAGE_RX
	 */
	if (req->bRequestType & USB_DIR_IN)
		pThis->bEnd0Stage = MGC_END0_STAGE_TX;
	else if (req->bRequestType & USB_DIR_OUT)
		pThis->bEnd0Stage = MGC_END0_STAGE_RX;

	spin_unlock(&pThis->Lock);
}


/**
 * Handle ep0 interrupt of a device, lock & release pThis.
 * @param pThis this
 */
u8 MGC_HdrcServiceFunctionEp0(MGC_LinuxCd * pThis)
{
	u16 wCsrVal;		/* */
	u16 wCount;		/* bytes available */
	const u8 *pBase = (u8 *) pThis->pRegs;

	spin_lock(&pThis->Lock);
	MGC_SelectEnd(pBase, 0);	/* select ep0 */
	wCsrVal = MGC_ReadCsr16(pBase, MGC_O_HDRC_CSR0, 0);
	wCount = MGC_ReadCsr8(pBase, MGC_O_HDRC_COUNT0, 0);

	DEBUG_CODE(4, {
		u8 myaddr = MGC_Read8(pBase, MGC_O_HDRC_FADDR);
		u8 devctl = MGC_Read8(pBase, MGC_O_HDRC_DEVCTL);
		printk(KERN_INFO "%s: wCrsVal=0x%x, wCount=%d, myaddr=%0x, "
			"mode=%s, ep0stage=%s\n",
			__FUNCTION__, wCsrVal, wCount, myaddr,
			decode_devctl(devctl),
			decode_ep0stage(pThis->bEnd0Stage));
		});

	/* I sent a stall.. need to acknowledge it now.. */
	if (wCsrVal & MGC_M_CSR0_P_SENTSTALL) {
		DBG(-1002, "acking stall while in ep0stage=%s\n",
		    decode_ep0stage(pThis->bEnd0Stage));
		MGC_WriteCsr16(pBase, MGC_O_HDRC_CSR0, 0,
			       wCsrVal & ~MGC_M_CSR0_P_SENTSTALL);
		pThis->bEnd0Stage = MGC_END0_STAGE_SETUP;
	}

	/* setup ended prematurely, abort it  */
	if (wCsrVal & MGC_M_CSR0_P_SETUPEND) {
		DBG(-1002, "acking setupend while in ep0stage=%s\n",
		    decode_ep0stage(pThis->bEnd0Stage));
		/* clearing it */
		MGC_WriteCsr16(pBase, MGC_O_HDRC_CSR0, 0,
			       MGC_M_CSR0_P_SVDSETUPEND);
		pThis->bEnd0Stage = MGC_END0_STAGE_SETUP;
	}

	spin_unlock(&pThis->Lock);

	/* handle completion interrupt */
	if (!wCsrVal && !wCount) {
		handle_ep0_completion_irq(pThis);
		return TRUE;
	}

	switch (pThis->bEnd0Stage) {

	case MGC_END0_STAGE_STATUSOUT:
	case MGC_END0_STAGE_STATUSIN:
		pThis->bEnd0Stage = MGC_END0_STAGE_SETUP;
		break;

	case MGC_END0_STAGE_TX:
		/* the more common IN path is completion */
		if (wCsrVal & MGC_M_CSR0_TXPKTRDY) {
			DBG(1, "MGC_END0_STAGE_TX\n");
			ep0_txstate(pThis);
		}
		break;

	case MGC_END0_STAGE_RX:
		ep0_rxstate(pThis);
		break;

	case MGC_END0_STAGE_SETUP:
		if (wCsrVal & MGC_M_CSR0_RXPKTRDY) {
			struct usb_ctrlrequest	setup;
			int handled = 0;

			if (wCount != 8) {
				ERR("SETUP packet len %d != 8 ?\n", wCount);
				break;
			}

			MGC_ReadSetup(pThis, &setup);

			/* sometimes the RESET won't be reported */
			if (unlikely(pThis->g.speed == USB_SPEED_UNKNOWN)) {
				u8	power;

				printk(KERN_ERR "%s: peripheral reset "
						"irq lost!\n",
						musb_driver_name);
				power = MGC_Read8(pBase, MGC_O_HDRC_POWER);
				pThis->g.speed = (power & MGC_M_POWER_HSMODE)
				    ? USB_SPEED_HIGH : USB_SPEED_FULL;

			}

			/* sequence #3 (no data stage) */
			if (setup.wLength == 0) {
				u16 wCsrVal = MGC_M_CSR0_P_SVDRXPKTRDY
					| MGC_M_CSR0_P_DATAEND;

				pThis->bEnd0Stage = MGC_END0_STAGE_STATUSIN;
				handled = service_zero_data_request(
						pThis, &setup);
				if (handled < 0)
					wCsrVal |= MGC_M_CSR0_P_SENDSTALL;

				/* ack or stall the request */
				DBG(3, "handled=%d, wCsrVal=%s, ep0stage=%s\n",
					handled, decode_csr0(wCsrVal),
					decode_ep0stage(pThis->bEnd0Stage));

				spin_lock(&pThis->Lock);
				MGC_SelectEnd(pBase, 0);
				MGC_WriteCsr16(pBase, MGC_O_HDRC_CSR0,
					       0, wCsrVal);
				spin_unlock(&pThis->Lock);
			} else {
				/* sequence #1 (IN to host) */
				if (setup.bRequestType & USB_DIR_IN) {
					pThis->bEnd0Stage = MGC_END0_STAGE_TX;
					handled = service_in_request(
							pThis, &setup);

				/* sequence #2 (OUT from host) */
				} else {
					pThis->bEnd0Stage = MGC_END0_STAGE_RX;
					handled = service_out_request(
							pThis, &setup);

					/* ack, let DATA stage begin */
					if (handled >= 0) {
						spin_lock(&pThis->Lock);
						MGC_SelectEnd(pThis->pRegs, 0);
						MGC_WriteCsr16(pThis->pRegs,
							MGC_O_HDRC_CSR0, 0,
							MGC_M_CSR0_P_SVDRXPKTRDY);
						spin_unlock(&pThis->Lock);
					}
				}

				/* protocol stall? */
				if (handled < 0) {
					DBG(3, "stall (%d)\n", handled);
					spin_lock(&pThis->Lock);
					MGC_SelectEnd(pBase, 0);
					MGC_WriteCsr16(pBase,
					       MGC_O_HDRC_CSR0, 0,
					       MGC_M_CSR0_P_SVDRXPKTRDY
					       | MGC_M_CSR0_P_SENDSTALL);
					spin_unlock(&pThis->Lock);
				}
			}
		} else {
			/* ignore */
		}
		break;

		/* handle the application stall on Ep0 */
	default:{
		u16 wCsrVal = MGC_M_CSR0_P_SENDSTALL;
		switch (pThis->bEnd0Stage & ~MGC_END0_STAGE_STALL_BIT) {
		case MGC_END0_STAGE_TX:
			wCsrVal |= MGC_M_CSR0_TXPKTRDY;
			break;
		case MGC_END0_STAGE_RX:
			wCsrVal |= MGC_M_CSR0_RXPKTRDY;
			break;
		}

		DBG(3, "Application stall from ep0stage=%s\n",
				decode_ep0stage(pThis->bEnd0Stage));
		spin_lock(&pThis->Lock);
		MGC_SelectEnd(pBase, 0);
		MGC_WriteCsr16(pBase, MGC_O_HDRC_CSR0, 0, wCsrVal);
		spin_unlock(&pThis->Lock);

		pThis->bEnd0Stage = MGC_END0_STAGE_SETUP;
		}
		break;
	}

	return 1;
}
