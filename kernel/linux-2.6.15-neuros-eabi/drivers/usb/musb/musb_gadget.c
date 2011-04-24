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
#include <linux/module.h>
#include <linux/smp.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/moduleparam.h>
#include <linux/stat.h>

#include "musbdefs.h"


/* endpoint operations */
static int MGC_GadgetEnableEnd(struct usb_ep *ep,
			       const struct usb_endpoint_descriptor *desc);

static int MGC_GadgetDisableEnd(struct usb_ep *ep);

static struct usb_request *MGC_GadgetAllocRequest(struct usb_ep *ep,
						  int gfp_flags);
static void MGC_GadgetFreeRequest(struct usb_ep *ep, struct usb_request *req);
static int MGC_GadgetQueue(struct usb_ep *ep, struct usb_request *req,
			   int gfp_flags);
static int MGC_GadgetDequeue(struct usb_ep *ep, struct usb_request *req);

int MGC_GadgetFifoStatus(struct usb_ep *ep);

int MGC_GadgetVbusSession(struct usb_gadget *gadget, int is_active);
int MGC_GadgetVbusDraw(struct usb_gadget *gadget, unsigned mA);
int MGC_GadgetPullup(struct usb_gadget *gadget, int is_on);

/**************************************************************************
Gadget End points
**************************************************************************/

struct musb_request {
	struct usb_request request;
	struct musb *musb;
	u8 bTx;			/* endpoint direction */
	u8 bEnd;
};

static inline struct musb_request *to_musb_request(struct usb_request *req)
{
	return req ? container_of(req, struct musb_request, request) : NULL;
}

/**************************************************************************
 static
**************************************************************************/

struct usb_ep_ops MGC_GadgetEndpointOperations = {
	.enable = MGC_GadgetEnableEnd,
	.disable = MGC_GadgetDisableEnd,
	.alloc_request = MGC_GadgetAllocRequest,
	.free_request = MGC_GadgetFreeRequest,
	.alloc_buffer = MGC_GadgetAllocBuffer,
	.free_buffer = MGC_GadgetFreeBuffer,
	.queue = MGC_GadgetQueue,
	.dequeue = MGC_GadgetDequeue,
	.set_halt = MGC_GadgetSetHalt,
	.fifo_status = MGC_GadgetFifoStatus,
	.fifo_flush = MGC_GadgetFifoFlush
};

extern struct usb_gadget_ops MGC_GadgetOperations;

/**************************************************************************
Handling completion
**************************************************************************/

/*
 * Immediately complete a request.
 *
 * @param pRequest the request to complete
 * @param status the status to complete the request with
 * @context: controller locked, IRQs blocked.
 */
static void MGC_CompleteRequest(struct usb_request *pRequest, int status)
{
	struct musb_request	*req;
	struct musb		*musb;

	req = to_musb_request(pRequest);

#ifdef MUSB_PARANOID
	BUG_ON(!pRequest);
#endif

	list_del(&pRequest->list);
	if (req->request.status == -EINPROGRESS)
		req->request.status = status;
	musb = req->musb;

	spin_unlock(&musb->Lock);
	req->request.complete(&MGC_aGadgetLocalEnd[req->bEnd].end_point,
			&req->request);
	spin_lock(&musb->Lock);
}

/* ----------------------------------------------------------------------- */

/*
 * Flush requests queued to an endpoint using the status. Synchronous.
 * caller locked controller and blocked irqs
 * @param pEnd the end to flush the request from
 * @param status the status to complete the requests with
 * @context: controller locked, IRQs blocked.
 */
static void MGC_FlushEpRequests(struct musb_ep * pEnd, const int status)
{
	struct musb_request	*req = NULL;

	while (!list_empty(&(pEnd->req_list))) {
		req = container_of(pEnd->req_list.next, struct musb_request,
				request.list);
		DBG(2, "flushing %s request %p\n",
				pEnd->end_point.name,
				&req->request);
		MGC_CompleteRequest(&req->request, status);
	}
}

/**************************************************************************
* Endpoint locking
**************************************************************************/

#if 0
static void ep_spin_lock(MGC_GadgetLocalEnd * pEnd)
{
	unsigned long flags;
	MGC_LinuxCd *pThis = MGC_GetEpDriver(pEnd);
	const u8 bEnd = pEnd->bEndNumber;

	DBG(2, "<== bEnd=%d\n", bEnd);

	/* disable IRQs for the ep */
	if (bEnd) {
		spin_lock_irqsave(&pThis->Lock, flags); {
			MGC_SelectEnd(pThis->pRegs, bEnd);
			u16 reg =
			    (pEnd->
			     bIsTx) ? MGC_O_HDRC_INTRTXE : MGC_O_HDRC_INTRRXE;
			MGC_Write16(pThis->pRegs, reg,
				    MGC_Read16(pThis->pRegs,
					       reg) & ~(1 << bEnd));
		}
		spin_unlock_irqrestore(&pThis->Lock, flags);
	}

	/* lock the ep */
	spin_lock(&pEnd->Lock);
	DBG(2, "==>\n");
}

static void ep_spin_unlock(MGC_GadgetLocalEnd * pEnd)
{
	unsigned long flags;
	MGC_LinuxCd *pThis = MGC_GetEpDriver(pEnd);
	const u8 bEnd = pEnd->bEndNumber;

	DBG(2, "<== bEnd=%d\n", bEnd);

	/* unlock the ep */
	spin_unlock(&pEnd->Lock);

	/* enable IRQs for the ep */
	if (bEnd) {
		spin_ spin_lock_irqsave(&pThis->Lock, flags); {
			MGC_SelectEnd(pThis->pRegs, bEnd);
			u16 reg =
			    (pEnd->
			     bIsTx) ? MGC_O_HDRC_INTRTXE : MGC_O_HDRC_INTRRXE;
			MGC_Write16(pThis->pRegs, reg,
				    MGC_Read16(pThis->pRegs,
					       reg) | (1 << bEnd));
		}
		spin_unlock_irqrestore(&pThis->Lock, flags);
	}

	DBG(2, "==>\n");
}
#endif

/**************************************************************************
Tx/Rx Data
**************************************************************************/

#if MUSB_DEBUG>0

static char *dump_usb_request(struct usb_request *req)
{
	static char buff[256];
	if (req) {
		sprintf(buff, "req %p, length %d%s, actual %d, status %d",
				req, req->length,
				req->zero ? ", zero" : "",
				req->actual, req->status);
	} else {
		sprintf(buff, "null request");
	}

	return buff;
}

#endif

static inline int get_ep_packet_size(MGC_LinuxCd * pThis, u8 bEnd)
{
	int size = MGC_aGadgetLocalEnd[bEnd].wPacketSize;

	if ((USB_ENDPOINT_XFER_BULK == MGC_aGadgetLocalEnd[bEnd].bTrafficType)
	    && pThis->bBulkSplit) {
		size = pThis->aLocalEnd[bEnd].wMaxPacketSizeTx;
	}

	return size;
}

/*
 * An endpoint is transmitting data. This can be called either from 
 * the IRQ routine or from GadgetQueue to kickstart a request on an 
 * endpoint.
 *
 * @param pThis
 * @param req
 * @context: controller locked, IRQs blocked.
 */
static void txstate(struct musb *pThis, struct musb_request *req)
{
	u8 bEnd;
	MGC_GadgetLocalEnd *pEnd;
	struct usb_request *pRequest;
	u16 wFifoCount = 0, wCsrVal;
	const u8 *pBase = (u8 *) pThis->pRegs;

#ifdef CONFIG_USB_INVENTRA_DMA
	MGC_DmaController *pDmaController;
	MGC_DmaChannel *pDmaChannel;
#endif

#ifdef MUSB_PARANOID
	if (!req) {
		ERR("cannot call txstate without request\n");
		return;
	}
#endif

	bEnd = req->bEnd;
	pRequest = &req->request;
	pEnd = &(MGC_aGadgetLocalEnd[req->bEnd]);

	wFifoCount = min(get_ep_packet_size(pThis, bEnd),
			 (int)(pRequest->length - pRequest->actual));

#ifdef MUSB_PARANOID
	if (pRequest->length - pRequest->actual < 0) {
		ERR("NEGATIVE FIFOCOUNT! pRequest->actual=%d, wFifoCount=%d, pRequest->length=%d\n",
				pRequest->actual, wFifoCount, pRequest->length);
		wFifoCount = 0;
	}

	if (pRequest->actual + wFifoCount > pRequest->length) {
		ERR("trying to write PAST length! pRequest->actual=%d, wFifoCount=%d, pRequest->length=%d\n",
				pRequest->actual, wFifoCount, pRequest->length);
		wFifoCount = pRequest->length - pRequest->actual;
	}
#endif

	/* read TXCSR before */
	MGC_SelectEnd(pBase, bEnd);
	wCsrVal = MGC_ReadCsr16(pBase, MGC_O_HDRC_TXCSR, bEnd);

	if (wCsrVal & MGC_M_TXCSR_TXPKTRDY) {
		DBG(4, "%s old packet still ready , txcsr %03x\n",
				pEnd->end_point.name, wCsrVal);
		return;
	}

	DBG(3, "bEnd=0x%x, wPacketSize=0x%x, wFifoCount=%d, txcsr %03x, %s\n",
			bEnd, pEnd->wPacketSize, wFifoCount,
			wCsrVal, dump_usb_request(pRequest));

	/* stalled?? */
	if (wCsrVal & MGC_M_TXCSR_P_SENDSTALL) {
		DBG(3, "completing stalled request pRequest=%p\n", pRequest);
		MGC_CompleteRequest(pRequest, -EPIPE);
		return;
	}

#ifdef CONFIG_USB_INVENTRA_DMA
	pDmaController = pThis->pDmaController;
	pDmaChannel = pThis->aLocalEnd[bEnd].pDmaChannel;
	if (!pRequest->dma && pDmaChannel) {
		/* release previously-allocated channel */
		pDmaController->pfDmaReleaseChannel(pDmaChannel);
		pEnd->pDmaChannel = NULL;
	} else if (pRequest->dma) {
		u8 bDmaOk = FALSE;

		/* candidate for DMA */
		if (pDmaController && !pDmaChannel) {
			pDmaChannel = pThis->aLocalEnd[bEnd].pDmaChannel =
			    pDmaController->
			    pfDmaAllocateChannel(pDmaController->pPrivateData,
						 bEnd, TRUE, pEnd->bTrafficType,
						 pEnd->wPacketSize);
		}

		if (pDmaChannel) {
			pDmaChannel->dwActualLength = 0L;
			pEnd->dwRequestSize = min(pRequest->length,
						  pDmaChannel->dwMaxLength);
			bDmaOk = pDmaController->pfDmaProgramChannel(
					pDmaChannel, pEnd->wPacketSize,
					pDmaChannel->bDesiredMode,
					pRequest->dma, pEnd->dwRequestSize);
			if (bDmaOk) {
				wFifoCount = 0;
				wCsrVal |= MGC_M_TXCSR_AUTOSET
					| MGC_M_TXCSR_DMAENAB
					| (pDmaChannel->bDesiredMode
					    ? MGC_M_TXCSR_DMAMODE : 0);
			} else {
				pDmaController->
				    pfDmaReleaseChannel(pDmaChannel);
				pThis->aLocalEnd[bEnd].pDmaChannel = NULL;
			}
		}
	}
#endif

	MGC_HdrcLoadFifo(pBase, bEnd, wFifoCount,
			 (u8 *) (pRequest->buf + pRequest->actual));
	pRequest->actual += wFifoCount;

	wCsrVal |= MGC_M_TXCSR_TXPKTRDY;

	/* now write out the thing */
	DBG(3, "TX/IN packet length=%d actual=%d, txcsr=%04x, %d/%d\n",
			pRequest->length, pRequest->actual, wCsrVal, wFifoCount,
			MGC_ReadCsr16(pBase, MGC_O_HDRC_TXMAXP, bEnd));
	MGC_WriteCsr16(pBase, MGC_O_HDRC_TXCSR, bEnd, wCsrVal);
}

/*
 * Data ready for a request; called from IRQ; no need to disable the
 * IRQS bc they are already disabled. The end point CANNOT be locked
 * when entering this routine because it would deadlock. By design, 
 * this is called only after txstate has been called.
 *
 * @param pThis
 * @param bEnd
 * @context: controller NOT locked
 */
void MGC_HdrcServiceDeviceTxAvail(MGC_LinuxCd * pThis, u8 bEnd)
{
	u16 wCsrVal;
	unsigned long flags;
	struct usb_request *pRequest;
	u8 *pBase = (u8 *) pThis->pRegs;
	MGC_GadgetLocalEnd *pEnd = &(MGC_aGadgetLocalEnd[bEnd]);

	DBG(4, "<== %s, bEnd=%d\n", pEnd->end_point.name, bEnd);

	do {
		spin_lock_irqsave(&pThis->Lock, flags);
		MGC_SelectEnd(pBase, bEnd);
		wCsrVal = MGC_ReadCsr16(pBase, MGC_O_HDRC_TXCSR, bEnd);

		if (wCsrVal & MGC_M_TXCSR_P_SENTSTALL) {
			wCsrVal &= ~MGC_M_TXCSR_P_SENTSTALL;
			MGC_WriteCsr16(pBase, MGC_O_HDRC_TXCSR, bEnd, wCsrVal);

			pRequest = MGC_CurrentRequest(pEnd);
			DBG(3, "request %p was stalled\n", pRequest);
			if (pRequest)
				MGC_CompleteRequest(pRequest, -EPIPE);

			break;
		}

		if (wCsrVal & MGC_M_TXCSR_P_UNDERRUN) {
#ifdef MUSB_CONFIG_PROC_FS
			pThis->aLocalEnd[bEnd].dwMissedTxPackets++;
#endif
			wCsrVal &= ~MGC_M_TXCSR_P_UNDERRUN;
			MGC_WriteCsr16(pBase, MGC_O_HDRC_TXCSR, bEnd, wCsrVal);
			DBG(3, "underrun on ep%d\n", bEnd);
		}

		pRequest = MGC_CurrentRequest(pEnd);
		if (pRequest) {

			/* the zero flag request the transmission of a zero data pkt 
			 * (when required). The zero flag is reset to zero,
			 * after the zero packet has been submitted. */
			if (pRequest->actual == pRequest->length) {

				if (pRequest->zero &&
				    (pRequest->length
				     && (pRequest->length %
					 get_ep_packet_size(pThis, bEnd)) == 0)
				    && !(wCsrVal & MGC_M_TXCSR_P_SENTSTALL)
				    ) {
					const u16 wCsrVal = MGC_M_TXCSR_MODE
					    | MGC_M_TXCSR_TXPKTRDY;

					pRequest->zero = 0;

					DBG(3, "sending zero pkt\n");

					MGC_SelectEnd(pBase, bEnd);
					MGC_WriteCsr16(pBase, MGC_O_HDRC_TXCSR,
						       bEnd, wCsrVal);
					spin_unlock_irqrestore(&pThis->Lock,
							       flags);
					break;
				}

				DBG(3, "completing pRequest=%p on bEnd=0x%x\n",
				    pRequest, bEnd);
				MGC_CompleteRequest(pRequest, 0);

				/* kickstart next request if available */
				pRequest = pEnd->desc
						? MGC_CurrentRequest(pEnd)
						: NULL;
				if (!pRequest) {
					DBG(3, "bEnd=0x%x idle now\n", bEnd);
					break;
				}
			}

			/* txstate unlock the ep before writing */
			txstate(pThis, (struct musb_request *)pRequest);
		}

	} while (FALSE);

	spin_unlock_irqrestore(&pThis->Lock, flags);
}

/* ------------------------------------------------------------ */

/*
 * ep locked		[ REMOVE after checking, shouldn't be needed ]
 *
 * @param pThis the controller
 * @param req the request 
 * @context: controller locked, IRQs blocked.
 */
static void rxstate(struct musb *pThis, struct musb_request *req)
{
	u16 wCsrVal = 0;
	const u8 bEnd = req->bEnd;
	struct usb_request *pRequest = &req->request;
	const u8 *pBase = (u8 *) pThis->pRegs;
	MGC_GadgetLocalEnd *pEnd = &(MGC_aGadgetLocalEnd[bEnd]);
	u16 wFifoCount = 0, wCount = pEnd->wPacketSize;

#ifdef CONFIG_USB_INVENTRA_DMA
	u8 bDmaOk = FALSE;
	MGC_DmaController *pDmaController;
	MGC_DmaChannel *pDmaChannel;
	MGC_DmaChannelStatus bDmaStatus;
#endif

	MGC_SelectEnd(pBase, bEnd);
	wCsrVal = MGC_ReadCsr16(pBase, MGC_O_HDRC_RXCSR, bEnd);
	if (wCsrVal & MGC_M_RXCSR_RXPKTRDY) {

		/* this also handle residual (if any) */
		wCount = MGC_ReadCsr16(pBase, MGC_O_HDRC_RXCOUNT, bEnd);
		if (pRequest->actual < pRequest->length) {
#ifdef CONFIG_USB_INVENTRA_DMA
			pDmaController = pThis->pDmaController;
			pDmaChannel = pThis->aLocalEnd[bEnd].pDmaChannel;
			if (pDmaChannel && pRequest->dma && in_interrupt()) {
				/* get channel status and see if we need to continue transfer */
				bDmaStatus =
				    pDmaController->
				    pfDmaGetChannelStatus(pDmaChannel);
				switch (bDmaStatus) {
				case MGC_DMA_STATUS_FREE:
					pRequest->actual =
					    pDmaChannel->dwActualLength;
					if (pRequest->actual < pRequest->length) {
						pDmaChannel->dwActualLength =
						    0L;
						pEnd->dwRequestSize =
						    min(pRequest->length,
							pDmaChannel->
							dwMaxLength);
						bDmaOk =
						    pDmaController->
						    pfDmaProgramChannel
						    (pDmaChannel,
						     pEnd->wPacketSize,
						     pDmaChannel->bDesiredMode,
						     pRequest->dma +
						     pRequest->actual,
						     pEnd->dwRequestSize);
						if (bDmaOk)
							return;
					}
					break;
				case MGC_DMA_STATUS_BUSY:
					return;
				default:
					/* TODO: is this the right code? */
					pRequest->status = -ECONNRESET;
				}
			} else {
				/* must be residual case */
				if (!pRequest->dma && pDmaChannel) {
					/* release previously-allocated channel */
					pDmaController->
					    pfDmaReleaseChannel(pDmaChannel);
					pDmaChannel =
					    pThis->aLocalEnd[bEnd].pDmaChannel =
					    NULL;
				} else if (pRequest->dma) {
					/* candidate for DMA */
					if (pDmaController && !pDmaChannel) {
						pDmaChannel =
						    pThis->aLocalEnd[bEnd].
						    pDmaChannel =
						    pDmaController->
						    pfDmaAllocateChannel
						    (pDmaController->
						     pPrivateData, bEnd, FALSE,
						     pEnd->bTrafficType,
						     pEnd->wPacketSize);
					}
				}

				if (pDmaChannel
				    && (pRequest->actual < pRequest->length)) {
					pDmaChannel->dwActualLength = 0L;
					pEnd->dwRequestSize =
					    min(pRequest->length,
						pDmaChannel->dwMaxLength);
					bDmaOk =
					    pDmaController->
					    pfDmaProgramChannel(pDmaChannel,
								pEnd->
								wPacketSize,
								pDmaChannel->
								bDesiredMode,
								pRequest->dma,
								pEnd->
								dwRequestSize);
					if (bDmaOk) {
						wCsrVal |=
						    (MGC_M_RXCSR_AUTOCLEAR |
						     MGC_M_RXCSR_DMAENAB);
						/* this special sequence is required to get DMAReq to activate */
						MGC_WriteCsr16(pBase,
							       MGC_O_HDRC_RXCSR,
							       bEnd,
							       wCsrVal |
							       MGC_M_RXCSR_DMAMODE);
						MGC_WriteCsr16(pBase,
							       MGC_O_HDRC_RXCSR,
							       bEnd, wCsrVal);
						return;
					} else {
						pDmaController->
						    pfDmaReleaseChannel
						    (pDmaChannel);
						pThis->aLocalEnd[bEnd].
						    pDmaChannel = NULL;
					}
				}
			}
#endif

			wFifoCount = (u16) min((int)wCount,
					       (int)(pRequest->length -
						     pRequest->actual));
			MGC_HdrcUnloadFifo(pBase, bEnd, wFifoCount,
					   (u8 *) (pRequest->buf +
						   pRequest->actual));
			pRequest->actual += wFifoCount;

			DBG(3,
			    "wFifoCount=%d, bEnd=0x%x, pEnd->wPacketSize=0x%x, wCount=0x%x, %s\n",
			    wFifoCount, bEnd, pEnd->wPacketSize, wCount,
			    dump_usb_request(pRequest));

			/* ack the read! */
			wCsrVal &= ~MGC_M_RXCSR_RXPKTRDY;
			MGC_WriteCsr16(pBase, MGC_O_HDRC_RXCSR, bEnd, wCsrVal);
		}
	}

	/* reach the end or short packet detected */
	if (pRequest->actual == pRequest->length || wCount < pEnd->wPacketSize)
		MGC_CompleteRequest(pRequest, 0);
}

/*
 * Data ready for a request; called from IRQ
 * @param pThis the controller
 * @param req the request 
 */
void MGC_HdrcServiceDeviceRxReady(MGC_LinuxCd * pThis, u8 bEnd)
{
	u16 wCsrVal;
	unsigned long flags;
	struct usb_request *pRequest = NULL;
	u8 *pBase = (u8 *) pThis->pRegs;
	struct musb_ep	*pEnd = &(MGC_aGadgetLocalEnd[bEnd]);

	/* executed from interrupt no need to disable them */
	spin_lock_irqsave(&pThis->Lock, flags);
	MGC_SelectEnd(pBase, bEnd);
	wCsrVal = MGC_ReadCsr16(pBase, MGC_O_HDRC_RXCSR, bEnd);
	if (wCsrVal & MGC_M_RXCSR_P_SENTSTALL) {
		wCsrVal &= ~MGC_M_RXCSR_P_SENTSTALL;
		MGC_WriteCsr16(pBase, MGC_O_HDRC_RXCSR, bEnd, wCsrVal);
		DBG(3, "received stall on ep%d\n", bEnd);
		spin_unlock_irqrestore(&pThis->Lock, flags);
		return;
	}

	if (wCsrVal & MGC_M_RXCSR_P_OVERRUN) {
#ifdef MUSB_CONFIG_PROC_FS
		pThis->aLocalEnd[bEnd].dwMissedRxPackets++;
#endif
		wCsrVal &= ~MGC_M_RXCSR_P_OVERRUN;
		MGC_WriteCsr16(pBase, MGC_O_HDRC_RXCSR, bEnd, wCsrVal);
		DBG(3, "Received overrun on ep%d\n", bEnd);
	}
#ifdef MUSB_CONFIG_PROC_FS
	if (wCsrVal & MGC_M_RXCSR_INCOMPRX) {
		pThis->aLocalEnd[bEnd].dwErrorRxPackets++;
	}
#endif

	/* analyze request if the ep is not inactive */
	pRequest = pEnd->desc ? MGC_CurrentRequest(pEnd) : NULL;
	if (pRequest)
		rxstate(pThis, to_musb_request(pRequest));
	else
		DBG(3, "Rx: bytes waiting on %sep=0x%x\n",
				pEnd->desc ? "" : "inactive ",
				bEnd);

	spin_unlock_irqrestore(&pThis->Lock, flags);
}

/**************************************************************************
Gadget Functions
**************************************************************************/

/* new interface */
extern u8 MGC_HdrcServiceFunctionEp0(MGC_LinuxCd * pThis);

/*
 *
 * @param pThis
 */
void MGC_HdrcServiceDeviceDefaultEnd(MGC_LinuxCd * pThis)
{
	MGC_HdrcServiceFunctionEp0(pThis);
}

/*
 * The endpoint selection policy should have already selected
 * an appropriate usb_ep from us, since we follow the naming convention.
 * @param ep
 * @param desc
 */
static int MGC_GadgetEnableEnd(struct usb_ep *ep,
			       const struct usb_endpoint_descriptor *desc)
{
	unsigned long flags;
	struct musb_ep	*pEnd;
	struct musb	*pThis;
	u8		*pBase;
	u8		bEnd;
	u16		csr;
	unsigned	tmp;
	int		status = -EINVAL;

	if (!ep || !desc)
		return -EINVAL;

	pEnd = to_musb_ep(ep);
	pThis = pEnd->pThis;
	pBase = pThis->pRegs;
	bEnd = pEnd->bEndNumber;

	/* ep0 is always enabled */
	if (bEnd == 0)
		return -EINVAL;

	spin_lock_irqsave(&pThis->Lock, flags);

	if (pEnd->desc) {
		status = -EBUSY;
		goto fail;
	}
	pEnd->bTrafficType = desc->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK;

	/* check direction and maxpacket size against endpoint,
	 * enable the interrupts for the endpoint, set the endpoint
	 * packet size, set the mode, clear the fifo
	 */
	if ((desc->bEndpointAddress & USB_ENDPOINT_NUMBER_MASK) != bEnd)
		goto fail;

	/* REVISIT this rules out high bandwidth periodic transfers */
	tmp = le16_to_cpu(desc->wMaxPacketSize);
	if (tmp & ~0x07ff)
		goto fail;
	pEnd->wPacketSize = tmp;

	MGC_SelectEnd(pBase, bEnd);
	if (desc->bEndpointAddress & USB_DIR_IN) {
		u16 wIntrTxE = MGC_Read16(pBase, MGC_O_HDRC_INTRTXE);

		pEnd->bIsTx = 1;
		if (tmp > pEnd->hw_ep->wMaxPacketSizeTx)
			goto fail;

		wIntrTxE |= (1 << bEnd);
		MGC_Write16(pBase, MGC_O_HDRC_INTRTXE, wIntrTxE);

		MGC_WriteCsr16(pBase, MGC_O_HDRC_TXMAXP, bEnd, tmp);

		csr = MGC_M_TXCSR_MODE | MGC_M_TXCSR_CLRDATATOG
				| MGC_M_TXCSR_FLUSHFIFO;
		if (pEnd->bTrafficType == USB_ENDPOINT_XFER_ISOC)
			csr |= MGC_M_TXCSR_ISO;

		/* set twice in case of double buffering */
		MGC_WriteCsr16(pBase, MGC_O_HDRC_TXCSR, bEnd, csr);
		MGC_WriteCsr16(pBase, MGC_O_HDRC_TXCSR, bEnd, csr);

	} else {
		u16 wIntrRxE = MGC_Read16(pBase, MGC_O_HDRC_INTRRXE);

		pEnd->bIsTx = 0;
		if (tmp > pEnd->hw_ep->wMaxPacketSizeRx)
			goto fail;

		wIntrRxE |= (1 << bEnd);
		MGC_Write16(pBase, MGC_O_HDRC_INTRRXE, wIntrRxE);

		MGC_WriteCsr16(pBase, MGC_O_HDRC_RXMAXP, bEnd, tmp);

		/* force shared fifo to OUT-only mode */
		if (pEnd->hw_ep->bIsSharedFifo) {
			csr = MGC_Read16(pBase, MGC_O_HDRC_TXCSR);
			csr &= ~MGC_M_TXCSR_MODE;
			MGC_WriteCsr16(pBase, MGC_O_HDRC_TXCSR, bEnd, csr);
		}

		csr = MGC_M_RXCSR_FLUSHFIFO | MGC_M_RXCSR_CLRDATATOG;
		if (pEnd->bTrafficType == USB_ENDPOINT_XFER_ISOC)
			csr |= MGC_M_RXCSR_P_ISO;
		else if (pEnd->bTrafficType == USB_ENDPOINT_XFER_INT)
			csr |= MGC_M_RXCSR_DISNYET;

		/* set twice in case of double buffering */
		MGC_WriteCsr16(pBase, MGC_O_HDRC_RXCSR, bEnd, csr);
		MGC_WriteCsr16(pBase, MGC_O_HDRC_RXCSR, bEnd, csr);
	}

	pEnd->desc = desc;
	pEnd->bInactive = MUSB_GADGET_EP_ACTIVE;

	status = 0;

	pr_debug("%s periph: enabled %s for %s %s, maxpacket %d\n",
			musb_driver_name, pEnd->end_point.name,
			({ char *s; switch (pEnd->bTrafficType) {
			 case USB_ENDPOINT_XFER_BULK:	s = "bulk"; break;
			 case USB_ENDPOINT_XFER_INT:	s = "int"; break;
			 default:			s = "iso"; break;
			 }; s; }), 
			 pEnd->bIsTx ? "IN" : "OUT", pEnd->wPacketSize);

fail:
	spin_unlock_irqrestore(&pThis->Lock, flags);
	return status;
}

/*
 * Disable an endpoint flushing all requests queued.
 *
 * @param struct usb_ep *ep the endpoint to disable.
 */
static int MGC_GadgetDisableEnd(struct usb_ep *ep)
{
	unsigned int flags;
	struct musb	*pThis;
	u8		bEnd;
	struct musb_ep	*pEnd;

	pEnd = to_musb_ep(ep);
	pThis = pEnd->pThis;
	bEnd = pEnd->bEndNumber;

	/* REVISIT:  abort DMA for current request... */

	/* disable the interrupts for the endpoints 
	 * reset the endpoint size to zero 
	 */
	spin_lock_irqsave(&pThis->Lock, flags);
	MGC_SelectEnd(pThis->pRegs, bEnd);
	if (pEnd->bIsTx) {
		u16 wIntrTxE = MGC_Read16(pThis->pRegs, MGC_O_HDRC_INTRTXE);
		wIntrTxE &= ~(1 << bEnd);
		MGC_Write16(pThis->pRegs, MGC_O_HDRC_INTRTXE, wIntrTxE);
		MGC_WriteCsr16(pThis->pRegs, MGC_O_HDRC_TXMAXP, bEnd, 0);
	} else {
		u16 wIntrRxE = MGC_Read16(pThis->pRegs, MGC_O_HDRC_INTRRXE);
		wIntrRxE &= ~(1 << bEnd);
		MGC_Write16(pThis->pRegs, MGC_O_HDRC_INTRRXE, wIntrRxE);
		MGC_WriteCsr16(pThis->pRegs, MGC_O_HDRC_RXMAXP, bEnd, 0);
	}

	pEnd->desc = NULL;

	MGC_FlushEpRequests(pEnd, -ESHUTDOWN);
	spin_unlock_irqrestore(&(pThis->Lock), flags);

	DBG(2, "%s periph: disabled %s\n", musb_driver_name,
			pEnd->end_point.name);

	return 0;
}

/*
 * Allocate a request for an endpoint.
 * @param ep
 * @param gfp_flags
 */
static struct usb_request *MGC_GadgetAllocRequest(struct usb_ep *ep,
						  int gfp_flags)
{
	struct musb_ep		*musb_ep = to_musb_ep(ep);
	struct musb_request	*pRequest = NULL;

	pRequest = kzalloc(sizeof *pRequest, gfp_flags);
	if (pRequest) {
		INIT_LIST_HEAD(&pRequest->request.list);
		pRequest->bEnd = musb_ep->bEndNumber;
	}

	return &pRequest->request;
}

/*
 * Free a request
 *
 * @param ep
 * @param req
 */
static void MGC_GadgetFreeRequest(struct usb_ep *ep, struct usb_request *req)
{
	kfree(to_musb_request(req));
}

#ifdef MUSB_PARANOID
static int ReqCount = 0;
static int ReqCap = 7;
static int ReqEnd = -1;
#endif

/*
 * @context: controller locked, IRQs blocked.
 */
static void MGC_RestartRequest(struct musb *pThis, struct usb_request *_req)
{
	struct musb_request *req;

	req = to_musb_request(_req);
	DBG(3, "<== starting %s request %p on ep=%d\n",
	    req->bTx ? "TX/IN" : "RX/OUT",
	    _req, req->bEnd);

	if (req->bTx) {
		txstate(pThis, req);
	} else {
		rxstate(pThis, req);
	}
}

/*
 * Queue requests to endpoints for execution.
 * 
 * @param struct usb_ep *ep
 * @param struct usb_request *pRequest
 * @param int gfp_flags
 */
static int MGC_GadgetQueue(struct usb_ep *ep, struct usb_request *req,
			   int gfp_flags)
{
	struct musb_ep		*pEnd;
	struct musb_request	*pRequest;
	struct musb		*pThis;
	u8			bEnd;
	int			status = 0;
	unsigned long		lockflags;

	if (!ep || !req)
		return -EINVAL;

	pEnd = to_musb_ep(ep);
	pThis = pEnd->pThis;
	bEnd = pEnd->bEndNumber;

	pRequest = to_musb_request(req);
	pRequest->musb = pThis;

	DBG(4, "<== to %s request=%p\n", ep->name, req);

	/* request is mine now... */
	pRequest->request.actual = 0;
	pRequest->request.status = -EINPROGRESS;
	pRequest->bEnd = bEnd;
	pRequest->bTx = pEnd->bIsTx;

	/* lock the endpoint */
	EP_SPIN_LOCK_IRQSAVE(ep, lockflags);

	/* don't queue if the ep is down */
	if (!pEnd->desc && bEnd) {
		WARN("req %p queued to %s while ep %s\n",
				req, ep->name, "disabled");
		status = -ESHUTDOWN;
		goto cleanup;
	}

	if (bEnd == 0) {
		switch (pThis->bEnd0Stage) {
		case MGC_END0_STAGE_RX:		/* control-OUT data */
		case MGC_END0_STAGE_TX:		/* control-IN data */
		case MGC_END0_STAGE_STATUSIN:	/* zero-length data */
			break;
		default:
			WARN("ep0 request queued in state %d\n",
					pThis->bEnd0Stage);
			status = -EINVAL;
			goto cleanup;
		}
	}

	/* add pRequest to the list */
	list_add_tail(&(pRequest->request.list), &(pEnd->req_list));

	/* it this is not the head of the queue, done... */
	if (pRequest != to_musb_request(MGC_CurrentRequest(pEnd))) {
		goto cleanup;
	}

	/* don't start if ep is halted */
	if (pEnd->bInactive == MUSB_GADGET_EP_HALTED) {
		WARN("req %p queued to %s while ep %s\n",
				req, ep->name, "halted");
		goto cleanup;
	}
#ifdef MUSB_PARANOID
	if (bEnd == ReqEnd) {
		ReqCount++;
	}

	if (ReqCount >= ReqCap) {
		WARN("ReqCount=%d on ep%d\n", ReqCount, bEnd);
		goto cleanup;
	}
#endif

// FIXME surely "unlocked" here is the wrong idea ...
// that is, dropping the lock we now hold.
// the problem is the usual "two locks for one thing":
// we hold ep->lock, this path uses this->lock for the same thing
// ... and hey, precondition for ep0_*xstate() is "must be locked"

	/* start the request otherwise; the EP MUST BE UNLOCKED */
	if (bEnd == 0) {
		EP_SPIN_UNLOCK_IRQRESTORE(ep, lockflags);
		DBG(3, "queue to %s (%s), pRequest->length=%d\n",
				ep->name, pEnd->bIsTx ? "IN/TX" : "OUT/RX",
				pRequest->request.length);

		/* sequence #1, IN ... start writing the data */
		if (pThis->bEnd0Stage == MGC_END0_STAGE_TX)
			ep0_txstate(pThis);

		/* else for sequence #2 (OUT), caller provides a buffer
		 * before the next packet arrives.  deferred responses
		 * (after SETUP is acked) are racey.
		 *
		 * REVISIT sequence #3 (no-data) is ugly...
		 */
	} else {
// NOTE:  invariant, controller locked on entry here
		MGC_RestartRequest(pThis, req);
		goto cleanup;
	}

	return 0;
cleanup:
	EP_SPIN_UNLOCK_IRQRESTORE(ep, lockflags);
	return status;
}

/*
 * Dequeue a request
 *
 * @param ep the endpoint
 * @param pRequest the request to dequeue  
 */
static int MGC_GadgetDequeue(struct usb_ep *ep, struct usb_request *pRequest)
{
	struct musb_ep *pEnd = to_musb_ep(ep);

	if (!ep || !pRequest)
		return -EINVAL;

	DBG(3, "dequeuing from nEnd=0x%x, pEnd=%p, pRequest=%p\n",
			pEnd->bEndNumber, pEnd, pRequest);

#ifdef MUSB_PARANOID
	if (!MGC_GetEpDriver(ep)) {
		ERR("Gadget not initialized, pThis=NULL\n");
		return -EINVAL;
	}

	EP_SPIN_LOCK(pEnd); {
		struct usb_request *req = NULL;

		list_for_each_entry(req, &pEnd->req_list, list) {
			if (req == pRequest) {
				break;
			}
		}

		if (req != pRequest) {
			ERR("request %p not queued to ep%d\n", pRequest,
					pEnd->bEndNumber);
			return -EINVAL;
		}
	}
	EP_SPIN_UNLOCK(pEnd);
#endif

	/* flush the request returning -EINVAL; syncronnous */
	EP_SPIN_LOCK(pEnd);
	pRequest->status = -EINVAL;
	if (pRequest->complete) {
		list_del(&pRequest->list);
		EP_SPIN_UNLOCK(pEnd);
		pRequest->complete(&pEnd->end_point, pRequest);
	} else {
		EP_SPIN_UNLOCK(pEnd);
	}

	return 0;
}

/*
 * Set clear the halt bit of an endpoint. A halted enpoint won't tx/rx any
 * data but will queue requests. 
 * 
 * @param ep the endpoint
 * @param value != 0 => halt, 0 == active  
 */
int MGC_GadgetSetHalt(struct usb_ep *ep, int value)
{
	struct musb_ep		*pEnd;
	u8			bEnd;
	struct musb		*pThis;
	u8			*pBase;
	unsigned long		flags;
	u16			wCsr;
	struct musb_request	*pRequest = NULL;
	int			status = 0;

	if (!ep)
		return -EINVAL;

	pEnd = to_musb_ep(ep);
	bEnd = pEnd->bEndNumber;
	pThis = pEnd->pThis;
	pBase = pThis->pRegs;

	DBG(2, "<== end=%d, value=%d\n", bEnd, value);

	spin_lock_irqsave(&pThis->Lock, flags);

	if ((USB_ENDPOINT_XFER_ISOC == pEnd->bTrafficType)
			|| (bEnd && pEnd->desc == NULL)) {
		status = -EINVAL;
		goto done;
	}

	MGC_SelectEnd(pBase, bEnd);
	if (0 == bEnd) {
		/* REVISIT protocol stalls are only legal while we're
		 * waiting for the gadget driver to reply...
		 */
		wCsr = MGC_ReadCsr16(pBase, MGC_O_HDRC_CSR0, 0);
		if (value) {
			wCsr |= MGC_M_CSR0_P_SENDSTALL;
		} else {
			wCsr &= ~MGC_M_CSR0_P_SENDSTALL;
		}

		MGC_WriteCsr16(pBase, MGC_O_HDRC_CSR0, 0, wCsr);
		spin_unlock_irqrestore(&pThis->Lock, flags);
		return 0;
	}


	/* cannot portably stall with non-empty FIFO */
	pRequest = to_musb_request(MGC_CurrentRequest(pEnd));
	if (value && pEnd->bIsTx) {
		wCsr = MGC_ReadCsr16(pBase, MGC_O_HDRC_TXCSR, bEnd);
		if (wCsr & MGC_M_TXCSR_FIFONOTEMPTY) {
			DBG(3, "ep=%d: %d request(s), %s cannot halt it\n",
			    bEnd, queue_length(&pEnd->req_list),
			    dump_usb_request(&pRequest->request));
			spin_unlock_irqrestore(&pThis->Lock, flags);
			return -EAGAIN;
		}

	}

	/* set/clear the stall bit */
	DBG(3, "%s the stall bit\n", (value) ? "set" : "clear");
	if (pEnd->bIsTx) {
		wCsr = MGC_ReadCsr16(pBase, MGC_O_HDRC_TXCSR, bEnd);
		if (value) {
			wCsr |= MGC_M_TXCSR_P_SENDSTALL;
		} else {
			wCsr &= ~MGC_M_TXCSR_P_SENDSTALL;
		}

		MGC_WriteCsr16(pBase, MGC_O_HDRC_TXCSR, bEnd, wCsr);
	} else {
		wCsr = MGC_ReadCsr16(pBase, MGC_O_HDRC_RXCSR, bEnd);
		if (value) {
			wCsr |= MGC_M_RXCSR_P_SENDSTALL;
		} else {
			wCsr &= ~MGC_M_RXCSR_P_SENDSTALL;
		}

		MGC_WriteCsr16(pBase, MGC_O_HDRC_RXCSR, bEnd, wCsr);
	}

	/* REVISIT hardware should prevent progress till un-stall */
	pEnd->bInactive = value
			? MUSB_GADGET_EP_HALTED
			: MUSB_GADGET_EP_ACTIVE;

done:
	spin_unlock_irqrestore(&pThis->Lock, flags);

	/* the ep has been re-activated, re-start the request if one 
	 * is pending*/
	if (!value && pRequest) {
		DBG(3, "restarting the request\n");
		/* REVISIT races with other contexts ... */
		MGC_RestartRequest(pThis, &pRequest->request);
	}
	return status;
}

/****************************************************************
 * Registration operations 
 ****************************************************************/

/* Only this registration code "knows" the rule (from USB standards)
 * about there being only one external upstream port.  It assumes
 * all peripheral ports are external...
 */
static struct musb *the_gadget;

static void musb_gadget_release(struct device *dev)
{
	// kref_put(WHAT)
	dev_dbg(dev, "%s\n", __FUNCTION__);
}

/* called once during driver setup to initialize and link into
 * the driver model; memory is zeroed.
 */
int __init musb_gadget_setup(struct musb *pThis)
{
	int status;

	/* REVISIT minor race:  if (erroneously) setting up two
	 * musb peripherals at the same time, only the bus lock
	 * is probably held.
	 */
	if (the_gadget)
		return -EBUSY;
	the_gadget = pThis;

	pThis->g.ops = &MGC_GadgetOperations;
	pThis->g.is_dualspeed = 1;
	pThis->g.speed = USB_SPEED_UNKNOWN;
#ifdef CONFIG_USB_MUSB_OTG
	/* FIXME only if the board is using OTG mode */
	pThis->g.is_otg = 1;
#endif

	/* this "gadget" abstracts/virtualizes the controller */
	strcpy(pThis->g.dev.bus_id, "gadget");
	pThis->g.dev.parent = pThis->controller;
	pThis->g.dev.dma_mask = pThis->controller->dma_mask;
	pThis->g.dev.release = musb_gadget_release;
	pThis->g.name = musb_driver_name;

	MGC_InitGadgetEndPoints(pThis);

	/* (DIAG ISSUE?) messages should NOT break SET_ADDRESS */
	// MGC_SetDebugLevel(0);

	status = device_register(&pThis->g.dev);
	if (status != 0)
		the_gadget = 0;
	return status;
}

#if 0
 /**
  * Cleanup of the controller instance. 
 * @param pThis the controller instance.
  */
void musb_gadget_cleanup(MGC_LinuxCd * pThis)
{
	if (pThis != the_gadget)
		return;

	device_unregister(&pThis->g.dev);

	/* clean up everything */
	pThis->pGadgetDriver = NULL;

#ifdef CONFIG_USB_MUSB_OTG
	// IFF board is otg-enabled
	printk("FIXME -- %s usb_deregister_bus()\n", __FUNCTION__);
#endif
	the_gadget = NULL;
}
#endif

/*
 * Register the gadget driver. Used by gadget drivers when 
 * registering themselves with the controller.
 *
 * -EINVAL something went wrong (not driver)
 * -EBUSY another gadget is already using the controller
 * -ENOMEM no memeory to perform the operation
 *
 * @param driver the gadget driver
 * @return <0 if error, 0 if everything is fine 
 */
int usb_gadget_register_driver(struct usb_gadget_driver *driver)
{
	int retval;
	unsigned long flags;
	MGC_LinuxCd *pThis = the_gadget;

	/* REVISIT if this board is in pure-host mode, fail */

	if (!driver
			|| driver->speed != USB_SPEED_HIGH
	    		|| !driver->bind
			|| !driver->unbind
			|| !driver->setup)
		return -EINVAL;

	/* verify/impose that the driver is initialized */
	if (!pThis) {
		ERR("%s, no dev??\n", __FUNCTION__);
		return -ENODEV;
	}

	DBG(3, "registering driver %s\n", driver->function);
	spin_lock_irqsave(&pThis->Lock, flags);

	if (pThis->pGadgetDriver) {
		ERR("musb-gcd is already used by %s, unregister it first\n",
		    pThis->pGadgetDriver->driver.name);
		retval = -EBUSY;
	} else {
		pThis->pGadgetDriver = driver;
		pThis->g.dev.driver = &driver->driver;
		driver->driver.bus = NULL;
		retval = 0;
	}

	spin_unlock_irqrestore(&pThis->Lock, flags);

	if (retval == 0)
		retval = driver->bind(&pThis->g);
	if (retval != 0) {
		ERR("bind to driver %s failed --> %d\n",
		    driver->driver.name, retval);
		pThis->pGadgetDriver = NULL;
		pThis->g.dev.driver = NULL;
	}

	/* start peripheral and/or OTG engines */
	if (!retval) {
#ifdef CONFIG_USB_MUSB_OTG
		if (pThis->board_mode == MUSB_OTG) {
			DBG(3, "OTG startup...\n");

			/* REVISIT:  funcall to other code, which also
			 * handles power budgeting ... this way also
			 * ensures HdrcStart is indirectly called.
			 */
			retval = usb_register_root_hub(
					pThis->RootHub.pDevice,
					pThis->g.dev.parent);
			// ... and if that failed, back out ...
		} else
#endif
		{
			MGC_HdrcStart(pThis);
		}
	}

	return retval;
}

EXPORT_SYMBOL(usb_gadget_register_driver);

/*
 * Unregister the gadget driver. Used by gadget drivers when 
 * unregistering themselves from the controller.
 *
 * @param driver the gadget driver to unregister
 */
int usb_gadget_unregister_driver(struct usb_gadget_driver *driver)
{
	unsigned long flags;
	int retval;
	MGC_LinuxCd *pThis = the_gadget;

	if (!driver)
		return -EINVAL;

#ifdef MUSB_PARANOID
	if (!pThis) {
		ERR("Gadget not initialized, pThis=NULL\n");
		return -EINVAL;
	}
#endif

	// FIXME this call doesn't do much w/o host enabled...
	// and probably needs to mark the device as unavailable
	// and certainly needs to turn the pullup off
	MGC_HdrcStop(pThis);

	spin_lock_irqsave(&pThis->Lock, flags);

	/* FIXME flush/abort any pending requests */
	if (driver && pThis->pGadgetDriver == driver) {
		INFO("unregistering driver %s\n", driver->function);
		driver->unbind(&pThis->g);
		pThis->pGadgetDriver = NULL;
		pThis->g.dev.driver = NULL;
		retval = 0;
	} else
		retval = -EINVAL;

	spin_unlock_irqrestore(&pThis->Lock, flags);
	return retval;
}

EXPORT_SYMBOL(usb_gadget_unregister_driver);
