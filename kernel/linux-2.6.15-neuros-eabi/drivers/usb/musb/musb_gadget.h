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

#ifndef __MUSB_GADGET_H
#define __MUSB_GADGET_H

/**
 * struct musb_ep - peripheral side view of endpoint rx or tx side
 *
 * @field Lock spinlock
 * @field pRequest current request
 * @field wPacketSize programmed packet size
 * @field bTrafficType programmed traffic type
 * @field bIsTx TRUE if current direction is Tx
 */
struct musb_ep {
	/* stuff towards the head is basically write-once. */
	struct usb_ep			end_point;
	char				name[12];
	struct musb_hw_ep		*hw_ep;
	const struct usb_endpoint_descriptor	*desc;
	struct musb			*pThis;
	u8				bEndNumber;

	/* later things are modified based on usage */
	struct list_head		req_list;

#if MUSB_DEBUG>0
	u32 wPadFront;
#define MGC_LOCAL_PAD  0xa5deadfe
#endif

	spinlock_t Lock;

	unsigned int dwRequestSize;
	u16 wPacketSize;

	u8 bTrafficType;
	unsigned bIsTx:1;

	/* set when the ep is halted/request are queued but not executed */
	u8 bInactive;
#define MUSB_GADGET_EP_ACTIVE		0	/* ignoring ep->desc */
#define MUSB_GADGET_EP_HALTED		1

};

static inline struct musb_ep *to_musb_ep(struct usb_ep *ep)
{
	return ep ? container_of(ep, struct musb_ep, end_point) : NULL;
}

// FIXME remove typedefs
typedef struct musb_ep MGC_GadgetLocalEnd;

/* FIXME remove most globals */
extern MGC_GadgetLocalEnd MGC_aGadgetLocalEnd[];

extern void MGC_FlushRequests(struct musb *pThis, int code);

extern void MGC_HdrcServiceDeviceTxAvail(struct musb *pThis, u8 bEnd);
extern void MGC_HdrcServiceDeviceRxReady(struct musb *pThis, u8 bEnd);
extern void MGC_HdrcLoadFifo(const u8 * pBase, u8 bEnd,
			     u16 wCount, const u8 * pSource);
extern void MGC_HdrcUnloadFifo(const u8 * pBase, u8 bEnd,
			       u16 wCount, u8 * pDest);

/* export from musb_gadget.c */
extern int MGC_LinuxGadgetQueue(const u8 nEnd, struct usb_request *pRequest,
				int gfp_flags);
extern void MGC_GadgetReset(struct musb *pThis);
extern void MGC_GadgetResume(struct musb *pThis);
extern void MGC_GadgetSuspend(struct musb *pThis);
extern void MGC_GadgetDisconnect(struct musb *pThis);

extern void MGC_HdrcHandleGadgetEp0Request(struct usb_request *req);

/* gstorage only */
extern void MGC_HdrcServiceDeviceDefaultEnd(struct musb *pThis);


/* ------------------------------- globals  ---------------------------- */

extern struct usb_ep_ops MGC_GadgetEndpointOperations;

/* in gadgetcommon.c */
extern MGC_GadgetLocalEnd MGC_aGadgetLocalEnd[];	/* counters and queues */

/* ------------------------ function prototypes --------------------- */

// FIXME remove this; pThis isn't a driver!!!
#define MGC_GetEpDriver(_ep)	(to_musb_ep(_ep)->pThis)

extern void *MGC_GadgetAllocBuffer(struct usb_ep *ep, unsigned bytes,
				   dma_addr_t * dma, int gfp_flags);
extern void MGC_GadgetFreeBuffer(struct usb_ep *ep, void *buf, dma_addr_t dma,
				 unsigned bytes);
extern int MGC_GadgetFifoStatus(struct usb_ep *ep);
extern void MGC_GadgetFifoFlush(struct usb_ep *ep);

extern int musb_gadget_setup(struct musb *);

extern void MGC_HdrcServiceDeviceTxAvail(struct musb *pThis, u8 bEnd);
extern void MGC_HdrcServiceDeviceRxReady(struct musb *pThis, u8 bEnd);
extern void MGC_HdrcLoadFifo(const u8 * pBase, u8 bEnd,
			     u16 wCount, const u8 * pSource);
extern void MGC_HdrcUnloadFifo(const u8 * pBase, u8 bEnd,
			       u16 wCount, u8 * pDest);

/* export from musb_gadget.c */
extern int MGC_LinuxGadgetQueue(const u8 nEnd, struct usb_request *pRequest,
				int gfp_flags);

extern void MGC_HdrcHandleGadgetEp0Request(struct usb_request *req);

extern int ep0_txstate(struct musb *);

static inline struct usb_request *MGC_CurrentRequest(MGC_GadgetLocalEnd * pEnd)
{
	/* endpoint is locked indirectly, via controller */
	if (list_empty(&pEnd->req_list))
		return NULL;
	return list_entry(pEnd->req_list.next, struct usb_request, list);
}

extern int MGC_GadgetSetHalt(struct usb_ep *ep, int value);

/* --------------------------- commodities  --------------------------- */

/* FIXME abolish these spinlock wrappers ... also per-endpoint locks! */

#define EP_SPIN_LOCK(_ep)	spin_lock( &((MGC_GadgetLocalEnd*)(_ep))->Lock )
#define EP_SPIN_UNLOCK(_ep)	spin_unlock( &((MGC_GadgetLocalEnd*)(_ep))->Lock )

#define EP_SPIN_LOCK_IRQSAVE(_ep, _flags) spin_lock_irqsave(&((MGC_GadgetLocalEnd*)(_ep))->Lock, _flags )
#define EP_SPIN_UNLOCK_IRQRESTORE(_ep, _flags) spin_unlock_irqrestore( &((MGC_GadgetLocalEnd*)(_ep))->Lock, _flags)

#endif		/* __MUSB_GADGET_H */
