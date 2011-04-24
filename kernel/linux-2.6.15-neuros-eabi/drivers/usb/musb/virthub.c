/*****************************************************************
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

/*
 * A virtual hub intended for embedding in HCDs
 */

#include <linux/config.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/time.h>
#include <linux/timer.h>

#include <linux/usb.h>

#include "../core/hcd.h"

#include "musbdefs.h"
#include "musb_host.h"

/* FIXME most of this file will vanish when we convert this driver
 * over to the standard 2.6 hcd framework
 */

/****************************** FORWARDS ********************************/

static void MGC_VirtualHubActivateTimer(MGC_VirtualHub * pHub,
					void (*pfExpired) (unsigned long),
					unsigned long timeout);
static void MGC_VirtualHubCompleteIrq(MGC_VirtualHub * pHub, struct urb *pUrb,
				      int status);
static void MGC_VirtualHubTimerExpired(unsigned long ptr);

/****************************** GLOBALS *********************************/

/* device descriptor */
static u8 MGC_aVirtualHubDeviceDesc[] = {
	USB_DT_DEVICE_SIZE,
	USB_DT_DEVICE,
	0x00, 0x02,		/* bcdUSB */
	USB_CLASS_HUB,		/* bDeviceClass */
	0,			/* bDeviceSubClass */
	1,			/* bDeviceProtocol (single TT) */
	64,			/* bMaxPacketSize0 */
	0xd6, 0x4,		/* idVendor */
	0, 0,			/* idProduct */
	0, 0,			/* bcdDevice */
	0,			/* iManufacturer */
	0,			/* iProduct */
	0,			/* iSerialNumber */
	1			/* bNumConfigurations */
};

/* device qualifier */
static u8 MGC_aVirtualHubQualifierDesc[] = {
	sizeof(struct usb_qualifier_descriptor),
	USB_DT_DEVICE_QUALIFIER,
	0x00, 0x02,		/* bcdUSB */
	USB_CLASS_HUB,		/* bDeviceClass */
	0,			/* bDeviceSubClass */
	0,			/* bDeviceProtocol */
	64,			/* bMaxPacketSize0 */
	0xd6, 0x4,		/* idVendor */
	0, 0,			/* idProduct */
	0, 0,			/* bcdDevice */
	0,			/* iManufacturer */
	0,			/* iProduct */
	0,			/* iSerialNumber */
	1			/* bNumConfigurations */
};

/* Configuration descriptor */
static u8 MGC_VirtualHubConfigDesc[] = {
	USB_DT_CONFIG_SIZE,
	USB_DT_CONFIG,
	USB_DT_CONFIG_SIZE + USB_DT_INTERFACE_SIZE + USB_DT_ENDPOINT_SIZE, 0,
	0x01,			/* bNumInterfaces */
	0x01,			/* bConfigurationValue */
	0x00,			/* iConfiguration */
	0xE0,			/* bmAttributes (self-powered, remote wake) */
	0x00,			/* MaxPower */

	/* interface */
	USB_DT_INTERFACE_SIZE,
	USB_DT_INTERFACE,
	0x00,			/* bInterfaceNumber */
	0x00,			/* bAlternateSetting */
	0x01,			/* bNumEndpoints */
	USB_CLASS_HUB,		/* bInterfaceClass */
	0x00,			/* bInterfaceSubClass */
	0x00,			/* bInterfaceProtocol */
	0x00,			/* iInterface */

	/* endpoint */
	USB_DT_ENDPOINT_SIZE,
	USB_DT_ENDPOINT,
	USB_DIR_IN | 1,		/* bEndpointAddress: IN Endpoint 1 */
	USB_ENDPOINT_XFER_INT,	/* bmAttributes: Interrupt */
	(MGC_VIRTUALHUB_MAX_PORTS + 8) / 8, 0,	/* wMaxPacketSize */
	12			/* bInterval: 256 ms */
};

/* other-speed Configuration descriptor */
static u8 MGC_VirtualHubOtherConfigDesc[] = {
	USB_DT_CONFIG_SIZE,
	/* FIXME other-speed does NOT belong in config descriptors! */
	USB_DT_OTHER_SPEED_CONFIG,
	USB_DT_CONFIG_SIZE + USB_DT_INTERFACE_SIZE + USB_DT_ENDPOINT_SIZE, 0,
	0x01,			/* bNumInterfaces */
	0x01,			/* bConfigurationValue */
	0x00,			/* iConfiguration */
	0xE0,			/* bmAttributes (self-powered, remote wake) */
	0x00,			/* MaxPower */

	/* interface */
	USB_DT_INTERFACE_SIZE,
	USB_DT_INTERFACE,
	0x00,			/* bInterfaceNumber */
	0x00,			/* bAlternateSetting */
	0x01,			/* bNumEndpoints */
	USB_CLASS_HUB,		/* bInterfaceClass */
	0x00,			/* bInterfaceSubClass */
	0x00,			/* bInterfaceProtocol */
	0x00,			/* iInterface */

	/* endpoint */
	USB_DT_ENDPOINT_SIZE,
	USB_DT_ENDPOINT,
	USB_DIR_IN | 1,		/* bEndpointAddress: IN Endpoint 1 */
	USB_ENDPOINT_XFER_INT,	/* bmAttributes: Interrupt */
	(MGC_VIRTUALHUB_MAX_PORTS + 8) / 8, 0,	/* wMaxPacketSize */
	0xff			/* bInterval: 255 ms */
};

/***************************** FUNCTIONS ********************************/

/*
 * Generic timer activation helper. Requires the hub structure to 
 * be locked.
 * 
 * @param pHub pointer to hub struct
 * @param pfExpired callback function
 * @param timeout millisecs
 * @requires spin_lock(pHub->Lock)
 */
static void MGC_VirtualHubActivateTimer(MGC_VirtualHub * pHub,
					void (*pfExpired) (unsigned long),
					unsigned long timeout)
{

	del_timer(&pHub->Timer);	/* make sure another timer is not running */

	init_timer(&(pHub->Timer));
	pHub->Timer.function = pfExpired;
	pHub->Timer.data = (unsigned long)pHub;
	pHub->Timer.expires = jiffies + timeout * HZ / 1000;
	add_timer(&(pHub->Timer));
}

/*
 * assumes pHub to be locked!
 * @requires spin_lock(pHub->Lock)
 */
static void MGC_VirtualHubCompleteIrq(MGC_VirtualHub * pHub, struct urb *pUrb,
				      int status)
{
	int nLength, nPort;
	u8 bData, bBit;
	u8 *pData;

	pHub->bIsChanged = FALSE;

	/* how many bits are needed/possible */
	nLength = min(pUrb->transfer_buffer_length * 8, pHub->bPortCount + 1);
	bData = 0;
	bBit = 1;
	pData = (u8 *) pUrb->transfer_buffer;

	/* count 1..N to accomodate hub status bit */
	for (nPort = 1; nPort <= nLength; nPort++) {
		if (pHub->aPortStatusChange[nPort - 1].wChange & 1) {
			bData |= 1 << bBit;
		}
		if (++bBit > 7) {
			*pData++ = bData;
			bData = bBit = 0;
		}
	}

	if (bBit) {
		*pData++ = bData;
	}

	pUrb->actual_length = (int)pData - (int)pUrb->transfer_buffer;
	if (pUrb->actual_length && pUrb->complete) {
		DBG(4, "completing hub interrupt URB\n");
		pUrb->status = status;
		if (status < 0) {
			pUrb->hcpriv = NULL;
		}
		pUrb->complete(pUrb, NULL);
	}
}

/*
 * Timer expiration function to complete the interrupt URB on changes
 * @param ptr standard expiration param (hub pointer)
 */
static void MGC_VirtualHubTimerExpired(unsigned long ptr)
{
	MGC_VirtualHub *pHub = (MGC_VirtualHub *) ptr;
	struct urb *pUrb;

	spin_lock(&pHub->Lock);
	pUrb = pHub->pUrb;

	if (pUrb && (pUrb->hcpriv == pHub)) {
		u8 bPort;

		for (bPort = 0; bPort < pHub->bPortCount; bPort++) {
			if (pHub->aPortStatusChange[bPort].wChange) {
				MGC_VirtualHubCompleteIrq(pHub, pUrb, 0);
				break;
			}
		}

		/* re-activate timer only when the urb is still mine; pUrb->hcpriv is
		 * set to NULL on port disconnect */
		MGC_VirtualHubActivateTimer(pHub, MGC_VirtualHubTimerExpired,
					    pHub->wInterval);
	} else {
		DBG(3, "pUrb=%p, for me =%d\n", pUrb,
		    (pUrb) ? ((pUrb->hcpriv) ? 1 : 0) : -1);
	}

	spin_unlock(&pHub->Lock);
}

/*
 * Initialize the virtual hub.
 * @param pHub  
 * @param pBus
 * @param bPortCount
 * @param pPortServices
 */
u8 MGC_VirtualHubInit(MGC_VirtualHub * pHub, struct usb_bus *pBus,
		      u8 bPortCount, MGC_PortServices * pPortServices)
{
	u8 bPort;

	if (bPortCount > MGC_VIRTUALHUB_MAX_PORTS) {
		ERR("Cannot allocate a %d-port device (too many ports)",
		    bPortCount);
		return FALSE;
	}

	/* allocate device */
	pHub->pDevice = usb_alloc_dev(NULL, pBus, 0);
	if (!pHub->pDevice) {
		ERR("Cannot allocate a %d-port device", bPortCount);
		return FALSE;
	}

	DBG(3, "New device (%d-port virtual hub) @%#lx allocated\n",
	    bPortCount, (unsigned long)pHub->pDevice);

	pHub->pBus = pBus;
	pHub->pDevice->speed = USB_SPEED_HIGH;

	spin_lock_init(&pHub->Lock);
	pHub->pUrb = NULL;
	pHub->pPortServices = pPortServices;
	pHub->bPortCount = bPortCount;
	pHub->bIsChanged = FALSE;
	init_timer(&(pHub->Timer));	/* I will need this later */

	for (bPort = 0; bPort < bPortCount; bPort++) {
		pHub->aPortStatusChange[bPort].wStatus = 0;
		pHub->aPortStatusChange[bPort].wChange = 0;
	}

	return TRUE;
}

/* Implementation */
void MGC_VirtualHubDestroy(MGC_VirtualHub * pHub)
{

}

/* Implementation */
void MGC_VirtualHubStop(MGC_VirtualHub * pHub)
{
	/* stop interrupt timer */
	del_timer_sync(&pHub->Timer);
}

/* Submit an URB to the virtual hub.
 *  bRequest:	
 *		00	
 *		01	
 *		03		
 *
 *	bmRequestType:
 *		0x23
 *		0xa3
 *
 * @param pHub the hub urb should be submitted to 
 * @param pUrb the urb to submit 
 */
int MGC_VirtualHubSubmitUrb(MGC_VirtualHub * pHub, struct urb *pUrb)
{
	u8 bRecip;		/* from standard request */
	u8 bReqType;		/* from standard request */
	u8 bType;		/* requested descriptor type */
	u16 wValue;		/* from standard request */
	u16 wIndex;		/* from standard request */
	u16 wLength;		/* from standard request */
	u8 bPort;
	const struct usb_ctrlrequest *pRequest;
	u16 wSize = 0xffff;
	u8 *pData = (u8 *) pUrb->transfer_buffer;
	unsigned int pipe = pUrb->pipe;

	DBG(2, "<==\n");

	spin_lock(&pHub->Lock);
	usb_get_urb(pUrb);

	pUrb->hcpriv = pHub;
	pUrb->status = -EINPROGRESS;
	if (usb_pipeint(pipe)) {
		DBG(3, "is periodic status/change event\n");

		/* this is the one for periodic status/change events */
		pHub->pUrb = pUrb;
		pHub->wInterval =
		    (pUrb->interval <
		     16) ? (1 << (pUrb->interval - 1)) : pUrb->interval;
		spin_unlock(&pHub->Lock);
		return 0;
	}

	/* handle hub requests/commands */
	pRequest = (const struct usb_ctrlrequest *)pUrb->setup_packet;
	bReqType = pRequest->bRequestType & USB_TYPE_MASK;
	bRecip = pRequest->bRequestType & USB_RECIP_MASK;
	wValue = le16_to_cpu(pRequest->wValue);
	wIndex = le16_to_cpu(pRequest->wIndex);
	wLength = le16_to_cpu(pRequest->wLength);

	DBG(3,
	    "pRequest->bRequest=%02x, pRequest->bRequestType=%02x, wLength=%04x\n",
	    pRequest->bRequest, pRequest->bRequestType, wLength);

	switch (pRequest->bRequest) {
	case USB_REQ_GET_STATUS:
		DBG(3, "GET_STATUS(), bType=%02x, bRecip=%02x, wIndex=%04x\n",
		    bReqType, bRecip, wIndex);

		if (USB_TYPE_STANDARD == bReqType) {
			/* self-powered */
			pData[0] = (USB_RECIP_DEVICE == bRecip) ? 1 : 0;
			pData[1] = 0;
			wSize = 2;
		} else if (USB_TYPE_CLASS == bReqType) {
			if ((USB_RECIP_OTHER == bRecip)
			    && (wIndex <= pHub->bPortCount)) {
				/* port status/change report */
				memcpy(pData,
				       &(pHub->aPortStatusChange[wIndex - 1].
					 wStatus), 2);
				memcpy(&(pData[2]),
				       &(pHub->aPortStatusChange[wIndex - 1].
					 wChange), 2);

				/* reset change (TODO: lock) */
				pHub->aPortStatusChange[wIndex - 1].wChange = 0;
				wSize = 4;
			} else {
				/* hub status */
				memset(pData, 0, 4);
				wSize = 4;
			}

			DBG(2, "status report=%02x%02x%02x%02x\n",
			    pData[0], pData[1], pData[2], pData[3]);
		}
		break;

	case USB_REQ_CLEAR_FEATURE:
		bPort = (u8) (wIndex & 0xff) - 1;
		DBG(3, "CLR_FEAT bReqType=0x%x, wValue=0x%x, wIndex=0x%x\n",
		    bReqType, wValue, (wIndex & 0xff));
		if ((USB_TYPE_STANDARD == bReqType)
		    && (USB_RECIP_ENDPOINT == bRecip)) {
			wSize = 0;
			DBG(3, "END POINT FEATURE!\n");
		} else if (USB_TYPE_CLASS == bReqType) {

			if (USB_RECIP_OTHER == bRecip) {
				bPort = (u8) (wIndex & 0xff) - 1;
				DBG(3, "CLEAR_PORT_FEATURE(%d), port %d\n",
				    wValue, bPort);
				switch (wValue) {
				case USB_PORT_FEAT_CONNECTION:
				case USB_PORT_FEAT_OVER_CURRENT:
				case USB_PORT_FEAT_POWER:
				case USB_PORT_FEAT_LOWSPEED:
				case USB_PORT_FEAT_HIGHSPEED:
				case USB_PORT_FEAT_TEST:
				case USB_PORT_FEAT_INDICATOR:
					DBG(3, "feat 0x%02x, wIndex=%d\n",
					    wValue, bPort);
					wSize = 0;
					break;
				case USB_PORT_FEAT_ENABLE:
					DBG(4, "enable port %d\n", bPort);
					pHub->pPortServices-> pfSetPortEnable(
						pHub->pPortServices->pPrivateData,
						bPort, FALSE);
					wSize = 0;
					break;
				case USB_PORT_FEAT_SUSPEND:
					DBG(3, "suspend port %d\n", bPort);
					pHub->pPortServices->pfSetPortSuspend(
						pHub->pPortServices->pPrivateData,
						bPort, FALSE);
					wSize = 0;
					break;
				case USB_PORT_FEAT_RESET:
					DBG(4, "reset port %d\n", bPort);
					pHub->pPortServices->pfSetPortReset(
						pHub->pPortServices->pPrivateData,
						bPort, FALSE);
					wSize = 0;
					break;

					/* acknowledge changes: */
				case USB_PORT_FEAT_C_CONNECTION:
					DBG(3, "ack connection port %d\n",
					    bPort);
					pHub->aPortStatusChange[bPort].
					    wChange &= ~1;
					wSize = 0;
					break;
				case USB_PORT_FEAT_C_ENABLE:
					DBG(3, "ack enable port %d\n", bPort);
					pHub->aPortStatusChange[bPort].
					    wChange &= ~USB_PORT_STAT_ENABLE;
					wSize = 0;
					break;
				case USB_PORT_FEAT_C_SUSPEND:
					DBG(3, "ack suspend port %d\n", bPort);

					pHub->aPortStatusChange[bPort].
					    wChange &= ~USB_PORT_STAT_SUSPEND;
					wSize = 0;
					break;
				case USB_PORT_FEAT_C_RESET:
					DBG(3, "ack reset port %d\n", bPort);
					pHub->aPortStatusChange[bPort].
					    wChange &= ~USB_PORT_STAT_RESET;
					wSize = 0;
					break;
				case USB_PORT_FEAT_C_OVER_CURRENT:
					DBG(3, "ack over current port %d\n",
					    bPort);
					wSize = 0;
					break;

				default:
					INFO("clear feature 0x%02x on port=%d unknown\n", wValue, bPort);
					break;
				}
			} else {
				DBG(3, "clear wValue=%d on port=%d\n", wValue,
				    bPort);
				switch (wValue) {
				case C_HUB_LOCAL_POWER:
				case C_HUB_OVER_CURRENT:
					wSize = 0;
					break;
				}
			}
			pHub->bIsChanged = TRUE;
		}
		break;

	case USB_REQ_SET_FEATURE:
		if ((USB_TYPE_CLASS == bReqType) && (USB_RECIP_OTHER == bRecip)) {
			bPort = (u8) (wIndex & 0xff) - 1;
			DBG(3, "SET_PORT_FEATURE(0x%02x), port %d\n", wValue,
			    bPort);
			switch (wValue) {
			case USB_PORT_FEAT_SUSPEND:
				DBG(3, "suspend port %d\n", bPort);
				pHub->pPortServices->pfSetPortSuspend(
						pHub->pPortServices->pPrivateData,
						bPort, TRUE);
				pHub->aPortStatusChange[bPort].wStatus |=
				    USB_PORT_STAT_SUSPEND;
				pHub->bIsChanged = TRUE;
				wSize = 0;
				break;

			case USB_PORT_FEAT_RESET:
				DBG(3, "reset port %d\n", bPort);
				pHub->aPortStatusChange[bPort].wStatus |=
				    USB_PORT_STAT_RESET;
				pHub->aPortStatusChange[bPort].wStatus |=
				    USB_PORT_STAT_ENABLE;
				pHub->aPortStatusChange[bPort].wChange |=
				    USB_PORT_STAT_RESET;
				pHub->bIsChanged = TRUE;
				pHub->pPortServices->pfSetPortReset(
						pHub->pPortServices->pPrivateData,
						bPort, TRUE);
				wSize = 0;
				break;

			case USB_PORT_FEAT_POWER:
				DBG(3, "power port %d\n", bPort);
				pHub->pPortServices->pfSetPortPower(
						pHub->pPortServices->pPrivateData,
						bPort, TRUE);
				pHub->aPortStatusChange[bPort].wStatus |=
				    USB_PORT_STAT_POWER;
				wSize = 0;
				break;

			case USB_PORT_FEAT_ENABLE:
				DBG(3, "enable port %d\n", bPort);
				pHub->pPortServices->pfSetPortEnable(
						pHub->pPortServices->pPrivateData,
						bPort, TRUE);
				pHub->aPortStatusChange[bPort].wStatus |=
				    USB_PORT_STAT_ENABLE;
				wSize = 0;
				break;
			}
		} else {
			DBG(3, "SET_FEATURE(%04x), but feature unknown\n",
			    wValue);
		}
		break;

	case USB_REQ_SET_ADDRESS:
		pHub->bAddress = (wValue & 0x7f);
		DBG(3, "SET_ADDRESS(%x) \n", pHub->bAddress);
		wSize = 0;
		break;

	case USB_REQ_GET_DESCRIPTOR:
		if (USB_TYPE_CLASS == bReqType) {
			DBG(3, "GET_CLASS_DESCRIPTOR()\n");

			pData[0] = 9;
			pData[1] = 0x29;
			pData[2] = pHub->bPortCount;
			/* min characteristics */
			/* individual port power switching (given
			 * platform_data->set_vbus); no overcurrent
			 */
			pData[3] = 0x11;
			pData[4] = 0;
			/* REVISIT ... report platform_data->potpgt */
			/* PowerOn2PowerGood */
			pData[5] = 50;
			/* no current */
			pData[6] = 0;
			/* removable ports */
			pData[7] = 0;
			/* reserved */
			pData[8] = 0xff;
			wSize = pData[0];
		} else {
			bType = (u8) (wValue >> 8);
			DBG(3, "GET_DESCRIPTOR(%d)\n", bType);
			switch (bType) {
			case USB_DT_DEVICE:	/* 1 */
				wSize = min(wLength,
					(u16) MGC_aVirtualHubDeviceDesc[0]);
				memcpy(pData, MGC_aVirtualHubDeviceDesc, wSize);
				break;
			case USB_DT_DEVICE_QUALIFIER:
				wSize = min(wLength,
					(u16) MGC_aVirtualHubQualifierDesc[0]);
				memcpy(pData, MGC_aVirtualHubQualifierDesc,
				       wSize);
				break;
			case USB_DT_CONFIG:	/* 2 */
				wSize = min(wLength,
					(u16) MGC_VirtualHubConfigDesc[2]);
				memcpy(pData, MGC_VirtualHubConfigDesc, wSize);
				break;
			case USB_DT_OTHER_SPEED_CONFIG:
				wSize = min(wLength,
					(u16) MGC_VirtualHubOtherConfigDesc[2]);
				memcpy(pData, MGC_VirtualHubOtherConfigDesc,
				       wSize);
				break;
			}
		}
		break;

	case USB_REQ_GET_CONFIGURATION:
		DBG(3, "GET_CONFIG() => 1\n");
		pData[0] = 1;
		wSize = 1;
		break;

	case USB_REQ_SET_CONFIGURATION:
		DBG(3, "SET_CONFIG(%04x)\n", wValue);
		wSize = 0;
		break;

	}			/* END: switch on request type */

	if (0xffff == wSize) {
		pUrb->status = -EPIPE;
	} else {
		pUrb->actual_length = wSize;
		pUrb->status = 0;
	}

	spin_unlock(&pHub->Lock);
	if (pUrb->complete) {
		DBG(3, "completing URB status=%d\n", pUrb->status);
		pUrb->complete(pUrb, NULL);
		pUrb->hcpriv = NULL;
		usb_put_urb(pUrb);
		DBG(4, "URB completed\n");
	}

	DBG(2, "==> pUrb->status=%d %s, length=%d, completed=%s\n",
	    pUrb->status, (pUrb->status) ? "(STALL)" : "", pUrb->actual_length,
	    (pUrb->complete) ? "yes" : "no");
	return 0;
}

/* Implementation */
int MGC_VirtualHubUnlinkUrb(MGC_VirtualHub * pHub, struct urb *pUrb)
{
	DBG(2, "<==\n");

	spin_lock(&pHub->Lock);
	if (pUrb && (pHub->pUrb == pUrb) && (pUrb->hcpriv == pHub)) {
		pHub->bIsChanged = FALSE;

		pUrb->status = -ECONNRESET;
		pUrb->complete(pUrb, NULL);

		pUrb->hcpriv = NULL;
		pHub->pUrb = NULL;
	}

	spin_unlock(&pHub->Lock);
	usb_put_urb(pUrb);

	DBG(2, "==>\n");
	return 0;
}

/*
 * assumes bPortIndex < MGC_VIRTUALHUB_MAX_PORTS
 * AND pHub->Lock to be... locked :)
 */
static void MGC_SetVirtualHubPortSpeed(MGC_VirtualHub * pHub,
				       u8 bPortIndex, u8 bSpeed)
{
	u16 wSpeedMask = 0;

	switch (bSpeed) {
	case 0:
		wSpeedMask = USB_PORT_STAT_LOW_SPEED;
		break;
	case 2:
		wSpeedMask = USB_PORT_STAT_HIGH_SPEED;
		break;
	}

	pHub->aPortStatusChange[bPortIndex].wStatus &=
	    ~(USB_PORT_STAT_LOW_SPEED | USB_PORT_STAT_HIGH_SPEED);
	pHub->aPortStatusChange[bPortIndex].wStatus |= 1 | wSpeedMask;
	pHub->bIsChanged = TRUE;
}

/* Implementation */
void MGC_VirtualHubPortResetDone(MGC_VirtualHub * pHub, u8 bPortIndex,
				 u8 bHubSpeed)
{
	spin_lock(&pHub->Lock);

	DBG(3, "port %d reset complete\n", bPortIndex);
	if (bPortIndex < MGC_VIRTUALHUB_MAX_PORTS) {
		MGC_SetVirtualHubPortSpeed(pHub, bPortIndex, bHubSpeed);

		pHub->aPortStatusChange[bPortIndex].wStatus &=
		    ~USB_PORT_STAT_RESET;
		pHub->aPortStatusChange[bPortIndex].wStatus |=
		    USB_PORT_STAT_ENABLE;
		pHub->aPortStatusChange[bPortIndex].wChange =
		    USB_PORT_STAT_RESET | USB_PORT_STAT_ENABLE;
		pHub->bIsChanged = TRUE;
	}

	spin_unlock(&pHub->Lock);
}

/*
 * Connect a port on the virtual hub.
 *
 * @param pHub the virtual hub
 * @param bPortIndex the port that has been disconnected
 * @param bSpeed the port speed
 */
void MGC_VirtualHubPortConnected(MGC_VirtualHub * pHub, u8 bPortIndex,
				 u8 bSpeed)
{
	struct urb *pUrb;

	printk("port connected\n");
	DBG(2, "<== port %d connected, core reports speed=%d\n", bPortIndex,
	    bSpeed);

	if (bPortIndex < MGC_VIRTUALHUB_MAX_PORTS) {
		spin_lock(&pHub->Lock);

		pUrb = pHub->pUrb;
		MGC_SetVirtualHubPortSpeed(pHub, bPortIndex, bSpeed);
		pHub->aPortStatusChange[bPortIndex].wChange |= 1;

		if (pUrb && ((!pUrb->hcpriv) || (pUrb->hcpriv == pHub))) {
			pUrb->hcpriv = pHub;
			/* shorter time... it want it NOW! */
			MGC_VirtualHubActivateTimer(pHub,
						    MGC_VirtualHubTimerExpired,
						    1);
		}

		spin_unlock(&pHub->Lock);
	}
}

/*
 * Disconnect a port on the virtual hub.
 *
 * @param pHub the virtual hub
 * @param bPortIndex the port that has been disconnected
 */
void MGC_VirtualHubPortDisconnected(MGC_VirtualHub * pHub, u8 bPortIndex)
{
	struct urb *pUrb;

	DBG(2, "<== Port %d disconnected\n", bPortIndex);
	if (bPortIndex >= MGC_VIRTUALHUB_MAX_PORTS) {
		return;
	}

	spin_lock(&pHub->Lock);
	del_timer_sync(&pHub->Timer);

	pUrb = pHub->pUrb;
	pHub->aPortStatusChange[bPortIndex].wStatus &= ~1;
	pHub->aPortStatusChange[bPortIndex].wChange |= 1;
	pHub->bIsChanged = TRUE;

	if (pUrb && (pUrb->hcpriv == pHub)) {
		MGC_VirtualHubCompleteIrq(pHub, pUrb, -ESHUTDOWN);
	}

	spin_unlock(&pHub->Lock);
}

/* Implementation */
void MGC_VirtualHubPortResumed(MGC_VirtualHub * pHub, u8 bPortIndex)
{
	DBG(3, "Resume port %d\n", bPortIndex);
	if (bPortIndex >= MGC_VIRTUALHUB_MAX_PORTS) {
		return;
	}

	spin_lock(&pHub->Lock);
	pHub->aPortStatusChange[bPortIndex].wStatus &= ~USB_PORT_STAT_SUSPEND;
	pHub->aPortStatusChange[bPortIndex].wChange |= USB_PORT_STAT_SUSPEND;
	pHub->bIsChanged = TRUE;
	spin_unlock(&pHub->Lock);
}
