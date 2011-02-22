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

/*
 * An OTG state machine
 */

#include <linux/config.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/smp_lock.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/time.h>
#include <linux/timer.h>

#include <linux/usb.h>

#include "musbdefs.h"
#include "otg.h"

#include "../core/hcd.h"

/******************************* FORWARDS ********************************/

static void MGC_OtgMachineActivateTimer(MGC_OtgMachine * pMachine,
					void (*pfExpired) (unsigned long),
					unsigned long timeout);
static void MGC_OtgMachineCancelTimer(MGC_OtgMachine * pMachine);
static void MGC_OtgMachineTimerExpired(unsigned long ptr);

/* static void* MGC_TestDeviceProbe(struct usb_device *dev, unsigned int i,
    const struct usb_device_id *id);
static void MGC_TestDeviceDisconnect(struct usb_device *dev, void *ptr);
*/

/******************************* GLOBALS *********************************/

/** SET_FEATURE(b_hnp_enable) request */
static u8 MGC_aSetHnpEnable[] = {
	USB_DIR_OUT | USB_TYPE_STANDARD | USB_RECIP_DEVICE,
	USB_REQ_SET_FEATURE,
	3, 0,			/* feature selector */
	0, 0,
	0, 0
};

/** SET_FEATURE(a_alt_hnp_support) request */
static u8 MGC_aSetAltHnpSupport[] = {
	USB_DIR_OUT | USB_TYPE_STANDARD | USB_RECIP_DEVICE,
	USB_REQ_SET_FEATURE,
	5, 0,			/* feature selector */
	0, 0,
	0, 0
};

static MGC_OtgMachine *MGC_pOtgMachine = NULL;

/****************************** FUNCTIONS ********************************/

/**
 * Generic timer activation helper
 * @param pMachine pointer to machine struct
 * @param pfExpired callback function
 * @param timeout millisecs
 */
static void MGC_OtgMachineActivateTimer(MGC_OtgMachine * pMachine,
					void (*pfExpired) (unsigned long),
					unsigned long timeout)
{
	DBG(2, "musbdrc: activating OTG timer for %ld msec\n", timeout);
	if (timeout) {
		init_timer(&(pMachine->Timer));

		pMachine->Timer.function = pfExpired;
		pMachine->Timer.data = (unsigned long)pMachine;
		pMachine->Timer.expires = jiffies + timeout * HZ / 1000;
		add_timer(&(pMachine->Timer));
	}
}

static void MGC_OtgMachineCancelTimer(MGC_OtgMachine * pMachine)
{
	if (pMachine->Timer.data) {
		/* stop timer */
		del_timer_sync(&pMachine->Timer);
		pMachine->Timer.data = (unsigned)NULL;
	}
}

/**
 * Timer expiration function to complete the interrupt URB on changes
 * @param ptr standard expiration param (hub pointer)
 */
static void MGC_OtgMachineTimerExpired(unsigned long ptr)
{
	MGC_OtgMachine *pMachine = (MGC_OtgMachine *) ptr;
	MGC_OtgServices *pServices = pMachine->pOtgServices;

	DBG(0, "<== pMachine->bState=%d\n", pMachine->bState);

	switch (pMachine->bState) {
	case MGC_OTG_STATE_B_SRP_INIT:
		INFO("musbdrc: SRP failed\n");
		pServices->pfOtgError(pServices->pPrivateData,
				      MGC_OTG_ERROR_B_SRP_FAILED);
		pServices->pfOtgSetSession(pServices->pPrivateData, FALSE);
		pMachine->bState = MGC_OTG_STATE_AB_IDLE;
		pServices->pfOtgState(pServices->pPrivateData,
				      pMachine->bState);
		break;

	case MGC_OTG_STATE_B_WAIT_ACON:
		if (pMachine->bRejecting) {
			pMachine->bRejecting = FALSE;
		} else {
			INFO("musbdrc: No response from A-device\n");
			pServices->pfOtgError(pServices->pPrivateData,
					      MGC_OTG_ERROR_NO_RESPONSE);
		}
		pMachine->bState = MGC_OTG_STATE_B_PERIPH;
		pServices->pfOtgSetHost(pServices->pPrivateData, FALSE);
		pServices->pfOtgSetSession(pServices->pPrivateData, TRUE);
		pServices->pfOtgState(pServices->pPrivateData,
				      pMachine->bState);
		break;

	case MGC_OTG_STATE_A_WAIT_BCON:
		INFO("musbdrc: No response from B-device\n");
		pServices->pfOtgError(pServices->pPrivateData,
				      MGC_OTG_ERROR_NO_RESPONSE);
		pServices->pfOtgSetSession(pServices->pPrivateData, FALSE);
		pMachine->bState = MGC_OTG_STATE_AB_IDLE;
		pServices->pfOtgState(pServices->pPrivateData,
				      pMachine->bState);
		break;

	case MGC_OTG_STATE_A_SUSPEND:
		INFO("musbdrc: No B-device HNP response\n");
		pServices->pfOtgError(pServices->pPrivateData,
				      MGC_OTG_ERROR_NO_RESPONSE);
		pServices->pfOtgSetSession(pServices->pPrivateData, FALSE);
		pMachine->bState = MGC_OTG_STATE_AB_IDLE;
		pServices->pfOtgState(pServices->pPrivateData,
				      pMachine->bState);
		break;
	}

	DBG(2, "==>");
}

/** Implementation */
u8 MGC_OtgMachineInit(MGC_OtgMachine * pMachine, MGC_OtgServices * pOtgServices)
{
	MGC_pOtgMachine = pMachine;

	memset(pMachine, 0, sizeof(MGC_OtgMachine));
	spin_lock_init(&pMachine->Lock);
	pMachine->pOtgServices = pOtgServices;

	pMachine->bState = MGC_OTG_STATE_AB_IDLE;
	pMachine->bRequest = MGC_OTG_REQUEST_UNKNOWN;
	pMachine->pUrb = usb_alloc_urb(0, GFP_ATOMIC);

	return pMachine->pUrb ? TRUE : FALSE;
}

#if 0
/** Implementation */
void MGC_OtgMachineDestroy(MGC_OtgMachine * pMachine)
{
	/* stop timer */
	MGC_OtgMachineCancelTimer(pMachine);
	if (pMachine->pUrb) {
		usb_free_urb(pMachine->pUrb);
	}

}
#endif

/** Implementation 
 * OTG STATES:
 MGC_OTG_STATE_AB_IDLE		0	IDLE 
 MGC_OTG_STATE_B_SRP_INIT	1	B-device initiating SRP with A-device 
 MGC_OTG_STATE_B_PERIPH		2	B-device is peripheral     
 MGC_OTG_STATE_B_WAIT_ACON	3 	B-device waiting for A-device to connect (as peripheral)
 MGC_OTG_STATE_B_HOST		4	B-device hosting A-device
 MGC_OTG_STATE_A_WAIT_BCON	5	A-device waiting for B-device to connect (as peripheral)    
 MGC_OTG_STATE_A_HOST		6	A-device hosting B-device
 MGC_OTG_STATE_A_SUSPEND,	7	A-device suspended bus due to software request
 MGC_OTG_STATE_A_PERIPH		8	A-device is peripheral

*/
void MGC_OtgMachineInputsChanged(MGC_OtgMachine * pMachine,
				 const MGC_OtgMachineInputs * pInputs)
{
	MGC_OtgServices *pServices = pMachine->pOtgServices;

	DBG(2,
	    "<== pMachine->bState=%d; OTG input: sess=%d, susp=%d, conn=%d, reset=%d, cid=%d, vberr=%d\n",
	    pMachine->bState, pInputs->bSession, pInputs->bSuspend,
	    pInputs->bConnection, pInputs->bReset, pInputs->bConnectorId,
	    pInputs->bVbusError);

	if (pInputs->bVbusError) {
		MGC_OtgMachineCancelTimer(pMachine);
		pMachine->bState = MGC_OTG_STATE_AB_IDLE;
		pServices->pfOtgState(pServices->pPrivateData,
				      pMachine->bState);
		return;
	}

	switch (pMachine->bState) {
	case MGC_OTG_STATE_AB_IDLE:
		if (pInputs->bSession && pInputs->bConnectorId) {
			if (pInputs->bReset) {
				pMachine->bState = MGC_OTG_STATE_B_PERIPH;
				pServices->pfOtgState(pServices->pPrivateData,
						      pMachine->bState);
			} else {
				pMachine->bState = MGC_OTG_STATE_B_SRP_INIT;
				pServices->pfOtgState(pServices->pPrivateData,
						      pMachine->bState);
				pServices->pfOtgRequestSession(pServices->
							       pPrivateData);
			}
		} else if (pInputs->bConnection) {
			/* 
			 * SKIP a state because connect IRQ comes so quickly after setting session,
			 * and only happens in host mode
			 */
			pMachine->bState = pInputs->bConnectorId ?
			    MGC_OTG_STATE_B_HOST : MGC_OTG_STATE_A_HOST;
			pServices->pfOtgState(pServices->pPrivateData,
					      pMachine->bState);
		} else if (pInputs->bSession) {
			pMachine->bState = MGC_OTG_STATE_A_WAIT_BCON;
			pServices->pfOtgState(pServices->pPrivateData,
					      pMachine->bState);
			MGC_OtgMachineActivateTimer(pMachine,
						    MGC_OtgMachineTimerExpired,
						    MGC_OTG_T_A_WAIT_BCON);
		}
		break;

	case MGC_OTG_STATE_B_SRP_INIT:
		if (pInputs->bReset) {
			MGC_OtgMachineCancelTimer(pMachine);
			pMachine->bState = MGC_OTG_STATE_B_PERIPH;
			pServices->pfOtgState(pServices->pPrivateData,
					      pMachine->bState);
		} else if (pInputs->bConnection) {
			MGC_OtgMachineCancelTimer(pMachine);
			pMachine->bState = pInputs->bConnectorId ?
			    MGC_OTG_STATE_B_HOST : MGC_OTG_STATE_A_HOST;
			pServices->pfOtgState(pServices->pPrivateData,
					      pMachine->bState);
		}
		break;

	case MGC_OTG_STATE_B_PERIPH:
		if (!pInputs->bSession) {
			pMachine->bState = MGC_OTG_STATE_AB_IDLE;
			pServices->pfOtgState(pServices->pPrivateData,
					      pMachine->bState);
		}

		if ((MGC_OTG_REQUEST_START_BUS == pMachine->bRequest) &&
		    pMachine->bHnpEnabled && pInputs->bSuspend) {
			pMachine->bState = MGC_OTG_STATE_B_WAIT_ACON;
			pServices->pfOtgState(pServices->pPrivateData,
					      pMachine->bState);
			//mdelay(10);
			//pServices->pfOtgSetSession(pServices->pPrivateData, FALSE);
			//pServices->pfOtgSetHost(pServices->pPrivateData, TRUE);
			MGC_OtgMachineActivateTimer(pMachine,
						    MGC_OtgMachineTimerExpired,
						    MGC_OTG_T_B_ASE0_BRST);
		}
		break;

	case MGC_OTG_STATE_B_WAIT_ACON:
		if (pInputs->bConnection) {
			MGC_OtgMachineCancelTimer(pMachine);
			pMachine->bState = MGC_OTG_STATE_B_HOST;
			pServices->pfOtgState(pServices->pPrivateData,
					      pMachine->bState);
		} else if (!pInputs->bSession) {
			MGC_OtgMachineCancelTimer(pMachine);
			pMachine->bState = MGC_OTG_STATE_AB_IDLE;
			pServices->pfOtgState(pServices->pPrivateData,
					      pMachine->bState);
		} else if (!pInputs->bSuspend) {
			MGC_OtgMachineCancelTimer(pMachine);
			pMachine->bState = MGC_OTG_STATE_B_PERIPH;
			pServices->pfOtgState(pServices->pPrivateData,
					      pMachine->bState);
		}
		break;

	case MGC_OTG_STATE_B_HOST:
		if (!pInputs->bConnection) {
			pMachine->bState = MGC_OTG_STATE_AB_IDLE;
			pServices->pfOtgState(pServices->pPrivateData,
					      pMachine->bState);
		} else if (pInputs->bConnection && !pInputs->bReset) {
			/* TODO: if we give up on Linux USB-core, get device desc 
			   and handle test device */
		}
		break;

	case MGC_OTG_STATE_A_WAIT_BCON:
		if (pInputs->bConnection) {
			MGC_OtgMachineCancelTimer(pMachine);
			pMachine->bState = MGC_OTG_STATE_A_HOST;
			pServices->pfOtgState(pServices->pPrivateData,
					      pMachine->bState);
		} else if (pInputs->bReset) {
			MGC_OtgMachineCancelTimer(pMachine);
			pMachine->bState = MGC_OTG_STATE_A_PERIPH;
			pServices->pfOtgState(pServices->pPrivateData,
					      pMachine->bState);
		}
		break;

	case MGC_OTG_STATE_A_HOST:
		if (!pInputs->bConnection) {
			pMachine->bState = MGC_OTG_STATE_A_WAIT_BCON;
			pServices->pfOtgState(pServices->pPrivateData,
					      pMachine->bState);
			MGC_OtgMachineActivateTimer(pMachine,
						    MGC_OtgMachineTimerExpired,
						    MGC_OTG_T_A_WAIT_BCON);
		} else if (pInputs->bConnection && !pInputs->bReset) {
			/* TODO: if we give up on Linux USB-core, get device desc 
			   and handle test device */
		}
		break;

	case MGC_OTG_STATE_A_SUSPEND:
		if (!pInputs->bSuspend) {
			MGC_OtgMachineCancelTimer(pMachine);
			pMachine->bState = MGC_OTG_STATE_A_HOST;
			pServices->pfOtgState(pServices->pPrivateData,
					      pMachine->bState);
		} else if (!pInputs->bConnection) {
			if (pMachine->bHnpEnabled) {
				pMachine->bState = MGC_OTG_STATE_A_PERIPH;
				pServices->pfOtgState(pServices->pPrivateData,
						      pMachine->bState);
			} else {
				pMachine->bState = MGC_OTG_STATE_A_WAIT_BCON;
				pServices->pfOtgState(pServices->pPrivateData,
						      pMachine->bState);
				MGC_OtgMachineActivateTimer(pMachine,
							    MGC_OtgMachineTimerExpired,
							    MGC_OTG_T_A_WAIT_BCON);
			}
		}
		break;

	case MGC_OTG_STATE_A_PERIPH:
		if (!pInputs->bSession) {
			pMachine->bState = MGC_OTG_STATE_AB_IDLE;
			pServices->pfOtgState(pServices->pPrivateData,
					      pMachine->bState);
		} else if (pInputs->bSuspend) {
			pMachine->bState = MGC_OTG_STATE_A_WAIT_BCON;
			pServices->pfOtgState(pServices->pPrivateData,
					      pMachine->bState);
			MGC_OtgMachineActivateTimer(pMachine,
						    MGC_OtgMachineTimerExpired,
						    MGC_OTG_T_A_WAIT_BCON);
		}
		break;
	}

}

static void MGC_OtgMachineUrbComplete(struct urb *pUrb, struct pt_regs *regs)
{
	MGC_OtgMachine *pMachine = (MGC_OtgMachine *) pUrb->context;

	if (!pUrb->status) {
		pMachine->bSetHnpEnable = TRUE;
		if (pMachine->bRejecting) {
			MGC_OtgMachineRequest(pMachine,
					      MGC_OTG_REQUEST_DROP_BUS);
		}
	}
}

/**
 * Enumerated device on the USB bus and decide whether the device shall
 * be accepted.
 * @param pMachine 
 * @param pDevice
 * @param bHasOtgDescriptor
 * @param bSupportsHubs 
 * @param bPortSupportsHnp
 * @param bOtherPortSupportsHnp
 */
u8 MGC_OtgMachineRootDeviceEnumerated(MGC_OtgMachine * pMachine,
				      struct usb_device *pDevice,
				      u8 bHasOtgDescriptor, u8 bSupportsHubs,
				      u8 bPortSupportsHnp,
				      u8 bOtherPortSupportsHnp)
{
	int found;
	struct urb *pUrb = pMachine->pUrb;
	MGC_OtgServices *pServices = pMachine->pOtgServices;

	/* TPL */
	pMachine->bRejecting = FALSE;

	DBG(3,
	    "Checking TPL; VID=%04x/PID=%04x, otg=%d, hubs=%d, hnp=%d, alt=%d\n",
	    pDevice->descriptor.idVendor, pDevice->descriptor.idProduct,
	    bHasOtgDescriptor, bSupportsHubs, bPortSupportsHnp,
	    bOtherPortSupportsHnp);

	if (!bSupportsHubs
	    && (USB_CLASS_HUB == pDevice->descriptor.bDeviceClass)) {
		printk(KERN_INFO "OTG: Hubs not supported\n");
		pServices->pfOtgError(pServices->pPrivateData,
				      MGC_OTG_ERROR_UNSUPPORTED_HUB);
		pMachine->bRejecting = TRUE;
	}

	found = (pMachine->pCheckDevice)	/* no TPL? accept */
	    ? (*pMachine->pCheckDevice) (pDevice) : TRUE;
	if (!found) {
		printk(KERN_INFO "OTG: Device not supported\n");
		pServices->pfOtgError(pServices->pPrivateData,
				      MGC_OTG_ERROR_UNSUPPORTED_DEVICE);
		pMachine->bRejecting = TRUE;
	}

	/* OTG SET_FEATURE */
	if (bHasOtgDescriptor) {
		pUrb->setup_packet = NULL;
		pUrb->complete = NULL;
		pUrb->dev = pDevice;
		pUrb->pipe = usb_sndctrlpipe(pDevice, 0);
		pUrb->transfer_buffer = NULL;
		pUrb->transfer_buffer_length = 0;
		if (bPortSupportsHnp) {
			DBG(2, "setting b_hnp_enable on %d\n",
			    pUrb->dev->devnum);
			pUrb->setup_packet = MGC_aSetHnpEnable;
			pUrb->complete = MGC_OtgMachineUrbComplete;
			pUrb->context = pMachine;
		} else if (bOtherPortSupportsHnp) {
			DBG(2, "setting a_alt_hnp_support on %d\n",
			    pUrb->dev->devnum);

			pUrb->setup_packet = MGC_aSetAltHnpSupport;
			pUrb->complete = MGC_OtgMachineUrbComplete;
			pUrb->context = pMachine;
		}

		/* submit */
		if (pMachine->bRejecting || bPortSupportsHnp
		    || bOtherPortSupportsHnp) {
			pMachine->pOtgServices->pfOtgSubmitUrb(pMachine->
							       pOtgServices->
							       pPrivateData,
							       pUrb);
		}
	} else if (pMachine->bRejecting) {
		MGC_OtgMachineRequest(pMachine, MGC_OTG_REQUEST_DROP_BUS);
	}

	return pMachine->bRejecting ? FALSE : TRUE;

}

/** Implementation */
void MGC_OtgMachineSetFeature(MGC_OtgMachine * pMachine, u16 wFeatureSelector)
{
	MGC_OtgServices *pServices = pMachine->pOtgServices;

	switch (wFeatureSelector) {
	case 3:
		pMachine->bHnpEnabled = TRUE;
		if (MGC_OTG_REQUEST_START_BUS == pMachine->bRequest) {
			pServices->pfOtgSetHost(pServices->pPrivateData, TRUE);
		}
		break;
	case 4:
		pMachine->bHnpSupported = TRUE;
		break;
	}

}

/** Implementation */
void MGC_OtgMachineRequest(MGC_OtgMachine * pMachine, u8 bRequest)
{
	MGC_OtgServices *pServices = pMachine->pOtgServices;

	pMachine->bRequest = bRequest;
	switch (bRequest) {
	case MGC_OTG_REQUEST_START_BUS:
		switch (pMachine->bState) {
		case MGC_OTG_STATE_AB_IDLE:
		case MGC_OTG_STATE_B_SRP_INIT:
			pServices->pfOtgSetSession(pServices->pPrivateData,
						   TRUE);
			break;
		case MGC_OTG_STATE_A_SUSPEND:
			pMachine->bState = MGC_OTG_STATE_A_HOST;
			pServices->pfOtgState(pServices->pPrivateData,
					      pMachine->bState);
			DBG(2, "resuming bus during A-SUSPEND...\n");
			pServices->pfOtgSetSuspend(pServices->pPrivateData,
						   FALSE);
			break;
		default:
			pServices->pfOtgSetSession(pServices->pPrivateData,
						   TRUE);
		}
		break;

	case MGC_OTG_REQUEST_DROP_BUS:
		switch (pMachine->bState) {
		case MGC_OTG_STATE_B_PERIPH:
		case MGC_OTG_STATE_B_HOST:
			pMachine->bState = MGC_OTG_STATE_B_PERIPH;
			pServices->pfOtgState(pServices->pPrivateData,
					      pMachine->bState);
			DBG(2, "suspending bus...\n");
			pServices->pfOtgSetHost(pServices->pPrivateData, FALSE);
			pServices->pfOtgSetSuspend(pServices->pPrivateData,
						   TRUE);
			break;
		case MGC_OTG_STATE_A_HOST:
		case MGC_OTG_STATE_A_WAIT_BCON:
		case MGC_OTG_STATE_A_SUSPEND:
		case MGC_OTG_STATE_A_PERIPH:
			pMachine->bState = MGC_OTG_STATE_AB_IDLE;
			pServices->pfOtgState(pServices->pPrivateData,
					      pMachine->bState);
			DBG(2, "suspending bus...\n");
			pServices->pfOtgSetHost(pServices->pPrivateData, FALSE);
			pServices->pfOtgSetSuspend(pServices->pPrivateData,
						   TRUE);
			break;
		default:
			ERR("DROP_BUS in illegal state %d\n", pMachine->bState);
		}
		break;
	case MGC_OTG_REQUEST_SUSPEND_BUS:
		switch (pMachine->bState) {
		case MGC_OTG_STATE_A_HOST:
			pMachine->bState = MGC_OTG_STATE_AB_IDLE;
			pServices->pfOtgState(pServices->pPrivateData,
					      pMachine->bState);
			DBG(2, "suspending bus...\n");
			pServices->pfOtgSetSuspend(pServices->pPrivateData,
						   TRUE);
			MGC_OtgMachineActivateTimer(pMachine,
						    MGC_OtgMachineTimerExpired,
						    MGC_OTG_T_AIDL_BDIS);
			break;
		default:
			ERR("SUSPEND_BUS in illegal state %d\n",
			    pMachine->bState);
		}
		break;
	case MGC_OTG_REQUEST_RESET:
		pMachine->bState = MGC_OTG_STATE_AB_IDLE;
		pServices->pfOtgState(pServices->pPrivateData,
				      pMachine->bState);
		break;
	}
}
