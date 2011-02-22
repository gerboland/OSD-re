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

#include <linux/usb.h>
#include "musbdefs.h"

/* Zap the driver (warm start) ... removed, use "rmmod/modprobe" */

/**
 * Start a session. Depeing on the controller mode (cable end) it will
 * pwer VBUS/initiate SRP and/or it will behave like a gadget.
 *
 */
void MGC_Session(MGC_LinuxCd * pThis)
{
	u8 bReg, sesn = 0;
#ifdef CONFIG_USB_MUSB_OTG
	MGC_OtgMachineInputs Inputs;
#endif

#ifdef MUSB_PARANOID
	if (!pThis) {
		ERR("Controller not initialized\n");
		return;
	}
#endif

	if (MUSB_IS_ERR(pThis)) {
		WARN("Error mode, zap the driver first\n");
	}
#ifdef CONFIG_USB_MUSB_OTG
	sesn = 1;
	if (MUSB_IS_OTG(pThis)) {
		sesn = 0;
	}
#endif

	if (sesn) {
		ERR("A %s session is active; terminate it first\n",
		    MUSB_MODE(pThis));
		return;
	}
#ifdef CONFIG_USB_MUSB_OTG
	/* connect interrupt can happen VERY soon after setting SESSION */
	memset(&Inputs, 0, sizeof(Inputs));
	Inputs.bSession = TRUE;

	/* OTG machine must be prepared for connect interrupt */
	Inputs.bConnectorId =
	    (MGC_Read8(pThis->pRegs, MGC_O_HDRC_DEVCTL) & MGC_M_DEVCTL_BDEVICE)
	    ? TRUE : FALSE;
	MGC_OtgMachineInputsChanged(&pThis->OtgMachine, &Inputs);
	MGC_OtgMachineRequest(&pThis->OtgMachine, MGC_OTG_REQUEST_START_BUS);
#else
	WARN("Start a session\n");
	MGC_OtgUpdate(pThis, FALSE, FALSE);
#endif

	/* WHY!?!?! this looks like a race condition to me */
	bReg = MGC_Read8(pThis->pRegs, MGC_O_HDRC_DEVCTL);
	MGC_Write8(pThis->pRegs, MGC_O_HDRC_DEVCTL,
		   bReg | MGC_M_DEVCTL_SESSION);
}
