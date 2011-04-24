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
 * Inventra Controller Driver (ICD) for Linux.
 *
 * This consists of a Host Controller Driver (HCD) and optionally,
 * a Gadget provider.
 */

/*
 * (Original comments by Mentor; responses "==" from DB.)
 *
 * Introduction.
 * The ICD works like the other Linux HCDs/Gadgets: it is threadless,
 * so it does everything either in response to an interrupt,
 * or during a call from an upper layer.
 * It implements a virtual root hub, so as to make uniform use
 * of the Linux hub driver.Linux
 *
 * 	==	It doesn't yet use much of the linux 2.6 usbcore
 * 		framework though, including most of that root hub
 * 		and important fixes for device disconnection and
 * 		urb unlinking.
 *
 * The Linux (host-side) USB core has no concept of binding (the authors
 * apparently missed the point of the pipe discussion in the USB spec).
 *
 * 	==	That issue is late binding vs early, which generally
 * 		doesn't matter.  The USB spec doesn't require early
 * 		binding.  Linux uses late binding even for periodic
 * 		transfers (iso, interrupt), and reserves bandwidth as
 * 		long as the completion callback leaves an urb queued.
 *
 * Instead, class drivers simply submit URBs, and an HCD may reject
 * or defer them if sufficient resources are not available.
 * This means class drivers have no way to know if their requirements
 * can possibly be fulfilled, and may be blocked indefinitely by others,
 * without the end-user knowing why.
 *
 * 	==	Class drivers report the errors up the driver stack
 * 		just like any other kind of error.  If the userspace
 * 		code doesn't know enough to tell those end users to
 * 		shut down some other USB applications, that's just bad
 * 		userspace code.  It could break with early binding too.
 *
 * 		(Note that this only matters with periodic transfers,
 * 		and bulk transfers -- as with usb storage media -- have
 * 		no problem sharing bandwith in ways that seem "fair".)
 *
 * Therefore, whether things will work depends on the order of URB submissions
 * (which is dictated by the order of device insertion and/or driver loading
 * and thread scheduling).
 *
 * The URB encodes pipe information in an integer,
 * requiring table searches (hurting performance).
 *
 * 	==	Yes, eventually that should go away.  More current kernels
 * 		(starting maybe at 2.6.11) allow host_endpoint to almost
 * 		completely replace the ancient Linux 2.2 "pipe" wierdness.
 *
 * For the HDRC, local endpoint 0 is the only choice for control traffic,
 * so it is reprogrammed as needed, and locked during transfers.
 * Bulk transfers are queued to the available local endpoint with
 * the smallest possible FIFO in the given direction
 * that will accomodate the transactions.
 *
 * A typical response to the completion of a periodic URB is immediate
 * submission of another one, so the HCD does not assume it can reprogram
 * a local periodic-targetted endpoint for another purpose.
 * Instead, submission of a periodic URB is taken as a permanent situation,
 * so that endpoint is untouched.
 *
 * 	===	Linux nuance:  its slot is reserved in the periodic schedule
 * 		until its driver stops submitting urbs.
 *
 * One could imagine reprogramming periodic endpoints for other uses
 * between their polling intervals, effectively interleaving traffic on them.
 * Unfortunately, this assumes no device would ever NAK periodic tokens.
 * This is because the core no notification to software when an attempted
 * periodic transaction is NAKed (its NAKlimit feature is only for
 * control/bulk).
 */

/*
 * This gets many kinds of configuration information:
 * 	- Kconfig for everything user-configurable
 * 	- <asm/arch/hdrc_cnf.h> for SOC or family details
 * 	- platform_device for addressing, irq, and platform_data
 * 	- platform_data is mostly for board-specific informarion
 *
 * Most of the conditional compilation will vanish.
 */

#include <linux/config.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/usb.h>

#include <asm/io.h>
#include <asm/arch/memory.h>

#if MUSB_DEBUG > 0
#define dbgPrint(args...)   printk(args)
#define MUSB_VERSION_SUFFIX	 "(" __DATE__ ")"
#else
#define dbgPrint(args...)
#endif

// #ifdef CONFIG_USB_MUSB_HDRC_HCD
#include <linux/usb.h>
#include "../core/hcd.h"
// #endif

#include "musbdefs.h"
// #ifdef CONFIG_USB_MUSB_HDRC_HCD
#include "musb_host.h"
extern struct usb_operations MGC_LinuxOperations;
// #endif
#include "musb_hdrdf.h"

#ifdef CONFIG_USB_TI_CPPI_DMA
#include "cppi_dma.h"
#endif


/***************************** CONSTANTS ********************************/

#define DRIVER_AUTHOR "Mentor Graphics Corp. and Texas Instruments"
#define DRIVER_DESC "Inventra Dual-Role USB Controller Driver"

#define MUSB_VERSION_BASE "2.2a/db-0.2.8"

#ifndef MUSB_VERSION_SUFFIX
#define MUSB_VERSION_SUFFIX	 ""
#endif
#define MUSB_VERSION	MUSB_VERSION_BASE MUSB_VERSION_SUFFIX

#define DRIVER_INFO DRIVER_DESC "v" MUSB_VERSION

static const char longname[] = DRIVER_INFO;
const char musb_driver_name[] = "musb_hdrc";

/* this module is always GPL, the gadget might not... */
MODULE_DESCRIPTION(DRIVER_INFO);
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_LICENSE("GPL");

/* time (millseconds) to wait before a restart */
#define MUSB_RESTART_TIME        5000

/* how many babbles to allow before giving up */
#define MUSB_MAX_BABBLE_COUNT    10

/* WEIRD KLUDGE! */
#define IS_INVALID_ADDRESS(_x) (((unsigned long)_x)<(unsigned long)1024)


/****************************** FORWARDS ********************************/

extern void MGC_HdrcEnableDmaReq(MGC_LinuxCd * pThis, u8 bEnd);

/* Linux USBD glue */

/* driver functions */
static irqreturn_t MGC_LinuxIsr(int irq, void *__hci, struct pt_regs *r);

/* HDRC functions */
static void MGC_HdrcRestart(unsigned long pParam);

static void MGC_HdrcDisable(MGC_LinuxCd * pThis);
static u8 MGC_HdrcInit(u16 wType, MGC_LinuxCd * pThis);

extern void funcCppiCompletionIsr(void);

/* OTG glue, used by the OTG services */
#ifdef CONFIG_USB_MUSB_OTG
static void MGC_LinuxOtgError(void *pPrivateData, u8 bError);
static void MGC_LinuxOtgState(void *pPrivateData, u8 bState);
static void MGC_LinuxOtgSetSession(void *pPrivateData, u8 bSession);
static void MGC_LinuxOtgSetHost(void *pPrivateData, u8 bHost);
static void MGC_LinuxOtgRequestSession(void *pPrivateData);
static void MGC_LinuxOtgSetSuspend(void *pPrivateData, u8 bSuspend);
static int MGC_LinuxOtgSubmitUrb(void *pPrivateData, struct urb *pUrb);
#endif

static void MGC_LinuxOtgInputChanged(MGC_LinuxCd * pThis, u8 devctl,
				     u8 reset, u8 connection, u8 suspend);

/****************************** GLOBALS *********************************/

unsigned int MGC_nIndex = 0;

#ifdef CONFIG_USB_INVENTRA_HCD_POLLING
static int MGC_nPollerPid = 0;	/* PID of poller */
static DECLARE_COMPLETION(MGC_PollerExited);
#endif

/* for high speed test mode */
const u8 MGC_aTestPacket[MGC_TEST_PACKET_SIZE] = {
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa, 0xaa,
	0xaa, 0xee, 0xee, 0xee, 0xee, 0xee, 0xee, 0xee,
	0xee, 0xfe, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
	0xff, 0xff, 0xff, 0xff, 0xff, 0x7f, 0xbf, 0xdf,
	0xef, 0xf7, 0xfb, 0xfd, 0xfc, 0x7e, 0xbf, 0xdf,
	0xef, 0xf7, 0xfb, 0xfd, 0x7e
};

/*************************************************************************
 * HDRC functions
**************************************************************************/

#ifdef CONFIG_USB_MUSB_HDRC_HCD
/*
 * Timer completion callback to finish resume handling started in ISR
 */
static void MGC_HdrcDropResume(unsigned long pParam)
{
	u8 power;
	MGC_LinuxCd *pThis = (MGC_LinuxCd *) pParam;
	void *pBase = pThis->pRegs;

	DBG(2, "<==\n");

	power = MGC_Read8(pBase, MGC_O_HDRC_POWER);
	MGC_Write8(pBase, MGC_O_HDRC_POWER, power & ~MGC_M_POWER_RESUME);

	MGC_VirtualHubPortResumed(&pThis->RootHub, 0);

	MGC_OtgUpdate(pThis, FALSE, FALSE);
}
#endif

/*
 * Timer completion callback to request session again
 * (to avoid self-connecting)
 */
static void MGC_HdrcRestart(unsigned long pParam)
{
	MGC_HdrcStart((MGC_LinuxCd *) pParam);
}

/*
 * Load an HDRC FIFO
 *
 * @param pBase base address of HDRC
 * @param bEnd local endpoint
 * @param wCount how many bytes to load
 * @param pSource data buffer
 */
void MGC_HdrcLoadFifo(const u8 *pBase, u8 bEnd, u16 wCount, const u8 *pSource)
{
	void __iomem *fifo = MGC_FIFO_OFFSET(bEnd) + (void __iomem *)pBase;

	DBG(3, "%cX ep%d fifo %p count %d buf %p\n",
			'T', bEnd, fifo, wCount, pSource);

	/* we can't assume unaligned reads work */
	if (likely((0x01 & (int) pSource) == 0)) {
		u16	index = 0;

		/* best case is 32bit-aligned source address */
		if ((0x02 & (int) pSource) == 0) {
			if (wCount >= 4) {
				writesl(fifo, pSource + index, wCount >> 2);
				index += wCount & ~0x03;
			}
			if (wCount & 0x02) {
				MGC_Write16(fifo, 0, *(u16*)&pSource[index]);
				index += 2;
			}
		} else {
			if (wCount >= 2) {
				writesw(fifo, pSource + index, wCount >> 1);
				index += wCount & ~0x01;
			}
		}
		if (wCount & 0x01)
			MGC_Write8(fifo, 0, pSource[index]);
	} else  {
		/* byte aligned */
		writesb(fifo, pSource, wCount);
	}
}

/*
 * Unload an HDRC FIFO
 *
 * @param pBase base address of HDRC
 * @param bEnd local endpoint
 * @param wCount how many bytes to unload
 * @param pDest data buffer
 */
void MGC_HdrcUnloadFifo(const u8 *pBase, u8 bEnd, u16 wCount, u8 *pDest)
{
	void __iomem *fifo = MGC_FIFO_OFFSET(bEnd) + (void __iomem *)pBase;

	DBG(3, "%cX ep%d fifo %p count %d buf %p\n",
			'R', bEnd, fifo, wCount, pDest);

	/* we can't assume unaligned writes work */
	if (likely((0x01 & (int) pDest) == 0)) {
		u16	index = 0;

		/* best case is 32bit-aligned destination address */
		if ((0x02 & (int) pDest) == 0) {
			if (wCount >= 4) {
				readsl(fifo, pDest, wCount >> 2);
				index = wCount & ~0x03;
			}
			if (wCount & 0x02) {
				*(u16*)&pDest[index] = MGC_Read16(fifo, 0);
				index += 2;
			}
		} else {
			if (wCount >= 2) {
				readsw(fifo, pDest, wCount >> 1);
				index = wCount & ~0x01;
			}
		}
		if (wCount & 0x01)
			MGC_Write8(fifo, 0, pDest[index]);
			pDest[index] = MGC_Read8(fifo, 0);
	} else  {
		/* byte aligned */
		readsb(fifo, pDest, wCount);
	}
}

static void stop_host(MGC_LinuxCd * pThis, int vberr)
{
	ERR("Stopping due to Vbus error\n");

	MGC_HdrcStop(pThis);

#ifdef CONFIG_USB_MUSB_HDRC_HCD
	MGC_VirtualHubPortDisconnected(&pThis->RootHub, 0);
	pThis->pRootDevice = NULL;
#endif

	MGC_OtgUpdate(pThis, vberr, FALSE);
}

/*
 * Interrupt Service Routine to record USB "global" interrupts.
 * Since these do not happen often and signify things of
 * paramount importance, it seems OK to check them individually;
 * there is an ORDER to perform the tests check p35 of the MUSBHDRC
 * manual.
 *
 * @param pThis instance pointer
 * @param bIntrUSB register contents
 * @param devctl
 * @param power
 */
static int MGC_HdrcServiceUsbStage0(MGC_LinuxCd * pThis, u8 bIntrUSB,
				    u8 devctl, u8 power)
{
	int handled = 0;
#ifdef CONFIG_USB_MUSB_OTG
	MGC_OtgMachineInputs Inputs;
#endif

#ifdef CONFIG_USB_MUSB_HDRC_HCD
	u8 bSpeed = 1;
	u8 bHubSpeed = 2;
#endif
	u8 *pBase = (u8 *) pThis->pRegs;

	DBG(2, "<== Power=%02x, DevCtl=%02x, bIntrUSB=0x%x\n", power, devctl,
	    bIntrUSB);

	/* in host mode when a device resume me (from power save)
	 * in device mode when the host resume me; it shold not change
	 * "identity".
	 */
	if (bIntrUSB & MGC_M_INTR_RESUME) {
		handled++;
		DBG(3, "RESUME\n");

		if (devctl & MGC_M_DEVCTL_HM) {
#ifdef CONFIG_USB_MUSB_HDRC_HCD
			/* REVISIT:  this is where SRP kicks in, yes? */
			MUSB_HST_MODE(pThis);	/* unnecessary */
			power &= ~MGC_M_POWER_SUSPENDM;
			MGC_Write8(pBase, MGC_O_HDRC_POWER,
				   power | MGC_M_POWER_RESUME);
			MGC_LinuxSetTimer(pThis, MGC_HdrcDropResume,
					  (unsigned long)pThis, 40);
#endif
		} else {
#ifdef CONFIG_USB_GADGET_MUSB_HDRC
			MUSB_DEV_MODE(pThis);	/* unnecessary */
			MGC_GadgetResume(pThis);
#endif
		}
	}

	/* p35 MUSBHDRC manual for the order of the tests */
	if (bIntrUSB & MGC_M_INTR_SESSREQ) {
		DBG(1, "SESSION_REQUEST (FUNCTION MODE)\n");

		/* NOTE i might get a sesison request WHILE switchign between B and A
		 * device investigatge about that; check p35 of the manual
		 } */
#ifdef MUSB_PARANOID
		if (!(devctl & MGC_M_DEVCTL_HM)) {
			ERR("Received a SESSION_REQUEST when connected to the B end!\n");
			stop_host(pThis, FALSE);
			return handled;
		}
#endif

		/* time critical code (turn on VBUS); inherent race condition */
		MGC_Write8(pBase, MGC_O_HDRC_DEVCTL, MGC_M_DEVCTL_SESSION);
		pThis->bEnd0Stage = MGC_END0_START;
		MUSB_HST_MODE(pThis);

		handled++;

#ifdef CONFIG_USB_MUSB_OTG
		memset(&Inputs, 0, sizeof(Inputs));
		Inputs.bSession = TRUE;
		Inputs.bConnectorId = FALSE;
		Inputs.bReset = FALSE;
		Inputs.bConnection = FALSE;
		Inputs.bSuspend = FALSE;
		MGC_OtgMachineInputsChanged(&(pThis->OtgMachine), &Inputs);
#endif
	}
	/* VBUSError is bad, shutdown &  go to error mode and ignore
	 * the other interrups; p35 MUSBHDRC manual for the order
	 of the tests */
	if (bIntrUSB & MGC_M_INTR_VBUSERROR) {
		handled++;

#ifdef MUSB_PARANOID
		if (!(devctl & MGC_M_DEVCTL_HM)) {
			ERR("Received a MGC_M_INTR_VBUSERROR when connected to the B end!\n");
			stop_host(pThis, FALSE);
			return handled;
		}
#endif
		ERR("V_BUS ERROR??? stopping host\n");
		printk("VBUS error interrupt \n");
		stop_host(pThis, TRUE);
		MUSB_ERR_MODE(pThis, MUSB_ERR_VBUS);
	}

	/* connect is valid only when in host mode; ignore it if in device mode;
	   p35 MUSBHDRC manual for the order of the tests */
	if (bIntrUSB & MGC_M_INTR_CONNECT) {
		handled++;
		DBG(1, "RECEIVED A CONNECT (goto host mode)\n");

#ifdef MUSB_PARANOID
		if (!(devctl & MGC_M_DEVCTL_HM)) {
			ERR("Received a CONNECT when connected to the B end!\n");
			stop_host(pThis, FALSE);
			return handled;	/* VERY ODD!!! */
		}
#endif

#ifdef CONFIG_USB_MUSB_HDRC_HCD
		pThis->pRootDevice = NULL;
		pThis->bEnd0Stage = MGC_END0_START;

		/* reset the addres... probably not needed */
		MGC_Write8(pThis->pRegs, MGC_O_HDRC_FADDR, 0);

		/* flush endpoints when transitioning from Device Mode */
		if (MUSB_IS_DEV(pThis)) {
#if 0
			u8 bEnd;

			// REVISIT these calls don't shut down the peripheral
			// side; and will be needed only given OTG ...
			for (bEnd = 0; bEnd < pThis->bEndCount; bEnd++) {
				MGC_HdrcStopEnd(pThis, bEnd);
			}
#endif
		}
#ifdef CONFIG_USB_MUSB_OTG
		pThis->bDelayPortPowerOff = FALSE;
#endif

		if (devctl & MGC_M_DEVCTL_LSDEV) {
			bSpeed = 3;
			bHubSpeed = 0;
		} else if (devctl & MGC_M_DEVCTL_FSDEV) {
			/* NOTE: full-speed is "speculative" until reset */
			bSpeed = 2;
			bHubSpeed = 1;
		}

		pThis->bRootSpeed = bSpeed;
		if (pThis->bIsMultipoint) {
			/* set speed for EP0 */
			MGC_SelectEnd(pBase, 0);
			MGC_WriteCsr8(pBase, MGC_O_HDRC_TYPE0, 0,
				      (bSpeed << 6));
		}

		MUSB_HST_MODE(pThis);

		/* indicate new connection to OTG machine */
		MGC_LinuxOtgInputChanged(pThis, devctl, FALSE, TRUE, FALSE);
		MGC_VirtualHubPortConnected(&pThis->RootHub, 0, bHubSpeed);

#else
		ERR("GADGET mode: Don't know what to do with a connect");
		ERR("GADGET mode: OTG support was not compiled");
#endif
	}

	/* saved one bit: bus reset and babble share the same bit;
	 * If I am host is a babble! i must be the only one allowed
	 * to reset the bus; when in otg mode it means that I have
	 * to switch to device
	 */
	if (bIntrUSB & MGC_M_INTR_RESET) {
		DBG(1, "BUS RESET\n");

		if (devctl & MGC_M_DEVCTL_HM) {
			ERR("calling stop host in response to RESET\n");
			stop_host(pThis, FALSE);

			/* restart session after cooldown unless threshold reached */
			if (pThis->nBabbleCount++ < MUSB_MAX_BABBLE_COUNT) {
				MGC_LinuxSetTimer(pThis, MGC_HdrcRestart,
						  (unsigned long)pThis,
						  MUSB_RESTART_TIME);
			}
		} else {
#ifdef CONFIG_USB_GADGET_MUSB_HDRC
			pThis->bEnd0Stage = MGC_END0_START;
			MUSB_DEV_MODE(pThis);
			MGC_GadgetReset(pThis);
#endif

			/* reading reset state from Power register does NOT work */
			MGC_LinuxOtgInputChanged(pThis, devctl, TRUE, FALSE,
						 (power & MGC_M_POWER_SUSPENDM)
						 ? TRUE : FALSE);
		}

		handled++;
	}

	return handled;
}

/*
 * Interrupt Service Routine to record USB "global" interrupts.
 * Since these do not happen often and signify things of
 * paramount importance, it seems OK to check them individually;
 * there is an ORDER to perform the tests check p35 of the MUSBHDRC
 * manual.
 *
 * @param pThis instance pointer
 * @param bIntrUSB register contents
 * @param devctl
 * @param power
 */
static int MGC_HdrcServiceUsbStage1(MGC_LinuxCd * pThis, u8 bIntrUSB,
				    u8 devctl, u8 power)
{
	int handled = 0;

#ifdef CONFIG_USB_MUSB_HDRC_HCD
	u8 bEnd;
	u16 wFrame;
	u8 *pBase = (u8 *) pThis->pRegs;

	/* p35 MUSBHDRC manual for the order of the tests */
	if (bIntrUSB & MGC_M_INTR_SOF) {
		DBG(6, "START_OF_FRAME\n");
		handled++;

		/* start any periodic Tx transfers waiting for current frame */
		wFrame = MGC_Read16(pBase, MGC_O_HDRC_FRAME);
		for (bEnd = 1;
		     (bEnd < pThis->bEndCount)
		     && (pThis->wEndMask >= (1 << bEnd)); bEnd++) {
			if (pThis->aLocalEnd[bEnd].dwWaitFrame >= wFrame) {
				pThis->aLocalEnd[bEnd].dwWaitFrame = 0;
				if (!pThis->aLocalEnd[bEnd].bEnableDmaReq) {
					printk("starting periodic Tx on %d\n",
					       bEnd);
					MGC_HdrcStartTx(pThis, bEnd);
				} else {
					printk
					    ("Enabling DMA req for periodic on %d\n",
					     bEnd);
					MGC_HdrcEnableDmaReq(pThis, bEnd);
				}
			}
		}		/* end of for loop */
	}
#endif

	/* p35 MUSBHDRC manual for the order of the tests */
	if ((bIntrUSB & MGC_M_INTR_DISCONNECT) && !pThis->bIgnoreDisconnect) {
		DBG(1, "DISCONNECT\n");
		handled++;

		/* need to check it against pThis, because the devctl is going
		 * low as soon as the device gets disconnected */
		if (MUSB_IS_HST(pThis)) {
			DBG(3, "Disconnecting a port of VirtualHub\n");

			/* FIXME khubd handles all this if we report
			 * things correctly, properly shutting down and
			 * disconnecting it all ...
			 */
			printk(KERN_WARNING "%s: DISCONNECT MISHANDLED! "
					"instability may follow...\n",
					musb_driver_name);

#ifdef CONFIG_USB_MUSB_HDRC_HCD
			MGC_VirtualHubPortDisconnected(&pThis->RootHub, 0);
			pThis->pRootDevice = NULL;

			/* flush endpoints */
			for (bEnd = 0; bEnd < pThis->bEndCount; bEnd++) {
				MGC_HdrcStopEnd(pThis, bEnd);
			}
#endif

#ifdef MUSB_CONFIG_PROC_FS
			if (pThis->pfDisconnectListener) {
				pThis->pfDisconnectListener(pThis->
							    pDisconnectListenerParam);
			}
#endif
		} else {
#ifdef CONFIG_USB_GADGET_MUSB_HDRC
			MGC_GadgetDisconnect(pThis);
#endif
		}

		/* KLUDGE: race condition, doing this right away might prevent
		 * the virtual hub/usbcore to process the last urbs. As a matter
		 * of facts this code should be called after the "disconnect" */
		// mdelay(500);
		MGC_OtgUpdate(pThis, FALSE, FALSE);
	}

	/* I cannot get suspend while in host mode! go to error mode and ignore
	 * the other signals; need to be last (see manual p35)s  */
	if (bIntrUSB & MGC_M_INTR_SUSPEND) {
		DBG(1, "RECEIVED SUSPEND\n");
		handled++;

		if (devctl & MGC_M_DEVCTL_HM) {
			ERR("Calling stop Host in response to SUSPD \n");
			stop_host(pThis, FALSE);
		} else {
#ifdef CONFIG_USB_GADGET_MUSB_HDRC
			MGC_GadgetSuspend(pThis);
#endif
		}

		/* reading suspend state from Power register does NOT work */
		MGC_LinuxOtgInputChanged(pThis, devctl, FALSE, FALSE, TRUE);
	}

	return handled;
}

/*
* Program the HDRC to start (enable interrupts, etc.).
*/
void MGC_HdrcStart(MGC_LinuxCd * pThis)
{
	u8 bEnd = 1;
	u8 *pBase = (u8 *) pThis->pRegs;
	u16 val;
	u32 reg32, old32;
	u8 state;

	DBG(2, "<==\n");

#ifdef	MUSB_STATISTICS
	/* zero counters */
	for (bEnd = 0; bEnd < pThis->bEndCount; bEnd++) {
		pThis->aLocalEnd[bEnd].dwTotalRxBytes = 0;
		pThis->aLocalEnd[bEnd].dwTotalRxPackets = 0;
		pThis->aLocalEnd[bEnd].dwErrorRxPackets = 0;
		pThis->aLocalEnd[bEnd].dwTotalTxBytes = 0;
		pThis->aLocalEnd[bEnd].dwTotalTxPackets = 0;
		pThis->aLocalEnd[bEnd].dwErrorTxPackets = 0;
	}
#endif

	/* TODO: always set ISOUPDATE in POWER (periph mode) and leave it on! */

	/*  Set INT enable registers, enable interrupts */
#ifndef CONFIG_USB_INVENTRA_HCD_POLLING
	MGC_Write16(pBase, MGC_O_HDRC_INTRTXE, pThis->wEndMask);
	MGC_Write16(pBase, MGC_O_HDRC_INTRRXE, pThis->wEndMask & 0xfffe);

#if 1 // ifndef CONFIG_ARCH_DAVINCI
	MGC_Write8(pBase, MGC_O_HDRC_INTRUSBE, 0xf7);
#endif

#ifdef CONFIG_ARCH_DAVINCI
	/*
	 * Workaround for current IRQ handling issue:  IRQs must be
	 * manipulated in both Mentor and TI register sets.
	 */
	reg32 = ((pThis->wEndMask) & USB_OTG_TX_ENDPTS_MASK)
			<< USB_OTG_TXINT_SHIFT;
	MGC_Write32((pBase - MENTOR_BASE_OFFSET), USB_OTG_INT_MASK_SET_REG,
		    reg32);
	old32 = reg32;
	reg32 = ((pThis-> wEndMask & 0xfffe) & USB_OTG_RX_ENDPTS_MASK)
			<< USB_OTG_RXINT_SHIFT;
	MGC_Write32((pBase - MENTOR_BASE_OFFSET), USB_OTG_INT_MASK_SET_REG,
		    reg32);
	reg32 |= old32;

#ifdef MUSB_SCHEDULER
	/* TODO: enable SOF interrupts only when needed (maybe in ProgramBusState
	 * when the schedule is non-empty) */
	val = 0xff;
#else
	val = (u16) ~ MGC_M_INTR_SOF;
#endif	/* SCHEDULER */

	reg32 |= ((val & 0x01ff) << USB_OTG_USBINT_SHIFT);

	MGC_Write32((pBase - MENTOR_BASE_OFFSET), USB_OTG_INT_MASK_SET_REG,
		    reg32);
#endif	/* DAVINCI */
#endif	/* !POLLING */

	MGC_Write8(pBase, MGC_O_HDRC_TESTMODE, 0);

	/* enable high-speed/low-power and start session */
	MGC_Write8(pBase, MGC_O_HDRC_POWER,
		   MGC_M_POWER_SOFTCONN | MGC_M_POWER_HSENAB);

	switch (pThis->board_mode) {
	case MUSB_HOST:
	case MUSB_OTG:
		MGC_Write8(pBase, MGC_O_HDRC_DEVCTL, MGC_M_DEVCTL_SESSION);
		break;
	case MUSB_PERIPHERAL:
		state = MGC_Read8(pBase, MGC_O_HDRC_DEVCTL);
		MGC_Write8(pBase, MGC_O_HDRC_DEVCTL,
			   state & ~MGC_M_DEVCTL_SESSION);
		break;
	}

#if 1
	pr_debug("DEVCTL %02x, POWER %02x\n",
		MGC_Read8(pBase, MGC_O_HDRC_DEVCTL),
		MGC_Read8(pBase, MGC_O_HDRC_POWER)
		);
#endif
}

/*
 * Disable the HDRC (disable & flush interrupts);
 * @param pThis the controller to disable
 */
static void MGC_HdrcDisable(MGC_LinuxCd * pThis)
{
	u8 *pBase = (u8 *) pThis->pRegs;

#ifdef CONFIG_ARCH_DAVINCI
	/* because we don't set CTRLR.UINT, "important" to:
	 *  - not read/write INTRUSB/INTRUSBE
	 *  - use INTSETR/INTCLRR instead
	 */
	MGC_Write32(pBase - MENTOR_BASE_OFFSET, USB_OTG_INT_MASK_CLR_REG,
		USB_OTG_USBINT_MASK | USB_OTG_TXINT_MASK | USB_OTG_RXINT_MASK);
	MGC_Write8(pBase, MGC_O_HDRC_DEVCTL, 0);
	MGC_Write32((pBase - MENTOR_BASE_OFFSET), USB_OTG_EOI_REG, 0);
#else
	u16 temp;

	/* disable interrupts */
	MGC_Write8(pBase, MGC_O_HDRC_INTRUSBE, 0);
	MGC_Write16(pBase, MGC_O_HDRC_INTRTX, 0);
	MGC_Write16(pBase, MGC_O_HDRC_INTRRX, 0);

	/* off */
	MGC_Write8(pBase, MGC_O_HDRC_DEVCTL, 0);

	/*  flush pending interrupts */
	temp = MGC_Read8(pBase, MGC_O_HDRC_INTRUSB);
	temp = MGC_Read16(pBase, MGC_O_HDRC_INTRTX);
	temp = MGC_Read16(pBase, MGC_O_HDRC_INTRRX);
#endif

	DBG(3, "Interrupts disabled\n");
}

/*
* Program the HDRC to stop (disable interrupts, etc.).
*/
void MGC_HdrcStop(MGC_LinuxCd * pThis)
{
	DBG(2, "<==\n");

	/* flush endpoints */
#ifdef CONFIG_USB_MUSB_HDRC_HCD
	{
		u8 bEnd;
		MGC_HdrcDisable(pThis);
		for (bEnd = 0; bEnd < min(16, (int)pThis->bEndCount); bEnd++) {
			MGC_HdrcStopEnd(pThis, bEnd);
		}
	}
#endif
}

#ifdef MUSB_C_DYNFIFO_DEF
#define DYN_FIFO_SIZE (1<<(MUSB_C_RAM_BITS+2))

//#undef MUSB_HDRC_EPSDESCRIPTOR
#if 0
//#ifdef MUSB_HDRC_EPSDESCRIPTOR

#include "musb_epdescriptors.h"

/*
 */
static int __init
configure_fifo(MGC_LinuxCd * pThis, MGC_LinuxLocalEnd * pEnd,
			  u8 bDir, u16 wSize, u16 wFifoOffset)
{
	void *pBase = pThis->pRegs;

	int dynEpSz = 0;
	for (dynEpSz = 0; (wSize >> dynEpSz); dynEpSz++) {
		if ((wSize >> dynEpSz) & 1) {
			dynEpSz -= 3;
			break;
		}
	}

	/* configure the FIFO */
	MGC_SelectEnd(pBase, pEnd->bEnd);

	switch (bDir) {
	case MUSB_EPD_D_TX:
		MGC_Write8(pBase, MGC_O_HDRC_TXFIFOSZ, dynEpSz);
		MGC_Write16(pBase, MGC_O_HDRC_TXFIFOADD, wFifoOffset >> 3);

		pEnd->wMaxPacketSizeTx = wSize;
		pEnd->wMaxPacketSizeRx = 0;
		break;
	case MUSB_EPD_D_RX:
		MGC_Write8(pBase, MGC_O_HDRC_RXFIFOSZ, dynEpSz);
		MGC_Write16(pBase, MGC_O_HDRC_RXFIFOADD, wFifoOffset >> 3);

		pEnd->wMaxPacketSizeTx = 0;
		pEnd->wMaxPacketSizeRx = wSize;
		break;
	case MUSB_EPD_D_INOUT:
		MGC_Write8(pBase, MGC_O_HDRC_TXFIFOSZ, dynEpSz);
		MGC_Write8(pBase, MGC_O_HDRC_RXFIFOSZ, dynEpSz);
		MGC_Write16(pBase, MGC_O_HDRC_TXFIFOADD, wFifoOffset >> 3);
		MGC_Write16(pBase, MGC_O_HDRC_RXFIFOADD, wFifoOffset >> 3);

		pEnd->wMaxPacketSizeTx = pEnd->wMaxPacketSizeRx = wSize;
		pEnd->bIsSharedFifo = TRUE;
		break;

	default:
		ERR("direction %d not supported\n", bDir);
		break;
	}

	/* flush the FIFO */

	return 0;
}

static int __init
autoconfig_fifos(MGC_LinuxCd * pThis,
			    struct MUSB_EpFifoDescriptor *dscs, u16 wFifoOffset)
{
	u16 wFifoSize = DYN_FIFO_SIZE, wShared;
	u8 bEnd, isoc = 0, bulk = 0, intr = 0, autoc = 0, ctrl = 0;

	for (bEnd = 0; bEnd < pThis->bEndCount; bEnd++) {
		if (bEnd == 0) {	/* ep0 */
			wFifoSize -= MGC_END0_FIFOSIZE;
			continue;
		}

		if (!dscs[bEnd].wSize) {
			switch (dscs[bEnd].bType) {
			case MUSB_EPD_T_CNTRL:
				ctrl++;
				break;
			case MUSB_EPD_T_ISOC:
				isoc++;
				break;
			case MUSB_EPD_T_BULK:
				bulk++;
				break;
			case MUSB_EPD_T_INTR:
				intr++;
				break;
			case MUSB_EPD_AUTOCONFIG:
				autoc++;
				break;
			}
		} else {
			wFifoSize -= dscs[bEnd].wSize;
		}
	}

	/* redistribute space, isoc first, bulk then, shared generic;
	 * not implemented yet; use the other method
	 */
	if (ctrl) {
		autoc += ctrl;
	}

	if (isoc) {
		autoc += isoc;
	}

	if (bulk) {
		autoc += bulk;
	}

	if (intr) {
		autoc += intr;
	}

	INFO("wFifoSize=%d, autoc=%d\n", wFifoSize, autoc);

	/* distribute the remaining space */
	wShared = wFifoSize / autoc;
	if (wShared) {
		u16 wShift = (1 << 15);
		while (wShift && !(wShared & wShift)) {
			wShift = wShift >> 1;
		}
		wShared &= ~(wShift - 1);
	}

	for (bEnd = 0; bEnd < pThis->bEndCount; bEnd++) {
		MGC_LinuxLocalEnd *pEnd = &(pThis->aLocalEnd[bEnd]);

		if (dscs[bEnd].wSize) {	/* only the autoconfig points */
			continue;
		}

		if (bEnd == 0) {	/* ep0 is fixed in size */
			int rc = configure_fifo(pThis, pEnd, MUSB_EPD_D_INOUT,
						MGC_END0_FIFOSIZE, 0);
			if (rc == 0) {
				wFifoOffset += MGC_END0_FIFOSIZE;
			}

		} else {
			int rc = configure_fifo(pThis, pEnd, dscs[bEnd].bDir,
						wShared, wFifoOffset);
			if (rc == 0) {
				wFifoOffset +=
				    MUSB_aEpFifoDescriptors[bEnd].wSize;
			}
		}
	}

	return 0;
}

/*
 * MGC_HdrcConfigureEps - (disabled) configure endpoints to match a table
 * @param pThis the controller
 */
static void __init MGC_HdrcConfigureEps(MGC_LinuxCd * pThis)
{
	u8 bEnd = 1;
	MGC_LinuxLocalEnd *pEnd;
	void *pBase = pThis->pRegs;
	u16 wFifoOffset = 0;

	/* use the defined end points */
	pThis->bEndCount = MUSB_C_NUM_EPS;
	pThis->wEndMask |= (1 << MUSB_C_NUM_EPS);

	/* endpoint 0 got default configuration */
	MGC_SelectEnd(pBase, 0);
	MGC_Write8(pBase, MGC_O_HDRC_TXFIFOSZ, 3);
	MGC_Write8(pBase, MGC_O_HDRC_RXFIFOSZ, 3);
	MGC_Write16(pBase, MGC_O_HDRC_TXFIFOADD, 0);
	MGC_Write16(pBase, MGC_O_HDRC_RXFIFOADD, 0);
	wFifoOffset = MGC_END0_FIFOSIZE;

#ifdef MUSB_PARANOID
	if (MUSB_aEpFifoDescriptors[0].bType != MUSB_EPD_T_CNTRL
	    && MUSB_aEpFifoDescriptors[0].bType != MUSB_EPD_AUTOCONFIG) {
		WARN("ep0 must be control with fixed size of %d\n",
		     MGC_END0_FIFOSIZE);
	}
#endif

	/* entry 0 is ignored, it gotta bhe the control endpoint */
	for (bEnd = 1; bEnd < MUSB_C_NUM_EPS; bEnd++) {
		pEnd = &(pThis->aLocalEnd[bEnd]);
		pEnd->bIsSharedFifo = FALSE;
		pEnd->wMaxPacketSizeTx = pEnd->wMaxPacketSizeRx = 0;
		pEnd->bEnd = bEnd;

		switch (MUSB_aEpFifoDescriptors[bEnd].bType) {
		case MUSB_EPD_T_CNTRL:
			if (bEnd) {
				WARN("Control ep when ep!=0 (ep%d); skipping\n",
				     bEnd);
				goto invalid;
			}
			break;

		case MUSB_EPD_T_ISOC:
			break;

		case MUSB_EPD_T_BULK:
			switch (MUSB_aEpFifoDescriptors[bEnd].bDir) {
			case MUSB_EPD_D_TX:
				pThis->bBulkTxEnd = bEnd;
				break;
			case MUSB_EPD_D_RX:
				pThis->bBulkRxEnd = bEnd;
				break;
			case MUSB_EPD_D_INOUT:
				WARN("INOUTBULK not supported yet (ep%d); sharing it\n", bEnd);
				break;
			default:
				ERR("direction %d not supported for ep%d\n",
				    MUSB_aEpFifoDescriptors[bEnd].bDir, bEnd);
				goto invalid;
				break;
			}
			break;

		case MUSB_EPD_T_INTR:
			break;

			/*  */
		case MUSB_EPD_AUTOCONFIG:
			break;

		default:
			ERR("ep%d type %d not supported\n", bEnd,
			    MUSB_aEpFifoDescriptors[bEnd].bType);
			goto invalid;
			break;
		}

		printk("wFifoOffset = %d bEnd = %d size=%d \n", wFifoOffset,
		       bEnd, MUSB_aEpFifoDescriptors[bEnd].wSize);
		/* configure the ep if I have all the data */
		if (MUSB_aEpFifoDescriptors[bEnd].wSize) {
			int rc =
			    configure_fifo(pThis, pEnd,
					   MUSB_aEpFifoDescriptors[bEnd].bDir,
					   MUSB_aEpFifoDescriptors[bEnd].wSize,
					   wFifoOffset);
			if (rc == 0) {
				wFifoOffset +=
				    MUSB_aEpFifoDescriptors[bEnd].wSize;
				printk("wFifoOffset = %d bEnd = %d\n",
				       wFifoOffset, bEnd);
			}
		}

	      invalid:		/* do nothing */

	}

#ifdef MUSB_PARANOID
	if (wFifoOffset > DYN_FIFO_SIZE) {
		ERR("Allocated %d bytes, more than the allowed %d\n",
		    wFifoOffset, DYN_FIFO_SIZE);
	} else {
		INFO("Allocated %d bytes, out of %d\n",
		     wFifoOffset, DYN_FIFO_SIZE);
	}
#endif

	/* globally optmize the fifo allocation for the rest of eps */
	/*autoconfig_fifos(pThis, MUSB_aEpFifoDescriptors, wFifoOffset); */
}

#else				/* not disabled */

/* mode 0 == ep1:512/-        ep2:-/512    ep3:256  ep4:256
 * mode 1 == ep1:512*2/-      ep2:-/512*2  ep3:256  ep4:256
 * mode 2 == ep1:512*2/512*2  ep2:512/512  ep3:512  ep4:512
 * ... define more as you need them, even application-custom
 */
static ushort fifo_mode = 0;

/* "modprobe ... fifo_mode=1" etc */
module_param (fifo_mode, ushort, 0644);

/*
 * MGC_HdrcConfigureEps - configure endpoints to match compiled-in defaults
 * @param pThis the controller
 */
static void __init MGC_HdrcConfigureEps(MGC_LinuxCd * pThis)
{
	u8 bEnd = 0;
	MGC_LinuxLocalEnd *pEnd;
	void *pBase = pThis->pRegs;
	u16 wFifoOffset = MGC_END0_FIFOSIZE;

	DBG(2, "<== manual ep setup, dynfifo\n");

	/* use the defined end points */
	pThis->bEndCount = MUSB_C_NUM_EPS;
	/* pThis->wEndMask |= (1 << MUSB_C_NUM_EPS); */

	/* Dynamic FIFO sizing: use pre-computed values for EP0 */
	MGC_SelectEnd(pBase, bEnd);
	MGC_Write8(pBase, MGC_O_HDRC_TXFIFOSZ, 3);
	MGC_Write8(pBase, MGC_O_HDRC_RXFIFOSZ, 3);
	MGC_Write16(pBase, MGC_O_HDRC_TXFIFOADD, 0);
	MGC_Write16(pBase, MGC_O_HDRC_RXFIFOADD, 0);
	pThis->wEndMask = 1;
	bEnd++;

#if 0
#if MGC_DFIFO_ISO_TX >= 0
	MGC_SelectEnd(pBase, bEnd);
	pEnd = &(pThis->aLocalEnd[bEnd]);

	/* reserve ISO Tx */
	MGC_Write8(pBase, MGC_O_HDRC_TXFIFOSZ, MGC_DFIFO_ISO_TX_VAL);
	pEnd->wMaxPacketSizeTx =
	    1 << ((MGC_DFIFO_ISO_TX_VAL & 0xf) + 3 +
		  (MGC_DFIFO_ISO_TX_VAL >> 4));
	pEnd->wMaxPacketSizeRx = 0;
	MGC_Write16(pBase, MGC_O_HDRC_TXFIFOADD, wFifoOffset >> 3);
	/* move to next */
	wFifoOffset += pEnd->wMaxPacketSizeTx;
	pEnd->bIsSharedFifo = FALSE;
	bEnd++;

#endif

#if MGC_DFIFO_ISO_RX >= 0
	MGC_SelectEnd(pBase, bEnd);
	pEnd = &(pThis->aLocalEnd[bEnd]);

	/* reserve ISO Rx */
	MGC_Write8(pBase, MGC_O_HDRC_RXFIFOSZ, MGC_DFIFO_ISO_RX_VAL);
	pEnd->wMaxPacketSizeTx = 0;
	pEnd->wMaxPacketSizeRx =
	    1 << ((MGC_DFIFO_ISO_RX_VAL & 0xf) + 3 +
		  (MGC_DFIFO_ISO_RX_VAL >> 4));
	MGC_Write16(pBase, MGC_O_HDRC_RXFIFOADD, wFifoOffset >> 3);

	/* move to next */
	wFifoOffset += pEnd->wMaxPacketSizeRx;
	pEnd->bIsSharedFifo = FALSE;
	bEnd++;
#endif
#endif
	/* REVISIT:  test double buffering for these first two.
	 * Use both RX and TX sides for the first endpoint...
	 */

 	/* bulk TX/OUT (reserved, on host side) */
	MGC_SelectEnd(pBase, bEnd);
	pEnd = &(pThis->aLocalEnd[bEnd]);
	/* reserve bulk */
	pEnd->wMaxPacketSizeRx = 0;
	pEnd->wMaxPacketSizeTx =
	    1 << ((MGC_DFIFO_BLK_VAL & 0xf) + 3 + (MGC_DFIFO_BLK_VAL >> 4));
	MGC_Write8(pBase, MGC_O_HDRC_TXFIFOSZ, MGC_DFIFO_BLK_VAL
			| ((fifo_mode == 1 || fifo_mode == 2) ? 0x10 : 0)
			);
	MGC_Write16(pBase, MGC_O_HDRC_TXFIFOADD, wFifoOffset >> 3);
	pThis->bBulkTxEnd = bEnd;
	/* move to next */
	wFifoOffset += pEnd->wMaxPacketSizeTx
			* ((fifo_mode == 1 || fifo_mode == 2) ? 2 : 1);
	pThis->wEndMask |= (1 << bEnd);
	bEnd++;

	/* bulk RX/IN (reserved, on host side) */
	MGC_SelectEnd(pBase, bEnd);
	pEnd = &(pThis->aLocalEnd[bEnd]);
	pEnd->wMaxPacketSizeTx = 0;
	pEnd->wMaxPacketSizeRx =
	    1 << ((MGC_DFIFO_BLK_VAL & 0xf) + 3 + (MGC_DFIFO_BLK_VAL >> 4));
	MGC_Write8(pBase, MGC_O_HDRC_RXFIFOSZ, MGC_DFIFO_BLK_VAL
			| ((fifo_mode == 1 || fifo_mode == 2) ? 0x10 : 0)
			);
	MGC_Write16(pBase, MGC_O_HDRC_RXFIFOADD, wFifoOffset >> 3);
	pThis->bBulkRxEnd = bEnd;

	/* move to next */
	wFifoOffset += pEnd->wMaxPacketSizeRx
			* ((fifo_mode == 1 || fifo_mode == 2) ? 2 : 1);
	pThis->wEndMask |= (1 << bEnd);
	bEnd++;

	/* take care of the remaining eps */
	for (; bEnd < MUSB_C_NUM_EPS; bEnd++) {
		MGC_SelectEnd(pBase, bEnd);
		pEnd = &(pThis->aLocalEnd[bEnd]);
		MGC_Write8(pBase, MGC_O_HDRC_TXFIFOSZ, MGC_DFIFO_ALL_VAL);
		MGC_Write8(pBase, MGC_O_HDRC_RXFIFOSZ, MGC_DFIFO_ALL_VAL);
		pEnd->wMaxPacketSizeTx = pEnd->wMaxPacketSizeRx =
		    1 << (MGC_DFIFO_ALL_VAL + 3);
		pEnd->bIsSharedFifo = TRUE;
		MGC_Write16(pBase, MGC_O_HDRC_TXFIFOADD, wFifoOffset >> 3);
		MGC_Write16(pBase, MGC_O_HDRC_RXFIFOADD, wFifoOffset >> 3);
		wFifoOffset += pEnd->wMaxPacketSizeRx;
		pThis->wEndMask |= (1 << bEnd);
	}

	if (wFifoOffset > DYN_FIFO_SIZE) {
#ifdef MUSB_PARANOID
		ERR("Allocated %d bytes, more than the allowed %d\n",
		    wFifoOffset, DYN_FIFO_SIZE);
#endif
	} else
		DBG(1, "Allocated %d bytes, out of %d\n",
		     wFifoOffset, DYN_FIFO_SIZE);

	DBG(2, "<==\n");
}
#endif

#else

/*
 * MGC_HdrcConfigureEps - when MUSB_C_DYNFIFO_DEF is false
 * @param pThis the controller
 */
static void __init MGC_HdrcConfigureEps(MGC_LinuxCd * pThis)
{
	u8 bEnd = 0, reg;
	MGC_LinuxLocalEnd *pEnd;
	void *pBase = pThis->pRegs;
	/* how many of a given size/direction found: */
	u8 b2kTxEndCount = 0;
	u8 b2kRxEndCount = 0;
	u8 b1kTxEndCount = 0;
	u8 b1kRxEndCount = 0;
	/* the smallest 2k or 1k ends in Tx or Rx direction: */
	u8 b2kTxEnd = 0;
	u8 b2kRxEnd = 0;
	u8 b1kTxEnd = 0;
	u8 b1kRxEnd = 0;
	/* for tracking smallest: */
	u16 w2kTxSize = 0;
	u16 w1kTxSize = 0;
	u16 w2kRxSize = 0;
	u16 w1kRxSize = 0;

	DBG(2, "<== static silicon ep config\n");

	for (bEnd = 1; bEnd < MUSB_C_NUM_EPS; bEnd++) {
		MGC_SelectEnd(pBase, bEnd);
		pEnd = &(pThis->aLocalEnd[bEnd]);

		/* read from core */
		reg = MGC_ReadCsr8(pBase, MGC_O_HDRC_FIFOSIZE, bEnd);
		if (!reg) {
			/* 0's returned when no more endpoints */
			break;
		}

		pEnd->wMaxPacketSizeTx = 1 << (reg & 0x0f);
		/* shared TX/RX FIFO? */
		if ((reg & 0xf0) == 0xf0) {
			pEnd->wMaxPacketSizeRx = 1 << (reg & 0x0f);
			pEnd->bIsSharedFifo = TRUE;
		} else {
			pEnd->wMaxPacketSizeRx = 1 << ((reg & 0xf0) >> 4);
			pEnd->bIsSharedFifo = FALSE;
		}

		/* track certain sizes to try to reserve a bulk resource */
		if (pEnd->wMaxPacketSizeTx >= 2048) {
			b2kTxEndCount++;
			if (!b2kTxEnd || (pEnd->wMaxPacketSizeTx < w2kTxSize)) {
				b2kTxEnd = bEnd;
				w2kTxSize = pEnd->wMaxPacketSizeTx;
			}
		}

		if (pEnd->wMaxPacketSizeRx >= 2048) {
			b2kRxEndCount++;
			if (!b2kRxEnd || (pEnd->wMaxPacketSizeRx < w2kRxSize)) {
				b2kRxEnd = bEnd;
				w2kRxSize = pEnd->wMaxPacketSizeRx;
			}
		}

		if (pEnd->wMaxPacketSizeTx >= 1024) {
			b1kTxEndCount++;
			if (!b1kTxEnd || (pEnd->wMaxPacketSizeTx < w1kTxSize)) {
				b1kTxEnd = bEnd;
				w1kTxSize = pEnd->wMaxPacketSizeTx;
			}
		}

		if (pEnd->wMaxPacketSizeRx >= 1024) {
			b1kRxEndCount++;
			if (!b1kRxEnd || (pEnd->wMaxPacketSizeRx < w1kTxSize)) {
				b1kRxEnd = bEnd;
				w1kRxSize = pEnd->wMaxPacketSizeRx;
			}
		}

		pThis->bEndCount++;
		pThis->wEndMask |= (1 << bEnd);
	}			/* init queues etc. etc. etc. */

	/* if possible, reserve the smallest 2k-capable Tx end for bulk */
	if (b2kTxEnd && (b2kTxEndCount > 1)) {
		pThis->bBulkTxEnd = b2kTxEnd;
		INFO("Reserved end %d for bulk double-buffered Tx\n", b2kTxEnd);
	}
	/* ...or try 1k */
	else if (b1kTxEnd && (b1kTxEndCount > 1)) {
		pThis->bBulkTxEnd = b1kTxEnd;
		INFO("Reserved end %d for bulk Tx\n", b1kTxEnd);
	}

	/* if possible, reserve the smallest 2k-capable Rx end for bulk */
	if (b2kRxEnd && (b2kRxEndCount > 1)) {
		pThis->bBulkRxEnd = b2kRxEnd;
		INFO("Reserved end %d for bulk double-buffered Rx\n", b2kRxEnd);
	}
	/* ...or try 1k */
	else if (b1kRxEnd && (b1kRxEndCount > 1)) {
		pThis->bBulkRxEnd = b1kRxEnd;
		INFO("Reserved end %d for bulk Rx\n", b1kRxEnd);
	}

	DBG(2, "<==\n");
}
#endif

/*
* Discover HDRC configuration.
* @param wType
* @param pThis the controller instance
*/
static u8 __init MGC_HdrcInit(u16 wType, MGC_LinuxCd * pThis)
{
#ifdef MUSB_AHB_ID
	u32 dwData;
#endif
	u8 reg;
	char *type;
	u16 wRelease, wRelMajor, wRelMinor;
	char aInfo[78], aRevision[32], aDate[12];
	void *pBase = pThis->pRegs;

	DBG(2, "<==\n");

	/* log core options */
	MGC_SelectEnd(pBase, 0);
	reg = MGC_ReadCsr8(pBase, MGC_O_HDRC_CONFIGDATA, 0);

	strcpy(aInfo, (reg & MGC_M_CONFIGDATA_UTMIDW) ? "UTMI-16" : "UTMI-8");
	if (reg & MGC_M_CONFIGDATA_DYNFIFO) {
		strcat(aInfo, ", dyn FIFOs");
	}
	if (reg & MGC_M_CONFIGDATA_MPRXE) {
		//pThis->bBulkCombine = TRUE;
		strcat(aInfo, ", bulk combine");
	}
	if (reg & MGC_M_CONFIGDATA_MPTXE) {
		//pThis->bBulkSplit = TRUE;
		strcat(aInfo, ", bulk split");
	}
	if (reg & MGC_M_CONFIGDATA_HBRXE) {
		strcat(aInfo, ", HB-ISO Rx");
	}
	if (reg & MGC_M_CONFIGDATA_HBTXE) {
		strcat(aInfo, ", HB-ISO Tx");
	}
	if (reg & MGC_M_CONFIGDATA_SOFTCONE) {
		strcat(aInfo, ", SoftConn");
	}

	printk(KERN_DEBUG "%s: ConfigData=0x%02x (%s)\n",
			musb_driver_name, reg, aInfo);

#ifdef MUSB_AHB_ID
	dwData = MGC_Read32(pBase, 0x404);
	sprintf(aDate, "%04d-%02x-%02x", (dwData & 0xffff),
		(dwData >> 16) & 0xff, (dwData >> 24) & 0xff);
	/* FIXME ID2 and ID3 are unused */
	dwData = MGC_Read32(pBase, 0x408);
	printk("ID2=%lx\n", (long unsigned)dwData);
	dwData = MGC_Read32(pBase, 0x40c);
	printk("ID3=%lx\n", (long unsigned)dwData);
	reg = MGC_Read8(pBase, 0x400);
	wType = ('M' == reg) ? MUSB_CONTROLLER_MHDRC : MUSB_CONTROLLER_HDRC;
#else
	aDate[0] = 0;
#endif
	if (MUSB_CONTROLLER_MHDRC == wType) {
		pThis->bIsMultipoint = 1;
		type = "M";
	} else {
		pThis->bIsMultipoint = 0;
		type = "";
	}

	/* log release info */
	wRelease = MGC_Read16(pBase, 0x6c);
	wRelMajor = (wRelease >> 10) & 0x1f;
	wRelMinor = wRelease & 0x3ff;
	snprintf(aRevision, 32, "%d.%d%s", wRelMajor,
		 wRelMinor, (wRelease & 0x8000) ? "RC" : "");
	printk(KERN_DEBUG "%s: %sHDRC RTL version %s %s\n",
			musb_driver_name, type, aRevision, aDate);

	/* configure ep0 */
	pThis->aLocalEnd[0].wMaxPacketSizeTx = MGC_END0_FIFOSIZE;
	pThis->aLocalEnd[0].wMaxPacketSizeRx = MGC_END0_FIFOSIZE;

	/* discover endpoint configuration */
	pThis->bBulkTxEnd = 0;
	pThis->bBulkRxEnd = 0;
	pThis->bEndCount = 1;
	pThis->wEndMask = 1;

#ifdef MUSB_C_DYNFIFO_DEF
	if (!(reg & MGC_M_CONFIGDATA_DYNFIFO)) {
		ERR("Dynamic FIFOs not detected in hardware; please rebuild software\n");
		return FALSE;
	}
#else
	if (reg & MGC_M_CONFIGDATA_DYNFIFO) {
		ERR("Dynamic FIFOs detected in hardware; please rebuild\n");
		return FALSE;
	}
#endif

	MGC_HdrcConfigureEps(pThis);

	/* claim the bulk tx/rx ends */
	if (pThis->bBulkTxEnd) {
		pThis->aLocalEnd[pThis->bBulkTxEnd].bIsClaimed = TRUE;
	}

	if (pThis->bBulkRxEnd) {
		pThis->aLocalEnd[pThis->bBulkRxEnd].bIsClaimed = TRUE;
	}

	return TRUE;
}

/*************************************************************************
 * Linux HCD functions
**************************************************************************/

#define IS_TIMER_INITILIZED(_t) ((_t)->magic==TIMER_MAGIC)

/* FIXME this should just mod_timer or be an error ... */

/*
 * Generic timer creation.
 * @param pThis instance pointer
 * @param pfFunc timer fire callback
 * @param pParam parameter for callback
 * @param millisecs how many milliseconds to set
 */
void MGC_LinuxSetTimer(MGC_LinuxCd * pThis, void (*pfFunc) (unsigned long),
		       unsigned long pParam, unsigned long millisecs)
{
	DBG(2, "<==\n");

	if (IS_TIMER_INITILIZED(&pThis->Timer)) {
		del_timer_sync((&pThis->Timer));	/* make sure another timer is not running */
	}

	init_timer(&(pThis->Timer));
	pThis->Timer.function = pfFunc;
	pThis->Timer.data = (unsigned long)pParam;
	pThis->Timer.expires = jiffies + (HZ * millisecs) / 1000;
	add_timer(&(pThis->Timer));
}


/* -------------------------------- MEMORY ----------------------------- */

/* many common platforms have dma-coherent caches, which means that it's
 * safe to use kmalloc() memory for all i/o buffers without using any
 * cache flushing calls.  (unless you're trying to share cache lines
 * between dma and non-dma activities, which is a slow idea in any case.)
 */

#if	defined(CONFIG_X86)
#define USE_KMALLOC

#elif	defined(CONFIG_PPC) && !defined(CONFIG_NOT_COHERENT_CACHE)
#define USE_KMALLOC

#elif	defined(CONFIG_MIPS) && !defined(CONFIG_DMA_NONCOHERENT)
#define USE_KMALLOC

/* NOTE: there are other cases, including an x86-64 one ...  */

/* also use kmalloc when DMA is disabled! */
#elif	defined(CONFIG_USB_INVENTRA_FIFO)
#define USE_KMALLOC

#endif

/*
 * Allocate memory for a buffer that might use DMA.
 *
 * @param pThis
 * @param bytes
 * @param gfp_flags
 * @param dma NULL when DMAble memeory is not requested.
 */
void *MGC_AllocBufferMemory(MGC_LinuxCd * pThis,
			    size_t bytes, int gfp_flags, dma_addr_t * dma)
{
	void *addr;

#ifdef	USE_KMALLOC
	addr = kmalloc(bytes, gfp_flags);
	if (addr)
		*dma = virt_to_phys(addr);
#else
	/* this allocates 2^X pages; the problem is that X is never negative.
	 * e.g. allocating 32 bytes wastes most of page... the host side has
	 * some private dma pools to get rid of most of that cost.
	 */
#ifdef CONFIG_USB_GADGET_MUSB_HDRC
	/* FIXME for dual-role configs, peripheral side should use
	 * the same buffer pools the host side does...
	 */
#endif
	addr = dma_alloc_coherent(pThis->controller, bytes, dma, gfp_flags);
#endif
	if (!addr)
		*dma = DMA_ADDR_INVALID;
	return addr;
}

/*
 * Free memory previously allocated with AllocBufferMemory.
 * @param pThis
 * @param bytes
 * @param dma
 */
void MGC_FreeBufferMemory(MGC_LinuxCd * pThis,
			  size_t bytes, void *address, dma_addr_t dma)
{
#ifndef	USE_KMALLOC
	if (dma != DMA_ADDR_INVALID)
		dma_free_coherent(pThis->controller, bytes, address, dma);
	else
#endif
		kfree(address);
}

/*************************************************************************
 * Linux driver hooks
**************************************************************************/

/*
 * Interrupt Service Routine.
 *
 * @param irq interrupt line associated with the controller
 * @param hci data structure for the host controller
 * @param r holds the snapshot of the processor's context before
 *             the processor entered interrupt code. (not used here)
 */

static irqreturn_t MGC_LinuxIsr(int irq, void *__hci, struct pt_regs *r)
{
	u32 nSource;
	u32 reg32;

#if MUSB_DEBUG > 0
#ifdef CONFIG_USB_MUSB_HDRC_HCD
	u16 wIntrTxCheck, wIntrRxCheck;
#endif
#endif
	MGC_LinuxCd *pThis = (MGC_LinuxCd *) __hci;
	const void *pBase = pThis->pRegs;
	u8 devctl = MGC_Read8(pBase, MGC_O_HDRC_DEVCTL);
	u8 power = MGC_Read8(pBase, MGC_O_HDRC_POWER);
	u16 wIntrUsbValue = 0;
	u16 wIntrTxValue = 0, wIntrRxValue = 0;
#ifdef CONFIG_USB_TI_CPPI_DMA
	u32 intStatusTx = 0, intStatusRx = 0;
#endif

#if 1	// ifndef CONFIG_ARCH_DAVINCI
#ifdef CONFIG_USB_INVENTRA_FIFO
	wIntrUsbValue = MGC_Read8(pBase, MGC_O_HDRC_INTRUSB);
	wIntrTxValue = MGC_Read16(pBase, MGC_O_HDRC_INTRTX);
	wIntrRxValue = MGC_Read16(pBase, MGC_O_HDRC_INTRRX);
#endif
#endif

#if  defined(CONFIG_ARCH_DAVINCI) || defined (CONFIG_USB_TI_CPPI_DMA)
	/* read from PDR block now */
	/* printk("TI FIFO int \n"); */
	reg32 = MGC_Read32((pBase - MENTOR_BASE_OFFSET),
		       USB_OTG_INT_SRC_MASKED_REG);
	/* acknowledge the interrupt */
	MGC_Write32((pBase - MENTOR_BASE_OFFSET), USB_OTG_INT_SRC_CLR_REG,
		    reg32);

	wIntrRxValue =
	    (u16) ((reg32 & USB_OTG_RXINT_MASK) >> USB_OTG_RXINT_SHIFT);
	wIntrTxValue =
	    (u16) ((reg32 & USB_OTG_TXINT_MASK) >> USB_OTG_TXINT_SHIFT);
	wIntrUsbValue =
	    (u16) ((reg32 & USB_OTG_USBINT_MASK) >> USB_OTG_USBINT_SHIFT);

#ifdef CONFIG_USB_TI_CPPI_DMA
	/*  printk("CPPI int block \n"); */
	intStatusTx =
	    MGC_Read32(pBase - MENTOR_BASE_OFFSET, USB_OTG_TXCPPI_MASKED_REG);
	intStatusRx =
	    MGC_Read32(pBase - MENTOR_BASE_OFFSET, USB_OTG_RXCPPI_MASKED_REG);
	//dbgPrint("\t *** UsbInt:0x%x Tx:0x%x rx:0x%x CPPIRx:0x%x CPPITx:0x%x\n", wIntrUsbValue , wIntrTxValue , wIntrRxValue, intStatusRx,intStatusTx);
	if (intStatusTx | intStatusRx) {
		/*printk("in CPPI handler\n"); */
		funcCppiCompletionIsr();
		if (!(wIntrUsbValue | wIntrTxValue | wIntrRxValue)) {
			/*printk("completed CPPI ISR alone \n"); */
			return IRQ_HANDLED;
		}
	}
#endif
#endif

	/* determine if we were the cause; return if not */
	nSource = wIntrUsbValue | wIntrTxValue | wIntrRxValue;
	DEBUG_CODE(10, if (!nSource) {
		   INFO("IRQ [mode=%s] nSource=%d\n", MUSB_MODE(pThis),
			nSource);}
	) ;

#ifndef CONFIG_USB_INVENTRA_HCD_POLLING
	if (!nSource) {
		return IRQ_NONE;
	}
#endif

	DBG(3,
	    "<== [%ld]: IRQ RECEIVED [devmode=%s, hwmode=%s] IntrUSB=%04x, IntrTx=%04x, IntrRx=%04x\n",
	    jiffies, MUSB_MODE(pThis),
	    (devctl & MGC_M_DEVCTL_HM) ? "host" : "function", wIntrUsbValue,
	    wIntrTxValue, wIntrRxValue);

	/* corruption check */
#ifdef MUSB_PARANOID
	if (MGC_ISCORRUPT(pThis)) {
		INFO("stopping before ISR, the controller structure is corrupted\n");
		MGC_HdrcStop(pThis);
		MUSB_ERR_MODE(pThis, MUSB_ERR_CORRUPTED);

		return IRQ_HANDLED;
	}
#endif

	/* the core can interrupt us for multiple reasons, I.E. more than an
	 * interrupt line can be asserted; service the globa interrupt first.
	 * Global interrups are used to signal connect/disconnect/vbuserr
	 * etc. processed in two phase */
	if (wIntrUsbValue) {
		MGC_HdrcServiceUsbStage0(pThis, wIntrUsbValue, devctl, power);
#ifdef CONFIG_ARCH_DAVINCI
		if (wIntrUsbValue & (1 << 8))
			DBG(1, "DRVVBUS %d\n", MGC_Read32(
					pBase - MENTOR_BASE_OFFSET,
					USB_OTG_STAT_REG));
#endif
	}
#ifdef MUSB_PARANOID
	if (wIntrTxValue || wIntrRxValue) {	/* got data! */
		if (((devctl & MGC_M_DEVCTL_HM) && !MUSB_IS_HST(pThis))
		    || (!(devctl & MGC_M_DEVCTL_HM) && !MUSB_IS_DEV(pThis))) {
			if (wIntrUsbValue) {
				MGC_HdrcServiceUsbStage1(pThis, wIntrUsbValue,
							 devctl, power);
			} else {
				WARN("early interrupt while in hm=%d: otg machine hasn't done yet\n", devctl & MGC_M_DEVCTL_HM);
			}

			return IRQ_HANDLED;
		}
	}
#endif

	/* ignore requests when in error */
	if (MUSB_IS_ERR(pThis)) {
		if (wIntrUsbValue) {
			MGC_HdrcServiceUsbStage1(pThis, wIntrUsbValue, devctl,
						 power);
		} else {
			ERR("Error mode, please ZAP the driver!\n");
			MGC_HdrcDisable(pThis);
		}

		return IRQ_HANDLED;
	}

	/* handle tx/rx on endpoints; each bit of wIntrTxValue is an endpoint,
	 * endpoint 0 first (p35 of the manual) bc is "SPECIAL" treatment;
	 * WARNING: when operating as device you might start receving traffic
	 * to ep0 before anything else happens so be ready for it */
	do {
		u8 bShift = 0;
		u32 reg = wIntrTxValue;

		if (reg & 1) {	/* EP0 */
			if (devctl & MGC_M_DEVCTL_HM) {
#ifdef CONFIG_USB_MUSB_HDRC_HCD
#ifdef MUSB_CONFIG_PROC_FS
				if (pThis->pfDefaultEndHandler) {
					pThis->pfDefaultEndHandler(pThis->
								   pDefaultEndHandlerParam);
				} else
#endif
					MGC_HdrcServiceDefaultEnd(pThis);
#endif
			} else {
#ifdef CONFIG_USB_GADGET_MUSB_HDRC
				MGC_HdrcServiceDeviceDefaultEnd(pThis);
#endif
			}
		}
#ifdef MUSB_PARANOID
		if (MGC_ISCORRUPT(pThis)) {
			INFO("after servicing Ep0 interrupt\n");
			break;
		}
#endif

		/* TX on endpoints 1-15 */
		bShift = 1;
		reg >>= 1;
		while (reg) {
			if (reg & 1) {
				if (devctl & MGC_M_DEVCTL_HM) {
#ifdef CONFIG_USB_MUSB_HDRC_HCD
					MGC_HdrcServiceTxAvail(pThis, bShift);
#endif
				} else {
#ifdef CONFIG_USB_GADGET_MUSB_HDRC
					MGC_HdrcServiceDeviceTxAvail(pThis,
								     bShift);
#endif
				}
			}
			reg >>= 1;
			bShift++;
		}

#ifdef CONFIG_USB_MUSB_HDRC_HCD
		DEBUG_CODE(10, wIntrTxCheck =
			   MGC_Read16(pBase, MGC_O_HDRC_INTRTX);
			   if (wIntrTxCheck && (wIntrTxCheck == wIntrTxValue)) {
			   ERR
			   ("Unhandled TX interrupt, wIntrTx=%04x wIntrTxCheck=%04x; DRC stopped\n",
			    wIntrTxValue, wIntrTxCheck); for (bShift = 0;
							      bShift <
							      pThis->bEndCount;
							      bShift++) {
			   MGC_HdrcDumpRegs(pThis->pRegs, MUSB_IS_HST(pThis)
					    && pThis->bIsMultipoint, bShift);}
			   MGC_HdrcStop(pThis);
			   MUSB_ERR_MODE(pThis, MUSB_ERR_IRQ);
			   MGC_VirtualHubPortDisconnected(&pThis->RootHub, 0);
			   pThis->pRootDevice = NULL;}
		) ;
#endif

#ifdef MUSB_PARANOID
		if (MGC_ISCORRUPT(pThis)) {
			INFO("after servicing Tx interrupt\n");
			break;
		}
#endif

		/* RX on endpoints 1-15 */
		reg = wIntrRxValue;
		bShift = 1;
		reg >>= 1;
		while (reg) {
			if (reg & 1) {
				if (devctl & MGC_M_DEVCTL_HM) {
#ifdef CONFIG_USB_MUSB_HDRC_HCD
					MGC_HdrcServiceRxReady(pThis, bShift);
#endif
				} else {
#ifdef CONFIG_USB_GADGET_MUSB_HDRC
					MGC_HdrcServiceDeviceRxReady(pThis,
								     bShift);
#endif
				}
			}

			reg >>= 1;
			bShift++;
		}

#ifdef CONFIG_USB_MUSB_HDRC_HCD
		DEBUG_CODE(10, wIntrRxCheck =
			   MGC_Read16(pBase, MGC_O_HDRC_INTRRX);
			   if (wIntrRxCheck && (wIntrRxCheck == wIntrRxValue)) {
			   printk(KERN_ERR
				  "%s:%d: Unhandled RX interrupt, IntrRx=%04x; IntrRxCheck=%04x DRC stopped\n",
				  __FUNCTION__, __LINE__, wIntrRxValue,
				  wIntrRxCheck); for (bShift = 0;
						      bShift < pThis->bEndCount;
						      bShift++) {
			   MGC_HdrcDumpRegs(pThis->pRegs, MUSB_IS_HST(pThis)
					    && pThis->bIsMultipoint, bShift);}
			   MGC_HdrcStop(pThis);
			   MUSB_ERR_MODE(pThis, MUSB_ERR_IRQ);
			   MGC_VirtualHubPortDisconnected(&pThis->RootHub, 0);
			   pThis->pRootDevice = NULL;}
		) ;
#endif

		/* Global interrups are used to signal connect/disconnect/vbuserr
		 * etc. processed in two phase */
		if (wIntrUsbValue) {
			MGC_HdrcServiceUsbStage1(pThis, wIntrUsbValue, devctl,
						 power);
		}
#ifdef MUSB_PARANOID
		if (MGC_ISCORRUPT(pThis)) {
			INFO("stopping after servicing Rx interrupt\n");
		}
#endif
	} while (0);

#ifdef MUSB_PARANOID
	if (MGC_ISCORRUPT(pThis)) {
		MGC_HdrcStop(pThis);
		MUSB_ERR_MODE(pThis, MUSB_ERR_CORRUPTED);
	}
#endif

#ifdef	CONFIG_ARCH_DAVINCI
	MGC_Write32((pBase - MENTOR_BASE_OFFSET), USB_OTG_EOI_REG, 0);
#endif
	DBG(4, "IRQ HANDLED [devmode=%s]\n", MUSB_MODE(pThis));

	return IRQ_HANDLED;
}

#ifdef CONFIG_USB_INVENTRA_HCD_POLLING
static int MGC_LinuxPoller(void *pData)
{
	MGC_LinuxCd *pThis = (MGC_LinuxCd *) pData;

	printk(KERN_INFO "%s: started\n", __FUNCTION__);

	lock_kernel();

	/*
	 * This thread doesn't need any user-level access,
	 * so get rid of all our resources
	 */
	daemonize(pThis->controller->bus_id);

	/* Send me a signal to get me die (for debugging) */
	do {
#if 0
		u32 src = MGC_Read32(pThis->pRegs - 0x400, 0x020 /*INTSRCR*/);
		u8 ctl = MGC_Read8(pThis->pRegs, MGC_O_HDRC_DEVCTL);
		static u8 lastctl;

		if ((src & 0x00ff1f1f) || lastctl != ctl)
			pr_debug("IRQ: s%06x c %02x(%02x)\n", src, ctl,
				 ctl ^ lastctl);
		lastctl = ctl;

		ctl = MGC_Read8(pThis->pRegs, MGC_O_HDRC_POWER);
		if (ctl & MGC_M_POWER_RESET)
			pr_debug("USB RESET: pwr %02x\n", ctl);
#endif

		MGC_LinuxIsr(pThis->nIrq, pThis, NULL);
		msleep_interruptible(10);
	} while (!signal_pending(current));

	dev_dbg(pThis->controller, "MGC_LinuxPoller exiting");

	unlock_kernel();
	complete_and_exit(&MGC_PollerExited, 0);

}
#endif

/*************************************************************************
 * ON THE GO ROUTINES
**************************************************************************/

/* utility routine */
static void
MGC_LinuxOtgInputChanged(MGC_LinuxCd * pThis, u8 devctl, u8 reset,
			 u8 connection, u8 suspend)
{
#ifdef CONFIG_USB_MUSB_OTG
	MGC_OtgMachine *otgm = &pThis->OtgMachine;
	MGC_OtgMachineInputs Inputs;

	DBG(2, "<==\n");

	/* reading suspend state from Power register does NOT work */
	memset(&Inputs, 0, sizeof(Inputs));

	Inputs.bSession = (devctl & MGC_M_DEVCTL_SESSION) ? TRUE : FALSE;
	Inputs.bSuspend = suspend;
	Inputs.bConnection = connection;
	Inputs.bReset = reset;
	Inputs.bConnectorId = (devctl & MGC_M_DEVCTL_BDEVICE) ? TRUE : FALSE;

	MGC_OtgMachineInputsChanged(otgm, &Inputs);
#endif
}

/*
 *
 * @param pThis
 * @param bVbusError
 * @param bConnect
 */
void MGC_OtgUpdate(MGC_LinuxCd * pThis, u8 bVbusError, u8 bConnect)
{
#ifdef CONFIG_USB_MUSB_OTG
	MGC_OtgMachineInputs Inputs;
	u8 *pBase = (u8 *) pThis->pRegs;
	u8 devctl = MGC_Read8(pBase, MGC_O_HDRC_DEVCTL);
	u8 power = MGC_Read8(pBase, MGC_O_HDRC_POWER);

	DBG(2, "<== Power=%02x, DevCtl=%02x, bVbusError=%d, bConnect=%d\n",
	    power, devctl, bVbusError, bConnect);

	/* speculative */
	memset(&Inputs, 0, sizeof(Inputs));
	Inputs.bSession = (devctl & MGC_M_DEVCTL_SESSION) ? TRUE : FALSE;
	Inputs.bConnectorId = (devctl & MGC_M_DEVCTL_BDEVICE) ? TRUE : FALSE;
	Inputs.bReset = (power & MGC_M_POWER_RESET) ? TRUE : FALSE;
	Inputs.bConnection = bConnect;
	Inputs.bVbusError = bVbusError;
	Inputs.bSuspend = (power & MGC_M_POWER_SUSPENDM) ? TRUE : FALSE;
	MGC_OtgMachineInputsChanged(&(pThis->OtgMachine), &Inputs);
#endif				/* CONFIG_USB_MUSB_OTG */
}

#ifdef CONFIG_USB_MUSB_OTG

/*
 * Change the role of the driver
 *
 * @param pPrivateData
 * @param bState the state to transition to
 */
static void MGC_LinuxOtgState(void *pPrivateData, u8 bState)
{
	MGC_LinuxCd *pThis = (MGC_LinuxCd *) pPrivateData;

	DBG(2, "<== New state=%d, current mode is %s\n", bState,
	    MUSB_MODE(pThis));
	switch (bState) {
	case MGC_OTG_STATE_A_HOST:
	case MGC_OTG_STATE_B_HOST:
		/* TODO: graceful Gadget shutdown */
		MUSB_HST_MODE(pThis);
		break;

	case MGC_OTG_STATE_A_PERIPH:
	case MGC_OTG_STATE_B_PERIPH:
		/* TODO: graceful host shutdown */
		MUSB_DEV_MODE(pThis);
		break;

	default:
		/* TODO: graceful host shutdown */
		/* TODO: graceful Gadget shutdown */
		MUSB_OTG_MODE(pThis);
		break;
	}
	DBG(2, "==> new mode is %s\n", MUSB_MODE(pThis));
}

static void MGC_LinuxOtgError(void *pPrivateData, u8 bError)
{
	MGC_LinuxCd *pThis = (MGC_LinuxCd *) pPrivateData;

	pThis->bOtgError = bError;
	switch (bError) {
	case MGC_OTG_ERROR_B_SRP_FAILED:
		ERR("SRP failed\n");
		break;
	case MGC_OTG_ERROR_NO_RESPONSE:
		ERR("Device not responding\n");
		break;
	}

	/* TODO: keep somewhere and have IOCTL or something? */
}

static void MGC_LinuxOtgSetSession(void *pPrivateData, u8 bSession)
{
	MGC_LinuxCd *pThis = (MGC_LinuxCd *) pPrivateData;
	u8 *pBase = (u8 *) pThis->pRegs;
	u8 devctl = MGC_Read8(pBase, MGC_O_HDRC_DEVCTL);

	DBG(2, "==>\n");

	if (bSession) {
		devctl |= MGC_M_DEVCTL_SESSION;
		MGC_Write8(pBase, MGC_O_HDRC_DEVCTL, devctl);
	} else if (MUSB_IS_DEV(pThis)) {
		pThis->pRootDevice = NULL;
	} else {
		devctl &= ~MGC_M_DEVCTL_SESSION;
		MGC_VirtualHubPortDisconnected(&pThis->RootHub, 0);
		pThis->pRootDevice = NULL;
		MGC_Write8(pBase, MGC_O_HDRC_DEVCTL, devctl);
	}

	DBG(2, "<==\n");
}

static void MGC_LinuxOtgSetHost(void *pPrivateData, u8 bHost)
{
	MGC_LinuxCd *pThis = (MGC_LinuxCd *) pPrivateData;
	u8 *pBase = (u8 *) pThis->pRegs;
	u8 devctl;

	DBG(2, "==>\n");
	devctl = MGC_Read8(pBase, MGC_O_HDRC_DEVCTL);

	if (bHost) {
		MGC_Write8(pBase, MGC_O_HDRC_DEVCTL, devctl | MGC_M_DEVCTL_HR);
	} else {
		MGC_Write8(pBase, MGC_O_HDRC_DEVCTL, devctl & ~MGC_M_DEVCTL_HR);
	}
	DBG(2, "<==\n");
}

static void MGC_LinuxOtgRequestSession(void *pPrivateData)
{
	MGC_LinuxCd *pThis = (MGC_LinuxCd *) pPrivateData;
	u8 *pBase = (u8 *) pThis->pRegs;
	u8 devctl;

	DBG(2, "==>\n");
	devctl = MGC_Read8(pBase, MGC_O_HDRC_DEVCTL);
	devctl |= MGC_M_DEVCTL_SESSION;
	MGC_Write8(pBase, MGC_O_HDRC_DEVCTL, devctl);
	DBG(2, "<==\n");
}

static void MGC_LinuxOtgSetSuspend(void *pPrivateData, u8 bSuspend)
{
	MGC_LinuxCd *pThis = (MGC_LinuxCd *) pPrivateData;
	u8 *pBase = (u8 *) pThis->pRegs;

	DBG(2, "==>\n");

	pThis->bDelayPortPowerOff = TRUE;

	MUSB_OTG_MODE(pThis);

	MGC_LinuxSetPortSuspend(pThis, 0, bSuspend);
	MGC_VirtualHubPortDisconnected(&pThis->RootHub, 0);
	MGC_Write8(pBase, MGC_O_HDRC_FADDR, 0);
	MGC_SelectEnd(pBase, 0);

	MGC_WriteCsr16(pBase, MGC_O_HDRC_CSR0, 0, MGC_M_CSR0_FLUSHFIFO);
	MGC_WriteCsr16(pBase, MGC_O_HDRC_CSR0, 0, MGC_M_CSR0_FLUSHFIFO);
	pThis->bEnd0Stage = MGC_END0_START;
}

/*
 * Redirect the call to the proper SubmitUrb function.
 * @param pPrivateData
 * @param pUrb
 */
static int MGC_LinuxOtgSubmitUrb(void *pPrivateData, struct urb *pUrb)
{

	return MGC_LinuxSubmitUrb(pUrb, 0);
}
#endif

/* --------------------------------------------------------------------------
 * HOST DMA related code
 *
 */

#if defined(CONFIG_USB_INVENTRA_DMA) || defined(CONFIG_USB_TI_CPPI_DMA)

/*
 * used ONLY in host mode, I'll be moved to musb_host
 * @param pPrivateData
 * @param bLocalEnd
 * @param bTransmit
 */
static u8 MGC_LinuxDmaChannelStatusChanged(void *pPrivateData, u8 bLocalEnd,
					   u8 bTransmit)
{
	MGC_LinuxCd *pThis = (MGC_LinuxCd *) pPrivateData;
	MGC_LinuxLocalEnd *pEnd = &(pThis->aLocalEnd[bLocalEnd]);
	struct urb *pUrb = MGC_GetCurrentUrb(pEnd);
	const void *pBase = pThis->pRegs;
	u8 devctl = MGC_Read8(pBase, MGC_O_HDRC_DEVCTL);

	if (!bLocalEnd) {
		/* endpoint 0 */
		if (devctl & MGC_M_DEVCTL_HM) {
#ifdef MUSB_CONFIG_PROC_FS
			if (pThis->pfDefaultEndHandler) {
				pThis->pfDefaultEndHandler(pThis->
							   pDefaultEndHandlerParam);
			} else
#endif
			{
#ifdef CONFIG_USB_MUSB_HDRC_HCD
				MGC_HdrcServiceDefaultEnd(pThis);
#endif
			}
		} else {
#ifdef CONFIG_USB_GADGET_MUSB_HDRC
			MGC_HdrcServiceDeviceDefaultEnd(pThis);
#endif
		}
	} else {
		/* endpoints 1..15 */
		if (bTransmit) {
			if (devctl & MGC_M_DEVCTL_HM) {
#ifdef CONFIG_USB_MUSB_HDRC_HCD
				MGC_HdrcServiceTxAvail(pThis, bLocalEnd);
#endif
			} else {
#ifdef CONFIG_USB_GADGET_MUSB_HDRC
				MGC_HdrcServiceDeviceTxAvail(pThis, bLocalEnd);
#endif
			}
		} else {
			/* receive */
			if (devctl & MGC_M_DEVCTL_HM) {
#ifdef CONFIG_USB_MUSB_HDRC_HCD
				MGC_HdrcServiceRxReady(pThis, bLocalEnd);
#endif
			} else {
#ifdef CONFIG_USB_GADGET_MUSB_HDRC
				MGC_HdrcServiceDeviceRxReady(pThis, bLocalEnd);
#endif
			}
		}
	}

	/* trick: if end's URB changed; previous one completed;
	 * probably not needed now... */
	return (pUrb == MGC_GetCurrentUrb(pEnd)) ? FALSE : TRUE;
}
#endif

/* --------------------------------------------------------------------------
 * Init support
 */

static MGC_LinuxCd *allocate_instance(void)
{
	struct musb		*pThis;
	struct musb_hw_ep	*ep;
	int			epnum;

	/* allocate */
	pThis = kzalloc(sizeof *pThis, GFP_KERNEL);
	if (!pThis)
		return NULL;

#if MUSB_DEBUG > 0
	pThis->dwPadFront = MGC_PAD_FRONT;
	pThis->dwPadBack = MGC_PAD_BACK;
#endif

	for (epnum = 0, ep = pThis->aLocalEnd;
			epnum < MUSB_C_NUM_EPS;
			epnum++, ep++) {

		ep->bLocalEnd = epnum;
		spin_lock_init(&ep->Lock);
		INIT_LIST_HEAD(&(ep->urb_list));
		ep->musb = pThis;

#if MUSB_DEBUG > 0
		ep->dwPadFront = MGC_PAD_FRONT;
		ep->dwPadBack = MGC_PAD_BACK;
#endif
	}
	return pThis;
}

static void free_instance(MGC_LinuxCd * pThis)
{
	kfree(pThis);
}

/*
 * Perform generic per-controller initialization.
 *
 * @param pDevice the controller (already clocked, etc)
 * @param nIrq IRQ (interpretation is system-dependent)
 * @param pRegs pointer to controller registers,
 *  assumed already mapped into kernel space
 * @param pName name for bus
 */
static MGC_LinuxCd *__init
MGC_LinuxInitController(struct device *pDevice,
					    int nIrq, void *pRegs,
					    const char *pName)
{
	int status = -EINVAL;
	u8 bEnd;
	struct musb		*pThis;
	struct musb_hw_ep	*pEnd;
	char buf[8], *bufp = buf;
	struct musb_hdrc_platform_data *plat = pDevice->platform_data;

	/* The driver might handle more features than the board; OK.
	 * Fail when the board needs a feature that's not enabled.
	 */
	if (!plat) {
		dev_dbg(pDevice, "no platform_data?\n");
		return NULL;
	}
	switch (plat->mode) {
	case MUSB_HOST:
#ifdef CONFIG_USB_MUSB_HDRC_HCD
		break;
#else
		goto bad_config;
#endif
	case MUSB_PERIPHERAL:
#ifdef CONFIG_USB_GADGET_MUSB_HDRC
		break;
#else
		goto bad_config;
#endif
	case MUSB_OTG:
#ifdef CONFIG_USB_MUSB_OTG
		break;
#else
	      bad_config:
#endif
	default:
		dev_dbg(pDevice, "incompatible Kconfig role setting\n");
		return NULL;
	}

	/* assume vbus is off, and we'll turn it on later */

#ifdef CONFIG_ARCH_DAVINCI
	{
		void *__iomem regBase = pRegs - 0x400;

		// REVISIT assumes the module clock is on already,
		// with this module in the "enabled" state.

		/* reset the controller */
		MGC_Write32(regBase, USB_OTG_CTRL_REG, 0x1);

		// ... do we need the PHY clock so soon, or
		// can we keep OSCPDWN until session start?
		// keep PHYSPDWN until we need ID sensing?

		/* start the on-chip PHY and its PLL */
		MGC_Write32(IO_ADDRESS(USBPHY_CTL_PADDR), 0x00,
				USBPHY_SESNDEN | USBPHY_VBDTCTEN
				| USBPHY_PHYPLLON);

		/* wait till USBPHY_PHYCLKGD and/or reset to finish */
		msleep(5);

#if 0
		/* bypass the TI PDR IRQ handling? */
		MGC_Write8(regBase, USB_OTG_CTRL_REG, 0x8);
#endif

		pr_debug("DaVinci OTG revision %08x phy %03x control %02x\n",
			 MGC_Read32(regBase, USB_OTG_VERSION_REG),
			 MGC_Read32(IO_ADDRESS(USBPHY_CTL_PADDR), 0x00),
			 MGC_Read8(regBase, USB_OTG_CTRL_REG));
	}
#endif

#ifndef __sparc__
	snprintf(bufp, 8, "%d", nIrq);
#else
	bufp = __irq_itoa(nIrq);
#endif

	/* allocate */
	pThis = allocate_instance();
	if (!pThis)
		return NULL;

	pThis->controller = pDevice;
	dev_set_drvdata(pDevice, pThis);

	strcpy(pThis->aName, pName);
	pThis->pRegs = pRegs;
	spin_lock_init(&pThis->Lock);

	pThis->board_mode = plat->mode;

	pr_info("%s: USB %s mode controller at %p, IRQ %s\n",
			musb_driver_name,
			({char *s;
			switch (pThis->board_mode) {
			case MUSB_HOST:		s = "Host"; break;
			case MUSB_PERIPHERAL:	s = "Peripheral"; break;
			default:		s = "OTG"; break;
			}; s; }),
			pRegs, bufp);

#ifdef CONFIG_USB_INVENTRA_DMA
	pThis->pDmaController = MGC_HdrcDmaControllerFactory
			.pfNewDmaController(MGC_LinuxDmaChannelStatusChanged,
					pThis, (u8 *) pRegs);
	if (pThis->pDmaController) {
		DBG(2, "Inventra DMA enabled\n");
		pThis->pDmaController->pfDmaStartController(
				pThis->pDmaController->pPrivateData);
	}
#elif defined(CONFIG_USB_TI_CPPI_DMA)
	pThis->pDmaController = MGC_CppiDmaControllerFactory
			.pfNewDmaController(MGC_LinuxDmaChannelStatusChanged,
					pThis, (u8 *) pRegs);
	if (pThis->pDmaController) {
		DBG(2, "CPPI DMA enabled\n");
		pThis->pDmaController->pfDmaStartController(
				pThis->pDmaController->pPrivateData);
	}
#endif


#ifdef CONFIG_USB_INVENTRA_FIFO
/* ideally the setting should be abstracted in arch.c itself? */
pDevice->dma_mask = 0;
#endif

	/* be sure interrupts are disabled before connecting ISR */
	MGC_HdrcDisable(pThis);

	/* discover configuration */
	if (!MGC_HdrcInit(plat->multipoint
			   ? MUSB_CONTROLLER_MHDRC
			   : MUSB_CONTROLLER_HDRC, pThis)) {
		free_instance(pThis);
		return NULL;
	}

	/* print config */
	for (bEnd = 0; bEnd < pThis->bEndCount; bEnd++) {
		pEnd = &(pThis->aLocalEnd[bEnd]);
		if (pEnd->wMaxPacketSizeTx || pEnd->wMaxPacketSizeRx) {
			pr_info("%s: End %02d, %sTxSize=%04x/RxSize=%04x\n",
				musb_driver_name, bEnd,
				pEnd->bIsSharedFifo ? "Shared " : "",
				pEnd->wMaxPacketSizeTx,
				pEnd->wMaxPacketSizeRx);
		} else
			DBG(1, "End %02d: not configured\n", bEnd);
	}

	/* connect ISR */
#ifdef CONFIG_USB_INVENTRA_HCD_POLLING
	MGC_nPollerPid = kernel_thread(MGC_LinuxPoller, pThis,
				       CLONE_FS | CLONE_FILES | CLONE_SIGHAND);
	if (MGC_nPollerPid >= 0) {
		pThis->nIrq = 0;
	} else {
		free_instance(pThis);
		return NULL;
	}
#endif

	/* attach to the IRQ */
	if (request_irq (nIrq, MGC_LinuxIsr, 0 /*SA_SHIRQ */,
				pThis->aName, pThis)) {
		dev_err(pDevice, "request_irq %d failed!\n", nIrq);
		free_instance(pThis);
		return NULL;
	}

	pThis->nIrq = nIrq;

// FIXME:
//  - convert to the HCD framework
//  - if (board_mode == MUSB_OTG) do startup with peripheral
//  - ... involves refcounting updates

#ifdef CONFIG_USB_MUSB_HDRC_HCD
	/* host side needs more setup, except for no-host modes */
	if (pThis->board_mode != MUSB_PERIPHERAL) {
		/* allocate and register bus */
		pThis->pBus = usb_alloc_bus(&MGC_LinuxOperations);
		if (!pThis->pBus) {
			dev_dbg(pDevice, "usb_alloc_bus fail\n");
			free_irq(nIrq, pThis);
			free_instance(pThis);
			return NULL;
		}

		/* register the bus */
		pThis->pBus->controller = (struct device *)pDevice;
		pThis->pBus->bus_name = pThis->aName;
		if (pThis->board_mode == MUSB_OTG)
			pThis->pBus->otg_port = 1;
		pThis->pBus->hcpriv = (void *)pThis;

		/* FIXME:  hcd framework allocates the bus ...
		 * else bus->release(bus) method looks necessary
		 */

		status = usb_register_bus(pThis->pBus);
		// FIXME handle errors

		/* init virtual root hub */
		pThis->PortServices.pPrivateData = pThis;
		pThis->PortServices.pfSetPortPower = MGC_LinuxSetPortPower;
		pThis->PortServices.pfSetPortEnable = MGC_LinuxSetPortEnable;
		pThis->PortServices.pfSetPortSuspend = MGC_LinuxSetPortSuspend;
		pThis->PortServices.pfSetPortReset = MGC_LinuxSetPortReset;

		if (!MGC_VirtualHubInit(&pThis->RootHub, pThis->pBus, 1,
					 &pThis->PortServices)) {
			dev_dbg(pDevice, "Virtual Hub init failed\n");
			usb_deregister_bus(pThis->pBus);
			// kfree(bus)
			free_irq(nIrq, pThis);
			free_instance(pThis);
			return NULL;
		}
	}
#endif				/* CONFIG_USB_MUSB_HDRC_HCD */

	/* For the host-only role, we can activate right away.
	 * Otherwise, wait till the gadget driver hooks up.
	 *
	 * REVISIT switch to compile-time is_role_host() etc
	 * to get rid of #ifdeffery
	 */
	switch (pThis->board_mode) {
#ifdef CONFIG_USB_MUSB_HDRC_HCD
	case MUSB_HOST:
		MUSB_HST_MODE(pThis);
		status = usb_register_root_hub(pThis->RootHub.pDevice, pDevice);

#if 0
		/* FIXME 2.6.10 doesn't do root hub power budgets
		 * correctly for the EVM...
		 */
		/* can only report the power budget after hub driver binds */
		if (status == 0)
			hub_set_power_budget(pThis->RootHub.pDevice,
					     2 * (plat->power ? : 250));
#endif

		DBG(1, "%s mode, status %d, dev%02x%s\n",
			"HOST", status,
			MGC_Read8(pThis->pRegs, MGC_O_HDRC_DEVCTL),
			(MGC_Read8(pThis->pRegs, MGC_O_HDRC_DEVCTL)
					& MGC_M_DEVCTL_BDEVICE
				? "" : " (TROUBLE)"));
		break;
#endif
#ifdef CONFIG_USB_GADGET_MUSB_HDRC
	case MUSB_PERIPHERAL:
		MUSB_DEV_MODE(pThis);
		status = musb_gadget_setup(pThis);

		DBG(1, "%s mode, status %d, dev%02x\n",
			"PERIPHERAL", status,
			MGC_Read8(pThis->pRegs, MGC_O_HDRC_DEVCTL));
		break;
#endif
#ifdef CONFIG_USB_MUSB_OTG
	case MUSB_OTG:
		pThis->OtgServices.pPrivateData = pThis;
		pThis->OtgServices.pfOtgError = MGC_LinuxOtgError;
		pThis->OtgServices.pfOtgState = MGC_LinuxOtgState;
		pThis->OtgServices.pfOtgSetSession = MGC_LinuxOtgSetSession;
		pThis->OtgServices.pfOtgSetHost = MGC_LinuxOtgSetHost;
		pThis->OtgServices.pfOtgRequestSession =
		    MGC_LinuxOtgRequestSession;
		pThis->OtgServices.pfOtgSetSuspend = MGC_LinuxOtgSetSuspend;
		pThis->OtgServices.pfOtgSubmitUrb = MGC_LinuxOtgSubmitUrb;
		MGC_OtgMachineInit(&(pThis->OtgMachine), &(pThis->OtgServices));

		MUSB_OTG_MODE(pThis);
		status = musb_gadget_setup(pThis);

		DBG(1, "%s mode, status %d, dev%02x\n",
			"OTG", status,
			MGC_Read8(pThis->pRegs, MGC_O_HDRC_DEVCTL));
#endif
		break;
	}

	/* FIXME on error, clean up ... */

	if (status == 0)
		MGC_LinuxCreateProcFs("driver/musb_hdrc", pThis);

	return pThis;
}

/*
 * Release resources acquired by driver
 */
static void MGC_LinuxCdFree(MGC_LinuxCd * pThis)
{
	DBG(2, "<==\n");

	MGC_HdrcStop(pThis);
	MUSB_ERR_MODE(pThis, MUSB_ERR_SHUTDOWN);

#ifdef MUSB_CONFIG_PROC_FS
	MGC_LinuxDeleteProcFs(pThis);
#endif

#ifndef CONFIG_USB_INVENTRA_FIFO
	if (pThis->pDmaController) {
		pThis->pDmaController->pfDmaStopController(pThis->
							   pDmaController->
							   pPrivateData);
#if defined(CONFIG_USB_INVENTRA_DMA)
		MGC_HdrcDmaControllerFactory
#elif defined(CONFIG_USB_TI_CPPI_DMA)
		MGC_CppiDmaControllerFactory
#endif
		    .pfDestroyDmaController(pThis->pDmaController);
	}
#endif

#ifdef CONFIG_USB_MUSB_HDRC_HCD
	MGC_VirtualHubStop(&pThis->RootHub);
	MGC_VirtualHubDestroy(&pThis->RootHub);
#endif

#ifdef MUSB_POLL
	/* Kill the thread */
	kill_proc(MGC_nPollerPid, SIGTERM, 1);
	wait_for_completion(&MGC_PollerExited);
#endif

	msleep(1);

	if (pThis->nIrq) {
		free_irq(pThis->nIrq, pThis);
	}
#ifdef CONFIG_USB_MUSB_HDRC_HCD
	if (pThis->pBus->root_hub) {
		printk("FIXME -- %s usb_deregister_bus()\n", __FUNCTION__);
		usb_disconnect(&(pThis->pBus->root_hub));
	}
	usb_deregister_bus(pThis->pBus);
	// usb_free_bus (pThis->pBus);
#endif

#ifdef CONFIG_ARCH_DAVINCI
	/* powerdown the on-chip PHY and its oscillator */
	MGC_Write32(IO_ADDRESS(USBPHY_CTL_PADDR), 0x00,
			USBPHY_OSCPDWN | USBPHY_PHYSPDWN);

	// REVISIT interface and function clocks turn off here
#endif

	/* FIXME make sure all the different faces of this driver
	 * coordinate their refcounting, so the same release() is
	 * called when the host or gadget (or whatever) is the last
	 * one released
	 */

	// KFREE(pThis);

	DBG(2, "(not freed) ==>\n");
}

/*-------------------------------------------------------------------------*/

/* all implementations (PCI bridge to FPGA, VLYNQ, etc) should just
 * bridge to a platform device; this driver then suffices.
 */

static int __init musb_probe(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	int irq = platform_get_irq(pdev, 0);
	struct resource *iomem;
	MGC_LinuxCd *musb;

	iomem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!iomem || irq == 0)
		return -ENODEV;

	/* REVISIT this init routine should just return status.
	 * Plus, MENTOR_BASE_OFFSET is nonportable; and we're
	 * not guaranteed that a static I/O mapping will exist.
	 */
	musb = MGC_LinuxInitController(dev, irq, (void __iomem *)
				       IO_ADDRESS(iomem->start +
						  MENTOR_BASE_OFFSET),
				       dev->bus_id);
	if (!musb)
		return -ERANGE;
	return 0;
}

static int __exit musb_remove(struct device *dev)
{
	MGC_LinuxCdFree(dev_get_drvdata(dev));
	return 0;
}

static struct device_driver musb_driver = {
	.name = (char *)musb_driver_name,
	.bus = &platform_bus_type,
	.owner = THIS_MODULE,
	.probe = musb_probe,
	.remove = __exit_p(musb_remove),
	// suspend, resume
	// shutdown
};

/*-------------------------------------------------------------------------*/

static int __init MGC_ModuleInit(void)
{
#ifdef CONFIG_USB_MUSB_HDRC_HCD
	if (usb_disabled())
		return 0;
#endif

	pr_info("%s: version " MUSB_VERSION " "
#ifdef CONFIG_USB_INVENTRA_FIFO
	       "[pio]"
#elif defined(CONFIG_USB_TI_CPPI_DMA)
	       "[cppi-DMA]"
#elif defined(CONFIG_USB_INVENTRA_DMA)
	       "[musb-DMA]"
#else
	       "[?]"
#endif
	       " "
#ifdef CONFIG_USB_MUSB_OTG
		"[OTG: peripheral+host]"
#elif defined(CONFIG_USB_GADGET_MUSB_HDRC)
		"[peripheral]"
#elif defined(CONFIG_USB_MUSB_HDRC_HCD)
		"[host]"
#endif
	       " [debug=%d]\n",
	       musb_driver_name, MGC_GetDebugLevel());

	return driver_register(&musb_driver);
}

device_initcall(MGC_ModuleInit);

#if 0
/* not yet safe to rmmod */
static void __exit MGC_ModuleCleanup(void)
{
	driver_unregister(&musb_driver);
}

module_exit(MGC_ModuleCleanup);
#endif
