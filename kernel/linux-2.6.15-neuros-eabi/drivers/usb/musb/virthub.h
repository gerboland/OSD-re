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
 * Definitions for a virtual hub intended for embedding in HCDs
 */

#ifndef __MUSB_LINUX_VIRTUALHUB_H__
#define __MUSB_LINUX_VIRTUALHUB_H__

#include <linux/spinlock.h>
#include <linux/timer.h>

struct urb;
struct usb_bus;

/**
 * Introduction.
 * For USB controllers lacking embedded root hubs,
 * this module can be used as a virtual root hub,
 * with one or more controllers as the virtual hub's ports.
 */

/****************************** CONSTANTS ********************************/

/** Maximum number of ports to accomodate */
#define MGC_VIRTUALHUB_MAX_PORTS	7

/******************************** TYPES **********************************/

/**
 * Set a port's power on or off.
 * @param pPrivateData pPrivateData from port services
 * @param bPortIndex 0-based index of port
 * @param bPower TRUE to power on the port; FALSE to power off
 */
typedef void (*MGC_pfSetPortPower) (void *pPrivateData, u8 bPortIndex,
				    u8 bPower);

/**
 * Enable or disable a port.
 * @param pPrivateData pPrivateData from port services
 * @param bPortIndex 0-based index of port
 * @param bEnable TRUE to enable port; FALSE to disable
 */
typedef void (*MGC_pfSetPortEnable) (void *pPrivateData, u8 bPortIndex,
				     u8 bEnable);

/**
 * Set a port's suspend mode on or off.
 * @param pPrivateData pPrivateData from port services
 * @param bPortIndex 0-based index of port
 * @param bSuspend TRUE to suspend port; FALSE to resume
 */
typedef void (*MGC_pfSetPortSuspend) (void *pPrivateData, u8 bPortIndex,
				      u8 bSuspend);

/**
 * Set a port's reset on or off.
 * @param pPrivateData pPrivateData from port services
 * @param bPortIndex 0-based index of port
 * @param bReset TRUE to assert reset on the bus behind a port; FALSE to deassert
 */
typedef void (*MGC_pfSetPortReset) (void *pPrivateData, u8 bPortIndex,
				    u8 bReset);

/**
 * MGC_PortServices.
 * Services provided to a virtual by a USB port controller.
 * @field pPrivateData port controller's implementation data;
 * not to be interpreted by virtual hub
 * @param pfSetPortPower set-port-power call
 * @param pfSetPortEnable set-port-enable call
 * @param pfSetPortSuspend set-port-suspend call
 * @param pfSetPortReset set-port-reset call
 */
struct port_services {
	void *pPrivateData;
	MGC_pfSetPortPower pfSetPortPower;
	MGC_pfSetPortEnable pfSetPortEnable;
	MGC_pfSetPortSuspend pfSetPortSuspend;
	MGC_pfSetPortReset pfSetPortReset;
};
typedef struct port_services MGC_PortServices;

/**
 * MGC_HubPortStatusChange.
 * @field wStatus status
 * @field wChange change
 */
typedef struct {
	u16 wStatus;
	u16 wChange;
} MGC_HubPortStatusChange;

/**
 * MGC_VirtualHub.
 * Virtual USB hub instance data.
 * @field Lock spinlock
 * @field pBus our bus pointer
 * @field pDevice our device pointer
 * @field pUrb pointer to interrupt URB for status change
 * @field pPortServices pointer to port services
 * @field Timer interval timer for status change interrupts
 * @field aPortStatusChange status/change array
 * @field bPortCount how many ports
 * @field wInterval actual interval in milliseconds
 * @field bIsChanged TRUE if changes to report
 * @field bAddress address assigned by usbcore
 */
struct virtual_root {
	spinlock_t Lock;
	struct usb_bus *pBus;
	struct usb_device *pDevice;
	void *pUrb;
	MGC_PortServices *pPortServices;
	struct timer_list Timer;
	MGC_HubPortStatusChange aPortStatusChange[MGC_VIRTUALHUB_MAX_PORTS];
	u8 bPortCount;
	u16 wInterval;
	u8 bIsChanged;
	u8 bAddress;
};
typedef struct virtual_root MGC_VirtualHub;

/****************************** FUNCTIONS ********************************/

void MGC_LinuxSetPortPower(void *pPrivateData, u8 bPortIndex, u8 bPower);
void MGC_LinuxSetPortEnable(void *pPrivateData, u8 bPortIndex, u8 bEnable);
void MGC_LinuxSetPortSuspend(void *pPrivateData, u8 bPortIndex, u8 bSuspend);
void MGC_LinuxSetPortReset(void *pPrivateData, u8 bPortIndex, u8 bReset);

/**
 * Initialize a virtual hub.
 * @param pHub hub struct pointer; struct filled on success
 * @param pDevice pointer to bus
 * @param bPortCount how many ports to support
 * @param pPortServices port services
 * @return TRUE on success
 * @return FALSE on failure
 */
extern u8 MGC_VirtualHubInit(MGC_VirtualHub * pHub,
			     struct usb_bus *pBus,
			     u8 bPortCount, MGC_PortServices * pPortServices);

/**
 * Destroy a virtual hub
 */
extern void MGC_VirtualHubDestroy(MGC_VirtualHub * pHub);

/**
 * Stop a virtual hub
 */
extern void MGC_VirtualHubStop(MGC_VirtualHub * pHub);

/**
 * Submit an URB to a virtual hub.
 * @param pHub pointer to hub initialized by successful MGC_VirtualHubInit
 * @param pUrb URB pointer
 * @return Linux status code
 * @see #MGC_VirtualHubInit
 */
extern int MGC_VirtualHubSubmitUrb(MGC_VirtualHub * pHub, struct urb *pUrb);

/**
 * Unlink an URB from a virtual hub.
 * @param pHub pointer to hub initialized by successful MGC_VirtualHubInit
 * @param pUrb URB pointer
 * @return Linux status code
 * @see #MGC_VirtualHubInit
 */
extern int MGC_VirtualHubUnlinkUrb(MGC_VirtualHub * pHub, struct urb *pUrb);

/**
 * A port reset is complete
 * @param pHub pointer to hub initialized by successful MGC_VirtualHubInit
 * @param bPortIndex 0-based index of port
 * @see #MGC_VirtualHubInit
 */
extern void MGC_VirtualHubPortResetDone(MGC_VirtualHub * pHub,
					u8 bPortIndex, u8 bHubSpeed);

/**
 * A device has effectively been connected to a virtual hub port
 * @param pHub pointer to hub initialized by successful MGC_VirtualHubInit
 * @param bPortIndex 0-based index of port with connected device
 * @param bSpeed device speed (0=>low, 1=>full, 2=>high)
 * @see #MGC_VirtualHubInit
 */
extern void MGC_VirtualHubPortConnected(MGC_VirtualHub * pHub,
					u8 bPortIndex, u8 bSpeed);

/**
 * A device has effectively resumed a virtual hub port
 * @param pHub pointer to hub initialized by successful MGC_VirtualHubInit
 * @param bPortIndex 0-based index of port of resume
 * @see #MGC_VirtualHubInit
 */
extern void MGC_VirtualHubPortResumed(MGC_VirtualHub * pHub, u8 bPortIndex);

/**
 * A device has effectively been disconnected from a virtual hub port
 * @param pHub pointer to hub initialized by successful MGC_VirtualHubInit
 * @param bPortIndex 0-based index of port of disconnected device
 * @see #MGC_VirtualHubInit
 */
extern void MGC_VirtualHubPortDisconnected(MGC_VirtualHub * pHub,
					   u8 bPortIndex);

#endif				/* multiple inclusion protection */
