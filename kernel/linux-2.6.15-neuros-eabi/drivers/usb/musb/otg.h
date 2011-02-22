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
 * Interface to a generic OTG state machine for use by an OTG controller.
 */

#ifndef __MUSB_LINUX_OTG_H__
#define __MUSB_LINUX_OTG_H__

#include <linux/spinlock.h>
#include <linux/timer.h>

struct urb;
struct usb_device;

/**
 * Introduction.
 * An OTG state machine for use by a controller driver for an OTG controller
 * that wishes to be OTG-aware.
 * The state machine requires relevant inputs and a couple of services 
 * from the controller driver, and calls the controller driver to inform
 * it of the current state and errors.
 * Finally, it provides the necessary bus control service.
 */

/****************************** CONSTANTS ********************************/

/** 
 * Define this (in milliseconds) to a target-specific value to override default.
 * The OTG-spec minimum is 5000, and maximum is 6000 (see OTG spec errata).
 */
#ifndef MGC_OTG_T_B_SRP_FAIL
#define MGC_OTG_T_B_SRP_FAIL	5000
#endif

/** 
 * Define this (in milliseconds) to a target-specific value to override default.
 * This is the time an A-device should wait for a B-device to connect.
 * The OTG-spec minimum is 1000.
 * As a special case, for normal host-like behavior, you can set this to 0.
 */
#ifndef MGC_OTG_T_A_WAIT_BCON
#define MGC_OTG_T_A_WAIT_BCON	1000
#endif

/** 
 * Define this (in milliseconds) to a target-specific value to override default.
 * The OTG-spec minimum is 250.
 */
#ifndef MGC_OTG_T_AIDL_BDIS
#define MGC_OTG_T_AIDL_BDIS	250
#endif

//#define MGC_OTG_T_B_ASE0_BRST 4
#define MGC_OTG_T_B_ASE0_BRST	100

/**
 * MGC_OtgRequest.
 * A software request for the OTG state machine
 */
typedef enum {
	MGC_OTG_REQUEST_UNKNOWN,
    /** Request the bus */
	MGC_OTG_REQUEST_START_BUS,
    /** Drop the bus */
	MGC_OTG_REQUEST_DROP_BUS,
    /** Suspend the bus */
	MGC_OTG_REQUEST_SUSPEND_BUS,
    /** Reset the state machine */
	MGC_OTG_REQUEST_RESET
} MGC_OtgRequest;

/**
 * MGC_OtgState.
 * OTG state
 */
typedef enum {
    /** Idle, regardless of connector ID */
	MGC_OTG_STATE_AB_IDLE,
    /** B-device initiating SRP with A-device */
	MGC_OTG_STATE_B_SRP_INIT,
    /** B-device is peripheral */
	MGC_OTG_STATE_B_PERIPH,
    /** B-device waiting for A-device to connect (as peripheral) */
	MGC_OTG_STATE_B_WAIT_ACON,
    /** B-device hosting A-device */
	MGC_OTG_STATE_B_HOST,
    /** A-device waiting for B-device to connect (as peripheral) */
	MGC_OTG_STATE_A_WAIT_BCON,
    /** A-device hosting B-device */
	MGC_OTG_STATE_A_HOST,
    /** A-device suspended bus due to software request */
	MGC_OTG_STATE_A_SUSPEND,
    /** A-device is peripheral */
	MGC_OTG_STATE_A_PERIPH
} MGC_OtgState;

/**
 * MGC_OtgError.
 * OTG error
 */
typedef enum {
    /** No error */
	MGC_OTG_ERROR_NONE,
    /** B-device saw no response from A-device */
	MGC_OTG_ERROR_B_SRP_FAILED,
    /** A-device saw no connecting B-device */
	MGC_OTG_ERROR_NO_RESPONSE,
    /** Device not supported */
	MGC_OTG_ERROR_UNSUPPORTED_DEVICE,
    /** Hubs not supported */
	MGC_OTG_ERROR_UNSUPPORTED_HUB
} MGC_OtgError;

/******************************** TYPES **********************************/

/**
 * MGC_OtgMachineInputs.
 * The set of inputs which drives the state machine
 * @field bSession TRUE when a session is in progress; FALSE when not
 * @field bConnectorId TRUE for B-device; FALSE for A-device
 * (assumed valid only when a bSession is TRUE)
 * @field bReset TRUE when reset is detected (peripheral role only)
 * @field bConnection TRUE when connection is detected (host role only)
 * @field bSuspend TRUE when bus suspend is detected 
 * @field bVbusError TRUE when a Vbus error is detected
 */
typedef struct {
	u8 bSession;
	u8 bConnectorId;
	u8 bReset;
	u8 bConnection;
	u8 bSuspend;
	u8 bVbusError;
} MGC_OtgMachineInputs;

/**
 * An OTG error has occurred.
 * This is implemented by the controller driver.
 * For OTG certification, appropriate messages must be displayed.
 * @param pPrivateData the service provider's pPrivateData
 * @param bError one of the MGC_OTG_ERROR_* constants
 */
typedef void (*MGC_pfOtgError) (void *pPrivateData, u8 bError);

/**
 * A new OTG state has been entered.
 * This is implemented by the controller driver.
 * @param pPrivateData the service provider's pPrivateData
 * @param bState one of the MGC_OTG_STATE_* constants
 */
typedef void (*MGC_pfOtgState) (void *pPrivateData, u8 bState);

/**
 * Request the controller driver to start/stop a session.
 * This is implemented by the controller driver.
 * @param pPrivateData the service provider's pPrivateData
 * @param bSession TRUE to request a session if possible;
 * FALSE to avoid a session if possible
 */
typedef void (*MGC_pfOtgSetSession) (void *pPrivateData, u8 bSession);

/**
 * Request the controller driver to enter host mode when possible.
 * This is implemented by the controller driver.
 * @param pPrivateData the service provider's pPrivateData
 * @param bHost TRUE to enter host mode when possible;
 * FALSE to avoid entering host mode if possible
 */
typedef void (*MGC_pfOtgSetHost) (void *pPrivateData, u8 bHost);

/**
 * Request the controller driver to initiate SRP.
 * This is implemented by the controller driver.
 * @param pPrivateData the service provider's pPrivateData
 */
typedef void (*MGC_pfOtgRequestSession) (void *pPrivateData);

/**
 * Request the controller driver to set the suspend state of the bus.
 * This is implemented by the controller driver.
 * The OTG machine calls this only under appropriate conditions
 * as the A-device.
 * @param pPrivateData the service provider's pPrivateData
 * @param bSuspend TRUE to suspend bus; FALSE to resume it
 */
typedef void (*MGC_pfOtgSetSuspend) (void *pPrivateData, u8 bSuspend);

/**
 * Submit an URB to the controller driver.
 * This is implemented by the controller driver.
 * This is used to perform one or more SET_FEATURE requests
 * on the directly-connected device if appropriate,
 * when MGC_OtgMachineRootDeviceEnumerated is called
 * @param pPrivateData the service provider's pPrivateData
 * @param pUrb URB pointer
 * @return status code
 * @see #MGC_OtgMachineRootDeviceAddressed
 */
typedef int (*MGC_pfOtgSubmitUrb) (void *pPrivateData, struct urb * pUrb);

/**
 * MGC_OtgServices.
 * Services an OTG-capable controller driver provides to the OTG state machine
 * @field pPrivateData instance data for callbacks
 * @field pfOtgError callback to notify controller driver of OTG errors
 * @field pfOtgState callback to notify controller driver of an OTG state change
 * @field pfOtgSetSession callback to request controller driver to start/stop session
 * @field pfOtgRequestSession callback to request controller driver to initiate SRP
 * @field pfOtgSetSuspend callback to request controller driver to set suspend state
 * @field pfOtgSubmitUrb callback to submit URB to controller driver
 */
typedef struct {
	void *pPrivateData;
	MGC_pfOtgError pfOtgError;
	MGC_pfOtgState pfOtgState;
	MGC_pfOtgSetSession pfOtgSetSession;
	MGC_pfOtgSetHost pfOtgSetHost;
	MGC_pfOtgRequestSession pfOtgRequestSession;
	MGC_pfOtgSetSuspend pfOtgSetSuspend;
	MGC_pfOtgSubmitUrb pfOtgSubmitUrb;
} MGC_OtgServices;

/**
 * MGC_OtgMachine.
 * OTG state machine instance data.
 * @field Lock spinlock
 * @field bState current state (one of the MGC_OTG_STATE_* constants)
 * @field pOtgServices pointer to OTG services
 * @field Timer interval timer for status change interrupts
 * @field bState current state
 * @field bRequest current pending request
 * @field bHnpSupported TRUE if HNP is supported on A-device
 * @field bHnpEnabled TRUE if A-device enabled us to HNP
 * @field bSetHnpEnable TRUE if we apparently successfully enabled B for HNP
 * @field bRejecting TRUE if rejection in progress
 */
typedef struct {
	spinlock_t Lock;
	MGC_OtgServices *pOtgServices;
	struct timer_list Timer;
	struct urb *pUrb;
	u8 bState;
	u8 bRequest;
	u8 bHnpSupported;
	u8 bHnpEnabled;
	u8 bSetHnpEnable;
	u8 bRejecting;

	/* check tpl */
	 u8(*pCheckDevice) (struct usb_device * pDevice);

} MGC_OtgMachine;

/****************************** FUNCTIONS ********************************/

/**
 * Initialize an OTG state machine.
 * @param pMachine struct pointer; struct filled on success
 * @param pOtgServices OTG services
 * @return TRUE on success
 * @return FALSE on failure
 */
extern u8 MGC_OtgMachineInit(MGC_OtgMachine * pMachine,
			     MGC_OtgServices * pOtgServices);

/**
 * Destroy an OTG state machine
 * @param pMachine machine pointer
 * @see #MGC_OtgMachineInit
 */
extern void MGC_OtgMachineDestroy(MGC_OtgMachine * pMachine);

/**
 * OTG inputs have changed.
 * A controller driver calls this when anything in the 
 * MGC_OtgMachineInputs has changed
 * @param pMachine machine pointer
 * @param pInputs current inputs
 * @see #MGC_OtgMachineInit
 */
extern void MGC_OtgMachineInputsChanged(MGC_OtgMachine * pMachine,
					const MGC_OtgMachineInputs * pInputs);

/**
 * A controller driver calls this when the root device has been enumerated
 * (all descriptors read, but NOT yet configured).
 * This is the OTG machine's opportunity to perform SET_FEATURE(b_hnp_enable)
 * or SET_FEATURE(a_alt_hnp_support),
 * or to reject the device.
 * @param pMachine machine pointer
 * @param pDevice root device
 * @param bHasOtgDescriptor TRUE if the device's first configuration
 * has an OTG descriptor; FALSE otherwise
 * @param bSupportsHubs TRUE if hubs are supported by the controller;
 * FALSE if not
 * @param bPortSupportsHnp TRUE if the port to which the root device is 
 * connected supports HNP; FALSE otherwise
 * @param bOtherPortSupportsHnp if bPortSupportsHnp is FALSE, 
 * TRUE here means there is another port that does support HNP
 * @see #MGC_OtgMachineInit
 * @return TRUE to continue normally
 * @return FALSE to reject subsequent URBs because the port is being
 * shut down per OTG rules
 */
extern u8 MGC_OtgMachineRootDeviceEnumerated(MGC_OtgMachine * pMachine,
					     struct usb_device *pDevice,
					     u8 bHasOtgDescriptor,
					     u8 bSupportsHubs,
					     u8 bPortSupportsHnp,
					     u8 bOtherPortSupportsHnp);

/**
 * The Gadget received an OTG-relevant SET_FEATURE.
 * A Gadget should call this for proper OTG behavior.
 * @param pMachine machine pointer
 * @param bFeatureSelector the feature selector
 * @see #MGC_OtgMachineInit
 */
extern void MGC_OtgMachineSetFeature(MGC_OtgMachine * pMachine,
				     u16 wFeatureSelector);

/**
 * Issue an OTG request
 * @param pMachine machine pointer
 * @param bRequest one of the MGC_OTG_REQUEST_* constants
 * @see #MGC_OtgMachineInit
 */
extern void MGC_OtgMachineRequest(MGC_OtgMachine * pMachine, u8 bRequest);

#endif				/* multiple inclusion protection */
