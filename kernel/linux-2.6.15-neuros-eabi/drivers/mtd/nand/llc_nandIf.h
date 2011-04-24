/******************************************************************************
**+-------------------------------------------------------------------------+**
**|                            ****                                         |**
**|                            ****                                         |**
**|                            ******o***                                   |**
**|                      ********_///_****                                  |**
**|                      ***** /_//_/ ****                                  |**
**|                       ** ** (__/ ****                                   |**
**|                           *********                                     |**
**|                            ****                                         |**
**|                            ***                                          |**
**|                                                                         |**
**|         Copyright (c) 1998-2004 Texas Instruments Incorporated          |**
**|                        ALL RIGHTS RESERVED                              |**
**|                                                                         |**
**| Permission is hereby granted to licensees of Texas Instruments          |**
**| Incorporated (TI) products to use this computer program for the sole    |**
**| purpose of implementing a licensee product based on TI products.        |**
**| No other rights to reproduce, use, or disseminate this computer         |**
**| program, whether in part or in whole, are granted.                      |**
**|                                                                         |**
**| TI makes no representation or warranties with respect to the            |**
**| performance of this computer program, and specifically disclaims        |**
**| any responsibility for any damages, special or consequential,           |**
**| connected with the use of this program.                                 |**
**|                                                                         |**
**+-------------------------------------------------------------------------+**
******************************************************************************/

/**
 * \file	llc_nandIf.h
 *
 * \brief	NAND low-level controller interface
 *
 * This file contains the definitions that are used to abstract the low level
 * controller so that the DDC can be agnostic of the underlying controller.
 * The DDC shall assume a union of all functionalities and the specific
 * underlying controller shall implement or stub out specific interfaces. This
 * file contains the NAND LLC interfaces implemented for the NAND controller
 * on Hibari.
 *
 * (C) Copyright 2004, Texas Instruments, Inc
 *
 *	\note		Set tabstop to 4 (:se ts=4) while viewing this file in an
 *				editor
 *
 *	\author		Aman
 *
 *	\version	0.1
 */

#ifndef _LLC_NANDIF_H_
#define _LLC_NANDIF_H_

#include <asm/arch/csl3x/tistdtypes.h>
#include <asm/arch/csl3x/cslr.h>
#include "llc_nandTypes.h"


/**
 * \defgroup LLCNANDMain LLC NAND Interface Definition
 * 
 * Top-level Encapsulation of all documentation for NAND LLC
 *
 * @{
 */

/**
 * \defgroup LLCNandInterfaces LLC NAND Interfaces
 *
 * Functions exported by the LLC, which are typically called by the DDA to
 * initialize the context of operation of the LLC
 *
 * @{
 */

extern void LLC_nandInit(
						LLC_NandObj             *nandObj,
						Uint32                  instId);
extern void LLC_nandHwSetup(
						   LLC_NandHandle          hI2c,
						   LLC_NandHwConfig        *hwConfig);


/**
 * \defgroup LLCNandConfig LLC NAND Configuration
 *
 * Interfaces for configuring the NAND controller for a specific mode of
 * operation.
 *
 * @{
 */

#define LLC_nandSendCommand(hNand,cmd) (hNand->regs->CE0CLE = cmd)

/**< Set the NAND command
 * \param   hNand   [IN]    {LLC_NandObj *}   Handle to the LLC Nand Object
 * \param   cmd     [IN]    {LLC_NandCommand} Command to be set in Nand    */


#define LLC_nandSetAddr(hNand,addr) (hNand->regs->CE0ALE = addr)

/**< Set the NAND Address
 * \param   hNand   [IN]    {LLC_NandObj *} Handle to the LLC Nand Object
 * \param   addr    [IN]    {Uint32}        Address from where data should  
 *                                          read or to which data has to be
 *                                          written.                       */

/* @} LLCNANDConfig */

/**
 * \defgroup LLCNANDControl LLC NAND Control
 *
 * Interfaces for controlling the operation of the NAND controller when in a
 * specific mode of operation
 *
 * @{
 */

/* @} LLCNANDControl */

/**
 * \defgroup LLCNANDIo LLC NAND IO
 *
 * Interfaces for performing IO operations and for obtaining the status of
 * an IO operation
 *
 * @{
 */

#define LLC_nandReadByte(hNand) (hNand->regs->CE0DATA)

/**< Read data from NAND 
 * \param   hNand   [IN]    {Uint16}        Data to be read from NAND      */

#define LLC_nandWriteByte(hNand,data) \
    (hNand->regs->CE0DATA = data)

/**< Write data from NAND 
 * \param   hNand   [IN]    {LLC_NandObj *}	Handle to the LLC Nand Object
 * \param   data    [IN]    {Uint16}        Data to be written to NAND     */

/* @} LLCNANDIo */

/**
 * \defgroup LLCNANDIntr LLC NAND Interrupt Control
 *
 * Interfaces for controlling the Interrupt generation and handling and for
 * obtaining the status of an interrupt condition
 *
 * @{
 */

/* @} LLCNANDIntr */

/**
 * \defgroup LLCNANDDma LLC NAND DMA Operation and Control
 *
 * Interfaces for programming the DMA and for obtaining the status of a
 * specific DMA channel
 *
 * @{
 */

/* LLC_nandReadRevision */
#define LLC_nandReadRevision(hNand) (hNand->regs->NRCSR)


#define LLC_nandReadEcc1(hNand)  (hNand->regs->NANDF1ECC)
#define LLC_nandReadEcc2(hNand)  (hNand->regs->NANDF2ECC)
#define LLC_nandReadEcc3(hNand)  (hNand->regs->NANDF3ECC)
#define LLC_nandReadEcc4(hNand)  (hNand->regs->NANDF4ECC)

/* @} LLCNANDDma */

/* @} LLCNandInterfaces */

/* @} LLCNANDMain */

#endif  /* _LLC_NANDIF_H_ */

