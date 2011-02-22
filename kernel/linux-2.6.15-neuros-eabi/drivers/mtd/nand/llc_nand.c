/*******************************************************************************
**+--------------------------------------------------------------------------+**
**|                            ****                                          |**
**|                            ****                                          |**
**|                            ******o***                                    |**
**|                      ********_///_****                                   |**
**|                      ***** /_//_/ ****                                   |**
**|                       ** ** (__/ ****                                    |**
**|                           *********                                      |**
**|                            ****                                          |**
**|                            ***                                           |**
**|                                                                          |**
**|         Copyright (c) 1998-2004 Texas Instruments Incorporated           |**
**|                        ALL RIGHTS RESERVED                               |**
**|                                                                          |**
**| Permission is hereby granted to licensees of Texas Instruments           |**
**| Incorporated (TI) products to use this computer program for the sole     |**
**| purpose of implementing a licensee product based on TI products.         |**
**| No other rights to reproduce, use, or disseminate this computer          |**
**| program, whether in part or in whole, are granted.                       |**
**|                                                                          |**
**| TI makes no representation or warranties with respect to the             |**
**| performance of this computer program, and specifically disclaims         |**
**| any responsibility for any damages, special or consequential,            |**
**| connected with the use of this program.                                  |**
**|                                                                          |**
**+--------------------------------------------------------------------------+**
*******************************************************************************/

/** \file   llc_nand.c
	\brief  NAND LLC definitions. 

	This file contains the LLC definitions, that are used by the DDC.

	(C) Copyright 2004, Texas Instruments, Inc

	\author     AK Mistral
	\version    1.0
 */

#include <asm/arch/csl3x/tistdtypes.h>
#include <asm/arch/csl3x/csl_nand.h>
#include "llc_nandIf.h"
#include <asm/arch/memory.h>

//#include <assert.h>

/*
 *=============================================
 * Definitions of Externally Visible Functions
 *=============================================
 */
 
/**
 *  \brief Initialize the Low level controller.
 *
 *  The low level NAND controller is initialized, by this API,
 *  that is a part of the Init call.
 *
 *  \param  nandObj [IN]     NAND Object information
 *  \param  instId [IN]      Instance Id.
 *  \return None
 */
#define CSL_NAND_REG1                 (0x01E00000)

void LLC_nandInit(
				 LLC_NandObj         *nandObj,
				 Uint32              instId)
{


	nandObj->instId = instId;

	switch (nandObj->instId) {
	case CSL_NAND:
		nandObj->intrNumNand = 0xFF;
		nandObj->regs = (CSL_NandRegsOvly)DAVINCI_PERI_ADDR(CSL_NAND_REG1);
		printk ("NAND Registers %x\n", nandObj->regs);
		break;
	default:
		return;
	}
}


/**
 *  \brief Perform the Hw Setup of NAND.
 *
 *  The NAND low level controller is setup according to the hardware
 *  configuration requested.
 *
 *  \param  hNand [IN]      NAND Driver Instance Handle
 *  \param  hwConfig [IN]   Hardware Configuration information.
 *  \return CSL_SOK or CSL  Error code
 */
void LLC_nandHwSetup(
					LLC_NandHandle      hNand,
					LLC_NandHwConfig    *hwConfig)
{
	return;
}


int LLC_nandGetIntrNum(int deviceId)
{
	int     intrNum = 0;

	switch (deviceId) {
	case CSL_NAND:
		intrNum = 0xFF;
		break;
	default:
		break;
	}

	return(intrNum);
}

