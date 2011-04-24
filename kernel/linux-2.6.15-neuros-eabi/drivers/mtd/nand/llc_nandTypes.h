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
 * \file	llc_nandTypes.h
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

#ifndef _LLC_NANDTYPES_H_
#define _LLC_NANDTYPES_H_

//#include <csl_intc.h>
#include <asm/arch/csl3x/csl_nand.h>


/**
 * \defgroup LLCNandMain LLC NAND Interface Definition
 * 
 * Top-level Encapsulation of all documentation for NAND LLC
 *
 * @{
 */

/**
 * \defgroup LLCNandControllerSpecificDefs LLC Specific Definitions
 *
 * Definitions specific to this IP of the NAND
 *
 * @{
 */


typedef struct {
	Uint32              instId;
	CSL_NandRegsOvly    regs;
	Int				    intrNumNand;
	CSL_NandObj         cslNandObj;
	CSL_NandHandle      hCslNand;  
} LLC_NandObj, *LLC_NandHandle;


/* @} LLCNandControllerSpecificDefs */


/**
 * \defgroup LLCNandGenericDefs LLC NAND Generic Definitions Structures
 *
 * NAND definitions that are applicable to any controller and agnostic of a
 * specific IP block
 *
 * @{
 */


typedef enum {
	LLC_NAND_READ_LOPAGE            = 0x00,
	/**< Command to read the first half of the data array                  */
	LLC_NAND_READ_HIPAGE            = 0x01,
	/**< Command to read the second half of the data array                 */
	LLC_NAND_READ_BIGBLK_ENDPAGE    = 0x30,
	/**< Command to read the second half of the array for Big Block NAND   */
	LLC_NAND_READ_SPAREAREA         = 0x50,
	/**< Command to read the spare area  NAND                              */
	LLC_NAND_READ_DEVICEID          = 0x90,
	/**< Command to read the NAND device id                                */
	LLC_NAND_START_WRITE            = 0x80,
	/**< Command to start writing the data into the NAND                   */
	LLC_NAND_STOP_WRITE             = 0x10,
	/**< Command to stop writing the data into the NAND                    */
	LLC_NAND_RESET                  = 0xFF,
	/**< Command to reset the NAND device                                  */
	LLC_NAND_BLKERASE_START         = 0x60,
	/**< Command to start the block erase                                  */
	LLC_NAND_BLKERASE_END           = 0xD0,
	/**< Command to stop the block erase                                   */
	LLC_NAND_READ_STATUS            = 0x70
	/**< Command to read the status register                               */
} LLC_NandCommand;

typedef enum {
	LLC_NAND_BUSWDTH_8          = 0,
	/**< External Device Bus Width 8 bit            						*/
	LLC_NAND_BUSWDTH_16         = 1
	/**< External Device Bus Width 16 bit            						*/
} LLC_NandWidth;

typedef enum {
	LLC_NAND_PAGESZ_256         = 0,
	/**< Page size is 256 + 8bytes                                          */
	LLC_NAND_PAGESZ_512         = 1,
	/**< Page size is 512 + 16bytes                                         */
	LLC_NAND_PAGESZ_1024        = 2,
	/**< Page size is 1024 + 32 bytes                                       */
	LLC_NAND_PAGESZ_2048        = 3
	/**< Page size is 2048 + 64 bytes                                       */
}LLC_NandPageSize;

typedef struct {
	LLC_NandWidth       llcNandWidth;
	/**< External Device Width  											*/
	LLC_NandPageSize    llcNandPageSz;
	/**< NAND page Size         											*/
} LLC_NandBasicConfig;

/**
 * \brief NAND hardware initialization structure
 *
 * This structure contains the hardware initialization data. 
 */
typedef struct {
	LLC_NandBasicConfig         cfgBasic;
	/**< NAND Hardware control setup										*/
} LLC_NandHwConfig;

/* @} LLCNandGenericDefs */

/* @} LLCNandMain */

#endif	/* _LLC_NANDTYPES_H_ */
