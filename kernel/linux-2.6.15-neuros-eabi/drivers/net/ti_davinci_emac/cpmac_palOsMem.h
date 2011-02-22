/*
 * linux/drivers/net/ti_davinci_emac/cpmac_palOsMem.h
 *
 * EMAC driver Memory abstraction
 *
 * Copyright (C) 2005 Texas Instruments.
 *
 * ----------------------------------------------------------------------------
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 * ----------------------------------------------------------------------------
 Modifications:
 ver. 0.1 PSP Architecture Team
      0.2 Ajay Singh, Anant Gole - ported for linux 2.6 and DaVinci
 */

#ifndef __CPMAC_PAL_OSMEM_H__
#define __CPMAC_PAL_OSMEM_H__

#include "_tistdtypes.h"
#include <asm/page.h>
#include <linux/slab.h>
#include <linux/mm.h>
#include <asm/arch/memory.h>
#include <linux/platform_device.h>
#include "ddc_cpmacCfg.h"

#define PAL_OS_ERROR_INVALID_PARAM      -1
#define PAL_OS_ERROR_NO_RESOURCES       -2

/**
 * \brief PAL OS Memory Address Space
 *
 * Specifies type of memory address space
 */
typedef enum {
	PAL_OSMEM_ADDR_PRG = 0,
			/**< Program only address space */
	PAL_OSMEM_ADDR_DAT = 1,	    /**< Data only address */
	PAL_OSMEM_ADDR_IO = 2,	   /**< I/O only space address */
	PAL_OSMEM_ADDR_PRGDAT = 3,     /**< Unified prog-data */
	PAL_OSMEM_ADDR_PRGIO = 4,     /**< Unified prog-io */
	PAL_OSMEM_ADDR_DATIO = 5,     /**< Unified data-io */
	PAL_OSMEM_ADDR_UNIFIED = 6,	/**< Homogeneous, unified prog/data/io memory */
	PAL_OSMEM_ADDR_SPECIAL = 7
	      /**< Special or un-classified address range */
} PAL_OsMemAddrSpace;

/**
 * \brief PAL OS Memory attributes
 *
 * Memory attributes
 */
typedef struct {

	PAL_OsMemAddrSpace addrSpace;

} PAL_OsMemAttrs;

/**
 * \brief PAL OS Memory Alloc
 * Only segment 0 is recognized.
 * This function allocates only contiguous memory.
 * Alignment is not honored since Linux does not provide any 
 * facility for memory alignment from kmalloc. Only a 4 Byte alignement
 * can be expected for 32 bit CPUs and 8 byte for 64 Bit ones.
 * If more alignment is required, it should come from outside by rounding off
 * the memAddr and remembering the original for the call to PAL_osMemFree.
 */
static inline PAL_Result PAL_osMemAlloc(Uint32 segId,
					Uint32 numBytes,
					Uint16 alignment, Ptr * memAddr)
{

	*memAddr = kmalloc(numBytes, GFP_KERNEL);

	if (*memAddr == NULL) {

		return PAL_OS_ERROR_NO_RESOURCES;

	}

	return PAL_SOK;

}

/**
 * \brief PAL OS Memory Free
 * Only segment 0 is recognized.
 */
static inline PAL_Result PAL_osMemFree(Uint32 segId, Ptr memAddr,
				       Uint32 numBytes)
{

	kfree(memAddr);

	return PAL_SOK;

}

/**
 * \brief PAL OS Memory Set
 * This will crash if presented with invalid arguments.
 */
static inline PAL_Result PAL_osMemSet(Ptr memAddr, Char fillVal,
				      Uint32 numBytes)
{

	memset(memAddr, fillVal, numBytes);

	return PAL_SOK;

}

/**
 * \brief PAL OS Memory Copy
 * \note This will misbehave if presented with invalid arguments.
 */
static inline PAL_Result PAL_osMemCopy(Ptr dest, const Ptr src, Uint32 numBytes)
{

	memcpy(dest, src, numBytes);

	return PAL_SOK;

}

/* EMAC Virt to Phy address converter
 * This function converts a virtual address that falls into EMAC module 
 * into the correct physical address, for memory addresses it uses linux
 * virt_to_phys()
 */
static inline Uint32 PAL_osVirtToPhys(Uint32 addr)
{
	if ((addr & 0xFFFF0000) == DAVINCI_CPMAC_BASE_ADDR) {

		addr &= 0x00ffffff;

		addr |= 0x01c00000;

	}

	else {

		addr = virt_to_phys((void *)addr);

	}
	return addr;

}

#endif				/* __CPMAC_PAL_OSMEM_H__ */
