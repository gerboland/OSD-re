/*
 * linux/drivers/net/ti_davinci_emac/cpmac_palOsCache.h
 *
 * EMAC driver Cache abstraction
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
      0.2 Anant Gole - ported for DaVinci linux 2.6 - made the functions
                      as efficient as possible for ARM 926 cache by adding assembly code
 */

#ifndef __CPMAC_PAL_OSCACHE_H__
#define __CPMAC_PAL_OSCACHE_H__

#include "cpmac_palOsMem.h"

#define ARM926_DCACHE_LINESIZE      32

/**
 * \brief   PAL OS Cache Invalidate
 * 
 *      This function invalidates the cache region. 
 * \param   type is cache type viz. data or instruction cache.
 * \param   start is start address of the memory region.
 * \param   size is size of memory region 
 * \return  PAL_Result
 */
static inline PAL_Result PAL_osCacheInvalidate(PAL_OsMemAddrSpace type,
					       Uint32 start, Uint32 size)
{

	Uint i;

	Uint32 memEndAddr = size + (start & (ARM926_DCACHE_LINESIZE - 1));
	start = start & ~(ARM926_DCACHE_LINESIZE - 1);

	for (i = 0; i < memEndAddr;
	     i += ARM926_DCACHE_LINESIZE, start += ARM926_DCACHE_LINESIZE) {

		if (type == PAL_OSMEM_ADDR_DAT) {	/* Invalidate D Cache */

			__asm__
			    __volatile__("mcr	p15, 0, %0, c7, c6, 1"::"r"
					 (start));

		}

		else if (type == PAL_OSMEM_ADDR_PRG) {	/* Invalidate I Cache */

			__asm__
			    __volatile__("mcr	p15, 0, %0, c7, c5, 1"::"r"
					 (start));

		} else

			return PAL_OS_ERROR_INVALID_PARAM;

	}

	/* Flush Write back buffer */
	__asm__ __volatile__("mcr	p15, 0, r0, c7, c10, 4"::"r"(start));

	return PAL_SOK;

}

/**
 * \brief   PAL OS Cache Writeback
 * 
 * This function flushes (write backs) the cache content to the memory.
 *
 * \param   type is cache type viz. data or instruction cache.
 * \param   start is start address of the memory region.
 * \param   size is size of memory region 
 * \return  PAL_Result
 */
static inline PAL_Result PAL_osCacheWb(PAL_OsMemAddrSpace type, Uint32 start,
				       Uint32 size)
{

	Uint i;

	Uint32 memEndAddr = size + (start & (ARM926_DCACHE_LINESIZE - 1));
	start = start & ~(ARM926_DCACHE_LINESIZE - 1);

	for (i = 0; i < memEndAddr;
	     i += ARM926_DCACHE_LINESIZE, start += ARM926_DCACHE_LINESIZE) {

		if (type == PAL_OSMEM_ADDR_DAT) {	/* Flush D Cache */

			__asm__
			    __volatile__("mcr	p15, 0, %0, c7, c10, 1"::"r"
					 (start));

		} else {

			return PAL_OS_ERROR_INVALID_PARAM;

		}

	}

	/* Flush Write back buffer */
	__asm__ __volatile__("mcr	p15, 0, %0, c7, c10, 4"::"r"(start));

	return PAL_SOK;

}

/**
 * \brief   PAL OS Cache Writeback Invalidate
 * 
 * This function flushes (write backs) & invalidates the cache content to the memory.
 *
 * \param   type is cache type viz. data or instruction cache.
 * \param   start is start address of the memory region.
 * \param   size is size of memory region 
 * \return  PAL_Result
 */
static inline PAL_Result PAL_osCacheWbInv(PAL_OsMemAddrSpace type, Uint32 start,
					  Uint32 size)
{

	Uint i;

	Uint32 memEndAddr = size + (start & (ARM926_DCACHE_LINESIZE - 1));
	start = start & ~(ARM926_DCACHE_LINESIZE - 1);

	for (i = 0; i < memEndAddr;
	     i += ARM926_DCACHE_LINESIZE, start += ARM926_DCACHE_LINESIZE) {

		if (type == PAL_OSMEM_ADDR_DAT) {	/* Flush & Invalidate D Cache */

			__asm__
			    __volatile__("mcr	p15, 0, %0, c7, c14, 1"::"r"
					 (start));

		} else {

			return PAL_OS_ERROR_INVALID_PARAM;

		}

	}

	/* Flush Write back buffer */
	__asm__ __volatile__("mcr	p15, 0, %0, c7, c10, 4"::"r"(start));

	return PAL_SOK;

}

#endif				/* __CPMAC_PAL_OSCACHE_H__ */
