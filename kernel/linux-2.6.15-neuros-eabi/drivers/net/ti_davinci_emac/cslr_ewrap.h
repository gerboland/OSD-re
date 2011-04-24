/*
 * linux/drivers/net/ti_davinci_emac/cslr_ewrap.h
 *
 * EMAC module control register layer
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
 * Modifications:
 * 0.1 - Original file from AV&V
 * 0.2 Anant Gole - made modifications to remove CSL layer
 */

#ifndef _CSLR_EWRAP_1_08_H_
#define _CSLR_EWRAP_1_08_H_
/*********************************************************************
 * Copyright (C) 2003-2004 Texas Instruments Incorporated. 
 * All Rights Reserved 
 *********************************************************************/
 /** \file cslr_ewrap_1_08.h
 * 
 * \brief This file contains the Register Desciptions for EWRAP
 * 
 *********************************************************************/

/**************************************************************************\
* Register Overlay Structure
\**************************************************************************/
typedef struct {

	volatile Uint32 RSVD0;

	volatile Uint32 EWCTL;

	volatile Uint32 EWINTTCNT;

} EwrapRegs;

/**************************************************************************\
* Field Definition Macros
\**************************************************************************/

/* EWCTL */

#define CSL_EWRAP_EWCTL_INTEN_MASK       (0x00000001u)
#define CSL_EWRAP_EWCTL_INTEN_SHIFT      (0x00000000u)
#define CSL_EWRAP_EWCTL_INTEN_RESETVAL   (0x00000000u)

/*----INTEN Tokens----*/
#define CSL_EWRAP_EWCTL_INTEN_DISABLE    (0x00000000u)
#define CSL_EWRAP_EWCTL_INTEN_ENABLE     (0x00000001u)

#define CSL_EWRAP_EWCTL_RESETVAL         (0x00000000u)

/* EWINTTCNT */

#define CSL_EWRAP_EWINTTCNT_EWINTTCNT_MASK (0x0001FFFFu)
#define CSL_EWRAP_EWINTTCNT_EWINTTCNT_SHIFT (0x00000000u)
#define CSL_EWRAP_EWINTTCNT_EWINTTCNT_RESETVAL (0x00000000u)

#define CSL_EWRAP_EWINTTCNT_RESETVAL     (0x00000000u)

#endif				/* 
				 */
