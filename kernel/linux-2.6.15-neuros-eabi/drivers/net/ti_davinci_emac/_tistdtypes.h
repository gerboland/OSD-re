/*
 * linux/drivers/net/ti_davinci_emac/_tistdtypes.h
 *
 * TI Standard defines for primitive "C" types only
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
 ver. 1.0 PSP Architecture Team
      1.4 - Anant Gole - added DaVinci CPMAC specific port defines
 */

#ifndef __TI_STD_TYPES_
#define __TI_STD_TYPES_

/**
 * \defgroup TIBasicTypes TI Basic Types
 * 
 * All components - PAL, SRV, DDC, CSL shall use TI basic types 
 * to maintain compatibility of code with all systems. 
 * \n File _tistdtypes.h will adapt the types to the native compiler.
 */
/*@{*/

#include "tistdtypes.h"

#ifndef NULL
#define NULL 0
#endif				/* 
				 */
typedef unsigned int Uint;	/**< Unsigned base integer quantity */

#ifndef True
#define True    TRUE
#define False   FALSE
#endif				/* 
				 */

/* PAL Result - return value of a function  */
typedef Int PAL_Result;

#ifndef PAL_SOK
#define PAL_SOK 0
#endif				/* 
				 */

#endif				/* __TI_STD_TYPES */
