/*
 * linux/drivers/net/ti_davinci_emac/cpswhalcommon_stddef.h
 *
 * MDIO standard defines
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
 *      01Oct03   Michael Hanrahan      1.00   
 */

#ifndef _CPSWHALCOMMON_STDDEF_H
#define _CPSWHALCOMMON_STDDEF_H

#ifndef _INC_ADAM2

#ifndef size_t
#define size_t unsigned int
#endif				/* 
				 */

typedef char bit8;

typedef short bit16;

typedef int bit32;

typedef unsigned char bit8u;

typedef unsigned short bit16u;

typedef unsigned int bit32u;

#endif				/* 
				 */

#ifndef TRUE
#define TRUE (1==1)
#endif				/* 
				 */

#ifndef FALSE
#define FALSE !(TRUE)
#endif				/* 
				 */

#ifndef NULL
#define NULL 0
#endif				/* 
				 */

typedef const char HAL_CONTROL_KEY;

#endif				/* 
				 */
