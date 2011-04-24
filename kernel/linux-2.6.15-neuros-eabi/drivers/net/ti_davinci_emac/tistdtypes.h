/*
 * linux/drivers/net/ti_davinci_emac/tistdtypes.h
 *
 * TI standard types
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
 ver  0.1 PSP architecture team
 */

#ifndef _TISTDTYPES_H_
#define _TISTDTYPES_H_

/*
    The purpose of this header file is to consolidate all the primitive "C"
    data types into one file. This file is expected to be included in the
    basic types file exported by other software components, for example CSL.
 */

#ifndef _TI_STD_TYPES
#define _TI_STD_TYPES

#ifndef TRUE

typedef int Bool;
#define TRUE		((Bool) 1)
#define FALSE		((Bool) 0)

#endif

typedef int Int;
typedef unsigned int Uns;	/* deprecated type */
typedef char Char;
typedef char *String;
typedef void *Ptr;

/* unsigned quantities */
typedef unsigned int Uint32;
typedef unsigned short Uint16;
typedef unsigned char Uint8;

/* signed quantities */
typedef int Int32;
typedef short Int16;
typedef char Int8;

#endif				/* _TI_STD_TYPES */

#endif				/* _TISTDTYPES_H_ */
