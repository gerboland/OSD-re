#ifndef NEUROS_GENERIC__H
#define NEUROS_GENERIC__H
/*
 *  Copyright(C) 2006-2007 Neuros Technology International LLC. 
 *               <www.neurostechnology.com>
 *
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, version 2 of the License.
 *
 *  This program is distributed in the hope that, in addition to its 
 *  original purpose to support Neuros hardware, it will be useful 
 *  otherwise, but WITHOUT ANY WARRANTY; without even the implied 
 *  warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  
 *  See the GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 *
 ****************************************************************************
 *
 * Neuros generic driver header.
 *
 * REVISION:
 * 
 * 1) Initial creation. ----------------------------------- 2005-06-02 MG 
 *
 */
#ifdef __KERNEL__
    #include <linux/types.h>
#else
    #include <stdint.h>
#endif

#define NEUROS_GENERIC_MAJOR 104
#define NEUROS_GENERIC_IOC_MAGIC 'g'

#define NT_GENERIC_SET_ARM_CLOCK  _IOW(NEUROS_GENERIC_IOC_MAGIC, 8, int)

#endif /* NEUROS_GENERIC__H */

