/*
 *  Copyright(C) 2007 Neuros Technology International LLC.
 *               <www.neurostechnology.com>
 *
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, version 2 of the License.
 *
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
 * CS2 configuration header.
 *
 * REVISION:
 *
 *
 * 1) Initial creation. ---------------------------- 2007-08-27 TQ
 *
 */


#ifndef __ASM_ARCH_CS2CTL_H
#define __ASM_ARCH_CS2CTL_H

void dm320_controler_enable_nand(void);
void dm320_controler_enable_cfc(void);
void dm320_controler_get_lock(void);
void dm320_controler_release_lock(void);
void set_flash_type(void);
int  get_flash_type(void);

#endif
