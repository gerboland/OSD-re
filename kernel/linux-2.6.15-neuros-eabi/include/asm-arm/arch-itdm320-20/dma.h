/*
 *  linux/include/asm-arm/arch-integrator/dma.h
 *
 *  Copyright (C) 1997,1998 Russell King
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
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
#ifndef __ASM_ARCH_DMA_H
#define __ASM_ARCH_DMA_H

#define MAX_DMA_ADDRESS		0xffffffff

#define MAX_DMA_CHANNELS	1

/* ISOFT_PORT */
//Defined in asm/memory.h


//#if 0
static inline void __arch_adjust_zones(int node,
				       unsigned long *zone_size,
				       unsigned long *zhole_size) 
{
	if (node != 0) return;

//ISOFT: TODO 
// Dont hardcode 2=>MORMAL_ZONE
	zone_size[2] = zone_size[0];
	zone_size[0] = 0;
	//zone_size[1] = zone_size[0] - 256;
	//zone_size[0] = 256;

	zhole_size[2] = zhole_size[0];
	zhole_size[0] = 0;
}

#ifdef arch_adjust_zones
#undef arch_adjust_zones
#endif
#define arch_adjust_zones(node,size,holes) __arch_adjust_zones(node,size,holes)
//#endif
#endif /* _ASM_ARCH_DMA_H */

