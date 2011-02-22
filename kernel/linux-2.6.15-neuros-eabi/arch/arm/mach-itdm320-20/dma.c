/*
 *  linux/arch/arm/mach-itdm320-20/dma.c
 *
 *  by Ingenient Technologies
 *
 *  derived from:
 *  arch/arm/mach-shark/dma.c
 *  arch/arm/kernel/dma-ebsa285.c
 *  Copyright (C) 1998 Phil Blundell
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

#include <linux/config.h>
#include <linux/init.h>

#include <asm/dma.h>
#include <asm/mach/dma.h>

void __init arch_dma_init(dma_t *dma)
{
#ifdef CONFIG_ISA_DMA
	isa_init_dma(dma);
#endif
}
