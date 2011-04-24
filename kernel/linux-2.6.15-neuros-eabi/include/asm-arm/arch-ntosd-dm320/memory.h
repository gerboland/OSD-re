/*
 *  linux/include/asm-arm/arch-itdm320/mmu.h
 *
 *  Copyright (C) 1999 ARM Limited
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
#ifndef __ASM_ARCH_MMU_H
#define __ASM_ARCH_MMU_H

#include <linux/version.h>     /* for KERNEL_VERSION() */
#include <linux/bsp_config.h>  /* for PHYS_SDRAM_1 */


/*
 * Task size: 3GB
 */
/* Use UL prefix instead of suffix. */
/* This allows assembly code to grok this constant */
#define TASK_SIZE	UL(0xbf000000)
/* TASK_SIZE_26 has moved to asm-arm/memory.h  */
#if LINUX_VERSION_CODE  < KERNEL_VERSION(2,6,15)
#define TASK_SIZE_26	(0x04000000UL)
#endif

/*
 * This decides where the kernel will search for a free chunk of vm
 * space during mmap's.
 */
#define TASK_UNMAPPED_BASE (0x40000000)

#define CONFIG_SDRAM_START PHYS_SDRAM_1 

/*
 * Page offset: 3GB
 */
#define PAGE_OFFSET	UL(0xC0000000)
#define PHYS_OFFSET	CONFIG_SDRAM_START

/* Integrator Setting
 * #define PHYS_OFFSET	(0x00000000UL)
 */

#define __virt_to_phys__is_a_macro
#define __phys_to_virt__is_a_macro
#define __virt_to_phys(vpage) ((vpage) - PAGE_OFFSET + PHYS_OFFSET)
#define __phys_to_virt(ppage) ((ppage) - PHYS_OFFSET + PAGE_OFFSET)

/*
 * Bus view is the same as physical view
 */
#define __virt_to_bus__is_a_macro
#define __bus_to_virt__is_a_macro
#define __virt_to_bus(x)	__virt_to_phys(x)
#define __bus_to_virt(x)	__phys_to_virt(x)

#endif
