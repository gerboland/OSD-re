/*
 *  linux/include/asm-arm/arch-itdm320/hardware.h
 *
 *  This file contains the hardware definitions of the Integrator.
 *
 *  Copyright (C) 1999 ARM Limited.
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
#ifndef __ASM_ARCH_HARDWARE_H
#define __ASM_ARCH_HARDWARE_H

//#include <linux/mm.h>

#include <asm/arch/io_registers.h>
#include <asm/arch/irqs.h>
#include <asm/sizes.h>

/* WBB: \todo Need to put this in KConfig */
#define CONFIG_SYS_CLK_FREQ 27000000
#define CONFIG_ETHR_START 0x60000300
#define CONFIG_HDD_START  0x50000000
#define CONFIG_CFC_START  0x40000000
#define CONFIG_SSFDC_START 0x48000000

/*
 * Where in virtual memory the IO devices (timers, system controllers
 * and so on)
 */
#define IO_BASE    0xE0030000         // VA of IO 
#define IO_SIZE    SZ_4K             // How much?
#define IO_START   PHY_IO_BASE       // PA of IO

/* Note: WBB -> This is only used at setup time! Do Not Touch! */
#define AHB_BASE   0xE0035000        //VA of AHB
#define AHB_SIZE   SZ_4K
#define AHB_START  0x00060000        //PA of AHB

#define DSP_BASE   0xE0040000        //VA of DSP RAM
#define DSP_SIZE   SZ_128K
#define DSP_START  0x00040000        //PA of DSP RAM

#define ETHR_BASE  0xE1000000        //VA of Ethernet Chip
#define ETHR_SIZE  SZ_1M
#define ETHR_START CONFIG_ETHR_START // PA of Ethernet CHip

#define HDD_BASE   0xE2000000        // VA of HDD
#define HDD_SIZE   SZ_16M
#define HDD_START  CONFIG_HDD_START  // PA of HDD registers

#define CFC_BASE    0xE3000000         // VA of Compact Flash
#define CFC_SIZE    SZ_16M
#define CFC_START   CONFIG_CFC_START    // PA of Compact Flash IDE registers

// For DM320 FLASH device driver
#define  FLASH_BASE  0xE4000000    // VA of Flash Memory
#define  FLASH_SIZE  SZ_16M        // Size of Flash Memory
#define  FLASH_START 0x00100000    // PA of Flash Memory

// For USB device driver
#define  CONFIG_USB_IO_START  0x80000000
#define  USB_BASE   0xE5000000    // VA of USB IO Memory
#define  USB_SIZE   SZ_16K     // Size of USB IO Memory
#define  USB_START  CONFIG_USB_IO_START  // PA of USB IO Memory

#define IMEM_BASE   0xE6000000
#define IMEM_SIZE   (readl(0xE0035F04) - IMEM_START)
#define IMEM_START  virt_to_phys(high_memory)

// Macros for carry framebuffer address over from bootloader.
#if defined(CONFIG_ARCH_NTOSD_DM320)
#define HORIZON_SIZE  720
#define VERTICAL_SIZE 576
#define FB_BASE     (IMEM_BASE+IMEM_SIZE-FB_SIZE)
#define FB_SIZE     ((HORIZON_SIZE*VERTICAL_SIZE*2)/PAGE_SIZE+1)*PAGE_SIZE //maximum buffer size.
#define FB_START    (IMEM_START+IMEM_SIZE-FB_SIZE)
#endif

/* Macros for DSP VA2PA, PA2VA, and Straight writes */
#define __dsp(a) (DSP_BASE + a)
#define __dsp_v2p(a) ((a) - DSP_BASE + DSP_START)
#define __dsp_p2v(a) ((a) - DSP_START + DSP_BASE)

/* Macros for IDE HDD registers */
#define __ide_hdd(a) (HDD_BASE + a)
#define __ide_hdd_v2p(a) ((a) - HDD_BASE + HDD_START)
#define __ide_hdd_p2v(a) ((a) - HDD_START + HDD_BASE)

/* Macros for IDE CFC registers */
#define __ide_cfc(a) (CFC_BASE + a)
#define __ide_cfc_v2p(a) ((a) - CFC_BASE + CFC_START)
#define __ide_cfc_p2v(a) ((a) - CFC_START + CFC_BASE)

/* Macros for IMEM */
#define __imem(a) (IMEM_BASE + a)
#define __imem_v2p(a) ((a) - IMEM_BASE + IMEM_START)
#define __imem_p2v(a) ((a) - IMEM_START + IMEM_BASE)

/* macro to get at IO space when running virtually */
#define IO_ADDRESS(x) ((x | IO_BASE))

#endif

