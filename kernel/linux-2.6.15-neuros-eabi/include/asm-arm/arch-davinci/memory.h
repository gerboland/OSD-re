/*
 *  linux/include/asm-arm/arch-davinci/memory.h
 *
 *  BRIEF MODULE DESCRIPTION
 *      DAVINCI Virtual memory definitions
 *
 *  Copyright (C) 2004 Texas Instruments.
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *  THIS  SOFTWARE  IS PROVIDED   ``AS  IS'' AND   ANY  EXPRESS OR IMPLIED
 *  WARRANTIES,   INCLUDING, BUT NOT  LIMITED  TO, THE IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN
 *  NO  EVENT  SHALL   THE AUTHOR  BE    LIABLE FOR ANY   DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 *  NOT LIMITED   TO, PROCUREMENT OF  SUBSTITUTE GOODS  OR SERVICES; LOSS OF
 *  USE, DATA,  OR PROFITS; OR  BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 *  ANY THEORY OF LIABILITY, WHETHER IN  CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 *  THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *  You should have received a copy of the  GNU General Public License along
 *  with this program; if not, write  to the Free Software Foundation, Inc.,
 *  675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#ifndef __ASM_ARCH_MEMORY_H
#define __ASM_ARCH_MEMORY_H

/**************************************************************************
 * Included Files
 **************************************************************************/

#include <asm/arch/hardware.h>
#include <linux/version.h>     /* for KERNEL_VERSION() */
#include <linux/bsp_config.h>  /* for CONFIG_DSP_START_ADDR */

/**************************************************************************
 * Definitions
 **************************************************************************/

/* Task size: 3GB */

#define TASK_SIZE	UL(0xbf000000)
/* TASK_SIZE_26 has moved to asm-arm/memory.h  */
#if LINUX_VERSION_CODE  < KERNEL_VERSION(2,6,15)
#define TASK_SIZE_26	(0x04000000UL)
#endif

/* This decides where the kernel will search for a free chunk of vm
 * space during mmap's.
 */

#define TASK_UNMAPPED_BASE (0x40000000)
/*#define TASK_UNMAPPED_BASE (TASK_SIZE / 3)*/

/* The start of physical memory available to the kernel. This value
 * is used in the arch-specific bootup code like setup_arch &
 * bootmem_init.  This may or may not be the start of physical memory;
 * There may be memory reserved "in front" of the kernel for other
 * purposes.
 */
#define DAVINCI_DDR_BASE    0x80000000

#define DAVINCI_RESET_VECTOR_BASE 0x00000000
#define DAVINCI_IRAM_BASE   0x00008000 /* ARM Internal RAM (Data) */
#define DAVINCI_IRAM_VIRT   0xe1000000 

#define DAVINCI_PERI_PADDR  0x01c00000
#define DAVINCI_PERI_VADDR  0xe1400000
#define DAVINCI_PERI_SIZE   0x00400000

/* DaVinci Peripheral Base Address */
#define DAVINCI_PERI_ADDR(x)	((x) - DAVINCI_PERI_PADDR + DAVINCI_PERI_VADDR)


#define PHYS_OFFSET DAVINCI_DDR_BASE

/*
 * Ingenient memory layout 
 */

#define DSP_L1DSRAM_PADDR            0x11800000
#define DSP_L1DSRAM_SIZE             SZ_8M

#define INGENIENT_REBOOT_FLAG_OFFSET (SZ_16K-SZ_1K)
#define INGENIENT_REBOOT_FLAG_PADDR  (DAVINCI_IRAM_BASE+INGENIENT_REBOOT_FLAG_OFFSET)
#define INGENIENT_REBOOT_FLAG_VADDR  (DAVINCI_IRAM_VIRT+DAVINCI_IRAM_BASE+INGENIENT_REBOOT_FLAG_OFFSET)

#if defined( CONFIG_DSP_START_ADDR )
#define DSP_START                  CONFIG_DSP_START_ADDR
#else
#define DSP_START                  (DAVINCI_DDR_BASE + (3*SZ_16M))
#endif
#define DSP_SIZE                   ((PHYS_SDRAM_1)+(PHYS_SDRAM_1_SIZE)-(DSP_START))

#define DSP_IPC_START              (DSP_START - SZ_8K)
#define DSP_IPC_SIZE               SZ_8K

#define IMEM_START                 virt_to_phys(high_memory)
#define IMEM_SIZE                  (DSP_IPC_START - IMEM_START) 

#define DAVINCI_FLASH_VIRT         0xe1500000
#define DSP_IPC_BOOTMEM_BASE       0xe2500000
#define DSP_L1DSRAM_BASE           0xe2600000
#define DSP_IPC_BASE               (DSP_IPC_BOOTMEM_BASE + DSP_IPC_BOOTMEM_SIZE - DSP_IPC_SIZE)
#define IMEM_BASE                  imanage_imem_base // ioremap'ed address 
#ifndef __ASSEMBLY__
extern unsigned long IMEM_BASE;
#endif


#define INGENIENT_INTERNAL_ILOG_OFFSET       SZ_8K
#define INGENIENT_INTERNAL_ILOG_VADDR        (DAVINCI_IRAM_VIRT + DAVINCI_IRAM_BASE + INGENIENT_INTERNAL_ILOG_OFFSET)
#define INGENIENT_INTERNAL_ILOG_PADDR        (DAVINCI_IRAM_BASE + INGENIENT_INTERNAL_ILOG_OFFSET)
#define INGENIENT_INTERNAL_ILOG_MAX_SIZE     (INGENIENT_REBOOT_FLAG_OFFSET-INGENIENT_INTERNAL_ILOG_OFFSET)


/* Macros for DSP VA2PA, PA2VA, and Straight writes */
#define __dsp(a) (DSP_BASE + a)
#define __dsp_v2p(a) ((a) - DSP_BASE + DSP_START)
#define __dsp_p2v(a) ((a) - DSP_START + DSP_BASE)

/* Macros for IMEM */
#define __imem(a) (IMEM_BASE + a)
#define __imem_v2p(a) ((a) - IMEM_BASE + IMEM_START)
#define __imem_p2v(a) ((a) - IMEM_START + IMEM_BASE)


/*
 * Bus address is physical address
 */
#define __virt_to_bus(x) __virt_to_phys(x)
#define __bus_to_virt(x) __phys_to_virt(x)

#endif /* __ASM_ARCH_MEMORY_H */
