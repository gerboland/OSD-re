/*
 *
 * Copyright (C) 2004-2006 Ingenient Technologies
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
 *
 */

#ifndef _DSP_REGS_H
#define _DSP_REGS_H

#include <asm/arch/omap24xx.h>
#include <asm/arch/board-itomap2430.h>

#define IO_L4_WAKEUP_BASE       IO_ADDRESS(L4_WAKEUP_2430_BASE)

#define IO_RMCM_BASE            IO_ADDRESS(OMAP24XX_PRCM_BASE)
#define IO_CM_FCLKEN_DSP        (IO_RMCM_BASE+0x800)
#define IO_CM_ICLKEN_DSP        (IO_RMCM_BASE+0x810)
#define IO_RM_RSTCTRL_DSP       (IO_RMCM_BASE+0x850)

#define IVA21_MMU_REVISION      (IVA21_MMU_ADDR_VIR+0x000)
#define IVA21_MMU_SYSCONFIG     (IVA21_MMU_ADDR_VIR+0x010)
#define IVA21_MMU_SYSSTATUS     (IVA21_MMU_ADDR_VIR+0x014)
#define IVA21_MMU_IRQ_STATUS    (IVA21_MMU_ADDR_VIR+0x018)
#define IVA21_MMU_IRQ_ENABLE    (IVA21_MMU_ADDR_VIR+0x01C)
#define IVA21_MMU_CNTL          (IVA21_MMU_ADDR_VIR+0x044)
#define IVA21_MMU_FAULT_ADDR    (IVA21_MMU_ADDR_VIR+0x048)
#define IVA21_MMU_LOCK          (IVA21_MMU_ADDR_VIR+0x050)
#define IVA21_MMU_LD_TBL        (IVA21_MMU_ADDR_VIR+0x054)
#define IVA21_MMU_CAM           (IVA21_MMU_ADDR_VIR+0x058)
#define IVA21_MMU_RAM           (IVA21_MMU_ADDR_VIR+0x05c)
#define IVA21_MMU_READ_CAM      (IVA21_MMU_ADDR_VIR+0x068)
#define IVA21_MMU_READ_RAM      (IVA21_MMU_ADDR_VIR+0x06c)
#define IVA21_MMU_EMU_FLT_ADDR  (IVA21_MMU_ADDR_VIR+0x070)

#define CONTROL_IVA2_BOOTADDR   (IO_L4_WAKEUP_BASE+0x2400)
#define CONTROL_IVA2_BOOTMOD    (IO_L4_WAKEUP_BASE+0x2404)

#define PRCM_FCLKEN_DSP         (IO_L4_WAKEUP_BASE+0x6800)
#define PRCM_ICLKEN_DSP         (IO_L4_WAKEUP_BASE+0x6810)
#define PRCM_IDLEST_DSP         (IO_L4_WAKEUP_BASE+0x6820)
#define PRCM_RSTCTRL_DSP        (IO_L4_WAKEUP_BASE+0x6850)

#define IVA21_MMU_TLBMISS             (1 << 00)
#define IVA21_MMU_TRANSLATIONFAULT    (1 << 01)
#define IVA21_MMU_EMUMISS             (1 << 02)
#define IVA21_MMU_TABLEWALKFAULT      (1 << 03)
#define IVA21_MMU_MULTIHITFAULT       (1 << 04)

#endif // #ifndef _DSP_REGS_H
