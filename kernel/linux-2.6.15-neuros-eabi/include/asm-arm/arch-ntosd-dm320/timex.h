/*
 *  linux/include/asm-arm/arch-integrator/timex.h
 *
 *  Integrator architecture timex specifications
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

#ifndef __ASM_ARCH_TIMEX_H__
#define __ASM_ARCH_TIMEX_H__

/* We get the Prescalar from "asm/arch/time.h" */
#include <asm/arch/hardware.h>

#include <asm/param.h>

#define CONFIG_TIMER0_TMPRSCL          0x000A
#define CLOCK_TICK_RATE                (CONFIG_SYS_CLK_FREQ / CONFIG_TIMER0_TMPRSCL)
#define CONFIG_TIMER0_TMDIV            (CLOCK_TICK_RATE / HZ)

#define CONFIG_TIMER0_TMMD_STOP        0x0000
#define CONFIG_TIMER0_TMMD_ONE_SHOT    0x0001
#define CONFIG_TIMER0_TMMD_FREE_RUN    0x0002

#define CONFIG_TIMER1_TMMD_STOP        0x0000
#define CONFIG_TIMER1_TMMD_ONE_SHOT    0x0001
#define CONFIG_TIMER1_TMMD_FREE_RUN    0x0002

#define CONFIG_TIMER2_TMMD_STOP        0x0000
#define CONFIG_TIMER2_TMMD_ONE_SHOT    0x0001
#define CONFIG_TIMER2_TMMD_FREE_RUN    0x0002
#define CONFIG_TIMER2_TMMD_CCD_SHUTTER 0x0100
#define CONFIG_TIMER2_TMMD_CCD_STROBE  0x0200
#define CONFIG_TIMER2_TMMD_POLARITY    0x0400
#define CONFIG_TIMER2_TMMD_TRG_SELECT  0x0800
#define CONFIG_TIMER2_TMMD_TRG_READY   0x1000
#define CONFIG_TIMER2_TMMD_SIGNAL      0x2000

#define CONFIG_TIMER3_TMMD_STOP        0x0000
#define CONFIG_TIMER3_TMMD_ONE_SHOT    0x0001
#define CONFIG_TIMER3_TMMD_FREE_RUN    0x0002
#define CONFIG_TIMER3_TMMD_CCD_SHUTTER 0x0100
#define CONFIG_TIMER3_TMMD_CCD_STROBE  0x0200
#define CONFIG_TIMER3_TMMD_POLARITY    0x0400
#define CONFIG_TIMER3_TMMD_TRG_SELECT  0x0800
#define CONFIG_TIMER3_TMMD_TRG_READY   0x1000
#define CONFIG_TIMER3_TMMD_SIGNAL      0x2000

#endif
