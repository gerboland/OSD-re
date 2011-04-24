/*
 *  linux/include/asm-arm/arch-davinci/timex.h
 *
 *  BRIEF MODULE DESCRIPTION
 *      DAVINCI Virtual memofy definitions
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

#ifndef __ASM_ARCH_TIMEX_H
#define __ASM_ARCH_TIMEX_H

/* The system clock is driven off timer 0.
 *
 */

/* The source frequency is the 27MHz MXI clock */

#define CLOCK_SOURCE_FREQUENCY 27000000
#define DAVINCI_TIMER_FREQUENCY CLOCK_SOURCE_FREQUENCY

#define CLOCK_TICK_RATE	       (CLOCK_SOURCE_FREQUENCY)

/* timer0 memory map */
typedef volatile struct timer0_registers_t {
	unsigned int pid12;         /* 0x0 */
	unsigned int emumgt_clksped;/* 0x4 */
	unsigned int gpint_en;      /* 0x8 */
	unsigned int gpdir_dat;     /* 0xC */
	unsigned int tim12;         /* 0x10 */
	unsigned int tim34;         /* 0x14 */
	unsigned int prd12;         /* 0x18 */
	unsigned int prd34;         /* 0x1C */
	unsigned int tcr;           /* 0x20 */
	unsigned int tgcr;          /* 0x24 */
	unsigned int wdtcr;         /* 0x28 */
	unsigned int tlgc;          /* 0x2C */
	unsigned int tlmr;          /* 0x30 */
} timer0_registers;


//static inline cycles_t get_cycles (void)
//{
//	return 0;
//}

#endif /* __ASM_ARCH_TIMEX_H__ */

