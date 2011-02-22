/*
 *  linux/include/asm-arm/arch-integrator/time.h
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

/* ISOFT_PORT -commented*/
#if 0
#include <asm/arch/timex.h>

/* WBB: We are testing to see which of the following two 
 * functions perform better
 */
#if 1
 #define itdm320_timer_pending() ((~inw(IO_INTC_IRQ0)) & 1)
#else
 #define itdm320_timer_pending() ((inw(IO_INTC_IRQ0) & 1) == 0)
#endif

unsigned long itdm320_gettimeoffset(void)
{
	volatile unsigned long elapsed, tmp;
	volatile int pending;

	/* Compute the elapsed count. The current count tells us how
	 * many counts have elapsed since the last interrupt (COUNT UP)
	 */
	do {
		tmp     = inw(IO_TIMER0_TMCNT);
		pending = itdm320_timer_pending();
		elapsed = inw(IO_TIMER0_TMCNT);
	} while (elapsed < tmp);

	if (pending) elapsed += CONFIG_TIMER0_TMDIV;


	/* Convert the elapsed count to usecs. I guess there are 'tick' usecs
	 * between every interrupt. 
	 */
	return (unsigned long) ((elapsed * tick_usec) / CONFIG_TIMER0_TMDIV);
}

/* Timer IRQ handler for the IT DM320-20. */
static irqreturn_t itdm320_timer_interrupt(int irq,
					   void* dev_id,
					   struct pt_regs *regs)
{
	/* \todo WBB: Do we need to clear the interrupt here?
	 * Well we are going to do it anyways because 
	 * all the other 2.6.5 boards are doing it.
	 */
	//outw((inw(IO_INTC_IRQ0) | 1), IO_INTC_IRQ0);

	do_timer(regs);

	/* WBB: What is this profile thingy?
	 * I wonder if it is the new oprofile
	 * stuff?
	 */
	do_profile(regs);

	return IRQ_HANDLED;
}/* itdm320_timer_interrupt */

/*
 * Set up timer interrupt, and return the current time in seconds.
 */
void __init time_init(void)
{
	/* Turn off all timers */
	outw(CONFIG_TIMER0_TMMD_STOP, IO_TIMER0_TMMD);
	outw(CONFIG_TIMER1_TMMD_STOP, IO_TIMER1_TMMD);
	outw(CONFIG_TIMER2_TMMD_STOP, IO_TIMER2_TMMD);
	outw(CONFIG_TIMER3_TMMD_STOP, IO_TIMER3_TMMD);

	/* Setup the Prescalar */
	outw(CONFIG_TIMER0_TMPRSCL - 1, IO_TIMER0_TMPRSCL);

	/* Setup the Divisor */
	outw(CONFIG_TIMER0_TMDIV - 1, IO_TIMER0_TMDIV);

	/* Turn Timer0 to Free Run mode */
	outw(CONFIG_TIMER0_TMMD_FREE_RUN, IO_TIMER0_TMMD);

	gettimeoffset = itdm320_gettimeoffset;

	timer_irq.handler = itdm320_timer_interrupt;

	setup_irq(IRQ_TIMER0, &timer_irq);
}/* time_init */
#endif
