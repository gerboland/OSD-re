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

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mm.h>
#include <linux/it_timer.h>
#include <linux/config.h>
#include <linux/errno.h>

#include <asm/io.h>

#include <asm/arch/hardware.h>

#if defined(CONFIG_ARCH_NTDEV_DM320) || defined(CONFIG_ARCH_NTR3_DM320) || defined (CONFIG_ARCH_NTOSD_DM320) || defined(CONFIG_ARCH_ITDM320_20)
#define PLATFORM "DM320"

#define MAX_TIMERS 4

static u16 itimer_tmmd[MAX_TIMERS] = {
	IO_TIMER0_TMMD,
	IO_TIMER1_TMMD,
	IO_TIMER2_TMMD,
	IO_TIMER3_TMMD
};

static u16 itimer_tmprscl[MAX_TIMERS] = {
	IO_TIMER0_TMPRSCL,
	IO_TIMER1_TMPRSCL,
	IO_TIMER2_TMPRSCL,
	IO_TIMER3_TMPRSCL
};

static u16 itimer_tmdiv[MAX_TIMERS] = {
	IO_TIMER0_TMDIV,
	IO_TIMER1_TMDIV,
	IO_TIMER2_TMDIV,
	IO_TIMER3_TMDIV
};

static u16 itimer_tmtrg[MAX_TIMERS] = {
	IO_TIMER0_TMTRG,
	IO_TIMER1_TMTRG,
	IO_TIMER2_TMTRG,
	IO_TIMER3_TMTRG
};

#elif defined(CONFIG_ARCH_ITDM340_10)
#define PLATFORM "DM340"

#define MAX_TIMERS 4

static u16 itimer_tmmd[MAX_TIMERS] = {
	IO_TIMER0_TMMD,
	IO_TIMER1_TMMD,
	IO_TIMER2_TMMD,
	IO_TIMER3_TMMD
};

static u16 itimer_tmprscl[MAX_TIMERS] = {
	IO_TIMER0_TMPRSCL,
	IO_TIMER1_TMPRSCL,
	IO_TIMER2_TMPRSCL,
	IO_TIMER3_TMPRSCL
};

static u16 itimer_tmdiv[MAX_TIMERS] = {
	IO_TIMER0_TMDIV,
	IO_TIMER1_TMDIV,
	IO_TIMER2_TMDIV,
	IO_TIMER3_TMDIV
};

static u16 itimer_tmtrg[MAX_TIMERS] = {
	IO_TIMER0_TMTRG,
	IO_TIMER1_TMTRG,
	IO_TIMER2_TMTRG,
	IO_TIMER3_TMTRG
};

#else
#define PLATFORM "UNKNOWN"
#define MAX_TIMERS 1
#endif

#define ITIMER_DESC "Ingenient Technologies " PLATFORM " Timer Driver"

#define __itimer_nok(itimer) ((itimer == 0) || (itimer >= MAX_TIMERS))

static itimer_t itimers[MAX_TIMERS];

int request_itimer(itimer_t timer)
{
	if (__itimer_nok(timer)) return -ENODEV;

	if (itimers[timer]) return -EBUSY;

	itimers[timer] = 1;

	return 0;
}//request_itimer
EXPORT_SYMBOL(request_itimer);

void unrequest_itimer(itimer_t timer)
{
	if (__itimer_nok(timer)) return;

	itimer_set_mode(timer, 0);

	itimers[timer] = 0;
}//unrequest_itimer
EXPORT_SYMBOL(unrequest_itimer);

int itimer_get_mode(itimer_t timer){
	if(__itimer_nok(timer)) return -ENODEV;

	return inw(itimer_tmmd[timer]);
}//itimer_get_mode
EXPORT_SYMBOL(itimer_get_mode);

void itimer_set_prescalar(itimer_t timer, u16 value)
{
	if (__itimer_nok(timer)) return;
	if (!itimers[timer]) return;

	outw(value - 1, itimer_tmprscl[timer]);
}//itimer_set_prescalar
EXPORT_SYMBOL(itimer_set_prescalar);

void itimer_set_divisor(itimer_t timer, u16 value)
{
	if (__itimer_nok(timer)) return;
	if (!itimers[timer]) return;

	outw(value - 1, itimer_tmdiv[timer]);
}//itimer_set_divisor
EXPORT_SYMBOL(itimer_set_divisor);

void itimer_set_mode(itimer_t timer, u16 value)
{
	if (__itimer_nok(timer)) return;
	if (!itimers[timer]) return;

	outw(value, itimer_tmmd[timer]);
}//itimer_set_mode
EXPORT_SYMBOL(itimer_set_mode);

void itimer_trigger(itimer_t timer)
{
	if (__itimer_nok(timer)) return;
	if (!itimers[timer]) return;

	outw(1, itimer_tmtrg[timer]);
}//itimer_trigger
EXPORT_SYMBOL(itimer_trigger);

static int __init itimer_init(void)
{
	itimers[0] = 1;

	printk(KERN_INFO ITIMER_DESC "\n");

	return 0;
}//itimer_init

static void __exit itimer_exit(void)
{
	int i;

	for (i = 1; i < MAX_TIMERS; ++i)
		unrequest_itimer(i);
}//itimer_exit

MODULE_AUTHOR("Ingenient Technologies");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION(ITIMER_DESC);

module_init(itimer_init);
module_exit(itimer_exit);
