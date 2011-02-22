/*
 *  linux/arch/arm/mach-itdm320-20/irq.c
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
#include <linux/fs.h>
#include <linux/ptrace.h>
#include <linux/interrupt.h>

#include <asm/irq.h>
#include <asm/io.h>
#include <asm/mach/irq.h>

static spinlock_t  irq_lock;

/* IRQ = (Interrupt_Num % 16)
 * Register = ((Interrupt_Num - IRQ) / 8) + Register_Base
 */
#define __set_irq_reg(irq_var, reg_var, reg_base) \
	do {\
	    unsigned char tmp_irq = irq_var;\
\
	    irq_var %= 16;\
\
	    reg_var = reg_base + ((tmp_irq - irq_var) / 8);\
	} while(0)

static void itdm320_mask_irq(unsigned int irq)
{
	unsigned long flags;
	unsigned short reg_addr = 0;
	unsigned short irq_val = irq;

	spin_lock_irqsave(&irq_lock, flags);
	__set_irq_reg(irq_val, reg_addr, IO_INTC_EINT0);
	outw((inw(reg_addr) & ~(1 << irq_val)), reg_addr);
	spin_unlock_irqrestore(&irq_lock, flags);
}

static void itdm320_unmask_irq(unsigned int irq)
{
	unsigned long flags;
	unsigned short reg_addr = 0;
	unsigned short irq_val = irq;

	spin_lock_irqsave(&irq_lock, flags);
	__set_irq_reg(irq_val, reg_addr, IO_INTC_EINT0);
	outw((inw(reg_addr) | (1 << irq_val)), reg_addr);
	spin_unlock_irqrestore(&irq_lock, flags);
}

static void itdm320_ack_irq(unsigned int irq)
{
	unsigned short reg_addr = 0;
	unsigned short irq_val = irq;

	__set_irq_reg(irq_val, reg_addr, IO_INTC_IRQ0);

	outw((1 << irq_val), reg_addr);
}

static struct irqchip intc_chip = {
	.ack	= itdm320_ack_irq,
	.mask	= itdm320_mask_irq,
	.unmask = itdm320_unmask_irq,
};

void __init itdm320_init_irq(void)
{
	int irq;
	unsigned long flags;

	/* WBB: Do we need to disable every IRQ
	 * to begin with?  What about Entry
	 * addresses?
	 */

	spin_lock_irqsave(&irq_lock, flags);

	/* Clearing all FIQs and IRQs. */
	outw(0xFFFF, IO_INTC_IRQ0);
	outw(0xFFFF, IO_INTC_IRQ1);
	outw(0xFFFF, IO_INTC_IRQ2);

	outw(0xFFFF, IO_INTC_FIQ0);
	outw(0xFFFF, IO_INTC_FIQ1);
	outw(0xFFFF, IO_INTC_FIQ2);

	/* Masking all Interrupts. */
	outw(0, IO_INTC_EINT0);
	outw(0, IO_INTC_EINT1);
	outw(0, IO_INTC_EINT2);

	/* Setting INTC to all IRQs. */
	outw(0, IO_INTC_FISEL0);
	outw(0, IO_INTC_FISEL1);
	outw(0, IO_INTC_FISEL2);

	/* Setup Linux Interrupt Handlers.
	 * Should we be level, edge, or
	 * simple based "do" routines?
	 */
	for (irq = 0; irq < NR_IRQS; irq++) {
		set_irq_chip(irq, &intc_chip);
        set_irq_handler(irq, do_edge_IRQ);
		set_irq_flags(irq, IRQF_VALID);
	}

	spin_unlock_irqrestore(&irq_lock, flags);
}

