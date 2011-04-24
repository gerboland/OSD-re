/*
 * include/asm-arm/arch-CLPS711x/entry-macro.S
 *
 * Low-level IRQ helper macros for CLPS711X-based platforms
 *
 * This file is licensed under  the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

/*	ISOFT_PORT	*/
/* @ WBB: Modeled after the DM270 stuff ... We need to learn what this does */
#include <asm/hardware.h>               /* should be moved into entry-macro.S */
#include <asm/arch/irqs.h>              /* should be moved into entry-macro.S */
#include <asm/arch/hardware.h>

                .equ    efiqr, IO_ADDRESS(IO_INTC_FIQ0) & 0x20
                .macro  disable_fiq
@               mov     r12, #efiqr
@               strb    r12, [r12]
                .endm

                .macro  get_irqnr_and_base, irqnr, irqstat, base, tmp

        /* GIVEN:
         *   IRQ Entry Table Base Address (IO_INTC_IRQENTRY0)
         * RETURN:
         *   irqnr:  Interrupt number.  Zero corresponds
         *     to bit 0 of the status register
         *   irqstat, base, and tmp may be considered
         *     as scratch registers
         *   Z conditions means no outstanding interrupt
         */

                ldr     \tmp, =IO_ADDRESS(IO_INTC_IRQENTRY0) @ Addr LS 16-bits of entry
                ldrh    \irqnr, [\tmp]                 @ LS 16-bits of entry
                movs    \irqnr, \irqnr, lsr #2         @ Convert to index (set Z)
                subne   \irqnr, \irqnr, #1             @ Offset if non-zero

                .endm

                .macro  irq_prio_table
                .endm

