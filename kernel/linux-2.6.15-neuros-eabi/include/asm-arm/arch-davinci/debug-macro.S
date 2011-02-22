/* linux/include/asm-arm/arch-pxa/debug-macro.S
 *
 * Debugging macro include header
 *
 *  Copyright (C) 1994-1999 Russell King
 *  Moved from linux/arch/arm/kernel/debug.S by Ben Dooks
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
*/
		 .macro addruart, rx
                 /* Check if MMU is enabled */
                 mrc     p15, 0, \rx, c1, c0
                 tst     \rx, #1

                 ldreq   \rx, =DAVINCI_UART0_BASE_ADDR @ Physical IO Base
                 ldrne   \rx, =DAVINCI_PERI_VADDR      @ Virtual IO Base
                 addne   \rx, \rx, #(DAVINCI_UART0_BASE_ADDR-DAVINCI_PERI_PADDR) 
/*                 subne   \rx, \rx, #DAVINCI_PERI_PADDR */
		.endm
        
		.macro senduart,rd,rx
       		 strb	\rd, [\rx]
		.endm

		.macro waituart,rd,rx
1001:           ldr     \rd, [\rx, #0x14]       @ UARTx_SR Offset
                tst     \rd, #0x040             @ Check TX empty indication
                beq     1001b                   @ loop til 1=TX empty
		.endm

		.macro busyuart,rd,rx
1001:           ldr     \rd, [\rx, #0x14]       @ UARTx_SR Offset
                tst     \rd, #0x020             @ Check TX fifo ind.
                beq     1001b                   @ loop til 1=room in TX fifo
                .endm
        
