/* linux/include/asm-arm/arch-clps711x/debug-macro.S
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

/*	ISOFT_PORT	*/

#if 0

                .macro  senduart,rd,rx
                strb    \rd, [\rx]
                .endm

                .macro  busyuart,rd,rx
                .endm

                .macro  waituart,rd,rx
                .endm

		.macro  addruart,rx
                mrc     p15, 0, \rx, c1, c0
                tst     \rx, #1

                ldreq   \rx, =IO_START           @ Physical IO Base
                ldrne   \rx, =IO_BASE            @ Virtual IO Base

@               orr     \rx, \rx, #PHY_IO_BASE   @ Offset into Virtual, or nothing
                orr     \rx, \rx, #IO_UART0_DTRR @ UART0 base
                .endm

#endif

#if 1
#include <asm/arch/hardware.h>

                .macro  addruart,rx
                mrc     p15, 0, \rx, c1, c0
                tst     \rx, #1

                ldreq   \rx, =IO_START           @ Physical IO Base
                ldrne   \rx, =IO_BASE            @ Virtual IO Base

@               orr     \rx, \rx, #PHY_IO_BASE   @ Offset into Virtual, or nothing
                orr     \rx, \rx, #IO_UART0_DTRR @ UART0 base
                .endm

                .macro  senduart,rd,rx
                strh    \rd, [\rx, #0x00]       @ UARTx_DTRR Offset
                .endm

                .macro  waituart,rd,rx
1001:           ldrh    \rd, [\rx, #0x0c]       @ UARTx_SR Offset
                tst     \rd, #0x0400            @ Check TX fifo ind.
                beq     1001b                   @ loop til 1=room in TX fifo
                .endm

                .macro  busyuart,rd,rx
1002:           ldrh    \rd, [\rx, #0x0c]       @ UARTx_SR Offset
                tst     \rd, #0x0001            @ Check TX empty bit
                bne     1002b                   @ loop til 0=TX fifo empty
                .endm
#endif

