/*
 *  linux/include/asm-arm/arch-itdm320/irqs.h
 *
 *  Copyright (C) 1999 ARM Limited
 *  Copyright (C) 2004 Ingenient Technologies
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

/* 
 *  Interrupt numbers
 */
#define IRQ_TIMER0     0
#define IRQ_TIMER1     1
#define IRQ_TIMER2     2
#define IRQ_TIMER3     3
#define IRQ_CCD_VD0    4
#define IRQ_CCD_VD1    5
#define IRQ_CCD_WEN    6
#define IRQ_VENC       7
#define IRQ_SERIAL0    8
#define IRQ_SERIAL1    9
#define IRQ_EXT_HOST   10
#define IRQ_DSPHINT    11
#define IRQ_UART0      12
#define IRQ_UART1      13
#define IRQ_USB_DMA    14
#define IRQ_USB_CORE   15
#define IRQ_VLYNQ      16
#define IRQ_MTC0       17
#define IRQ_MTC1       18
#define IRQ_SD_MMC     19
#define IRQ_SDIO_MS    20
#define IRQ_GIO0       21
#define IRQ_GIO1       22
#define IRQ_GIO2       23
#define IRQ_GIO3       24
#define IRQ_GIO4       25
#define IRQ_GIO5       26
#define IRQ_GIO6       27
#define IRQ_GIO7       28
#define IRQ_GIO8       29
#define IRQ_GIO9       30
#define IRQ_GIO10      31
#define IRQ_GIO11      32
#define IRQ_GIO12      33
#define IRQ_GIO13      34
#define IRQ_GIO14      35
#define IRQ_GIO15      36
#define IRQ_PREVIEW0   37
#define IRQ_PREVIEW1   38
#define IRQ_WATCHDOG   39
#define IRQ_I2C        40
#define IRQ_CLKC       41

/* Embedded Debugging Interrupts */
#define IRQ_ICE        42
#define IRQ_ARMCOM_RX  43
#define IRQ_ARMCOM_TX  44

#define IRQ_RESERVED   45

#define NR_IRQS        46

/* Debugging purposes please remove -> WBB 04/12/2004 */
# define I8042_KBD_IRQ	1
# define I8042_AUX_IRQ	12
