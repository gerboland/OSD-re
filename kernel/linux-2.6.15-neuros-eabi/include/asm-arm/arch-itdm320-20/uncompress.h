/*
 *  linux/include/asm-arm/arch-itdm320-20/uncompress.h
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
#include <linux/version.h>
#include <asm/arch/hardware.h>
#include <asm/arch/io.h>

#define UART_DTRR_RX_VALID 0x1000
#define UART_DTRR_BREAK 0x0800
#define UART_DTRR_FRAME_ERR 0x0400
#define UART_DTRR_OVERRUN 0x0200
#define UART_DTRR_PARITY_ERR 0x0100
#define UART_DTRR_DATA 0x00FF

#define UART_MSR_RX_INT_TRIGGER 0x8000
#define UART_MSR_TX_INT_TRIGGER 0x4000
#define UART_MSR_RX_ERR_INT 0x2000
#define UART_MSR_PARITY 0x0010

#define UART_RFCR_FIFO_CLEAR 0x8000
#define UART_RFCR_ERR 0x4000
#define UART_RFCR_COUNT 0x003F

#define UART_TFCR_FIFO_CLEAR 0x8000
#define UART_TFCR_TRIGGER_LVL 0x0300
#define UART_TFCR_COUNT 0x003F

#define UART_SR_RX_TRIGGER 0x0800
#define UART_SR_TX_TRIGGER 0x0400
#define UART_SR_RX_ERR 0x0200
#define UART_SR_TIMEOUT 0x0100
#define UART_SR_RX_NOT_EMPTY 0x0004
#define UART_SR_TX_FIFO_EMPTY 0x0002
#define UART_SR_TX_REG_EMPTY 0x0001

/* Start of Structure Declarations */
struct uart_registers {
	unsigned short dtrr;
	unsigned short brsr;
	unsigned short msr;
	unsigned short rfcr;
	unsigned short tfcr;
	unsigned short lcr;
	unsigned short sr;
};

static struct uart_registers* itdm320_uart = (struct uart_registers*) (PHY_IO_BASE | IO_UART0_DTRR);

static void itdm320_putc(const char data)
{
	while (!(itdm320_uart -> sr & UART_SR_TX_TRIGGER));

	itdm320_uart -> dtrr = data;
}

static void base_putc(const char data)
{
	if (data == '\n')
		itdm320_putc('\r');

	itdm320_putc(data);
}

#if LINUX_VERSION_CODE  < KERNEL_VERSION(2,6,15)
static void puts(const char* buffer)
#else
static void putstr(const char* buffer)
#endif
{
	while (*buffer)
		base_putc(*buffer++);
}

/*
 * nothing to do
 */
#define arch_decomp_setup()

#define arch_decomp_wdog()
