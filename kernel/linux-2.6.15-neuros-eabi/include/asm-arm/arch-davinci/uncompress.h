/*
 * linux/include/asm-arm/arch-omap/uncompress.h
 *
 * Serial port stubs for kernel decompress status messages
 *
 *  Author:     Anant Gole
 * (C) Copyright (C) 2004, Texas Instruments, Inc
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

#include <asm/arch/hardware.h>

typedef volatile struct uart_registers_t {
	unsigned int rbr_thr;   /* 0x00 - Receive buffer register (read) | Transmit Holding register (write) */
	unsigned int ier;       /* 0x04 - Interrupt enable register */
	unsigned int iir_fcr;   /* 0x08 - Interrupt identification register (read) | Fifo control register (write) */
	unsigned int lcr;       /* 0x0C - Line Control register */
	unsigned int mcr;       /* 0x10 - Modem Control register */
	unsigned int lsr;       /* 0x14 - Line Status register */
	unsigned int msr;       /* 0x18 - Modem Status register */
	unsigned int scr;       /* 0x1C - Scratch register */
	unsigned int dll;       /* 0x20 - Divisor Latch LSB register */
	unsigned int dlh;       /* 0x24 - Divisor Latch MSB register */
	unsigned int pid1;      /* 0x28 - Peripheral ID register (Class/Revision) */
	unsigned int pid2;      /* 0x2C - Peripheral ID Type register */
	unsigned int pwremu;    /* 0x30 - Power Management register (MSB 8 bits) | Emulation Management register (LSB 8 bits) */
} uart_registers;

/* Initialize Serial port - This code is written to initialize UART0 only */
static void do_nothing(void)
{
    	unsigned int counter;
	for (counter = 0; counter < 0x200; counter++) {
	        /* Do nothing */
	}
}

/* Wait (busy loop) untill TX FIFO is empty and a character can be transmitted */
static void serial_waitfortxcharcomplete(void)
{
	uart_registers  *uartregs;

	uartregs = (uart_registers *) DAVINCI_UART0_BASE;
    	do_nothing();

	while (!uartregs->lsr & 0x20) {
      	/* Do Nothing */
    	}
}

static void serial_putc(const char c)
{
	uart_registers  *uartregs;

	uartregs = (uart_registers *) DAVINCI_UART0_BASE;
  	if (c=='\n')
     		serial_putc('\r');
  	uartregs->rbr_thr = c;

  	serial_waitfortxcharcomplete();
}

/* Send string on UART */
static void putstr (const char *str )
{
	uart_registers  *uartregs;

	uartregs = (uart_registers *) DAVINCI_UART0_BASE;
	while (*str != '\0') {
        	serial_putc (*str);
        	str++;
    	}
}

#define arch_decomp_setup()
#define arch_decomp_wdog()
