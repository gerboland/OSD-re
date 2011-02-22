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

#include <linux/config.h>
#include <linux/module.h>
#include <linux/tty.h>
#include <linux/ioport.h>
#include <linux/init.h>
#include <linux/serial.h>
#include <linux/console.h>
#include <linux/sysrq.h>
#include <linux/device.h>
#include <linux/serial_core.h>

#include <asm/io.h>
#include <asm/irq.h>
#include <asm/arch/gio.h>
#include <asm/arch/hardware.h>

#if 0
#ifndef dbg
#define dbg(format) printk(KERN_INFO "%s::%d> " format, __FUNCTION__, __LINE__)
#endif

#ifndef dbg_arg
#define dbg_arg(format, arg...) printk(KERN_INFO "%s::%d> " format, __FUNCTION__, __LINE__, ## arg)
#endif

#else

#ifndef dbg
#define dbg(format) do { } while (0)
#endif

#ifndef dbg_arg
#define dbg_arg(format, arg...) do { } while (0)
#endif

#endif

#define UART_NR		2
#define SERIAL_IT_NR	UART_NR

#define IT_UART_ISR_PASS_LIMIT 1
//#define IT_UART_ISR_PASS_LIMIT 256
#define UART_PORT_SIZE		32

/* Register defines */
#define UART_MSR_RX_INT_TRIGGER  0x8000
#define UART_MSR_TX_INT_TRIGGER  0x4000
#define UART_MSR_RX_ERR_INT      0x2000
#define UART_MSR_TOUT_15         0x0C00
#define UART_MSR_TOUT_7          0x0800
#define UART_MSR_TOUT_3          0x0400
#define UART_MSR_PARITY_EN       0x0010
#define UART_MSR_PARITY_SEL      0x0008
#define UART_MSR_STOP_BIT        0x0004
#define UART_MSR_CHAR_LEN        0x0001

#define UART_SR_RX_TRIGGER       0x0800
#define UART_SR_TX_TRIGGER       0x0400
#define UART_SR_RX_ERR           0x0200
#define UART_SR_TIMEOUT          0x0100
#define UART_SR_RX_NOT_EMPTY     0x0004
#define UART_SR_TX_FIFO_EMPTY    0x0002
#define UART_SR_TX_EMPTY         0x0001

#define UART_CLR_FIFO            0x8000

#define UART_LCR_BREAK           0x0100

#define UART_SR_RX_TRIGGER       0x0800
#define UART_SR_TX_TRIGGER       0x0400
#define UART_SR_RX_ERR           0x0200
#define UART_SR_TIMEOUT          0x0100
#define UART_SR_RX_NOT_EMPTY     0x0004
#define UART_SR_TX_FIFO_EMPTY    0x0002
#define UART_SR_TX_REG_EMPTY     0x0001

/* Helper functions */
#define __uart_get_int_status(port) inw(port -> iobase | IO_UART0_SR)

#define __uart_get_brsr(port) inw(port -> iobase | IO_UART0_BRSR)
#define __uart_set_brsr(val, port) outw(val, port -> iobase | IO_UART0_BRSR)

#define __uart_get_data(port) (inw(port -> iobase | IO_UART0_DTRR) & 0x00FF)
#define __uart_set_data(val, port)  (outw(val & 0x00FF, port -> iobase | IO_UART0_DTRR))

#define __uart_get_msr(port) (uart_msrs[port->line])
#define __uart_set_msr(val, port) \
	do {\
		uart_msrs[port->line] = val;\
		outw(uart_msrs[port->line], port -> iobase | IO_UART0_MSR);\
	} while (0)

#define __uart_get_data_count(port) (inw(port -> iobase | IO_UART0_TFCR) & 0x003F)

#define __uart_is_tx_ready(port) ((__uart_get_int_status(port) & UART_SR_TX_TRIGGER) > 0)
#define __uart_is_tx_empty(port) ((__uart_get_int_status(port) & UART_SR_TX_EMPTY) > 0)

#define __uart_is_rx_empty(port) ((inw(port -> iobase | IO_UART0_RFCR) & 0x001F) == 0)

#define __uart_enable_tx_int(port) __uart_set_msr(__uart_get_msr(port) | UART_MSR_TX_INT_TRIGGER, port)
#define __uart_enable_rx_int(port) __uart_set_msr(__uart_get_msr(port) | UART_MSR_RX_INT_TRIGGER, port)

#define __uart_enable_int(port) \
	do {\
		__uart_enable_tx_int(port);\
		__uart_enable_rx_int(port);\
	} while(0)

#define __uart_disable_tx_int(port) __uart_set_msr(__uart_get_msr(port) & ~(UART_MSR_TX_INT_TRIGGER), port)
#define __uart_disable_rx_int(port) __uart_set_msr(__uart_get_msr(port) & ~(UART_MSR_RX_INT_TRIGGER), port)

#define __uart_disable_int(port) \
	do {\
		__uart_disable_tx_int(port);\
		__uart_disable_rx_int(port);\
	} while(0)

#define __uart_enable_break(port) (outw(UART_LCR_BREAK, port -> iobase | IO_UART0_LCR))
#define __uart_disable_break(port) (outw(0, port -> iobase | IO_UART0_LCR))

#define __uart_clear_fifos(port) \
	do {\
		outw(inw(port -> iobase | IO_UART0_RFCR) | UART_CLR_FIFO, port -> iobase | IO_UART0_RFCR);\
		outw(inw(port -> iobase | IO_UART0_TFCR) | UART_CLR_FIFO, port -> iobase | IO_UART0_TFCR);\
	} while (0)

static unsigned short uart_msrs[UART_NR] = {0, 0};

#ifdef CONFIG_SERIAL_INGENIENT_IR
/* insert a mark if no rx int coming in for more than 125 ms, 
   insert  SYN 0x16
 */
static void it_rx_mark(struct uart_port *port)
{
	static  unsigned int jif_prev = 0; // 50 ms
	struct tty_struct* tty ;
	
	
	/* for UART1 only */
	
	if(port->line != 1)
		return;

	tty = port -> info -> tty;
	if((jiffies - jif_prev) > HZ/8){
		if (tty -> flip.count < TTY_FLIPBUF_SIZE) {
			*tty->flip.flag_buf_ptr++ = TTY_NORMAL;
			*tty->flip.char_buf_ptr++ = 0x16;
			tty->flip.count++;
		}
	}
	jif_prev = jiffies;
}
inline void it_uart1_port_init(void)
{
	/* IR receiver needs RX only */
	if(request_gio(GIO_UART1_RXD)){
		printk(KERN_ERR "Request of GIO %d for IR failed\n", GIO_UART1_RXD);
		return;
	}
	gio_set_fsel(GIO_UART1_RXD, 1);
}

#else
#define it_rx_mark(port) do{} while(0)
inline void it_uart1_port_init(void)
{
	if(request_gio(GIO_UART1_RXD)||request_gio(GIO_UART1_TXD)){
		printk(KERN_ERR "Request of GIOs for UART1 failed\n");
		/* UART1_TXD is shared with IDE_CFC_RESET on IT320 RevB board
		 * until hardware is changed, UART1 user should be
		 * notified that IDE hot plug would cause UART1 TX 
		 * failure.
		 */
//		return;

	}
	gio_set_fsel(GIO_UART1_RXD, 1);
	gio_set_fsel(GIO_UART1_TXD, 1);

	
}

//#define it_uart1_port_reselect()  gio_set_fsel(GIO_UART1_TXD, 1)
#endif



/* Start of UART Operation Functions */
static void it_uart_stop_tx(struct uart_port* port)
{
	__uart_disable_tx_int(port);
}

static void it_uart_start_tx(struct uart_port* port)
{
	__uart_enable_tx_int(port);
}

static void it_uart_stop_rx(struct uart_port* port)
{
	__uart_disable_rx_int(port);
}

static unsigned int it_uart_tx_empty(struct uart_port* port)
{
	return __uart_is_tx_empty(port) ? TIOCSER_TEMT : 0;
}

static void it_uart_enable_ms(struct uart_port* port)
{
}

static int it_uart_request_port(struct uart_port* port)
{
	/*
	port -> mapbase = get_zeroed_page(GFP_KERNEL);

	if (!port -> mapbase) return -EBUSY;

	port -> membase = (char*) port -> membase;

	return request_mem_region(port -> mapbase,
				  UART_PORT_SIZE,
				  "serial_ingenient") != NULL ? 0 : -EBUSY;
	*/
	return 0;
}

static void it_uart_release_port(struct uart_port* port)
{
	/*
	release_mem_region(port -> mapbase, UART_PORT_SIZE);

	free_page(port -> mapbase);
	*/
}

static void it_uart_config_port(struct uart_port* port, int flags)
{
	if (flags & UART_CONFIG_TYPE) {
		port->type = PORT_INGENIENT;

		it_uart_request_port(port);
	}
}

static int it_uart_verify_port(struct uart_port* port,
			       struct serial_struct* ser)
{
	int ret = 0;

	if (ser -> type != PORT_UNKNOWN && ser -> type != PORT_INGENIENT)
		ret = -EINVAL;
	if (ser -> irq < 0 || ser -> irq >= NR_IRQS)
		ret = -EINVAL;

	return ret;
}

static const char *it_uart_type(struct uart_port* port)
{
	return port -> type == PORT_INGENIENT ? "IT SoC" : NULL;
}

static void it_uart_set_termios(struct uart_port* port,
				struct termios* termios,
				struct termios* old_term)
{
	unsigned int baud;

	unsigned short quot, msr;

	unsigned long flags;

	msr = __uart_get_msr(port) & ~(UART_MSR_CHAR_LEN |
				       UART_MSR_STOP_BIT |
				       UART_MSR_PARITY_EN |
				       UART_MSR_PARITY_SEL);

	/*
	 * Ask the core to calculate the divisor for us.
	 */
	baud = uart_get_baud_rate(port, termios, old_term, 0, (port -> uartclk / 16)); 
	quot = uart_get_divisor(port, baud) - 1;

	

	switch (termios -> c_cflag & CSIZE) {
		case CS7: msr |= UART_MSR_CHAR_LEN; break;
	}

	if (termios -> c_cflag & CSTOPB)
		msr |= UART_MSR_STOP_BIT;

	if (termios -> c_cflag & PARENB) {
		msr |= UART_MSR_PARITY_EN;

		if (termios -> c_cflag & PARODD)
			msr |= UART_MSR_PARITY_SEL;
	}

	spin_lock_irqsave(&port -> lock, flags);

	uart_update_timeout(port, termios -> c_cflag, baud);

#if 0
	port->read_status_mask = AMBA_UARTRSR_OE;
	if (termios->c_iflag & INPCK)
		port->read_status_mask |= AMBA_UARTRSR_FE | AMBA_UARTRSR_PE;
	if (termios->c_iflag & (BRKINT | PARMRK))
		port->read_status_mask |= AMBA_UARTRSR_BE;

	/*
	 * Characters to ignore
	 */
	port->ignore_status_mask = 0;
	if (termios->c_iflag & IGNPAR)
		port->ignore_status_mask |= AMBA_UARTRSR_FE | AMBA_UARTRSR_PE;
	if (termios->c_iflag & IGNBRK) {
		port->ignore_status_mask |= AMBA_UARTRSR_BE;
		/*
		 * If we're ignoring parity and break indicators,
		 * ignore overruns too (for real raw support).
		 */
		if (termios->c_iflag & IGNPAR)
			port->ignore_status_mask |= AMBA_UARTRSR_OE;
	}

	/*
	 * Ignore all characters if CREAD is not set.
	 */
	if ((termios->c_cflag & CREAD) == 0)
		port->ignore_status_mask |= UART_DUMMY_RSR_RX;
#endif

	__uart_set_msr(msr, port);
	__uart_set_brsr(quot, port);

	spin_unlock_irqrestore(&port -> lock, flags);
}

static unsigned int it_uart_get_mctrl(struct uart_port* port)
{
	return TIOCM_CAR | TIOCM_DSR | TIOCM_CTS;
}

static void it_uart_set_mctrl(struct uart_port* port,
			      unsigned int mctrl)
{
}

static void it_uart_break_ctl(struct uart_port* port, int break_state)
{
	unsigned long flags;

	spin_lock_irqsave(&port -> lock, flags);

	if (break_state == -1)
		__uart_enable_break(port);
	else
		__uart_disable_break(port);

	spin_unlock_irqrestore(&port->lock, flags);
}

/* WBB: \todo Need to install error handling
 * for serial receives.
 */
static void it_uart_rx_chars(struct uart_port* port,
			     struct pt_regs* regs)
{
	struct tty_struct* tty = port -> info -> tty;
	unsigned char ch;

	while (!__uart_is_rx_empty(port)) {
		ch = __uart_get_data(port);

		port -> icount.rx++;

		if (tty -> flip.count < TTY_FLIPBUF_SIZE) {
			*tty->flip.flag_buf_ptr++ = TTY_NORMAL;
			*tty->flip.char_buf_ptr++ = ch;

			tty->flip.count++;
		}

		//uart_handle_sysrq_char(port, ch, regs);
	}

	tty_flip_buffer_push(tty);

#ifdef SUPPORT_SYSRQ
	port->sysrq = 0;
#endif
}

static void it_uart_tx_chars(struct uart_port* port)
{
	struct circ_buf *xmit = &port->info->xmit;
	int count;

	if (port->x_char) {
		__uart_set_data(port->x_char, port);

		port->icount.tx++;
		port->x_char = 0;

		return;
	}
	if (uart_circ_empty(xmit) || uart_tx_stopped(port)) {
		it_uart_stop_tx(port);

		return;
	}

	count = port->fifosize - __uart_get_data_count(port);

#if 1
	do {
		__uart_set_data(xmit->buf[xmit->tail], port);

		xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);

		port->icount.tx++;

		if (uart_circ_empty(xmit))
			break;
	} while (--count > 0);
#else
	do {
		while (count--) {
			__uart_set_data(xmit->buf[xmit->tail], port);

			xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);

			port->icount.tx++;

			if (uart_circ_empty(xmit))
				goto done;
		}//while

		count = port->fifosize - __uart_get_data_count(port);
	} while(count);

     done:
#endif

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(port);

	if (uart_circ_empty(xmit))
		it_uart_stop_tx(port);
}

static irqreturn_t it_uart_int(int irq, void* dev_id, struct pt_regs* regs)
{
	struct uart_port* port = dev_id;

	unsigned int pass = 0;
	unsigned short status;

	while ((status = __uart_get_int_status(port)) &&
	       (pass++ < IT_UART_ISR_PASS_LIMIT)) {

		if (status & UART_SR_RX_TRIGGER){
			it_rx_mark(port);
			it_uart_rx_chars(port, regs);
		}

		if ((status & UART_SR_TX_TRIGGER))
			it_uart_tx_chars(port);
	}//while

	return IRQ_HANDLED;
}

static int it_uart_startup(struct uart_port* port)
{
	int result;

	/*
	 * Allocate the IRQ
	 */

	result = request_irq(port -> irq,
			     it_uart_int,
			     0,
			     "it_uart",
			     port);

	if (result) {
		printk(KERN_ERR "Request of irq %d failed\n", port -> irq);

		return result;
	}


	/* 
 	it_uart1_port_init();
	*/
	__uart_clear_fifos(port);
	__uart_enable_int(port);

	return 0;
}

static void it_uart_shutdown(struct uart_port* port)
{
        /*
	 * Free the interrupt
	 */
	free_irq(port -> irq, port);

	/*
	 * disable all interrupts, disable the port
	 */
	__uart_disable_int(port);
	__uart_clear_fifos(port);
}

static struct uart_ops it_uops = {
	.tx_empty	= it_uart_tx_empty,
	.set_mctrl	= it_uart_set_mctrl,
	.get_mctrl	= it_uart_get_mctrl,
	.stop_tx	= it_uart_stop_tx,
	.start_tx	= it_uart_start_tx,
	.stop_rx	= it_uart_stop_rx,
	.enable_ms	= it_uart_enable_ms,
	.break_ctl	= it_uart_break_ctl,
	.startup	= it_uart_startup,
	.shutdown	= it_uart_shutdown,
	.set_termios	= it_uart_set_termios,
	.type		= it_uart_type,
	.release_port	= it_uart_release_port,
	.request_port	= it_uart_request_port,
	.config_port	= it_uart_config_port,
	.verify_port	= it_uart_verify_port,
};

static struct uart_port it_ports[UART_NR] = {
	{
		.iobase = IO_UART0_DTRR,
		.mapbase = IO_ADDRESS(IO_UART0_DTRR),
		.iotype = SERIAL_IO_MEM,
		.irq = IRQ_UART0,
		.uartclk = CONFIG_SYS_CLK_FREQ,
		.fifosize = 32,
		.ops = &it_uops,
		.flags = ASYNC_BOOT_AUTOCONF,
		.line = 0,
	},
	{
		.iobase = IO_UART1_DTRR,
		.mapbase = IO_ADDRESS(IO_UART1_DTRR),
		.iotype = SERIAL_IO_MEM,
		.irq = IRQ_UART1,
		.uartclk = CONFIG_SYS_CLK_FREQ,
		.fifosize = 32,
		.ops = &it_uops,
		.flags = ASYNC_BOOT_AUTOCONF,
		.line = 1,
	}
};

#ifdef CONFIG_SERIAL_INGENIENT_CONSOLE
static inline void it_uart_console_putc(struct uart_port* port,
					const char c)
{
	while(!__uart_is_tx_ready(port));

	__uart_set_data(c, port);
}

static void it_uart_console_write(struct console *con,
				  const char *s,
				  unsigned int count)
{
	struct uart_port* port = &it_ports[con -> index];

	unsigned short msr = __uart_get_msr(port);

	int i;

	__uart_disable_int(port);

	for(i = 0; i < count ; ++i) {
		if (s[i] == '\n')
			it_uart_console_putc(port, '\r');

		it_uart_console_putc(port, s[i]);
	}

	while(!__uart_is_tx_empty(port));

	__uart_set_msr(msr, port);
}

static void __init it_uart_console_get_options(struct uart_port* port,
					       int* baud,
					       int* parity,
					       int* bits)
{
	*baud = 115200;
	*parity = 'n';
	*bits = 8;

	/*
	unsigned short msr;

	msr = __uart_get_msr(port);

	*parity = 'n';

	if (msr & UART_MSR_PARITY_EN)
		*parity = ((msr & UART_MSR_PARITY_SEL) > 0) ? 'o' : 'e';

	*bits = ((msr & UART_MSR_CHAR_LEN) > 0) ? 7 : 8;

	*baud = port -> uartclk / (16 * (__uart_get_brsr(port) + 1));
	*/
}

static int __init it_uart_console_setup(struct console* con, char* options)
{
	struct uart_port* port;
	int baud = 115200;
	int bits = 8;
	int parity = 'n';
	int flow = 'n';

	/*
	 * Check whether an invalid uart number has been specified, and
	 * if so, search for the first available port that does have
	 * console support.
	 */
	if (con -> index >= UART_NR)
		con -> index = 0;

	port = &it_ports[con -> index];

	if (options)
		uart_parse_options(options, &baud, &parity, &bits, &flow);
	else
		it_uart_console_get_options(port, &baud, &parity, &bits);

	return uart_set_options(port, con, baud, parity, bits, flow);
}

extern struct uart_driver it_reg;
static struct console it_console = {
	.name		= "ttyS",
	.write		= it_uart_console_write,
	.device		= uart_console_device,
	.setup		= it_uart_console_setup,
	.flags		= CON_PRINTBUFFER,
	.index		= -1,
	.data		= &it_reg,
};

static int __init it_uart_console_init(void)
{
	register_console(&it_console);

	return 0;
}

console_initcall(it_uart_console_init);

#define IT_CONSOLE &it_console
#else
#define IT_CONSOLE NULL
#endif

struct uart_driver it_reg = {
	.owner			= THIS_MODULE,
	.driver_name		= "Ingenient Serial",
	.dev_name		= "ttyS",
	.major			= TTY_MAJOR,
	.minor			= 64,
	.nr			= UART_NR,
	.cons			= IT_CONSOLE,
};

static int __init it_uart_init(void)
{
	int ret = 0;

	printk(KERN_INFO "Ingenient Technologies TI SoC Serial Driver 1.0\n");

	if ((ret = uart_register_driver(&it_reg)) == 0) {
		int i;


		for (i = 0; i < UART_NR; i++)
			uart_add_one_port(&it_reg, &it_ports[i]);
	}
	if( UART_NR > 1)
		it_uart1_port_init();



	return ret;
}

static void __exit it_uart_exit(void)
{
	int i;

	for (i = 0; i < UART_NR; i++)
		uart_remove_one_port(&it_reg, &it_ports[i]);

	uart_unregister_driver(&it_reg);
}

module_init(it_uart_init);
module_exit(it_uart_exit);

MODULE_AUTHOR("Ingenient Technologies");
MODULE_DESCRIPTION("Ingenient Technologies TI SoC Serial Driver");
MODULE_LICENSE("GPL");
