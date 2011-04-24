/*
 *
 * Copyright (C) 2006 Ingenient Technologies
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/sched.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/completion.h>
#include <linux/version.h>

#include <asm/io.h>

#include <asm/arch/gio.h>
#include <asm/arch/hardware.h>

#define MG_HACK 1
#if MG_HACK // startof MG-HACK
void mem_outw (unsigned short val,unsigned long add){

        outw (val,add);
        wmb();
}

unsigned short mem_inw (unsigned long add){

        unsigned short val;

        val = inw (add);
        rmb();
        return val;
}
#endif // endof MG-HACK

#if 0
#define dbg(fmt, arg...) printk(KERN_INFO "%s:%d> " \
                                fmt, __func__, __LINE__ , ## arg)
#else
#define dbg(fmt...)
#endif

#define I2C_SCS_COND_START 0x0001
#define I2C_SCS_COND_STOP 0x0002
#define I2C_SCS_XMIT 0x0004
#if 0  // using 400KHz clock
#define I2C_SCS_CLKSEL_400KHZ 0x0008
#else  // using 100KHZ clock
// use 100KHz clock instead to work with MSP430.
#define I2C_SCS_CLKSEL_400KHZ 0x0000
#endif

#define I2C_ACK 0x0100

static DECLARE_COMPLETION(i2c_complete);

#ifdef CONFIG_I2C_ALGOIT_POLL
/* 
Because the i2c functions may be called from within an interrupt
context by the auto_exposure driver on Netcam boards,
we can't afford the possibility of sleeping or scheduling.
While technically it is still interrupt driven,
the kernel is not free to schedule or sleep; it spins until IO is
complete. 
*/
static volatile int io_completed;
/* 
The DM275 and DM320 occasionally "lose" interrupts.  To keep the 
kernel from hanging during these times, we implement a timeout.
*/
static volatile unsigned long io_timeout;
#define wait_for_completion(x) io_completed=0;io_timeout=jiffies+HZ;while((!io_completed) && (jiffies<io_timeout));
#define init_completion(x) /* *** do nothing *** */
#endif

static irqreturn_t i2c_it_handler(int irq,
				  void* dev_id,
				  struct pt_regs* regs)
{
#ifndef CONFIG_I2C_ALGOIT_POLL
	complete(&i2c_complete);
#else
	io_completed = 1;
#endif
	return IRQ_HANDLED;
}//i2c_it_handler

static int i2c_start(struct i2c_adapter* i2c_adap, struct i2c_msg* msg)
{
	int i;

	u16 flags = msg -> flags;

	for (i = i2c_adap -> retries; i; --i) {
		if (flags & I2C_M_TEN) {
			dbg("Cannot support ten bit address yet.\n");
			return 0;
		} else {
			u8 addr = (msg -> addr << 1);

			dbg("addr:= %0x flags:= %0x\n", addr, flags);
			if (flags & I2C_M_RD)
				addr |= 1;

			if (flags & I2C_M_REV_DIR_ADDR )
				addr ^= 1;

			outw((addr | I2C_ACK), IO_I2C_TXDATA);
			outw(I2C_SCS_CLKSEL_400KHZ |
			     I2C_SCS_COND_START |
			     I2C_SCS_XMIT,
			     IO_I2C_SCS);

			wait_for_completion(&i2c_complete);
			init_completion(&i2c_complete);

			dbg("I2C_RXDATA:= %x, ACK:= %x\n", inw(IO_I2C_RXDATA), I2C_ACK);
			if (!(inw(IO_I2C_RXDATA) & I2C_ACK))
				break;
		}//if/else
	}//for

	return i;
}//i2c_start

static void i2c_stop(struct i2c_adapter* i2c_adap)
{
	outw(I2C_SCS_CLKSEL_400KHZ |
	     I2C_SCS_COND_STOP |
	     I2C_SCS_XMIT,
	     IO_I2C_SCS);

	wait_for_completion(&i2c_complete);
	init_completion(&i2c_complete);
}//i2c_stop

static int sendbytes(struct i2c_adapter* i2c_adap, struct i2c_msg* msg)
{
	const u8* temp;

	int wrcount, count;

	unsigned short nak_ok = msg -> flags & I2C_M_IGNORE_NAK; 

    dbg("msg length: 0x%02X \n", msg->len);

	for (count = msg -> len, wrcount = 0, temp = msg -> buf;
	     count > 0;
	     --count, temp++, ++wrcount) {
		outw((*temp | I2C_ACK), IO_I2C_TXDATA);
		outw(I2C_SCS_CLKSEL_400KHZ |
		     I2C_SCS_XMIT,
		     IO_I2C_SCS);

		wait_for_completion(&i2c_complete);
		init_completion(&i2c_complete);

		if ((inw(IO_I2C_RXDATA) & I2C_ACK) && !nak_ok) {
			i2c_stop(i2c_adap);

			dbg("no ack from device addr: 0x%02X, d0: 0x%04X, d1: 0x%02X\n",
			    msg -> addr << 1, *(temp - 1), *temp);

			return -EREMOTEIO;
		}//if
		dbg("Acked from device 0x%02x\n for 0x%02x", msg -> addr << 1, *temp);		
	}//for

	dbg("sent bytes: 0x%02X \n", wrcount);

	return wrcount;
}//sendbytes

static int readbytes(struct i2c_adapter* i2c_adap, struct i2c_msg* msg)
{
	u8* temp = msg -> buf;

	int inval, rdcount, count = msg->len;

	dbg("read msg length: 0x%02X \n", msg->len);
	for (count = msg -> len, rdcount = 0, temp = msg -> buf;
	     count > 0;
	     --count, temp++, ++rdcount) {
		u16 ack = ((count - 1) == 0) << 8;

		outw((ack | 0xFF), IO_I2C_TXDATA);
		outw(I2C_SCS_CLKSEL_400KHZ |
		     I2C_SCS_XMIT,
		     IO_I2C_SCS);


		wait_for_completion(&i2c_complete);
		init_completion(&i2c_complete);

		udelay(2000);

		inval = inw(IO_I2C_RXDATA);

		if (!ack || (inval & I2C_ACK))
			*temp = (inval & 0xFF);
		else {
			i2c_stop(i2c_adap);

			return -ETIMEDOUT;
		}//if/else
	}//for

	dbg("read bytes: 0x%02X \n", rdcount);
	return rdcount;
}//readbytes

static int bit_xfer(struct i2c_adapter* i2c_adap,
		    struct i2c_msg msgs[],
		    int num)
{
	struct i2c_msg* pmsg;
	
	int i, ret;

	unsigned short nak_ok;

	for (i = 0; i < num; i++) {
		pmsg = &msgs[i];

		nak_ok = pmsg->flags & I2C_M_IGNORE_NAK;

		if (!(pmsg->flags & I2C_M_NOSTART)) {
		    dbg("MG- sending START.\n");
			ret = i2c_start(i2c_adap, pmsg);

			if (!ret && !nak_ok)
			  {
				dbg("MG- sending START failed. ret: %d nak_ok: %d\n", ret, nak_ok);
			    return -EREMOTEIO;
			  }
		}//if

		if (pmsg->flags & I2C_M_RD) {
			ret = readbytes(i2c_adap, pmsg);

			if (ret < pmsg -> len )
			  {
				dbg("MG- reading failed.\n");
				return (ret < 0) ? ret : -EREMOTEIO;
			  }
		} else {
			ret = sendbytes(i2c_adap, pmsg);

			if (ret < pmsg -> len )
			  {
				dbg("MG- sending failed.\n");
				return (ret < 0) ? ret : -EREMOTEIO;
			  }
		}//if/else
	}//for

	dbg("MG- sending STOP.\n");
	i2c_stop(i2c_adap);

	return num;
}//bit_xfer

static u32 bit_func(struct i2c_adapter *adap)
{
	return I2C_FUNC_SMBUS_EMUL |
	       I2C_FUNC_10BIT_ADDR | 
	       I2C_FUNC_PROTOCOL_MANGLING;
}//bit_func

/* Start Manadatory I2C Portions */
static struct i2c_algorithm i2c_it_algo = {
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,15))
        .name		= "ingenient algorithm",
	.id		= I2C_ALGO_IT,
#endif
	.master_xfer	= bit_xfer,
	.functionality	= bit_func,
};

static struct i2c_adapter i2c_it_adap = {
	.owner   = THIS_MODULE,
	.name    = "IT DM320 ADAPTER",
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,15))
	.id      =  I2C_ALGO_IT | I2C_HW_B_ITDM320,
#endif
	.algo    = &i2c_it_algo,
	.timeout = HZ,
	.retries = 3,
};


static int __init i2c_it_init (void)
{
	if (request_irq(IRQ_I2C,
			i2c_it_handler,
			SA_SAMPLE_RANDOM,
			"i2c",
			&i2c_it_adap) != 0)
		return -ENODEV;

	if (request_gio(GIO_I2C_SDA) || request_gio(GIO_I2C_SCL)) {
		free_irq(IRQ_I2C, NULL);

		return -ENODEV;
	}//if

	gio_set_dir(GIO_I2C_SDA, bit_hi);
	gio_set_dir(GIO_I2C_SCL, bit_hi);

	gio_set_fsel(GIO_I2C_SDA, 1);
	gio_set_fsel(GIO_I2C_SCL, 1);

	init_completion(&i2c_complete);

#if MG_HACK //MG -HACK to use PLL as I2C clock to slow down traffic.
	{
	  // disable I2C clock first.
	  mem_outw ((mem_inw (IO_CLK_MOD2) & (~(0x1000))),IO_CLK_MOD2);
	  
	  mem_outw ((mem_inw (IO_CLK_SEL0) | 0x0C00),IO_CLK_SEL0);
	  
	  mem_outw ((mem_inw (IO_CLK_DIV4) & (~(0x001F))) | 0x0A, IO_CLK_DIV4);
	  
	  // reenable I2C clock here.
	  mem_outw ((mem_inw (IO_CLK_MOD2) | 0x1000),IO_CLK_MOD2);
	}
#endif

	i2c_add_adapter(&i2c_it_adap);

	printk(KERN_INFO "Ingenient Technologies I2C Adapter\n");

	return 0;
}//i2c_it_init

static void __exit i2c_it_exit (void)
{

	free_irq(IRQ_I2C, &i2c_it_adap);

	i2c_del_adapter(&i2c_it_adap);

	gio_set_fsel(GIO_I2C_SDA, 0);
	gio_set_fsel(GIO_I2C_SCL, 0);

	unrequest_gio(GIO_I2C_SDA);
	unrequest_gio(GIO_I2C_SCL);
}//i2c_it_exit
/* End Manadatory I2C Portions */

MODULE_AUTHOR ("Ingenient Technologies <www.ingenient.com>");
MODULE_DESCRIPTION ("Ingenient Technologies I2C Adapter");
MODULE_LICENSE ("GPL");

module_init (i2c_it_init);
module_exit (i2c_it_exit);
