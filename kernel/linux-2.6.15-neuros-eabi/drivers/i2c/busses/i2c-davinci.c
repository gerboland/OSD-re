/*
 * linux/drivers/i2c/i2c-davinci.c
 *
 * TI DAVINCI I2C unified algorith+adapter driver
 *
 * Copyright (C) 2004 Texas Instruments.
 *
 * ----------------------------------------------------------------------------
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
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 * ----------------------------------------------------------------------------
 Modifications:
 ver. 1.0: Feb 2005, Vinod Mistral
 -
 *
 */
#include <linux/config.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/ioport.h>
#include <linux/version.h>
#include <linux/i2c.h>
#include <asm/irq.h>
#include <asm/io.h>
#include <asm/hardware/clock.h>
#include <asm/uaccess.h>
#include <linux/errno.h>
#include <linux/sched.h>
#include <asm/arch/hardware.h>
#include <linux/interrupt.h>
#include <linux/moduleparam.h>
#include <linux/err.h>
#include <linux/proc_fs.h>
#include <linux/sysctl.h>
#include <linux/wait.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>

#include <asm/arch/irqs.h>
#include <asm/arch/gio.h>

#include "i2c-davinci.h"

#define I2C_BASE        IO_ADDRESS(DAVINCI_I2C_BASE)
#define I2C_IOSIZE      (0x40)

/* For sysctl stuff */
#define	DAVINCI_I2C_DEBUG	1

/* ----- global defines ----------------------------------------------- */
#define MODULE_NAME 	        "DaVinci I2C"	/* Module name used to log messages */
#define DAVINCI_I2C_TIMEOUT     (1*HZ)	/* timeout waiting for an I2C transaction */

/* Undefine this to disable debugging */
#define	I2C_DAVINCI_DEBUG

#ifdef I2C_DAVINCI_DEBUG
static int i2c_davinci_debug = 0;
#define DEB0(format, arg...)	printk(KERN_ALERT MODULE_NAME " DEBUG: " format "\n",  ## arg )
#define DEB1(format, arg...)	\
	if (i2c_davinci_debug>=1) {	\
		printk(KERN_ALERT MODULE_NAME " DEBUG: " format "\n",  ## arg ); \
	}
#else
#define DEB0(fmt, args...)
#define DEB1(fmt, args...)
#endif

#define i2c_err(format, arg...) printk(KERN_ERR MODULE_NAME " ERROR: " format "\n",  ## arg )
#define i2c_warn(format, arg...) printk(KERN_WARNING MODULE_NAME " WARNING: " format "\n",  ## arg )

#define MAX_MESSAGES    65536	/* max number of messages */

#define I2C_DAVINCI_INTR_ALL    (DAVINCI_I2C_ICIMR_AAS_MASK | \
				 DAVINCI_I2C_ICIMR_SCD_MASK | \
				 /*DAVINCI_I2C_ICIMR_ICXRDY_MASK | */\
				 /*DAVINCI_I2C_ICIMR_ICRRDY_MASK | */\
				 DAVINCI_I2C_ICIMR_ARDY_MASK | \
				 DAVINCI_I2C_ICIMR_NACK_MASK | \
				 DAVINCI_I2C_ICIMR_AL_MASK)

/* Following are the default values for the module parameters */
#if defined( CONFIG_I2C_DAVINCI_SPEED_FAST )
static int i2c_davinci_busFreq = 400;	/* Fast Mode = 400 KHz */ 
#elif defined( CONFIG_I2C_DAVINCI_SPEED_STANDARD )
/* Standard Mode = 100 KHz */
static int i2c_davinci_busFreq = 100;	
#else
static int i2c_davinci_busFreq = 50;	
#endif

static unsigned long i2c_davinci_inputClock;	/* For I2C on DaVinci, the input clock frequency has been fixed at 27 MHz */
static int i2c_davinci_own_addr = 0x0a;	/* Randomly assigned own address */

/* Instance of the private I2C device structure */
static struct i2c_davinci_device i2c_davinci_dev;

static struct clk *i2c_clock;

static spinlock_t i2c_spinlock = SPIN_LOCK_UNLOCKED;

/*
 * This functions configures I2C and brings I2C out of reset.
 * This function is called during I2C init function. This function
 * also gets called if I2C encounetrs any errors. Clock calculation portion
 * of this function has been taken from some other driver.
 */
static int i2c_davinci_reset(struct i2c_davinci_device *dev)
{
	u16 psc;
	u32 clk;
        unsigned long flags;

        spin_lock_irqsave( &i2c_spinlock, flags );

	/* put I2C into reset */
	dev->regs->icmdr &= ~DAVINCI_I2C_ICMDR_IRS_MASK;

        /* NOTE: I2C Clock divider programming info 
 	 * As per I2C specs the following formulas provide prescalar and low/high divider values
 	 *
 	 * input clk --> PSC Div -----------> ICCL/H Div --> output clock
 	 *                       module clk
 	 *
 	 * output clk = module clk / (PSC + 1) [ (ICCL + d) + (ICCH + d) ]
 	 *
 	 * Thus,
 	 * (ICCL + ICCH) = clk = (input clk / ((psc +1) * output clk)) - 2d;
 	 *
 	 * where if PSC == 0, d = 7, 
 	 *       if PSC == 1, d = 6
 	 *       if PSC > 1 , d = 5
 	 */

	psc = 2; /* To get 9MHz clock */

        clk = ((i2c_davinci_inputClock/(psc + 1)) / (i2c_davinci_busFreq * 1000)) - 10;

	dev->regs->icpsc = psc;
	dev->regs->icclkh = (27 * clk) / 100; /* duty cycle should be 27% */
        dev->regs->icclkl = (clk - dev->regs->icclkh);

	DEB1("CLK  = %d\n", clk);
	DEB1("PSC  = %d\n", dev->regs->icpsc);
	DEB1("CLKL = %d\n", dev->regs->icclkl);
	DEB1("CLKH = %d\n", dev->regs->icclkh);

	/* Set Own Address: */
	dev->regs->icoar = i2c_davinci_own_addr;

	/* Enable interrupts */
	dev->regs->icimr = I2C_DAVINCI_INTR_ALL;

	/* Take the I2C module out of reset: */
	dev->regs->icmdr |= DAVINCI_I2C_ICMDR_IRS_MASK;
        
        spin_unlock_irqrestore( &i2c_spinlock, flags );

	return 0;
}

/*
 * Waiting on Bus Busy
 */
static int i2c_davinci_wait_for_bb(char allow_sleep)
{
	unsigned long timeout;

	timeout = jiffies + DAVINCI_I2C_TIMEOUT;
	while ((i2c_davinci_dev.regs->icstr) & DAVINCI_I2C_ICSTR_BB_MASK) {
		if (time_after(jiffies, timeout)) {
			i2c_warn("timeout waiting for bus ready, send stop bit.");

                        if ( allow_sleep ) {
                            init_completion( &i2c_davinci_dev.cmd_completion );
                        }

                        /* The BB bit can become stuck high is the I2C is reset */
                        /* while the SCL line is low, */
                        /* In a single-master system, we can safely send out a STOP here. */
                        i2c_davinci_dev.regs->icmdr = 
                            DAVINCI_I2C_ICMDR_TRX_MASK |
                            DAVINCI_I2C_ICMDR_IRS_MASK | DAVINCI_I2C_ICMDR_MST_MASK |
                            DAVINCI_I2C_ICMDR_RM_MASK | DAVINCI_I2C_ICMDR_STP_MASK;

                        if ( allow_sleep ) {
                            /* wait for the transaction to complete */
                            (void) wait_for_completion_interruptible_timeout( &i2c_davinci_dev.cmd_completion, 
                                                   DAVINCI_I2C_TIMEOUT );
                        }
                        i2c_davinci_dev.regs->icstr |= DAVINCI_I2C_ICSTR_BB_MASK;
			return -ETIMEDOUT;
		}
		if (allow_sleep)
			schedule_timeout(1);
	}

	return 0;
}

/*
 * Low level master read/write transaction. This function is called
 * from i2c_davinci_xfer.
 */
static int
i2c_davinci_xfer_msg(struct i2c_adapter *adap, struct i2c_msg *msg, int stop)
{
	struct i2c_davinci_device *dev = i2c_get_adapdata(adap);
	u8 zero_byte = 0;
	u32 flag = 0, stat = 0;
        unsigned long flags;
        int r;
        int cnt = 2000;

	/* Introduce a 20musec delay.  Required for Davinci EVM */
	while (cnt--);

	DEB1("addr: 0x%04x, len: %d, flags: 0x%x, stop: %d",
	     msg->addr, msg->len, msg->flags, stop);

        spin_lock_irqsave( &i2c_spinlock, flags );

	/* set the slave address */
	dev->regs->icsar = msg->addr;

	/* Sigh, seems we can't do zero length transactions. Thus, we
	 * can't probe for devices w/o actually sending/receiving at least
	 * a single byte. So we'll set count to 1 for the zero length
	 * transaction case and hope we don't cause grief for some
	 * arbitrary device due to random byte write/read during
	 * probes.
	 */
	if (msg->len == 0) {
		dev->buf = &zero_byte;
		dev->buf_len = 1;
	} else {
		dev->buf = msg->buf;
		dev->buf_len = msg->len;
	}

	dev->regs->iccnt = dev->buf_len;
	dev->cmd_err = 0;
        init_completion( &dev->cmd_completion );

	/* Clear any pending interrupts by reading the IVR */
	stat = dev->regs->icivr;

	/* Take I2C out of reset, configure it as master and set the start bit */
	flag =
	    DAVINCI_I2C_ICMDR_IRS_MASK | DAVINCI_I2C_ICMDR_MST_MASK |
	    DAVINCI_I2C_ICMDR_STT_MASK;

	/* if the slave address is ten bit address, enable XA bit */
	if (msg->flags & I2C_M_TEN)
		flag |= DAVINCI_I2C_ICMDR_XA_MASK;
	if (!(msg->flags & I2C_M_RD))
		flag |= DAVINCI_I2C_ICMDR_TRX_MASK;
	if (stop)
		flag |= DAVINCI_I2C_ICMDR_STP_MASK;

	/* write the data into mode register */
	dev->regs->icmdr = flag;

	/* Enable receive and transmit interrupts */
	if (msg->flags & I2C_M_RD)
		dev->regs->icimr |= DAVINCI_I2C_ICIMR_ICRRDY_MASK;
	else {
		dev->regs->icimr |= DAVINCI_I2C_ICIMR_ICXRDY_MASK;

     		/* Prime the pump */
		if ( dev->regs->icstr & DAVINCI_I2C_ICSTR_ICXRDY_MASK ) {
		  dev->regs->icdxr = *dev->buf++;
		  dev->buf_len--;
		}
	}

        spin_unlock_irqrestore( &i2c_spinlock, flags );

	/* wait for the transaction to complete */
	r = wait_for_completion_interruptible_timeout( &dev->cmd_completion, 
                                                   DAVINCI_I2C_TIMEOUT );

	dev->buf_len = 0;

        if ( r < 0 ) {
            return r;
        }
	if (r == 0 ) {
                printk( "I2C command timeout, icivr=0x%04x, status=0x%04x\n",
                        dev->regs->icivr, dev->regs->icstr );
		i2c_davinci_reset(dev);
		return -ETIMEDOUT;
	}

	/* no error */
	if (!dev->cmd_err)
		return msg->len;

	/* We have an error */
	if (dev->cmd_err & DAVINCI_I2C_ICSTR_NACK_MASK) {
		if (msg->flags & I2C_M_IGNORE_NAK)
			return msg->len;
		if (stop)
			dev->regs->icmdr |= DAVINCI_I2C_ICMDR_STP_MASK;
		return -EREMOTEIO;
	}
	if (dev->cmd_err & DAVINCI_I2C_ICSTR_AL_MASK ||
	    dev->cmd_err & DAVINCI_I2C_ICSTR_RSFULL_MASK) {
		i2c_davinci_reset(dev);
		return -EIO;
	}
	return msg->len;
}

/*
 * Prepare controller for a transaction and call i2c_davinci_xfer_msg

 */
static int
i2c_davinci_xfer(struct i2c_adapter *adap, struct i2c_msg msgs[], int num)
{
	int count;
	int ret = 0;

	DEB1("msgs: %d", num);

	if (num < 1 || num > MAX_MESSAGES)
		return -EINVAL;

	/* Check for valid parameters in messages */
	for (count = 0; count < num; count++)
		if (msgs[count].buf == NULL)
			return -EINVAL;

	if ((ret = i2c_davinci_wait_for_bb(1)) < 0)
		return ret;

	for (count = 0; count < num; count++) {
		DEB1("msg: %d, addr: 0x%04x, len: %d, flags: 0x%x",
		     count, msgs[count].addr, msgs[count].len,
		     msgs[count].flags);

		ret =
		    i2c_davinci_xfer_msg(adap, &msgs[count],
					 (count == (num - 1)));

		DEB1("ret: %d", ret);

		if (ret != msgs[count].len)
			break;
	}

	if (ret >= 0 && num > 1)
		ret = num;

	DEB1("ret xfer: %d", ret);

	return ret;
}

static u32 i2c_davinci_func(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_BYTE_DATA;
}

/*
 * This function marks a transaction as complete.
 */
static inline void i2c_davinci_complete_cmd(struct i2c_davinci_device *dev)
{
	complete(&dev->cmd_completion);
}

/*
 * Interrupt service routine. This gets called whenever an I2C interrupt
 * occurs.
 */
static irqreturn_t
i2c_davinci_isr(int this_irq, void *dev_id, struct pt_regs *reg)
{
	struct i2c_davinci_device *dev = dev_id;
	u32 stat;

	DEB1("i2c_davinci_isr()\n");

	while ((stat = dev->regs->icivr) != 0) {
//                printk( "\t%02x\n", stat );
		switch (stat) {
		case DAVINCI_I2C_ICIVR_INTCODE_AL:
			dev->cmd_err |= DAVINCI_I2C_ICSTR_AL_MASK;
			i2c_davinci_complete_cmd(dev);
			break;

		case DAVINCI_I2C_ICIVR_INTCODE_NACK:
			dev->cmd_err |= DAVINCI_I2C_ICSTR_NACK_MASK;
			i2c_davinci_complete_cmd(dev);
			break;

		case DAVINCI_I2C_ICIVR_INTCODE_RAR:
                        dev->regs->icstr |= DAVINCI_I2C_ICSTR_ARDY_MASK;
                        i2c_davinci_complete_cmd( dev );
                        break;

		case DAVINCI_I2C_ICIVR_INTCODE_RDR:
			if (dev->buf_len) {
				*dev->buf++ = dev->regs->icdrr;
				dev->buf_len--;
				if (dev->buf_len) {
					continue;
				} else {
					dev->regs->icimr &=
					    ~DAVINCI_I2C_ICIMR_ICRRDY_MASK;
				}
			}
			break;

		case DAVINCI_I2C_ICIVR_INTCODE_TDR:
			if (dev->buf_len) {
				dev->regs->icdxr = *dev->buf++;
				dev->buf_len--;
				if (dev->buf_len)
					continue;
				else {
					dev->regs->icimr &=
					    ~DAVINCI_I2C_ICIMR_ICXRDY_MASK;
                                        /* If no stop bit, then we are done ... */
                                        if ( !(dev->regs->icmdr & DAVINCI_I2C_ICMDR_STP_MASK) ) {
                                            i2c_davinci_complete_cmd( dev );
                                        }
				}
			}
			break;

		case DAVINCI_I2C_ICIVR_INTCODE_SCD:
                        dev->regs->icstr |= DAVINCI_I2C_ICSTR_SCD_MASK;
                        i2c_davinci_complete_cmd(dev);
                        break;

		case DAVINCI_I2C_ICIVR_INTCODE_AAS:
			i2c_warn("Address as slave interrupt");
			break;

		default:
                    printk( "Unknown status 0x%04x\n", stat );
			break;
		}		/* switch */
	}			/* while */
	return IRQ_HANDLED;
}

static int davinci_i2c_remove(struct device *dev)
{
	return 0;
}

static void davinci_i2c_device_release(struct device *dev)
{
	/* Nothing */
}

static struct i2c_algorithm i2c_davinci_algo = {
//	.name = "DAVINCI I2C algorithm",
//	.id = I2C_ALGO_EXP,
	.master_xfer = i2c_davinci_xfer,
	.smbus_xfer = NULL,
	.slave_send = NULL,
	.slave_recv = NULL,
	.algo_control = NULL,
	.functionality = i2c_davinci_func,
};

static struct i2c_adapter i2c_davinci_adap = {
	.owner = THIS_MODULE,
	.name = "DAVINCI I2C adapter",
//	.id = I2C_ALGO_EXP,
	.algo = &i2c_davinci_algo,
	.algo_data = NULL,
	.client_register = NULL,
	.client_unregister = NULL,
};

static struct device_driver davinci_i2c_driver = {
	.name = "davinci_i2c",
	.bus = &platform_bus_type,
	.remove = davinci_i2c_remove,
};

static struct platform_device davinci_i2c_device = {
	.name = "i2c",
	.id = -1,
	.dev = {
		.driver = &davinci_i2c_driver,
		.release = davinci_i2c_device_release,
		},
};

int davinci_i2c_fix_ths7353_lockup( void )
{
    printk( "Toggle I2C clock to clear THS7353 false ACK\n" );

    PINMUX1 &= ~(1 << 7);

    request_gio( GIO_I2C_SCL );
    request_gio( GIO_I2C_SDA );

    gio_set_bitset( GIO_I2C_SCL );
    gio_set_bitset( GIO_I2C_SDA );

    gio_set_dir( GIO_I2C_SCL, 0 );
    gio_set_dir( GIO_I2C_SDA, 0 );

    gio_set_bitclr( GIO_I2C_SCL );
    udelay( 100 );
    gio_set_bitset( GIO_I2C_SCL );
    udelay( 100 );
    gio_set_bitclr( GIO_I2C_SCL );
    udelay( 100 );
    gio_set_bitset( GIO_I2C_SCL );
    udelay( 100 );

    unrequest_gio( GIO_I2C_SCL );
    unrequest_gio( GIO_I2C_SDA );

    udelay( 100 );
        
    return 0;
}

static int __init i2c_davinci_init(void)
{
	int status;
	struct device 	*dev = NULL;

	DEB0("%s %s", __TIME__, __DATE__);

	DEB1("i2c_davinci_init()\n");

        davinci_i2c_fix_ths7353_lockup( );

#if 0
	if (i2c_davinci_busFreq > 200)
		i2c_davinci_busFreq = 400;	/*Fast mode */
	else
		i2c_davinci_busFreq = 100;	/*Standard mode */
#endif

	i2c_clock = clk_get (dev, "I2CCLK");	

	if (i2c_clock == NULL)
        	return -1;
        
	clk_use (i2c_clock);
	clk_enable (i2c_clock);
	i2c_davinci_inputClock = clk_get_rate (i2c_clock);

	DEB1 ("IP CLOCK = %ld\n", i2c_davinci_inputClock);

	memset(&i2c_davinci_dev, 0, sizeof(i2c_davinci_dev));

	i2c_davinci_dev.regs = (davinci_i2cregsovly)I2C_BASE;

	status = (int)request_region(I2C_BASE, I2C_IOSIZE, MODULE_NAME);
	if (!status) {
		i2c_err("I2C is already in use\n");
		return -ENODEV;
	}

	status = request_irq(IRQ_I2C, i2c_davinci_isr, 0, MODULE_NAME,
			     &i2c_davinci_dev);
	if (status) {
		i2c_err("failed to request I2C IRQ");
		goto do_release_region;
	}

	i2c_set_adapdata(&i2c_davinci_adap, &i2c_davinci_dev);
	status = i2c_add_adapter(&i2c_davinci_adap);
	if (status) {
		i2c_err("failed to add adapter");
		goto do_free_irq;
		return status;
	}

	i2c_davinci_reset(&i2c_davinci_dev);

	if (driver_register(&davinci_i2c_driver) != 0)
		printk(KERN_ERR "Driver register failed for davinci_i2c\n");
	if (platform_device_register(&davinci_i2c_device) != 0) {
		printk(KERN_ERR "Device register failed for i2c\n");
		driver_unregister(&davinci_i2c_driver);
	}

	return 0;

      do_free_irq:
	free_irq(IRQ_I2C, &i2c_davinci_dev);
      do_release_region:
	release_region(I2C_BASE, I2C_IOSIZE);

	return status;
}

static void __exit i2c_davinci_exit(void)
{
	struct i2c_davinci_device dev;

        clk_disable (i2c_clock);
        clk_unuse (i2c_clock);        
	i2c_del_adapter(&i2c_davinci_adap);
	dev.regs->icmdr = 0;
	free_irq(IRQ_I2C, &i2c_davinci_dev);
	release_region(I2C_BASE, I2C_IOSIZE);
        driver_unregister(&davinci_i2c_driver);
        platform_device_unregister(&davinci_i2c_device);
}

MODULE_AUTHOR("Texas Instruments India");
MODULE_DESCRIPTION("TI DAVINCI I2C bus adapter");
MODULE_LICENSE("GPL");

module_param(i2c_davinci_own_addr, int, 0);
MODULE_PARM_DESC(i2c_davinci_own_addr, "Set I2C own address");

module_param(i2c_davinci_busFreq, int, 0);
MODULE_PARM_DESC(i2c_davinci_busFreq,
		 "Set I2C bus frequency in KHz: 100 (Standard Mode) or 400 (Fast Mode)");

#if 0
module_param(i2c_davinci_inputClock, int, 0);
MODULE_PARM_DESC(i2c_davinci_inputClock,
		 "Set I2C input frequency in KHz: For I2C on DaVinci, the input clock frequency has been fixed at 27 MHz");
#endif

module_param(i2c_davinci_debug, int, 0);
MODULE_PARM_DESC(i2c_davinci_debug, "debug level - 0 off; 1 verbose;");

/* i2c may be needed to bring up other drivers */
subsys_initcall(i2c_davinci_init);
module_exit(i2c_davinci_exit);
