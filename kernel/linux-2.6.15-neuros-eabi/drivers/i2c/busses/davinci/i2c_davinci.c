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
#include <asm/arch/irqs.h>
#include "i2c_davinci.h"

#define USE_I2C_XFER_DAVINCI_BUSY_WAIT_HACK 1

/* For Virtio usage */
//#define DAVINCI_VIRTIO

/* For sysctl stuff */
#define	DAVINCI_I2C_DEBUG	1

/* ----- global defines ----------------------------------------------- */
#define MODULE_NAME 	        "DaVinci I2C"	/* Module name used to log messages */
#define I2C_DAVINCI_TIMEOUT     (1*HZ)		/* timeout waiting for an I2C transaction */

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

#define MAX_MESSAGES    65536			/* max number of messages */
#define	INT_I2C		IRQ_I2C //DAVINCI_IICINT_INTERRUPT  	/* I2C Interrupt No. from soc.h */

#define I2C_DAVINCI_INTR_ALL    (DAVINCI_I2C_ICIMR_AAS_MASK | \
				 DAVINCI_I2C_ICIMR_SCD_MASK | \
				 /*DAVINCI_I2C_ICIMR_ICXRDY_MASK | */\
				 /*DAVINCI_I2C_ICIMR_ICRRDY_MASK | */\
				 /*DAVINCI_I2C_ICIMR_ARDY_MASK | */\
				 DAVINCI_I2C_ICIMR_NACK_MASK | \
				 DAVINCI_I2C_ICIMR_AL_MASK)

/* Following are the default values for the module parameters */
static int i2c_davinci_busFreq 		= 100;		/* Default: Fast Mode = 400 KHz, Standard Mode = 100 KHz */
static int i2c_davinci_inputClock 	= 27000;	/* For I2C on DaVinci, the input clock frequency has been fixed at 27 MHz */
static int i2c_davinci_own_addr 	= 0x21;		/* Randomly assigned own address */

/* Instance of the private I2C device structure */
static struct i2c_davinci_device i2c_davinci_dev;

#undef I2C_DAVINCI_USE_INTERRUPTS

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

        printk( KERN_ERR "dev->regs = 0x%0x\n", dev->regs  );

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
 	 * ICCL = ICCH = clk = input clk / ( (psc +1) * output clk * 2 ) ;
 	 *
 	 * where if PSC == 0, d = 7,
 	 *       if PSC == 1, d = 6
 	 *       if PSC > 1 , d = 5
 	 */

	/* clock calculations */
	for (psc = 0; psc < 256; psc ++)
	{
		/* prescalar is a 8 bit register and can hold maximum 255 */
		if (psc > 255)
			return 1; 	/* Cannot go so high in divider value */

		clk = i2c_davinci_inputClock / ((psc + 1) * i2c_davinci_busFreq * 2);
		/* ICCL = ICCH = 16 bit register and can hold maximum 65535 */
		if (clk < 65535)
			break;
	}

	switch (psc)
	{
	case 0:
		clk = clk - 7;
		break;

	case 1:
		clk = clk - 6;
		break;

	default:
		clk = clk - 5;
		break;
	}

	/* CLKL/CLKH are gettting 0 or negative values - flag error */
	if (clk <= 0)
		return 1;

	/* setup clocks */
	/*Rishi - change hard coded values*/
	//	dev->regs->icpsc = psc;
	dev->regs->icpsc = 0x1a;
	/*Rishi - change hard coded values*/
	//dev->regs->icclkh = clk;
	//dev->regs->icclkl = (clk >> 16);
	dev->regs->icclkh = 0xa;
	dev->regs->icclkl = 0xa;

	/* Set Own Address: */
	dev->regs->icoar = i2c_davinci_own_addr;

	/* Enable interrupts */
#ifdef I2C_DAVINCI_USE_INTERRUPTS
	dev->regs->icimr = I2C_DAVINCI_INTR_ALL;
#else
        dev->regs->icimr = 0;
#endif
	/* Take the I2C module out of reset: */
	dev->regs->icmdr |= DAVINCI_I2C_ICMDR_IRS_MASK;

	return 0;
}

/*
 * Waiting on Bus Busy
 */
static int
i2c_davinci_wait_for_bb(char allow_sleep)
{

#if 0
	unsigned long timeout;
	timeout = jiffies + I2C_DAVINCI_TIMEOUT;
	while ((i2c_davinci_dev.regs->icstr) & DAVINCI_I2C_ICSTR_BB_MASK) {
		if (time_after(jiffies, timeout)) {
			i2c_warn("timeout waiting for bus ready");
			return -ETIMEDOUT;
		}
		if (allow_sleep)
			schedule_timeout(1);
	}
#endif
	return 0;
}

#ifdef I2C_DAVINCI_USE_INTERRUPTS
/*
 * Low level master read/write transaction. This function is called
 * from i2c_davinci_xfer.
 */
static int
i2c_davinci_xfer_msg(struct i2c_adapter *adap, struct i2c_msg *msg, int stop)
{
	struct i2c_davinci_device *dev = i2c_get_adapdata(adap);
	u8 zero_byte = 0;
	int retVal = 0;
	u32 flag = 0, stat = 0;

	DEB1("addr: 0x%04x, len: %d, flags: 0x%x, stop: %d",
	     msg->addr, msg->len, msg->flags, stop);

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
	dev->cmd_complete = 0;
	dev->cmd_err = 0;

	/* Clear any pending interrupts by reading the IVR */
	stat = dev->regs->icivr;

	/* Take I2C out of reset, configure it as master and set the start bit */
	flag = DAVINCI_I2C_ICMDR_IRS_MASK | DAVINCI_I2C_ICMDR_MST_MASK | DAVINCI_I2C_ICMDR_STT_MASK;

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
	else
		dev->regs->icimr |= DAVINCI_I2C_ICIMR_ICXRDY_MASK;

	/* wait for the transaction to complete */
	retVal = wait_for_completion_timeout(&dev->cmd_wait, I2C_DAVINCI_TIMEOUT);
        init_completion(&dev->cmd_wait);

	dev->buf_len = 0;

	/* this is the case of timeout */
	if (!retVal)
    	    	return msg->len;

	if (!dev->cmd_complete) {
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
#else
/*
 * Low level master read/write transaction. This function is called
 * from i2c_davinci_xfer.
 */
static int
i2c_davinci_xfer_msg(struct i2c_adapter *adap, struct i2c_msg *msg, int stop)
{
	struct i2c_davinci_device *dev = i2c_get_adapdata(adap);
	u8 zero_byte = 0;
	int retVal = msg->len;
	u32 flag = 0, stat = 0;

#if USE_I2C_XFER_DAVINCI_BUSY_WAIT_HACK == 1
    unsigned long timeoutJiffies = 3;
#endif
    unsigned long startJiffies;

	DEB1("addr: 0x%04x, len: %d, flags: 0x%x, stop: %d",
	     msg->addr, msg->len, msg->flags, stop);

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
	dev->cmd_complete = 0;
	dev->cmd_err = 0;

	/* Clear any pending interrupts by reading the IVR */
	stat = dev->regs->icivr;

	/* Take I2C out of reset, configure it as master and set the start bit */
	flag = DAVINCI_I2C_ICMDR_IRS_MASK | DAVINCI_I2C_ICMDR_MST_MASK | DAVINCI_I2C_ICMDR_STT_MASK;

	/* if the slave address is ten bit address, enable XA bit */
	if (msg->flags & I2C_M_TEN)
		flag |= DAVINCI_I2C_ICMDR_XA_MASK;
	if (!(msg->flags & I2C_M_RD))
		flag |= DAVINCI_I2C_ICMDR_TRX_MASK;

	/* write the data into mode register */
	dev->regs->icmdr = flag;

    startJiffies = jiffies;
    
	/* wait for the transaction to complete */
	if ((msg->flags & I2C_M_RD)) {
          for( ; dev->buf_len > 0 ; --dev->buf_len ) {
            while ( 0 == (dev->regs->icstr & (DAVINCI_I2C_ICSTR_ICRRDY_MASK | 
                                              DAVINCI_I2C_ICSTR_NACK_MASK) ) ) {
#if USE_I2C_XFER_DAVINCI_BUSY_WAIT_HACK == 1
#warning Busy Waiting. Is there a better way?
                if ( (jiffies - startJiffies) >= timeoutJiffies ) {
                    printk( KERN_ERR "I2C write timeout: address 0x%02x\n",
                             msg->addr );
                    retVal = -ETIMEDOUT;         
                    break;
                }
#else
                schedule_timeout(1);
#endif
            }
            *dev->buf++ = dev->regs->icdrr;
          }
        } else {
            for( ; dev->buf_len > 0 ; --dev->buf_len ) {
              dev->regs->icdxr = *dev->buf++;
              while ( 0 == (dev->regs->icstr & (DAVINCI_I2C_ICSTR_ICXRDY_MASK | 
                                                DAVINCI_I2C_ICSTR_NACK_MASK )) ) {
#if USE_I2C_XFER_DAVINCI_BUSY_WAIT_HACK == 1
#warning Busy Waiting. Is there a better way?
                if ( (jiffies - startJiffies) >= timeoutJiffies ) {
                    printk( KERN_ERR "I2C write timeout: address 0x%02x\n",
                             msg->addr );
                    retVal = -ETIMEDOUT;         
                    break;
                }
#else
                schedule_timeout(1);
#endif
              }
            }
        }

        if ( dev->regs->icstr & DAVINCI_I2C_ICSTR_NACK_MASK ) {
          retVal = -ENXIO;
          /* Clear interrupt */
          dev->regs->icstr = DAVINCI_I2C_ICSTR_NACK_MASK;
        }

        if (stop)
		dev->regs->icmdr |= DAVINCI_I2C_ICMDR_STP_MASK;

	return retVal;
}

#endif

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
		     count, msgs[count].addr, msgs[count].len, msgs[count].flags);

		ret = i2c_davinci_xfer_msg(adap, &msgs[count], (count == (num - 1)));

		DEB1("ret: %d", ret);

	        if (ret != msgs[count].len )
                  break;
	}

	if (ret >= 0 && num > 1)
		ret = num;

	DEB1("ret: %d", ret);

	return ret;
}

static u32
i2c_davinci_func(struct i2c_adapter *adap)
{
    return I2C_FUNC_I2C | I2C_FUNC_SMBUS_BYTE_DATA;// | I2C_FUNC_SMBUS_QUICK; //I2C_FUNC_SMBUS_EMUL;
}

/*
 * This function marks a transaction as complete.
 */
static inline void
i2c_davinci_complete_cmd(struct i2c_davinci_device *dev)
{
	dev->cmd_complete = 1;
	complete(&dev->cmd_wait);
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
			break;

		case DAVINCI_I2C_ICIVR_INTCODE_RDR:
			if (dev->buf_len) {
				*dev->buf++ = dev->regs->icdrr;
				dev->buf_len--;
				if (dev->buf_len)
					continue;
		   	        else
		   	        {
					i2c_davinci_complete_cmd(dev);
					dev->regs->icimr &= ~DAVINCI_I2C_ICIMR_ICRRDY_MASK;
				}
		        }
			break;

		case DAVINCI_I2C_ICIVR_INTCODE_TDR:
			if (dev->buf_len) {
#ifdef DAVINCI_VIRTIO
				if (dev->regs->icstr & DAVINCI_I2C_ICSTR_ICXRDY_MASK)
#endif
	        	        dev->regs->icdxr = *dev->buf++;
#ifdef DAVINCI_VIRTIO
	       			else
	       				continue;
#endif
				dev->buf_len--;
				if (dev->buf_len)
	       	        		continue;
				else
	                        {
					i2c_davinci_complete_cmd(dev);
					dev->regs->icimr &= ~DAVINCI_I2C_ICIMR_ICXRDY_MASK;
				}
			}
			break;

		case DAVINCI_I2C_ICIVR_INTCODE_SCD:
			break;

		case DAVINCI_I2C_ICIVR_INTCODE_AAS:
			i2c_warn("Addredd as slave interrupt");
			break;

		default:
			break;
		} /* switch */
	} /* while */
	return IRQ_HANDLED;
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

static int __init
i2c_davinci_init(void)
{
	int status;

	DEB0("%s %s", __TIME__, __DATE__);

	DEB1 ("i2c_davinci_init()\n");

	if (i2c_davinci_busFreq > 200)
		i2c_davinci_busFreq = 400;	/*Fast mode */
	else
		i2c_davinci_busFreq = 100;	/*Standard mode */

	memset(&i2c_davinci_dev, 0, sizeof(i2c_davinci_dev));
	init_completion(&i2c_davinci_dev.cmd_wait);

	i2c_davinci_sysctl_register();

	i2c_davinci_dev.regs = (davinci_i2cregsovly)DAVINCI_I2C_VADDR;

	status = (int) request_region (DAVINCI_I2C_BASE, DAVINCI_I2C_IOSIZE, MODULE_NAME);
	if (!status) {
		i2c_err ("I2C is already in use\n");
		return -ENODEV;
	}

	status = request_irq(INT_I2C, i2c_davinci_isr, 0, MODULE_NAME, &i2c_davinci_dev);
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

	return 0;

do_free_irq:
	free_irq(INT_I2C, &i2c_davinci_dev);
do_release_region:
    	release_region(DAVINCI_I2C_BASE, DAVINCI_I2C_IOSIZE);

	return status;
}

static void __exit
i2c_davinci_exit(void)
{
	struct i2c_davinci_device dev;

	i2c_del_adapter(&i2c_davinci_adap);
	dev.regs->icmdr = 0;
	free_irq(INT_I2C, &i2c_davinci_dev);
	i2c_davinci_sysctl_unregister();
}


/******************************************************************************
 *                                                                            *
 *		    The below portion of code is for the sysctl support               *
 *                                                                            *
 *****************************************************************************/

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,5)
static int i2c_sysctl_handler(ctl_table *ctl, int write, struct file * filp,
			      void __user *buffer, size_t *lenp, loff_t *ppos)
#else
static int i2c_sysctl_handler(ctl_table *ctl, int write, struct file * filp,
			      void __user *buffer, size_t *lenp)
#endif
{
	int ret;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,5)
	ret = proc_dointvec(ctl, write, filp, buffer, lenp, ppos);
#else
	ret = proc_dointvec(ctl, write, filp, buffer, lenp);
#endif

	if (write) {
    	        int *valp = ctl->data;
		int val = *valp;

		switch (ctl->ctl_name) {
        	case DAVINCI_I2C_DEBUG:
            	        if (val == 1)
	                        i2c_davinci_debug = 1;
                        if (val == 2)
                                i2c_davinci_debug = 2;
                        break;
		}
	}

	return ret;
}

struct i2c_sysctl_debug_settings {
		int     test;
} i2c_sysctl_debug_settings;

ctl_table i2c_debug_table[] = {
	{
	    	.ctl_name       = DAVINCI_I2C_DEBUG,
	        .procname       = "i2c_Debug",
	        .data           = &i2c_sysctl_debug_settings.test,
	        .maxlen         = sizeof(int),
	        .mode           = 0644,
	        .proc_handler   = &i2c_sysctl_handler,
	},
	{
		.ctl_name = 0
	}
};


static int testDefault = 0;
static struct ctl_table_header *davinci_i2c_sysctl_header;

static void i2c_davinci_sysctl_register(void)
{
	static int initialized;

   	if (initialized == 1)
  	        return;

	davinci_i2c_sysctl_header = register_sysctl_table(i2c_debug_table, 1);

	/* set the defaults */
	i2c_sysctl_debug_settings.test = testDefault;
	initialized = 1;
}

static void i2c_davinci_sysctl_unregister(void)
{
	if (davinci_i2c_sysctl_header)
  	        unregister_sysctl_table(davinci_i2c_sysctl_header);
}

MODULE_AUTHOR("Texas Instruments India");
MODULE_DESCRIPTION("TI DAVINCI I2C bus adapter");
MODULE_LICENSE("GPL");

module_param(i2c_davinci_own_addr, int, 0);
MODULE_PARM_DESC(i2c_davinci_own_addr, "Set I2C own address");

module_param(i2c_davinci_busFreq, int, 0);
MODULE_PARM_DESC(i2c_davinci_busFreq, "Set I2C bus frequency in KHz: 100 (Standard Mode) or 400 (Fast Mode)");

module_param(i2c_davinci_inputClock, int, 0);
MODULE_PARM_DESC(i2c_davinci_inputClock, "Set I2C input frequency in KHz: For I2C on DaVinci, the input clock frequency has been fixed at 27 MHz");

module_param(i2c_davinci_debug, int, 0);
MODULE_PARM_DESC(i2c_davinci_debug, "debug level - 0 off; 1 verbose;");

/* i2c may be needed to bring up other drivers */
subsys_initcall(i2c_davinci_init);
module_exit(i2c_davinci_exit);




