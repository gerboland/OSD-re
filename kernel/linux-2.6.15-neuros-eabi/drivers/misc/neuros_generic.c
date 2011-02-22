/*
 *  Copyright(C) 2006-2007 Neuros Technology International LLC. 
 *               <www.neurostechnology.com>
 *
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, version 2 of the License.
 *
 *  This program is distributed in the hope that, in addition to its 
 *  original purpose to support Neuros hardware, it will be useful 
 *  otherwise, but WITHOUT ANY WARRANTY; without even the implied 
 *  warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  
 *  See the GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 *
 ****************************************************************************
 *
 * Neuros generic device driver, various system controls.
 *
 * REVISION:
 *
 * 1) Initial creation. ----------------------------------- 2006-01-20 MG 
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/kthread.h>
#include <linux/jiffies.h>
#include <linux/poll.h>
#include <linux/input.h>

#include <asm/uaccess.h>
#include <asm/io.h>
#include <asm/irq.h>

#include <asm/arch/gio.h>
#include <asm/arch/hardware.h>

#include <linux/neuros_generic.h>

#define MOD_DESC "Neuros Neuros_Generic Driver (c) 2006"

#if 1
    #define dbg(fmt, arg...) \
        printk(KERN_INFO "%s:%d> " fmt, __func__, __LINE__ , ## arg)
#else
    #define dbg(fmt, arg...)
#endif


static void mem_outw (unsigned short val,unsigned long add)
{
	outw (val,add);
	wmb();
}

static unsigned short mem_inw (unsigned long add)
{
	unsigned short val;
	
	val = inw (add);
	rmb();
	return val;
}


// --- DEVICE -----------------------------------------------------------------

static int neuros_generic_open(struct inode * inode, struct file * file)
{
    return 0;
}

static int neuros_generic_release(struct inode * inode, struct file * file)
{
    return 0;
}

static ssize_t neuros_generic_read(struct file *filp, char __user *buff, size_t count, loff_t *ppos)
{ 
    return count;
}

static ssize_t neuros_generic_write(struct file * file, const char __user * buf, size_t count, loff_t *ppos)
{
	return count;
}

// clock in MHz
static void set_clock(int clock)
{
	 dbg("clock = %d\n", clock);

	 // WARNING: experimental, running clock above 202 is still risky, 
	 //                                clock above 243 is dangerous.

	 if (clock > 270)
	 {
		 // clock = 270MHz
		 mem_outw ((mem_inw (IO_CLK_PLLA) & 0xff00)|0x90, IO_CLK_PLLA);
	 }
	 else if (clock > 243)
	 {
		 // clock = 243MHz
		 mem_outw ((mem_inw (IO_CLK_PLLA) & 0xff00)|0x80, IO_CLK_PLLA);
	 }
	 else if (clock > 202)
	 {
		 // clock = 216MHz
		 mem_outw ((mem_inw (IO_CLK_PLLA) & 0xff00)|0x70, IO_CLK_PLLA);
	 }
	 else 
	 {
		 // clock = 202MHz
		 mem_outw ((mem_inw (IO_CLK_PLLA) & 0xff00)|0xe1, IO_CLK_PLLA);
	 }
	 udelay(500); // wait till PLL stablized.
}

static int neuros_generic_ioctl(struct inode * inode, struct file * file,
        unsigned int cmd, unsigned long arg)
{
    int ret = 0;
    int arg_size;

    if (_IOC_TYPE(cmd) != NEUROS_GENERIC_IOC_MAGIC)
    {
        ret = -EINVAL;
        goto bail;
    }

    arg_size = _IOC_SIZE(cmd);
    if (_IOC_DIR(cmd) & _IOC_READ)
        ret = !access_ok(VERIFY_WRITE, (void *)arg, arg_size);
    else if (_IOC_DIR(cmd) & _IOC_WRITE)
        ret = !access_ok(VERIFY_READ, (void *)arg, arg_size);
    if (ret) goto bail;

    switch (cmd)
    {
    case NT_GENERIC_SET_ARM_CLOCK:
		{
			int clock;
			copy_from_user(&clock, (void*)arg, sizeof(int));
			set_clock(clock);
			break;
		}
    default:
        ret = -EINVAL;
        break;
    }

bail:
    return ret;
}

static unsigned int neuros_generic_poll(struct file*filp, poll_table*wait)
{
    return 0;
}

static struct file_operations neuros_generic_fops = {
    .open    = neuros_generic_open,
    .release = neuros_generic_release,
    .read    = neuros_generic_read,
    .poll    = neuros_generic_poll,
    .write   = neuros_generic_write,
    .ioctl   = neuros_generic_ioctl,
};


// --- INIT / EXIT ------------------------------------------------------------

static const char * pname = "NEUROS_GENERIC(KM):";

static int __init neuros_generic_init(void)
{
    int status = 0;

    printk(KERN_INFO "\t" MOD_DESC "\n");
    status = register_chrdev(NEUROS_GENERIC_MAJOR, "neuros_ir", &neuros_generic_fops);
    if (status != 0)
    {
        if (status == -EINVAL) 
			printk(KERN_ERR "%s Couldn't register device: invalid major number %d.\n", 
				   pname, NEUROS_GENERIC_MAJOR);
        else if (status == -EBUSY) 
			printk(KERN_ERR "%s Couldn't register device: major number %d already busy.\n", 
				   pname, NEUROS_GENERIC_MAJOR);
        else printk(KERN_ERR "%s Couldn't register device: error %d.\n", pname, status);
        status = -1;
    }
    return status;
}

static void __exit neuros_generic_exit(void)
{
    unregister_chrdev(NEUROS_GENERIC_MAJOR, "neuros_generic");
}

MODULE_AUTHOR("Neuros");
MODULE_DESCRIPTION(MOD_DESC);
MODULE_LICENSE("Neuros Technology LLC");

module_init(neuros_generic_init);
module_exit(neuros_generic_exit);
