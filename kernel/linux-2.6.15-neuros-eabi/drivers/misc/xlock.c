/*
 *  Copyright(C) 2006 Neuros Technology International LLC. 
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
 * A driver for lock
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/fs.h>
#include <asm/uaccess.h>

#define DEVICE_NAME "xlock"
#define XLOCK_MAJOR 103

DECLARE_MUTEX(keybuf_sem);
static ssize_t flock_write(struct file * file, const char __user * buf, size_t count,
			 loff_t *ppos)
{
  int key;
  
  if (copy_from_user(&key, buf, sizeof(int)))
    goto bail;
  if (key == 0)
    {
      //printk("here is unlock\n");
      up(&keybuf_sem);
      //printk("here is unlock returned\n");
    }
  else
    {
      //printk("here is lock\n");
      down_interruptible(&keybuf_sem);
      //printk("here is lock returned\n");
      
    }
 bail:
  return count;
}

static struct file_operations xlock_fops = 
{
  .owner   = THIS_MODULE,
  .write   = flock_write,
};

static int __init xlock_init(void)
{
  int ret;
  ret = register_chrdev(XLOCK_MAJOR, DEVICE_NAME, &xlock_fops);
  return ret;
}

static void __exit xlock_exit(void)
{
  int ret;
  
  ret = unregister_chrdev(XLOCK_MAJOR, DEVICE_NAME);
  if (ret < 0)
    printk(KERN_ALERT "Unregister error: %d\n", ret);
}

MODULE_AUTHOR("Neuros Technologies");
MODULE_LICENSE("GPL");

module_init(xlock_init);
module_exit(xlock_exit);
