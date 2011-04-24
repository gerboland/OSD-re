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

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>

#include <asm/io.h>
#include <asm/uaccess.h>

#include <asm/arch/gio.h>
#include <asm/arch/leds.h>

#define IT_MAJOR_DEV 100

void it_led_off(unsigned char led)
{
	gio_set_bitclr(led);
}//it_led_off
EXPORT_SYMBOL(it_led_off);

void it_led_on(unsigned char led)
{
	gio_set_bitset(led);
}//it_led_on
EXPORT_SYMBOL(it_led_on);

static int it_led_open(struct inode* inode, struct file* file)
{
	int ret = 0;

	return ret;
}//it_led_open

static int it_led_release(struct inode* inode, struct file* file)
{
	int ret = 0;

	return ret;
}//it_led_release

static int it_led_ioctl(struct inode* inode,
			struct file* file,
			unsigned int cmd,
			unsigned long arg)
{
	int ret = 0;
	int arg_size;

	if (_IOC_TYPE(cmd) != IT_LED_IOC_MAGIC) return -EINVAL;
	if (_IOC_NR(cmd) > IT_LED_IOC_NR) return -EINVAL;

	/*  The direction is a bitmask, and VERIFY_WRITE catches R/W
	 *  transfers. 'Type' is user-oriented, while verify_area
	 *  is kernel-oriented, so the concept of "read" and "write" is
	 *  reversed 
	 */
	arg_size = _IOC_SIZE(cmd);

	if (_IOC_DIR(cmd) & _IOC_READ)
		ret = !access_ok(VERIFY_WRITE, (void*) arg, arg_size);
	else if (_IOC_DIR(cmd) & _IOC_WRITE)
		ret = !access_ok(VERIFY_READ, (void*) arg, arg_size);

	if (ret) return ret;
   
	switch (cmd) {
		case IT_IOCLED_ON:
			it_led_on(arg);

			break;
		case IT_IOCLED_OFF:
			it_led_off(arg);

			break;
		default: return -EINVAL;
	}//switch

	return ret;
}//it_led_ioctl

static struct file_operations it_led_fileops = {
	owner    : THIS_MODULE,
	ioctl    : it_led_ioctl,
	open     : it_led_open,
	release  : it_led_release,
};//it_usb_fileops

static int __init it_led_init(void)
{
	int ret = 0;

	printk(KERN_INFO "Ingenient Technologies LED Driver 1.0\n");

	if ((ret = register_chrdev(IT_MAJOR_DEV, "it_led", &it_led_fileops)) < 0) {
		printk(KERN_ERR "Could not register device driver.\n");

		goto out;
	}//if

	if (request_gio(IT_LED_1) ||
	    request_gio(IT_LED_2) //||
	    //request_gio(IT_LED_3) ||
	    //request_gio(IT_LED_4)
	    )
		return -ENODEV;

	gio_set_dir(IT_LED_1, bit_low);
	gio_set_dir(IT_LED_2, bit_low);
	//gio_set_dir(IT_LED_3, bit_low);
	//gio_set_dir(IT_LED_4, bit_low);

	gio_set_bitclr(IT_LED_1);
	gio_set_bitset(IT_LED_2);
	//gio_set_bitset(IT_LED_3);
	//gio_set_bitset(IT_LED_4);

     out:
	return ret;
}

static void __exit it_led_exit(void)
{
	unregister_chrdev(IT_MAJOR_DEV, "it_led");
}

module_init(it_led_init);
module_exit(it_led_exit);

MODULE_AUTHOR("Ingenient Technologies");
MODULE_DESCRIPTION("Ingenient Technologies LED Driver");
MODULE_LICENSE("GPL");
