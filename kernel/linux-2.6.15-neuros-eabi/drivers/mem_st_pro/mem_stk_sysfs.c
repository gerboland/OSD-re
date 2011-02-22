/*
 *
 * Copyright (C) 2005-2006 Ingenient Technologies
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

#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/idr.h>

#include <linux/ms/card.h>
#include <linux/ms/host.h>

#include "mem_stk.h"

MODULE_LICENSE("GPL");

#define dev_to_ms_card(d)	container_of(d, struct ms_card, dev)
#define to_ms_driver(d)		container_of(d, struct ms_driver, drv)
#define cls_dev_to_ms_host(d)	container_of(d, struct ms_host, class_dev)

#define MS_ATTR(name, fmt, args...)					\
static ssize_t ms_##name##_show (struct device *dev, struct device_attribute *attr, char *buf)	\
{									\
    struct ms_card *card = dev_to_ms_card(dev);				\
    return sprintf(buf, fmt, args);					\
}
MS_ATTR(scr, "%08x%08x\n",card->sys_info.serial_no[2],card->sys_info.serial_no[3]);

#define MS_ATTR_RO(name) __ATTR(name, S_IRUGO, ms_##name##_show, NULL)

static struct device_attribute ms_dev_attr_scr = MS_ATTR_RO(scr);

static void ms_release_card(struct device *dev)
{
    struct ms_card *card = dev_to_ms_card(dev);
    kfree(card);
}

/*
 * This currently matches any MMC driver to any MMC card - drivers
 * themselves make the decision whether to drive this card in their
 * probe method.  However, we force "bad" cards to fail.
 */
static int ms_bus_match(struct device *dev, struct device_driver *drv)
{
    struct ms_card *card = dev_to_ms_card(dev);
    return !ms_card_bad(card);
}

static int ms_bus_hotplug(struct device *dev, char **envp, int num_envp, 
			  char *buf, int buf_size)
{
#if 0
#define add_env(fmt,val)						\
    ({									\
      int len, ret = -ENOMEM;						\
      if (i < num_envp) {						\
	envp[i++] = buf;						\
	len = snprintf(buf, buf_size, fmt, val) + 1;			\
	buf_size -= len;						\
	buf += len;							\
	if (buf_size >= 0)						\
	  ret = 0;							\
      }									\
      ret;								\
    })
  
    for (i = 0; i < 12; i++)
        ccc[i] = card->csd.cmdclass & (1 << i) ? '1' : '0';
    ccc[12] = '\0';
  
    i = 0;
    add_env("MMC_CCC=%s", ccc);
    add_env("MMC_MANFID=%06x", card->cid.manfid);
    add_env("MMC_NAME=%s", ms_card_name(card));
    add_env("MMC_OEMID=%04x", card->cid.oemid);
#endif

    return 0;
}

static int ms_bus_suspend(struct device *dev, pm_message_t state)
{
    struct ms_driver *drv = to_ms_driver(dev->driver);
    struct ms_card *card = dev_to_ms_card(dev);
    int ret = 0;
    
    if (dev->driver && drv->suspend)
        ret = drv->suspend(card, state);
    return ret;
}

static int ms_bus_resume(struct device *dev)
{
    struct ms_driver *drv = to_ms_driver(dev->driver);
    struct ms_card *card = dev_to_ms_card(dev);
    int ret = 0;

    if (dev->driver && drv->resume)
        ret = drv->resume(card);
    return ret;
}

static struct bus_type ms_bus_type = {
	.name		= "mem_stk",
	.match		= ms_bus_match,
	.hotplug	= ms_bus_hotplug,
	.suspend	= ms_bus_suspend,
	.resume		= ms_bus_resume,
};

static int ms_drv_probe(struct device *dev)
{
    struct ms_driver *drv = to_ms_driver(dev->driver);
    struct ms_card *card = dev_to_ms_card(dev);
    return drv->probe(card);
}

static int ms_drv_remove(struct device *dev)
{
    struct ms_driver *drv = to_ms_driver(dev->driver);
    struct ms_card *card = dev_to_ms_card(dev);
    drv->remove(card);
    return 0;
}

/**
 *	ms_register_driver - register a media driver
 *	@drv: MS media driver
 */
int ms_register_driver(struct ms_driver *drv)
{
    drv->drv.bus = &ms_bus_type;
    drv->drv.probe = ms_drv_probe;
    drv->drv.remove = ms_drv_remove;
    return driver_register(&drv->drv);
}
EXPORT_SYMBOL(ms_register_driver);

/**
 *	ms_unregister_driver - unregister a media driver
 *	@drv: MS media driver
 */
void ms_unregister_driver(struct ms_driver *drv)
{
    drv->drv.bus = &ms_bus_type;
    driver_unregister(&drv->drv);
}
EXPORT_SYMBOL(ms_unregister_driver);

/*
 * Internal function.  Initialise a MS card structure.
 */
void ms_init_card(struct ms_card *card, struct ms_host *host)
{
    memset(card, 0, sizeof(struct ms_card));
    card->host = host;
    device_initialize(&card->dev);
    card->dev.parent = card->host->dev;
    card->dev.bus = &ms_bus_type;
    card->dev.release = ms_release_card;
}
EXPORT_SYMBOL(ms_init_card);

/*
 * Internal function.  Register a new MS card with the driver model.
 */
int ms_register_card(struct ms_card *card)
{
    int ret;

    snprintf(card->dev.bus_id, sizeof(card->dev.bus_id),
	     "%s:%04x", ms_hostname(card->host), *((unsigned int *)card->sys_info.serial_no));
    ret = device_add(&card->dev);
    if (ret == 0) 
      {
	ret = device_create_file(&card->dev, &ms_dev_attr_scr);
	if (ret)
	    device_del(&card->dev);
      }
    return ret;
}
EXPORT_SYMBOL(ms_register_card);

/*
 * Internal function.  Unregister a new MS card with the
 * driver model, and (eventually) free it.
 */
void ms_remove_card(struct ms_card *card)
{
    if (ms_card_present(card)) 
      {
	device_remove_file(&card->dev, &ms_dev_attr_scr);
	device_del(&card->dev);
      }
    put_device(&card->dev);
}
EXPORT_SYMBOL(ms_remove_card);

static void ms_host_classdev_release(struct class_device *dev)
{
    struct ms_host *host = cls_dev_to_ms_host(dev);
    kfree(host);
}

static struct class ms_host_class = {
	.name		= "mem_stk_host",
	.release	= ms_host_classdev_release,
};

static DEFINE_IDR(ms_host_idr);
spinlock_t ms_host_lock = SPIN_LOCK_UNLOCKED;

/*
 * Internal function. Allocate a new MS host.
 */
struct ms_host *ms_alloc_host_sysfs(int extra, struct device *dev)
{
    struct ms_host *host;

    host = kmalloc(sizeof(struct ms_host) + extra, GFP_KERNEL);
    if (host) 
      {
	memset(host, 0, sizeof(struct ms_host) + extra);
	
	host->dev = dev;
	host->class_dev.dev = host->dev;
	host->class_dev.class = &ms_host_class;
	class_device_initialize(&host->class_dev);
      }
    
    return host;
}
EXPORT_SYMBOL(ms_alloc_host_sysfs);

/*
 * Internal function. Register a new MS host with the MS class.
 */
int ms_add_host_sysfs(struct ms_host *host)
{
    if (!idr_pre_get(&ms_host_idr, GFP_KERNEL))
        return -ENOMEM;

    spin_lock(&ms_host_lock);
    host->index = idr_get_new(&ms_host_idr, host, &host->index);
    spin_unlock(&ms_host_lock);
    if (host->index == -1)
      {
	printk ("Error in %s while executing idr_get_new()",__FUNCTION__);
	return -1;
      }

    snprintf(host->class_dev.class_id, BUS_ID_SIZE,
	     "mem_stk%d", host->index);

    return class_device_add(&host->class_dev);
}
EXPORT_SYMBOL(ms_add_host_sysfs);

/*
 * Internal function. Unregister a MS host with the MS class.
 */
void ms_remove_host_sysfs(struct ms_host *host)
{
    class_device_del(&host->class_dev);
    spin_lock(&ms_host_lock);
    idr_remove(&ms_host_idr, host->index);
    spin_unlock(&ms_host_lock);
}
EXPORT_SYMBOL(ms_remove_host_sysfs);

/*
 * Internal function. Free a MS host.
 */
void ms_free_host_sysfs(struct ms_host *host)
{
    class_device_put(&host->class_dev);
}
EXPORT_SYMBOL(ms_free_host_sysfs);

static int __init ms_init(void)
{
    int ret = bus_register(&ms_bus_type);
    if (ret == 0) 
      {
	ret = class_register(&ms_host_class);
	if (ret)
	  bus_unregister(&ms_bus_type);
      }
    return ret;
}

static void __exit ms_exit(void)
{
    class_unregister(&ms_host_class);
    bus_unregister(&ms_bus_type);
}

module_init(ms_init);
module_exit(ms_exit);
