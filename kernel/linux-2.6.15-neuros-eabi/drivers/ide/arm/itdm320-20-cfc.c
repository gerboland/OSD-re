/*
 *  Copyright (C) 2006 Neuros Technology International LLC.
 *  Add in support for CFC hotplug, Terry Qiu <tqiu@neuros.com.cn>
 *
 * Copyright (C) 2004 Ingenient Technologies, Inc.
 * Author: Jay Williams <jay.williams@ingenient.com>
 *  
 * Derived loosely from ide-pmac.c, so:
 *
 *  Copyright (C) 2001 Broadcom Corporation
 *
 *  Which was Derived loosely from ide-pmac.c, so:
 *  
 *  Copyright (C) 1998 Paul Mackerras.
 *  Copyright (C) 1995-1998 Mark Lord
 *
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 */

#include <linux/config.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/ide.h>
#include <asm/irq.h>
#include <asm/io.h>
#include <asm/arch/io_registers.h>
#include <asm/arch/irqs.h>
#include <asm/arch/gio.h>
#include <linux/timer.h>
#include <asm/arch/cs2ctl.h>

/* Compact Flash Card Configuration registers */
#define CFC_CONFIG_OPTION_REG   (__ide_cfc(0x200))
#define CFC_CONFIG_STATUS_REG   (__ide_cfc(0x202))
#define CFC_PIN_REPLACEMENT_REG (__ide_cfc(0x204))
#define CFC_SOCKET_AND_COPY_REG (__ide_cfc(0x206))

/*
 *	Conventional PIO operations for ATA devices
 */
static u8 ide_itdm320_inb (unsigned long port)
{
        u8 temp;
	
	dm320_controler_get_lock();
    	dm320_controler_enable_cfc();
	temp = (u8) readb(port);
	dm320_controler_release_lock();
	return temp;
}

static u16 ide_itdm320_inw (unsigned long port)
{
        u16 temp;

	dm320_controler_get_lock();
	dm320_controler_enable_cfc();
	temp = (u16) readw(port);
	dm320_controler_release_lock();
	return temp;
}

static void ide_itdm320_insw (unsigned long port, void *addr, u32 count)
{
        dm320_controler_get_lock();
	dm320_controler_enable_cfc();
	readsw(port, addr, count);
	dm320_controler_release_lock();
}

static u32 ide_itdm320_inl (unsigned long port)
{
        u32 temp;

	dm320_controler_get_lock();
	dm320_controler_enable_cfc();
	temp = (u32) readl(port);
	dm320_controler_release_lock();
	return temp;
}

static void ide_itdm320_insl (unsigned long port, void *addr, u32 count)
{
	dm320_controler_get_lock();
	dm320_controler_enable_cfc();
	readsl(port, addr, count);
	dm320_controler_release_lock();
}

static void ide_itdm320_outb (u8 val, unsigned long port)
{
	dm320_controler_get_lock();
	dm320_controler_enable_cfc();
	writeb(val, port);
	dm320_controler_release_lock();
}

static void ide_itdm320_outbsync (ide_drive_t *drive, u8 addr, unsigned long port)
{
	dm320_controler_get_lock();
	dm320_controler_enable_cfc();
	writeb(addr, port);
	dm320_controler_release_lock();
}

static void ide_itdm320_outw (u16 val, unsigned long port)
{
	dm320_controler_get_lock();
	dm320_controler_enable_cfc();
	writew(val, port);
	dm320_controler_release_lock();
}

static void ide_itdm320_outsw (unsigned long port, void *addr, u32 count)
{
	dm320_controler_get_lock();
	dm320_controler_enable_cfc();
	writesw(port, addr, count);
	dm320_controler_release_lock();
}

static void ide_itdm320_outl (u32 val, unsigned long port)
{
	dm320_controler_get_lock();
	dm320_controler_enable_cfc();
	writel(val, port);
	dm320_controler_release_lock();
}

static void ide_itdm320_outsl (unsigned long port, void *addr, u32 count)
{
	dm320_controler_get_lock();
	dm320_controler_enable_cfc();
	writesl(port, addr, count);
	dm320_controler_release_lock();
}

typedef struct cfc_data_s {
  int inserted;
  int index;
 }cfc_data_t;

static struct work_struct cfc_work;
static cfc_data_t  cfc_data;
static spinlock_t osd_cfc_lock = SPIN_LOCK_UNLOCKED;

static void ide_cfc_hotplug(void)
{
        long unsigned int flags;
	static volatile long unsigned int count =0;
	static volatile long unsigned i_jiffies;
	volatile long unsigned c_jiffies;

	c_jiffies = jiffies;
	spin_lock_irqsave(&osd_cfc_lock, flags);
	if(count)
	  {
	    if((c_jiffies - i_jiffies) < 25)
	      {
		spin_unlock_irqrestore(&osd_cfc_lock, flags);
		return;
	      }
	  }
	count =1;
	udelay(1000);
	if(gio_get_bitset(GIO_CFC_DETECT))
	  {
	    printk(KERN_INFO "[ide_cfc_hotplug] pull out\n");
	    gio_set_bitset(GIO_CFC_HOTPLUG);
	    cfc_data.inserted = 0;
	  }
	else
	  {
	    printk(KERN_INFO "[ide_cfc_hotplug] inserted\n");
	    gio_set_bitset(GIO_CFC_RESET);
	    udelay(1000);
	    gio_set_bitclr(GIO_CFC_RESET);
	    mdelay(5);
	    gio_set_bitclr(GIO_CFC_HOTPLUG);
	    cfc_data.inserted = 1;
	  }

	i_jiffies = jiffies;
	spin_unlock_irqrestore(&osd_cfc_lock, flags);
	schedule_work(&cfc_work);
}

static irqreturn_t ide_cfc_hotplug_handler(int i, void *data, struct pt_regs *regs)
{
	ide_cfc_hotplug();
	return IRQ_HANDLED;
}

static void set_cfc_stuff(int index);
static void cfc_wq_handle(void *ptr)
{
        cfc_data_t *data = (cfc_data_t *)ptr;

	if(data->inserted)
	  {
	     ideprobe_init();
	     create_proc_ide_interfaces();
	  }
	else
	  {
	     ide_unregister(data->index);
	     set_cfc_stuff(data->index);
	  }
}

static void set_cfc_stuff(int index)
{
	ide_hwif_t *hwif;
	/* Set up our stuff, Note, __ide_cfc() can be found in hardware.h */
	hwif = &ide_hwifs[index];
	hwif->hw.io_ports[IDE_DATA_OFFSET]    = __ide_cfc(0x00);
	hwif->hw.io_ports[IDE_ERROR_OFFSET]   = __ide_cfc(0x01);
	hwif->hw.io_ports[IDE_NSECTOR_OFFSET] = __ide_cfc(0x02);
	hwif->hw.io_ports[IDE_SECTOR_OFFSET]  = __ide_cfc(0x03);
	hwif->hw.io_ports[IDE_LCYL_OFFSET]    = __ide_cfc(0x04);
	hwif->hw.io_ports[IDE_HCYL_OFFSET]    = __ide_cfc(0x05);
	hwif->hw.io_ports[IDE_SELECT_OFFSET]  = __ide_cfc(0x06);
	hwif->hw.io_ports[IDE_STATUS_OFFSET]  = __ide_cfc(0x07);
	hwif->hw.io_ports[IDE_CONTROL_OFFSET] = __ide_cfc(0x0E);
	hwif->hw.io_ports[IDE_IRQ_OFFSET]     = __ide_cfc(0x0F);
	hwif->hw.irq                          = IRQ_MTC1;
	hwif->noprobe                         = 0;

	/* Set callbacks for IDE I/O access. Can't use defaults since we don't I/O map
	   the IDE memory region. Instead we configure the MMU manually in core.c
	*/
	hwif->OUTB	    = ide_itdm320_outb;
	hwif->OUTBSYNC	= ide_itdm320_outbsync;
	hwif->OUTW	    = ide_itdm320_outw;
	hwif->OUTL	    = ide_itdm320_outl;
	hwif->OUTSW	    = ide_itdm320_outsw;
	hwif->OUTSL	    = ide_itdm320_outsl;
	hwif->INB	    = ide_itdm320_inb;
	hwif->INW    	= ide_itdm320_inw;
	hwif->INL	    = ide_itdm320_inl;
	hwif->INSW	    = ide_itdm320_insw;
	hwif->INSL	    = ide_itdm320_insl;
    
	memcpy(hwif->io_ports, hwif->hw.io_ports, sizeof(hwif->io_ports));
	hwif->irq                             = hwif->hw.irq;

}

void __init itdm320_20_ide_cfc_probe(void)
{
	int i;

	/* 
	 * Find the first untaken slot in hwifs 
	 */
	for (i = 0; i < MAX_HWIFS; i++) {
		if (!ide_hwifs[i].io_ports[IDE_DATA_OFFSET]) {
			break;
		}
	}
	if (i == MAX_HWIFS) {
		printk("No space for ITDM320-20 onboard IDE/CFC driver in "
               "ide_hwifs[].  Not enabled.\n");
		return;
	}

	cfc_data.index = i;
	cfc_data.inserted = 0;

    /* Reset the CFC, request the GIOs */
    if (request_gio(GIO_CFC_HOTPLUG)) {
       printk(KERN_ERR "Gio %d, was previously registered!\n", GIO_CFC_HOTPLUG);
       return;
    }
    if (request_gio(GIO_CFC_RESET)) {
       printk(KERN_ERR "Gio %d, was previously registered!\n", GIO_CFC_RESET);
       return;
    }
#if 1
    if (request_gio(GIO_NAND_CF1)) {
	printk(KERN_ERR "Gio %d, was previously registered!\n", GIO_NAND_CF1);
	return;
    }
#endif
    if(request_gio(GIO_CFC_DETECT))
      {
	printk(KERN_ERR "Gio %d, was previously regeistered!\n", GIO_CFC_DETECT);
	return;
      }
    gio_set_dir(GIO_CFC_DETECT, bit_hi);
    gio_set_dir(GIO_CFC_HOTPLUG, bit_low);
    gio_set_dir(GIO_CFC_RESET, bit_low);

    gio_set_dir(GIO_NAND_CF1, 0);
    gio_set_bitset(GIO_NAND_CF1);

    if(gio_get_bitset(GIO_CFC_DETECT)==0)
      {
	gio_set_bitset(GIO_CFC_RESET);
	udelay(1000);
	gio_set_bitclr(GIO_CFC_RESET);
	mdelay(5);
	gio_set_bitclr(GIO_CFC_HOTPLUG);

	cfc_data.inserted =1;
      }

    /* Select the attribute memory space */
    outw(0x0000, IO_EMIF_CFCTRL1); /* Seltect CF card interface */
    outw(0x0011, IO_EMIF_CFCTRL2); /* Use dynamic bus sizing and output
				      high REG when accessing CFC common memory */
                                       
    /* Setup the CFC configuration registers */
    writew(0x0000, CFC_CONFIG_OPTION_REG);
    
    set_cfc_stuff(i);

    printk("Ingenient onboard IDE/CFC configured as device %i\n", i);

#ifdef GIO_CFC_DETECT
	gio_enable_irq(GIO_CFC_DETECT, GIO_ANY_EDGE);
	request_irq(GIO_CFC_DETECT+21,ide_cfc_hotplug_handler,0,"ide_cfc_hotplug",NULL);
	INIT_WORK(&cfc_work, cfc_wq_handle, &cfc_data);
#endif
}

