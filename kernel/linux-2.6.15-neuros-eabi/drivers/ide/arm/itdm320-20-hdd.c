/*
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
#include <linux/dma-mapping.h>

/* IDE hardware resides in externel memory interface EM3.
   EM3 is programmed to reside at HDD_START.  However,
   because of logic to physical address issues, the following
   offset values are used:
*/
#define REGISTER_OFFSET 0x00400000 /* A21 = High */
#define CONTROL_OFFSET  0x00800000 /* A22 = High */
#define IDE_SHIFT 17

#if defined (CONFIG_ARCH_ITDM340_10)
/* Initialize DMA transfer parameters  */
/* to transfer an entire cache line at once */
#define HDD_DMA_INIT() \
    outw(0x20, IO_EMIF_DMAXSIZE ); \
    outw(0x20, IO_EMIF_ARMOFFSET ); \
    outw(0x20, IO_EMIF_MTCOFFSET ); 
#define SET_DMA_SIZE( word_count ) \
	outw(((word_count) + 15) >> 4, IO_EMIF_DMASIZE);
#else
#define HDD_DMA_INIT()
#define SET_DMA_SIZE( word_count ) \
	outw(((word_count) + 1) >> 1, IO_EMIF_DMASIZE);
#endif

/*
 *	Conventional PIO operations for ATA devices
 */

static u8 ide_itdm320_inb (unsigned long port)
{
	return (u8) readb(port);
}

static u16 ide_itdm320_inw (unsigned long port)
{
	return (u16) readw(port);
}

#if defined(CONFIG_DMA_HDD)
static void ide_itdm320_insw (unsigned long port, void *addr, u32 count)
{
	volatile u16 *d_ptr;
	unsigned long dest_addr;

        if ( count <= SECTOR_WORDS ) {
            readsw(port, addr, count);
            return;
        }

	d_ptr = (volatile u16*) addr;
	dest_addr = virt_to_phys((void*) d_ptr);

	if (((u32) dest_addr) & 0x2) {
		readsw(port, addr, 1);
		dest_addr += 2;
		count -= 2;
                if (!count) return;
	}//if

	while (inw(IO_EMIF_DMACTL) & 0x0001); // wait for pending outsw() if any
	outw(0x0830, IO_SDRAM_SDDMASEL); // MTC 1 burst DMA
	outw(0x0003, IO_EMIF_DMAMTCSEL); // DMA MTC Select: CS3
	outw(((port & 0x0FFF0000) >> 16) |
	     (1 << 15), IO_EMIF_MTCADDH);
	outw(port & 0x0000FFFF, IO_EMIF_MTCADDL);
	outw((dest_addr & 0x7FFF0000) >> 16, IO_EMIF_AHBADDH);
	outw(dest_addr & 0x0000FFFF, IO_EMIF_AHBADDL);
        SET_DMA_SIZE( count );

	/* Start the DMA transfer */
	outw(0x0002, IO_EMIF_DMACTL);
	outw(inw(IO_EMIF_DMACTL) | 1, IO_EMIF_DMACTL);

        // invalidate cache, so we read new data from DRAM 
        //arm926_dma_inv_range((unsigned long)addr, (unsigned long)addr + count );
        consistent_sync( addr, count, DMA_FROM_DEVICE );

}
#else
static void ide_itdm320_insw (unsigned long port, void *addr, u32 count)
{
	readsw(port, addr, count);
}
#endif

static u32 ide_itdm320_inl (unsigned long port)
{
	return (u32) readl(port);
}

static void ide_itdm320_insl (unsigned long port, void *addr, u32 count)
{
	readsl(port, addr, count);
}

static void ide_itdm320_outb (u8 val, unsigned long port)
{
	writeb(val, port);
}

static void ide_itdm320_outbsync (ide_drive_t *drive, u8 addr, unsigned long port)
{
	writeb(addr, port);
}


static void ide_itdm320_outw (u16 val, unsigned long port)
{
	writew(val, port);
}


#if defined(CONFIG_DMA_HDD)
static void ide_itdm320_outsw (unsigned long port, void *addr, u32 count)
{
	volatile u16 *d_ptr;

	unsigned long dest_addr;

        if ( count <= SECTOR_WORDS ) {
            writesw(port, addr, count);
            return;
        }

	d_ptr = (volatile u16*) addr;
	dest_addr = virt_to_phys((void*) d_ptr);

	if (((u32) dest_addr) & 0x2) {
		writesw(port, addr, 1);
		dest_addr += 2;
		count -= 2;
                if (!count) return;
	}//if


        if (  0 != (count&15) ) { 
            printk( "Warning: word count=%d\n", count );
        }

        // flush write data to DRAM 
//        arm926_dma_flush_range((unsigned long)addr, (unsigned long)addr + count );
        consistent_sync( addr, count, DMA_TO_DEVICE );

	while (inw(IO_EMIF_DMACTL) & 0x0001); // wait for DMA completion

	outw(0x0830, IO_SDRAM_SDDMASEL); // MTC 1 burst DMA
	outw(0x0003, IO_EMIF_DMAMTCSEL); // DMA MTC Select: CS3
	outw(((port & 0x0FFF0000) >> 16) |
	     (1 << 15), IO_EMIF_MTCADDH);
	outw(port & 0x0000FFFF, IO_EMIF_MTCADDL);
	outw((dest_addr & 0x7FFF0000) >> 16, IO_EMIF_AHBADDH);
	outw(dest_addr & 0x0000FFFF, IO_EMIF_AHBADDL);
        SET_DMA_SIZE( count );

	/* Start the DMA transfer */
	outw(0x0000, IO_EMIF_DMACTL);
	outw(inw(IO_EMIF_DMACTL) | 1, IO_EMIF_DMACTL);
}
#else
static void ide_itdm320_outsw (unsigned long port, void *addr, u32 count)
{
	writesw(port, addr, count);
}
#endif

static void ide_itdm320_outl (u32 val, unsigned long port)
{
	writel(val, port);
}

static void ide_itdm320_outsl (unsigned long port, void *addr, u32 count)
{
	writesl(port, addr, count);
}

#include <asm/arch/leds.h>
spinlock_t it_hdd_lock = SPIN_LOCK_UNLOCKED;

#ifdef GIO_HDD_HOTPLUG	
static void ide_itdm320_hotplug(void)
{
// for now, we just toggle the LED...
	static int usb_enabled = 0;
	long unsigned int flags;
	static volatile long unsigned int count = 0;
	static volatile long unsigned i_jiffies;
	volatile long unsigned c_jiffies;
	c_jiffies = jiffies;
	spin_lock_irqsave(&it_hdd_lock,flags);
	if (count){//ignore comparison during very first interrupt
		if ((c_jiffies - i_jiffies) < 25){
			spin_unlock_irqrestore(&it_hdd_lock,flags);
			return;
			}
		}
	count = 1;
	if (usb_enabled){
		it_led_off(IT_LED_1);
		gio_set_bitclr(GIO_USB_ENABLE);
		gio_set_inv(GIO_USB_ENABLE,bit_hi);
		usb_enabled = 0;
		}
	else	{
		it_led_on(IT_LED_1);
		gio_set_bitset(GIO_USB_ENABLE);
		gio_set_inv(GIO_USB_ENABLE,bit_low);
		usb_enabled = 1;
		}
	i_jiffies = jiffies;
	spin_unlock_irqrestore(&it_hdd_lock,flags);
}

static irqreturn_t ide_itdm320_hotplug_handler(int i, void * data,
        struct pt_regs * regs)
{
	ide_itdm320_hotplug();
	return IRQ_HANDLED;
}
#endif

void
itdm320_20_ide_wait_dma_complete( void )
{
	while (inw(IO_EMIF_DMACTL) & 0x0001)
            ; 
}

void __init itdm320_20_ide_hdd_probe(void)
{
	int i;
	ide_hwif_t *hwif;

	/* 
	 * Find the first untaken slot in hwifs 
	 */
	for (i = 0; i < MAX_HWIFS; i++) {
		if (!ide_hwifs[i].io_ports[IDE_DATA_OFFSET]) {
			break;
		}
	}
	if (i == MAX_HWIFS) {
		printk("No space for ITDM320-20 onboard IDE driver in ide_hwifs[].  Not enabled.\n");
		return;
	}

#ifdef GIO_USB_ENABLE    
    /* Drive USB_ENABLE low (Reference section 10, USB in the MP4900-BRD-DM320-20
       Hardware Design Spec)
    */
    if (request_gio(GIO_USB_ENABLE)) {
       printk(KERN_ERR "Gio %d, was previously registered!\n", GIO_USB_ENABLE);
       return;
    }
    gio_set_dir(GIO_USB_ENABLE, bit_low);
    gio_set_bitclr(GIO_USB_ENABLE);
#endif

#if defined(CONFIG_DMA_HDD)
#if defined(HDD_DMA_INIT)
    HDD_DMA_INIT();
#endif
#endif    
	/* Set up our stuff, Note, __ide_hdd() can be found in hardware.h */
	hwif = &ide_hwifs[i];
	hwif->hw.io_ports[IDE_DATA_OFFSET]    = __ide_hdd(REGISTER_OFFSET + (0x00 << IDE_SHIFT));
	hwif->hw.io_ports[IDE_ERROR_OFFSET]   = __ide_hdd(REGISTER_OFFSET + (0x01 << IDE_SHIFT));
    hwif->hw.io_ports[IDE_NSECTOR_OFFSET] = __ide_hdd(REGISTER_OFFSET + (0x02 << IDE_SHIFT));
	hwif->hw.io_ports[IDE_SECTOR_OFFSET]  = __ide_hdd(REGISTER_OFFSET + (0x03 << IDE_SHIFT));
	hwif->hw.io_ports[IDE_LCYL_OFFSET]    = __ide_hdd(REGISTER_OFFSET + (0x04 << IDE_SHIFT));
	hwif->hw.io_ports[IDE_HCYL_OFFSET]    = __ide_hdd(REGISTER_OFFSET + (0x05 << IDE_SHIFT));
	hwif->hw.io_ports[IDE_SELECT_OFFSET]  = __ide_hdd(REGISTER_OFFSET + (0x06 << IDE_SHIFT));
	hwif->hw.io_ports[IDE_STATUS_OFFSET]  = __ide_hdd(REGISTER_OFFSET + (0x07 << IDE_SHIFT));
	hwif->hw.io_ports[IDE_CONTROL_OFFSET] = __ide_hdd(CONTROL_OFFSET  + (0x06 << IDE_SHIFT));
	hwif->hw.io_ports[IDE_IRQ_OFFSET]     = __ide_hdd(CONTROL_OFFSET  + (0x07 << IDE_SHIFT));
	hwif->hw.irq                          = IRQ_GIO7;//IRQ_GIO11;
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
	printk("Ingenient onboard IDE/HDD configured as device %i\n", i);
#ifdef GIO_HDD_HOTPLUG       
	request_gio(GIO_HDD_HOTPLUG);
	gio_enable_irq(GIO_HDD_HOTPLUG,GIO_ANY_EDGE);
	request_irq(GIO_HDD_HOTPLUG + 21,ide_itdm320_hotplug_handler,0,"ide_usb_hotplug",NULL);
#endif
}

// interrupt 27 maps to gio6

