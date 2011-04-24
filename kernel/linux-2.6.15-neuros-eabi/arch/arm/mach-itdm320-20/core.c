/*
 *  linux/arch/arm/mach-itdm320/core.c
 *
 *  Copyright (C) 2004 Ingenient Technologies
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation.
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/version.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,15)
#define LINUX_2_6_15  
#endif
#ifndef LINUX_2_6_15
#include <linux/device.h>
#else /* LINUX_2_6_15 */
//#include <linux/device.h> /* Removed for ISOFT_PORT */
#include <linux/platform_device.h>/* ISOFT_PORT */
#endif /* LINUX_2_6_15 */
#include <linux/mm.h>

#include <asm/hardware.h>
#include <asm/io.h>
#include <asm/mach-types.h>
#include <asm/setup.h>

#ifdef LINUX_2_6_15
#include <asm/mach/time.h>
#endif /* LINUX_2_6_15 */
#include <asm/mach/map.h>
#include <asm/mach/arch.h>

#include <asm/arch/hardware.h>
#include <asm/arch/memory.h>

/* for NAND flash */
#include <asm/mach/flash.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>


extern void itdm320_init_irq(void);

static struct map_desc itdm320_io_desc[] __initdata = {
#ifndef LINUX_2_6_15
	{ IO_BASE, IO_START, IO_SIZE, MT_DEVICE },
	{ AHB_BASE, AHB_START, AHB_SIZE, MT_DEVICE },
	{ DSP_BASE, DSP_START, DSP_SIZE, MT_DEVICE },
	{ ETHR_BASE, ETHR_START, ETHR_SIZE, MT_DEVICE },
	{ HDD_BASE, HDD_START, HDD_SIZE, MT_DEVICE },
	{ CFC_BASE, CFC_START, CFC_SIZE, MT_DEVICE },
	{ FLASH_BASE, FLASH_START, FLASH_SIZE, MT_DEVICE },
	{ USB_BASE, USB_START, USB_SIZE, MT_DEVICE },
#else /* LINUX_2_6_15 */
	{ IO_BASE, __phys_to_pfn(IO_START), IO_SIZE, MT_DEVICE },
	{ AHB_BASE, __phys_to_pfn(AHB_START), AHB_SIZE, MT_DEVICE },
	{ DSP_BASE, __phys_to_pfn(DSP_START), DSP_SIZE, MT_DEVICE },
	{ ETHR_BASE, __phys_to_pfn(ETHR_START), ETHR_SIZE, MT_DEVICE },
	{ HDD_BASE, __phys_to_pfn(HDD_START), HDD_SIZE, MT_DEVICE },
	{ CFC_BASE, __phys_to_pfn(CFC_START), CFC_SIZE, MT_DEVICE },
	{ FLASH_BASE, __phys_to_pfn(FLASH_START), FLASH_SIZE, MT_DEVICE },
	{ USB_BASE, __phys_to_pfn(USB_START), USB_SIZE, MT_DEVICE },
#endif /* LINUX_2_6_15 */
};

static struct map_desc imem_map = {
	.virtual = IMEM_BASE,
	.type    = MT_DEVICE,
};

static struct resource imem_resources[] = {
	[0] = {
		//.start	= IMEM_START,		/* Physical */
		//.end	= IMEM_START + IMEM_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
};

static struct platform_device imem_device = {
	.name		= "imem",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(imem_resources),
	.resource	= imem_resources,
};

static struct resource smc91x_resources[] = {
	[0] = {
		.start	= ETHR_START,		/* Physical */
		.end	= ETHR_START + ETHR_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= 31,				/* Really GPIO 0 */
		.end	= 31,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device smc91x_device = {
	.name		= "smc91x",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(smc91x_resources),
	.resource	= smc91x_resources,
};

static struct resource dm320_usb_resources[] = {
	[0] = {
		.start	= USB_START,		/* Physical */
		.end	= USB_START + USB_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= IRQ_USB_CORE,				/* Really GPIO 0 */
		.end	= IRQ_USB_CORE,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device dm320_usb_device = {
	.name		= "DM320-USB-HCD",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(dm320_usb_resources),
	.resource	= dm320_usb_resources,
};

static struct resource hdd_resources[] = {
   [0] = {
      .start = HDD_START, /* Physical */
      .end   = HDD_START + HDD_SIZE - 1,
      .flags = IORESOURCE_MEM,
   },
   [1] = {
      .start = 32,
      .end   = 32,
      .flags = IORESOURCE_IRQ,
   },
};

static struct platform_device hdd_device = {
   .name       = "IDE-HDD",
   .id         = 0,
   .num_resources = ARRAY_SIZE(hdd_resources),
   .resource      = hdd_resources,
};

static struct resource cfc_resources[] = {
   [0] = {
      .start = CFC_START, /* Physical */
      .end   = CFC_START + CFC_SIZE - 1,
      .flags = IORESOURCE_MEM,
   },
   [1] = {
      .start = 18,  // MTC1
      .end   = 18,
      .flags = IORESOURCE_IRQ,
   },
};

static struct platform_device cfc_device = {
   .name       = "IDE-CFC",
   .id         = 0,
   .num_resources = ARRAY_SIZE(cfc_resources),
   .resource      = cfc_resources,
};

static struct resource sd_mmc_resources[] = {
#if 0
   [0] = {
/* Dont hardcode */
      .start = 0xCFC_START, /* Physical */
      .end   = CFC_START + CFC_SIZE - 1,
      .flags = IORESOURCE_MEM,
   },
#endif
   [0] = {
      .start = 29,  // MTC1
      .end   = 29,
      .flags = IORESOURCE_IRQ,
   },
};
static struct platform_device sd_mmc_device = {
   .name       = "SD-MMC",
   .id         = 0,
   .num_resources = ARRAY_SIZE(sd_mmc_resources),
   .resource      = sd_mmc_resources,
};

static struct resource mem_stk_resources[] = {
#if 0
   [0] = {
/* Dont hardcode */
      .start = 0xCFC_START, /* Physical */
      .end   = CFC_START + CFC_SIZE - 1,
      .flags = IORESOURCE_MEM,
   },
#endif
   [0] = {
      .start = 29,  // MTC1
      .end   = 29,
      .flags = IORESOURCE_IRQ,
   },
};

static struct platform_device mem_stk_device = {
   .name       = "MEM_STICK",
   .id         = 0,
   .num_resources = ARRAY_SIZE(mem_stk_resources),
   .resource      = mem_stk_resources,
};


#if defined( CONFIG_MTD_NAND )
static struct mtd_partition nand_partitions[] = {
	{
		.name		= "filesystem",
		.size		= MTDPART_SIZ_FULL,
		.offset		= MTDPART_OFS_APPEND,
	},
};

static struct nand_platform_data nand_data = {
	.options	= NAND_SAMSUNG_LP_OPTIONS,
	.parts		= nand_partitions,
	.nr_parts	= ARRAY_SIZE(nand_partitions),
};

static struct resource nand_resource = {
	.flags		= IORESOURCE_MEM,
};

static struct platform_device nand_device = {
	.name		= "dm320nand",
	.id		= 0,
	.dev		= {
              .platform_data	= &nand_data,
	},
	.num_resources	= 1,
	.resource	= &nand_resource,
};
#endif


static struct platform_device *devices[] __initdata = {
	&smc91x_device,
	&hdd_device,
	&cfc_device,
	&imem_device,
	&sd_mmc_device,
	&dm320_usb_device,
	&mem_stk_device,
#if defined( CONFIG_MTD_NAND )
        &nand_device,
#endif
};

static void __init itdm320_mach_init(void)
{
#ifndef LINUX_2_6_15
	imem_map.physical = IMEM_START;
#else /* LINUX_2_6_15 */
	//printk("itdm320_mach_init\n");
	//imem_map.physical = IMEM_START;/* ISOFT_PORT */
	imem_map.pfn = __phys_to_pfn(IMEM_START);/* ISOFT_PORT */
#endif /* LINUX_2_6_15 */
	imem_map.length = IMEM_SIZE;

#ifndef LINUX_2_6_15
	imem_resources[0].start = IMEM_START;
	imem_resources[0].end = imem_map.physical + imem_map.length - 1;
#else /* LINUX_2_6_15 */
	imem_resources[0].start =IMEM_START;
	//imem_resources[0].end = imem_map.physical + imem_map.length - 1;/* ISOFT_PORT */
	//imem_resources[0].end = imem_map.pfn + imem_map.length - 1;/* ISOFT_PORT */
	imem_resources[0].end =IMEM_START + imem_map.length - 1;/* ISOFT_PORT */
#endif /* LINUX_2_6_15 */

	iotable_init(&imem_map, 1);

#if defined( CONFIG_MTD_NAND )
        /* NAND/SmartMedia interface */
        nand_resource.start = CONFIG_SSFDC_START;
        nand_resource.end   = CONFIG_SSFDC_START + (SZ_4K-1);

        /* NANDBSP : chip select timings */
// Micron MT29F4G08* - timings based on datasheet
//        outw( 0x330a, IO_EMIF_CS2CTRL1 ); 
//        outw( 0x1338, IO_EMIF_CS2CTRL2 );

// Samsung K9K1G08U0M
        outw( 0x4407, IO_EMIF_CS2CTRL1 ); 
        outw( 0x1121, IO_EMIF_CS2CTRL2 );

        /* enable NAND/smartmedia */
        outw( 1, IO_EMIF_CFCTRL1 );
        outw( 0, IO_EMIF_CFCTRL2 );

        /* page size */
        /* 0=256 bytes, 1=512 bytes, 2=1024 bytes, 3=2048 bytes */
        outw( 3, IO_EMIF_PAGESZ ); 
#endif

	(void) platform_add_devices(devices, ARRAY_SIZE(devices));
}

static void __init itdm320_map_io(void)
{
	iotable_init(itdm320_io_desc, ARRAY_SIZE(itdm320_io_desc));
}

#ifdef LINUX_2_6_15
/* WBB: We are testing to see which of the following two
 * functions perform better
 */
#if 1
 #define itdm320_timer_pending() ((~inw(IO_INTC_IRQ0)) & 1)
#else
 #define itdm320_timer_pending() ((inw(IO_INTC_IRQ0) & 1) == 0)
#endif

unsigned long itdm320_gettimeoffset(void)
{
        volatile unsigned long elapsed, tmp;
        volatile int pending;

        /* Compute the elapsed count. The current count tells us how
         * many counts have elapsed since the last interrupt (COUNT UP)
         */
        do {
                tmp     = inw(IO_TIMER0_TMCNT);
                pending = itdm320_timer_pending();
                elapsed = inw(IO_TIMER0_TMCNT);
        } while (elapsed < tmp);

        if (pending) elapsed += CONFIG_TIMER0_TMDIV;


        /* Convert the elapsed count to usecs. I guess there are 'tick' usecs
         * between every interrupt.
         */
        return (unsigned long) ((elapsed * tick_usec) / CONFIG_TIMER0_TMDIV);
}

/* ISOFT_PORT */
/* For the new timer style */

/* Timer IRQ handler for the IT DM320-20. */
static irqreturn_t itdm320_timer_interrupt(int irq,
                                           void* dev_id,
                                           struct pt_regs *regs)
{
 write_seqlock(&xtime_lock);
 timer_tick(regs);
 write_sequnlock(&xtime_lock);
 return IRQ_HANDLED;

#ifdef OLD
        /* \todo WBB: Do we need to clear the interrupt here?
         * Well we are going to do it anyways because
         * all the other 2.6.5 boards are doing it.
         */
        //outw((inw(IO_INTC_IRQ0) | 1), IO_INTC_IRQ0);

        do_timer(regs);

        /* WBB: What is this profile thingy?
         * I wonder if it is the new oprofile
         * stuff?
         */
        do_profile(regs);

        return IRQ_HANDLED;
#endif
}/* itdm320_timer_interrupt */

static struct irqaction itdm320_timer_irq = {
        .name           = "ITDM320 Timer Tick",
        .flags          = SA_INTERRUPT | SA_TIMER,
        .handler        = itdm320_timer_interrupt,
};

/*
 * Set up timer interrupt, and return the current time in seconds.
 */
void __init itdm320_timer_init(void)
{
        /* Turn off all timers */
        outw(CONFIG_TIMER0_TMMD_STOP, IO_TIMER0_TMMD);
        outw(CONFIG_TIMER1_TMMD_STOP, IO_TIMER1_TMMD);
        outw(CONFIG_TIMER2_TMMD_STOP, IO_TIMER2_TMMD);
        outw(CONFIG_TIMER3_TMMD_STOP, IO_TIMER3_TMMD);

        /* Setup the Prescalar */
        outw(CONFIG_TIMER0_TMPRSCL - 1, IO_TIMER0_TMPRSCL);

        /* Setup the Divisor */
        outw(CONFIG_TIMER0_TMDIV - 1, IO_TIMER0_TMDIV);

        /* Turn Timer0 to Free Run mode */
        outw(CONFIG_TIMER0_TMMD_FREE_RUN, IO_TIMER0_TMMD);

/*
         * Make irqs happen for the system timer
         */
        setup_irq(IRQ_TIMER0, &itdm320_timer_irq);

#ifdef OLD
        gettimeoffset = itdm320_gettimeoffset;

        timer_irq.handler = itdm320_timer_interrupt;

        setup_irq(IRQ_TIMER0, &timer_irq);
#endif
}/* time_init */

struct sys_timer itdm320_timer = {
        .init           = itdm320_timer_init,
        .offset         = itdm320_gettimeoffset,
};

MACHINE_START(ITDM320_20, "IT DM320-20")
	/* Ingenient Technologies */
        .phys_ram       = PHYS_OFFSET,
       // .phys_ram       = PHYS_OFFSET|0x20000,
        .phys_io        = IO_START & 0xFFF00000,
        .io_pg_offst    = ((IO_BASE & 0xFFF00000) >> 18) & 0xfffc,
        .boot_params    = (PHYS_OFFSET | 0x0100),
        .map_io         = itdm320_map_io,
        .init_irq       = itdm320_init_irq,
        .timer          = &itdm320_timer,		/* ISOFT_PORT?	*/
	.init_machine    = itdm320_mach_init,
MACHINE_END


//arch_initcall(itdm320_mach_init);
/* ISOFT_PORT */
#else
MACHINE_START(ITDM320_20, "IT DM320-20")
	MAINTAINER("Ingenient Technologies")
        BOOT_MEM(PHYS_OFFSET, IO_START & 0xFFF00000, IO_BASE & 0xFFF00000)
	BOOT_PARAMS(PHYS_OFFSET | 0x0100)
	MAPIO(itdm320_map_io)
	INITIRQ(itdm320_init_irq)
     INIT_MACHINE(itdm320_mach_init)
MACHINE_END
#endif /* LINUX_2_6_15 */
