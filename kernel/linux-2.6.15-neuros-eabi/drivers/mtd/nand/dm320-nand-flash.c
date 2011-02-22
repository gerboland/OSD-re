/*
 * dm320 nand flash driver 
 *
 * based on drivers/mtd/nand/omap-nand-flash.c
 *
 * Copyright (c) 2004 Texas Instruments, Jian Zhang <jzhang@ti.com>
 * Copyright (c) 2004 David Brownell
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/types.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>

#include <asm/io.h>
#include <asm/hardware.h>
#include <asm/mach-types.h>
#include <asm/mach/flash.h>

#include <asm/io.h>
#include <asm/arch/hardware.h>
#include <asm/arch/io_registers.h>
#include <asm/arch/gio.h>
#include <asm/arch/gios.h>
#include <asm/arch/cs2ctl.h>

#define	DRIVER_NAME	"dm320nand"

#ifdef CONFIG_MTD_PARTITIONS
static const char *part_probes[] = { "cmdlinepart", NULL };
#endif

struct dm320_nand_info {
	struct nand_platform_data *pdata;
	struct mtd_partition	*parts;
	struct mtd_info		mtd;
	struct nand_chip	nand;
        void __iomem *p_nand;
};

#define CHIP_DELAY    25 /* the nand chip need 25us */

#ifdef CONFIG_MTD_PARTITIONS
/* Define partitions for nand device */
static struct mtd_partition nand_partitions[] = {
	{
		.name		= "NAND-boot",
		.size		= 0x00080000,
		.offset		= 0x0,
		.mask_flags     = MTD_WRITEABLE,  /* force read-only */
	},{
		.name		= "bootloader",
		.size		= 0x00040000,
		.offset		= 0x00080000,
		.mask_flags     = MTD_WRITEABLE,  /* force read-only */
	},{
		.name		= "u-env",
		.size		= 0x00020000,
		.offset		= 0x000C0000,
	},{
		.name		= "kernel",
		.size		= 0x00180000,
		.offset		= 0x000E0000,
		.mask_flags     = MTD_WRITEABLE,  /* force read-only */
	},{
		.name		= "cramfs",
		.size		= 0x00D20000,
		.offset		= 0x00260000,
		.mask_flags     = MTD_WRITEABLE,  /* force read-only */
	},{
		.name		= "jffs2",
		.size		= 0x00100000,
		.offset		= 0x01180000,
	},{
		.name		= "yaffs",
		.size		= 0x06D80000,
		.offset		= 0x01280000,
	},
};
#endif

/*
 *	hardware specific access to control-lines
 *	NOTE:  boards may use different bits for these!!
 */
#define	MEM_NAND_CMD	0x00
#define	MEM_NAND_ADDR	0x04
#define	MEM_NAND_DATA	0x08

static void dm320_nand_hwcontrol(struct mtd_info *mtd, int cmd)
{
	struct dm320_nand_info *info = container_of(mtd, struct dm320_nand_info, mtd);
	register struct nand_chip *this = mtd->priv;
        void __iomem *p_nand = info->p_nand;

	dm320_controler_get_lock();
	dm320_controler_enable_nand();

	switch(cmd){

	case NAND_CTL_SETCLE: 
	        this->IO_ADDR_W = p_nand + MEM_NAND_CMD; 
		break;
	case NAND_CTL_CLRCLE: 
	        this->IO_ADDR_W = p_nand + MEM_NAND_DATA; 
		break;

	case NAND_CTL_SETALE: 
	        this->IO_ADDR_W = p_nand + MEM_NAND_ADDR; 
		break;
	case NAND_CTL_CLRALE:
		this->IO_ADDR_W = p_nand + MEM_NAND_DATA;
		break;

	case NAND_CTL_SETNCE:
		/* assert (force assert) chip enable */
	        outw( 2, IO_EMIF_BUSWAITMD );
		outw( (inw(IO_EMIF_SMCTRL) & ~ 0x1), IO_EMIF_SMCTRL );
		break;

	case NAND_CTL_CLRNCE:
 		/* deassert chip enable */
	        outw( 3, IO_EMIF_BUSWAITMD );
		outw( (inw(IO_EMIF_SMCTRL) | 0x1), IO_EMIF_SMCTRL );
		break;
	}

	this->IO_ADDR_R = this->IO_ADDR_W;
	dm320_controler_release_lock();	
}

static int dm320_nand_dev_ready(struct mtd_info *mtd)
{
	struct dm320_nand_info *info = container_of(mtd, struct dm320_nand_info, mtd);

	return info->pdata->dev_ready(info->pdata);
}

static u_char dm320_nand_read_byte(struct mtd_info *mtd)
{
	struct nand_chip *this = mtd->priv;
	u_char temp;

	dm320_controler_get_lock();
	dm320_controler_enable_nand();
	temp = readb(this->IO_ADDR_R);
	dm320_controler_release_lock();
	return temp;
}

static void dm320_nand_write_byte(struct mtd_info *mtd, u_char byte)
{
	struct nand_chip *this = mtd->priv;

	dm320_controler_get_lock();
	dm320_controler_enable_nand();
	writeb(byte, this->IO_ADDR_W);
	dm320_controler_release_lock();
}

static u16 dm320_nand_read_word(struct mtd_info *mtd)
{
	struct nand_chip *this = mtd->priv;
	u16 temp;

	dm320_controler_get_lock();
	dm320_controler_enable_nand();
	temp = readw(this->IO_ADDR_R);
	dm320_controler_release_lock();
	return temp;
}

static void dm320_nand_write_word(struct mtd_info *mtd, u16 word)
{
	struct nand_chip *this = mtd->priv;

	dm320_controler_get_lock();
	dm320_controler_enable_nand();
	writew(word, this->IO_ADDR_W);
	dm320_controler_release_lock();
}

static void dm320_nand_write_buf(struct mtd_info *mtd, const u_char *buf, int len)
{
	int i;
	struct nand_chip *this = mtd->priv;

	dm320_controler_get_lock();
	dm320_controler_enable_nand();
	for (i=0; i<len; i++)
		writeb(buf[i], this->IO_ADDR_W);
	dm320_controler_release_lock();
}

static void dm320_nand_read_buf(struct mtd_info *mtd, u_char *buf, int len)
{
	int i;
	struct nand_chip *this = mtd->priv;

	dm320_controler_get_lock();
	dm320_controler_enable_nand();
	for (i=0; i<len; i++)
		buf[i] = readb(this->IO_ADDR_R);
	dm320_controler_release_lock();
}

static int dm320_nand_verify_buf(struct mtd_info *mtd, const u_char *buf, int len)
{
	int i;
	struct nand_chip *this = mtd->priv;

	dm320_controler_get_lock();
	dm320_controler_enable_nand();
	for (i=0; i<len; i++)
		if (buf[i] != readb(this->IO_ADDR_R))
		  {
		      dm320_controler_release_lock();
		      return -EFAULT;
		  }
	dm320_controler_release_lock();
	return 0;
}

static int __devinit dm320_nand_probe(struct platform_device *pdev)
{
	struct dm320_nand_info		*info;
	struct nand_platform_data	*pdata = pdev->dev.platform_data;
	struct resource			*res = pdev->resource;
	unsigned long			size = res->end - res->start + 1;
	int				err;

	if(get_flash_type()) return -ENXIO; /* is nor flash, needn't probe nand driver */
	info = kmalloc(sizeof(struct dm320_nand_info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	memset(info, 0, sizeof(struct dm320_nand_info));

#if defined( GIO_SM_HOTPLUG )
        /* DM320 board - activate FET switch */
        err = request_gio( GIO_SM_HOTPLUG );
        if ( 0 != err ) {
            printk( "%s: Could not allocate GIO\n" ,
                    __FUNCTION__ );
            err = -ENODEV;
            goto out_free_info;
        }

        /* Output, active low */
        gio_set_dir( GIO_SM_HOTPLUG, 0 );
        gio_set_bitclr( GIO_SM_HOTPLUG );
#endif

	if (!request_mem_region(res->start, size, pdev->dev.driver->name)) {
		err = -EBUSY;
		goto out_free_info;
	}

	info->nand.IO_ADDR_R = ioremap(res->start, size);
	if (!info->nand.IO_ADDR_R) {
		err = -ENOMEM;
		goto out_release_mem_region;
	}
#if 0 /* use the same gio with CF card, this has been done in CF driver */
        if (request_gio(GIO_NAND_CF1)) 
	  {
	      printk(KERN_ERR "Gio %d, was previously registered!\n", GIO_NAND_CF1);
	      err = -ENODEV;
	      goto out_iounmap;
	  }
#endif
        gio_set_dir(GIO_NAND_CF1, 0);
	gio_set_bitclr(GIO_NAND_CF1);
	/* enable NAND/smartmedia */
	outw( 1, IO_EMIF_CFCTRL1 );
	outw( 0, IO_EMIF_CFCTRL2 );
	outw( 3, IO_EMIF_BUSWAITMD );
	outw( 1, IO_EMIF_SMCTRL);

        info->p_nand = info->nand.IO_ADDR_R;
        printk( KERN_INFO "dm320_nand_probe: mapped virtual 0x%08x\n",
                (unsigned int)info->p_nand );
	info->nand.IO_ADDR_W = info->nand.IO_ADDR_R;
	info->nand.hwcontrol = dm320_nand_hwcontrol;
	info->nand.write_byte = dm320_nand_write_byte;
	info->nand.read_byte  = dm320_nand_read_byte;
	info->nand.write_word = dm320_nand_write_word;
	info->nand.read_word  = dm320_nand_read_word;
	info->nand.write_buf  = dm320_nand_write_buf;
	info->nand.read_buf   = dm320_nand_read_buf;
	info->nand.verify_buf = dm320_nand_verify_buf;
	info->nand.eccmode = NAND_ECC_SOFT;
	info->nand.options = pdata->options;
	if (pdata->dev_ready)
		info->nand.dev_ready = dm320_nand_dev_ready;
	else
	        info->nand.chip_delay = CHIP_DELAY; 

	info->mtd.name = pdev->dev.bus_id;
	info->mtd.priv = &info->nand;

	info->pdata = pdata;

        if (nand_scan(&info->mtd, 1)) {
            err = -ENXIO;
            goto out_iounmap;
        }
	info->mtd.owner = THIS_MODULE;

#ifdef CONFIG_MTD_PARTITIONS
	pdata->parts    = nand_partitions;
	pdata->nr_parts = ARRAY_SIZE(nand_partitions);
	err = parse_mtd_partitions(&info->mtd, part_probes, &info->parts, 0);
	if (err > 0)
		add_mtd_partitions(&info->mtd, info->parts, err);
	else if (err <= 0 && pdata->parts)
		add_mtd_partitions(&info->mtd, pdata->parts, pdata->nr_parts);
	else
#endif
		add_mtd_device(&info->mtd);

	platform_set_drvdata(pdev, info);

	return 0;

out_iounmap:
	iounmap(info->nand.IO_ADDR_R);
out_release_mem_region:
	release_mem_region(res->start, size);
out_free_info:
	kfree(info);

	return err;
}

static int dm320_nand_remove(struct platform_device *pdev)
{
	struct dm320_nand_info *info = platform_get_drvdata(pdev);

	platform_set_drvdata(pdev, NULL);
	/* Release NAND device, its internal structures and partitions */
	nand_release(&info->mtd);
	iounmap(info->nand.IO_ADDR_R);
	kfree(info);
	return 0;
}

static struct platform_driver dm320_nand_driver = {
	.probe		= dm320_nand_probe,
	.remove		= dm320_nand_remove,
	.driver		= {
		.name	= DRIVER_NAME,
	},
};
MODULE_ALIAS(DRIVER_NAME);

static int __init dm320_nand_init(void)
{
	return platform_driver_register(&dm320_nand_driver);
}

static void __exit dm320_nand_exit(void)
{
	platform_driver_unregister(&dm320_nand_driver);
}

module_init(dm320_nand_init);
module_exit(dm320_nand_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Jian Zhang <jzhang@ti.com> (and others)");
MODULE_DESCRIPTION("Glue layer for NAND flash on TI DM320 boards");

