/*
 * $Id: itdm.c,v 1.1 2005/12/08 22:21:00 jwhittington Exp $
 *
 * Neuros NOR flash mapping driver, modified form original 
 * Ingenient driver.
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/init.h>

#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/partitions.h>

#include <asm/io.h>

static struct mtd_info *mymtd;

#define WINDOW_SIZE PHYS_FLASH_SIZE*CFG_MAX_FLASH_BANKS
#define WINDOW_ADDR PHYS_FLASH_1

struct map_info itdm_map = {
	.name = "Neuros NOR Flash",
	.size = WINDOW_SIZE,
	.bankwidth = 2,
	.phys = WINDOW_ADDR,
};

#ifdef CONFIG_MTD_PARTITIONS
static struct mtd_partition *mtd_parts;
static int                   mtd_parts_nb;
const char *part_probes[] = {"cmdlinepart", NULL};

static struct mtd_partition itdm_partitions[] = {
	{
		.name =		"bootloader",
		.size =		0x040000,
		.offset =	0x0,
		.mask_flags =	MTD_WRITEABLE,  /* force read-only */
	}, {
		.name =		"u-env",
		.size =		0x010000,
		.offset =	0x040000,
	}, {
		.name =		"k-env",
		.size =		0x010000,
		.offset =	0x050000,
	}, {
		.name =		"kernel",
		.size =		0x120000,
		.offset =	0x060000,
	}, {
		.name =		"cramfs",
		.size =		0x610000,
		.offset =	0x180000,
	}, {
		.name =		"jffs2",
		.size =		0x060000,
		.offset =	0x790000,
	}, {
		.name =		"ramfs",
		.size =		0x010000,
		.offset =	0x7F0000,
	}
};
#define NUM_PARTITIONS	(sizeof(itdm_partitions)/sizeof(struct mtd_partition))

#endif /* CONFIG_MTD_PARTITIONS */

int __init init_itdm(void)
{
	static const char *rom_probe_types[] = { "cfi_probe", "jedec_probe", "map_rom", 0 };
	const char **type;

	printk(KERN_NOTICE "Neuros NOR flash device: %x at %x\n", WINDOW_SIZE, WINDOW_ADDR);
	itdm_map.virt = ioremap(WINDOW_ADDR, WINDOW_SIZE);

	if (!itdm_map.virt) {
		printk("Failed to ioremap\n");
		return -EIO;
	}
       	printk(KERN_NOTICE "Flash device virtual mapping at %p\n", itdm_map.virt);

	simple_map_init(&itdm_map);

	mymtd = 0;
	type = rom_probe_types;
	for(; !mymtd && *type; type++) {
		mymtd = do_map_probe(*type, &itdm_map);
	}
	if (mymtd) {
		mymtd->owner = THIS_MODULE;

		add_mtd_device(mymtd);

#ifdef CONFIG_MTD_PARTITIONS
		mtd_parts_nb = parse_mtd_partitions(mymtd, part_probes, 
						    &mtd_parts, 0);

		if (mtd_parts_nb > 0)
		{
			add_mtd_partitions (mymtd, mtd_parts, mtd_parts_nb);
			return 0;
		}

		if (NUM_PARTITIONS != 0) 
		{
			printk(KERN_NOTICE 
			       "Using Neuros partition definition\n");
			add_mtd_partitions (mymtd, itdm_partitions, NUM_PARTITIONS);
			return 0;
		}

#endif

		return 0;
	}

	iounmap((void *)itdm_map.virt);
	return -ENXIO;
}

static void __exit cleanup_itdm(void)
{
#ifdef CONFIG_MTD_PARTITIONS
	if (mtd_parts_nb) {
		del_mtd_partitions(mymtd);
		kfree(mtd_parts);
	} else if (NUM_PARTITIONS) {
		del_mtd_partitions(mymtd);
	} else {
		del_mtd_device(mymtd);
	}
#else
	del_mtd_device(mymtd);
#endif
	map_destroy(mymtd);

	iounmap((void *)itdm_map.virt);
	itdm_map.virt = 0;
}

module_init(init_itdm);
module_exit(cleanup_itdm);


MODULE_LICENSE("GPL");
MODULE_AUTHOR("Neuros Technologies");
MODULE_DESCRIPTION("NOR flash device driver");
