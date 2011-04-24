/*
 *  linux/include/linux/mmc/host.h
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 *  Host driver specific definitions.
 */
#ifndef LINUX_MMC_HOST_H
#define LINUX_MMC_HOST_H

#include <linux/ms/ms.h>

struct ms_ios {
	unsigned int	clock;			/* clock rate */
	unsigned short	vdd;
	unsigned char	bus_mode;		/* command output mode */
	unsigned char	chip_select;		/* SPI chip select */
	unsigned char	power_mode;		/* power supply mode */

#define MS_POWER_OFF		0
#define MS_POWER_UP		1
#define MS_POWER_ON		2

	unsigned char	bus_width;		/* data bus width */

#define MS_BUS_WIDTH_1		0
#define MS_BUS_WIDTH_4		2
	unsigned int 	change_clk; // 1-change to 40Mhz 0-Maintain the old setting
};

struct ms_host_ops {
	void	(*request)(struct ms_host *host, struct ms_request *req);
	void	(*set_ios)(struct ms_host *host, struct ms_ios *ios);
	int	(*get_ro)(struct ms_host *host);
};

struct ms_card;
struct device;

struct ms_host {
	struct device		*dev;
	struct class_device	class_dev;
	int			index;
	struct ms_host_ops	*ops;
	unsigned int		f_min;
	unsigned int		f_max;
	u32			ocr_avail;
	int (*read_attb_data)(struct ms_host *host,void *buf,short blksz_bits);
	int (*readdata)(struct ms_host *host,struct ms_request *mrq);
	int (*writedata)(struct ms_host *host,struct ms_request *mrq);
	int (*src_data)(struct ms_host *host,struct ms_request *mrq,unsigned int *relative,char* scr_reg);
	int (*read_blk_data)(unsigned char* data,int data_size,struct ms_host *host,struct ms_request *mrq);
	int (*write_blk_data)(unsigned char* data,int data_size,struct ms_host *host,struct ms_request *mrq);
	void (*update_ms_page)(struct ms_host *host,short page_add,short block_size,struct ms_request *mrq,unsigned char*cur_blk_data);
	int (*card_read)(char *to,char *from,int nbytes);
	void (*go_to_transfer)(void);
	void (*enable_interrupt)(void);
	void (*disable_interrupt)(void);
	void (*change_clk25m)(void);
	
	char *scr_register;
	unsigned long		caps;		/* Host capabilities */

#define MS_CAP_4_BIT_DATA	(1 << 0)	/* Can the host do 4 bit transfers */

	/* host specific block data */
	unsigned int		max_seg_size;	/* see blk_queue_max_segment_size */
	unsigned short		max_hw_segs;	/* see blk_queue_max_hw_segments */
	unsigned short		max_phys_segs;	/* see blk_queue_max_phys_segments */
	unsigned short		max_sectors;	/* see blk_queue_max_sectors */
	unsigned short		unused;

	/* private data */
	struct ms_ios		ios;		/* current io bus settings */
	u32			ocr;		/* the current OCR setting */

	unsigned int		mode;		/* current card mode of host */
#define MS_MODE_MS		0
#define MS_MODE_MS_PRO		1

	struct list_head	cards;		/* devices attached to this host */

	unsigned short *log_address;

	wait_queue_head_t	wq;
	wait_queue_head_t	wait_data;
	spinlock_t		lock;		/* card_busy lock */
	struct ms_card		*card_busy;	/* the MS card claiming host */
	struct ms_card		*card_selected;	/* the selected MS card */

	struct work_struct	detect;
	struct work_struct	read_write;

	unsigned long		private[0] ____cacheline_aligned;
};

extern struct ms_host *ms_alloc_host(int extra, struct device *);
extern int ms_add_host(struct ms_host *);
extern void ms_remove_host(struct ms_host *);
extern void ms_free_host(struct ms_host *);

static inline void *ms_priv(struct ms_host *host)
{
	return (void *)host->private;
}

#define ms_dev(x)	((x)->dev)
#define ms_hostname(x)	((x)->class_dev.class_id)

extern int ms_suspend_host(struct ms_host *, pm_message_t);
extern int ms_resume_host(struct ms_host *);

extern void ms_detect_change(struct ms_host *, unsigned long delay);
extern void ms_request_done(struct ms_host *, struct ms_request *);

#endif

