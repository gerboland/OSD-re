/*
 * Block driver for media (i.e., flash cards)
 *
 * Copyright 2002 Hewlett-Packard Company
 *
 * Use consistent with the GNU GPL is permitted,
 * provided that this copyright notice is
 * preserved in its entirety in all copies and derived works.
 *
 * HEWLETT-PACKARD COMPANY MAKES NO WARRANTIES, EXPRESSED OR IMPLIED,
 * AS TO THE USEFULNESS OR CORRECTNESS OF THIS CODE OR ITS
 * FITNESS FOR ANY PARTICULAR PURPOSE.
 *
 * Many thanks to Alessandro Rubini and Jonathan Corbet!
 *
 * Author:  Andrew Christian
 *          28 May 2002
 */
#include <linux/moduleparam.h>
#include <linux/module.h>
#include <linux/init.h>

#include <linux/sched.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/hdreg.h>
#include <linux/kdev_t.h>
#include <linux/blkdev.h>
#include <linux/devfs_fs_kernel.h>
#include <linux/ms/card.h>
#include <asm/system.h>
#include <asm/uaccess.h>

#include "mem_stk.h"
#include "mem_stk_queue.h"
#include "mem_stk_debug.h"

MODULE_LICENSE("GPL");

/*
 * max 8 partitions per card
 */
#define MS_SHIFT	3
static int major = 33;

struct ms_blk_data {
	spinlock_t	lock;
	struct gendisk	*disk;
	struct ms_queue queue;
	unsigned int	usage;
	unsigned int	block_bits;
};

static DECLARE_MUTEX(open_lock);

static struct ms_blk_data *ms_blk_get(struct gendisk *disk)
{
    struct ms_blk_data *md;

    down(&open_lock);
    md = disk->private_data;
    if (md && md->usage == 0)
        md = NULL;
    if (md)
        md->usage++;
    up(&open_lock);
    
    return md;
}

static void ms_blk_put(struct ms_blk_data *md)
{
    down(&open_lock);
    md->usage--;
    if (md->usage == 0) 
      {
	put_disk(md->disk);
	ms_cleanup_queue(&md->queue);
	kfree(md);
      }
    up(&open_lock);
}

static int ms_blk_open(struct inode *inode, struct file *filp)
{
    struct ms_blk_data *md;
    int ret = -ENXIO;

    md = ms_blk_get(inode->i_bdev->bd_disk);
    if (md) 
      {
	if (md->usage == 2)
	    check_disk_change(inode->i_bdev);
	ret = 0;

	if ((filp->f_mode & FMODE_WRITE) &&
	    ms_card_readonly(md->queue.card))
	  ret = -EROFS;
      }

    return ret;
}

static int ms_blk_release(struct inode *inode, struct file *filp)
{
    struct ms_blk_data *md = inode->i_bdev->bd_disk->private_data;

    ms_blk_put(md);
    return 0;
}

#define CARD_TYPE 0xAA
static int ms_blk_ioctl(struct inode *inode, struct file *filp, 
			unsigned int cmd, unsigned long arg)
{
    struct block_device *bdev = inode->i_bdev;

    if (cmd == HDIO_GETGEO) 
      {
	struct hd_geometry geo;

	ms_debug_msg ("IN IOCTL CALL HDIO_GETGEO\n");
	memset(&geo, 0, sizeof(struct hd_geometry));
	
	geo.cylinders	= get_capacity(bdev->bd_disk) / (4 * 16);
	geo.heads	= 4;
	geo.sectors	= 16;
	geo.start	= get_start_sect(bdev);
	
	return copy_to_user((void __user *)arg, &geo, sizeof(geo))
	  ? -EFAULT : 0;
      }
    else if (cmd == BLKRRPART) 
      {
	printk (KERN_INFO "Not implemented\n");
	return 0;
      }
    else if (cmd == CARD_TYPE)
      {
	struct ms_blk_data *md = inode->i_bdev->bd_disk->private_data;
	if (ms_card_ms_pro(md->queue.card))	
	    *(int*)arg = 0x1;
	else if (ms_card_ms(md->queue.card))
	    *(int *)arg = 0x2;
	else
	    *(int *)arg = 0;
	return 0;
      }

    return -ENOTTY;
}

static struct block_device_operations ms_bdops = {
	.open			= ms_blk_open,
	.release		= ms_blk_release,
	.ioctl			= ms_blk_ioctl,
	.owner			= THIS_MODULE,
};

struct ms_blk_request {
	struct ms_request	mrq;
	struct ms_command	cmd;
	struct ms_command	stop;
	struct ms_data		data;
};

int killed = 0;
static int ms_blk_prep_rq(struct ms_queue *mq, struct request *req)
{
    struct ms_blk_data *md = mq->data;
    int stat = BLKPREP_OK;

    /*
     * If we have no device, we haven't finished initialising.
     */
    if (rq_data_dir(req) == READ)
      {
	if (!md || !mq->card || ms_card_dead(mq->card)) 
	    stat = BLKPREP_KILL;
      }
    else
      {
	if (!md || !mq->card) 
	    stat = BLKPREP_KILL;
      }

    return stat;
}

static int ms_blk_issue_rq(struct ms_queue *mq, struct request *req)
{
    struct ms_blk_data *md = mq->data;
    struct ms_card *card = md->queue.card;
    int ret;
    char arg[16];
    int retry;
    int data_length = 0;
	
    if (ms_card_claim_host(card))
      {
	data_length = (req->nr_sectors >> (md->block_bits - 9))*0x200;
	ms_card_set_dead(card);
	goto cmd_err;
      }
	
    do {
        struct ms_blk_request brq;
	char buf[8];
	
	memset ((void*)buf,0,sizeof(buf));
	memset(&brq, 0, sizeof(struct ms_blk_request));

	brq.mrq.cmd = &brq.cmd;
	brq.mrq.data = &brq.data;
		
	buf[0] = req->sector;		
	brq.cmd.arg = buf;
	brq.cmd.flags = DATA_RW_CMD;
	brq.data.blksz_bits = md->block_bits;
	brq.data.blocks = req->nr_sectors >> (md->block_bits - 9);
	brq.stop.opcode = STOP;
	brq.stop.arg = 0;
	brq.stop.flags = REG_RW_CMD;
	
	if (rq_data_dir(req) == READ) 
	  {
	    brq.cmd.opcode	= READ_PAGE_DATA;
	    brq.data.flags |= MS_DATA_READ;
	  } 
	else 
	  {
	    brq.cmd.opcode = WRITE_PAGE_DATA;
	    brq.cmd.flags = DATA_RW_CMD;
	    brq.data.flags |= MS_DATA_WRITE;
	  }
		
	brq.mrq.stop = brq.data.blocks > 1 ? &brq.stop : NULL;
	brq.data.sg = mq->sg;
	brq.data.sg_len = blk_rq_map_sg(req->q, req, brq.data.sg);
#if 1
	arg[0] = 0x15;
	arg[1] = 0x08;
	arg[2] = 0x10;
	arg[3] = 0x0a;
		
	if (set_card_regadd(card->host,arg) != SUCCESS)
	  {
	    ms_card_set_dead(card);
	    goto cmd_err;
	  }
		
	if (card->sys_info.interface_type)
	  {
	    if (ms_card_ms(card))
	      {
		if (set_card_regiters(card->host,(req->nr_sectors),PARALLEL,req->sector,
				      card->sys_info.block_size,USER_DATA,0x20,0xF8,0xFF,0,0xa))
		  {
		    ms_card_set_dead(card);
		    goto cmd_err;
		  }
	      }
	    else
	      {
		if (set_card_regiters(card->host,(req->nr_sectors),PARALLEL,req->sector,
				      card->sys_info.block_size,USER_DATA,0x20,0,0,0,0xa))
		  {
		    ms_card_set_dead(card);
		    goto cmd_err;
		  }
	      }
	  }
	else
	  {
	    if (set_card_regiters(card->host,(req->nr_sectors),SERIAL,req->sector,
				  card->sys_info.block_size,USER_DATA,0x20,0xF8,0xFF,0,0xa))
	      {
		ms_card_set_dead(card);
		goto cmd_err;
	      }
	  }

	retry = 0;
	do {
	    if (retry++ > 2500)
	      {
		ms_card_set_dead(card);
		goto cmd_err;
	      }
	} while ((get_int(card->host,(char*)buf,NORM_COMP))!=SUCCESS);

	if (rq_data_dir(req) == READ)
	  {
	    if (ms_card_ms_pro(card))
	      {
		if (set_cmd(card->host,READ_DATA,REG_RW_CMD,NORM_DATA_TRANS) != SUCCESS)
		  {
		    ms_card_set_dead(card);
		    goto cmd_err;	
		  }
	      }
	    else if (ms_card_ms(card))
	      {
		if (set_cmd(card->host,BLOCK_READ,REG_RW_CMD,NORM_DATA_TRANS) != SUCCESS )
		  {
		    if (set_cmd(card->host,BLOCK_END,REG_RW_CMD,NORM_DATA_TRANS) == FAILURE)
		      {
			ms_card_set_dead(card);
			goto cmd_err;
		      }
		    if (set_cmd(card->host,CLEAR_BUF,REG_RW_CMD,NORM_DATA_TRANS) == FAILURE)
		      {
			ms_card_set_dead(card);
			goto cmd_err;	
		      }
		  }
	      }
	  }
	else
	  {
	    if (ms_card_ms_pro(card))
	      {
		if (set_cmd(card->host,WRITE_DATA,REG_RW_CMD,NORM_DATA_TRANS))
		  {
		    ms_card_set_dead(card);
		    goto cmd_err;
		  }
	      }
	    else
	      {
		if (ms_card_ms(card))
		  {
		    short page_add;
		    
		    card->cur_blk_data = (unsigned char*)kmalloc ((card->sys_info.block_size*1024),GFP_KERNEL);
		    card->cur_blk_no = get_blk_number(card->host,req->sector,card->sys_info.block_size);
		    page_add = (req->sector - ((req->sector/(card->sys_info.block_size * 2)) * (card->sys_info.block_size * 2)));
		    if (card->sys_info.interface_type)
		      {
			if (set_card_regiters(card->host,(req->nr_sectors),PARALLEL,(req->sector-page_add),
					      card->sys_info.block_size,USER_DATA,0x00,0xF8,0xFF,0,0xa))
			  {
			    ms_card_set_dead(card);
			    kfree (card->cur_blk_data);
			    goto cmd_err;
			  }
		      }
		    else
		      {
			if (set_card_regiters(card->host,(req->nr_sectors),SERIAL,(req->sector-page_add),
					      card->sys_info.block_size,USER_DATA,0x00,0xF8,0xFF,0,0xa))
			  {
			    ms_card_set_dead(card);
			    kfree (card->cur_blk_data);
			    goto cmd_err;
			  }
		      }
		    if (set_cmd(card->host,BLOCK_READ,REG_RW_CMD,NORM_DATA_TRANS) != SUCCESS )
		      {
			set_cmd(card->host,BLOCK_END,REG_RW_CMD,NORM_DATA_TRANS);
			set_cmd(card->host,CLEAR_BUF,REG_RW_CMD,NORM_DATA_TRANS);
			ms_card_set_dead(card);
			kfree (card->cur_blk_data);
			goto cmd_err;	
		      }
		    if (ms_wait_for_read_data_blk(card->host,&brq.mrq,card->sys_info.block_size*1024,card->cur_blk_data)==FAILURE)
		      {
			kfree (card->cur_blk_data);
			goto cmd_err;
		      }
		    update_ms_page(card->host,page_add,card->sys_info.block_size,&brq.mrq,card->cur_blk_data);
		    if (set_cmd(card->host,BLOCK_ERASE,REG_RW_CMD,NORM_COMP) != SUCCESS )
		      {
			set_cmd(card->host,BLOCK_END,REG_RW_CMD,NORM_COMP);
			set_cmd(card->host,CLEAR_BUF,REG_RW_CMD,NORM_COMP);
			ms_card_set_dead(card);
			kfree (card->cur_blk_data);
			goto cmd_err;
		      }
		    if (card->sys_info.interface_type)
		      {
			if (set_card_regiters(card->host,(req->nr_sectors),PARALLEL,(req->sector-page_add),
					      card->sys_info.block_size,USER_DATA,0x00,0xF8,0xFF,0,0xa))
			  {
			    ms_card_set_dead(card);
			    kfree (card->cur_blk_data);
			    goto cmd_err;
			  }
		      }
		    else
		      {
			if (set_card_regiters(card->host,(req->nr_sectors),SERIAL,(req->sector-page_add),
					      card->sys_info.block_size,USER_DATA,0x00,0xF8,0xFF,0,0xa))
			  {
			    ms_card_set_dead(card);
			    kfree (card->cur_blk_data);
			    goto cmd_err;
			  }
		      }
					
		    if (set_cmd(card->host,BLOCK_WRITE,REG_RW_CMD,NORM_DATA_TRANS) != SUCCESS )
		      {
			set_cmd(card->host,BLOCK_END,REG_RW_CMD,NORM_DATA_TRANS);
			set_cmd(card->host,CLEAR_BUF,REG_RW_CMD,NORM_DATA_TRANS);
			ms_card_set_dead(card);
			kfree (card->cur_blk_data);
			goto cmd_err;
		      }
		    if (ms_wait_for_write_data_blk(card->host,&brq.mrq,card->sys_info.block_size*1024,card->cur_blk_data)==FAILURE)
		      {
			kfree (card->cur_blk_data);
			goto cmd_err;
		      }
		    kfree (card->cur_blk_data);
		  }
	      }
	  }
#endif
	if (rq_data_dir(req) == READ) 
	  {
	    if (ms_wait_for_read_data(card->host,&brq.mrq) == FAILURE)
	      {
		ms_card_set_dead(card);	
		brq.data.error = 1;
	      }
	  }
	else
	  {
	    if (ms_card_ms_pro(card))
	      {
		if (ms_wait_for_write_data(card->host,&brq.mrq) == FAILURE)
		  {
		    ms_card_set_dead(card);
		    brq.data.error = 1;
		  }
	      }
	  }

	if (brq.cmd.error) 
	  {
	    ms_debug_msg(KERN_ERR "%s: error %d sending read/write command\n",
			 req->rq_disk->disk_name, brq.cmd.error);
	    ms_card_set_dead(card);
	    goto cmd_err;
	  }
	if (brq.data.error) 
	  {
	    ms_debug_msg(KERN_ERR "%s: error %d transferring data\n",
			 req->rq_disk->disk_name, brq.data.error);
	    ms_card_set_dead(card);	
	    goto cmd_err;
	  }
	if (brq.stop.error) 
	  {
	    ms_debug_msg(KERN_ERR "%s: error %d sending stop command\n",
			 req->rq_disk->disk_name, brq.stop.error);
	    ms_card_set_dead(card);
	    goto cmd_err;
	  }
	retry = 0;

#if 1 
	do {
	    if (retry++ > 2500)
	      {
		ms_card_set_dead(card);
		goto cmd_err;
	      }
	} while ((get_int(card->host,(char*)buf,NORM_COMP))!=SUCCESS);
#endif
	/*
	 * A block was successfully transferred.
	 */
	spin_lock_irq(&md->lock);
	ret = end_that_request_chunk(req, 1, brq.data.bytes_xfered);	
	if (!ret) 
	  {
	  /*
	   * The whole request completed successfully.
	   */
	    add_disk_randomness(req->rq_disk);
	    blkdev_dequeue_request(req);
	    end_that_request_last(req);
	  }
	spin_unlock_irq(&md->lock);
    } while (ret);
	
    arg[0] = 0x02;
    arg[1] = 0x08;
    arg[2] = 0x10;
    arg[3] = 0x08;
    
    if (set_card_regadd(card->host,arg))
      {
	ms_card_set_dead(card);
	goto cmd_err;
      }
    if (set_card_regiters(card->host,0x02,SERIAL,0x0,0,0,0x20,0,0,0,8))
      {
	ms_card_set_dead(card);
	goto cmd_err;
      }
    ms_card_release_host(card);
    
    return 1;
	
 cmd_err:
    ms_card_release_host(card);
    /*
     * This is a little draconian, but until we get proper
     * error handling sorted out here, its the best we can
     * do - especially as some hosts have no idea how much
     * data was transferred before the error occurred.
     */
    data_length = (req->nr_sectors >> (md->block_bits - 9))*0x200;
    
    if (rq_data_dir(req) == READ)
      {
	spin_lock_irq(&md->lock);
	do {
	    ret = end_that_request_chunk(req,-5,data_length);
	} while (ret);	
	add_disk_randomness(req->rq_disk);
	blkdev_dequeue_request(req);
	end_that_request_last(req);
	spin_unlock_irq(&md->lock);
	return 0;
      }
    else
      {
	spin_lock_irq(&md->lock);
	do {
	    ret = end_that_request_chunk(req,-5,data_length);
	} while (ret);
	add_disk_randomness(req->rq_disk);
	blkdev_dequeue_request(req);
	end_that_request_last(req);
	spin_unlock_irq(&md->lock);
      }
    return 0;
}

#define MS_NUM_MINORS	(256 >> MS_SHIFT)

static unsigned long dev_use[MS_NUM_MINORS/(8*sizeof(unsigned long))];

static struct ms_blk_data *ms_blk_alloc(struct ms_card *card)
{
    struct ms_blk_data *md;
    int devidx, ret;

    devidx = find_first_zero_bit(dev_use, MS_NUM_MINORS);
    if (devidx >= MS_NUM_MINORS)
        return ERR_PTR(-ENOSPC);
    __set_bit(devidx, dev_use);

    md = kmalloc(sizeof(struct ms_blk_data), GFP_KERNEL);
    if (md) 
      {
	memset(md, 0, sizeof(struct ms_blk_data));
	md->disk = alloc_disk(1 << MS_SHIFT);
	if (md->disk == NULL) 
	  {
	    kfree(md);
	    md = ERR_PTR(-ENOMEM);
	    goto out;
	  }

	spin_lock_init(&md->lock);
	md->usage = 1;
	
	ret = ms_init_queue(&md->queue, card, &md->lock);
	if (ret) 
	  {
	    put_disk(md->disk);
	    kfree(md);
	    md = ERR_PTR(ret);
	    goto out;
	  }
	killed = 0;
	md->queue.prep_fn = ms_blk_prep_rq;
	md->queue.issue_fn = ms_blk_issue_rq;
	md->queue.data = md;
	
	md->disk->major	= major;	
	md->disk->first_minor = devidx << MS_SHIFT;
	md->disk->fops = &ms_bdops;
	md->disk->private_data = md;
	md->disk->queue = md->queue.queue;
	md->disk->driverfs_dev = &card->dev;
	
	sprintf(md->disk->disk_name, "mem_stkblk%d", devidx);
	sprintf(md->disk->devfs_name, "mem_stk/blk%d", devidx);
	
	md->block_bits = 9;/* 512 bytes */

	blk_queue_hardsect_size(md->queue.queue, 1 << md->block_bits);
	if (ms_card_ms(card))
	  {
	    set_capacity(md->disk,(card->sys_info.block_size * card->sys_info.user_area_block));
	  }
	else
	  { 
	    if (ms_card_ms_pro(card))
	      {
		set_capacity(md->disk,(card->sys_info.unit_size * card->sys_info.block_size * card->sys_info.user_area_block));
	      }
	  }
      }
 out:
    return md;
}

static int ms_blk_set_blksize(struct ms_blk_data *md, struct ms_card *card)
{
    int err = 0;
	
    ms_card_claim_host(card);
    if (card->sys_info.interface_type)
      {
	if (ms_card_ms_pro(card))
	    err = set_card_regiters(card->host,((card->sys_info.unit_size) >> 9),PARALLEL,0x0,0,0,0x20,0,0,0,8);

	if (ms_card_ms(card))
	    err = set_card_regiters(card->host,((card->sys_info.unit_size) >> 9),SERIAL,0x0,0,0,0x20,0,0,0,8);
      }
    else
        err = set_card_regiters(card->host,((card->sys_info.unit_size) >> 9),SERIAL,0x0,0,0,0x20,0,0,0,8);
    ms_card_release_host(card);
    
    if (err == FAILURE) 
      {
	ms_debug_msg(KERN_ERR "%s: unable to set block size to %d\n",
		     md->disk->disk_name,err);
	return -EINVAL;
      }
    return 0;
}

static int ms_blk_probe(struct ms_card *card)
{
    struct ms_blk_data *md;
    int err;

    /*
     * Check that the card supports the command class(es) we need.
     */
    if (ms_card_ms(card))
      {
	if (card->sys_info.page_size < 0x200) 
	  {
	    printk(KERN_WARNING "%s: read blocksize too small (%u)\n",
		   ms_card_id(card),card->sys_info.page_size);
	    return -ENODEV;
	  }
      }
    else
      { 
	if (ms_card_ms_pro(card))
	  {
	    if (card->sys_info.unit_size < 0x200) 
	      {
		printk(KERN_WARNING "%s: read blocksize too small (%u)\n",
		       ms_card_id(card),card->sys_info.unit_size);
		return -ENODEV;
	      }
	  }
      }

    md = ms_blk_alloc(card);

    if (IS_ERR(md))
        return PTR_ERR(md);

    err = ms_blk_set_blksize(md, card);
    if (err)	
        goto out;
	
    if (ms_card_ms(card))
      {
	printk(KERN_INFO "DiskName %s: card-id: %s  card capacity: %luKiB readonly: %s\n",
	       md->disk->disk_name, ms_card_id(card),
	       (unsigned long)(card->sys_info.block_size * card->sys_info.user_area_block),
	       ms_card_readonly(card)?"(ro)":"(rw)");
      }
    else
      { 
	if (ms_card_ms_pro(card))
	  {
	    printk(KERN_INFO "DiskName %s: card-id: %s  card capacity: %luKiB readonly: %s\n",
		   md->disk->disk_name, ms_card_id(card),
		   (unsigned long)(card->sys_info.unit_size * card->sys_info.block_size * 
				   card->sys_info.user_area_block)>>10, 
		   ms_card_readonly(card)?"(ro)":"(rw)");
	  }
      }
	
    ms_set_drvdata(card, md);
    add_disk(md->disk);
    return 0;
    
 out:
    ms_blk_put(md);
    
    return err;
}

static void ms_blk_remove(struct ms_card *card)
{
    struct ms_blk_data *md = ms_get_drvdata(card);
    unsigned char arg[8];

    memset ((void*)arg,0,sizeof(arg));
    if (md) 
      {
	int devidx;
	del_gendisk(md->disk);
	
	/*
	 * I think this is needed.
	 */
	md->disk->queue = NULL;
	devidx = md->disk->first_minor >> MS_SHIFT;
	__clear_bit(devidx, dev_use);
	
	ms_blk_put(md);
      }
    ms_set_drvdata(card, NULL);
}
EXPORT_SYMBOL(ms_blk_remove);

#ifdef CONFIG_PM
static int ms_blk_suspend(struct ms_card *card, pm_message_t state)
{
    struct ms_blk_data *md = ms_get_drvdata(card);

    if (md) 
        ms_queue_suspend(&md->queue);

    return 0;
}

static int ms_blk_resume(struct ms_card *card)
{
    struct ms_blk_data *md = ms_get_drvdata(card);

    if (md) 
      {
	ms_blk_set_blksize(md, card);
	ms_queue_resume(&md->queue);
      }
    return 0;
}
#else
#define	ms_blk_suspend	NULL
#define ms_blk_resume	NULL
#endif

static struct ms_driver ms_driver = {
	.drv		= {
		.name	= "mem_stkblk",
	},
	.probe		= ms_blk_probe,
	.remove		= ms_blk_remove,
	.suspend	= ms_blk_suspend,
	.resume		= ms_blk_resume,
};

static int __init ms_blk_init(void)
{
    int res = -ENOMEM;

    res = register_blkdev(major, "mem_stk");
    if (res < 0) 
      {
	ms_debug_msg(KERN_WARNING "Unable to get major %d for MS media: %d\n",
		     major, res);
	goto out;
      }
    if (major == 0)
        major = res;
    
    devfs_mk_dir("mem_stk");
    return ms_register_driver(&ms_driver);
    
 out:
    return res;
}

static void __exit ms_blk_exit(void)
{
    ms_unregister_driver(&ms_driver);
    devfs_remove("mem_stk");
    unregister_blkdev(major, "mem_stk");
}

module_init(ms_blk_init);
module_exit(ms_blk_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("MEMORY STICK block device driver");

module_param(major, int, 0444);
MODULE_PARM_DESC(major, "specify the major device number for MS block driver");
