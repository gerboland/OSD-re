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
#include <linux/blkdev.h>
#include <linux/ms/card.h>
#include <linux/ms/host.h>
#include "mem_stk_queue.h"
#include "mem_stk_debug.h"

MODULE_LICENSE("GPL");

#define MS_QUEUE_EXIT		(1 << 0)
#define MS_QUEUE_SUSPENDED	(1 << 1)

/*
 * Prepare a MMC request.  Essentially, this means passing the
 * preparation off to the media driver.  The media driver will
 * create a ms_io_request in req->special.
 */
static int ms_prep_request(struct request_queue *q, struct request *req)
{
    struct ms_queue *mq = q->queuedata;
    int ret = BLKPREP_KILL;

    if (req->flags & REQ_SPECIAL) 
      {
	/*
	 * Special commands already have the command
	 * blocks already setup in req->special.
	 */
	BUG_ON(!req->special);
	ret = BLKPREP_OK;
      } 
    else if (req->flags & (REQ_CMD | REQ_BLOCK_PC)) 
      {
	/*
	 * Block I/O requests need translating according
	 * to the protocol.
	 */
	ret = mq->prep_fn(mq, req);
      } 
    else 
      {
	/*
	 * Everything else is invalid.
	 */
	blk_dump_rq_flags(req, "MMC bad request");
      }

    if (ret == BLKPREP_OK)
        req->flags |= REQ_DONTPREP;
    
    return ret;
}
static int ms_queue_thread(void *d)
{
    struct ms_queue *mq = d;
    struct request_queue *q = mq->queue;
    DECLARE_WAITQUEUE(wait, current);

    /*
     * Set iothread to ensure that we aren't put to sleep by
     * the process freezing.  We handle suspension ourselves.
     */
    current->flags |= PF_MEMALLOC|PF_NOFREEZE;

    daemonize("msqd");
    complete(&mq->thread_complete);
    down(&mq->thread_sem);
    add_wait_queue(&mq->thread_wq, &wait);
    
    do {
        struct request *req = NULL;

	spin_lock_irq(q->queue_lock);
	set_current_state(TASK_INTERRUPTIBLE);
	if (!blk_queue_plugged(q))
	    mq->req = req = elv_next_request(q);
	spin_unlock_irq(q->queue_lock);
	
	if (!req) 
	  {
	    if ((mq->flags & MS_QUEUE_EXIT))
		break;
	    up(&mq->thread_sem);
	    schedule();
	    down(&mq->thread_sem);
	    continue;
	  }
	set_current_state(TASK_RUNNING);
	mq->issue_fn(mq, req);
    } while (1);

    remove_wait_queue(&mq->thread_wq, &wait);
    up(&mq->thread_sem);
    complete_and_exit(&mq->thread_complete,0);
    return 0;
}

/*
 * Generic MS request handler.  This is called for any queue on a
 * particular host.  When the host is not busy, we look for a request
 * on any queue on this host, and attempt to issue it.  This may
 * not be the queue we were asked to process.
 */
static void ms_request(request_queue_t *q)
{
  struct ms_queue *mq = q->queuedata;
  
  if (!mq->req)
      wake_up(&mq->thread_wq);
}

/**
 * ms_init_queue - initialise a queue structure.
 * @mq: ms queue
 * @card: ms card to attach this queue
 * @lock: queue lock
 *
 * Initialise a MS card request queue.
 */
int ms_init_queue(struct ms_queue *mq, struct ms_card *card, spinlock_t *lock)
{
    struct ms_host *host = card->host;
    u64 limit = BLK_BOUNCE_HIGH;
    int ret;
    
    if (host->dev->dma_mask && *host->dev->dma_mask)
        limit = *host->dev->dma_mask;

    mq->card = card;
    mq->queue = blk_init_queue(ms_request, lock);
    if (!mq->queue)
        return -ENOMEM;

    blk_queue_prep_rq(mq->queue, ms_prep_request);
    blk_queue_bounce_limit(mq->queue, limit);
    blk_queue_max_sectors(mq->queue, host->max_sectors);
    blk_queue_max_phys_segments(mq->queue, host->max_phys_segs);
    blk_queue_max_hw_segments(mq->queue, host->max_hw_segs);
    blk_queue_max_segment_size(mq->queue, host->max_seg_size);
    mq->queue->queuedata = mq;
    mq->req = NULL;

    mq->sg = kmalloc(sizeof(struct scatterlist) * host->max_phys_segs,
		     GFP_KERNEL);
    if (!mq->sg) 
      {
	ret = -ENOMEM;
	goto cleanup;
      }

    init_completion(&mq->thread_complete);
    init_waitqueue_head(&mq->thread_wq);
    init_MUTEX(&mq->thread_sem);

    ret = kernel_thread(ms_queue_thread, mq, CLONE_KERNEL);
    if (ret >= 0) 
      {
	wait_for_completion(&mq->thread_complete);
	init_completion(&mq->thread_complete);
	ret = 0;
	goto out;
      }
    
 cleanup:
    kfree(mq->sg);
    mq->sg = NULL;
    
    blk_cleanup_queue(mq->queue);
 out:
    return ret;
}
EXPORT_SYMBOL(ms_init_queue);

void ms_cleanup_queue(struct ms_queue *mq)
{
    mq->flags |= MS_QUEUE_EXIT;
    wake_up(&mq->thread_wq);
    wait_for_completion(&mq->thread_complete);
    kfree(mq->sg);
    mq->sg = NULL;
    blk_cleanup_queue(mq->queue);
    mq->card = NULL;
}
EXPORT_SYMBOL(ms_cleanup_queue);

/**
 * ms_queue_suspend - suspend a MMC request queue
 * @mq: MMC queue to suspend
 *
 * Stop the block request queue, and wait for our thread to
 * complete any outstanding requests.  This ensures that we
 * won't suspend while a request is being processed.
 */
void ms_queue_suspend(struct ms_queue *mq)
{
    request_queue_t *q = mq->queue;
    unsigned long flags;

    if (!(mq->flags & MS_QUEUE_SUSPENDED)) 
      {
	mq->flags |= MS_QUEUE_SUSPENDED;
	spin_lock_irqsave(q->queue_lock, flags);
	blk_stop_queue(q);
	spin_unlock_irqrestore(q->queue_lock, flags);
	down(&mq->thread_sem);
    }
}
EXPORT_SYMBOL(ms_queue_suspend);

/**
 * ms_queue_resume - resume a previously suspended MMC request queue
 * @mq: MMC queue to resume
 */
void ms_queue_resume(struct ms_queue *mq)
{
    request_queue_t *q = mq->queue;
    unsigned long flags;

    if (mq->flags & MS_QUEUE_SUSPENDED) 
      {
	mq->flags &= ~MS_QUEUE_SUSPENDED;
	up(&mq->thread_sem);
	spin_lock_irqsave(q->queue_lock, flags);
	blk_start_queue(q);
	spin_unlock_irqrestore(q->queue_lock, flags);
      }
}
EXPORT_SYMBOL(ms_queue_resume);
