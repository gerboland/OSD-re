/*
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

#ifndef MMC_QUEUE_H
#define MMC_QUEUE_H

struct request;
struct task_struct;

struct ms_queue {
	struct ms_card		*card;
	struct completion	thread_complete;
	wait_queue_head_t	thread_wq;
	struct semaphore	thread_sem;
	unsigned int		flags;
	struct request		*req;
	int			(*prep_fn)(struct ms_queue *, struct request *);
	int			(*issue_fn)(struct ms_queue *, struct request *);
	void			*data;
	struct request_queue	*queue;
	struct scatterlist	*sg;
};

struct ms_io_request {
	struct request		*rq;
	int			num;
	struct ms_command	selcmd;		/* ms_queue private */
	struct ms_command	cmd[4];		/* max 4 commands */
};

extern int ms_init_queue(struct ms_queue *, struct ms_card *, spinlock_t *);
extern void ms_cleanup_queue(struct ms_queue *);
extern void ms_queue_suspend(struct ms_queue *);
extern void ms_queue_resume(struct ms_queue *);

#endif
