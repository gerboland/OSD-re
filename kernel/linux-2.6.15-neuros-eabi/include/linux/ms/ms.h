/*
 *  linux/include/linux/ms/ms.h
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifndef MS_H
#define MS_H

#include <linux/list.h>
#include <linux/interrupt.h>
#include <linux/device.h>

struct request;
struct ms_data;
struct ms_request;

struct ms_command {
	u16			opcode;
	u8* 			arg;//Command format is 16-bit format
	u16 			size;
	u8			resp[8];
	u16 			status0;
	u16 			status1;
	u16 			cmd_index;
	
	unsigned int		flags;		/* expected response type */

/*
 * These are the response types, and correspond to valid bit
 * patterns of the above flags.  One additional valid pattern
 * is all zeros, which means we don't expect a response.
 */

	unsigned int		retries;	/* max number of retries */
	unsigned int		error;		/* command error */
	struct ms_data		*data;		/* data segment associated with cmd */
	struct ms_request	*mrq;		/* assoicated request */
};

struct ms_data {
	unsigned int		timeout_ns;	/* data timeout (in ns, max 80ms) */
	unsigned int		timeout_clks;	/* data timeout (in clocks) */
	unsigned int		blksz_bits;	/* data block size */
	unsigned int		blocks;		/* number of blocks */
	unsigned int		error;		/* data error */
	unsigned int		flags;

#define MS_DATA_WRITE	(1 << 11)
#define MS_DATA_READ	(1 << 10)

	unsigned int		bytes_xfered;

	struct ms_command	*stop;		/* stop command */
	struct ms_request	*mrq;		/* assoicated request */

	unsigned int		sg_len;		/* size of scatter list */
	struct scatterlist	*sg;		/* I/O scatter list */
};

struct ms_request {
	struct ms_command	*cmd;
	struct ms_data		*data;
	struct ms_command	*stop;

	void			*done_data;	/* completion data */
	void			(*done)(struct ms_request *);/* completion function */
};

struct ms_host;
struct ms_card;

extern int ms_wait_for_req(struct ms_host *, struct ms_request *);
extern int ms_wait_for_cmd(struct ms_host *, struct ms_command *, int);
extern int ms_wait_for_app_cmd(struct ms_host *, unsigned int,
	struct ms_command *, int);

extern int __ms_claim_host(struct ms_host *host, struct ms_card *card);

static inline void ms_claim_host(struct ms_host *host)
{
	__ms_claim_host(host, (struct ms_card *)-1);
}

extern void ms_release_host(struct ms_host *host);

#endif
