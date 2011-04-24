/*
 * linux/drivers/mmc/davinci.c
 *
 * TI DaVinci MMC controller file
 *
 * Copyright (C) 2004 Texas Instruments.
 *
 * ----------------------------------------------------------------------------
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
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 * ----------------------------------------------------------------------------
 Modifications:
 ver. 1.0: Oct 2005, Purushotam Kumar   Initial version
 ver 1.1:  Nov  2005, Purushotam Kumar  Solved bugs
 ver 1.2:  Jan  2066, Purushotam Kumar   Added card remove insert support
 -
 *

 */

#include <linux/config.h>
#include <linux/module.h>
#include <linux/tty.h>
#include <linux/ioport.h>
#include <linux/init.h>
#include <linux/console.h>
#include <linux/blkdev.h>
#include <linux/device.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/hardware.h>
#include <linux/mmc/host.h>
#include <linux/mmc/card.h>
#include <linux/mmc/protocol.h>
#include <asm/arch/irqs.h>
#include <asm/arch/hardware.h>
#include <asm/hardware/clock.h>

#include "davinci_mmc.h"
#include <asm/arch/edma.h>

extern void davinci_clean_channel(int chno);

//#define MMC_DEBUG
#define MMCSD_INIT_CLOCK 		200000	/*MMCSD Init clock in Hz in opendain mode */
#define DRIVER_NAME 			"mmc0"
#define MMCINT_INTERRUPT    		IRQ_MMCINT
#define MMCSD_REGS_BASE_ADDR  		DAVINCI_MMC_SD_BASE
#define TCINTEN 			(0x1<<20)

#ifdef MMC_CARD_INS_REM_SUPPORT
#define MULTIPILER_TO_HZ 1	/* This macro could not be defined to 0 (ZERO) or -ve value.This value is multiplied to "HZ" 
				   while requesting for timer interrupt every time for probing card. */
#endif

struct clk *mmcclkp = NULL;
mmcsd_configdef mmcsdcfg = {
	32,		/**< read write thresholds (in bytes) can be any power of 2 from 2 to 64 */
	1		/**< To use the DMA or not-- 1- Use DMA, 0-Interrupt mode*/
};

mmcsd_regs *mmcsdregs;
static unsigned int mmc_input_clk = 0;

#ifdef MMC_CARD_INS_REM_SUPPORT
static unsigned int iscardbusy = 0;	/* Used to identify whether card being used currently by linux core or not */
static unsigned int iscard_detect_progress = 0;	/* used to identify whether card probe(detection) is currently in progress or not */
static unsigned int is_init_progress = 0;	/* used to identify whether core is icurrently initilizing the card or not */
static unsigned int is_req_queued_up = 0;	/*used to identify whether core request has been queue up or 
						   not because request has come when card detection/probe was in progress */
static struct mmc_host *que_mmc_host = NULL;	/* data struture to queue one request */
static struct mmc_request *que_mmc_request = NULL;	/* data structure to queue one request */
#endif

static unsigned int iscardinitialized = 0;	/* tells whether card is initizlzed or not */
static unsigned int new_cardstate = 0;	/* tells current state of card */

#ifdef MMC_CARD_INS_REM_SUPPORT
static DEFINE_SPINLOCK(mmc_lock);
#endif

static void mmc_davinci_start_command(struct mmc_davinci_host *host,
				      struct mmc_command *cmd)
{
	u32 cmdreg = 0;
	u32 resptype = 0;
	u32 cmdtype = 0;
	int byteCnt = 0, i = 0;
#ifdef MMC_CARD_INS_REM_SUPPORT
	unsigned long flags;
#endif

#ifdef MMC_DEBUG
	printk("\nMMCSD : CMD%d, argument 0x%08x", cmd->opcode, cmd->arg);
	if (cmd->flags & MMC_RSP_SHORT)
		printk(", 32-bit response");
	if (cmd->flags & MMC_RSP_LONG)
		printk(", 128-bit response");
	if (cmd->flags & MMC_RSP_CRC)
		printk(", CRC");
	if (cmd->flags & MMC_RSP_BUSY)
		printk(", busy notification");
	else
		printk(", No busy notification");
	printk("\n");
#endif
	host->cmd = cmd;

	/* Protocol layer does not provide response type,
	 * but our hardware needs to know exact type, not just size!
	 */
	switch (cmd->flags & MMC_RSP_MASK) {
	case MMC_RSP_NONE:
		/* resp 0 */
		break;
	case MMC_RSP_SHORT:
		/* resp 1, resp 1b */
		/* OR resp 3!! (assume this if bus is set opendrain) */
		if (host->bus_mode == MMC_BUSMODE_OPENDRAIN) {
			resptype = 3;
			if (cmd->opcode == 3)
				resptype = 1;
		} else {
			resptype = 1;
		}
		break;
	case MMC_RSP_LONG:
		/* resp 2 */
		resptype = 2;
		break;
	}

	/* Protocol layer does not provide command type, but our hardware
	 * needs it!
	 * any data transfer means adtc type (but that information is not
	 * in command structure, so we flagged it into host struct.)
	 * However, telling bc, bcr and ac apart based on response is
	 * not foolproof:
	 * CMD0  = bc  = resp0  CMD15 = ac  = resp0
	 * CMD2  = bcr = resp2  CMD10 = ac  = resp2
	 *
	 * Resolve to best guess with some exception testing:
	 * resp0 -> bc, except CMD15 = ac
	 * rest are ac, except if opendrain
	 */

	if (host->datadir) {
		cmdtype = DAVINCI_MMC_CMDTYPE_ADTC;
	} else if (resptype == 0 && cmd->opcode != 15) {
		cmdtype = DAVINCI_MMC_CMDTYPE_BC;
	} else if (host->bus_mode == MMC_BUSMODE_OPENDRAIN) {
		cmdtype = DAVINCI_MMC_CMDTYPE_BCR;
	} else {
		cmdtype = DAVINCI_MMC_CMDTYPE_AC;
	}
	/*Set command Busy or not */
	if (cmd->flags & MMC_RSP_BUSY) {
		/*Linux core sending BUSY which is not defined for cmd 24 as per mmc standard */
		if (cmd->opcode != 24) {

			cmdreg = cmdreg | (1 << 8);
		}
	}

	/*Set command index */
	cmdreg |= cmd->opcode;

	/*Setting initialize clock */
	if (cmd->opcode == 0) {
		cmdreg = cmdreg | (1 << 14);
	}

	/*Set for generating DMA Xfer event */
	if ((host->use_dma == 1) && (host->data != NULL)
	    && ((cmd->opcode == 18) || (cmd->opcode == 25)
		|| (cmd->opcode == 24) || (cmd->opcode == 17))) {
		cmdreg = cmdreg | (1 << 16);
	}

	/*Setting whether command involves data transfer or not */
	if (cmdtype == DAVINCI_MMC_CMDTYPE_ADTC) {
		cmdreg = cmdreg | (1 << 13);
	}

	/*Setting whether stream or block transfer */
	if (cmd->flags & MMC_DATA_STREAM) {
		cmdreg = cmdreg | (1 << 12);
	}

	/*Setting whether data read or write */
	if (host->datadir == DAVINCI_MMC_DATADIR_WRITE) {
		cmdreg = cmdreg | (1 << 11);
	}

	/*Setting response type */
	cmdreg = cmdreg | (resptype << 9);

	if (host->bus_mode == MMC_BUSMODE_PUSHPULL) {
		cmdreg = cmdreg | (1 << 7);
	}

	/*set Command timeout */
	mmcsdregs->mmctor = 0xFFFF;

	/*Enable interrupt */
	if (host->datadir == DAVINCI_MMC_DATADIR_WRITE) {
		if (host->use_dma != 1) {
			mmcsdregs->mmcim = (MMCSD_EVENT_EOFCMD |
					    MMCSD_EVENT_WRITE |
					    MMCSD_EVENT_ERROR_CMDCRC |
					    MMCSD_EVENT_ERROR_DATACRC |
					    MMCSD_EVENT_ERROR_CMDTIMEOUT |
					    MMCSD_EVENT_ERROR_DATATIMEOUT |
					    MMCSD_EVENT_BLOCK_XFERRED);
		} else {
			mmcsdregs->mmcim = (MMCSD_EVENT_EOFCMD |
					    MMCSD_EVENT_ERROR_CMDCRC |
					    MMCSD_EVENT_ERROR_DATACRC |
					    MMCSD_EVENT_ERROR_CMDTIMEOUT |
					    MMCSD_EVENT_ERROR_DATATIMEOUT |
					    MMCSD_EVENT_BLOCK_XFERRED);
		}
	} else if (host->datadir == DAVINCI_MMC_DATADIR_READ) {
		if (host->use_dma != 1) {
			mmcsdregs->mmcim = (MMCSD_EVENT_EOFCMD |
					    MMCSD_EVENT_READ |
					    MMCSD_EVENT_ERROR_CMDCRC |
					    MMCSD_EVENT_ERROR_DATACRC |
					    MMCSD_EVENT_ERROR_CMDTIMEOUT |
					    MMCSD_EVENT_ERROR_DATATIMEOUT |
					    MMCSD_EVENT_BLOCK_XFERRED);
		} else {
			mmcsdregs->mmcim = (MMCSD_EVENT_EOFCMD |
					    MMCSD_EVENT_ERROR_CMDCRC |
					    MMCSD_EVENT_ERROR_DATACRC |
					    MMCSD_EVENT_ERROR_CMDTIMEOUT |
					    MMCSD_EVENT_ERROR_DATATIMEOUT |
					    MMCSD_EVENT_BLOCK_XFERRED);
		}

	} else {
		mmcsdregs->mmcim = (MMCSD_EVENT_EOFCMD |
				    MMCSD_EVENT_ERROR_CMDCRC |
				    MMCSD_EVENT_ERROR_DATACRC |
				    MMCSD_EVENT_ERROR_CMDTIMEOUT |
				    MMCSD_EVENT_ERROR_DATATIMEOUT);

	}

	/* It is required by controoler b4 WRITE command that FIFO should be populated with 32 bytes */
	if ((host->datadir == DAVINCI_MMC_DATADIR_WRITE) &&
	    (cmdtype == DAVINCI_MMC_CMDTYPE_ADTC) && (host->use_dma != 1)) {
		byteCnt = mmcsdcfg.rwThreshold;
		host->bytesleft -= mmcsdcfg.rwThreshold;
		for (i = 0; i < (byteCnt / 4); i++) {
			mmcsdregs->mmcdxr = *host->buffer;
			host->buffer++;
		}
	}

	if (cmd->opcode == 7) {
#ifdef MMC_CARD_INS_REM_SUPPORT
		spin_lock_irqsave(&mmc_lock, flags);
#endif
		new_cardstate = 1;
		iscardinitialized = 1;
#ifdef MMC_CARD_INS_REM_SUPPORT
		host->old_cardstate = new_cardstate;
		is_init_progress = 0;
		spin_unlock_irqrestore(&mmc_lock, flags);
#endif
	}
#ifdef MMC_CARD_INS_REM_SUPPORT
	if (cmd->opcode == 1) {
		spin_lock_irqsave(&mmc_lock, flags);
		is_init_progress = 1;
		spin_unlock_irqrestore(&mmc_lock, flags);
	}
#endif

#ifdef MMC_CARD_INS_REM_SUPPORT
	host->iscore_command = 1;
#endif
	mmcsdregs->mmcarghl = cmd->arg;
	mmcsdregs->mmccmd = cmdreg;

}

static void mmc_davinci_dma_cb(int lch, u16 ch_status, void *data)
{
	int sync_dev = 0;
	struct mmc_davinci_host *host = (struct mmc_davinci_host *)data;

	if (DMA_COMPLETE == ch_status) {

		if (host->cmd == NULL && host->data == NULL) {
			if (host->datadir == DAVINCI_MMC_DATADIR_READ) {
				sync_dev = DAVINCI_DMA_MMCTXEVT;	/*Write */
			} else {
				sync_dev = DAVINCI_DMA_MMCRXEVT;	/*Read */
			}
#ifdef MMC_DEBUG
			printk
			    ("MMC- interrupt from DMA when no request has been made\n");
#endif
			davinci_stop_dma(sync_dev);
			return;
		}

		if (host->datadir == DAVINCI_MMC_DATADIR_READ) {
			sync_dev = DAVINCI_DMA_MMCTXEVT;	/*Write */
		} else {
			sync_dev = DAVINCI_DMA_MMCRXEVT;	/*Read */
		}
		davinci_stop_dma(sync_dev);
	} else {
#ifdef MMC_DEBUG
		/* Handing of Event missed interreupt from DMA */
		printk
		    ("MMC : Event miss interrupt has been generated by DMA\n");
#endif
		if (host->datadir == DAVINCI_MMC_DATADIR_READ) {
			sync_dev = DAVINCI_DMA_MMCTXEVT;	/*Write */
		} else {
			sync_dev = DAVINCI_DMA_MMCRXEVT;	/*Read */
		}
		davinci_clean_channel(sync_dev);
	}
}

static int mmc_davinci_start_dma_transfer(struct mmc_davinci_host *host,
					  struct mmc_request *req)
{
	const char *dev_name;
	int sync_dev, r, edmach = 0, tcc = 0;
	unsigned char i, j;
	unsigned short acnt, bcnt, ccnt;
	unsigned int src_port, dst_port, tempcCnt;
	enum address_mode mode_src, mode_dst;
	enum fifo_width fifowidthsrc, fifowidthdst;
	unsigned short srcbidx, dstbidx;
	unsigned short srccidx, dstcidx;
	unsigned short bcntrld;
	enum sync_dimension sync_mode;
	edmacc_paramentry_regs temp;
	enum dma_event_q queueno = EVENTQ_0;
	int edmachan_num;
	unsigned int num_eight_words = (req->data->blocks * 512) / 32;
	static unsigned int option_read = 0;
	static unsigned int option_write = 0;
	static unsigned char dma_read_req = 1;
	static unsigned char dma_write_req = 1;

#define MAX_C_CNT		64000

	if ((req->data->flags & MMC_DATA_WRITE)) {
		sync_dev = DAVINCI_DMA_MMCTXEVT;	/*Write */
		dev_name = "MMC_WRITE";

		if (dma_write_req) {
			r = davinci_request_dma(sync_dev, dev_name,
						mmc_davinci_dma_cb, host,
						&edmach, &tcc, queueno);
			if (r != 0) {
				printk
				    ("MMC: davinci_request_dma() failed with %d\n",
				     r);
				return r;
			}
			dma_write_req = 0;
		}
	} else {
		sync_dev = DAVINCI_DMA_MMCRXEVT;	/*Read */
		dev_name = "MMC_READ";
		if (dma_read_req) {
			r = davinci_request_dma(sync_dev, dev_name,
						mmc_davinci_dma_cb, host,
						&edmach, &tcc, queueno);
			if (r != 0) {
				printk
				    ("MMC: davinci_request_dma() failed with %d\n",
				     r);
				return r;
			}
			dma_read_req = 0;
		}
	}

	if ((req->data->flags & MMC_DATA_WRITE)) {
		/*AB Sync Transfer */
		/* Acnt =32, Bcnt= , Cnt=1 */

		sync_dev = DAVINCI_DMA_MMCTXEVT;	/*Write */
		acnt = 4;
		bcnt = 8;
		if (num_eight_words > MAX_C_CNT) {
			tempcCnt = MAX_C_CNT;
			ccnt = tempcCnt;
		} else {
			ccnt = num_eight_words;
			tempcCnt = ccnt;
		}

		src_port = (unsigned int)virt_to_phys(req->data->req->buffer);
		mode_src = INCR;
		fifowidthsrc = W8BIT;	/* It's not cared as modeDsr is INCR */
		srcbidx = 4;
		srccidx = 32;
		dst_port = MMCSD_REGS_BASE_ADDR + 0x2C;
		mode_dst = INCR;
		fifowidthdst = W8BIT;	/* It's not cared as modeDsr is INCR */
		dstbidx = 0;
		dstcidx = 0;
		bcntrld = 8;
		sync_mode = ABSYNC;

	} else {
		sync_dev = DAVINCI_DMA_MMCRXEVT;	/*Read */
		acnt = 4;
		bcnt = 8;
		if (num_eight_words > MAX_C_CNT) {
			tempcCnt = MAX_C_CNT;
			ccnt = tempcCnt;
		} else {
			ccnt = num_eight_words;
			tempcCnt = ccnt;
		}

		src_port = MMCSD_REGS_BASE_ADDR + 0x28;
		mode_src = INCR;
		fifowidthsrc = W8BIT;
		srcbidx = 0;
		srccidx = 0;
		dst_port = (unsigned int)virt_to_phys(req->data->req->buffer);
		mode_dst = INCR;
		fifowidthdst = W8BIT;	/* It's not cared as modeDsr is INCR */
		dstbidx = 4;
		dstcidx = 32;
		bcntrld = 8;
		sync_mode = ABSYNC;
	}

	davinci_set_dma_src_params(sync_dev, src_port, mode_src, fifowidthsrc);
	davinci_set_dma_dest_params(sync_dev, dst_port, mode_dst, fifowidthdst);
	davinci_set_dma_src_index(sync_dev, srcbidx, srccidx);
	davinci_set_dma_dest_index(sync_dev, dstbidx, dstcidx);
	davinci_set_dma_transfer_params(sync_dev, acnt, bcnt, ccnt, bcntrld,
					sync_mode);

	host->edma_ch_details.cnt_chanel = 0;
	davinci_get_dma_params(sync_dev, &temp);
	if (sync_dev == DAVINCI_DMA_MMCTXEVT) {
		if (option_write == 0) {
			option_write = temp.opt;
		} else {
			temp.opt = option_write;
			davinci_set_dma_params(sync_dev, &temp);
		}
	}
	if (sync_dev == DAVINCI_DMA_MMCRXEVT) {
		if (option_read == 0) {
			option_read = temp.opt;
		} else {
			temp.opt = option_read;
			davinci_set_dma_params(sync_dev, &temp);
		}
	}

	if (num_eight_words > MAX_C_CNT) {	/* Linking will be performed */
		davinci_get_dma_params(sync_dev, &temp);
		temp.opt &= ~TCINTEN;
		davinci_set_dma_params(sync_dev, &temp);

		for (i = 0; i < EDMA_MAX_LOGICAL_CHA_ALLOWED; i++) {
			if (i != 0) {
				j = i - 1;
				davinci_get_dma_params(host->edma_ch_details.
						       chanel_num[j], &temp);
				temp.opt &= ~TCINTEN;
				davinci_set_dma_params(host->edma_ch_details.
						       chanel_num[j], &temp);
			}

			host->edma_ch_details.cnt_chanel++;
			davinci_request_dma(DAVINCI_EDMA_PARAM_ANY, "LINK",
					    NULL, NULL, &edmachan_num,
					    &sync_dev, queueno);
			host->edma_ch_details.chanel_num[i] = edmachan_num;
			ccnt = temp.ccnt & 0x0000FFFF;
			if (sync_dev == DAVINCI_DMA_MMCTXEVT) {
				temp.src = temp.src + (acnt * bcnt * ccnt);
			} else {
				temp.dst = temp.dst + (acnt * bcnt * ccnt);
			}
			temp.opt |= TCINTEN;

			if ((num_eight_words - tempcCnt) > MAX_C_CNT) {
				temp.ccnt =
				    (temp.ccnt & 0xFFFF0000) | MAX_C_CNT;
				ccnt = temp.ccnt & 0x0000FFFF;
				tempcCnt = tempcCnt + ccnt;
			} else {
				temp.ccnt =
				    (temp.
				     ccnt & 0xFFFF0000) | (num_eight_words -
							   tempcCnt);
				ccnt = temp.ccnt & 0x0000FFFF;
				tempcCnt = tempcCnt + ccnt;
			}
			davinci_set_dma_params(edmachan_num, &temp);
			if (i != 0) {
				j = i - 1;
				davinci_dma_link_lch(host->edma_ch_details.
						     chanel_num[j],
						     edmachan_num);
			}
			if (tempcCnt == num_eight_words) {
				break;
			}

		}
		davinci_dma_link_lch(sync_dev,
				     host->edma_ch_details.chanel_num[0]);
	}
#if	0
	davinci_get_dma_params(sync_dev, &temp);
	printk("temp.options=%x, Channel no=%x\n", temp.opt, sync_dev);
	printk("temp.srcAddr=%x\n", temp.src);
	printk("temp.aCntbCnt=%x\n", temp.a_b_cnt);
	printk("temp.dstAddr=%x\n", temp.dst);
	printk("temp.srcDstBidx=%x\n", temp.src_dst_bidx);
	printk("temp.linkBcntrld=%x\n", temp.link_bcntrld);
	printk("temp.srcDstCidx=%x\n", temp.src_dst_cidx);
	printk("temp.ccnt=%x\n", temp.ccnt);
#endif

	davinci_start_dma(sync_dev);
	return 0;
}

static void mmc_davinci_prepare_data(struct mmc_davinci_host *host,
				     struct mmc_request *req)
{
	int timeout;
	host->data = req->data;
	if (req->data == NULL) {
		host->datadir = DAVINCI_MMC_DATADIR_NONE;
		mmcsdregs->mmcblen = 0;
		mmcsdregs->mmcnblk = 0;
		return;
	}
#ifdef MMC_DEBUG
	printk
	    ("MMCSD : Data xfer (%s %s), DTO %d cycles + %d ns, %d blocks of %d bytes\r\n",
	     (req->data->flags & MMC_DATA_STREAM) ? "stream" : "block",
	     (req->data->flags & MMC_DATA_WRITE) ? "write" : "read",
	     req->data->timeout_clks, req->data->timeout_ns, req->data->blocks,
	     1 << req->data->blksz_bits);
#endif

	/* Convert ns to clock cycles by assuming 20MHz frequency
	 * 1 cycle at 20MHz = 500 ns
	 */
	timeout = req->data->timeout_clks + req->data->timeout_ns / 500;
	if (timeout > 0xffff) {
		timeout = 0xffff;
	}

	mmcsdregs->mmctod = timeout;
	mmcsdregs->mmcnblk = req->data->blocks;
	mmcsdregs->mmcblen = (1 << req->data->blksz_bits);
	host->datadir = (req->data->flags & MMC_DATA_WRITE) ?
	    DAVINCI_MMC_DATADIR_WRITE : DAVINCI_MMC_DATADIR_READ;

	/*Configure the FIFO */
	switch (host->datadir) {
	case DAVINCI_MMC_DATADIR_WRITE:
		mmcsdregs->mmcfifoctl = mmcsdregs->mmcfifoctl | 0x1;
		mmcsdregs->mmcfifoctl = 0x0;
		mmcsdregs->mmcfifoctl = mmcsdregs->mmcfifoctl | (1 << 1);
		mmcsdregs->mmcfifoctl = mmcsdregs->mmcfifoctl | (1 << 2);
		break;

	case DAVINCI_MMC_DATADIR_READ:
		mmcsdregs->mmcfifoctl = mmcsdregs->mmcfifoctl | 0x1;
		mmcsdregs->mmcfifoctl = 0x0;
		mmcsdregs->mmcfifoctl = mmcsdregs->mmcfifoctl | (1 << 2);
		break;
	default:
		break;
	}

	if ((host->use_dma == 1)
	    && (mmc_davinci_start_dma_transfer(host, req) == 0)) {
		host->buffer = NULL;
		host->bytesleft = 0;
	} else {
		/*Revert to CPU Copy */
		host->buffer = (u32 *) (req->data->req->buffer);
		host->bytesleft =
		    req->data->blocks * (1 << req->data->blksz_bits);
		host->use_dma = 0;
	}
}

static void mmc_davinci_request(struct mmc_host *mmc, struct mmc_request *req)
{
	struct mmc_davinci_host *host = mmc_priv(mmc);
#ifdef MMC_CARD_INS_REM_SUPPORT
	unsigned long flags;
	if (!iscard_detect_progress) {
		spin_lock_irqsave(&mmc_lock, flags);
		iscardbusy = 1;
		spin_unlock_irqrestore(&mmc_lock, flags);
#endif
		mmc_davinci_prepare_data(host, req);
		mmc_davinci_start_command(host, req->cmd);
#ifdef MMC_CARD_INS_REM_SUPPORT
	} else {
		/*Queu up the request as card dectection is being excuted currently */
		que_mmc_host = mmc;
		que_mmc_request = req;
		spin_lock_irqsave(&mmc_lock, flags);
		is_req_queued_up = 1;
		spin_unlock_irqrestore(&mmc_lock, flags);
	}
#endif

}

static unsigned int calculate_freq_for_card(unsigned int mmc_req_freq)
{
	unsigned int mmcfreq = 0, cpu_arm_clk = 0, mmc_pushpull = 0;
	cpu_arm_clk = mmc_input_clk;

	if (cpu_arm_clk > (2 * mmc_req_freq)) {
		mmc_pushpull =
		    ((unsigned int)cpu_arm_clk / (2 * mmc_req_freq)) - 1;
	} else {
		mmc_pushpull = 0;
	}

	mmcfreq = (unsigned int)cpu_arm_clk / (2 * (mmc_pushpull + 1));

	if (mmcfreq > mmc_req_freq) {
		mmc_pushpull = mmc_pushpull + 1;
	}

	return mmc_pushpull;
}

static void mmc_davinci_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	unsigned short status;
	unsigned int opendrain_freq = 0, cpu_arm_clk = 0, mmcpushpull_freq = 0;
	struct mmc_davinci_host *host = mmc_priv(mmc);

	cpu_arm_clk = mmc_input_clk;
#ifdef MMC_DEBUG
	printk(" MMCSD: clock %dHz busmode %d powermode %d Vdd %d.%02d\r\n",
	       ios->clock, ios->bus_mode, ios->power_mode,
	       ios->vdd / 100, ios->vdd % 100);
#endif

	if (ios->bus_mode == MMC_BUSMODE_OPENDRAIN) {

		opendrain_freq =
		    ((unsigned int)cpu_arm_clk / (2 * MMCSD_INIT_CLOCK)) - 1;
		mmcsdregs->mmcclk =
		    (mmcsdregs->mmcclk & ~(0xFF)) | opendrain_freq;

	} else {
		mmcpushpull_freq = calculate_freq_for_card(ios->clock);
		mmcsdregs->mmcclk =
		    (mmcsdregs->mmcclk & ~(0xFF)) | mmcpushpull_freq;
	}
	host->bus_mode = ios->bus_mode;
	if (ios->power_mode == MMC_POWER_UP) {
		/* Send clock cycles, poll completion */
		mmcsdregs->mmcarghl = 0x0;
		mmcsdregs->mmccmd = 0x4000;
		status = 0;
		while (!(status & (MMCSD_EVENT_EOFCMD))) {
			status = mmcsdregs->mmcst0;
		}
	}
}

static void mmc_davinci_xfer_done(struct mmc_davinci_host *host,
				  struct mmc_data *data)
{
#ifdef MMC_CARD_INS_REM_SUPPORT
	unsigned long flags;
#endif
	host->data = NULL;
	host->datadir = DAVINCI_MMC_DATADIR_NONE;
	if (data->error == MMC_ERR_NONE)
		data->bytes_xfered += data->blocks * (1 << data->blksz_bits);

	if (data->error == MMC_ERR_TIMEOUT) {
#ifdef MMC_CARD_INS_REM_SUPPORT
		spin_lock_irqsave(&mmc_lock, flags);
		iscardbusy = 0;
		spin_unlock_irqrestore(&mmc_lock, flags);
#endif
		mmc_request_done(host->mmc, data->mrq);
		return;
	}

	if (!data->stop) {
		host->req = NULL;
#ifdef MMC_CARD_INS_REM_SUPPORT
		spin_lock_irqsave(&mmc_lock, flags);
		iscardbusy = 0;
		spin_unlock_irqrestore(&mmc_lock, flags);
#endif
		mmc_request_done(host->mmc, data->mrq);
		return;
	}
	mmc_davinci_start_command(host, data->stop);
}

static void mmc_davinci_cmd_done(struct mmc_davinci_host *host,
				 struct mmc_command *cmd)
{
#ifdef MMC_CARD_INS_REM_SUPPORT
	unsigned long flags;
#endif
	host->cmd = NULL;
	switch (cmd->flags & MMC_RSP_MASK) {
	case MMC_RSP_NONE:
		/* resp 0 */
		break;

	case MMC_RSP_SHORT:
		/* response types 1, 1b, 3, 4, 5, 6 */
		cmd->resp[0] = mmcsdregs->mmcrsp67;
		break;

	case MMC_RSP_LONG:
		/* response type 2 */
		cmd->resp[3] = mmcsdregs->mmcrsp01;
		cmd->resp[2] = mmcsdregs->mmcrsp23;
		cmd->resp[1] = mmcsdregs->mmcrsp45;
		cmd->resp[0] = mmcsdregs->mmcrsp67;
		break;
	}

	if (host->data == NULL || cmd->error != MMC_ERR_NONE) {
		host->req = NULL;
		if (cmd->error == MMC_ERR_TIMEOUT) {
			cmd->mrq->cmd->retries = 0;
		}
#ifdef MMC_CARD_INS_REM_SUPPORT
		spin_lock_irqsave(&mmc_lock, flags);
		iscardbusy = 0;
		spin_unlock_irqrestore(&mmc_lock, flags);
#endif
		mmc_request_done(host->mmc, cmd->mrq);
	}

}

static irqreturn_t mmc_davinci_irq(int irq, void *dev_id, struct pt_regs *regs)
{
	struct mmc_davinci_host *host = (struct mmc_davinci_host *)dev_id;
	u16 status;
	int end_command;
	int end_transfer;
	int byteCnt = 0, i = 0;
#ifdef MMC_CARD_INS_REM_SUPPORT
	unsigned long flags;
#endif

#ifdef MMC_CARD_INS_REM_SUPPORT
	if (host->iscore_command) {
#endif
		if (host->cmd == NULL && host->data == NULL) {
			status = mmcsdregs->mmcst0;
#ifdef MMC_DEBUG
			printk(KERN_INFO "MMC: Spurious interrupt 0x%04x\r\n",
			       status);
#endif
			/*Disable the interrupt from mmcsd */
			mmcsdregs->mmcim = 0;
			return IRQ_HANDLED;
		}
#ifdef MMC_CARD_INS_REM_SUPPORT
	}
#endif
	end_command = 0;
	end_transfer = 0;

	status = mmcsdregs->mmcst0;
	if (status == 0) {
		return IRQ_HANDLED;
	}
#ifdef MMC_CARD_INS_REM_SUPPORT
	if (host->iscore_command) {
#endif
		if (iscardinitialized) {
			if (new_cardstate == 0) {
				if (host->cmd) {
					host->cmd->error |= MMC_ERR_TIMEOUT;
					mmc_davinci_cmd_done(host, host->cmd);
				}
#ifdef MMC_DEBUG
				printk
				    ("MMC :print from code segment excuted when card removed\n");
#endif
				return IRQ_HANDLED;
			}
		}

		while (status != 0) {
			if (host->datadir == DAVINCI_MMC_DATADIR_WRITE) {
				if (status & MMCSD_EVENT_WRITE) {
					/* Buffer almost empty */
					if (host->bytesleft > 0) {
						byteCnt = mmcsdcfg.rwThreshold;
						host->bytesleft -=
						    mmcsdcfg.rwThreshold;
						for (i = 0; i < (byteCnt / 4);
						     i++) {
							mmcsdregs->mmcdxr =
							    *host->buffer;
							host->buffer++;
						}
					}
				}
			}

			if (host->datadir == DAVINCI_MMC_DATADIR_READ) {
				if (status & MMCSD_EVENT_READ) {
					/* Buffer almost empty */
					if (host->bytesleft > 0) {
						byteCnt = mmcsdcfg.rwThreshold;
						host->bytesleft -=
						    mmcsdcfg.rwThreshold;
						for (i = 0; i < (byteCnt / 4);
						     i++) {
							*host->buffer =
							    mmcsdregs->mmcdrr;
							host->buffer++;
						}
					}
				}
			}

			if (status & MMCSD_EVENT_BLOCK_XFERRED) {
				/* Block sent/received */
				if (host->data != NULL) {
					end_transfer = 1;
				}
			}

			if (status & MMCSD_EVENT_ERROR_DATATIMEOUT) {
				/* Data timeout */
				if ((host->data) && (new_cardstate != 0)) {
					host->data->error |= MMC_ERR_TIMEOUT;
#ifdef MMC_CARD_INS_REM_SUPPORT
					spin_lock_irqsave(&mmc_lock, flags);
#endif
					new_cardstate = 0;
#ifdef MMC_CARD_INS_REM_SUPPORT
					iscardinitialized = 0;
					spin_unlock_irqrestore(&mmc_lock,
							       flags);
#endif
#ifdef MMC_DEBUG
					printk
					    ("MMCSD: Data timeout, CMD%d and status is %x\r\n",
					     host->cmd->opcode, status);
#endif
					end_transfer = 1;
					host->cmd->error |= MMC_ERR_TIMEOUT;
				}
#ifdef MMC_DEBUG
				printk
				    ("MMCSD: Data timeout, CMD%d and status is %x\r\n",
				     host->cmd->opcode, status);
#endif
			}

			if (status & MMCSD_EVENT_ERROR_DATACRC) {
				/* Data CRC error */
				if (host->data) {
					host->data->error |= MMC_ERR_BADCRC;
					printk(KERN_DEBUG
					       "MMCSD: Data CRC error, bytes left %d\r\n",
					       host->bytesleft);
					end_transfer = 1;
				} else {
					printk(KERN_DEBUG
					       "MMCSD: Data CRC error\r\n");
				}
			}

			if (status & MMCSD_EVENT_ERROR_CMDTIMEOUT) {
				/* Command timeout */
				if (host->cmd) {
					/* Timeouts are normal in case of MMC_SEND_STATUS */
					if (host->cmd->opcode !=
					    MMC_ALL_SEND_CID) {
#ifdef MMC_DEBUG
						printk
						    ("MMCSD: Command timeout, CMD%d and status is %x\r\n",
						     host->cmd->opcode, status);
#endif
#ifdef MMC_CARD_INS_REM_SUPPORT
						spin_lock_irqsave(&mmc_lock,
								  flags);
#endif
						new_cardstate = 0;
#ifdef MMC_CARD_INS_REM_SUPPORT
						iscardinitialized = 0;
						spin_unlock_irqrestore
						    (&mmc_lock, flags);
#endif

					}
					host->cmd->error |= MMC_ERR_TIMEOUT;
					end_command = 1;

				}
			}

			if (status & MMCSD_EVENT_ERROR_CMDCRC) {
				/* Command CRC error */
				printk(KERN_ERR "MMCSD: Command CRC error\r\n");
				if (host->cmd) {
					host->cmd->error |= MMC_ERR_BADCRC;
					end_command = 1;
				}
			}

			if (status & MMCSD_EVENT_EOFCMD) {
				/* End of command phase */
				end_command = 1;
			}

			if (host->data == NULL) {
				status = mmcsdregs->mmcst0;
				if (status != 0) {
#ifdef MMC_DEBUG
					printk
					    ("Status is %x at end of ISR when host->data is NULL",
					     status);
#endif
					status = 0;

				}
			} else {
				status = mmcsdregs->mmcst0;
			}
		}

		if (end_command) {
			mmc_davinci_cmd_done(host, host->cmd);
		}

		if (end_transfer) {
			mmc_davinci_xfer_done(host, host->data);
		}
#ifdef MMC_CARD_INS_REM_SUPPORT
	} else {
		if (host->cmdcode == 13) {
			if (status & MMCSD_EVENT_EOFCMD) {
				spin_lock_irqsave(&mmc_lock, flags);
				new_cardstate = 1;
				spin_unlock_irqrestore(&mmc_lock, flags);

			} else {
				spin_lock_irqsave(&mmc_lock, flags);
				new_cardstate = 0;
				iscardinitialized = 0;
				spin_unlock_irqrestore(&mmc_lock, flags);
			}

			spin_lock_irqsave(&mmc_lock, flags);
			iscard_detect_progress = 0;
			spin_unlock_irqrestore(&mmc_lock, flags);

			if (is_req_queued_up) {
				mmc_davinci_request(que_mmc_host,
						    que_mmc_request);
				spin_lock_irqsave(&mmc_lock, flags);
				is_req_queued_up = 0;
				spin_unlock_irqrestore(&mmc_lock, flags);
			}

		}

		if (host->cmdcode == 1) {
			if (status & MMCSD_EVENT_EOFCMD) {
				spin_lock_irqsave(&mmc_lock, flags);
				new_cardstate = 1;
				iscardinitialized = 0;
				spin_unlock_irqrestore(&mmc_lock, flags);
			} else {

				spin_lock_irqsave(&mmc_lock, flags);
				new_cardstate = 0;
				iscardinitialized = 0;
				spin_unlock_irqrestore(&mmc_lock, flags);
			}

			spin_lock_irqsave(&mmc_lock, flags);
			iscard_detect_progress = 0;
			spin_unlock_irqrestore(&mmc_lock, flags);

			if (is_req_queued_up) {
				mmc_davinci_request(que_mmc_host,
						    que_mmc_request);
				spin_lock_irqsave(&mmc_lock, flags);
				is_req_queued_up = 0;
				spin_unlock_irqrestore(&mmc_lock, flags);
			}

		}

		if (host->cmdcode == 0) {
			if (status & MMCSD_EVENT_EOFCMD) {
				host->iscore_command = 0;
				host->cmdcode = 1;
#ifdef MMC_DEBUG
				printk("MMC-Probing mmc with cmd1\n");
#endif
				/*Issue cmd1 */
				mmcsdregs->mmcarghl = 0x80300000;
				mmcsdregs->mmccmd = 0x00000601;

			} else {
				spin_lock_irqsave(&mmc_lock, flags);
				new_cardstate = 0;
				iscardinitialized = 0;
				iscard_detect_progress = 0;
				spin_unlock_irqrestore(&mmc_lock, flags);
			}
		}

	}
#endif
	return IRQ_HANDLED;
}

static struct mmc_host_ops mmc_davinci_ops = {
	.request = mmc_davinci_request,
	.set_ios = mmc_davinci_set_ios,
};

#ifdef MMC_CARD_INS_REM_SUPPORT
void mmc_check_card(unsigned long data)
{
	struct mmc_davinci_host *host = (struct mmc_davinci_host *)data;
	unsigned long flags;
	if ((!iscard_detect_progress) || (!is_init_progress)) {
		if (iscardinitialized) {
			host->iscore_command = 0;
			host->cmdcode = 13;
			spin_lock_irqsave(&mmc_lock, flags);
			iscard_detect_progress = 1;
			spin_unlock_irqrestore(&mmc_lock, flags);
			/*Issue cmd13 */
			mmcsdregs->mmcarghl = 0x10000;
			mmcsdregs->mmccmd = 0x0000028D;
		} else {
			host->iscore_command = 0;
			host->cmdcode = 0;
			spin_lock_irqsave(&mmc_lock, flags);
			iscard_detect_progress = 1;
			spin_unlock_irqrestore(&mmc_lock, flags);
			/*Issue cmd0 */
			mmcsdregs->mmcarghl = 0;
			mmcsdregs->mmccmd = 0x4000;
		}
		mmcsdregs->mmcim = (MMCSD_EVENT_EOFCMD |
				    MMCSD_EVENT_ERROR_CMDCRC |
				    MMCSD_EVENT_ERROR_DATACRC |
				    MMCSD_EVENT_ERROR_CMDTIMEOUT |
				    MMCSD_EVENT_ERROR_DATATIMEOUT);

	}
}
static void davinci_mmc_check_status(unsigned long data)
{
	unsigned long flags;
	struct mmc_davinci_host *host = (struct mmc_davinci_host *)data;
	if (!iscardbusy) {
		if (host->old_cardstate ^ new_cardstate) {
			mmc_detect_change(host->mmc);
			spin_lock_irqsave(&mmc_lock, flags);
			host->old_cardstate = new_cardstate;
			spin_unlock_irqrestore(&mmc_lock, flags);
		} else {
			mmc_check_card(data);
		}

	}
	mod_timer(&host->timer, jiffies + MULTIPILER_TO_HZ * HZ);
}
#endif

static void initMmcsdHost(void)
{
	mmcsdregs->mmcctl = mmcsdregs->mmcctl | 0x1;	/*CMD line portion is diabled and in reset state */
	mmcsdregs->mmcctl = mmcsdregs->mmcctl | (1 << 1);	/*DAT line portion is diabled and in reset state */

	mmcsdregs->mmcclk = 0x0;
	mmcsdregs->mmcclk = mmcsdregs->mmcclk | (1 << 8);

	mmcsdregs->mmctor = 0xFFFF;
	mmcsdregs->mmctod = 0xFFFF;

	mmcsdregs->mmcctl = mmcsdregs->mmcctl & ~(0x1);
	mmcsdregs->mmcctl = mmcsdregs->mmcctl & ~(1 << 1);
}

static int davinci_mmcsd_probe(struct device *dev)
{
	struct mmc_davinci_host *host;
	struct mmc_host *mmc;
	int ret;

	mmc = mmc_alloc_host(sizeof(struct mmc_davinci_host), dev);
	if (!mmc) {
		ret = -ENOMEM;
		goto out;
	}

	mmcsdregs = (mmcsd_regs *) IO_ADDRESS(MMCSD_REGS_BASE_ADDR);
	initMmcsdHost();

	mmc->ops = &mmc_davinci_ops;
	mmc->f_min = 312500;
	mmc->f_max = 20000000;
	mmc->ocr_avail = MMC_VDD_32_33;

	host = mmc_priv(mmc);
	host->mmc = mmc;	/*Important */

	host->use_dma = mmcsdcfg.use_dma;
	host->irq = MMCINT_INTERRUPT;
	host->sd_support = 1;
	ret =
	    request_irq(MMCINT_INTERRUPT, mmc_davinci_irq, 0, DRIVER_NAME,
			host);

	if (ret)
		goto out;

	dev_set_drvdata(dev, host);
	mmc_add_host(mmc);
#ifdef MMC_CARD_INS_REM_SUPPORT

	init_timer(&host->timer);
	host->timer.data = (unsigned long)host;
	host->timer.function = davinci_mmc_check_status;
	host->timer.expires = jiffies + MULTIPILER_TO_HZ * HZ;
	add_timer(&host->timer);

#endif

	return 0;

      out:
	/* TBD: Free other resources too. */

	return ret;
}

static int davinci_mmcsd_remove(struct device *dev)
{
	struct mmc_davinci_host *host = dev_get_drvdata(dev);

	dev_set_drvdata(dev, NULL);
	if (host) {
		mmc_remove_host(host->mmc);
		free_irq(host->irq, host);
	}
	davinci_free_dma(DAVINCI_DMA_MMCTXEVT);
	davinci_free_dma(DAVINCI_DMA_MMCRXEVT);
	return 0;

}

#ifdef CONFIG_PM
static int davinci_mmcsd_suspend(struct device *dev, u32 state, u32 level)
{
	struct mmc_host *mmc = dev_get_drvdata(dev);
	int ret = 0;

	if (mmc && level == SUSPEND_DISABLE)
		ret = mmc_suspend_host(mmc, state);

	return ret;
}

static int davinci_mmcsd_resume(struct device *dev, u32 level)
{
	struct mmc_host *mmc = dev_get_drvdata(dev);
	int ret = 0;

	if (mmc && level == RESUME_ENABLE)
		ret = mmc_resume_host(mmc);

	return ret;
}

#else

#define davinci_mmcsd_suspend	NULL
#define davinci_mmcsd_resume	NULL

#endif

static struct device_driver davinci_mmcsd_driver = {
	.name = DRIVER_NAME,
	.bus = &platform_bus_type,
	.probe = davinci_mmcsd_probe,
	.remove = davinci_mmcsd_remove,
	.suspend = davinci_mmcsd_suspend,
	.resume = davinci_mmcsd_resume,
};

static void mmc_release(struct device *dev)
{
	/* Nothing to release? */
}

static u64 mmc_dmamask = 0xffffffff;

static struct resource mmc_resources[] = {
	{
	 .start = IO_ADDRESS(MMCSD_REGS_BASE_ADDR),
	 .end = IO_ADDRESS((MMCSD_REGS_BASE_ADDR) + 0x74),
	 .flags = IORESOURCE_MEM,
	 },
	{
	 .start = MMCINT_INTERRUPT,
	 .flags = IORESOURCE_IRQ,
	 },
};

static struct platform_device mmc_davinci_device = {
	.name = DRIVER_NAME,
	.id = 1,
	.dev = {
		.release = mmc_release,
		.dma_mask = &mmc_dmamask,
		},
	.num_resources = ARRAY_SIZE(&mmc_resources),
	.resource = mmc_resources,
};

static int davinci_mmcsd_init(void)
{
	int ret = 0;
	struct clk *clkp = NULL;

	clkp = clk_get(NULL, "MMCSDCLK");
	if (clkp != NULL) {
		mmcclkp = clkp;
		clk_use(mmcclkp);
		clk_enable(mmcclkp);
		mmc_input_clk = clk_get_rate(mmcclkp);

		ret = platform_device_register(&mmc_davinci_device);
		if (ret != 0)
			goto free1;

		ret = driver_register(&davinci_mmcsd_driver);
		if (ret == 0)
			return 0;

	      free1:
		platform_device_unregister(&mmc_davinci_device);
	}

	return -ENODEV;
}

static void __exit davinci_mmcsd_exit(void)
{

	driver_unregister(&davinci_mmcsd_driver);
	platform_device_unregister(&mmc_davinci_device);
	clk_disable(mmcclkp);
	clk_unuse(mmcclkp);
}

module_init(davinci_mmcsd_init);
module_exit(davinci_mmcsd_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("MMCSD driver for Davinci MMC controller");
