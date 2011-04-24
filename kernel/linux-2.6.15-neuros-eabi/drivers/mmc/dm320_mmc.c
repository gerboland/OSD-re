/*
 *  it_mmcsd_clear_response_reg(&cmd);
 *  linux/drivers/mmc/mmci.c - ARM PrimeCell MMCI PL180/1 driver
 *
 *  Copyright (C) 2003 Deep Blue Solutions, Ltd, All Rights Reserved.
 *
 *  11/13, 2006 mgao@neuros  
 *  DMA response on READ side fixed. SD with blocksize > 512B support added in.
 *  Interrupt based response still screwy, but it seems working anyway.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/config.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/highmem.h>
#include <linux/platform_device.h>
#include <linux/mmc/host.h>
#include <linux/mmc/protocol.h>
#include <linux/mmc/machmmc.h>
#include <linux/mmc/card.h>

#include <asm/div64.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/scatterlist.h>
#include <asm/hardware/amba.h>
#include <linux/mmc/mmc.h>

#include <linux/interrupt.h>
#include <asm/arch/gios.h>
#include <asm/arch/gio.h>
#include <asm/arch/irqs.h>

#include "dm320_mmc.h"

MODULE_LICENSE("GPL");

#define ROUND2BLKSZ(rm, sz) (((rm) + (sz) - 1) / (sz))
#define DRIVER_NAME "SD-MMC"

// remembers the allocated data structure.
static struct mmc_host *mmc_hdl = NULL;
static int timer_inited = 0;

#ifdef FINE_TUNING
static int max_wr = 0;
static int max_rd = 0;
#endif

static unsigned int fmax = 515633;
static int bCardInitialized = 0;
static int dma_in_progress = 0;
static int command_in_progress = 0;
static spinlock_t  mmc_lock;
static DECLARE_COMPLETION(dma_complete);
static void init_sd_mmc(void);

static void change_endian(int big)
{
    short reg = 0; 
	
	/* reset the sd/mmc controller */
	reg = inw(IO_MMC_CONTROL);

	if (big)
	  {
		if (reg & 0x600) return;

		/* reset the sd/mmc controller */
		outw(0x03, IO_MMC_CONTROL);
		  
		/* change to little endian, and release reset */
		outw(reg | 0x600, IO_MMC_CONTROL);
	  }
	else
	  {
		if (!(reg & 0x600)) return;

		/* reset the sd/mmc controller */
		outw(0x03, IO_MMC_CONTROL);
		  
		/* change to little endian, and release reset */
		outw(reg &~ 0x600, IO_MMC_CONTROL);		
	  }
}

static void enable_interrupt(void)
{
    unsigned long flags;
    spin_lock_irqsave(&mmc_lock, flags);
    outw(0x01FF, IO_MMC_INT_ENABLE);
    outw((inw(IO_INTC_EINT1) | 0x18),IO_INTC_EINT1);
    spin_unlock_irqrestore(&mmc_lock, flags);
}

static void disable_interrupt(void)
{
    unsigned long flags;

    spin_lock_irqsave(&mmc_lock, flags);
    outw((inw(IO_INTC_EINT1) &~ 0x18),IO_INTC_EINT1);
    outw(0x0000, IO_MMC_INT_ENABLE);
    spin_unlock_irqrestore(&mmc_lock, flags);
}

static u16 mmc_mask_irq(void)
{
    u16 reg_val;
    reg_val = inw(IO_MMC_INT_ENABLE);
    outw(0x0000, IO_MMC_INT_ENABLE);
    return(reg_val & 0x0FFF);
}

static void mmc_umask_irq(u16 reg_val)
{
    outw(reg_val & 0x0FFF, IO_MMC_INT_ENABLE); 
}

static void it_mmcsd_get_status(struct mmc_command *cmd)
{
    u16 irq_mask;

    irq_mask = mmc_mask_irq();

    cmd->status0 = inw(IO_MMC_STATUS0);
    cmd->status1 = inw(IO_MMC_STATUS1);
    cmd->resp[0] = inw(IO_MMC_RESPONSE0);
    cmd->resp[1] = inw(IO_MMC_RESPONSE1);
    cmd->resp[2] = inw(IO_MMC_RESPONSE2);
    cmd->resp[3] = inw(IO_MMC_RESPONSE3);
    cmd->resp[4] = inw(IO_MMC_RESPONSE4);
    cmd->resp[5] = inw(IO_MMC_RESPONSE5);
    cmd->resp[6] = inw(IO_MMC_RESPONSE6);
    cmd->resp[7] = inw(IO_MMC_RESPONSE7);
    cmd->cmd_index = inw (IO_MMC_COMMAND_INDEX);

    mmc_umask_irq(irq_mask);
}

static void it_mmcsd_clear_response_reg(struct mmc_command *cmd){
    u16 irq_mask;

    irq_mask = mmc_mask_irq();

    outw(0,IO_MMC_RESPONSE0);
    outw(0,IO_MMC_RESPONSE1);
    outw(0,IO_MMC_RESPONSE2);
    outw(0,IO_MMC_RESPONSE3);
    outw(0,IO_MMC_RESPONSE4);
    outw(0,IO_MMC_RESPONSE5);
    outw(0,IO_MMC_RESPONSE6);
    outw(0,IO_MMC_RESPONSE7);
    outw(0,IO_MMC_COMMAND_INDEX);
    
    cmd->resp[0] = 0;
    cmd->resp[1] = 0;
    cmd->resp[2] = 0;
    cmd->resp[3] = 0;
    cmd->resp[4] = 0;
    cmd->resp[5] = 0;
    cmd->resp[6] = 0;
    cmd->resp[7] = 0;

    mmc_umask_irq(irq_mask);
}


#if  0
static void it_mmcsd_print_status(struct mmc_command *cmd){
    
    printk("Clk control : %x\n",inw(inw(IO_MMC_MEM_CLK_CONTROL)));
    printk("Control Reg : %x\n",inw(IO_MMC_CONTROL));
    printk("status0 : %x \n",cmd->status0);
    printk("status1 : %x \n",cmd->status1);
    printk("resp[0] : %x \n",cmd->resp[0]);
    printk("resp[1] : %x \n",cmd->resp[1]);
    printk("resp[2] : %x \n",cmd->resp[2]);
    printk("resp[3] : %x \n",cmd->resp[3]);
    printk("resp[4] : %x \n",cmd->resp[4]);
    printk("resp[5] : %x \n",cmd->resp[5]);
    printk("resp[6] : %x \n",cmd->resp[6]);
    printk("resp[7] : %x \n",cmd->resp[7]);
    printk("cmd_index : %x \n",cmd->cmd_index);
}
#else
static void it_mmcsd_print_status(struct mmc_command *cmd){

    printk("\n status0 : %x \t",cmd->status0);
    printk("resp[6] : %x\t",cmd->resp[6]);
    printk("resp[7] : %x\t",cmd->resp[7]);
    printk("cmd_index : %x\t",cmd->cmd_index);
    printk("\n");
}
#endif

// speed: 0: low clock speed.
//        1: high speed for SD only.
static void init_clocks(int speed)
{
    short reg = 0;
    u16 irq_mask;

    irq_mask = mmc_mask_irq();

    /* disable clock */
    reg = inw(IO_MMC_MEM_CLK_CONTROL);
    outw(reg & 0xFEFF, IO_MMC_MEM_CLK_CONTROL);    

    /* set clock */
    outw(inw(IO_CLK_INV) | 0x1,IO_CLK_INV);

    /* set function clock */
    outw(((inw(IO_CLK_DIV3) & 0xFF00) | (0x01)) ,IO_CLK_DIV3);
    
    udelay(500);

    /* set mmc clock to ~ 400 kHz */ /* 338 KHZ */
	if (speed)
	  {
		outw(0x00,IO_MMC_MEM_CLK_CONTROL);
	  }
	else 
	  {
		outw(0x40,IO_MMC_MEM_CLK_CONTROL);
	  }
    udelay(500);

    /* enable clock */
    reg = inw(IO_MMC_MEM_CLK_CONTROL);
    outw(reg | 0x0100, IO_MMC_MEM_CLK_CONTROL);
    reg = inw(IO_MMC_MEM_CLK_CONTROL);

    mmc_umask_irq(irq_mask);
}

static void change_clk25m(void)
{
    init_clocks(1);
}

static void mmci_request_end(struct mmci_host *host, struct mmc_request *mrq)
{
    host->mrq = NULL;
    host->cmd = NULL;
    if (mrq->data){
        mrq->data->bytes_xfered = host->data_xfered;
    }

    mmc_request_done(host->mmc, mrq);
}

#if (DEBUG_MMC_COMMAND == 1)
static void mmci_print_cmd(struct mmc_command *cmd)
{
    char* str;

    switch(cmd->opcode & 0x3F)
    {
        case MMC_GO_IDLE_STATE:
            str = "MMC_GO_IDLE_STATE";
            break;

        case MMC_SEND_OP_COND:
            str = "MMC_SEND_OP_COND";
            break;

        case MMC_ALL_SEND_CID:
            str = "MMC_ALL_SEND_CID";
            break;

        case MMC_SET_RELATIVE_ADDR:
            str = "MMC_SET_RELATIVE_ADDR";
            break;

        case MMC_SET_DSR:
            str = "MMC_SET_DSR";
            break;

        case MMC_SELECT_CARD:
            str = "MMC_SELECT_CARD";
            break;

        case MMC_SEND_CSD:
            str = "MMC_SEND_CSD";
            break;

        case MMC_SEND_CID:
            str = "MMC_SEND_CID";
            break;

        case MMC_READ_DAT_UNTIL_STOP:
            str = "MMC_READ_DAT_UNTIL_STOP";
            break;

        case MMC_STOP_TRANSMISSION:
            str = "MMC_STOP_TRANSMISSION";
            break;

        case MMC_SEND_STATUS:
            str = "MMC_SEND_STATUS";
            break;

        case MMC_GO_INACTIVE_STATE:
            str = "MMC_GO_INACTIVE_STATE";
            break;

        case MMC_SET_BLOCKLEN:
            str = "MMC_SET_BLOCKLEN";
            break;

        case MMC_READ_SINGLE_BLOCK:
            str = "MMC_READ_SINGLE_BLOCK";
            break;

        case MMC_READ_MULTIPLE_BLOCK:
            str = "MMC_READ_MULTIPLE_BLOCK";
            break;

        case MMC_WRITE_DAT_UNTIL_STOP:
            str = "MMC_WRITE_DAT_UNTIL_STOP";
            break;

        case MMC_SET_BLOCK_COUNT:
            str = "MMC_SET_BLOCK_COUNT";
            break;

        case MMC_WRITE_BLOCK:
            str = "MMC_WRITE_BLOCK";
            break;

        case MMC_WRITE_MULTIPLE_BLOCK:
            str = "MMC_WRITE_MULTIPLE_BLOCK";
            break;

        case MMC_PROGRAM_CID:
            str = "MMC_PROGRAM_CID";
            break;

        case MMC_PROGRAM_CSD:
            str = "MMC_PROGRAM_CSD";
            break;

        case MMC_SET_WRITE_PROT:
            str = "MMC_SET_WRITE_PROT";
            break;

        case MMC_CLR_WRITE_PROT:
            str = "MMC_CLR_WRITE_PROT";
            break;

        case MMC_SEND_WRITE_PROT:
            str = "MMC_SEND_WRITE_PROT";
            break;

        case MMC_ERASE_GROUP_START:
            str = "MMC_ERASE_GROUP_START";
            break;

        case MMC_ERASE_GROUP_END:
            str = "MMC_ERASE_GROUP_END";
            break;

        case MMC_ERASE:
            str = "MMC_ERASE";
            break;

        case MMC_FAST_IO:
            str = "MMC_FAST_IO";
            break;

        case MMC_GO_IRQ_STATE:
            str = "MMC_GO_IRQ_STATE";
            break;

        case MMC_LOCK_UNLOCK:
            str = "MMC_LOCK_UNLOCK";
            break;

        case MMC_APP_CMD:
            str = "MMC_APP_CMD";
            break;

        case MMC_GEN_CMD:
            str = "MMC_GEN_CMD";
            break;

//        case SD_SEND_RELATIVE_ADDR:
//            str = "SD_SEND_RELATIVE_ADDR";
//            break;

        case SD_APP_SET_BUS_WIDTH:
            str = "SD_APP_SET_BUS_WIDTH";
            break;

        case SD_APP_OP_COND:
            str = "SD_APP_OP_COND";
            break;

        case SD_APP_SEND_SCR:
            str = "SD_APP_SEND_SCR";
            break;

//        case SD_APP_DIS_PULL_UP:
//            str = "SD_APP_DIS_PULL_UP";
//            break;

        default:
            str = "unknown opcode";
            break;
    }

    printk("%s %x\n", str, cmd->arg);
}
#endif

static void mmci_start_command(struct mmci_host *host, struct mmc_command *cmd, u32 c)
{
    u32 marg;
    u16 cmd1;

    u16 irq_mask;
    irq_mask = mmc_mask_irq();

#if (DEBUG_MMC_COMMAND == 1)
    mmci_print_cmd(cmd);
#endif

    it_mmcsd_clear_response_reg(cmd);

    host->cmd = cmd;

    marg = cmd->arg; 
    cmd1 = cmd->opcode | cmd->flags;    

    outw(marg & 0xFFFF,IO_MMC_ARG_LOW);
    outw((marg >> 16),IO_MMC_ARG_HI);
    outw(cmd1, IO_MMC_COMMAND);

	command_in_progress = 1;

    bCardInitialized = 1;

    mmc_umask_irq(irq_mask);
}

/*
 * Handle completion of command and data transfers.
 */
static irqreturn_t dm320_irq(int irq, void* dev_id, struct pt_regs* regs)
{
    struct mmci_host *host = dev_id;
    struct mmc_command cmd_data;
    struct mmc_command* cmd;
    u16 irq_mask;

    irq_mask = mmc_mask_irq();
    if (host->cmd)
    {
        cmd = host->cmd;
        if((unsigned long)host->cmd == 0x6b6b6b6b)
        {
            printk("\n Its too early to get an interrupt \n");
            it_mmcsd_get_status(&cmd_data);
            it_mmcsd_print_status(&cmd_data);
            dump_stack();
            panic("Early Interrupt\n");
            goto done;
        }
    }
    else
    {
        cmd = &cmd_data;
    }

    it_mmcsd_get_status(cmd);

    if (cmd->status0 & MMC_INT_TIME_OUT_ERROR) {
        //printk("Card has not responded %X\n", cmd->status0 & MMC_INT_TIME_OUT_ERROR);
        cmd->error = MMC_ERR_TIMEOUT;
    } else if (cmd->status0 & MMC_INT_CRC_ERROR){
        printk("CRC ERROR %08x\n", cmd->status0 & MMC_INT_CRC_ERROR);
        cmd->error = MMC_ERR_BADCRC;
    }

    // Must read IO_MMC_SD_DMA_STATUS1 or interrupts stop.
    // Check both DMADNE & DATDNE because sometimes one
    // does not happen.  See newest errata for DMASZEN.
    if ((dma_in_progress) && 
        (cmd->status0 & (MMC_INT_DMA_DONE | MMC_INT_DATA_DONE)) && 
        ((inw(IO_MMC_SD_DMA_STATUS1) & MMC_DMA_STATUS_RUNNING) == 0))
    {
  	    struct mmc_command tmpcmd;
        dma_in_progress = 0;
        outw(0, IO_MMC_ARG_LOW);
        outw(0, IO_MMC_ARG_HI);
        outw(12 | 0x0200 | 0x0100 | 0x0080, IO_MMC_COMMAND);

		do
		{
		  it_mmcsd_get_status(&tmpcmd);    

		  if (cmd->status0 & MMC_INT_TIME_OUT_ERROR) 
			{
			  printk("INT-WARNING: request timed out!\n");
			  cmd->error = MMC_ERR_TIMEOUT;
			  break;
			}
		  
		  else if (cmd->status0 & MMC_INT_CRC_ERROR)
			{
			  printk("INT-CRC ERROR: request bailed.\n");
			  cmd->error = MMC_ERR_BADCRC;
			  break;
			}
		  
		}while( !((tmpcmd.status0 & 0x4)==4));

        complete(&dma_complete);
    }
	//printk("cmd->data = %p\n",cmd->data);
    if (host->cmd && !cmd->data && !(cmd->status1 & 0x1) && command_in_progress) 
    {
        command_in_progress = 0;
        mmci_request_end(host, cmd->mrq);
    } 

done:
    mmc_umask_irq(irq_mask);
    return IRQ_HANDLED;
}

static void mmci_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
    struct mmci_host *host = mmc_priv(mmc);
    u16 irq_mask;

    WARN_ON(host->mrq != NULL);

    irq_mask = mmc_mask_irq();

    host->mrq = mrq;

    mmci_start_command(host, mrq->cmd, 0);
#if 0 // can't do this, system will hickup.
	do
	  {
		it_mmcsd_get_status(host->cmd);    
		
		if (host->cmd->status0 & MMC_INT_TIME_OUT_ERROR) 
		  {
			printk("WARNING 1: request timed out!\n");
			host->cmd->error = MMC_ERR_TIMEOUT;
			goto bail;
		  }
		
		else if (host->cmd->status0 & MMC_INT_CRC_ERROR)
		  {
			printk("CRC ERROR 1: request bailed.\n");
			host->cmd->error = MMC_ERR_BADCRC;
			goto bail;
		  }
		//printk("status0 = %X status1 = %X\n", host->cmd->status0, host->cmd->status1);
	  }while( !((host->cmd->status0 & 0x4)==4));
	// wait till CMD is send

	while( (host->cmd->status1 & 0x1))
	  {
		it_mmcsd_get_status(host->cmd);    
		
		if (host->cmd->status0 & MMC_INT_TIME_OUT_ERROR) 
		  {
			printk("WARNING 2: request timed out!\n");
			host->cmd->error = MMC_ERR_TIMEOUT;
			goto bail;
		  }
		
		else if (host->cmd->status0 & MMC_INT_CRC_ERROR)
		  {
			printk("CRC ERROR 2: request bailed.\n");
			host->cmd->error = MMC_ERR_BADCRC;
			goto bail;
		  }
		//printk("status0 = %X status1 = %X\n", host->cmd->status0, host->cmd->status1);
	  }
	// wait till card is not busy.
 bail:
	mmci_request_end(host, host->cmd->mrq);
#endif

    mmc_umask_irq(irq_mask);
}

static void mmci_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
    volatile u16 reg = 0;
    u16 irq_mask;

    irq_mask = mmc_mask_irq();

    reg = inw (IO_MMC_CONTROL);
    if (mmc->ios.bus_width == MMC_BUS_WIDTH_4)
    {
        if ((reg & (1 << 2)) == 0)
        {
            /* reset the sd/mmc controller */
            outw(0x03, IO_MMC_CONTROL);

            /* change to 4-data line mode, release reset */
            outw(reg | (1 << 2), IO_MMC_CONTROL); 
        }
    }
    else
    {
        if ((reg & (1 << 2)) != 0)
        {
            /* reset the sd/mmc controller */
            outw(0x03, IO_MMC_CONTROL);

            /* change to 1-data line mode, release reset */
            outw(reg &~ (1 << 2), IO_MMC_CONTROL);
        }
    }

    mmc_umask_irq(irq_mask);
}


static int write_to_card(struct mmc_host *mmc,struct mmc_request *mrq)
{
    u32 current_add;
    u32 marg;
    int timeout;
    struct mmc_command cmd;
    struct mmci_host *host = mmc_priv(mmc);
    u16 irq_mask;
    u16 cmd1;
    u16 no_of_blks;
	//u16 blk_sz = 1<<mrq->data->blksz_bits;
    u16 blk_sz = 512;
    timeout = 0xffff;

    do {
        it_mmcsd_get_status(&cmd);
    } while (!(cmd.status0 & (MMC_INT_DAT_TRX_RDY | 
        MMC_INT_CRC_ERROR | MMC_INT_TIME_OUT_ERROR)) && 
        !(cmd.status0 & 0x1) &&
        --timeout);

    if (!(cmd.status0 & (MMC_INT_DAT_TRX_RDY | 
        MMC_INT_CRC_ERROR | MMC_INT_TIME_OUT_ERROR)) && 
        !(cmd.status0 & 0x1))
    {
	  printk("%s %s %d error timeout waiting for transmit to complete\n", 
			 __FILE__, __FUNCTION__, __LINE__); 
    }

    irq_mask = mmc_mask_irq();

    mrq->cmd->error = 0;
	mrq->cmd->mrq = mrq;
    host->cmd = mrq->cmd;

    if (mrq->data) 
    {
        mrq->cmd->data = mrq->data;
        mrq->data->error = 0;
        mrq->data->mrq = mrq;
        if (mrq->stop) 
        {
            mrq->data->stop = mrq->stop;
            mrq->stop->error = 0;
            mrq->stop->mrq = mrq;
        }
    }
    host->data = mrq->data;
    host->size = mrq->data->blocks << mrq->data->blksz_bits;
    host->data_xfered = 0;
    mmci_init_sg(host, mrq->data);    
    current_add = mrq->cmd->arg;
    no_of_blks = mrq->data->blocks;

	// change to little endian.
	change_endian(0);

    do{
        unsigned long flags;
        unsigned int remain, len = 0;
        short *buffer_short = NULL;
        static char *buffer = NULL;

#if 0 // seting block length.
	  outw(blk_sz & 0xFFFF,IO_MMC_ARG_LOW);
	  outw(0, IO_MMC_ARG_HI);
	  outw(16 | 0x0200, IO_MMC_COMMAND);

	  do
		{
		  it_mmcsd_get_status(&cmd);
		} while((!(cmd.status0&0x04))||(cmd.status1 & 0x01));
#endif
        /*
        * Map the current scatter buffer.
        */
        buffer = mmci_kmap_atomic(host, &flags) + host->sg_off;
        if (buffer == NULL)
            mmc_debug_msg ("NULL data Pointer\n");

        remain = host->sg_ptr->length - host->sg_off;

        mmc_debug_msg ("WRITE_REMAIN : %d\n",remain);

        outw(ROUND2BLKSZ(remain, blk_sz),IO_MMC_NR_BLOCKS);
		outw(blk_sz, IO_MMC_BLOCK_LENGTH);

        marg = current_add;
        outw(marg & 0xFFFF,IO_MMC_ARG_LOW);
        outw((marg >> 16),IO_MMC_ARG_HI);

        cmd1 = 25 | 0x2000 |0x0800 | 0x0200;

        buffer_short = (short*)buffer;

        outw(((0xDFFF) & inw (IO_MMC_SD_DMA_MODE)), IO_MMC_SD_DMA_MODE); // Disable DMA

        outw (0x0013, IO_SDRAM_SDDMASEL);

        cmd1 |= 0x8000;
        outw(cmd1, IO_MMC_COMMAND);

        //printk("Started writing %x blocks from the address %x\n",no_of_blks,current_add);
        //outw(no_of_blks,IO_MMC_NR_BLOCKS);

        outw(((unsigned long)buffer_short & 0xFFFF),IO_MMC_SD_DMA_ADDR_LOW);

        outw((((unsigned long)buffer_short >> 16) & 0xFFFF),IO_MMC_SD_DMA_ADDR_HI);

        outw((0xFFFF),IO_MMC_SD_DMA_TIMEOUT); // TIMEOUT

        outw((1 << 12) | inw (IO_MMC_SD_DMA_MODE),IO_MMC_SD_DMA_MODE);  // Direction

        outw((0x2000) | inw (IO_MMC_SD_DMA_MODE),IO_MMC_SD_DMA_MODE);   // Enable DMA

		dma_in_progress = 1;

        init_completion(&dma_complete);
        outw((MMC_INT_DMA_DONE | MMC_INT_DATA_DONE), IO_MMC_INT_ENABLE);

        outw((0x0001),IO_MMC_SD_DMA_TRIGGER);

        wait_for_completion(&dma_complete);

		mmc_mask_irq();

#if 0
		do {
			it_mmcsd_get_status(&cmd);
			printk("status = %X\n", cmd.status0);
		} while (!(cmd.status0 & (MMC_INT_DAT_TRX_RDY | MMC_INT_CRC_ERROR | MMC_INT_TIME_OUT_ERROR)) && 
				 !(cmd.status0 & 0x1));
#endif

        //it_mmcsd_print_status(&cmd);
        //printk ("Device Revision Number : %x\n",inw (IO_BUSC_REVR));

        len = remain;

        host->data_xfered += len;

        /*
        *Unmap the buffer.
        */
        mmci_kunmap_atomic(host, &flags);
        host->sg_off += len;
        host->size -= len;
        remain -= len;

#ifdef FINE_TUNING
		if (len > max_wr){printk("max_wr=%d\n", max_wr=len);}
#endif

#if 0
		if (!mmci_next_sg(host)){
		  it_mmcsd_get_status(&cmd);  
		  break;
		}
		//else printk("continue\n");

        if(mmc->card_busy && mmc_card_blockrw(mmc->card_busy))
            current_add += len/0x200;
        else 
            current_add += len;
#else
		break;
#endif
    }while(1);

    mrq->data->bytes_xfered = host->data_xfered;
    mmc_debug_msg ("mrq->data->bytes_xfered : %d\n",mrq->data->bytes_xfered);

	mmc_umask_irq(irq_mask);
    return 0;
}

static int read_from_card(struct mmc_host *mmc,struct mmc_request *mrq)
{
    u32 current_add;
    u16 no_of_blks;
    u32 marg;
    struct mmc_command cmd;
    struct mmci_host *host = mmc_priv(mmc);
    u16 cmd1;
    u16 irq_mask;
	//u16 blk_sz = 1<<mrq->data->blksz_bits;
	u16 blk_sz = 512;

    irq_mask = mmc_mask_irq();

    mrq->cmd->error = 0;
	mrq->cmd->mrq = mrq;
    host->cmd = mrq->cmd;

	if (mrq->data) {
	  mrq->cmd->data = mrq->data;
	  mrq->data->error = 0;
	  mrq->data->mrq = mrq;
	  if (mrq->stop) {
		mrq->data->stop = mrq->stop;
		mrq->stop->error = 0;
		mrq->stop->mrq = mrq;
	  }
	}
	
    host->data = mrq->data;
	host->size = mrq->data->blocks << mrq->data->blksz_bits;
	host->data_xfered = 0;
    mmci_init_sg(host, mrq->data);    
	
	// change to little endian.
	change_endian(0);
    /*************WORKING******************/
    /*
	 * Map the current scatter buffer.
	 20*/
    it_mmcsd_get_status(&cmd);
    it_mmcsd_clear_response_reg(&cmd);
	
	current_add = mrq->cmd->arg;

	no_of_blks = mrq->data->blocks;// * host->sg_len; //To read more than segment
    do{
	  unsigned long flags;
	  unsigned int remain, len = 0;
	  char *buffer;
	  char *buffer_short = NULL;
	  
	  buffer = mmci_kmap_atomic(host, &flags) + host->sg_off;

	  if (buffer == NULL) printk ("ERROR: NULL data Pointer\n");

	  remain = host->sg_ptr->length - host->sg_off;

	  buffer_short = buffer;
	  
#if 0 // seting block length.
	  outw(blk_sz & 0xFFFF,IO_MMC_ARG_LOW);
	  outw(0, IO_MMC_ARG_HI);
	  outw(16 | 0x0200, IO_MMC_COMMAND);

	  do
		{
		  it_mmcsd_get_status(&cmd);
		} while((!(cmd.status0&0x04))||(cmd.status1 & 0x01));
#endif

	  outw(ROUND2BLKSZ(remain, blk_sz),IO_MMC_NR_BLOCKS);
	  outw(blk_sz, IO_MMC_BLOCK_LENGTH);
	  
	  outw(((0xDFFF) & inw (IO_MMC_SD_DMA_MODE)),IO_MMC_SD_DMA_MODE);
	  
	  outw ((0x0013),IO_SDRAM_SDDMASEL);
	  
	  outw(((0xEFFF) & inw (IO_MMC_SD_DMA_MODE)),IO_MMC_SD_DMA_MODE);
	  
	  //outw((0x0400 | inw (IO_MMC_SD_DMA_MODE)),IO_MMC_SD_DMA_MODE);//Enable word swap for Reading
	  
	  outw((0xF000),IO_MMC_SD_DMA_TIMEOUT); //2.4576ms
	  
	  outw(((unsigned long)buffer_short & 0xFFFF),IO_MMC_SD_DMA_ADDR_LOW);
	  
	  outw((((unsigned long)buffer_short >> 16) & 0xFFFF),IO_MMC_SD_DMA_ADDR_HI);
	  
	  outw(((0x2000) | inw (IO_MMC_SD_DMA_MODE)),IO_MMC_SD_DMA_MODE);
	  
	  marg = current_add;

	  cmd1 = 18 | 0x8000 | 0x0200 | 0x2000 | 0x0080;
	  outw(marg & 0xFFFF,IO_MMC_ARG_LOW);
	  outw((marg >> 16),IO_MMC_ARG_HI);
	  outw(cmd1,IO_MMC_COMMAND);

	  dma_in_progress = 1;

	  init_completion(&dma_complete);
	  outw((MMC_INT_DMA_DONE | MMC_INT_DATA_DONE), IO_MMC_INT_ENABLE);
	  
	  outw((0x0001),IO_MMC_SD_DMA_TRIGGER);
	  
	  wait_for_completion(&dma_complete);
	
	  mmc_mask_irq();

	  len = remain;
	  host->data_xfered += len;
	  
	  /*
	   *Unmap the buffer.
	   */
	  mmci_kunmap_atomic(host, &flags);
	  host->sg_off += len;
	  host->size -= len;
	  remain -= len;

#ifdef FINE_TUNING
	  if (len > max_rd){printk("max_rd=%d\n", max_rd=len);}
#endif

	  if (!mmci_next_sg(host)){
        it_mmcsd_get_status(&cmd);    
        break;
	  }

      if(mmc->card_busy && mmc_card_blockrw(mmc->card_busy))
          current_add += len/0x200;
      else 
          current_add += len;

    }while(1);
    
    mrq->data->bytes_xfered = host->data_xfered;
    mmc_debug_msg ("mrq->data->bytes_xfered : %d\n",mrq->data->bytes_xfered);
	
    mmc_umask_irq(irq_mask);
    host->data = NULL;
    return 0;
}

static int src_read(struct mmc_host *mmc,struct mmc_request *mrq,unsigned int *relative,char* scr_reg)
{
    u32 marg;
    int read_count;
    struct mmc_command cmd;
    short temp_data;
    u16 cmd1,reg;
    u16 irq_mask;

    irq_mask = mmc_mask_irq();

	// switching clocks for SD card.
	init_clocks(1);

    memset(scr_reg,0,8*sizeof(char));
    
    do{

        char *buffer = NULL;
		short *buffer_short = NULL;
		int count = 0;

		buffer = scr_reg;

		it_mmcsd_clear_response_reg(&cmd);
		it_mmcsd_get_status(&cmd);
		//it_mmcsd_print_status(&cmd);    
		
        outw(1,IO_MMC_NR_BLOCKS);

		outw(0x200, IO_MMC_BLOCK_LENGTH);

		buffer_short = (short*)buffer;
		
		
        marg = 0x200;
		// added in a init clock after power on.
        cmd1 = 51 | 0x0200 | 0x2000 | 0x0080 | 0x4000;
        outw(marg & 0xFFFF,IO_MMC_ARG_LOW);
        outw((marg >> 16),IO_MMC_ARG_HI);
        outw(cmd1,IO_MMC_COMMAND);
		
        {
		  int t = 0;
		  do {
			it_mmcsd_get_status(&cmd);
			if (++t == 0xfffff)
			  {
				printk("sd/mmc src_read timeout waiting for busy card\n");
				goto bail;
			  }
		  } while( !((cmd.status0 & 0x4)==4));
		  //checking for busy state and response of the command
        }
		
        //it_mmcsd_print_status(&cmd);    
		
        read_count = 0;
        do{
		  it_mmcsd_get_status(&cmd);
		  //it_mmcsd_print_status(&cmd);    
		  reg = cmd.status0;
		  if (reg & 0x0040)
			{
			  mmc_debug_msg ("<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<Read CRC error\n");
			  mmc_debug_msg ("STATUS0 :%x\tSTATUS1 :%x\n",cmd.status0,cmd.status1);
			  break;
			}
		  if ((cmd.status0 & 0x400))
			{
			  temp_data = 0;
			  temp_data = inw (IO_MMC_RX_DATA);
			  mmc_debug_msg_rw ("%x\t",temp_data);
			  if (read_count < 4){        
                *buffer = (temp_data >> 8) & 0xFF;
                buffer = buffer + 1;
                *buffer = temp_data & 0xFF;    
                buffer = buffer + 1;
			  }
			  read_count++;
			  if (read_count > 0x200)
                break;
			}
		  else    
			if (cmd.status0 & 0x10){
			  printk("Command Time OUT");
			  break;
			}
		  //printk("reg =  %X read_count = %X\n", reg, read_count);
		} while (!(reg &0x0001));

        do{
		  if (count ++ > 100)
            break;
		  it_mmcsd_get_status(&cmd);
		}while((cmd.status1 & 0x1));

		buffer_short = NULL ;
		break;
		
    }while(1);

 bail:	
    mmc_umask_irq(irq_mask);

    return 0;
}

static void host_configuration(void)
{
    short reg = 0;
    u16 irq_mask;

    irq_mask = mmc_mask_irq();

    /* NONE-MS mode, must be done first, otherwize 
       can't work with smartmedia, though they are using
       different controller
       if ms card inserted then needn't do this. */
    if(gio_get_bitset(GIO_MS_CARDDETECT))
         outw(0, IO_MEM_STICK_MODE);

    reg = inw(IO_CLK_MOD2);
    outw(reg & (~(0x4000)), IO_CLK_MOD2); // disable ms clk
    outw(0, IO_MEM_STICK_MODE); //NONE-MS mode
    reg = inw(IO_CLK_MOD2);
    outw(reg | 0x0800, IO_CLK_MOD2); //enable mmc clk

    init_clocks(0); // defaults to low SD speed clock.

    outw(0x0, IO_MMC_SD_DMA_MODE);

    /* reset the sd/mmc controller */
    outw(0x03, IO_MMC_CONTROL);

    /*Response timeout register */
    outw(255,IO_MMC_RESPONSE_TIMEOUT);

    /*Data timeout register*/
    reg = inw (IO_MMC_RESPONSE_TIMEOUT);
    outw(reg | 0x1F00,IO_MMC_RESPONSE_TIMEOUT);
    outw(0xFFFF,IO_MMC_READ_TIMEOUT);

    /* remove reset, do not use DMASZEN (see newest errata) */
    /* set big endian for src_read and little endian for read/write */ 
    outw(0x600, IO_MMC_CONTROL);
	
    mmc_umask_irq(irq_mask);
}

static void start_debounce_timer(void);
static void mmci_plug_status(unsigned long data)
{
    struct mmci_host *host = (struct mmci_host *)data;
	u16 status;
    int reg;

	//printk("status = %X:\n", inw(IO_GIO_CARD_ST));
	//printk("connect = %X:\n", gio_get_bitset(GIO_SD_CARDDETECT));

	status = gio_get_bitset(GIO_SD_CARDDETECT);
	if(status == 0) //plug-in
	  {
		//TBD: remove below code once the mount logic is figured out.
		if (mmc_hdl->ios.power_mode != MMC_POWER_OFF) 
		  {
			start_debounce_timer();
			return;
		  }
		//TBD:

		printk("card pluged in. \n");
		reg = inw(IO_MEM_STICK_MODE);
		rmb();
		outw(0, IO_MEM_STICK_MODE); //NONE-MS mode
		wmb();
		
		reg = inw (IO_CLK_MOD2);
		rmb();
		outw(reg & (~(0x4000)), IO_CLK_MOD2);//DISABLE MS CLK
		wmb();
		outw (0,IO_MEM_STICK_MODE);
		
		reg = inw( IO_CLK_MOD2 );
		rmb();
		outw(reg | 0x0800, IO_CLK_MOD2); //Enable MMC CLK
		wmb();
		host_configuration();
	  }
	else
	  {
		printk("card unpluged. \n");
	  }
	mmc_detect_change(host->mmc, 50);
}


static int get_write_protect_status(struct mmc_host *mmc)
{

    mmc_debug_msg("Write protect status : %x\n",gio_get_bitset(GIO_SDCARD_WP));
    return gio_get_bitset(GIO_SDCARD_WP);
}    

static struct mmc_host_ops mmci_ops = {
        .request    = mmci_request,
        .set_ios    = mmci_set_ios,
        .get_ro     = get_write_protect_status,
};

static int mmci_probe(struct device *dev)
{
    struct mmc_platform_data *plat = dev->platform_data;
    struct mmci_host *host = NULL;
    struct mmc_host *mmc = NULL;
    int ret;

    mmc_mask_irq();

    mmc = mmc_alloc_host(sizeof(struct mmci_host), dev);

	// remembers it.
	mmc_hdl = mmc;

	//TBD: remove below code once the mount logic is figured out.
	mmc_hdl->ios.power_mode = MMC_POWER_OFF; 
	//TBD:

    bCardInitialized = 0;
    
    if (!mmc) {
        ret = -ENOMEM;
        mmc_debug_msg(" mmc_alloc_host failed \n");
        goto rel_regions;
    }

    host = mmc_priv(mmc);
    host->cmd = NULL;

    mmc->readdata = read_from_card;//used in calling the read system call
    mmc->writedata = write_to_card;//used in calling the write system call
    mmc->src_data = src_read;//used in calling src read function
    mmc->enable_interrupt = enable_interrupt;
    mmc->disable_interrupt = disable_interrupt;
    mmc->change_clk25m = change_clk25m;

    host->plat = plat;
    host->mmc = mmc;

    mmc->ops = &mmci_ops;

    mmc->ocr_avail = 1 << 16;//Has to be done by platform data
    /*
     * We can do SGIO
     */
    mmc->max_hw_segs = 16;
    mmc->max_phys_segs = NR_SG;

    /*
     * Since we only have a 16-bit data length register, we must
     * ensure that we don't exceed 2^16-1 bytes in a single request.
     * Choose 64 (512-byte) sectors as the limit.
     */
    mmc->max_sectors = 64;

    /*
     * Set the maximum segment size.  Since we aren't doing DMA
     * (yet) we are only limited by the data length register.
     */
    mmc->max_seg_size = mmc->max_sectors << 9;
        
    ret = request_irq(IRQ_SD_MMC, &dm320_irq, SA_SHIRQ, DRIVER_NAME " (cmd)", host);
    if (ret){    
        printk ("Interrupt %d may be busy\n",IRQ_SD_MMC);
        free_irq(IRQ_SD_MMC,host);
    }

    dev_set_drvdata(dev,mmc);

    mmc_add_host(mmc);
    init_sd_mmc();
    host_configuration(); //Host controller configuration    

    enable_interrupt();

    mmc_debug_msg ("Leaving mmci probe\n");

    return 0;

 rel_regions:
    return ret;
}

static int mmci_remove(struct device *dev)
{
    struct mmc_host *mmc;
    mmc_debug_msg ("entering %s\n",__FUNCTION__);

	disable_interrupt();
	mmc = dev_get_drvdata(dev);

    dev_set_drvdata(dev, NULL);

    if (mmc) {
        struct mmci_host *host = mmc_priv(mmc);
		if (timer_inited)
		  {
			del_timer_sync(&host->timer);
			timer_inited = 0;
		  }
        mmc_remove_host(mmc);

        mmc_free_host(mmc);
        
        free_irq(IRQ_SD_MMC,mmc_priv(mmc));
    }
    
    mmc_debug_msg ("leaving %s\n",__FUNCTION__);
	printk("I am here, leaving.\n");

	//enable_interrupt(); 
	//enable only the detection interrupt.
	{
	  unsigned long flags;
	  spin_lock_irqsave(&mmc_lock, flags);
	  outw((inw(IO_INTC_EINT1) | 0x8),IO_INTC_EINT1);
	  spin_unlock_irqrestore(&mmc_lock, flags);
	}	
    
    return 0;
}

static struct device_driver mmci_driver = { 
    .name       = DRIVER_NAME,
    .bus        = &platform_bus_type,
    .probe      = mmci_probe,
    .remove     = mmci_remove,
    .suspend    = NULL,
    .resume     = NULL,
};

static void start_debounce_timer(void)
{
    struct mmci_host *host = mmc_priv(mmc_hdl);
	
	if (timer_inited)
	  {
		mod_timer(&host->timer, jiffies + HZ);
	  }
	else
	  {
		init_timer(&host->timer);
		host->timer.data = (unsigned long)host;
		host->timer.function = mmci_plug_status;
		host->timer.expires = jiffies + HZ;
		add_timer(&host->timer);
		timer_inited = 1;
	  }
}

static irqreturn_t it_mmcsd_detect_interrupt(int flags, void * data, struct pt_regs * regs)
{
    u16 irq_mask;

    irq_mask = mmc_mask_irq();

	printk("SD/MMC detect interrupt fired\n");
	start_debounce_timer();

    mmc_umask_irq(irq_mask);

    return IRQ_HANDLED;
}


static void init_sd_mmc(void)
{
    int status;

	outw (0x0003, IO_GIO_CARD_SET);  //inversion & card detect enable

    status = request_gio(GIO_SD_CARDDETECT);
    if (status){
        mmc_debug_msg("\tUnable to register gio1 %d\n", GIO_SD_CARDDETECT);
    }
    gio_enable_irq(GIO_SD_CARDDETECT, GIO_ANY_EDGE);

    /* ms card detect GIO register, because it need be used in this driver */
    status = request_gio(GIO_MS_CARDDETECT);
    if (status)
        printk("\tUnable to register gio1 %d\n",GIO_MS_CARDDETECT);
    gio_enable_irq(GIO_MS_CARDDETECT,GIO_ANY_EDGE);

    mmc_debug_msg ("IRQ_GIO8 : %d\t\n",IRQ_GIO8);

    status = request_irq(IRQ_GIO8 /* GIO_SD_CARDDETECT */,it_mmcsd_detect_interrupt,0,"mmc/sd DMA controller",NULL);
    if (status){
        mmc_debug_msg("\tUnable to register irq \t\n");
    }

    status = request_gio(GIO_SDCARD_WP);
    if (status){
        mmc_debug_msg("\tUnable to register gio2 %d\n",GIO_SDCARD_WP);
    }

    gio_set_dir(GIO_SDCARD_WP,bit_hi);
}

static int __init mmci_init(void)
{
    int val = 0;

    printk("Configure the MMC card controler\n");
    val = driver_register(&mmci_driver);

    if(!gio_get_bitset(GIO_SD_CARDDETECT))
	  {
		start_debounce_timer();
	  }
    return val;
}

static void __exit mmci_exit(void)
{
    unsigned short reg;

    disable_interrupt();

    reg = inw(IO_MMC_MEM_CLK_CONTROL);
    outw(reg & 0xFEFF, IO_MMC_MEM_CLK_CONTROL);

    unrequest_gio(GIO_SD_CARDDETECT);
    gio_disable_irq(GIO_MS_CARDDETECT);
    unrequest_gio(GIO_MS_CARDDETECT);
    free_irq(IRQ_GIO8,NULL);
    unrequest_gio(GIO_SDCARD_WP);
    driver_unregister(&mmci_driver);
}

module_init(mmci_init);
module_exit(mmci_exit);
module_param(fmax, uint, 0444);

MODULE_DESCRIPTION("ARM PrimeCell PL180/181 Multimedia Card Interface driver");
MODULE_LICENSE("GPL");
