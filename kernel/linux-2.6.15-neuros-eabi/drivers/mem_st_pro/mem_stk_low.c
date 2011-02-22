/*
 *
 * Copyright (C) 2005-2006 Ingenient Technologies
 *
 * Copyright (C) 2006 Neuros Technology LLC.
 * Brought in DMA support and support for card with capacity > 1GB. mgao@neuros
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
#include <linux/ms/host.h>
#include <asm/div64.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/scatterlist.h>
#include <asm/hardware/amba.h>
#include <linux/ms/ms.h>
#include <linux/ms/mem_stick_reg.h>
#include <linux/interrupt.h>
#include <asm/arch/gios.h>
#include <asm/arch/gio.h>
#include <asm/arch/irqs.h>

#include "mem_stk_low.h"
#include "mem_stk_debug.h"

MODULE_LICENSE("GPL");

#define IRQ_MS_CARDDETECT IRQ_GIO5

static struct clk *clk_get(struct device *dev, const char *id);
static void clk_put(struct clk *clk);
static int clk_enable(struct clk *clk);
static int clk_use(struct clk *clk);
static unsigned long clk_get_rate(struct clk *clk);
static void clk_unuse(struct clk *clk);
void change_clk25m(void);
void display_MS_CONTROL_REGS(void);
static void enable_interrupt(void);
static void disable_interrupt(void);

/* mem_stk.c */
unsigned short is_command_end(unsigned char reg);

static int command_in_progress = 0;
static int dma_in_progress = 0;	
static int timer_inited = 0;
static spinlock_t  ms_lock;

static DECLARE_COMPLETION(dma_complete);
#define DMA_WAIT_INIT() do{dma_in_progress = 1; init_completion(&dma_complete);}while(0)
#define DMA_WAIT_DONE() do{wait_for_completion(&dma_complete); dma_in_progress = 0;}while(0)


static void init_sd_ms(void);
struct msi_host *gmsi_host = NULL; /* Need to be put in dev->priv */

static void mem_outw (unsigned short val,unsigned long add)
{
    outw (val,add);
    wmb();
}

static unsigned short mem_inw (unsigned long add)
{
    unsigned short val;
	
    val = inw (add);
    rmb();
    return val;
}

void display_MS_CONTROL_REGS(void)
{
    printk ("Value of IO_MEM_STICK_MODE           : %x\n",mem_inw(IO_MEM_STICK_MODE));
    printk ("Value of IO_MEM_STICK_CMD            : %x\n",mem_inw(IO_MEM_STICK_CMD));
    printk ("Value of IO_MEM_STICK_DATA           : %x\n",mem_inw(IO_MEM_STICK_DATA));
    printk ("Value of IO_MEM_STICK_STATUS         : %x\n",mem_inw(IO_MEM_STICK_STATUS));
    printk ("Value of IO_MEM_STICK_SYS            : %x\n",mem_inw(IO_MEM_STICK_SYS));
    printk ("Value of IO_MEM_STICK_ENDIAN         : %x\n",mem_inw(IO_MEM_STICK_ENDIAN));
    printk ("Value of IO_MEM_STICK_INT_STATUS     : %x\n",mem_inw(IO_MEM_STICK_INT_STATUS));
    printk ("Value of IO_MEM_STICK_DMA_TRG        : %x\n",mem_inw(IO_MEM_STICK_DMA_TRG));
    printk ("Value of IO_MEM_STICK_DMA_MODE       : %x\n",mem_inw(IO_MEM_STICK_DMA_MODE));
    printk ("Value of IO_MEM_STICK_SDRAM_ADDL     : %x\n",mem_inw(IO_MEM_STICK_SDRAM_ADDL));
    printk ("Value of IO_MEM_STICK_SDRAM_ADDH     : %x\n",mem_inw(IO_MEM_STICK_SDRAM_ADDH));
    printk ("Value of IO_MEM_STICK_DMA_STATUS     : %x\n",mem_inw(IO_MEM_STICK_DMA_STATUS));
}

static void enable_interrupt(void)
{
    unsigned long flags;
    spin_lock_irqsave(&ms_lock, flags);
    outw((inw(IO_INTC_EINT1) | 0x08),IO_INTC_EINT1);
    spin_unlock_irqrestore(&ms_lock, flags);
}

static void disable_interrupt(void)
{
    unsigned long flags;

    spin_lock_irqsave(&ms_lock, flags);
    outw((inw(IO_INTC_EINT1) &~ 0x8),IO_INTC_EINT1);
    spin_unlock_irqrestore(&ms_lock, flags);
}

#define DRIVER_NAME "MEM_STICK"

void change_clk25m(void)
{
}

static void msi_request_end(struct msi_host *host, struct ms_request *mrq)
{
    host->mrq = NULL;
    host->cmd = NULL;
    
    if (mrq->data)
	mrq->data->bytes_xfered = host->data_xfered;

    spin_unlock(&host->lock);
    ms_request_done(host->ms, mrq);
    spin_lock(&host->lock);
}

static void msi_start_command(struct msi_host *host, struct ms_command *cmd, u32 c)
{
    host->cmd = cmd;
    command_in_progress = 1;
    do{
        if (mem_inw(IO_MEM_STICK_STATUS) & 0x1000)
	  {
	    mem_outw ((cmd->opcode | cmd->size),IO_MEM_STICK_CMD);
	    break;
	  }
    }while(1);
}

static void msi_cmd_irq(struct msi_host *host, struct ms_command *cmd, unsigned int status)
{
    host->cmd = NULL;
	
    if (status & MS_TIME_OUT) 
      {
        cmd->error = MEM_STICK_TIME_OUT;
	//printk("Card has not responded: status = %d\n", status);
      } 
    else if (status & MS_CRC_ERROR)
      {
        printk("CRC ERROR\n");
	cmd->error = MEM_STICK_CRC_ERROR;
      }
    
    if (!cmd->data || cmd->error != MEM_STICK_NO_ERROR) 
        msi_request_end(host, cmd->mrq);
}

void receive_data(short num,unsigned char *rxbuf,unsigned short *status)
{	
    unsigned short reg;
    int i=0;
    unsigned short data;
	
    do{
        reg = mem_inw(IO_MEM_STICK_STATUS);
	/* FIFO has data or full */
	if (reg & (0x4000 | 0x10))
	  {	
	    *status &= ~MS_TIME_OUT;
	    break;
	  }	
	if (i++ > 255)
	  {
	    *status |= MS_TIME_OUT;			
	    printk("No data received\n");			
	    
	    while (!(mem_inw(IO_MEM_STICK_STATUS) & 0x1000));
	    *status = mem_inw(IO_MEM_STICK_STATUS);
	    return;
	  }
    }while (!(reg & 0x4000)); /* data in FIFO */
	
    i= 0;
    while(1)
      {
        reg = mem_inw(IO_MEM_STICK_STATUS);
	
	if (!(reg&0x4000) && (i >= num))
	    break;
		
	if ((!(reg & 0x20)) && (reg &0x4000))
	  {
	    data = mem_inw(IO_MEM_STICK_DATA);
	    rxbuf[i] = (data >> 8) & 0xFF;
	    i++;
	    rxbuf[i] = data & 0xFF;
	    i++;
	  }
      }
    while (!(mem_inw(IO_MEM_STICK_STATUS) & 0x1000));
    *status = mem_inw(IO_MEM_STICK_STATUS);
}

void send_data(unsigned char* buf,short num,short *status)
{
    unsigned short reg;
    int i;
    
    do{
        reg = mem_inw(IO_MEM_STICK_STATUS);
    }while (!(reg & 0x4000) & !(reg & 0x20)); /* data in FIFO */
    
    i = 0;
    do{
        reg = mem_inw(IO_MEM_STICK_STATUS);
	
	if (!(reg&0x4000) & (i >= num))
	    break;
	
	if ((!(reg & 0x10)) && (reg &0x4000))
	  {
	    mem_outw((((buf[i] << 8) & 0xFF00)|(buf[i+1] & 0xFF)),IO_MEM_STICK_DATA);
	    i+=2;
	  }
    }while (1);
    
    while (!(mem_inw(IO_MEM_STICK_STATUS) & 0x1000));
    *status = mem_inw(IO_MEM_STICK_STATUS);
}

/*
 * Handle completion of command and data transfers.
 */
static irqreturn_t msi_irq(int irq, void *dev_id, struct pt_regs *regs)
{
    struct msi_host *host = dev_id;
    int ret = 1;
    struct ms_command *cmd = NULL;
    struct ms_command temp_cmd;	
    
    mem_outw ((mem_inw (IO_MEM_STICK_SYS) | MSINTCLR),IO_MEM_STICK_SYS);/* Clear all interrupts */
    
    spin_lock(&host->lock);
    
    if (dma_in_progress)
      {
	if (!(mem_inw(IO_MEM_STICK_DMA_STATUS) & MSRUN))
	  {
	    mem_outw(MSBTRG,IO_MEM_STICK_DMA_TRG);/* DMA BREAK */
	    mem_outw((mem_inw(IO_MEM_STICK_SYS) & (~MSDAM))|MSDRQSL,IO_MEM_STICK_SYS );/* DMA MODE */
	    complete(&dma_complete);
	    goto bail;
	  }
      }

    cmd = host->cmd;
    if (!cmd)
      {
	memset (&temp_cmd,0,sizeof(struct ms_command));
	cmd = &temp_cmd;
      }
    cmd->status0 = mem_inw(IO_MEM_STICK_INT_STATUS);

    if (cmd->flags == REG_RW_CMD)
      {
	if ((cmd->status0) & (MS_CRC_ERROR|MS_TIME_OUT|MS_ERROR|MS_DATA_REQ))
	  {
	    if (cmd->status0 & MS_DATA_REQ)
	      {
		if (mem_inw(IO_MEM_STICK_SYS) & (1<<8))
		  {	
		    if (cmd->size > 8)
			send_data(cmd->arg,16,&cmd->status0);
		    else
			send_data(cmd->arg,8,&cmd->status0);
		  }
		else
		    receive_data(8,(unsigned char*)cmd->resp,&cmd->status0);
	      }
	    if (command_in_progress)
	      {
		msi_cmd_irq(host,cmd,cmd->status0);
		command_in_progress = 0;
	      }
	  }
	//else printk ("%s what is this then ??\n",__FUNCTION__);
      } 
    //else printk ("%s really??\n",__FUNCTION__);
    
 bail:
    spin_unlock(&host->lock);
    return IRQ_RETVAL(ret);
}

static void msi_request(struct ms_host *ms, struct ms_request *mrq)
{
    struct msi_host *host = ms_priv(ms);
	
    WARN_ON(host->mrq != NULL);
    spin_lock_irq(&host->lock);
    host->mrq = mrq;
    msi_start_command(host, mrq->cmd, 0);
    spin_unlock_irq(&host->lock);
}

static void msi_set_ios(struct ms_host *ms, struct ms_ios *ios)
{
    if (ms->ios.bus_width == PARALLEL)
        mem_outw ((mem_inw (IO_MEM_STICK_SYS) & ~(MSSRAC) ),IO_MEM_STICK_SYS);/* parallel mode enable */
    else
        mem_outw ((mem_inw (IO_MEM_STICK_SYS) | (MSSRAC) ),IO_MEM_STICK_SYS);/* serial mode enable */
    
    if (ms->ios.change_clk == 1)
      {
	mem_outw ((mem_inw (IO_CLK_MOD2) & (~(0x4000))),IO_CLK_MOD2);/* disable MS clk.       */
	mem_outw ((mem_inw (IO_CLK_SEL0) | 0x00),IO_CLK_SEL0);       /* MS clk source as PLLB */
	mem_outw ((mem_inw (IO_CLK_DIV2) & (~(0x1F))),IO_CLK_DIV2);  /* dividing factor is 2  */
	mem_outw ((mem_inw (IO_CLK_DIV2) | 0x5),IO_CLK_DIV2);        /* dividing factor is 2  */
	mem_outw ((mem_inw (IO_CLK_MOD2) | 0x4000),IO_CLK_MOD2);     /* Enable MS clk.        */
      }

    if (ms->ios.change_clk == 2)
      {
	mem_outw ((mem_inw (IO_CLK_MOD2) & (~(0x4000))),IO_CLK_MOD2);/* disable MS clk.       */
	mem_outw ((mem_inw (IO_CLK_SEL0) | 0x00),IO_CLK_SEL0);       /* MS clk source as PLLB */
	mem_outw ((mem_inw (IO_CLK_DIV2) & (~(0x1F))),IO_CLK_DIV2);  /* dividing factor is 2  */
	mem_outw ((mem_inw (IO_CLK_DIV2) | 0xD),IO_CLK_DIV2);        /* dividing factor is 2  */
	mem_outw ((mem_inw (IO_CLK_MOD2) | 0x4000),IO_CLK_MOD2);     /* Enable MS clk.        */
      }
}

int write_to_card(struct ms_host *ms,struct ms_request *mrq)
{
    struct msi_host *host = ms_priv(ms);

    disable_interrupt();

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
    msi_init_sg(host, mrq->data);
    
    do{
        unsigned long flags;
	unsigned int remain, len = 0;
	char *buffer = NULL;
	int retry = 0;
	
	buffer = msi_kmap_atomic(host, &flags) + host->sg_off;
	if (buffer == NULL)	
	    ms_debug_msg ("NULL data Pointer\n");
	
        remain = host->sg_ptr->length - host->sg_off;
	while (remain)
	  {
	    unsigned short reg, status;
	    unsigned char buf_temp[8];
	    int rqst;
	    
	    do {
	        reg = mem_inw(IO_MEM_STICK_STATUS);
	    } while (!(reg & 0x1000));

	    mem_outw (GET_INT | 0x1,IO_MEM_STICK_CMD);
	    receive_data(8,buf_temp,&status);

	    status = is_command_end(buf_temp[0]);
	    if (NORM_DATA_TRANS == status)
	      {
		retry = 0;
		rqst = (remain > 512)? 512: remain;
		
		mem_outw((mem_inw(IO_MEM_STICK_SYS) &(~MSDRQSL))| MSDAM,IO_MEM_STICK_SYS );
		mem_outw((0x00<<10)|0x2000|0x1000|rqst, IO_MEM_STICK_DMA_MODE);
		mem_outw (0x0002, IO_SDRAM_SDDMASEL);
		mem_outw((((unsigned long)buffer) & 0xFFFF),IO_MEM_STICK_SDRAM_ADDL);
		mem_outw((((unsigned long)buffer >> 16) & 0xFFFF),IO_MEM_STICK_SDRAM_ADDH);	
		
		enable_interrupt();
		mem_outw (WRITE_PAGE_DATA | rqst,IO_MEM_STICK_CMD);
		DMA_WAIT_INIT();
		mem_outw(MSSTRG,IO_MEM_STICK_DMA_TRG);
		DMA_WAIT_DONE();
		disable_interrupt();
		
		len += rqst;
		remain -= rqst;
		buffer += rqst;
	      }
	    else if (NORM_COMP == status) break;
	  }
	
        host->data_xfered += len;
        msi_kunmap_atomic(host, &flags);
        host->sg_off += len;
        host->size -= len;
	
        if (!msi_next_sg(host))
	   break;
    }while(1);
	
    mrq->data->bytes_xfered = host->data_xfered;
    host->data = NULL;

    enable_interrupt();
    
    return 0;
}

void update_ms_page_ms(struct ms_host *ms,short page_add,short block_size,
		       struct ms_request *mrq,unsigned char*cur_blk_data)
{	
    char *buffer = NULL;
    char  *buffer_char = NULL;
    unsigned int remain, len = 0;
    unsigned long flags;
    struct msi_host *host = ms_priv(ms);
    int jj,ii;
    
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
    msi_init_sg(host, mrq->data);
    
    /*************WORKING******************/
    /*
     * Map the current scatter buffer.
     20*/
    
    buffer = msi_kmap_atomic(host, &flags) + host->sg_off;
    if (buffer == NULL)
        ms_debug_msg ("NULL data Pointer\n");
    
    remain = host->sg_ptr->length - host->sg_off;
    buffer_char = buffer;
    
    jj = 0;
    ii = 0;
    do{
        if ((buffer_char-buffer) == remain)
	  break;
      
	memcpy((unsigned char*)(cur_blk_data+(page_add * 512)),buffer_char,512);	
	buffer_char = buffer_char+512;
	page_add += 1;
	
	if (page_add > ((block_size*2)-1))			
	  break;
      
    }while(1);
    
    len = buffer_char - buffer;
    host->data_xfered += len;
    /*
     *Unmap the buffer.
     */
    msi_kunmap_atomic(host, &flags);
    host->sg_off += len;
    host->size -= len;
    remain -= len;
    mrq->data->bytes_xfered = host->data_xfered;
}

int write_to_card_blk(unsigned char* data,int data_size,
		      struct ms_host *ms,struct ms_request *mrq)
{
    char  *buffer = NULL;
    struct msi_host *host = ms_priv(ms);
    
    spin_lock(&host->lock);
    local_irq_disable();
    buffer = data;
    
    do{
        int ii=0,jj=0;
	unsigned short reg1,reg;
	unsigned char buf_temp[8];
	unsigned short status;	
	
	/* Checking to read the data */
	jj = 0;
	ii = 0;
	do{
	    do{
	        reg1 = mem_inw(IO_MEM_STICK_STATUS);
		if (reg1 & MS_TIME_OUT)
		    goto write_out;
		
		if (reg1 &0x1000)
		  {
		    mem_outw (GET_INT | 0x1,IO_MEM_STICK_CMD);
		    break;
		  }
	    }while(1);

	    receive_data(8,buf_temp,&status);
	    if (is_command_end(buf_temp[0]) == NORM_DATA_TRANS)
	      {
		if (((unsigned long)buffer-(unsigned long)data) == data_size)
		  break;
		mem_outw (WRITE_PAGE_DATA | 512,IO_MEM_STICK_CMD);

		do{
		  reg = mem_inw(IO_MEM_STICK_STATUS);
		  if (reg & MS_TIME_OUT)
		    goto write_out;
		}while (!(reg & 0x4000)); /* data in FIFO */
		
		send_data((unsigned char *)buffer,512,&status);
		buffer = buffer+512;
	      }
	    if ((is_command_end(buf_temp[0]) == NORM_COMP))
	        break;
	    
	}while(jj++ < 255);
	
	if (((unsigned long)buffer-(unsigned long)data) == data_size)
	    break;
	/*
	 *Unmap the buffer.
	 */
    }while(1);
 
write_out:
    local_irq_enable();
    mem_outw ((mem_inw (IO_MEM_STICK_SYS) | MSIEN),IO_MEM_STICK_SYS);/* Enable interrupts */
    spin_unlock(&host->lock);
    
    host->data = NULL;
    
    return 0;	
}

int read_from_card_ms(struct ms_host *ms,struct ms_request *mrq)
{
    struct msi_host *host = ms_priv(ms);
	
    disable_interrupt();

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
    msi_init_sg(host, mrq->data);	
    
    do{
        unsigned long flags;
	unsigned int remain, len = 0;
	char *buffer;
	
	buffer = msi_kmap_atomic(host, &flags) + host->sg_off;
	if (buffer == NULL)	ms_debug_msg ("NULL data Pointer\n");
	remain = host->sg_ptr->length - host->sg_off;
		
	while (remain)
	  {
	    unsigned short reg, status;
	    unsigned char buf_temp[8];
	    int rqst;
	    
	    do {
	        reg = mem_inw(IO_MEM_STICK_STATUS);
	    } while (!(reg & 0x1000));

	    mem_outw (GET_INT | 0x1,IO_MEM_STICK_CMD);
	    receive_data(8,buf_temp,&status);
	    status = is_command_end(buf_temp[0]);

	    if (NORM_DATA_TRANS == status)
	      {
		rqst = (remain > 512)? 512: remain;
		
		mem_outw((mem_inw(IO_MEM_STICK_SYS) &(~MSDRQSL))| MSDAM,IO_MEM_STICK_SYS );
		mem_outw((0x00<<10)|0x2000|rqst, IO_MEM_STICK_DMA_MODE);
		mem_outw (0x0002, IO_SDRAM_SDDMASEL);
		mem_outw((((unsigned long)buffer) & 0xFFFF),IO_MEM_STICK_SDRAM_ADDL);
		mem_outw((((unsigned long)buffer >> 16) & 0xFFFF),IO_MEM_STICK_SDRAM_ADDH);	
		
		enable_interrupt();
		mem_outw (READ_PAGE_DATA | rqst,IO_MEM_STICK_CMD);
		DMA_WAIT_INIT();
		mem_outw(MSSTRG,IO_MEM_STICK_DMA_TRG);
		DMA_WAIT_DONE();
		disable_interrupt();
		
		len += rqst;
		remain -= rqst;
		buffer += rqst;
	      }
	    else if (NORM_COMP == status) break;
	  }
	host->data_xfered += len;
	
	/*
	 * Unmap the buffer.
	 */
	msi_kunmap_atomic(host, &flags);
	host->sg_off += len;
	host->size -= len;
	remain -= len;
	
	if (!msi_next_sg(host))
	    break;
    }while(1);
    
    mrq->data->bytes_xfered = host->data_xfered;
    host->data = NULL;    

    enable_interrupt();
    
    return 0;
}

int read_from_card_blk(unsigned char* data,int data_size,
		       struct ms_host *ms,struct ms_request *mrq)
{
    struct msi_host *host = ms_priv(ms);
    unsigned char* data_temp = data;	
    int jj;
    int ii;
    unsigned short reg1=0,status = 0,reg = 0;
    unsigned char buf_temp[8];

    spin_lock(&host->lock);
    local_irq_disable();
	
    do{
        /* Checking to read the data */
        jj = 0;
	ii = 0;
	do{
	    do{
	        reg1 = mem_inw(IO_MEM_STICK_STATUS);
		if (reg1 & MS_TIME_OUT)
		    goto read_out;
				
		if (reg1 &0x1000)
		  {
		    mem_outw (GET_INT | 0x1,IO_MEM_STICK_CMD);
		    break;
		  }
	    }while(1);
	    
	    receive_data(8,buf_temp,&status);
	    if (is_command_end(buf_temp[0]) == NORM_DATA_TRANS)
	      {
		if ((data_temp - data) == data_size)
		    break;
		mem_outw (READ_PAGE_DATA | 512,IO_MEM_STICK_CMD);
		do{
		    reg = mem_inw(IO_MEM_STICK_STATUS);
		}while (!(reg & 0x4000)); /* data in FIFO */
				
		receive_data(512,(unsigned char *)data_temp,&status);
		data_temp = data_temp+512;
	      }
	    if ((is_command_end(buf_temp[0]) == NORM_COMP))
	        break;

	}while(jj++ < 255);

    }while((data_temp - data) != data_size);
	
read_out:
    local_irq_enable();
    mem_outw ((mem_inw (IO_MEM_STICK_SYS) | MSIEN),IO_MEM_STICK_SYS);//Enable interrupts
    spin_unlock(&host->lock);
    
    return 0;
}

int read_attribute_data(struct ms_host *ms,void *buf,short blksz_bits)
{		
    int i = 0,j=0;
    unsigned short reg;
    struct msi_host *host = ms_priv(ms);
    unsigned char buf_temp[8];
    unsigned short status,reg1 = 0;
    
    memset(buf_temp,0,sizeof(buf_temp));
    spin_lock(&host->lock);
    local_irq_disable();
    
    j =0;
    while (j < (blksz_bits))
      {
	i = 0;
	do{
	    reg1 = mem_inw(IO_MEM_STICK_STATUS);
	    if (reg1 & MS_TIME_OUT)
	        goto read_at_out;
	    if (reg1 & 0x1000)
	      {
		mem_outw (GET_INT | 0x1,IO_MEM_STICK_CMD);
		break;
	      }
	}while(1);

	receive_data(8,buf_temp,&status);
	if (is_command_end(buf_temp[0]) == NORM_DATA_TRANS)
	  {
	    mem_outw (READ_PAGE_DATA | 512,IO_MEM_STICK_CMD);
	    do{
	        reg = mem_inw(IO_MEM_STICK_STATUS);
		if (reg & MS_TIME_OUT)
		    goto read_at_out;
	    }while (!(reg & 0x4000)); /* data in FIFO */
	    receive_data(512,((unsigned char*)buf + j*512),&status);
	    j++;
	  }
	if (is_command_end(buf_temp[0]) == NORM_COMP)
	    break;
      }
 read_at_out:
    local_irq_enable();
    spin_unlock(&host->lock);
    return 0;
}

static volatile int plug_count = 0;
static int dat3stat_new = 0;

static void host_configuration(void)
{		
    ms_debug_msg("configuring the Host\n");

    mem_outw ((mem_inw (IO_CLK_MOD2) & (~(0x4000))),IO_CLK_MOD2); /* disable MS clk.       */
    mem_outw ((mem_inw (IO_CLK_SEL0) | 0x00),IO_CLK_SEL0);        /* MS clk source as PLLB */
    mem_outw ((mem_inw (IO_CLK_DIV2) & (~(0x1F))),IO_CLK_DIV2);   /* dividing factor is 2  */
    mem_outw ((mem_inw (IO_CLK_DIV2) | 0xD),IO_CLK_DIV2);         /* dividing factor is 2  */
    mem_outw ((mem_inw (IO_CLK_MOD2) | 0x4000),IO_CLK_MOD2);      /* Enable MS clk.        */
    mem_outw (MSRST,IO_MEM_STICK_SYS);                            /* MS Reset              */
    while (mem_inw(IO_MEM_STICK_SYS) & 0x8000);                   /* Waiting for reseting the MS controller */

    mem_outw((mem_inw(IO_INTC_EINT1) & 0xFFFC),IO_INTC_EINT1);
    mem_outw ((mem_inw (IO_MEM_STICK_SYS) | MSINTEN),IO_MEM_STICK_SYS); /* Enable interrupts */
    mem_outw ((mem_inw (IO_MEM_STICK_SYS) & (~MSIEN)),IO_MEM_STICK_SYS);/* Enable interrupts */
    mem_outw ((mem_inw (IO_MEM_STICK_SYS) | MSDRQSL),IO_MEM_STICK_SYS); /* Data request interrupts enabled */
    mem_outw((mem_inw(IO_INTC_EINT1) | 0x18),IO_INTC_EINT1);
    mem_outw((mem_inw(IO_INTC_EINT1) | 0x3),IO_INTC_EINT1);
    mem_outw ((mem_inw (IO_MEM_STICK_SYS) | MSINTCLR),IO_MEM_STICK_SYS);/* Clear all interrupts */
    while (mem_inw(IO_MEM_STICK_SYS) & 0x0800);

    mem_outw ((mem_inw (IO_MEM_STICK_SYS) |(0x000F) ),IO_MEM_STICK_SYS);/* Time out value is 7  */
    mem_outw (MSEND,IO_MEM_STICK_ENDIAN);                               /* BigEndian format     */
}

static void msi_check_status(unsigned long data)
{
    u16 status;

    status = gio_get_bitset(GIO_MS_CARDDETECT);
    rmb();
    if(status == 0)
      {
	mem_outw(mem_inw(IO_CLK_MOD2) & (~(0x0800)), IO_CLK_MOD2 ); /* disable MMC clock */
	mem_outw(mem_inw(IO_CLK_MOD2) | (0x4000) , IO_CLK_MOD2 );   /* Enable MS clock   */
	mem_outw (MSMDE,IO_MEM_STICK_MODE);                         /* MS mode           */
      }
    if (gmsi_host)
        ms_detect_change(gmsi_host->ms,20);
}

static struct ms_host_ops msi_ops = {
	.request        = msi_request,
	.set_ios        = msi_set_ios,
};

static int msi_probe(struct device *dev)
{
    struct ms_platform_data *plat = dev->platform_data;
    struct msi_host *host = NULL;
    struct ms_host *ms = NULL;
    int ret;
	
    ms = ms_alloc_host(sizeof(struct msi_host), dev);
    if (!ms) 
      {
	ret = -ENOMEM;
	ms_debug_msg(" ms_alloc_host failed \n");
	goto rel_regions;
      }
	
    host = ms_priv(ms);
    host->cmd = NULL;
    host->clk = clk_get(dev, "MCLK");
    
    ms->read_attb_data = read_attribute_data;
    ms->readdata = read_from_card_ms;         /* used in calling the read system call          */
    ms->writedata = write_to_card;            /* used in calling the write system call         */
    ms->read_blk_data = read_from_card_blk;   /* used in calling the block read before write   */
    ms->write_blk_data = write_to_card_blk;   /* used in calling the block write               */
    ms->update_ms_page = update_ms_page_ms;   /* used in calling the update the pages to write */
    ms->change_clk25m = change_clk25m;
    
    ret = clk_use(host->clk);
    if (ret)
        goto clk_free;
    ret = clk_enable(host->clk);
    if (ret)
        goto clk_unuse;
	
    host->plat = plat;
    host->mclk = clk_get_rate(host->clk);
    host->ms = ms;

    ms->ops = &msi_ops;
    ms->ocr_avail = 1 << 16; /* Has to be done by platform data */
    ms->max_hw_segs = 16;
    ms->max_phys_segs = NR_SG;
    
    /*
     * Since we only have a 16-bit data length register, we must
     * ensure that we don't exceed 2^16-1 bytes in a single request.
     * Choose 64 (512-byte) sectors as the limit.
     */
    ms->max_sectors = 64;
    
    /*
     * Set the maximum segment size.  Since we aren't doing DMA
     * (yet) we are only limited by the data length register.
     */
    ms->max_seg_size = ms->max_sectors << 9;
	
    spin_lock_init(&host->lock);
    ret = request_irq(IRQ_SD_MMC, msi_irq, SA_SHIRQ, DRIVER_NAME " (cmd)", host);
    if (ret)
      {	
	printk ("Interrupt %d may be busy\n",IRQ_SD_MMC);
	free_irq(IRQ_SD_MMC,host);
      }
    
    dev_set_drvdata(dev,ms);
    ms_add_host(ms);
    gmsi_host = host;
    host_configuration();		
    init_sd_ms();
    
    return 0;
	
 clk_unuse:
    clk_unuse(host->clk);
 clk_free:
    clk_put(host->clk);
    
 rel_regions:
    return ret;
}

static int msi_remove(struct device *dev)
{
    struct ms_host *ms = dev_get_drvdata(dev);
	
    plug_count = 0;
    dat3stat_new = 0;
    dev_set_drvdata(dev, NULL);
    
    if (ms) 
      {
	if (timer_inited)
	  {
	    del_timer_sync(&gmsi_host->timer);
	    timer_inited = 0;
	  }
	ms_remove_host(ms);
	ms_free_host(ms);
	free_irq(IRQ_SD_MMC,ms_priv(ms));
      }
    
    return 0;
}

#define msi_suspend	NULL
#define msi_resume	NULL

static struct device_driver msi_driver = { 
	.name		= DRIVER_NAME,
	.bus        = &platform_bus_type,
	.probe		= msi_probe,
	.remove		= msi_remove,
	.suspend	= msi_suspend,
	.resume		= msi_resume,
};


static void start_debounce_timer(void)
{
    struct msi_host *host = gmsi_host;
	
    if (timer_inited)
	mod_timer(&host->timer, jiffies + HZ);
    else
      {
	init_timer(&host->timer);
	host->timer.data = (unsigned long)host;
	host->timer.function = msi_check_status;
	host->timer.expires = jiffies + HZ;
	add_timer(&host->timer);
	timer_inited = 1;
      }
}

static irqreturn_t it_mssd_detect_interrupt(int flags, void * data, struct pt_regs * regs)
{
    printk("MS detect interrupt fired\n");
    start_debounce_timer();

    return IRQ_HANDLED;
}

static void init_sd_ms(void)
{
    int status;
	
#if 0 /* done in mmc driver initial */
    status = request_gio(GIO_MS_CARDDETECT);
    if (status)
        printk("\tUnable to register gio1 %d\n",GIO_MS_CARDDETECT);
    gio_enable_irq(GIO_MS_CARDDETECT,GIO_ANY_EDGE);
#endif
    
    status = request_irq(IRQ_MS_CARDDETECT, it_mssd_detect_interrupt,0,"ms/sd DMA controller",NULL);
    if (status)
		printk("\tUnable to register irq \t\n");
}

static int __init msi_init(void)
{
    int val = 0;

    printk("Configure the MS card controler\n");
    val = driver_register(&msi_driver);

    if(gio_get_bitset(GIO_MS_CARDDETECT)==0)
        start_debounce_timer();
    else
        mem_outw ((mem_inw (IO_CLK_MOD2) & (~(0x4000))),IO_CLK_MOD2);//disable MS clk.
    
    return val;
}

static void __exit msi_exit(void)
{
    plug_count = 0;
    dat3stat_new = 0;

#if 0 /* done in mmc driver */   
    gio_disable_irq(GIO_MS_CARDDETECT);
    unrequest_gio(GIO_MS_CARDDETECT);
#endif
    free_irq(IRQ_GIO8,NULL);
    driver_unregister(&msi_driver);
}

module_init(msi_init);
module_exit(msi_exit);

static struct clk *clk_get(struct device *dev, const char *id)
{
    /* printk(" %s called \n",__FUNCTION__); */
}
static void clk_put(struct clk *clk)
{
    /* printk(" %s called \n",__FUNCTION__); */
}

static int clk_enable(struct clk *clk)
{
    /* printk(" %s called \n",__FUNCTION__); */
    return 0;
}

static int clk_use(struct clk *clk)
{
    /* printk(" %s called \n",__FUNCTION__); */
    return 0;
}
static unsigned long clk_get_rate(struct clk *clk)
{
    /* printk(" %s called \n",__FUNCTION__); */
    return 0;
}
static void clk_unuse(struct clk *clk)
{
    /* printk(" %s called \n",__FUNCTION__); */
}

MODULE_DESCRIPTION("OSD Memory Stick Card Interface driver");
MODULE_LICENSE("GPL");
