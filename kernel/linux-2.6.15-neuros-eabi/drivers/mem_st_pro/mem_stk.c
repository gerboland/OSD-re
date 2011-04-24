/*
 *  linux/drivers/mem_st_pro/mem_stk.c
 *
 *  Copyright (C) 2003-2004 Russell King, All Rights Reserved.
 *  SD support Copyright (C) 2004 Ian Molton, All Rights Reserved.
 *  SD support Copyright (C) 2005 Pierre Ossman, All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/config.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/completion.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/pagemap.h>
#include <linux/err.h>
#include <asm/scatterlist.h>
#include <linux/scatterlist.h>
#include <linux/ms/card.h>
#include <linux/ms/host.h>

#include "mem_stk.h"
#include "mem_stk_debug.h"

MODULE_LICENSE("GPL");

#ifdef CONFIG_MS_DEBUG
#define DBG(x...)	ms_debug_msg(KERN_DEBUG x)
#else
#define DBG(x...)	do { } while (0)
#endif
#define MS_DUO_GET_INT_RETRIES 0xff /*this is to work with the memory-stick-duo where initialization takes lots of retries to pass/fail.*/ 
#define MS_INTERFACE_RETRIES 0x6fff  /*changed to 0x6fff because record to ms card pro may need more retries*/
#define CMD_RETRIES	3
#define ATRB_RETRIES	255

short write_protected (struct ms_host *host);	
unsigned short get_log_block(struct ms_host *host,struct system_information sys_info);

unsigned short swap8(unsigned short reg )
{
    reg = ( ( (reg & 0xFF00)  >> 8) | (reg & 0x00FF) << 8 ) ;
    return reg;
}

unsigned int swap16(unsigned int reg )
{
    reg = ( ( (reg & 0xFFFF0000)  >> 16) | (reg & 0x0000FFFF) << 16 ) ;
    return reg;
}

unsigned int swap32(unsigned int reg )
{
    reg = swap16 (reg);
    reg = (swap8 ((reg >> 16) & 0xFFFF ) << 16 ) | (swap8 ((reg)& 0xFFFF ));
    return reg;
}

/**
 *	ms_request_done - finish processing an MS command
 *	@host: MS host which completed command
 *	@mrq: MS request which completed
 *
 *	MS drivers should call this function when they have completed
 *	their processing of a command.  This should be called before the
 *	data part of the command has completed.
 */                                            
void ms_request_done(struct ms_host *host, struct ms_request *mrq)
{
    struct ms_command *cmd = mrq->cmd;
    int err = mrq->cmd->error;
    if (!err && cmd->retries) 
      {
	ms_debug_msg ("%s %d cmd->retries : %d ERR : %d \n",__FUNCTION__,__LINE__,cmd->retries,err);
	cmd->retries--;
	cmd->error = 0;
	host->ops->request(host, mrq);
      } 
    else if (mrq->done) 
      {
	ms_debug_msg ("%s %d cmd->retries : %d ERR : %d \n",__FUNCTION__,__LINE__,cmd->retries,err);
	mrq->done(mrq);
      }
}
EXPORT_SYMBOL(ms_request_done);

/**
 *	ms_start_request - start a command on a host
 *	@host: MS host to start command on
 *	@mrq: MS request to start
 *
 *	Queue a command on the specified host.  We expect the
 *	caller to be holding the host lock with interrupts disabled.
 */
void ms_start_request(struct ms_host *host, struct ms_request *mrq)
{
    WARN_ON(host->card_busy == NULL);
	
    mrq->cmd->error = 0;
    mrq->cmd->mrq = mrq;

    if ((mrq->data) && (mrq->cmd->flags == DATA_RW_CMD )) 
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
    host->ops->request(host, mrq);
}
EXPORT_SYMBOL(ms_start_request);

static void ms_wait_done(struct ms_request *mrq)
{
    complete(mrq->done_data);
}

int ms_wait_for_req(struct ms_host *host, struct ms_request *mrq)
{
    DECLARE_COMPLETION(complete);
    mrq->done_data = &complete;
    mrq->done = ms_wait_done;
    
    ms_start_request(host, mrq);
    wait_for_completion(&complete);	
    
    return 0;
}
EXPORT_SYMBOL(ms_wait_for_req);

/**
 *	ms_wait_for_cmd - start a command and wait for completion
 *	@host: MS host to start command
 *	@cmd: MS command to start
 *	@retries: maximum number of retries
 *
 *	Start a new MS command for a host, and wait for the command
 *	to complete.  Return any error that occurred while the command
 *	was executing.  Do not attempt to parse the response.
 */
int ms_wait_for_cmd(struct ms_host *host, struct ms_command *cmd, int retries)
{
    struct ms_request mrq;

    BUG_ON(host->card_busy == NULL);

    memset(&mrq, 0, sizeof(struct ms_request));
    cmd->retries = retries;
    mrq.cmd = cmd;
    cmd->data = NULL;
    ms_wait_for_req(host, &mrq);
    
    return cmd->error;
}
EXPORT_SYMBOL(ms_wait_for_cmd);

/**
 *	__ms_claim_host - exclusively claim a host
 *	@host: ms host to claim
 *	@card: ms card to claim host for
 *
 *	Claim a host for a set of operations.  If a valid card
 *	is passed and this wasn't the last card selected, select
 *	the card before returning.
 *
 *	Note: you should use ms_card_claim_host or ms_claim_host.
 */
int __ms_claim_host(struct ms_host *host, struct ms_card *card)
{
    DECLARE_WAITQUEUE(wait, current);
    unsigned long flags;
    int err = 0;
    unsigned char arg[8];

    add_wait_queue(&host->wq, &wait);
    spin_lock_irqsave(&host->lock, flags);
    while (1) 
      {
	set_current_state(TASK_UNINTERRUPTIBLE);
	if (host->card_busy == NULL)
	    break;
	spin_unlock_irqrestore(&host->lock, flags);
	schedule();
	spin_lock_irqsave(&host->lock, flags);
      }
    set_current_state(TASK_RUNNING);
    host->card_busy = card;
    spin_unlock_irqrestore(&host->lock, flags);
    remove_wait_queue(&host->wq, &wait);

    if (card != (void *)-1)
      {
	if ( get_int(host,arg,NORM_COMP) == NO_CARD)
	  {
	    ms_debug_msg (">>>>>>>>>setting card dead<<<<<<<<<<\n");
	    ms_card_set_dead(card);
	    err = 1;
	  }
      }
    return err;
}
EXPORT_SYMBOL(__ms_claim_host);

/**
 *	ms_release_host - release a host
 *	@host: ms host to release
 *
 *	Release a MS host, allowing others to claim the host
 *	for their operations.
 */
void ms_release_host(struct ms_host *host)
{
    unsigned long flags;

    BUG_ON(host->card_busy == NULL);

    spin_lock_irqsave(&host->lock, flags);
    if (host->card_busy)
        host->card_busy = NULL;
    spin_unlock_irqrestore(&host->lock, flags);
	
    wake_up(&host->wq);
}
EXPORT_SYMBOL(ms_release_host);

/*
 * Locate a MS card on this MS host given a raw CID.
 */
static struct ms_card *ms_find_card(struct ms_host *host, u8 *sys_info)
{
    struct ms_card *card;
    list_for_each_entry(card, &host->cards, node) 
      {
	if (memcmp(&card->sys_info, sys_info, sizeof(card->sys_info)) == 0)
	    return card;
      }
    return NULL;
}

/*
 * Allocate a new MS card
 */
static struct ms_card *ms_alloc_card(struct ms_host *host, u8 *sys_info, unsigned int *frca)
{
    struct ms_card *card;

    card = kmalloc(sizeof(struct ms_card), GFP_KERNEL);
    if (!card)
        return ERR_PTR(-ENOMEM);
    ms_init_card(card, host);
    memcpy(&card->sys_info,sys_info, sizeof(card->sys_info));
    return card;
}

unsigned short is_command_end(unsigned char reg)
{
    unsigned char ced = (reg & 0x80);
    unsigned char err = (reg & 0x40);
    unsigned char breq = (reg & 0x20);
    unsigned char cmdnk = (reg & 0x01);
    
    if (cmdnk)
      {
	printk("command not excuted\n");
	return CMD_NOT_EXE;
      }
	
    if (ced && err )
      {
	printk("%s: command error termination\n", __BASE_FILE__);
	return CMD_ERR_TER;
      }
    if (err && breq)
      {
	printk("%s: data request error\n", __BASE_FILE__);
	return DATA_REQ_ERR;
      }
    if (!ced && !breq)
        return CMD_EXE;
    if (!err && breq)
        return NORM_DATA_TRANS;
    if (ced && !err)
        return NORM_COMP;

    printk("%s: WARNING: invalid status", __BASE_FILE__);
    return 0XFF;
}	
EXPORT_SYMBOL(is_command_end);

/*Get int values from the memory stick register*/
unsigned short get_int(struct ms_host *host,char *arg,unsigned short status)
{
	struct ms_command cmd;
	int i=0;
	unsigned int retry=0;
	if (host->mode == MS_MODE_MS_PRO)
		retry=MS_INTERFACE_RETRIES;
	else if (host->mode == MS_MODE_MS)
		retry=MS_DUO_GET_INT_RETRIES;
	do
	{
		cmd.opcode = GET_INT;
		cmd.arg = arg;
		cmd.size = 1;
		cmd.flags = REG_RW_CMD;
		ms_wait_for_cmd(host, &cmd,0);
		if ((cmd.error == MEM_STICK_TIME_OUT) || (cmd.error == MEM_STICK_CRC_ERROR))
			return NO_CARD;
		if (is_command_end(cmd.resp[0]) == status)
			return SUCCESS;
	}while (i++ < retry);

	ms_debug_msg("cmd.resp[0] : %x\n",cmd.resp[0]);
	ms_debug_msg("cmd.resp[1] : %x\n",cmd.resp[1]);
	ms_debug_msg("cmd.resp[2] : %x\n",cmd.resp[2]);
	ms_debug_msg("cmd.resp[3] : %x\n",cmd.resp[3]);
	ms_debug_msg("cmd.resp[4] : %x\n",cmd.resp[4]);
	ms_debug_msg("cmd.resp[5] : %x\n",cmd.resp[5]);
	ms_debug_msg("cmd.resp[6] : %x\n",cmd.resp[6]);
	ms_debug_msg("cmd.resp[7] : %x\n",cmd.resp[7]);

	return FAILURE;
}
EXPORT_SYMBOL(get_int);

/*set the register address and count value in the card 
to read/write from the card*/

unsigned short set_card_regadd(struct ms_host *host,char *arg)
{
    struct ms_command cmd;
    int i;
    i = 0;
	
    cmd.opcode = SET_RW_REG_ADRS;
    cmd.arg = arg;
    cmd.size = 4;
    cmd.flags = REG_RW_CMD;
    
    ms_wait_for_cmd(host, &cmd,0);
    if ((cmd.error == MEM_STICK_TIME_OUT) || (cmd.error == MEM_STICK_CRC_ERROR))
        return NO_CARD;
    
    return SUCCESS;
}
EXPORT_SYMBOL(set_card_regadd);

/*Get register values from the memory stick register*/
static unsigned short get_card_regiters(struct ms_host *host,char *arg)
{
    struct ms_command cmd;

    cmd.opcode = READ_REG;
    cmd.arg = arg;
    cmd.size = 8;
    cmd.flags = REG_RW_CMD;
	
    ms_wait_for_cmd(host, &cmd,0);
	
    arg[0] = cmd.resp[0];
    arg[1] = cmd.resp[1];
    arg[2] = cmd.resp[2];
    arg[3] = cmd.resp[3];
    arg[4] = cmd.resp[4];
    arg[5] = cmd.resp[5];
    arg[6] = cmd.resp[6];
    arg[7] = cmd.resp[7];	
    
    if ((cmd.error == MEM_STICK_TIME_OUT) || (cmd.error == MEM_STICK_CRC_ERROR))
        return NO_CARD;

    return SUCCESS;
}
EXPORT_SYMBOL(get_card_regiters);

unsigned short get_blk_number(struct ms_host *host,int address,unsigned short sys_bk_size)
{
    short blk_add = 0;
    address = address/(sys_bk_size * 2);
    blk_add = host->log_address[address];
    return blk_add;
}	
EXPORT_SYMBOL(get_blk_number);

unsigned short set_card_regiters(struct ms_host *host, short size,
				 char mode, int address,
				 unsigned short sys_bk_size, char address_mode,
				 char cmd_mode, char over_write_flag,
				 char manage_flag,short log_add,short cmd_size)
{
    struct ms_command cmd;
    unsigned char buf[16];
    unsigned char page_add = 0x00;
    int blk_add;	
    short logic_add = 0;
	
    if ((host->mode == MS_MODE_MS) && (sys_bk_size != 0))
      {
	page_add = address - ((address/(sys_bk_size * 2)) * (sys_bk_size * 2));
	address = address/(sys_bk_size * 2);
		
	if (address_mode  == USER_DATA)
	  {
	    logic_add = address;
	    blk_add = host->log_address[address];
	    address = blk_add;			
	    ms_debug_msg("LOGICAL ADDRESS:%x\n",logic_add);
	    ms_debug_msg("PAGE ADDRESS : %x\t",page_add);
	    ms_debug_msg("BLOCK ADDRESS : %x\n",address);
	  }
      }
	
    if (mode == 0x80)
        host->ios.bus_width = SERIAL;
    else if (mode == 0x00)			
        host->ios.bus_width = PARALLEL;
	
    if (host->mode == MS_MODE_MS_PRO)
      {	
	  //size = 1;
	  buf[0] = mode;
	  buf[1] = (size & 0xFF00) >> 8;
	  buf[2] = (size & 0xFF);
	  buf[3] = (address & 0xFF000000) >> 24;
	  buf[4] = (address & 0x00FF0000) >> 16;
	  buf[5] = (address & 0x0000FF00) >> 8;
	  buf[6] = (address & 0x000000FF);
	  buf[7] = 0x00;
      }
    else if (host->mode == MS_MODE_MS)
      {	
	if (host->ios.bus_width == PARALLEL)
	  {		
	    ms_debug_msg("Setting Parallel\n");
	    buf[0] = 0x88;
	  }
	else if (host->ios.bus_width == SERIAL)
	  {
	    ms_debug_msg("Setting Serial\n");
	    buf[0] = 0x80;
	  }
	buf[1] = (address & 0xFF0000) >> 16;
	buf[2] = (address & 0x00FF00) >> 8;
	buf[3] = (address & 0x0000FF);
	buf[4] = cmd_mode;  /* mode access; */
	buf[5] = (page_add);
	buf[6] = over_write_flag;
	buf[7] = manage_flag;
	buf[8] = (logic_add & 0xFF00) >> 8;
	buf[9] = (logic_add & 0xFF);
      }
	
    cmd.opcode = WRITE_REG;
    cmd.arg = buf;
    cmd.size = cmd_size;
    cmd.flags = REG_RW_CMD;
    ms_wait_for_cmd(host, &cmd,0);
    host->ops->set_ios(host,&host->ios);
    
    if ((cmd.error == MEM_STICK_TIME_OUT) || (cmd.error == MEM_STICK_CRC_ERROR))
        return FAILURE;
    
    return SUCCESS;
}
EXPORT_SYMBOL(set_card_regiters);

unsigned short set_cmd(struct ms_host *host,unsigned short command,
		       unsigned short flag,unsigned short status)
{
    struct ms_command cmd;
    unsigned char buf[8];
    short status_in ;
	
    buf[0] = (command & 0xFF00) >> 8;	
    buf[1] = (command & 0xFF);
	
    cmd.opcode = SET_CMD;
    cmd.arg = buf;
    cmd.size = 1;
    cmd.flags = flag;
    
    ms_wait_for_cmd(host, &cmd,0);
    if ((cmd.error == MEM_STICK_TIME_OUT) || (cmd.error == MEM_STICK_CRC_ERROR))
      {
	ms_debug_msg("NO RESPONSE\n");	
	return FAILURE;
      }
	
    status_in = get_int(host,buf,status);
    if (status_in == NO_CARD)
      {
	ms_debug_msg ("MAY BE NO CARD WHEN GET_INT : %d\n",__LINE__);			
	return FAILURE;
      }
    if (status_in == FAILURE)
      {
	ms_debug_msg("MAY BE NOT  IN THE SPECIFIED STATE : %d\n",__LINE__);
	return NOT_DATA_STATE;
      }

    return SUCCESS;
}
EXPORT_SYMBOL(set_cmd);

int ms_wait_for_read_data(struct ms_host *host, struct ms_request *mrq)
{	
    char buf[8];
    int retry = 0;

    host->readdata(host,mrq);	
    while (1)
      {
	if (get_int(host,(char*)buf,NORM_COMP) == SUCCESS) break;
	if (retry++ > MS_INTERFACE_RETRIES) return FAILURE;
      }
    if (host->mode == MS_MODE_MS)
      {	
	if (get_int(host,buf,NORM_COMP) == FAILURE)
	  {
	    ms_debug_msg("SETING BLOCK_END COMMAND %d\n",__LINE__);
	    set_cmd(host,BLOCK_END,REG_RW_CMD,NORM_COMP);
	    ms_debug_msg("SETING BUFFER CLEAR COMMAND %d\n",__LINE__);
	    set_cmd(host,CLEAR_BUF,REG_RW_CMD,NORM_COMP);
	    return FAILURE;	
	  }
      }
#if 0
    else if (host->mode == MS_MODE_MS_PRO)
      {
	if (set_cmd(host,STOP,REG_RW_CMD,NORM_COMP) == FAILURE)
	    return FAILURE;
      }
#endif
    return SUCCESS;	
}
EXPORT_SYMBOL(ms_wait_for_read_data);

int ms_wait_for_write_data(struct ms_host *host, struct ms_request *mrq)
{
    char buf[8];
    int retry = 0;

    host->writedata(host,mrq);
    while (1)
      {
	if (get_int(host,(char*)buf,NORM_COMP) == SUCCESS) break;
	if (retry++ >  MS_INTERFACE_RETRIES) return FAILURE;
      }
    if (host->mode == MS_MODE_MS)
      {	
	if (get_int(host,buf,NORM_COMP) == FAILURE)
	  {
	    ms_debug_msg("SETING BLOCK_END COMMAND %d\n",__LINE__);
	    set_cmd(host,BLOCK_END,REG_RW_CMD,NORM_COMP);
	    ms_debug_msg("SETING BUFFER CLEAR COMMAND %d\n",__LINE__);
	    set_cmd(host,CLEAR_BUF,REG_RW_CMD,NORM_COMP);
	    return FAILURE;	
	  }
      }
#if 0
    else if (host->mode == MS_MODE_MS_PRO)
      {
	if (set_cmd(host,STOP,REG_RW_CMD,NORM_COMP)== FAILURE)
	    return FAILURE;
      }
#endif
    return SUCCESS;
}
EXPORT_SYMBOL(ms_wait_for_write_data);

int ms_wait_for_read_data_blk(struct ms_host *host, struct ms_request *mrq,
			      int data_size,unsigned char* data)
{	
    char buf[8];

    memset ((void*)buf,0,sizeof(buf));

    host->read_blk_data(data,data_size,host,mrq);	
    if (host->mode == MS_MODE_MS)
      {	
	if (get_int(host,buf,NORM_COMP) == FAILURE)
	  {
	    ms_debug_msg("SETING BLOCK_END COMMAND %d\n",__LINE__);
	    set_cmd(host,BLOCK_END,REG_RW_CMD,NORM_COMP);
	    ms_debug_msg("SETING BUFFER CLEAR COMMAND %d\n",__LINE__);
	    set_cmd(host,CLEAR_BUF,REG_RW_CMD,NORM_COMP);
	    return FAILURE;	
	  }
      }
    return SUCCESS;
}
EXPORT_SYMBOL(ms_wait_for_read_data_blk);

int ms_wait_for_write_data_blk(struct ms_host *host, struct ms_request *mrq,
			       int data_size,unsigned char* data)
{
    char buf[8];

    memset ((void*)buf,0,sizeof(buf));

    host->write_blk_data(data,data_size,host,mrq);
    if (get_int(host,buf,NORM_COMP) == FAILURE)
      {
	ms_debug_msg("SETING BLOCK_END COMMAND %d\n",__LINE__);
	set_cmd(host,BLOCK_END,REG_RW_CMD,NORM_COMP);
	ms_debug_msg("SETING BUFFER CLEAR COMMAND %d\n",__LINE__);
	set_cmd(host,CLEAR_BUF,REG_RW_CMD,NORM_COMP);
	return FAILURE;
      }
    return SUCCESS;
}
EXPORT_SYMBOL(ms_wait_for_write_data_blk);


void update_ms_page(struct ms_host *host,short page_add,short block_size, 
		    struct ms_request *mrq,unsigned char*cur_blk_data)
{
        host->update_ms_page((struct ms_host *)host,page_add,block_size,
			     (struct ms_request*)mrq,cur_blk_data);
}
EXPORT_SYMBOL(update_ms_page);

/*
 * Apply power to the MS stack.
 */
static void ms_power_up(struct ms_host *host)
{
    host->ios.power_mode = MS_POWER_ON;
    host->ios.bus_width = SERIAL;
    host->ios.change_clk = 2;
    host->ops->set_ios(host, &host->ios);
}

static void ms_power_off(struct ms_host *host)
{
    host->ios.bus_width = SERIAL;
    host->ios.power_mode = MS_POWER_OFF;
    host->ios.change_clk = 2;
    host->ops->set_ios(host, &host->ios);
}

/*
 * If this command times out, it is not an error; there are no further cards
 * to be discovered.  Add new cards to the list.
 *
 * Create a ms_card entry for each discovered card, assigning
 * it an RCA, and save the raw CID for decoding later.
 */
static void ms_discover_cards(struct ms_host *host)
{
    struct ms_card *card;
    unsigned int first_rca = 1, err;

    struct system_information *sys_info_temp;
    struct system_information sys_info;
    struct attribute_info atrb_info;
    struct entry_table entry_tab[NO_OF_ENTITY];
    unsigned char model_name[0x30];
    unsigned char arg[8];
    
    int i,j,k = 0; /* jchen: k use for counting the read attrib ms-card */
    unsigned short status;
    unsigned char *buf_atrb = NULL;
    int boot_block = 0;	
    int block_add = 0;

    buf_atrb = (unsigned char *)kmalloc(1024,GFP_KERNEL);
    memset ((void*)buf_atrb,0,1024);
    
    if (host->mode == MS_MODE_MS)
      {
	do{		
	    ms_debug_msg("block add : %x\n",block_add);	
	    if (block_add > 12)
	      {
		ms_debug_msg("No Boot block : %d\n",__LINE__);
		return;
	      }

	    ms_debug_msg("SETING CARD REGISTER VALUES %d\n",__LINE__);
	    if (set_card_regiters(host,0x0,SERIAL,block_add,0,0,0x20,0,0,0,8) == FAILURE)
	      {
		kfree(buf_atrb);
		return;
	      }
		
	    ms_debug_msg("SETING BLOCK_READ COMMAND %d\n",__LINE__);
	    if ((status = set_cmd(host,BLOCK_READ,REG_RW_CMD,NORM_DATA_TRANS)) == FAILURE)
	      {
		kfree(buf_atrb);
		return;
	      }
	    if (status == NOT_DATA_STATE)
	      {
		ms_debug_msg("Not in the DATA STATE %d\n",__LINE__);
		kfree(buf_atrb);
		return;
	      }
	    if (status == SUCCESS)
	      {
		host->read_attb_data(host,buf_atrb,0x02);
		
		if ( *((unsigned short*)buf_atrb) == 0x0100)
		  {
		    ms_debug_msg("This is a Boot Block\n");
		    sys_info_temp = (struct system_information *)((unsigned long)buf_atrb + (unsigned long)0x1A0);
		    sys_info_temp->block_size = swap8(sys_info_temp->block_size);
		    sys_info_temp->total_block = swap8(sys_info_temp->total_block);
		    sys_info_temp->user_area_block = swap8(sys_info_temp->user_area_block);
		    sys_info_temp->page_size = swap8(sys_info_temp->page_size);
		    sys_info_temp->date.year = swap8(sys_info_temp->date.year);
		    sys_info_temp->implem_capacity = swap8(sys_info_temp->implem_capacity);
		    sys_info_temp->controller_no = swap8(sys_info_temp->controller_no);
		    sys_info_temp->controller_func = swap8(sys_info_temp->controller_func);
		    sys_info_temp->start_sector = swap8(sys_info_temp->start_sector);
		    sys_info_temp->unit_size = swap8(sys_info_temp->unit_size);
		    sys_info_temp->controller_code = swap8(sys_info_temp->controller_code);
					
		    sys_info = *sys_info_temp;
#if 0
		    ms_debug_msg ("DEVICE INFORMATION\n");
		    printk ("block_size : %x\n",sys_info.block_size);
		    printk ("Total block : %x\n",sys_info.total_block);
		    printk ("page_size : %x\n",sys_info.page_size);
		    printk ("user_area_block : %x\n",sys_info.user_area_block);
		    printk ("interface_type : %x\n",sys_info.interface_type);
		    printk ("format_type : %x\n",sys_info.format_type);
		    printk ("device_type : %x\n",sys_info.device_type);
		    printk ("Size of user area: %d KB\n",(sys_info.block_size * sys_info.user_area_block));
#endif
		    boot_block = 1;
		  }
#if 0 
		else
		  {
		    printk("No Boot block %d\n",__LINE__);
		    kfree(buf_atrb);
		    printk ("%s %d\n",__FUNCTION__,__LINE__);
		  }
#endif
		ms_debug_msg("SENDING GET_INT COMMAND %d\n",__LINE__);
		if (get_int(host,arg,NORM_COMP) == FAILURE)
		  {
		    ms_debug_msg("SETING BLOCK_END COMMAND %d\n",__LINE__);
		    status = set_cmd(host,BLOCK_END,REG_RW_CMD,NORM_COMP);
		    
		    ms_debug_msg("SETING BLOCK_CLEAR COMMAND %d\n",__LINE__);
		    status = set_cmd(host,CLEAR_BUF,REG_RW_CMD,NORM_COMP);
		  }
		if (boot_block == 1)
		  {
		    get_log_block(host,sys_info);
		    break;
		  }
	      }
	    else
	      {	
		ms_debug_msg("FAILURE %d\n",__LINE__);
		kfree(buf_atrb);
		return;
	      }
	    block_add += 1;
	}while(1);
      }
    if (host->mode == MS_MODE_MS_PRO)
      {	
	ms_debug_msg("SETING CARD REGISTER VALUES %d\n",__LINE__);
	if (set_card_regiters(host,0x02,SERIAL,0x0,0,0,0x20,0,0,0,8))
	    return;	
		
	ms_debug_msg ("SETING READ_ATRB COMMAND %d\n",__LINE__);
	do{
		status = set_cmd(host,READ_ATRB,REG_RW_CMD,NORM_DATA_TRANS);
		/* read atrb could be busy here, retry few time to make sure
		 * we prevent the busy issue.
		 */
		if(k > ATRB_RETRIES)
		{
			printk(KERN_ERR "Failure to read the unknown MS-card attribute, please format your MS-card.\n") ;
			/* free the atrb buffer & return to prevent freeze in kernel. */
			kfree(buf_atrb);
			return;
		}
		k++ ;
		udelay(10) ;
	}while(status == FAILURE) ;
	//status = set_cmd(host,READ_ATRB,REG_RW_CMD,NORM_DATA_TRANS);
	
	i = 0;
	do{
	    if (i++ > 255)
	      {
		ms_debug_msg("SETTING FAILURE %d\n",__LINE__);
		status = FAILURE;
		break;
	      }
	    else
	      {
		if (get_int(host,arg,NORM_DATA_TRANS) == SUCCESS)
		  {
		    status = SUCCESS;	
		    break;
		  }
	      }	
	}while(1);
	
	if (status == SUCCESS)
	  {
	    host->read_attb_data(host,buf_atrb,0x02);
	    if (swap8(*((unsigned short *)(buf_atrb))) == 0xA5C3)
	      {
		atrb_info.done = 1;
		atrb_info.sig_code = swap8(*((unsigned short *)(buf_atrb)));
		atrb_info.version = swap8(*((unsigned short *)(buf_atrb + 2)));
		atrb_info.no_of_entity = (unsigned short)(*((buf_atrb + 4)));
	      }
	    else
		ms_debug_msg("Attribute signature code is wrong\n");

	    if (atrb_info.no_of_entity > NO_OF_ENTITY)
		ms_debug_msg("Number of Entity is too large");
	    
	    for (j = 0;j < atrb_info.no_of_entity; j++)
	      {
		entry_tab[j].add =(unsigned char *) swap32(*((unsigned int*)(buf_atrb + 16 + (j*ENTRY_SIZE))));
		entry_tab[j].size =swap32 (*((unsigned int *)(buf_atrb + 16 + (j*ENTRY_SIZE)+4)));
		entry_tab[j].info_id =*((unsigned char *)(buf_atrb + 16 + (j*ENTRY_SIZE)+8));
	      }
	    
	    for (j=0;j<atrb_info.no_of_entity;j++)
	      {
		if ( ((entry_tab[j].info_id) == 0x10) 
		     && ((unsigned int)(entry_tab[j].add) <= 1024)
		     && ((unsigned int)(entry_tab[j].add) >= 0x1A0))
		  {
		    sys_info_temp = (struct system_information *)(buf_atrb + (unsigned int)(entry_tab[j].add));
		    sys_info_temp->block_size = swap8(sys_info_temp->block_size);
		    sys_info_temp->total_block = swap8(sys_info_temp->total_block);
		    sys_info_temp->user_area_block = swap8(sys_info_temp->user_area_block);
		    sys_info_temp->page_size = swap8(sys_info_temp->page_size);
		    sys_info_temp->date.year = swap8(sys_info_temp->date.year);
		    sys_info_temp->start_sector = swap8(sys_info_temp->start_sector);
		    sys_info_temp->unit_size = swap8(sys_info_temp->unit_size);
		    sys_info_temp->controller_code = swap8(sys_info_temp->controller_code);
					
		    sys_info = *sys_info_temp;
#if 0
		    printk ("DEVICE INFORMATION\n");
		    printk ("unit_size : %x\n",sys_info.unit_size);
		    printk ("ms_sub_class : %x\n",sys_info.ms_sub_class);
		    printk ("interface_type : %x\n",sys_info.interface_type);
		    printk ("format_type : %x\n",sys_info.format_type);
		    printk ("device_type : %x\n",sys_info.device_type);
		    printk ("Size of user area: %d MB\n",(sys_info.unit_size * sys_info.block_size * sys_info.user_area_block)/(1024*1024));
#endif
		  }/* for IF */
		
		if ( ((entry_tab[j].info_id) == 0x15) 
		     && ((unsigned int)(entry_tab[j].add) <= 1024) 
		     && ((unsigned int)(entry_tab[j].add) >= 0x1A0))
		  {
		    for (i=0;i<0x30;i++)
		      {
			model_name[i] = (unsigned char)*((unsigned char *)buf_atrb + (unsigned int)(entry_tab[j].add) + i);
			ms_debug_msg("%c ",model_name[i]);						
		      }
		  }
	      }
	    kfree(buf_atrb);
	    
	  }
	else
	  {
	    kfree(buf_atrb);
	    return;
	  }
      }
    card = (struct ms_card *)ms_find_card(host,(unsigned char *)&sys_info);
    if (!card) 
      {	
	card = ms_alloc_card(host,(unsigned char *)&sys_info, &first_rca);
	if (IS_ERR(card)) 
	  {
	    err = PTR_ERR(card);
	    return;	
	  }
	list_add(&card->node, &host->cards);
      }
    card->state &= ~MS_STATE_DEAD;
	
    memset(arg,0,sizeof(arg));
    arg[0] = 0x02;
    arg[1] = 0x08;
    arg[2] = 0x10;
    arg[3] = 0x08;
    
    ms_debug_msg("SETING CARD REGISTER VALUES %d\n",__LINE__);
    if (set_card_regadd(host,arg) != SUCCESS)
        ms_card_set_dead(card);
    else
      {
	short status;
		
	if (host->mode == MS_MODE_MS_PRO) 
	    ms_card_set_ms_pro(card);
	else
	  { 
	    if (host->mode == MS_MODE_MS) 
	        ms_card_set_ms(card);
	  }
	
	status = write_protected(host);
	if (status == READ_WRITE) 
	  {
	    ms_debug_msg(KERN_WARNING "%s: host does not "
			 "support reading read-only "
			 "switch. assuming write-enable.\n",
			 ms_hostname(host));
	  }
	else if (status == READ_ONLY)
	    ms_card_set_readonly(card);
	else if (status == NO_CARD)
	    ms_card_set_dead(card);
      }
}

unsigned short get_log_block(struct ms_host *host,struct system_information sys_info)
{		
    int blk_address = 0;
    char arg[8];
    short status;
    short boot_blocks[2];
    short b_count = 0;
    int total = 0;
    
    ms_debug_msg("Entering %s\n",__FUNCTION__);
	
    if (host->log_address == NULL)
        host->log_address = (unsigned short *)kmalloc((sys_info.total_block*2),GFP_KERNEL);
    else
      {
	kfree(host->log_address);
	host->log_address = (unsigned short *)kmalloc((sys_info.total_block*2),GFP_KERNEL);
      }
    memset ((unsigned short*)host->log_address,0xAA,sys_info.total_block);
    memset(arg,0,sizeof(arg));
    
    arg[0] = 0x16;
    arg[1] = 0x08;
    arg[2] = 0x10;
    arg[3] = 0x08;

    ms_debug_msg("SETING CARD REGISTER VALUES %d\n",__LINE__);
    if (set_card_regadd(host,arg))
        return 0;
	
    blk_address = 0;
    while(blk_address < (sys_info.total_block * sys_info.block_size * 2))
      {
	if (set_card_regiters(host,0x02,SERIAL,blk_address,sys_info.block_size,0,0x40,0,0,0,8))
	    ms_debug_msg("set_card_regiters Error %d\n",__LINE__);
		
	status = set_cmd(host,BLOCK_READ,REG_RW_CMD,NORM_COMP);
	if (status == FAILURE)
	    return 0;
		
	if (get_card_regiters(host,arg))
	    return 0;
		
	if (!(arg[1] & 0x04))
	  {
	    boot_blocks[b_count] = blk_address;
	    goto skip_update;
	  } 
#if 1 
	if ((unsigned int)((arg[2]  << 8)|(arg[3])) == 0xFFFF)
	  goto skip_update;
#endif
	if ((arg[0] != 0xF8) && ( (unsigned int)((arg[2]  << 8)|(arg[3])) == 0xFFFF) ) 
	  {
	    ms_debug_msg("NOT A VALID ADDRESS\n");
	    ms_debug_msg("BLOCK ADDRESS : %x\n",(unsigned short)(blk_address / (sys_info.block_size * 2)));
	    goto skip_update;
	  }
	if ( (arg[0]&0x60) && (arg[1] & 0x04) && (arg[1] & 0x08))
	  {
	    if (blk_address == 0)	
	        host->log_address[(unsigned int)((arg[2]  << 8)|(arg[3]))] = 0;
	    else
	        host->log_address[(unsigned int)((arg[2]  << 8)|(arg[3]))] = (unsigned short)(blk_address / (sys_info.block_size * 2));
	    total++;
	  }
      skip_update:
	memset(arg,0,sizeof(arg));
	
	if (get_int(host,arg,NORM_COMP) == FAILURE)
	  {
	    status = set_cmd(host,BLOCK_END,REG_RW_CMD,NORM_COMP);
	    status = set_cmd(host,CLEAR_BUF,REG_RW_CMD,NORM_COMP);
	  }
	blk_address = blk_address + sys_info.block_size * 2;
      }
	
    ms_debug_msg("TOTAL USEFUL BLOCKS :%x\n",total);
    return 1;
}

short write_protected (struct ms_host *host)
{	
    char arg[8];	
    memset(arg,0,sizeof(arg));
	
    if (get_card_regiters(host,arg) == NO_CARD)
        return NO_CARD;
	
    if (arg[0]&0x1)
        return READ_ONLY;			
    else
        return READ_WRITE;
}

/*
 * Check whether cards we already know about are still present.
 * We do this by requesting status, and checking whether a card
 * responds.
 *
 * A request for status does not cause a state change in data
 * transfer mode.
 */
static void ms_check_cards(struct ms_host *host)
{
    struct list_head *l, *n;
    char arg[8];
	
    list_for_each_safe(l, n, &host->cards) 
      {
	struct ms_card *card = ms_list_to_card(l);
	
	if ( get_int(host,arg,NORM_COMP) != NO_CARD)
	    continue;
	ms_debug_msg("**********************setting card dead***************\n");
	ms_card_set_dead(card);
      }
}

static void ms_setup(struct ms_host *host)
{
  if (host->ios.power_mode != MS_POWER_ON) 
    { /* FINDING MS or MSPRO */
      char arg[8];
      
      ms_debug_msg("SENDING INT COMMAND\n");
      host->mode = MS_MODE_MS;
      ms_power_up(host);

      /*For reading at initial time*/
      memset(arg,0,sizeof(arg));
      arg[0] = 0x02;
      arg[1] = 0x08;
      arg[2] = 0x10;	
      arg[3] = 0x08;
      
      ms_debug_msg("SETING CARD REGISTER VALUES %d\n",__LINE__);
      if (set_card_regadd(host,arg) == NO_CARD)
	{
	  ms_debug_msg ("MAY BE NO CARD %d\n",__LINE__);
	  return;
	}
		
      ms_debug_msg("GETING CARD REGISTER VALUES %d\n",__LINE__);
      if (get_card_regiters(host,arg) == NO_CARD)
	{
	  ms_debug_msg("MAY BE NO CARD %d\n",__LINE__);
	  return;
	}
      if (arg[2] == 0x1)
	{	
	  printk("Memory stick pro\n");
	  host->mode = MS_MODE_MS_PRO;
	  host->ios.bus_width = SERIAL;
	  host->ios.change_clk = 1;
	  host->ops->set_ios(host,&host->ios);
	  host->ios.change_clk = 0;
	  
	}
      else
	{
	  printk("Memory stick\n");	
	  host->mode = MS_MODE_MS;
	  host->ios.bus_width = SERIAL;
	  host->ios.change_clk = 2;
	  host->ops->set_ios(host,&host->ios);
	  host->ios.change_clk = 0;
	}
    }

  ms_discover_cards(host);
  ms_debug_msg("leaving %s\n",__FUNCTION__);
}

/**
 *	ms_detect_change - process change of state on a MS socket
 *	@host: host which changed state.
 *	@delay: optional delay to wait before detection (jiffies)
 *
 *	All we know is that card(s) have been inserted or removed
 *	from the socket(s).  We don't know which socket or cards.
 */
void ms_detect_change(struct ms_host *host, unsigned long delay)
{	
    if (delay)
        schedule_delayed_work(&host->detect, delay);
    else
        schedule_work(&host->detect);
}
EXPORT_SYMBOL(ms_detect_change);

static void ms_rescan(void *data)
{
    struct ms_host *host = data;
    struct list_head *l, *n;

    ms_debug_msg("%s %d\n",__FUNCTION__,__LINE__);	
    ms_claim_host(host);
    
    ms_debug_msg("host->ios.power_mode : %x\n",host->ios.power_mode);

    if (host->ios.power_mode == MS_POWER_ON)
        ms_check_cards(host);	
		
    ms_setup(host);
    ms_release_host(host);

    list_for_each_safe(l, n, &host->cards) 
      {
	struct ms_card *card = ms_list_to_card(l);
	/*
	 * If this is a new and good card, register it.
	 */
	if (!ms_card_present(card) && !ms_card_dead(card)) 
	  {
	    if (ms_register_card(card))
		ms_card_set_dead(card);
	    else
	        ms_card_set_present(card);
	  }
	/*
	 * If this card is dead, destroy it.
	 */
	if (ms_card_dead(card)) 
	  {
	    mdelay(10);
	    list_del(&card->node);
	    ms_remove_card(card);
	  }
      }
    if (list_empty(&host->cards))
        ms_power_off(host);
}

/**
 *	ms_alloc_host - initialise the per-host structure.
 *	@extra: sizeof private data structure
 *	@dev: pointer to host device model structure
 *
 *	Initialise the per-host structure.
 */
struct ms_host *ms_alloc_host(int extra, struct device *dev)
{
    struct ms_host *host;

    host = ms_alloc_host_sysfs(extra, dev);
    if (host) 
      {
	spin_lock_init(&host->lock);
	init_waitqueue_head(&host->wq);
	INIT_LIST_HEAD(&host->cards);
	
	INIT_WORK(&host->detect, ms_rescan, host);
	/*
	 * By default, hosts do not support SGIO or large requests.
	 * They have to set these according to their abilities.
	 */
	host->max_hw_segs = 1;
	host->max_phys_segs = 1;
	host->max_sectors = 1 << (PAGE_CACHE_SHIFT - 9);
	host->max_seg_size = PAGE_CACHE_SIZE;
      }

    return host;
}
EXPORT_SYMBOL(ms_alloc_host);

/**
 *	ms_add_host - initialise host hardware
 *	@host: ms host
 */
int ms_add_host(struct ms_host *host)
{
    int ret;

    ret = ms_add_host_sysfs(host);
    if (ret == -1) 
      ms_power_off(host);
    
    return ret;
}
EXPORT_SYMBOL(ms_add_host);

/**
 *	ms_remove_host - remove host hardware
 *	@host: ms host
 *
 *	Unregister and remove all cards associated with this host,
 *	and power down the MS bus.
 */
void ms_remove_host(struct ms_host *host)
{
    struct list_head *l, *n;

    list_for_each_safe(l, n, &host->cards) 
      {
	struct ms_card *card = ms_list_to_card(l);
	ms_remove_card(card);
      }
    ms_power_off(host);
    ms_remove_host_sysfs(host);
}
EXPORT_SYMBOL(ms_remove_host);

/**
 *	ms_free_host - free the host structure
 *	@host: ms host
 *
 *	Free the host once all references to it have been dropped.
 */
void ms_free_host(struct ms_host *host)
{
    flush_scheduled_work();
    ms_free_host_sysfs(host);
}
EXPORT_SYMBOL(ms_free_host);

#ifdef CONFIG_PM

/**
 *	ms_suspend_host - suspend a host
 *	@host: ms host
 *	@state: suspend mode (PM_SUSPEND_xxx)
 */
int ms_suspend_host(struct ms_host *host, pm_message_t state)
{
    ms_debug_msg ("%s CALLED \n",__FUNCTION__);
    ms_claim_host(host);
    ms_deselect_cards(host);
    ms_power_off(host);
    ms_release_host(host);
    
    return 0;
}
EXPORT_SYMBOL(ms_suspend_host);

/**
 *	ms_resume_host - resume a previously suspended host
 *	@host: ms host
 */
int ms_resume_host(struct ms_host *host)
{
    ms_detect_change(host, 0);

    return 0;
}
EXPORT_SYMBOL(ms_resume_host);

#endif
