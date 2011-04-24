/*
 *  linux/drivers/mmc/mmc.h
 *
 *  Copyright (C) 2003 Russell King, All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifndef _MS_H
#define _MS_H

unsigned short set_card_regadd(struct ms_host *host,char *arg);
unsigned short set_card_regiters(struct ms_host *host,short size,char mode,int address,unsigned short sys_bk_size,char address_mode,char cmd_mode,char over_write_flag,char manage_flag,short log_add,short cmd_size);
unsigned short set_cmd(struct ms_host *host,unsigned short command,unsigned short flag,unsigned short status);
unsigned short get_int(struct ms_host *host,char *arg,unsigned short status);
unsigned short get_blk_number(struct ms_host *host,int address,unsigned short sys_bk_size);
int ms_wait_for_read_data(struct ms_host *host, struct ms_request *mrq);
int ms_wait_for_write_data(struct ms_host *host, struct ms_request *mrq);
int ms_wait_for_read_data_blk(struct ms_host *host, struct ms_request *mrq,int data_size,unsigned char* data);
int ms_wait_for_write_data_blk(struct ms_host *host, struct ms_request *mrq,int data_size,unsigned char* data);
void update_ms_page(struct ms_host *host,short page_add,short block_size, struct ms_request *mrq,unsigned char*cur_blk_data);

/* core-internal functions */
void ms_init_card(struct ms_card *card, struct ms_host *host);
int ms_register_card(struct ms_card *card);
void ms_remove_card(struct ms_card *card);

struct ms_host *ms_alloc_host_sysfs(int extra, struct device *dev);
int ms_add_host_sysfs(struct ms_host *host);
void ms_remove_host_sysfs(struct ms_host *host);
void ms_free_host_sysfs(struct ms_host *host);

#endif
