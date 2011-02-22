/*
    it_mmcsd_clear_response_reg(&cmd);
 *  linux/drivers/mmc/mmci.c - ARM PrimeCell MMCI PL180/1 driver
 *
 *  Copyright (C) 2003 Deep Blue Solutions, Ltd, All Rights Reserved.
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
#include <linux/platform_device.h>  //JZ
#include <linux/mmc/host.h>
#include <linux/mmc/protocol.h>
#include <linux/mmc/machmmc.h>

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

#define GIO_SD_CARDDETECT 8
#define GIO_SDCARD_WP 36


#include "dm320_mmc.h"
#include "itmmcsd.h"
#include "mmc_debug.h"

MODULE_LICENSE("GPL");
#ifdef CONFIG_ARCH_ITDM320_20
static struct clk *clk_get(struct device *dev, const char *id);
static void clk_put(struct clk *clk);
static int clk_enable(struct clk *clk);
static int clk_use(struct clk *clk);
static unsigned long clk_get_rate(struct clk *clk);
static void clk_unuse(struct clk *clk);
void enable_interrupt(void);
void disable_interrupt(void);
void it_mmcsd_get_status(struct mmc_command *);
void change_clk25m(void);
#endif

static unsigned short mmc_inw(unsigned long port)
{
    unsigned short retval = inw(port);
    return retval;
}

static unsigned short mmc_outw(unsigned short data, unsigned long port)
{
    outw(data,port);
    return 0;
}
#define DRIVER_NAME "SD-MMC"

static unsigned int fmax = 515633;
static int bCardInitialized = 0;



void it_mmcsd_get_status(struct mmc_command *cmd)
{

        cmd->status0 = inw(IO_MMC_STATUS0);
    rmb();
        cmd->status1 = inw(IO_MMC_STATUS1);
    rmb();
        cmd->resp[0] = inw(IO_MMC_RESPONSE0);
    rmb();
        cmd->resp[1] = inw(IO_MMC_RESPONSE1);
    rmb();
        cmd->resp[2] = inw(IO_MMC_RESPONSE2);
    rmb();
        cmd->resp[3] = inw(IO_MMC_RESPONSE3);
    rmb();
        cmd->resp[4] = inw(IO_MMC_RESPONSE4);
    rmb();
        cmd->resp[5] = inw(IO_MMC_RESPONSE5);
    rmb();
        cmd->resp[6] = inw(IO_MMC_RESPONSE6);
    rmb();
        cmd->resp[7] = inw(IO_MMC_RESPONSE7);
    rmb();
        cmd->cmd_index = inw (IO_MMC_COMMAND_INDEX);
    rmb();
}

void it_mmcsd_clear_response_reg(struct mmc_command *cmd){

        outw(0,IO_MMC_RESPONSE0);
    wmb();
        outw(0,IO_MMC_RESPONSE1);
    wmb();
        outw(0,IO_MMC_RESPONSE2);
    wmb();
        outw(0,IO_MMC_RESPONSE3);
    wmb();
        outw(0,IO_MMC_RESPONSE4);
    wmb();
        outw(0,IO_MMC_RESPONSE5);
    wmb();
        outw(0,IO_MMC_RESPONSE6);
    wmb();
        outw(0,IO_MMC_RESPONSE7);
    wmb();
        outw(0,IO_MMC_COMMAND_INDEX);
    wmb();
    
    cmd->resp[0] = 0;
        cmd->resp[1] = 0;
        cmd->resp[2] = 0;
        cmd->resp[3] = 0;
        cmd->resp[4] = 0;
        cmd->resp[5] = 0;
        cmd->resp[6] = 0;
        cmd->resp[7] = 0;

}
#if  0
void it_mmcsd_print_status(struct mmc_command *cmd){
    
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
void it_mmcsd_print_status(struct mmc_command *cmd){

        printk("\n status0 : %x \t",cmd->status0);
    printk("resp[6] : %x\t",cmd->resp[6]);
        printk("resp[7] : %x\t",cmd->resp[7]);
        printk("cmd_index : %x\t",cmd->cmd_index);
}

#endif
void enable_interrupt(){
    
    volatile u16 reg = 0;
    //Enable interrupt
    /* bring the sd/mmc controller to idle state */
    reg = inw(IO_MMC_CONTROL);
        outw(reg | 0x03,IO_MMC_CONTROL);

    outw((inw(IO_INTC_EINT1) & 0xFFFC),IO_INTC_EINT1);
        outw(0x0FF,IO_MMC_INT_ENABLE);
        outw((inw(IO_INTC_EINT1) | 0x3),IO_INTC_EINT1);

        /* release idle state */
        reg = inw(IO_MMC_CONTROL);
        outw(reg & 0xFFFC,IO_MMC_CONTROL);
        reg = inw(IO_MMC_CONTROL);

}
void disable_interrupt(){

    volatile u16 reg = 0;
        //Disable interrupt

        /* bring the sd/mmc controller to idle state */
        reg = inw(IO_MMC_CONTROL);
    rmb();
        outw(reg | 0x03,IO_MMC_CONTROL);
    wmb();

        outw((inw(IO_INTC_EINT1) & 0xFFFC),IO_INTC_EINT1);
    wmb();
        outw(0x0000,IO_MMC_INT_ENABLE);
    wmb();
        outw((inw(IO_INTC_EINT1) | 0x3),IO_INTC_EINT1);
    wmb();

        /* release idle state */
    reg = inw(IO_MMC_CONTROL);
    rmb();
        outw(reg & 0xFFFC,IO_MMC_CONTROL);
    wmb();
        reg = inw(IO_MMC_CONTROL);
    rmb();
}
void change_clk25m(){

        volatile u16 reg = 0;
            
        //mmc_debug_msg ("%s\n",__FUNCTION__);    
            /* bring the sd/mmc controller to idle state */
        reg = inw (IO_MMC_CONTROL);
        rmb();
        //mmc_debug_msg ("reg : %x\n",reg);    
            outw(reg | 0x03,IO_MMC_CONTROL);
        wmb();

                /*diable clock*/
                reg = inw(IO_MMC_MEM_CLK_CONTROL);
        rmb();
                outw(reg & 0xFEFF, IO_MMC_MEM_CLK_CONTROL);
        wmb();
                reg = inw(IO_MMC_MEM_CLK_CONTROL);

                outw(inw(IO_CLK_INV) | 0x1,IO_CLK_INV);
        wmb();    

                /* set function clock */
                outw(((inw(IO_CLK_DIV3) & 0xFF00 )| 0x01) ,IO_CLK_DIV3);
        wmb();

                /* set mmc clock to ~ 25 MHz */ 
                outw(((inw(IO_MMC_MEM_CLK_CONTROL) & 0xFF00) | 0x0),IO_MMC_MEM_CLK_CONTROL);
        wmb();
                
        /* release idle state */
                reg = inw(IO_MMC_CONTROL);
        rmb();
                outw(reg & 0xFFFC,IO_MMC_CONTROL);
        wmb();

                /* enable clock */
                reg = inw(IO_MMC_MEM_CLK_CONTROL);
        rmb();
                outw(reg | 0x0100, IO_MMC_MEM_CLK_CONTROL);
        wmb();
}
static void mmci_start_data(struct mmci_host *host, struct mmc_data *data)
{
    unsigned int datactrl, timeout, irqmask;
    unsigned long long clks;

    host->data = data;
    host->size = data->blocks << data->blksz_bits;
    host->data_xfered = 0;

    mmci_init_sg(host, data);
        
    mmc_debug_msg ("%s %d\n",__FUNCTION__,__LINE__);

    clks = (unsigned long long)data->timeout_ns * host->cclk;
    do_div(clks, 1000000000UL);

    mmc_debug_msg ("%s %d\n",__FUNCTION__,__LINE__);

    timeout = data->timeout_clks + (unsigned int)clks;

    mmc_debug_msg ("host->size :  %x\n",host->size);
    mmc_debug_msg ("data->blocks : %x\n",data->blocks);
    mmc_debug_msg ("data->blksz_bits : %x\n",data->blksz_bits);

    mmc_outw(data->blksz_bits,IO_MMC_BLOCK_LENGTH);
        mmc_outw(data->blocks,IO_MMC_NR_BLOCKS);

    datactrl = MCI_DPSM_ENABLE | data->blksz_bits << 4;
    if (data->flags & MMC_DATA_READ) {
        datactrl |= MCI_DPSM_DIRECTION;
        irqmask = MCI_RXFIFOHALFFULLMASK;
    } else {
        /*
         * We don't actually need to include "FIFO empty" here
         * since its implicit in "FIFO half empty".
         */
        irqmask = MCI_TXFIFOHALFEMPTYMASK;
    }
}

void mmc_wait_for_op_cond(struct mmc_command *cmd){

u32 marg;
u16 cmd1;
u32 oper_volt = 0x4;

do    {
        marg = 0x0;
        cmd1 = 55 | 0x0200 | 0x0080; //| 0x4000 | 0x0800;
        mmc_outw(marg & 0xFFFF,IO_MMC_ARG_LOW);
        mmc_outw((marg >> 16),IO_MMC_ARG_HI);
        mmc_outw(cmd1,IO_MMC_COMMAND);
        it_mmcsd_get_status(cmd);
        it_mmcsd_clear_response_reg(cmd); //clear the response register in the register
        //it_mmcsd_print_status(cmd);

        marg = (oper_volt << 14 );
        cmd1 = 41 | 0x0600 | 0x0080;//| 0x4000;
        mmc_outw(marg & 0xFFFF,IO_MMC_ARG_LOW);

        mmc_outw((marg >> 16),IO_MMC_ARG_HI);
        mmc_outw(cmd1,IO_MMC_COMMAND);
        it_mmcsd_get_status(cmd);
        it_mmcsd_clear_response_reg(cmd); //clear the response register in the register
        //it_mmcsd_print_status(cmd);
        }while (!(cmd->resp[7] & 0x8000));
}

EXPORT_SYMBOL(mmc_wait_for_op_cond);

static void
mmci_request_end(struct mmci_host *host, struct mmc_request *mrq)
{
    host->mrq = NULL;
    host->cmd = NULL;
    
    if (mrq->data){
        mrq->data->bytes_xfered = host->data_xfered;
    }
    /*
     * Need to drop the host lock here; mmc_request_done may call
     * back into the driver...
     */
    spin_unlock(&host->lock);

    mmc_request_done(host->mmc, mrq);
    spin_lock(&host->lock);
}

static void mmci_stop_data(struct mmci_host *host)
{
    host->data = NULL;
}

#if DEBUG_MMC_COMMAND
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

    printk("\n%s %x\n", str, cmd->arg);
}
#endif

static void
mmci_start_command(struct mmci_host *host, struct mmc_command *cmd, u32 c)
{
    u32 marg;
    u16 cmd1;

#if DEBUG_MMC_COMMAND
    mmci_print_cmd(cmd);
#endif

    it_mmcsd_clear_response_reg(cmd);

    host->cmd = cmd;

        marg = cmd->arg; 
        cmd1 = cmd->opcode | cmd->flags | 0x0000;    

        outw(marg & 0xFFFF,IO_MMC_ARG_LOW);
        outw((marg >> 16),IO_MMC_ARG_HI);
        outw(cmd1,IO_MMC_COMMAND);
    wmb();
    bCardInitialized = 1;
}

static void
mmci_data_irq(struct mmci_host *host, struct mmc_data *data,
          unsigned int status)
{
    if (status & MMC_INT_DATA_DONE) {
        host->data_xfered += 1 << data->blksz_bits;
    }
    if (status & (MCI_DATACRCFAIL|MCI_DATATIMEOUT|MCI_TXUNDERRUN|MCI_RXOVERRUN)) {
        if (status & MCI_DATACRCFAIL)
            data->error = MMC_ERR_BADCRC;
        else if (status & MCI_DATATIMEOUT)
            data->error = MMC_ERR_TIMEOUT;
        else if (status & (MCI_TXUNDERRUN|MCI_RXOVERRUN))
            data->error = MMC_ERR_FIFO;
        status |= MCI_DATAEND;
    }
    if (status & MCI_DATAEND) {
        mmci_stop_data(host);
        if (!data->stop) {
            mmci_request_end(host, data->mrq);
        } else {
            mmci_start_command(host, data->stop, 0);
        }
    }
}

static void
mmci_cmd_irq(struct mmci_host *host, struct mmc_command *cmd,
         unsigned int status)
{
    host->cmd = NULL;
    
    if (status & MMC_INT_RSP_TIMEOUT) {
        cmd->error = MMC_ERR_TIMEOUT;
        mmc_debug_msg ("Card has not responded \n");
    } else if (status & MMC_INT_RSP_CRC_ERROR){
        mmc_debug_msg("CRC ERROR\n");
        cmd->error = MMC_ERR_BADCRC;
    }

    if (!cmd->data || cmd->error != MMC_ERR_NONE) {
        mmci_request_end(host, cmd->mrq);
    } else if ((cmd->data->flags & MMC_DATA_READ)) {
        mmc_debug_msg("Data transfer\n");
        mmci_start_data(host, cmd->data);
        host->mrq = NULL;
            host->cmd = NULL;
    }
}
/*
 * Handle completion of command and data transfers.
 */
static volatile int in_interrupt = 1;
static irqreturn_t mmci_irq(int irq, void *dev_id, struct pt_regs *regs)
{
    struct mmci_host *host = dev_id;
    int ret = 1;
    struct mmc_command *cmd,data_cmd;
    struct mmc_data *data;

    spin_lock(&host->lock);
    //local_irq_disable();

        if (in_interrupt){
                mmc_debug_msg(" done -\t\n");
                in_interrupt = 0;
                }
    cmd = host->cmd;

    if (cmd){
    mmc_debug_msg ("Command is not NULL\n");    
    if((unsigned long ) cmd == 0x6b6b6b6b)
    {
        printk("\n Its too early to get an interrupt \n");
        it_mmcsd_get_status(&data_cmd);
        it_mmcsd_print_status(&data_cmd);
        dump_stack();
        panic("Early Interrupt\n");
        
        spin_unlock(&host->lock);
        return IRQ_HANDLED;

    }
    it_mmcsd_get_status(cmd);
    //it_mmcsd_print_status(cmd);
    }
    else{
    mmc_debug_msg("Command is NULL\n");
    it_mmcsd_get_status(&data_cmd);
//    it_mmcsd_print_status(&data_cmd);
    cmd = &data_cmd;
    }

    data = host->data;
        if ((cmd->status0 & (MMC_INT_READ_CRC_ERROR|MMC_INT_WRITE_CRC_ERROR|MMC_INT_READ_TIMEOUT|MMC_INT_DATA_DONE)) |
        (cmd->status1 & (MMC_INT_DAT_RCV_RDY_S1|MMC_INT_DAT_TRX_RDY_S1))){
                mmci_data_irq(host,data,(cmd->status0|cmd->status1 << 16));
        }

    if ((cmd->status0 & (MMC_INT_RSP_CRC_ERROR | MMC_INT_RSP_TIMEOUT | MMC_INT_RSP_DONE))){
                        mmci_cmd_irq(host, cmd, cmd->status0);
    }

        in_interrupt = 1;
    ret = 1; 
    //local_irq_enable();
    spin_unlock(&host->lock);
    return IRQ_RETVAL(ret);
}

static void mmci_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
    struct mmci_host *host = mmc_priv(mmc);
    
    WARN_ON(host->mrq != NULL);

    spin_lock_irq(&host->lock);

    host->mrq = mrq;
    
    if (mrq->data && mrq->data->flags & MMC_DATA_READ){
        mmci_start_data(host, mrq->data);
    }

    mmci_start_command(host, mrq->cmd, 0);

    spin_unlock_irq(&host->lock);
}

static void mmci_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
        volatile u16 reg = 0;


                /* bring the sd/mmc controller to idle state */
                reg = inw (IO_MMC_CONTROL);
                rmb();
                outw(reg | 0x03,IO_MMC_CONTROL);
                wmb();
        
        if (mmc->ios.bus_width == MMC_BUS_WIDTH_4){

        /*changing to 4-data line mode*/
        outw(reg | (1 << 2),IO_MMC_CONTROL); 
        wmb();    
        }
        else{

        /*changing to 4-data line mode*/
                outw(reg & (~(1 << 2)),IO_MMC_CONTROL);
                wmb();
        }
        

                /* release idle state */
                reg = inw(IO_MMC_CONTROL);
                rmb();
                outw(reg & 0xFFFC,IO_MMC_CONTROL);
                wmb();
}


int write_to_card(struct mmc_host *mmc,struct mmc_request *mrq){

    u32 current_add;
    u16 no_of_blks;
    u32 marg;
        u16 cmd1,reg;
    int write_count;
    volatile unsigned short reg1 = 0;
#ifndef DMA_TRANSFER
    short write_data;
#endif
    struct mmc_command cmd;
    struct mmci_host *host = mmc_priv(mmc);

    spin_lock(&host->lock);
    disable_interrupt();
    local_irq_disable();

    /* disable clock */
        reg = mmc_inw(IO_MMC_MEM_CLK_CONTROL);    
        mmc_outw(reg & 0xFEFF, IO_MMC_MEM_CLK_CONTROL);

    mrq->cmd->error = 0;
        mrq->cmd->mrq = mrq;

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

    /*************WORKING******************/
    /*
    * Map the current scatter buffer.
    20*/
    it_mmcsd_get_status(&cmd);
    it_mmcsd_clear_response_reg(&cmd);

    current_add = mrq->cmd->arg;
        no_of_blks = mrq->data->blocks;

    do{
     unsigned long flags;
         unsigned int remain, len = 0;
         char *buffer = NULL;
    short *buffer_short = NULL;
    short stop_status = 0;
    short store_DMASEL;

    buffer = mmci_kmap_atomic(host, &flags) + host->sg_off;
    if (buffer == NULL)
                mmc_debug_msg ("NULL data Pointer\n");

    remain = host->sg_ptr->length - host->sg_off;

    mmc_debug_msg ("WRITE_REMAIN : %d\n",remain);
    
        outw((remain/512),IO_MMC_NR_BLOCKS);

    
    buffer_short = (short*)buffer;
#ifdef DMA_TRANSFER
     /* bring the sd/mmc controller to idle state */
        reg = inw(IO_MMC_CONTROL);
        outw(reg | 0x03,IO_MMC_CONTROL);
        reg = inw(IO_MMC_CONTROL);

        outw(((inw(IO_MMC_CONTROL) | 0x10)&0xFBFF),IO_MMC_CONTROL);

        /* release idle state */
        reg = inw(IO_MMC_CONTROL);
        outw(reg & 0xFFFC,IO_MMC_CONTROL);
        reg = inw(IO_MMC_CONTROL);

        outw(((0xDFFF) & inw (IO_MMC_SD_DMA_MODE)),IO_MMC_SD_DMA_MODE);//Enabling DMA
        wmb();
        outw((0x0002),IO_MMC_SD_DMA_TRIGGER);//DMA break
        wmb();
        outw ((0x0003),IO_SDRAM_SDDMASEL);

        //printk("Started writing %x blocks from the address %x\n",no_of_blks,current_add);
        //outw(no_of_blks,IO_MMC_NR_BLOCKS);
        wmb();

        store_DMASEL = inw (IO_SDRAM_SDDMASEL);
        rmb();
        reg = store_DMASEL;
        //printk ("Value in IO_SDRAM_SDDMASEL : %x\n",reg);

        outw(((1 << 12) | inw (IO_MMC_SD_DMA_MODE)),IO_MMC_SD_DMA_MODE);//DIRECTION

        //outw((0x0400 | inw (IO_MMC_SD_DMA_MODE)),IO_MMC_SD_DMA_MODE);//Enable word swap for Reading

        outw((0xF000),IO_MMC_SD_DMA_TIMEOUT); //2.4576ms //TIMEOUT

        wmb();
        //outw((no_of_blks * 512),0x04B0);
        outw(remain,0x04B0);//DMA SIZE

        outw(((unsigned long)buffer_short & 0xFFFF),IO_MMC_SD_DMA_ADDR_LOW);
        wmb();
        outw((((unsigned long)buffer_short >> 16) & 0xFFFF),IO_MMC_SD_DMA_ADDR_HI);
        wmb();
        outw(((0x2000) | inw (IO_MMC_SD_DMA_MODE)),IO_MMC_SD_DMA_MODE);
    wmb();
#endif    
    marg = current_add;
    cmd1 = 25 |0x4000| 0x2000 |0x0800 | 0x0200 | 0x00;
        outw(marg & 0xFFFF,IO_MMC_ARG_LOW);
        outw((marg >> 16),IO_MMC_ARG_HI);
        outw(cmd1,IO_MMC_COMMAND);
    wmb();

        write_count = 0;
#ifndef DMA_TRANSFER    
    temp = 0;
        it_mmcsd_get_status(&cmd);
        reg = cmd.status0;
    if ((cmd.status1 & 0x4))
        {
    write_data = *buffer_short;
    buffer_short += 1;

    mmc_debug_msg_rw ("%x,",(write_data)& 0xFF);
    mmc_debug_msg_rw ("%x\t",((write_data >> 8) & 0xFF));

    temp = (write_data & 0xFF00) >> 8;
    write_data = ((write_data & 0xFF) << 8) | temp;
    mmc_debug_msg ("***********in first condition**************\n");
    
        outw(write_data,IO_MMC_TX_DATA);
        write_count += 1;
        }

        do
               {
            temp = 0;
                        it_mmcsd_get_status(&cmd);
                        reg = cmd.status0;
                        if (reg & 0x0020)
                                {
                                        mmc_debug_msg ("<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<Write CRC error\n");
                    mmc_debug_msg ("STATUS0 :%x\tSTATUS1 :%x\n",cmd.status0,cmd.status1);
                    host->data->error = MMC_ERR_BADCRC;    
                    goto end_write;
                                }
                        if ((cmd.status1 & 0x4))
                                {
                    write_data = *buffer_short;
                    mmc_debug_msg_rw ("%x,",(write_data)& 0xFF);
                    mmc_debug_msg_rw ("%x\t",((write_data >> 8) & 0xFF));
                         temp = (write_data & 0xFF00) >> 8;
                        write_data = ((write_data & 0xFF) << 8) | temp;
                    buffer_short += 1;
    
                                        outw(write_data,IO_MMC_TX_DATA);
                    wmb();
                                        write_count += 1;
                                }
                            else{
                    if (cmd.status0 & 0x10){
                        mmc_debug_msg("Command Time OUT");
                        mmc_debug_msg ("S0 : %x\tS1 : %x\n",cmd.status0,cmd.status1);
                                break;
                    }
                }      
    } while (!(reg &0x0001));

        do{
        it_mmcsd_get_status(&cmd);
    }while((!(cmd.status0 & 0x2)) && (cmd.status1 & 0x1));
#else
    do
         {
                it_mmcsd_get_status(&cmd);
        if (cmd.status0 & MMC_INT_RSP_TIMEOUT){
                        host->data->error = 1;
                        goto complete;
                }
         }while(!((cmd.status0 & 0x200)==0x200));//checking for respone of the command

                //it_mmcsd_print_status(&cmd);
                //printk ("Device Revision Number : %x\n",inw (IO_BUSC_REVR));

                outw((0x0001),IO_MMC_SD_DMA_TRIGGER);
                wmb();

        reg = 0;
                reg1 = 0;
                do
                {
                        if (inw (IO_MMC_SD_DMA_STATUS1) & 0x1000){
                                reg = 0x1000;
                        }
                        it_mmcsd_get_status(&cmd);
                        if ((cmd.status0 & MMC_INT_RSP_TIMEOUT) |(cmd.status0 & MMC_INT_WRITE_CRC_ERROR)){
                                host->data->error = 1;
                outw((0x0002),IO_MMC_SD_DMA_TRIGGER);
                        wmb();
                                goto complete;
                        }
                }while((reg & 0x1000) && (!((cmd.status0 & 0x0100)==0x0100)));//Checking for DMA complete.

                write_count = 0;

                outw((0x0002),IO_MMC_SD_DMA_TRIGGER);
                wmb();

                //printk("bytes read : %d\n",(read_count*2));
#if 0
                for(i=0;i< (512*no_of_blks);i++){
                        printk("%4c",*(buffer_short));
                        buffer_short += 1;
                }
#endif
#endif
#ifndef DMA_TRANSFER
    len = (write_count*2);
#else
    len = remain;
#endif
    mmc_debug_msg ("len : %d\n",len);
    host->data_xfered += len;
    /*
        *Unmap the buffer.
        */
    mmci_kunmap_atomic(host, &flags);
    host->sg_off += len;
    host->size -= len;
    remain -= len;

    /*************WORKING******************/
    /*Stop command*/
    it_mmcsd_get_status(&cmd);    
    it_mmcsd_clear_response_reg(&cmd);

    marg = 0;
    cmd1 = 12 | 0x0200 | 0x0100 | 0x0080;
    outw(marg & 0xFFFF,IO_MMC_ARG_LOW);
    outw((marg >> 16),IO_MMC_ARG_HI);
    outw(cmd1,IO_MMC_COMMAND);

    stop_status = 0;
    do
    {
        it_mmcsd_get_status(&cmd);
        if (cmd.status0 & 0x4)    
            stop_status |= cmd.status0;    
        if (cmd.status0 & 0x2)
            stop_status |= cmd.status0;
        if (cmd.status0 & MMC_INT_RSP_TIMEOUT){
                        host->data->error = 1;
                        goto complete;
                }
    }while( !((stop_status& 0x6)==6));//checking for busy state and response of the command

    if (remain){
        break;
    }

    if (!mmci_next_sg(host)){
        it_mmcsd_get_status(&cmd);    
        break;
    }
    current_add += len;
    
    }while(1);
    
    mrq->data->bytes_xfered = host->data_xfered;
    mmc_debug_msg ("mrq->data->bytes_xfered : %d\n",mrq->data->bytes_xfered);

complete:    
    /* enable clock */
        reg = mmc_inw(IO_MMC_MEM_CLK_CONTROL);
        mmc_outw(reg | 0x0100, IO_MMC_MEM_CLK_CONTROL);

    local_irq_enable();
    enable_interrupt();
        spin_unlock(&host->lock);
    host->data = NULL;

    return 0;


}
int read_from_card(struct mmc_host *mmc,struct mmc_request *mrq){

    u32 current_add;
    u16 no_of_blks;
    u32 marg;
        u16 cmd1,reg;
    int read_count;
    struct mmc_command cmd;
    struct mmci_host *host = mmc_priv(mmc);
    
    spin_lock(&host->lock);
    disable_interrupt();
    local_irq_disable();
    
    /* disable clock */
        reg = inw(IO_MMC_MEM_CLK_CONTROL);    
        outw(reg & 0xFEFF, IO_MMC_MEM_CLK_CONTROL);
    rmb();
    
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
    short store_DMASEL = 0;

    buffer = mmci_kmap_atomic(host, &flags) + host->sg_off;
    if (buffer == NULL)
                mmc_debug_msg ("NULL data Pointer\n");
    remain = host->sg_ptr->length - host->sg_off;
    
    mmc_debug_msg (">>>>>>>>>>>>REMAIN : %d\n",remain);

        outw((remain/512),IO_MMC_NR_BLOCKS);
    buffer_short = buffer;
#ifdef DMA_TRANSFER
     /* bring the sd/mmc controller to idle state */
        reg = inw(IO_MMC_CONTROL);
        outw(reg | 0x03,IO_MMC_CONTROL);
        reg = inw(IO_MMC_CONTROL);

        outw(((inw(IO_MMC_CONTROL) | 0x10)&0xFDFF),IO_MMC_CONTROL);

        /* release idle state */
        reg = inw(IO_MMC_CONTROL);
        outw(reg & 0xFFFC,IO_MMC_CONTROL);
        reg = inw(IO_MMC_CONTROL);

    outw(((0xDFFF) & inw (IO_MMC_SD_DMA_MODE)),IO_MMC_SD_DMA_MODE);
        wmb();
    outw((0x0002),IO_MMC_SD_DMA_TRIGGER);
        wmb();
    outw ((0x0003),IO_SDRAM_SDDMASEL);

        //printk("Started Reading %x blocks from the address %x\n",no_of_blks,current_add);
        //outw(no_of_blks,IO_MMC_NR_BLOCKS);
        wmb();

        store_DMASEL = inw (IO_SDRAM_SDDMASEL);
        rmb();
        reg = store_DMASEL;
        //printk ("Value in IO_SDRAM_SDDMASEL : %x\n",reg);

    outw(((0xEFFF) & inw (IO_MMC_SD_DMA_MODE)),IO_MMC_SD_DMA_MODE);
        
    //outw((0x0400 | inw (IO_MMC_SD_DMA_MODE)),IO_MMC_SD_DMA_MODE);//Enable word swap for Reading
    
    outw((0xF000),IO_MMC_SD_DMA_TIMEOUT); //2.4576ms

    wmb();
    //outw((no_of_blks * 512),0x04B0);
    outw(remain,0x04B0);//DMA SIZE

    outw(((unsigned long)buffer_short & 0xFFFF),IO_MMC_SD_DMA_ADDR_LOW);
    wmb();
    outw((((unsigned long)buffer_short >> 16) & 0xFFFF),IO_MMC_SD_DMA_ADDR_HI);
    wmb();
    outw(((0x2000) | inw (IO_MMC_SD_DMA_MODE)),IO_MMC_SD_DMA_MODE);
    wmb();
#endif

        marg = current_add;
        cmd1 = 18 | 0x0200 | 0x4000 | 0x2000 | 0x0080 | 0x0000;
        outw(marg & 0xFFFF,IO_MMC_ARG_LOW);
        outw((marg >> 16),IO_MMC_ARG_HI);
        outw(cmd1,IO_MMC_COMMAND);

    do
        {
        it_mmcsd_get_status(&cmd);
        if ((cmd.status0 & MMC_INT_RSP_TIMEOUT) | (cmd.status0 & MMC_INT_RSP_CRC_ERROR)){
                        host->data->error = 1;
            goto complete_read;
                }
        }while( !((cmd.status0 & 0x4)==4));//checking for busy state and respone of the read command
 

        read_count = 0;

#ifndef DMA_TRANSFER
        do{
        it_mmcsd_get_status(&cmd);
                reg = cmd.status0;
                if (reg & 0x0040)
                {
                        mmc_debug_msg ("<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<Read CRC error\n");
            mmc_debug_msg ("STATUS0 :%x\tSTATUS1 :%x\n",cmd.status0,cmd.status1);
                        break;
                }
                if ((cmd.status1 & 0x8))
                {
            temp_data = inw (IO_MMC_RX_DATA);
            mmc_debug_msg_rw ("%x\t",temp_data);
            *buffer_short = (temp_data >> 8) & 0xFF;
            buffer_short = buffer_short + 1;
            *buffer_short = temp_data & 0xFF;    
            buffer_short = buffer_short + 1;
            read_count++;

                }
        else    
                        if (cmd.status0 & 0x10){
                            mmc_debug_msg("Command Time OUT");
                                break;
        }
    } while (!(reg &0x0001));

    temp_data = inw (IO_MMC_RX_DATA);
        mmc_debug_msg_rw ("%x\t",temp_data);
        *buffer_short = (temp_data >> 8) & 0xFF;
        buffer_short = buffer_short + 1;
        *buffer_short = temp_data & 0xFF;
        buffer_short = buffer_short + 1;
        read_count++;

        do{
        it_mmcsd_get_status(&cmd);
    }while((cmd.status1 & 0x1));

    if(reg &1)
    {
        int temp = inw(IO_MMC_NR_BLOCKS_COUNT);
        mmc_debug_msg ("No of blocks remaining to be transferd :%d\n",temp);
    }
#else
         do
         {
        it_mmcsd_get_status(&cmd);
        if (cmd.status0 & MMC_INT_RSP_TIMEOUT){
                        host->data->error = 1;
            goto complete_read;
                }
         }while(!((cmd.status0 & 0x400)==0x400));//checking for respone of the command

                //it_mmcsd_print_status(&cmd);
                //printk ("Device Revision Number : %x\n",inw (IO_BUSC_REVR));

                outw((0x0001),IO_MMC_SD_DMA_TRIGGER);
                wmb();
#if 0
                do
                {
                        reg = inw (IO_MMC_SD_DMA_STATUS1);
                        rmb();
                }while( (reg & 0x1000));//Checking for DMA complete.

                read_count = 0;

                do
                {
                        it_mmcsd_get_status(&cmd);
                }while( !((cmd.status0 & 0x0100)==0x0100));//Checking for DMA Done
#else
        reg = 0;
                do
                {
                        if (inw (IO_MMC_SD_DMA_STATUS1) & 0x1000){
                                reg = 0x1000;
                        }
                        it_mmcsd_get_status(&cmd);
                        if ((cmd.status0 & MMC_INT_READ_TIMEOUT) |(cmd.status0 & MMC_INT_READ_CRC_ERROR)){
                                host->data->error = 1;
                outw((0x0002),IO_MMC_SD_DMA_TRIGGER);
                        wmb();
                goto complete_read;
                        }
                }while((reg & 0x1000) && (!((cmd.status0 & 0x0100)==0x0100)));//Checking for DMA complete.
#endif

                outw((0x0002),IO_MMC_SD_DMA_TRIGGER);
                wmb();

                //printk("bytes read : %d\n",(read_count*2));
#if 0
                for(i=0;i< (512*no_of_blks);i++){
                        printk("%4c",*(buffer_short));
                        buffer_short += 1;
                }
#endif
#endif
#ifndef DMA_TRANSFER
    len = buffer_short - buffer;
#else
    len = remain;
#endif

    mmc_debug_msg ("len : %d\n",len);
    host->data_xfered += len;

    /*
        * Unmap the buffer.
        */
    mmci_kunmap_atomic(host, &flags);
    buffer = NULL;
    buffer_short = NULL ;
    host->sg_off += len;
    host->size -= len;
    remain -= len;

    /*************WORKING******************/
    /*Stop command*/
    it_mmcsd_get_status(&cmd);    
    it_mmcsd_clear_response_reg(&cmd);

    marg = 0;
    cmd1 = 12 | 0x0200 | 0x0100 | 0x0080;
    outw(marg & 0xFFFF,IO_MMC_ARG_LOW);
    outw((marg >> 16),IO_MMC_ARG_HI);
    outw(cmd1,IO_MMC_COMMAND);

    do
    {
        it_mmcsd_get_status(&cmd);    
        mmc_debug_msg("S0 : %x\t S1 : %x\n",cmd.status0,cmd.status1);
        if ((cmd.status0 & MMC_INT_RSP_TIMEOUT) | (cmd.status0 & MMC_INT_RSP_CRC_ERROR)){
                        host->data->error = 1;
            goto complete_read;
                }
    }while( !((cmd.status0 & 0x4)==4));//checking for busy state and respone of the command


    if (remain){        
        mmc_debug_msg ("REMAIN AFTER READ : %d\n",remain);
    }

    if (!mmci_next_sg(host)){
        it_mmcsd_get_status(&cmd);    
        break;
    }
    current_add += len;
    
    }while(1);
    

    mrq->data->bytes_xfered = host->data_xfered;
    mmc_debug_msg ("mrq->data->bytes_xfered : %d\n",mrq->data->bytes_xfered);

complete_read:
    /* enable clock */
        reg = mmc_inw(IO_MMC_MEM_CLK_CONTROL);
        outw(reg | 0x0100, IO_MMC_MEM_CLK_CONTROL);


    local_irq_enable();
    enable_interrupt();
        spin_unlock(&host->lock);

    host->data = NULL;

    return 0;
}
int src_read(struct mmc_host *mmc,struct mmc_request *mrq,unsigned int *relative,char* scr_reg){

    u32 marg;
        u16 cmd1,reg;
    int read_count;
    struct mmc_command cmd;
    struct mmci_host *host = mmc_priv(mmc);
    short temp_data;

    spin_lock(&host->lock);
    disable_interrupt();
    local_irq_disable();
    memset(scr_reg,0,8*sizeof(char));
    
    /* disable clock */
        reg = inw(IO_MMC_MEM_CLK_CONTROL);    
    rmb();
        outw(reg & 0xFEFF, IO_MMC_MEM_CLK_CONTROL);
    wmb();
       
#if 0    
    printk ("%s %d\n",__FUNCTION__,__LINE__);
    mrq->cmd->error = 0;
        mrq->cmd->mrq = mrq;
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

    it_mmcsd_get_status(&cmd);
    it_mmcsd_clear_response_reg(&cmd);

    current_add = mrq->cmd->arg;
        no_of_blks = mrq->data->blocks;// * host->sg_len; //To read more than segment

#endif
    do{

        char *buffer = NULL;
    short *buffer_short = NULL;
    int count = 0;

    buffer = scr_reg;

    it_mmcsd_clear_response_reg(&cmd);
    it_mmcsd_get_status(&cmd);
    //it_mmcsd_print_status(&cmd);    

        outw(1,IO_MMC_NR_BLOCKS);
    outw(0x200,IO_MMC_BLOCK_LENGTH);

    buffer_short = (short*)buffer;


        marg = 0x200;
        cmd1 = 51 | 0x0200 | 0x2000 | 0x0080 | 0x0000 | 0x0000;
        outw(marg & 0xFFFF,IO_MMC_ARG_LOW);
        outw((marg >> 16),IO_MMC_ARG_HI);
        outw(cmd1,IO_MMC_COMMAND);

#if 1 
    do
        {
                it_mmcsd_get_status(&cmd);
        //mdelay(10);
    }while( !((cmd.status0 & 0x4)==4));//checking for busy state and response of the command
#endif

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
    } while (!(reg &0x0001));
        do{
        if (count ++ > 100)
            break;
        it_mmcsd_get_status(&cmd);
    }while((cmd.status1 & 0x1));
    buffer_short = NULL ;
    break;

    }while(1);

    /* enable clock */
    
        reg = mmc_inw(IO_MMC_MEM_CLK_CONTROL);
        outw(reg | 0x0100, IO_MMC_MEM_CLK_CONTROL);

    local_irq_enable();
    enable_interrupt();
        spin_unlock(&host->lock);
    return 0;
}

static volatile int plug_count = 0;
static int dat3stat_new = 0;
static int dat3stat_old = 0;
static int dat3stat_count = 0;

static void host_configuration(void){
    
    short reg = 0;
    

    /*Erase past settings*/
    outw(0x0,IO_MMC_CONTROL);
    outw(0x0,IO_MMC_MEM_CLK_CONTROL);
//    outw(0x0,IO_MEM_STICK_MODE);//MS mode
    
    /*HOST CONFIGURATION STARTS */
    /* bring the controller to reset state */
        reg = inw(IO_MMC_CONTROL);
        outw(reg | 0x03,IO_MMC_CONTROL);
    wmb();
        reg = inw(IO_MMC_CONTROL);
    rmb();

    /* configure mmc controller */
        reg = inw(IO_MMC_CONTROL);
        outw(reg | 0x0200 | 0x0400 | 0x0000,IO_MMC_CONTROL);
    wmb();
        reg = inw(IO_MMC_CONTROL);
    rmb();

    /*Response timeout register */
    outw(255,IO_MMC_RESPONSE_TIMEOUT);
    wmb();

    /*Data timeout register*/
    reg = inw (IO_MMC_RESPONSE_TIMEOUT);
    rmb();
    outw(reg | 0x1F00,IO_MMC_RESPONSE_TIMEOUT);
        wmb();
    outw(0xFFFF,IO_MMC_READ_TIMEOUT);
    wmb();

    /* disable clock */
        reg = inw(IO_MMC_MEM_CLK_CONTROL);
        outw(reg & 0xFEFF, IO_MMC_MEM_CLK_CONTROL);    
    wmb();
        reg = inw(IO_MMC_MEM_CLK_CONTROL);
    rmb();

    /* set clock */
        outw(inw(IO_CLK_INV) | 0x1,IO_CLK_INV);
    wmb();

        /* set function clock */
        outw(( (mmc_inw(IO_CLK_DIV3) & 0xFF00) | (0x01)) ,IO_CLK_DIV3);
    wmb();

        /* set mmc clock to ~ 400 kHz */ /* 338 KHZ */
        outw((0),IO_MMC_MEM_CLK_CONTROL);
    wmb();
        outw(((mmc_inw(IO_MMC_MEM_CLK_CONTROL) & 0xFF00) | (0x40)),IO_MMC_MEM_CLK_CONTROL);
    wmb();
        outw((0x40),IO_MMC_MEM_CLK_CONTROL);
    wmb();

    /* release controller reset state */
        reg = inw(IO_MMC_CONTROL);
    rmb();
        outw(reg & 0xFFFC,IO_MMC_CONTROL);
    wmb();
        reg = inw(IO_MMC_CONTROL);
    rmb();
    
    /* enable clock */
        reg = inw(IO_MMC_MEM_CLK_CONTROL);
    rmb();
        outw(reg | 0x0100, IO_MMC_MEM_CLK_CONTROL);
    wmb();
        reg = inw(IO_MMC_MEM_CLK_CONTROL);
    rmb();

    /*HOST CONFIGURATION ENDS*/

}

static void mmci_check_status(unsigned long data)
{
    struct mmci_host *host = (struct mmci_host *)data;
    u16 status;
    
    status = gio_get_bitset(GIO_SD_CARDDETECT) & !(inw(IO_GIO_CARD_ST));
    rmb();
    if (status ^ host->oldstat){
        host_configuration();
        mmc_detect_change(host->mmc, 0);
        printk("card state changed\n");
        goto detect_called;
    }
    if (inw (IO_MMC_STATUS1) & 0x10)
        dat3stat_new = 1;    
    else 
        dat3stat_new = 0;

    if ((dat3stat_old == 1) && (dat3stat_new == 1)){
        if ((dat3stat_count == 0) && (plug_count == 0)){
            host_configuration();
            mmc_detect_change(host->mmc, 0);
            dat3stat_count = 1;
        }
        
    }

detect_called:
    host->oldstat = status;
    dat3stat_old = dat3stat_new;
    mod_timer(&host->timer, jiffies + HZ);
    outw (0x0000,IO_GIO_CARD_SET);
    wmb();
    plug_count = 0;        
}
static int get_write_protect_status(struct mmc_host *mmc){

    mmc_debug_msg("Write protect status : %x\n",gio_get_bitset(GIO_SDCARD_WP));
    return gio_get_bitset(GIO_SDCARD_WP);
}    

static struct mmc_host_ops mmci_ops = {
        .request        = mmci_request,
        .set_ios        = mmci_set_ios,
    .get_ro        = get_write_protect_status,
};


static int mmci_probe(struct device *dev)
{
    struct mmc_platform_data *plat = dev->platform_data;
    struct mmci_host *host = NULL;
    struct mmc_host *mmc = NULL;
    int ret;
    volatile u16 reg;
    
#ifdef DMA_TRANSFER
    mmc_debug_msg("DMA TRANSFER ENABLED\n");
#endif    
    mmc = mmc_alloc_host(sizeof(struct mmci_host), dev);
    bCardInitialized = 0;
    
    if (!mmc) {
        ret = -ENOMEM;
        mmc_debug_msg(" mmc_alloc_host failed \n");
        goto rel_regions;
    }

    host = mmc_priv(mmc);
    host->cmd = NULL;
    host->clk = clk_get(dev, "MCLK");

    mmc->readdata = read_from_card;//used in calling the read system call
    mmc->writedata = write_to_card;//used in calling the write system call
    mmc->src_data = src_read;//used in calling src read function
    mmc->enable_interrupt = enable_interrupt;
    mmc->disable_interrupt = disable_interrupt;
    mmc->change_clk25m = change_clk25m;

    host_configuration();//Host controller configuration    
    
    ret = clk_use(host->clk);
    if (ret)
        goto clk_free;

    ret = clk_enable(host->clk);
    if (ret)
        goto clk_unuse;

    host->plat = plat;
    host->mclk = clk_get_rate(host->clk);
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
        
    
    spin_lock_init(&host->lock);

    reg = inw(IO_INTC_EINT1);
    rmb();
        reg &= ~(3 << 0);
        outw(reg,IO_INTC_EINT1);
    wmb();

    //ret = request_irq(IRQ_SD_MMC, mmci_irq, SA_SHIRQ, DRIVER_NAME " (cmd)", host);
    //ret = request_irq(IRQ_SD_MMC, mmci_irq, SA_INTERRUPT, DRIVER_NAME " (cmd)", host);
    //ret = request_irq(IRQ_SD_MMC, mmci_irq,SA_PROBE, DRIVER_NAME " (cmd)", host);
    ret = request_irq(IRQ_SD_MMC, mmci_irq,0, DRIVER_NAME " (cmd)", host);
    if (ret){    
        printk ("Interrupt %d may be busy\n",IRQ_SD_MMC);
        free_irq(IRQ_SD_MMC,host);
    }

    outw((mmc_inw(IO_INTC_EINT1) & 0xFFFC),IO_INTC_EINT1);
    wmb();
    outw(0x114,IO_MMC_INT_ENABLE);
    wmb();    
    outw((mmc_inw(IO_INTC_EINT1) | 0x3),IO_INTC_EINT1);
    wmb();
    
    dev_set_drvdata(dev,mmc);

    mmc_add_host(mmc);

    init_timer(&host->timer);
    host->timer.data = (unsigned long)host;
    host->timer.function = mmci_check_status;
    host->timer.expires = jiffies + HZ;
    add_timer(&host->timer);


    mmc_debug_msg ("Leaving mmci probe\n");

    return 0;

 clk_unuse:
    clk_unuse(host->clk);
 clk_free:
    clk_put(host->clk);

 rel_regions:
    return ret;
}

static int mmci_remove(struct device *dev)
{
    struct mmc_host *mmc = dev_get_drvdata(dev);
    mmc_debug_msg ("entering %s\n",__FUNCTION__);

    plug_count = 0;
    dat3stat_new = 0;

    dev_set_drvdata(dev, NULL);

    if (mmc) {
        struct mmci_host *host = mmc_priv(mmc);

        del_timer_sync(&host->timer);

        mmc_remove_host(mmc);

        mmc_free_host(mmc);
        
        free_irq(IRQ_SD_MMC,mmc_priv(mmc));
    }
    
    mmc_debug_msg ("leaving %s\n",__FUNCTION__);
    
    return 0;
}

#define mmci_suspend    NULL
#define mmci_resume    NULL

static struct device_driver mmci_driver = { 
    .name        = DRIVER_NAME,
    .bus            = &platform_bus_type,
    .probe        = mmci_probe,
    .remove        = mmci_remove,
    .suspend    = mmci_suspend,
    .resume        = mmci_resume,
};
static irqreturn_t it_mmcsd_detect_interrupt(int flags, void * data, struct pt_regs * regs){
        int inserted = 0;
    if (!plug_count)
    {
            inserted = gio_get_bitset(GIO_SD_CARDDETECT);
            outw (0x0003,IO_GIO_CARD_SET);  //inversion & card detect enable
        wmb();

            if (inserted & !inw(IO_GIO_CARD_ST)){
                    mmc_debug_msg("Card pluged out\t\n");
        }
            else{
                        mmc_debug_msg("Card pluged in\t\n");
        }

        plug_count += 1;
            return IRQ_HANDLED;
    }
    else{    
        if (plug_count < 5){ //5 is debounce count
            plug_count += 1;
            return IRQ_HANDLED;
        }
        else{
               plug_count = 0;
               return IRQ_HANDLED;
        }
    }
}


static void init_sd_mmc(void)
{
    int status;

    status = request_gio(GIO_SD_CARDDETECT);
        if (status){
                mmc_debug_msg("\tUnable to register gio1 %d\n",GIO_SD_CARDDETECT);
                }
        gio_enable_irq(GIO_SD_CARDDETECT,GIO_ANY_EDGE);
    
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

    val = driver_register(&mmci_driver);
    init_sd_mmc();
    return val;
}

static void __exit mmci_exit(void)
{
    unsigned short reg;
    plug_count = 0;
    dat3stat_new = 0;
    disable_interrupt();

        reg = inw(IO_MMC_MEM_CLK_CONTROL);
        rmb();
        outw(reg & 0xFEFF, IO_MMC_MEM_CLK_CONTROL);
        wmb();

    unrequest_gio(GIO_SD_CARDDETECT);
    free_irq(IRQ_GIO8,NULL);
    unrequest_gio(GIO_SDCARD_WP);
    driver_unregister(&mmci_driver);
}

module_init(mmci_init);
module_exit(mmci_exit);
module_param(fmax, uint, 0444);

#ifdef CONFIG_ARCH_ITDM320_20
static struct clk *clk_get(struct device *dev, const char *id)
{
    //mmc_debug_msg(" %s called \n",__FUNCTION__);
}
static void clk_put(struct clk *clk)
{
    //mmc_debug_msg(" %s called \n",__FUNCTION__);
}

static int clk_enable(struct clk *clk)
{
    //mmc_debug_msg(" %s called \n",__FUNCTION__);
    return 0;
}

static int clk_use(struct clk *clk)
{
    //mmc_debug_msg(" %s called \n",__FUNCTION__);
    return 0;
}
static unsigned long clk_get_rate(struct clk *clk)
{
    //mmc_debug_msg(" %s called \n",__FUNCTION__);
        //return clk->rate;
    return 0;
}
static void clk_unuse(struct clk *clk)
{
    mmc_debug_msg(" %s called \n",__FUNCTION__);
}

#endif

MODULE_DESCRIPTION("ARM PrimeCell PL180/181 Multimedia Card Interface driver");
MODULE_LICENSE("GPL");
