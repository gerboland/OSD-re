/*
 * drivers/mmc/omap24xx_mmc.c
 *
 * Driver for OMAP24xx MMC controller.
 *
 * Author: Texas Instruments
 *
  * Copyright (C) 2004 Texas Instruments.
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 * 
 * History:
 *
 * Jan 10 2006  Madhusudhan Chikksture - 2430 HSMMC controller driver
 */

#include <linux/config.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/string.h>
#include <linux/ioport.h>
#include <linux/spinlock.h>
#include <linux/timer.h>
#include <linux/blkdev.h>
#include <linux/i2c.h>

#include <linux/mmc/mmc.h>
#include <linux/mmc/host.h>
#include <linux/mmc/protocol.h>
#include <linux/mmc/card.h>

#include <asm/io.h>
#include <asm/irq.h>
#include <asm/dma.h>
#include <asm/hardware.h>
#include <asm/arch/bus.h>
#include <asm/arch/mux.h>
#include <asm/arch/gpio.h>
#include <asm/arch/dma.h>
#include <asm/arch/clock.h>
#include <asm/arch/menelaus.h>
#include <asm/arch/sys_info.h>
#include <asm/semaphore.h>

#include "omap243x_mmc.h"

static int initstream = 0;

/* global variables */
/* Time out Value for the mmc wait for command */
#define MMC_WAIT_TIMEOUT    10
#ifdef CONFIG_MMC_DEBUG
#define DBG(x...)       printk(x)
#else
#define DBG(x...)       do { } while (0)
#endif

/************************ Hotplug specific function prototypes and macros ****/
#define CONTROL_PADCONF_sdrc_a14  0x0030

#define GPIO_0           0
#define DEBOUNCE_ENABLE  1
#define DEBOUNCE_TIME    10

static void mmcpower_slot_enable(unsigned char reg, unsigned char val);

/* Power control APIs */
static __inline__ void disable_powertoslot2(void)
{
    mmcpower_slot_enable(DCDC_CTRL1, SLOT2_POWEROFF);
    mmcpower_slot_enable(DCDC_CTRL3, SLOT2_DESELECT);
}
static __inline__ void enable_powertoslot2(void)
{
    mmcpower_slot_enable(DCDC_CTRL1, SLOT2_POWERON);
    mmcpower_slot_enable(DCDC_CTRL3, SLOT2_SELECT);
}
static __inline__ void disable_powertoslot1(void)
{
    /*  Disable MMC SLOT1 and Power off
     *          */
    writeb(0x0, OMAP24XX_VA_SYSTEM_CONTROL_BASE + PBIAS_LITE);    /* set PBIAS LITE to zero */
    mmcpower_slot_enable(MCT_CTRL3, SLOT1_DESELECT);
    mmcpower_slot_enable(LDO_CTRL7, SLOT1_POWEROFF);
}
static __inline__ void enable_powertoslot1(void)
{
    /*
     ** For MMC1, Toggle PBios before every powerup
     *          */
    writeb(0x0, OMAP24XX_VA_SYSTEM_CONTROL_BASE + PBIAS_LITE);    /* set PBIAS LITE to zero */
    /* Enable SLOT1 and Power On
     *         */
    mmcpower_slot_enable(MCT_CTRL3, SLOT1_SELECT);
    mmcpower_slot_enable(LDO_CTRL7, SLOT1_POWERON);
    /* 200ms delay required for PBIAS configuration
     */
    mdelay(100);
    writeb(0x3, OMAP24XX_VA_SYSTEM_CONTROL_BASE + PBIAS_LITE);
}

/* Enable MMC slot and power to the socket
 */
void mmcpower_slot_enable(unsigned char reg, unsigned char val)
{
    struct i2c_adapter *adap;
    struct i2c_msg msg[1];
    unsigned char data[2];

    adap = i2c_get_adapter(0);
    msg->addr = 0x72;
    msg->flags = 0;
    msg->len = 2;
    msg->buf = data;
    data[0] = reg;
    data[1] = val;
    i2c_transfer(adap, msg, 1);
}

/************************************************************************/
static void
mmc_omap_start_command(struct mmc_omap_host *host, struct mmc_command *cmd)
{
    u32 cmdreg = 0;
    u32 resptype;
    u32 cmdtype;
    u32 arg;
    int counter = 1000;

    DBG("%s: CMD%d, argument 0x%08x", host->mmc->host_name, cmd->opcode,
        cmd->arg);
    if (cmd->flags & MMC_RSP_SHORT)
        DBG(", 32-bit response");
    if (cmd->flags & MMC_RSP_LONG)
        DBG(", 128-bit response");
    if (cmd->flags & MMC_RSP_CRC)
        DBG(", CRC");
    if (cmd->flags & MMC_RSP_BUSY)
        DBG(", busy notification");
    DBG("\n");
    host->cmd = cmd;

    cmdtype = 0;
    writel(0xffffffff, OMAP_HSMMC_STAT);    /* Clear status bits */
    writel(INT_EN_MASK, OMAP_HSMMC_ISE);
    writel(INT_EN_MASK, OMAP_HSMMC_IE);

    /* Protocol layer does not provide response type,
     * but our hardware needs to know exact type, not just size!
     *
     * Yes it does, but you have to look at all the flag bits.
     * Extra definitions have been added to make this more
     * obvious. --rmk
     */
#define RMASK (MMC_RSP_MASK|MMC_RSP_CRC)
    switch (cmd->flags & RMASK) {
    default:
    case MMC_RSP_NONE & RMASK:
        resptype = 0;
        break;
    case MMC_RSP_R1 & RMASK:
        /* resp 1, resp 1b */
        resptype = 2;
        break;
    case MMC_RSP_R2 & RMASK:
        /* resp 2 */
        resptype = 1;
        break;
    case MMC_RSP_R3 & RMASK:
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
    cmdreg = (cmd->opcode << 24) | (resptype << 16) | (cmdtype << 22);

    if (cmd->opcode == 17 || cmd->opcode == 18) {
        writel((readl(OMAP_HSMMC_CON) & EIGHT_BIT), OMAP_HSMMC_CON);
        writel((readl(OMAP_HSMMC_HCTL) & ONE_BIT), OMAP_HSMMC_HCTL);
        cmdreg |= DP_SELECT | DDIR | MSBS | BCE | DMA_EN;

    } else if (cmd->opcode == 24 || cmd->opcode == 25) {
        writel((readl(OMAP_HSMMC_CON) & EIGHT_BIT), OMAP_HSMMC_CON);
        writel((readl(OMAP_HSMMC_HCTL) & ONE_BIT), OMAP_HSMMC_HCTL);
        cmdreg |= DP_SELECT | MSBS | BCE | DMA_EN;
        cmdreg &= ~(DDIR);
    }

    if (cmd->opcode == 0 || cmd->opcode == 1 || cmd->opcode == 2)
        writel((readl(OMAP_HSMMC_CON) | OD), OMAP_HSMMC_CON);

    if (cmd->opcode == 0) {
        if (initstream == 0) {
            disable_irq(MMC_IRQ);
            writel((readl(OMAP_HSMMC_CON) |
                INIT_STREAM), OMAP_HSMMC_CON);
            writel(0x00000000, OMAP_HSMMC_CMD);
            while ((readl(OMAP_HSMMC_STAT) & CC) != 0x1
                   || (counter != 0))
                counter--;
            counter = 1000;
            writel((readl(OMAP_HSMMC_CON) &
                ~INIT_STREAM), OMAP_HSMMC_CON);
            writel((readl(OMAP_HSMMC_CON) |
                INIT_STREAM), OMAP_HSMMC_CON);
            writel(0x00000000, OMAP_HSMMC_CMD);
            while ((readl(OMAP_HSMMC_STAT) & CC) != 0x1
                   || (counter != 0))
                counter--;
            writel((readl(OMAP_HSMMC_CON) &
                ~INIT_STREAM), OMAP_HSMMC_CON);
            enable_irq(MMC_IRQ);
            initstream = 1;
        }
    }

    if (cmd->opcode == 1)
        cmd->arg = 0xffc000;

    arg |= cmd->arg & 0xffff;
    arg |= cmd->arg >> 16;
    writel(cmd->arg, OMAP_HSMMC_ARG);
    writel(cmdreg, OMAP_HSMMC_CMD);
}

static void
mmc_omap_xfer_done(struct mmc_omap_host *host, struct mmc_data *data)
{
    host->data = NULL;
    host->datadir = OMAP_MMC_DATADIR_NONE;

    if (data->error == MMC_ERR_NONE)
        data->bytes_xfered += data->blocks * (1 << data->blksz_bits);
    if (!data->stop) {
        host->mrq = NULL;
        mmc_request_done(host->mmc, data->mrq);
        return;
    }
    mmc_omap_start_command(host, data->stop);
}

static void
mmc_omap_cmd_done(struct mmc_omap_host *host, struct mmc_command *cmd)
{
    host->cmd = NULL;

    switch (cmd->flags & MMC_RSP_MASK) {
    case MMC_RSP_NONE:
        /* resp 0 */
        break;
    case MMC_RSP_SHORT:
        /* response types 1, 1b, 3, 4, 5, 6 */
        cmd->resp[0] = readl(OMAP_HSMMC_RSP10);
        DBG("%s: Response %08x\n", host->mmc->host_name, cmd->resp[0]);
        break;
    case MMC_RSP_LONG:
        /* response type 2 */
        cmd->resp[3] = readl(OMAP_HSMMC_RSP10);
        cmd->resp[2] = readl(OMAP_HSMMC_RSP32);
        cmd->resp[1] = readl(OMAP_HSMMC_RSP54);
        cmd->resp[0] = readl(OMAP_HSMMC_RSP76);
        DBG("%s: Response %08x %08x %08x %08x\n", host->mmc->host_name,
            cmd->resp[0], cmd->resp[1], cmd->resp[2], cmd->resp[3]);
        break;
    }

    if (host->data == NULL || cmd->error != MMC_ERR_NONE) {
        DBG("%s: End request, err %x\n", host->mmc->host_name,
            cmd->error);
        host->mrq = NULL;
        mmc_request_done(host->mmc, cmd->mrq);

    }

}

static irqreturn_t mmc_omap_irq(int irq, void *dev_id, struct pt_regs *regs)
{
    struct mmc_omap_host *host = (struct mmc_omap_host *)dev_id;
    u32 status;
    int end_command;
    int end_transfer;

    if (host->cmd == NULL && host->data == NULL) {
        status = readl(OMAP_HSMMC_STAT);
        if (status != 0) {
            writel(status, OMAP_HSMMC_STAT);
        }
        return IRQ_HANDLED;
    }

    end_command = 0;
    end_transfer = 0;
    if (host->cmd) {
        if (host->cmd->opcode == MMC_SELECT_CARD
            || host->cmd->opcode == MMC_SWITCH) {
            int counter = 1000;
            while (counter) {
                if ((readl(OMAP_HSMMC_STAT) & CC) == 0x1)
                    break;
                counter--;
            }
        }
    }
    status = readl(OMAP_HSMMC_STAT);
    if (status & (OMAP_HSMMC_CMD_TIMEOUT)) {
        if (host->cmd) {
            /* Timeouts are normal in case of MMC_SEND_STATUS */
            if (host->cmd->opcode != MMC_ALL_SEND_CID    /* &&
                                       !mmc_omap_cover_is_open(host) */ )
                ;                       
            host->cmd->error |= MMC_ERR_TIMEOUT;
            end_command = 1;
        }
    }

    if (status & (OMAP_HSMMC_DATA_TIMEOUT)) {
        printk("%s: Data timeout\n", host->mmc->host_name);
        if (host->data) {
            host->data->error |= MMC_ERR_TIMEOUT;
            end_transfer = 1;
        }
    }

    if ((status & (OMAP_HSMMC_CMD_CRC))) {
        printk("%s: Command CRC error\n", host->mmc->host_name);
        if (host->cmd) {
            host->cmd->error |= MMC_ERR_BADCRC;
            end_command = 1;
        }
    }

    if ((status & (OMAP_HSMMC_DATA_CRC))) {
        if (host->data) {
            host->data->error |= MMC_ERR_BADCRC;
            printk("%s: Data CRC error, bytes left %d\n",
                   host->mmc->host_name, host->bytesleft);
            end_transfer = 1;
        } else {
            printk(KERN_DEBUG "%s: Data CRC error\n",
                   host->mmc->host_name);
        }

    }

    if ((status & CC)) {
        end_command = 1;
    }

    if (status & TC) {
        end_transfer = 1;
    }
    writel(status, OMAP_HSMMC_STAT);

    if (end_command) {
        mmc_omap_cmd_done(host, host->cmd);
    }
    if (end_transfer) {
        mmc_omap_xfer_done(host, host->data);
    }
    return IRQ_HANDLED;
}

/*
 * DMA call back function
 */

static void mmc_omap_dma_cb(int lch, u16 ch_status, void *data)
{
    struct mmc_omap_host *host = (struct mmc_omap_host *)data;
    int dma_ch;
    /* FIXME: We ignore the possible errors for now. */
    if (host->dma_ch < 0) {
        printk(KERN_ERR "%s: DMA callback while DMA not enabled?\n",
               host->mmc->host_name);
        return;
    }
    dma_ch = host->dma_ch;
    host->dma_ch = -1;

    omap_free_dma(dma_ch);
    up(&host->sem);
}

static int mmc_omap_start_dma_transfer(struct mmc_omap_host *host,
                       struct mmc_request *req)
{
    int sync_dev, dma_ch, r;
    const char *dev_name;

    /* If for some reason the DMA transfer is still active,
     * we wait for timeout period and free the dma*/
    if (host->dma_ch != -1) {
        set_current_state(TASK_INTERRUPTIBLE);
        schedule_timeout(100);
        if (down_trylock(&host->sem)) {
            dma_ch = host->dma_ch;
            host->dma_ch = -1;
            omap_free_dma(dma_ch);
            up(&host->sem);
            return 1;
        }
    } else {
        if (down_trylock(&host->sem)) {
            printk("\n Semaphore was not initialized \n");
            BUG();
        }
    }

    if (!(req->data->flags & MMC_DATA_WRITE)) {
        sync_dev = OMAP_DMA_MMC_RX;
        dev_name = "MMC1 read";
    } else {
        sync_dev = OMAP_DMA_MMC_TX;
        dev_name = "MMC1 write";
    }

    r = omap_request_dma(sync_dev, dev_name, mmc_omap_dma_cb, host,
                 &dma_ch);

    if (r != 0) {
        printk("%s: omap_request_dma() failed with %d\n",
               host->mmc->host_name, r);
        return r;
    }

    if (!(req->data->flags & MMC_DATA_WRITE)) {
        /* 16 frames/block, 32 bytes/frame */

        omap_set_dma_src_params(dma_ch, 0x00, MMC_SRC, 0, 0);
        omap_set_dma_dest_params(dma_ch, 0x01,
                     virt_to_phys(req->data->req->buffer),
                     0, 0);
        r = 0;

    } else {

        omap_set_dma_dest_params(dma_ch, 0x00, MMC_SRC, 0, 0);
        omap_set_dma_src_params(dma_ch, 0x01,
                    virt_to_phys(req->data->req->buffer), 0,
                    0);
        r = 0;

    }
    omap_set_dma_transfer_params(dma_ch, OMAP_DMA_DATA_TYPE_S32, 128,
                     req->data->blocks, OMAP_DMA_SYNC_BLOCK,
                     sync_dev, r);
    host->dma_ch = dma_ch;
    omap_start_dma(dma_ch);
    return 0;
}

static void mmc_omap_prepare_data(struct mmc_omap_host *host,
                  struct mmc_request *req)
{
    host->data = req->data;

    if (req->data == NULL) {
        host->datadir = OMAP_MMC_DATADIR_NONE;
        writel(0, OMAP_HSMMC_BLK);
        return;
    }

    writel((1 << req->data->blksz_bits), OMAP_HSMMC_BLK);
    writel((readl(OMAP_HSMMC_BLK) | req->data->blocks << 16),
           OMAP_HSMMC_BLK);

    host->datadir = (req->data->flags & MMC_DATA_WRITE) ?
        OMAP_MMC_DATADIR_WRITE : OMAP_MMC_DATADIR_READ;

    if (host->use_dma && mmc_omap_start_dma_transfer(host, req) == 0) {
        host->buffer = NULL;
        host->bytesleft = 0;
    } else {
        /* Revert to CPU copy */
        host->buffer = (u16 *) req->data->req->buffer;
        host->bytesleft =
            req->data->blocks * (1 << req->data->blksz_bits);
        host->dma_ch = -1;
    }

}

static inline int is_broken_card(struct mmc_card *card)
{
    int i;
    struct mmc_cid *c = &card->cid;
    static const struct broken_card_cid {
        unsigned int manfid;
        char prod_name[8];
        unsigned char hwrev;
        unsigned char fwrev;
    } broken_cards[] = {
        {
    0x00150000, "\x30\x30\x30\x30\x30\x30\x15\x00", 0x06,
                0x03},};

    for (i = 0; i < sizeof(broken_cards) / sizeof(broken_cards[0]); i++) {
        const struct broken_card_cid *b = broken_cards + i;

        if (b->manfid != c->manfid)
            continue;
        if (memcmp(b->prod_name, c->prod_name, sizeof(b->prod_name)) !=
            0)
            continue;
        if (b->hwrev != c->hwrev || b->fwrev != c->fwrev)
            continue;
        return 1;
    }
    return 0;
}

static void omap24xx_mmc_request(struct mmc_host *mmc, struct mmc_request *req)
{
    struct mmc_omap_host *host = mmc_priv(mmc);

    WARN_ON(host->mrq != NULL);

    host->mrq = req;

    /* Some cards (vendor left unnamed to protect the guilty) seem to
     * require this delay after power-up. Otherwise we'll get mysterious
     * data timeouts. */
    if (req->cmd->opcode == MMC_SEND_CSD) {
        struct mmc_card *card;
        int broken_present = 0;

        list_for_each_entry(card, &mmc->cards, node) {
            if (is_broken_card(card)) {
                broken_present = 1;
                break;
            }
        }
        if (broken_present) {
            static int complained = 0;

            if (!complained) {
                printk(KERN_WARNING
                       "%s: Broken card workaround enabled\n",
                       host->mmc->host_name);
                complained = 1;
            }
            if (in_interrupt()) {
                /* This is nasty */
                printk(KERN_ERR
                       "Sleeping in IRQ handler, FIXME please!\n");
                dump_stack();
                mdelay(100);
            } else {
                set_current_state(TASK_UNINTERRUPTIBLE);
                schedule_timeout(100 * HZ / 1000);
            }
        }
    }
    mmc_omap_prepare_data(host, req);
    mmc_omap_start_command(host, req->cmd);
}

/*
 * PRCM clk setup
 */

static void mmc_clk_setup(int on)
{
}

/*  Disable clock to the card
 */
static void omap_mmc_stop_clock(void)
{
    /* Disable clock to the card
     */
    writel(readl(OMAP_HSMMC_SYSCTL) & ~(CEN), OMAP_HSMMC_SYSCTL);
    if ((readl(OMAP_HSMMC_SYSCTL) & CEN) != 0x0)
        printk
            ("\n MMC clock not stoped, clock freq can not be altered\n");
}

static void omap24xx_mmc_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
    u16 dsor = 0;
    int counter = 1000;
    unsigned long regVal;

    DBG("%s: set_ios: clock %dHz busmode %d powermode %d Vdd %x\n",
        host->mmc->host_name, ios->clock, ios->bus_mode, ios->power_mode,
        ios->vdd);

    if (ios->clock == 0) {
        /* Disable MMC_SD_CLK */
        mmc_clk_setup(0);
    } else {
        /* Enable MMC_SD_CLK */
        mmc_clk_setup(1);

        dsor = OMAP_MMC_MASTER_CLOCK / ios->clock;
        if (dsor < 1)
            dsor = 1;

        if (OMAP_MMC_MASTER_CLOCK / dsor > ios->clock)
            dsor++;

        if (dsor > 250)
            dsor = 250;

        if (dsor == 4)
            dsor = 5;
        if (dsor == 3)
            dsor = 4;
    }

    switch (ios->power_mode) {
    case MMC_POWER_OFF:
        break;
    case MMC_POWER_UP:
    case MMC_POWER_ON:
        break;
    }
    omap_mmc_stop_clock();
    regVal = readl(OMAP_HSMMC_SYSCTL);
    regVal = regVal & ~(CLKD_MASK);
    regVal = regVal | (dsor << 6);
    regVal = regVal | (DTO << 16);
    writel(regVal, OMAP_HSMMC_SYSCTL);
    writel(readl(OMAP_HSMMC_SYSCTL) | ICE, OMAP_HSMMC_SYSCTL);

    /* wait till the ICS bit is set
     * */
    while ((readl(OMAP_HSMMC_SYSCTL) & ICS) != 0x2 || (counter != 0)) {
        counter--;
    }
    /* Enable clock to the card
     * */
    writel(readl(OMAP_HSMMC_SYSCTL) | CEN, OMAP_HSMMC_SYSCTL);
}

static struct mmc_host_ops mmc_omap_ops = {
    .request = omap24xx_mmc_request,
    .set_ios = omap24xx_mmc_set_ios,
};

static int omap24xx_mmc_probe(struct omap_dev *dev)
{
    struct mmc_host *mmc;
    struct mmc_omap_host *host;
    int ret = 0;

    mmc = mmc_alloc_host(sizeof(struct mmc_omap_host), &dev->dev);
    if (!mmc) {
        ret = -ENOMEM;
        goto out;
    }

    host = mmc_priv(mmc);
    host->mmc = mmc;

    sema_init(&host->sem, 1);

    host->use_dma = OMAP_USE_DMA;
    host->dma_ch = -1;
    host->irq = MMC_IRQ;
    mmc->ops = &mmc_omap_ops;
    mmc->f_min = 400000;
    mmc->f_max = 24000000;
    mmc->ocr_avail = MMC_VDD_32_33 | MMC_VDD_33_34;

    ret = request_irq(host->irq, mmc_omap_irq, 0, DRIVER_NAME, host);
    if (ret) {
        printk("\nUnable to grab HSMMC IRQ");
        return ret;
    }

    writel(INT_EN_MASK, OMAP_HSMMC_ISE);
    writel(INT_EN_MASK, OMAP_HSMMC_IE);
    omap_set_drvdata(dev, mmc);
    mmc_add_host(mmc);
    return 0;

      out:
    return ret;
}

static int omap24xx_mmc_remove(struct omap_dev *dev)
{
    struct mmc_host *mmc = omap_get_drvdata(dev);
    struct mmc_omap_host *host = mmc_priv(mmc);

    omap_set_drvdata(dev, NULL);
    mmc_remove_host(mmc);
    free_irq(host->irq, host);
    del_timer_sync(&host->detect_timer);
    mmc_free_host(mmc);
    return 0;

}

#ifdef CONFIG_PM
static int omap24xx_mmc_suspend(struct omap_dev *dev, u32 state)
{
    struct mmc_omap_host *host = omap_get_drvdata(dev);
    struct clk *hsmmci, *hsmmcf;
    int ret = 0;

    if (host && host->suspended)
        return 0;

/* Todo -- Disable both the MMC abd CD IRQ's and stop the active dma
 */
/* Disable internel clock to the card
*/
    omap_mmc_stop_clock();
#ifdef CONFIG_OMAP2430_MMC1
    disable_powertoslot1();
    hsmmci = clk_get(NULL, "mmchs1_ick");
    hsmmcf = clk_get(NULL, "mmchs1_fck");
    clk_unuse(hsmmci);
    clk_unuse(hsmmcf);

/*  Disable MMC SLOT2 and Power off
 */
#elif CONFIG_OMAP2430_MMC2
    disable_powertoslot2();
    hsmmci = clk_get(NULL, "mmchs2_ick");
    hsmmcf = clk_get(NULL, "mmchs2_fck");
    clk_unuse(hsmmci);
    clk_unuse(hsmmcf);
#endif

    if (host) {
        ret = mmc_suspend_host(host->mmc, state);
        if (ret == 0)
            host->suspended = 1;
    }

    return ret;

}

static int omap24xx_mmc_resume(struct omap_dev *dev)
{
    struct mmc_omap_host *host = omap_get_drvdata(dev);
    struct clk *hsmmci, *hsmmcf;
    int ret = 0;

    if (host && !host->suspended)
        return 0;

#ifdef CONFIG_OMAP2430_MMC1
/* Enable SLOT1 and Power On
*/
    enable_powertoslot1();
    hsmmci = clk_get(NULL, "mmchs1_ick");
    hsmmcf = clk_get(NULL, "mmchs1_fck");
    clk_use(hsmmci);
    clk_use(hsmmcf);

/* Enable SLOT2 and Power On
 */
#elif CONFIG_OMAP2430_MMC2
    enable_powertoslot2();
    hsmmci = clk_get(NULL, "mmchs2_ick");
    hsmmcf = clk_get(NULL, "mmchs2_fck");
    clk_use(hsmmci);
    clk_use(hsmmcf);
#endif
/* Todo -- Enable the IRQ and start the clock
 */

    if (host) {
        ret = mmc_resume_host(host->mmc);
        if (ret == 0)
            host->suspended = 0;
    }

    return ret;
}

#else
#define omap24xx_mmc_suspend        NULL
#define omap24xx_mmc_resume         NULL
#endif

static void omap_24xx_release(struct device *dev)
{
    /* Nothing to be releaseed */
}

static struct omap_driver omap24xx_mmc_driver = {
    .drv = {
        .name = "omap2430-mmc",
        },
    .devid = OMAP24xx_MMC_DEVID,
    .busid = OMAP_BUS_L3,
    .clocks = 0,
    .probe = omap24xx_mmc_probe,
    .remove = omap24xx_mmc_remove,
    .suspend = omap24xx_mmc_suspend,
    .resume = omap24xx_mmc_resume,
};

static struct omap_dev omap24xx_mmc_device = {
    .name = "omap2430-mmc",
    .devid = OMAP24xx_MMC_DEVID,
    .dev = {
        .release = omap_24xx_release,
        },
    .busid = OMAP_BUS_L3,
    .mapbase = (void *)0x4809C000,
    .res = {
        .start = OMAP_HSMMC_BASE,
        .end = OMAP_HSMMC_BASE + 0x01FC,
        },
    .irq = {
        MMC_IRQ,
        },
};

static int __init omap24xx_mmc_init(void)
{
    struct clk *hsmmci, *hsmmcf;
#ifdef CONFIG_OMAP2430_MMC1
    /* Enable functional and interface clock to MMC1
     */
    enable_powertoslot1();
    hsmmci = clk_get(NULL, "mmchs1_ick");    /* mmc1 clks */
    hsmmcf = clk_get(NULL, "mmchs1_fck");
    clk_use(hsmmci);
    clk_use(hsmmcf);
    hsmmcf = clk_get(NULL, "mmchsdb1_fck");    /* mmc1 debounce clk */
    clk_safe(hsmmcf);
    writel(readl(OMAP_HSMMC_CAPA) | VS30, OMAP_HSMMC_CAPA);
    writel((readl(OMAP_HSMMC_HCTL) | SDVS30), OMAP_HSMMC_HCTL);

#elif CONFIG_OMAP2430_MMC2
    /* Enable SLOT2 and Power On
     */
    enable_powertoslot2();
    hsmmci = clk_get(NULL, "mmchs2_ick");    /* mmc2 clks */
    hsmmcf = clk_get(NULL, "mmchs2_fck");
    clk_use(hsmmci);
    clk_use(hsmmcf);
    hsmmcf = clk_get(NULL, "mmchsdb2_fck");    /* mmc2 debounce clk */
    clk_safe(hsmmcf);

    writel(readl(OMAP_HSMMC_CAPA) | VS18, OMAP_HSMMC_CAPA);
    writel((readl(OMAP_HSMMC_HCTL) | SDVS18), OMAP_HSMMC_HCTL);
#endif

    writel(readl(OMAP_HSMMC_SYSCONFIG) | SIDLE_MODE, OMAP_HSMMC_SYSCONFIG);

/*  Enable SD bus power
 *  */
    writel(readl(OMAP_HSMMC_HCTL) | SDBP, OMAP_HSMMC_HCTL);

    if (omap_driver_register(&omap24xx_mmc_driver)) {
        printk(KERN_ERR MMC_NAME ":failed to register MMC driver\n");
        return -ENODEV;
    }

    if (omap_device_register(&omap24xx_mmc_device)) {
        printk(KERN_ERR MMC_NAME ":failed to register MMC device\n");
        goto unregister_driver;
    }
    return 0;
      unregister_driver:
    omap_driver_unregister(&omap24xx_mmc_driver);
    return -ENODEV;

}

static void __exit omap24xx_mmc_cleanup(void)
{
    struct clk *hsmmci, *hsmmcf;

#ifdef CONFIG_OMAP2430_MMC1
    hsmmci = clk_get(NULL, "mmchs1_ick");    /* mmc1 clks */
    hsmmcf = clk_get(NULL, "mmchs1_fck");
    clk_unuse(hsmmci);
    clk_unuse(hsmmcf);
#elif CONFIG_OMAP2430_MMC2
    hsmmci = clk_get(NULL, "mmchs2_ick");    /* mmc2 clks */
    hsmmcf = clk_get(NULL, "mmchs2_fck");
    clk_unuse(hsmmci);
    clk_unuse(hsmmcf);
#endif
    omap_driver_unregister(&omap24xx_mmc_driver);
    omap_device_unregister(&omap24xx_mmc_device);

}

module_init(omap24xx_mmc_init);
module_exit(omap24xx_mmc_cleanup);

MODULE_DESCRIPTION("OMAP Multimedia Card driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Texas Instruments");
