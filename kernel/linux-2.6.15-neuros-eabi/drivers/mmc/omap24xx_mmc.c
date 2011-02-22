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
 * 2004/10/07 Madhusudhan Chikksture - Modified to integrate MMC hotplug requirements on
 *                                     2420 and Menelaus platforms
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

#include "omap24xx_mmc.h"

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
/************************************************************************/
static __inline__ u8 omap_mmc_pin_out(u32 offset, u8 val)
{
    return writeb(val, OMAP24XX_VA_SYSTEM_CONTROL_BASE + offset);
}

static void
mmc_omap_start_command(struct mmc_omap_host *host, struct mmc_command *cmd)
{
    u32 cmdreg;
    u32 resptype;
    u32 cmdtype;

    udelay(500);

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
        resptype = 1;
        break;
    case MMC_RSP_R2 & RMASK:
        /* resp 2 */
        resptype = 2;
        break;
    case MMC_RSP_R3 & RMASK:
        resptype = 3;
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
        cmdtype = OMAP_MMC_CMDTYPE_ADTC;
    } else if (resptype == 0 && cmd->opcode != 15) {
        cmdtype = OMAP_MMC_CMDTYPE_BC;
    } else if (host->bus_mode == MMC_BUSMODE_OPENDRAIN) {
        cmdtype = OMAP_MMC_CMDTYPE_BCR;
    } else {
        cmdtype = OMAP_MMC_CMDTYPE_AC;
    }

    cmdreg = cmd->opcode | (resptype << 8) | (cmdtype << 12);

    if (host->bus_mode == MMC_BUSMODE_OPENDRAIN)
        cmdreg |= 1 << 6;

    if (cmd->flags & MMC_RSP_BUSY)
        cmdreg |= 1 << 11;

    if (host->datadir == OMAP_MMC_DATADIR_READ)
        cmdreg |= 1 << 15;

    OMAP_MMC_WRITE(host->base, CTO, 200);
    OMAP_MMC_WRITE(host->base, ARGL, cmd->arg & 0xffff);
    OMAP_MMC_WRITE(host->base, ARGH, cmd->arg >> 16);
    OMAP_MMC_WRITE(host->base, IE,
               OMAP_MMC_STAT_A_EMPTY | OMAP_MMC_STAT_A_FULL |
               OMAP_MMC_STAT_CMD_CRC | OMAP_MMC_STAT_CMD_TOUT |
               OMAP_MMC_STAT_DATA_CRC | OMAP_MMC_STAT_DATA_TOUT |
               OMAP_MMC_STAT_END_OF_CMD | OMAP_MMC_STAT_CARD_ERR |
               OMAP_MMC_STAT_END_OF_DATA);
    OMAP_MMC_WRITE(host->base, CMD, cmdreg);

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
        cmd->resp[0] =
            OMAP_MMC_READ(host->base, RSP6) |
            (OMAP_MMC_READ(host->base, RSP7) << 16);
        DBG("%s: Response %08x\n", host->mmc->host_name, cmd->resp[0]);
        break;
    case MMC_RSP_LONG:
        /* response type 2 */
        cmd->resp[3] =
            OMAP_MMC_READ(host->base, RSP0) |
            (OMAP_MMC_READ(host->base, RSP1) << 16);
        cmd->resp[2] =
            OMAP_MMC_READ(host->base, RSP2) |
            (OMAP_MMC_READ(host->base, RSP3) << 16);
        cmd->resp[1] =
            OMAP_MMC_READ(host->base, RSP4) |
            (OMAP_MMC_READ(host->base, RSP5) << 16);
        cmd->resp[0] =
            OMAP_MMC_READ(host->base, RSP6) |
            (OMAP_MMC_READ(host->base, RSP7) << 16);
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
    u16 status;
    int end_command;
    int end_transfer;
    int ii;

    if (host->cmd == NULL && host->data == NULL) {
        status = OMAP_MMC_READ(host->base, STAT);
        printk(KERN_INFO "%s: Spurious interrupt 0x%04x\n",
               host->mmc->host_name, status);
        if (status != 0) {
            OMAP_MMC_WRITE(host->base, STAT, status);
            OMAP_MMC_WRITE(host->base, IE, 0);
        }
        return IRQ_HANDLED;
    }

    end_command = 0;
    end_transfer = 0;

    while ((status = OMAP_MMC_READ(host->base, STAT)) != 0) {
        OMAP_MMC_WRITE(host->base, STAT, status);    // Reset status bits
        DBG("\tMMC IRQ %04x\n", status);

        if ((status & OMAP_MMC_STAT_A_FULL) ||
            ((status & OMAP_MMC_STAT_END_OF_DATA) &&
             (host->bytesleft > 0))) {
            // Buffer almost full
            ii = host->bytesleft / 2;
            if (ii > 32)
                ii = 32;
            host->bytesleft -= ii * 2;
            while (ii-- > 0)
                *host->buffer++ =
                    OMAP_MMC_READ(host->base, DATA);
        }

        if (status & OMAP_MMC_STAT_A_EMPTY) {
            // Buffer almost empty
            ii = host->bytesleft / 2;
            if (ii > 32)
                ii = 32;
            host->bytesleft -= ii * 2;
            while (ii-- > 0)
                OMAP_MMC_WRITE(host->base, DATA,
                           *host->buffer++);
        }

        if (status & OMAP_MMC_STAT_END_OF_DATA) {
            // Block sent/received
            end_transfer = 1;
        }

        if (status & OMAP_MMC_STAT_DATA_TOUT) {
            // Data timeout
            printk(KERN_DEBUG "%s: Data timeout\n",
                   host->mmc->host_name);
            if (host->data) {
                host->data->error |= MMC_ERR_TIMEOUT;
                end_transfer = 1;
            }
        }

        if (status & OMAP_MMC_STAT_DATA_CRC) {
            // Data CRC error
            if (host->data) {
                host->data->error |= MMC_ERR_BADCRC;
                printk(KERN_DEBUG
                       "%s: Data CRC error, bytes left %d\n",
                       host->mmc->host_name, host->bytesleft);
                end_transfer = 1;
            } else {
                printk(KERN_DEBUG "%s: Data CRC error\n",
                       host->mmc->host_name);
            }
        }

        if (status & OMAP_MMC_STAT_CMD_TOUT) {
            // Command timeout
            if (host->cmd) {
                /* Timeouts are normal in case of MMC_SEND_STATUS */
                if (host->cmd->opcode != MMC_ALL_SEND_CID    /* &&
                                           !mmc_omap_cover_is_open(host) */ )
                    printk(KERN_DEBUG
                           "%s: Command timeout, CMD%d\n",
                           host->mmc->host_name,
                           host->cmd->opcode);
                host->cmd->error |= MMC_ERR_TIMEOUT;
                end_command = 1;
            }
        }

        if (status & OMAP_MMC_STAT_CMD_CRC) {

            /* Command CRC error */
            printk(KERN_DEBUG "%s: Command CRC error\n",
                   host->mmc->host_name);
            if (host->cmd) {
                host->cmd->error |= MMC_ERR_BADCRC;
                end_command = 1;
            }
        }

        if (status & OMAP_MMC_STAT_OCR_BUSY) {
            // OCR Busy
            if (host->cmd && host->cmd->opcode != MMC_SEND_OP_COND
                && host->cmd->opcode != MMC_SET_RELATIVE_ADDR) {
                printk(KERN_DEBUG
                       "%s: OCR busy error, CMD%d\n",
                       host->mmc->host_name, host->cmd->opcode);
            }
        }

        if (status & OMAP_MMC_STAT_CARD_ERR) {
            // Card status error
            printk(KERN_DEBUG "%s: Card status error\n",
                   host->mmc->host_name);
            if (host->cmd) {
                host->cmd->error |= MMC_ERR_FAILED;
                end_command = 1;
            }
            if (host->data) {
                host->data->error |= MMC_ERR_FAILED;
                end_transfer = 1;
            }
        }

        /*
         * NOTE: On 1610 the END_OF_CMD may come too early when
         *       starting a write 
         */
        if ((status & OMAP_MMC_STAT_END_OF_CMD) &&
            (!(status & OMAP_MMC_STAT_A_EMPTY))) {
            // End of command phase
            end_command = 1;
        }
    }
    if (end_command) {
        mmc_omap_cmd_done(host, host->cmd);
    }
    if (end_transfer) {
        mmc_omap_xfer_done(host, host->data);
    }

    return IRQ_HANDLED;
}

static void mmc_omap_detect(unsigned long data)
{
    struct mmc_host *mmc = (struct mmc_host *)data;
    mmc_detect_change(mmc);
}

/* Interrupt service routine for handling card insertion and removal
 */
static irqreturn_t mmc_omap_irq_cd(int irq, void *dev_id, struct pt_regs *regs)
{
    struct mmc_omap_host *host = dev_id;
    u32 board = 0;
    u32 card_status = 0;
    int reg = 0;

    board = get_board_type();
    if (board == BOARD_H4_MENELAUS) {
        menelaus_mmcread_err(&reg, MENELAUS_MCT_PIN_ST, return -EIO);
        if ((reg & 0x01) == 1)
            card_status = 1;
        else
            card_status = 0;
    } else
        card_status = omap_get_gpio_datain(GPIO_0);

    if (card_status) {

        /*
         * remove - we need to delay 50ms since the socket
         * tells us before the card has physically disconnected.
         */

        mod_timer(&host->detect_timer, jiffies + 5 * HZ / 100);
    } else {

        /*
         * insert - we can do this immediately, but make
         * sure the timer is stopped.
         */

        del_timer(&host->detect_timer);
        mmc_detect_change(host->mmc);
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

        sync_dev = MMC_DMA_RX_REQUEST + 1;
        dev_name = "MMC1 read";
    } else {
        sync_dev = MMC_DMA_TX_REQUEST + 1;
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

        omap_set_dma_src_params(dma_ch, 0x00,
                    (OMAP_MMC_BASE + OMAP_MMC_REG_DATA), 0,
                    0);
#warning BUG: Do NOT use virt_to_phys - use the DMA API interface
        omap_set_dma_dest_params(dma_ch, 0x01,
                     virt_to_phys(req->data->req->buffer),
                     0, 0);

        OMAP_MMC_WRITE(host->base, BUF, 0x8f0f);

        r = 0;

    } else {

        omap_set_dma_dest_params(dma_ch, 0x00,
                     (OMAP_MMC_BASE + OMAP_MMC_REG_DATA), 0,
                     0);
#warning BUG: Do NOT use virt_to_phys - use the DMA API interface
        omap_set_dma_src_params(dma_ch, 0x01,
                    virt_to_phys(req->data->req->buffer), 0,
                    0);

        OMAP_MMC_WRITE(host->base, BUF, 0x0f8f);

        r = 0;

    }
    omap_set_dma_transfer_params(dma_ch, OMAP_DMA_DATA_TYPE_S16, 16,
                     16 * req->data->blocks, 0x02, sync_dev, r);
    host->dma_ch = dma_ch;
    omap_start_dma(dma_ch);

    return 0;
}

static void mmc_omap_prepare_data(struct mmc_omap_host *host,
                  struct mmc_request *req)
{
    int timeout;

    host->data = req->data;

    if (req->data == NULL) {
        host->datadir = OMAP_MMC_DATADIR_NONE;
        OMAP_MMC_WRITE(host->base, BLEN, 0);
        OMAP_MMC_WRITE(host->base, NBLK, 0);
        OMAP_MMC_WRITE(host->base, BUF, 0);
        return;
    }

    DBG("%s: Data xfer (%s %s), DTO %d cycles + %d ns, %d blocks of %d bytes\n", host->mmc->host_name, (req->data->flags & MMC_DATA_STREAM) ? "stream" : "block", (req->data->flags & MMC_DATA_WRITE) ? "write" : "read", req->data->timeout_clks, req->data->timeout_ns, req->data->blocks, 1 << req->data->blksz_bits);

    /* Convert ns to clock cycles by assuming 20MHz frequency
     * 1 cycle at 20MHz = 500 ns
     */
    timeout = req->data->timeout_clks + req->data->timeout_ns / 500;
    if (timeout > 0xffff)
        timeout = 0xffff;

    OMAP_MMC_WRITE(host->base, DTO, timeout);
    OMAP_MMC_WRITE(host->base, NBLK, req->data->blocks - 1);
    OMAP_MMC_WRITE(host->base, BLEN, (1 << req->data->blksz_bits) - 1);

    host->datadir = (req->data->flags & MMC_DATA_WRITE) ?
        OMAP_MMC_DATADIR_WRITE : OMAP_MMC_DATADIR_READ;

    if (host->use_dma && mmc_omap_start_dma_transfer(host, req) == 0) {
        host->buffer = NULL;
        host->bytesleft = 0;
    } else {
        /* Revert to CPU copy */
        OMAP_MMC_WRITE(host->base, BUF, 0x1f1f);

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
 * Turn the on/off the power to the MMC socket.
 */
static void mmc_omap_power(struct mmc_omap_host *host, int on)
{
    unsigned long reg_val;
    if (on) {
        reg_val = OMAP_MMC_READ(host->base, CON);
        OMAP_MMC_WRITE(host->base, CON, reg_val | (0x1 << 11));
    } else {
        reg_val = OMAP_MMC_READ(host->base, CON);
        OMAP_MMC_WRITE(host->base, CON, reg_val | (0x0 << 11));
    }

}

/*
 * PRCM clk setup
 */

static void mmc_clk_setup(int on)
{
#if 0
    unsigned long reg_val;

    if (on) {
        reg_val = omap_prcmreg_read(PRCM_CLK_EN_PLL);
        omap_prcmreg_write(reg_val | (0xf << 0), PRCM_CLK_EN_PLL);

        reg_val = omap_prcmreg_read(PRCM_CLK_SEL1_PLL);
        omap_prcmreg_write(reg_val | (0 << 3), PRCM_CLK_SEL1_PLL);

        reg_val = omap_prcmreg_read(PRCM_FCLK_EN1_CORE);
        omap_prcmreg_write(reg_val | (1 << 26), PRCM_FCLK_EN1_CORE);

        reg_val = omap_prcmreg_read(PRCM_FCLK_EN2_CORE);
        omap_prcmreg_write(reg_val | (1 << 26), PRCM_FCLK_EN2_CORE);

    } else {
        reg_val = omap_prcmreg_read(PRCM_CLK_EN_PLL);
        omap_prcmreg_write(reg_val | (0x0 << 0), PRCM_CLK_EN_PLL);

        reg_val = omap_prcmreg_read(PRCM_CLK_SEL1_PLL);
        omap_prcmreg_write(reg_val | (0 << 3), PRCM_CLK_SEL1_PLL);

        reg_val = omap_prcmreg_read(PRCM_FCLK_EN1_CORE);
        omap_prcmreg_write(reg_val | (0 << 26), PRCM_FCLK_EN1_CORE);

        reg_val = omap_prcmreg_read(PRCM_FCLK_EN2_CORE);
        omap_prcmreg_write(reg_val | (0 << 26), PRCM_FCLK_EN2_CORE);
    }
#endif
}

static void omap24xx_mmc_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
    struct mmc_omap_host *host = mmc_priv(mmc);
    int dsor = 0;

    DBG("%s: set_ios: clock %dHz busmode %d powermode %d Vdd %x\n",
        host->mmc->host_name, ios->clock, ios->bus_mode, ios->power_mode,
        ios->vdd);

    if (ios->clock == 0) {
        /* Disable MMC_SD_CLK */
        mmc_clk_setup(0);
    } else {
        /* Enable MMC_SD_CLK */
        mmc_clk_setup(1);

        dsor = MMC_REF_CLOCK / ios->clock;
        if (dsor < 1)
            dsor = 1;

        if (MMC_REF_CLOCK / dsor > ios->clock)
            dsor++;

        if (dsor > 250)
            dsor = 250;
    }

    switch (ios->power_mode) {
    case MMC_POWER_OFF:
        dsor |= 0 << 11;
        break;
    case MMC_POWER_UP:
    case MMC_POWER_ON:
        dsor |= 1 << 11;
        break;
    }

    host->bus_mode = ios->bus_mode;

    OMAP_MMC_WRITE(host->base, CON, dsor);

    /*
     * MMC_POWER_UP means "apply power" only.  It does *not* mean start
     * clocks.  This is what MMC_POWER_ON means.
     */
    if (ios->power_mode == MMC_POWER_ON) {
        int wait_counter = 0;
        /* Send clock cycles, poll completion */
        OMAP_MMC_WRITE(host->base, IE, 0);
        OMAP_MMC_WRITE(host->base, STAT, 0xffff);
        OMAP_MMC_WRITE(host->base, CMD, 1 << 7);

        /* We wait for the mmc command to be completed. If its not done, then the following 
         * timeout mechanism will ensure that the system does not hang  
         */

        while ((0 == (OMAP_MMC_READ(host->base, STAT) & 1))
               && (wait_counter < MMC_WAIT_TIMEOUT)) {
            wait_counter++;
            set_current_state(TASK_INTERRUPTIBLE);
            schedule_timeout(10);
        }
        if (MMC_WAIT_TIMEOUT == wait_counter) {
            printk(KERN_ERR
                   "%s: Timed out waiting for cmd status - further results undefined\n",
                   __FUNCTION__);
            return;
        }
        OMAP_MMC_WRITE(host->base, STAT, 1);
    }

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
    u32 board = 0;

    mmc = mmc_alloc_host(sizeof(struct mmc_omap_host), &dev->dev);
    if (!mmc) {
        ret = -ENOMEM;
        goto out;
    }

    host = mmc_priv(mmc);
    host->mmc = mmc;

    sema_init(&host->sem, 1);

    init_timer(&host->detect_timer);
    host->detect_timer.function = mmc_omap_detect;
    host->detect_timer.data = (unsigned long)mmc;

    host->use_dma = OMAP_USE_DMA;
    host->dma_ch = -1;
    host->irq = OMAP_MMC_IRQ;
    host->base = ioremap(OMAP_MMC_BASE, MMC_REG_SIZE);
    if (!host->base) {
        printk("MMC: cannot map MMIO\n");
        goto free;
    }

    mmc->ops = &mmc_omap_ops;
    mmc->f_min = 400000;
    mmc->f_max = 24000000;
    mmc->ocr_avail = MMC_VDD_32_33 | MMC_VDD_33_34;

    omap2_cfg_reg(H15_2420_MMC_CLKI);
    omap2_cfg_reg(G19_2420_MMC_CLKO);
    omap2_cfg_reg(H18_2420_MMC_CMD);
    omap2_cfg_reg(F20_2420_MMC_DAT);
    omap2_cfg_reg(F19_2420_MMC_DATDIR);
    omap2_cfg_reg(G18_2420_MMC_CMDDIR);

    ret = request_irq(host->irq, mmc_omap_irq, 0, DRIVER_NAME, host);
    if (ret)
        goto unmap;

    /*
     * Request an irq for card detection
     */

    board = get_board_type();
    if (board == BOARD_H4_MENELAUS) {
        ret =
            request_irq(MMC_CARDDETECT_IRQ, mmc_omap_irq_cd,
                SA_INTERRUPT, DRIVER_NAME, host);
        enable_irq(MMC_CARDDETECT_IRQ);
    }

    else {
        ret =
            request_irq(MMC_HP_IRQ, mmc_omap_irq_cd, 0, DRIVER_NAME,
                host);
    }

    if (ret)
        goto unirq;

    omap_set_drvdata(dev, mmc);
    mmc_add_host(mmc);
    return 0;

      unirq:
    free_irq(host->irq, host);
      unmap:
    iounmap(host->base);
      free:
    mmc_free_host(mmc);
      out:
    return ret;
}

static int omap24xx_mmc_remove(struct omap_dev *dev)
{
    struct mmc_host *mmc = omap_get_drvdata(dev);
    struct mmc_omap_host *host = mmc_priv(mmc);
    u32 board = 0;

    omap_set_drvdata(dev, NULL);

    mmc_remove_host(mmc);

    board = get_board_type();
    if (board == BOARD_H4_MENELAUS)
        free_irq(MMC_CARDDETECT_IRQ, host);
    else
        free_irq(MMC_HP_IRQ, host);
    free_irq(host->irq, host);

    del_timer_sync(&host->detect_timer);

    mmc_omap_power(host, 0);

    iounmap(host->base);

    mmc_free_host(mmc);

    return 0;

}

#ifdef CONFIG_PM
static int omap24xx_mmc_suspend(struct omap_dev *dev, u32 state)
{
        int ret = 0;
        struct mmc_omap_host *host = omap_get_drvdata(dev);
        if (host && host->suspended)
                return 0;

        if (host) {
                ret = mmc_suspend_host(host->mmc, state);
                if (ret == 0)
                        host->suspended = 1;
        }
        return ret;

}

static int omap24xx_mmc_resume(struct omap_dev *dev)
{
       int ret = 0;
        int wait_counter = 0;
        struct mmc_omap_host *host = omap_get_drvdata(dev);
        if (host && !host->suspended)
                return 0;

        if (host) {
                ret = mmc_resume_host(host->mmc);
                if (ret == 0)
                        host->suspended = 0;
        }

        while ((0 == (OMAP_MMC_READ(host->base, CON) & (1<<11)))
               && (wait_counter < MMC_WAIT_TIMEOUT)) {
                wait_counter++;
                 msleep_interruptible(1);
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
        .name = DRIVER_NAME,
        },
    .devid = OMAP24xx_MMC_DEVID,
    .busid = OMAP_BUS_L4,
    .clocks = 0,
    .probe = omap24xx_mmc_probe,
    .remove = omap24xx_mmc_remove,
    .suspend = omap24xx_mmc_suspend,
    .resume = omap24xx_mmc_resume,
};

static struct omap_dev omap24xx_mmc_device = {
    .name = "omap-mmc",
    .devid = OMAP24xx_MMC_DEVID,
    .dev = {
        .release = omap_24xx_release,
        },
    .busid = OMAP_BUS_L4,
    .mapbase = (void *)OMAP_MMC_BASE,
    .res = {
        .start = IO_ADDRESS(OMAP_MMC_BASE),
        .end = IO_ADDRESS(OMAP_MMC_BASE) + 0x7f,
        },
    .irq = {
        OMAP_MMC_IRQ,
        },
};

static int __init omap24xx_mmc_init(void)
{
    int ret;
    u32 board;
    int reg = 0;
    struct clk *mmci, *mmcf;

    board = get_board_type();
    if (board == BOARD_H4_MENELAUS) {
        /* This is to enable slot 1 and slot 2 on menelaus boards.
         */
        menelaus_mmcread_err(&reg, MENELAUS_MCT_CTRL3, return -EIO);
        reg |= ENABLE_BOTH_SLOTS;
        menelaus_mmcwrite_err(reg, MENELAUS_MCT_CTRL3, return -EIO);

        /* This is to enable Card detect input buffer for both slots
         */
        menelaus_mmcread_err(&reg, MENELAUS_MCT_CTRL2, return -EIO);
        reg |= 0xF0;
        menelaus_mmcwrite_err(reg, MENELAUS_MCT_CTRL2, return -EIO);

        menelaus_mmcread_err(&reg, MENELAUS_MCT_CTRL1, return -EIO);
        reg &= 0xFC;
        menelaus_mmcwrite_err(reg, MENELAUS_MCT_CTRL1, return -EIO);

        /* This is to set the VMMC mode  */
        menelaus_mmcread_err(&reg, MENELAUS_LDO_CTRL7, return -EIO);
        reg |= 0x03;
        menelaus_mmcwrite_err(reg, MENELAUS_LDO_CTRL7, return -EIO);

    }

    ret = omap_driver_register(&omap24xx_mmc_driver);
    if (ret != 0)
        return ret;

    ret = omap_device_register(&omap24xx_mmc_device);
    if (ret != 0)
        goto err1;

    if (board == BOARD_H4_MENELAUS) {
        /* Enable the Card Detect signal Debounce and buffer */
        menelaus_mmcread_err(&reg, MENELAUS_MCT_CTRL2, return -EIO);
        reg |= 0xF0;
        menelaus_mmcwrite_err(reg, MENELAUS_MCT_CTRL2, return -EIO);

    } else {
        omap_mmc_pin_out(CONTROL_PADCONF_sdrc_a14, 0x3);
        omap_set_gpio_debounce(GPIO_0, DEBOUNCE_ENABLE);
        omap_set_gpio_debounce_time(GPIO_0, DEBOUNCE_TIME);
        omap_set_gpio_direction(GPIO_0, OMAP24XX_DIR_INPUT);
        omap_set_gpio_edge_ctrl(GPIO_0, OMAP_GPIO_BOTH_EDGES);
        gpio_unmask_irq(GPIO_0);
    }
    mmci = clk_get(NULL, "mmc_ick");
    mmcf = clk_get(NULL, "mmc_fck");
    clk_use(mmci);
    clk_use(mmcf);
    clk_safe(mmcf);
                     
    return 0;

      err1:
    omap_driver_unregister(&omap24xx_mmc_driver);
    return ret;
}

static void __exit omap24xx_mmc_cleanup(void)
{
    omap_device_unregister(&omap24xx_mmc_device);

    omap_driver_unregister(&omap24xx_mmc_driver);
}

module_init(omap24xx_mmc_init);
module_exit(omap24xx_mmc_cleanup);

MODULE_DESCRIPTION("OMAP Multimedia Card driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Texas Instruments");
