/*
 * linux/drivers/i2c/i2c-omap.c
 *
 * TI OMAP I2C master mode driver
 *
 * Copyright (C) 2003 MontaVista Software, Inc.
 * Copyright (C) 2004 Texas Instruments.
 *
 * Updated to work with multiple I2C interfaces on 24xx by
 * Tony Lindgren <tony@atomide.com> and Imre Deak <imre.deak@nokia.com>
 * Copyright (C) 2005 Nokia Corporation
 *
 * Cleaned up by Juha Yrjölä <juha.yrjola@nokia.com>
 *
 * ----------------------------------------------------------------------------
 * This file was highly leveraged from i2c-elektor.c:
 *
 * Copyright 1995-97 Simon G. Vogl
 *           1998-99 Hans Berglund
 *
 * With some changes from Kyösti Mälkki <kmalkki@cc.hut.fi> and even
 * Frodo Looijaard <frodol@dds.nl>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#define USE_BIG_BAD_BIT_BANG_HACK 1     // UBBBBH enable

// #define DEBUG

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/completion.h>
#include <linux/platform_device.h>

#include <asm/io.h>
#include <asm/hardware/clock.h>

/* ----- global defines ----------------------------------------------- */
static const char driver_name[] = "i2c_omap";

#define MODULE_NAME "OMAP I2C"
#define OMAP_I2C_TIMEOUT (msecs_to_jiffies(500)) /* timeout waiting for the controller to respond */

#define DEFAULT_OWN             1    /* default own I2C address */
#define MAX_MESSAGES            65536    /* max number of messages */

#define OMAP_I2C_REV_REG        0x00
#define OMAP_I2C_IE_REG         0x04
#define OMAP_I2C_STAT_REG       0x08
#define OMAP_I2C_IV_REG         0x0c
#define OMAP_I2C_SYSS_REG       0x10
#define OMAP_I2C_BUF_REG        0x14
#define OMAP_I2C_CNT_REG        0x18
#define OMAP_I2C_DATA_REG       0x1c
#define OMAP_I2C_SYSC_REG       0x20
#define OMAP_I2C_CON_REG        0x24
#define OMAP_I2C_OA_REG         0x28
#define OMAP_I2C_SA_REG         0x2c
#define OMAP_I2C_PSC_REG        0x30
#define OMAP_I2C_SCLL_REG       0x34
#define OMAP_I2C_SCLH_REG       0x38
#define OMAP_I2C_SYSTEST_REG    0x3c

/* I2C Interrupt Enable Register (OMAP_I2C_IE): */
#define OMAP_I2C_IE_XRDY    (1 << 4)    /* TX data ready int enable */
#define OMAP_I2C_IE_RRDY    (1 << 3)    /* RX data ready int enable */
#define OMAP_I2C_IE_ARDY    (1 << 2)    /* Access ready int enable */
#define OMAP_I2C_IE_NACK    (1 << 1)    /* No ack interrupt enable */
#define OMAP_I2C_IE_AL      (1 << 0)    /* Arbitration lost int ena */

/* I2C Status Register (OMAP_I2C_STAT): */
#define OMAP_I2C_STAT_SBD   (1 << 15)    /* Single byte data */
#define OMAP_I2C_STAT_BB    (1 << 12)    /* Bus busy */
#define OMAP_I2C_STAT_ROVR  (1 << 11)    /* Receive overrun */
#define OMAP_I2C_STAT_XUDF  (1 << 10)    /* Transmit underflow */
#define OMAP_I2C_STAT_AAS   (1 << 9)    /* Address as slave */
#define OMAP_I2C_STAT_AD0   (1 << 8)    /* Address zero */
#define OMAP_I2C_STAT_XRDY  (1 << 4)    /* Transmit data ready */
#define OMAP_I2C_STAT_RRDY  (1 << 3)    /* Receive data ready */
#define OMAP_I2C_STAT_ARDY  (1 << 2)    /* Register access ready */
#define OMAP_I2C_STAT_NACK  (1 << 1)    /* No ack interrupt enable */
#define OMAP_I2C_STAT_AL    (1 << 0)    /* Arbitration lost int ena */

/* I2C Buffer Configuration Register (OMAP_I2C_BUF): */
#define OMAP_I2C_BUF_RDMA_EN    (1 << 15)    /* RX DMA channel enable */
#define OMAP_I2C_BUF_XDMA_EN    (1 << 7)    /* TX DMA channel enable */

/* I2C Configuration Register (OMAP_I2C_CON): */
#define OMAP_I2C_CON_EN     (1 << 15)    /* I2C module enable */
#define OMAP_I2C_CON_BE     (1 << 14)    /* Big endian mode */
#define OMAP_I2C_CON_STB    (1 << 11)    /* Start byte mode (master) */
#define OMAP_I2C_CON_MST    (1 << 10)    /* Master/slave mode */
#define OMAP_I2C_CON_TRX    (1 << 9)    /* TX/RX mode (master only) */
#define OMAP_I2C_CON_XA     (1 << 8)    /* Expand address */
#define OMAP_I2C_CON_RM     (1 << 2)    /* Repeat mode (master only) */
#define OMAP_I2C_CON_STP    (1 << 1)    /* Stop cond (master only) */
#define OMAP_I2C_CON_STT    (1 << 0)    /* Start condition (master) */

/* I2C System Test Register (OMAP_I2C_SYSTEST): */
#define OMAP_I2C_SYSTEST_ST_EN        (1 << 15)    /* System test enable */
#define OMAP_I2C_SYSTEST_FREE         (1 << 14)    /* Free running mode */
#define OMAP_I2C_SYSTEST_TMODE_MASK   (3 << 12)    /* Test mode select */
#define OMAP_I2C_SYSTEST_TMODE_SHIFT  (12)        /* Test mode select */
#define OMAP_I2C_SYSTEST_SCL_I        (1 << 3)    /* SCL line sense in */
#define OMAP_I2C_SYSTEST_SCL_O        (1 << 2)    /* SCL line drive out */
#define OMAP_I2C_SYSTEST_SDA_I        (1 << 1)    /* SDA line sense in */
#define OMAP_I2C_SYSTEST_SDA_O        (1 << 0)    /* SDA line drive out */

/* I2C System Status register (OMAP_I2C_SYSS): */
#define OMAP_I2C_SYSS_RDONE        1        /* Reset Done */

/* I2C System Configuration Register (OMAP_I2C_SYSC): */
#define OMAP_I2C_SYSC_SRST        (1 << 1)    /* Soft Reset */

/* REVISIT: Use platform_data instead of module parameters */
static int clock = 100;    /* Default: Fast Mode = 400 KHz, Standard = 100 KHz */
module_param(clock, int, 0);
MODULE_PARM_DESC(clock, "Set I2C clock in kHz: 100 or 400 (Fast Mode)");

static int own;
module_param(own, int, 0);
MODULE_PARM_DESC(own, "Address of OMAP I2C master (0 for default == 1)");

struct omap_i2c_dev {
    struct device       *dev;
    void __iomem        *base;        /* virtual */
    int                 irq;
    struct clk          *iclk;        /* Interface clock */
    struct clk          *fclk;        /* Functional clock */
    struct completion   cmd_complete;
    u16                 cmd_err;
    u8                  *buf;
    size_t              buf_len;
    struct i2c_adapter  adapter;
    unsigned            rev1:1;
    u8                  own_address;
};

static inline void omap_i2c_write_reg(struct omap_i2c_dev *i2c_dev,
                      int reg, u16 val)
{
    __raw_writew(val, i2c_dev->base + reg);
}

static inline u16 omap_i2c_read_reg(struct omap_i2c_dev *i2c_dev, int reg)
{
    return __raw_readw(i2c_dev->base + reg);
}

#ifdef CONFIG_ARCH_OMAP24XX
static int omap_i2c_get_clocks(struct omap_i2c_dev *dev, int bus)
{
    if (!cpu_is_omap24xx())
        return 0;

    dev->iclk = clk_get(NULL,
        bus == 1 ? "i2c1_ick" : "i2c2_ick");
    if (IS_ERR(dev->iclk)) {
        return -ENODEV;
    }

    dev->fclk = clk_get(NULL,
        bus == 1 ? "i2c1_fck" : "i2c2_fck");
    if (IS_ERR(dev->fclk)) {
        clk_put(dev->fclk);
        return -ENODEV;
    }

    return 0;
}

static void omap_i2c_put_clocks(struct omap_i2c_dev *dev)
{
    clk_put(dev->fclk);
    clk_put(dev->iclk);
}

static void omap_i2c_enable_clocks(struct omap_i2c_dev *dev)
{
    clk_enable(dev->iclk);
    clk_enable(dev->fclk);
}

static void omap_i2c_disable_clocks(struct omap_i2c_dev *dev)
{
    clk_disable(dev->iclk);
    clk_disable(dev->fclk);
}

#else
#define omap_i2c_get_clocks(x, y)        0
#define omap_i2c_enable_clocks(x)    do {} while (0)
#define omap_i2c_disable_clocks(x)    do {} while (0)
#define omap_i2c_put_clocks(x)        do {} while (0)
#endif

static void omap_i2c_reset(struct omap_i2c_dev *dev)
{
    u16 psc;
    unsigned long fclk_rate;

    if (!dev->rev1) {
        omap_i2c_write_reg(dev, OMAP_I2C_SYSC_REG, OMAP_I2C_SYSC_SRST);
        /* For some reason we need to set the EN bit before the
         * reset done bit gets set. */
        omap_i2c_write_reg(dev, OMAP_I2C_CON_REG, OMAP_I2C_CON_EN);
        while (!(omap_i2c_read_reg(dev, OMAP_I2C_SYSS_REG) & 0x01));
    }
    omap_i2c_write_reg(dev, OMAP_I2C_CON_REG, 0);

    if (cpu_class_is_omap1()) {
        struct clk *armxor_ck;
        unsigned long armxor_rate;

        armxor_ck = clk_get(NULL, "armxor_ck");
        if (IS_ERR(armxor_ck)) {
            printk(KERN_WARNING "i2c: Could not get armxor_ck\n");
            armxor_rate = 12000000;
        } else {
            armxor_rate = clk_get_rate(armxor_ck);
            clk_put(armxor_ck);
        }

        if (armxor_rate > 16000000)
            psc = (armxor_rate + 8000000) / 12000000;
        else
            psc = 0;

        fclk_rate = armxor_rate;
    } else if (cpu_class_is_omap2()) {
        fclk_rate = 12000000;
        psc = 0;
    }

    /* Setup clock prescaler to obtain approx 12MHz I2C module clock: */
#if defined(CONFIG_MACH_ITOMAP2430)
    // [SPP]  Need to re-visit clock choices.
    omap_i2c_write_reg(dev, OMAP_I2C_PSC_REG, psc+7);
#else
    omap_i2c_write_reg(dev, OMAP_I2C_PSC_REG, psc);
#endif

    /* Program desired operating rate */
    fclk_rate /= (psc + 1) * 1000;
    if (psc > 2)
        psc = 2;

    omap_i2c_write_reg(dev, OMAP_I2C_SCLL_REG,
               fclk_rate / (clock * 2) - 7 + psc);
    omap_i2c_write_reg(dev, OMAP_I2C_SCLH_REG,
               fclk_rate / (clock * 2) - 7 + psc);

    /* Set Own Address: */
    omap_i2c_write_reg(dev, OMAP_I2C_OA_REG, dev->own_address);

    /* Take the I2C module out of reset: */
    omap_i2c_write_reg(dev, OMAP_I2C_CON_REG, OMAP_I2C_CON_EN);

    /* Enable interrupts */
    omap_i2c_write_reg(dev, OMAP_I2C_IE_REG,
               (OMAP_I2C_IE_XRDY | OMAP_I2C_IE_RRDY |
                OMAP_I2C_IE_ARDY | OMAP_I2C_IE_NACK |
                OMAP_I2C_IE_AL));
}

/*
 * Waiting on Bus Busy
 */
static int omap_i2c_wait_for_bb(struct omap_i2c_dev *dev)
{
    unsigned long timeout;

    timeout = jiffies + OMAP_I2C_TIMEOUT;
    while (omap_i2c_read_reg(dev, OMAP_I2C_STAT_REG) & OMAP_I2C_STAT_BB) {
        if (time_after(jiffies, timeout)) {
            dev_warn(dev->dev, "timeout waiting for bus ready\n");
            return -ETIMEDOUT;
        }
        msleep(1);
    }

    return 0;
}

#if (USE_BIG_BAD_BIT_BANG_HACK != 1)

/*
 * Low level master read/write transaction.
 */
static int omap_i2c_xfer_msg(struct i2c_adapter *adap,
                 struct i2c_msg *msg, int stop)
{
    struct omap_i2c_dev *dev = i2c_get_adapdata(adap);
    int r;
    u16 w;
    u8 zero_byte = 0;

    dev_dbg(dev->dev, "addr: 0x%04x, len: %d, flags: 0x%x, stop: %d\n",
        msg->addr, msg->len, msg->flags, stop);

    omap_i2c_write_reg(dev, OMAP_I2C_SA_REG, msg->addr);

    /* Sigh, seems we can't do zero length transactions. Thus, we
     * can't probe for devices w/o actually sending/receiving at least
     * a single byte. So we'll set count to 1 for the zero length
     * transaction case and hope we don't cause grief for some
     * arbitrary device due to random byte write/read during
     * probes.
     */
    /* REVISIT: Could the STB bit of I2C_CON be used with probing? */
    if (msg->len == 0) {
        dev->buf = &zero_byte;
        dev->buf_len = 1;
    } else {
        dev->buf = msg->buf;
        dev->buf_len = msg->len;
    }
    omap_i2c_write_reg(dev, OMAP_I2C_CNT_REG, dev->buf_len);

#if defined(CONFIG_MACH_ITOMAP2430)
    // [SPP] fill the fifo outside the interrupt
    while (dev->buf_len--)
        omap_i2c_write_reg(dev, OMAP_I2C_DATA_REG, *dev->buf++);
#endif

    init_completion(&dev->cmd_complete);
    dev->cmd_err = 0;

    w = OMAP_I2C_CON_EN | OMAP_I2C_CON_MST | OMAP_I2C_CON_STT;
    if (msg->flags & I2C_M_TEN)
        w |= OMAP_I2C_CON_XA;
    if (!(msg->flags & I2C_M_RD))
        w |= OMAP_I2C_CON_TRX;
    if (stop)
        w |= OMAP_I2C_CON_STP;
    omap_i2c_write_reg(dev, OMAP_I2C_CON_REG, w);

    r = wait_for_completion_interruptible_timeout(&dev->cmd_complete,
                              OMAP_I2C_TIMEOUT);
    dev->buf_len = 0;
    if (r < 0)
        return r;
    if (r == 0) {
        dev_err(dev->dev, "controller timed out\n");
        omap_i2c_reset(dev);
        return -ETIMEDOUT;
    }

    if (likely(!dev->cmd_err))
        return 0;

    /* We have an error */
    if (dev->cmd_err & OMAP_I2C_STAT_NACK) {
        if (msg->flags & I2C_M_IGNORE_NAK)
            return 0;
        if (stop) {
            u16 w;

            w = omap_i2c_read_reg(dev, OMAP_I2C_CON_REG);
            w |= OMAP_I2C_CON_STP;
            omap_i2c_write_reg(dev, OMAP_I2C_CON_REG, w);
        }
        return -EREMOTEIO;
    }
    if (dev->cmd_err & (OMAP_I2C_STAT_AL | OMAP_I2C_STAT_ROVR |
                OMAP_I2C_STAT_XUDF))
        omap_i2c_reset(dev);
    return -EIO;
}


/*
 * Prepare controller for a transaction and call omap_i2c_xfer_msg
 * to do the work during IRQ processing.
 */
static int
omap_i2c_xfer(struct i2c_adapter *adap, struct i2c_msg msgs[], int num)
{
    struct omap_i2c_dev *dev = i2c_get_adapdata(adap);
    int i;
    int r = 0;

    if (num < 1 || num > MAX_MESSAGES)
        return -EINVAL;

    /* Check for valid parameters in messages */
    for (i = 0; i < num; i++)
        if (msgs[i].buf == NULL)
            return -EINVAL;

    omap_i2c_enable_clocks(dev);

    /* REVISIT: initialize and use adap->retries */
    if ((r = omap_i2c_wait_for_bb(dev)) < 0)
        goto out;

    for (i = 0; i < num; i++) {
        dev_dbg(dev->dev, "msg: %d, addr: 0x%04x, len: %d, flags: 0x%x\n",
            i, msgs[i].addr, msgs[i].len, msgs[i].flags);
        r = omap_i2c_xfer_msg(adap, &msgs[i], (i == (num - 1)));
        if (r != 0)
            break;
    }

    if (r == 0 && num > 1)
        r = num;
out:
    omap_i2c_disable_clocks(dev);
    return r;
}

#else // #if (USE_BIG_BAD_BIT_BANG_HACK == 1)

#define WRITE_CYCLE 0xFE // bit mask for indicating start of a write cycle
#define READ_CYCLE  0x01 // bit mask for indication start of a read cycle

#define I2C_REV_MINOR          (0x0f)
#define I2C_REV_MAJOR          (0xf0)
#define I2C_REV_MAJOR_SHIFT    (4)

#define I2C_AL_IE              (1 <<  0)
#define I2C_NACK_IE            (1 <<  1)
#define I2C_ARDY_IE            (1 <<  2)
#define I2C_RRDY_IE            (1 <<  3)
#define I2C_XRDY_IE            (1 <<  4)
#define I2C_GC_IE              (1 <<  5)

#define I2C_STAT_AL            (1 <<  0)
#define I2C_STAT_NACK          (1 <<  1)
#define I2C_STAT_ARDY          (1 <<  2)
#define I2C_STAT_RRDY          (1 <<  3)
#define I2C_STAT_XRDY          (1 <<  4)
#define I2C_STAT_GC            (1 <<  5)
#define I2C_STAT_AAS           (1 <<  9)
#define I2C_STAT_XUDF          (1 << 10)
#define I2C_STAT_ROVR          (1 << 11)
#define I2C_STAT_BB            (1 << 12)
#define I2C_STAT_SBD           (1 << 15)
#define I2C_STAT_CLEAR_MASK    (0x3f)

#define I2C_SYSS_RDONE         (1 <<  0)

#define I2C_BUF_XDMA_EN        (1 <<  7)
#define I2C_BUF_RDMA_EN        (1 << 15)

#define I2C_SYSC_SRST          (1 <<  1)

#define I2C_CON_STT            (1 <<  0)
#define I2C_CON_STP            (1 <<  1)
#define I2C_CON_XA             (1 <<  8)
#define I2C_CON_TRX            (1 <<  9)
#define I2C_CON_MST            (1 << 10)
#define I2C_CON_STB            (1 << 11)
#define I2C_CON_BE             (1 << 14)
#define I2C_CON_12C_EN         (1 << 15)

#define I2C_OA_MASK            (0x3ff)
#define I2C_SA_MASK            (0x3ff)
#define I2C_PSC_MASK           (0xff)
#define I2C_SCLL_MASK          (0xff)
#define I2C_SCLH_MASK          (0xff)

#define I2C_SYSTEST_SDA_O      (1 << 0)
#define I2C_SYSTEST_SDA_I      (1 << 1)
#define I2C_SYSTEST_SCL_O      (1 << 2)
#define I2C_SYSTEST_SCL_I      (1 << 3)
#define I2C_SYSTEST_SSB        (1 << 11)
#define I2C_SYSTEST_TMODE_MASK (3 << 12)
#define I2C_SYSTEST_TMODE_FUNC (0 << 12)
#define I2C_SYSTEST_TMODE_TEST (2 << 12)
#define I2C_SYSTEST_TMODE_LOOP (3 << 12)
#define I2C_SYSTEST_FREE       (1 << 14)
#define I2C_SYSTEST_ST_EN      (1 << 15)

#define I2C_SHORT_DELAY()   udelay(1)
#define I2C_BIG_DELAY()     udelay(10)

typedef enum
{
    I2C_1 = 0,
    I2C_2,
    NUM_I2C
} i2c_id_type;

typedef volatile struct
{
  volatile u16           rev;                 /* 0x00 */
  volatile u16           pad1;                /* 0x02 */
  volatile u16           ie;                  /* 0x04 */
  volatile u16           pad2;                /* 0x06 */
  volatile u16           stat;                /* 0x08 */
  volatile u16           pad3[3];             /* 0x0A - 0x0F */
  volatile u16           syss;                /* 0x10 */
  volatile u16           pad4;                /* 0x12 */
  volatile u16           buf;                 /* 0x14 */
  volatile u16           pad5;                /* 0x16 */
  volatile u16           cnt;                 /* 0x18 */
  volatile u16           pad6;                /* 0x1A */
  volatile u16           data;                /* 0x1C */
  volatile u16           pad7;                /* 0x1E */
  volatile u16           sysc;                /* 0x20 */
  volatile u16           pad8;                /* 0x22 */
  volatile u16           con;                 /* 0x24 */
  volatile u16           pad9;                /* 0x26 */
  volatile u16           oa;                  /* 0x28 */
  volatile u16           pad10;               /* 0x2A */
  volatile u16           sa;                  /* 0x2C */
  volatile u16           pad11;               /* 0x2E */
  volatile u16           psc;                 /* 0x30 */
  volatile u16           pad12;               /* 0x32 */
  volatile u16           scll;                /* 0x34 */
  volatile u16           pad13;               /* 0x36 */
  volatile u16           sclh;                /* 0x38 */
  volatile u16           pad14;               /* 0x3A */
  volatile u16           sysTest;             /* 0x3C */
  volatile u16           pad15;               /* 0x3E */
} i2c_register_map;

void i2c_set_slc(i2c_id_type i2c_id, u32 val);
void i2c_set_sda(i2c_id_type i2c_id, u32 val);
void i2c_send_bit(i2c_id_type i2c_id,u32 bit);

static u32 SDA = 1;
static u32 SCL = 1;

static inline void i2c_reg_wr16(volatile u16* addr, u16 val) 
{
  I2C_SHORT_DELAY();
  *addr = val;
}

static inline u16 i2c_reg_rd16(volatile u16* addr)
{
  u16                    val;

  I2C_SHORT_DELAY();
  val = *addr;
  return(val);
}

#define I2C_REGEW(addr, val)         i2c_reg_wr16((volatile u16*)&addr, (u16)val)
#define I2C_REGER(addr)              i2c_reg_rd16((volatile u16*)&addr)
#define  _I2C_REGS(id) (((i2c_register_map *)pI2CBaseAddrs[id]))

static const i2c_register_map *pI2CBaseAddrs[NUM_I2C] = 
{
    (i2c_register_map *)IO_ADDRESS(0x48070000), /* I2C1 phys address */
    (i2c_register_map *)IO_ADDRESS(0x48072000), /* I2C2 phys address */
};

int i2c_test_pinO(i2c_id_type i2c_id, u32 highSCL, u32 highSDA)
{
    u16 regVal;
    
    regVal = I2C_SYSTEST_ST_EN | I2C_SYSTEST_TMODE_LOOP;

    if (highSCL != 0)
    {
        regVal |= I2C_SYSTEST_SCL_O;
    }
    if (highSDA != 0)
    {
        regVal |= I2C_SYSTEST_SDA_O;
    }

    I2C_REGEW(_I2C_REGS(i2c_id)->sysTest, regVal);

    return(0);
}

u32 i2c_test_sda_pin_i(i2c_id_type i2c_id)
{
    if (I2C_REGER(_I2C_REGS(i2c_id)->sysTest) & I2C_SYSTEST_SDA_I)
    {
        return(1);
    }
    else
    {
        return(0);
    }
}

u32 i2c_test_scl_pin_i(i2c_id_type i2c_id)
{
    if (I2C_REGER(_I2C_REGS(i2c_id)->sysTest) & I2C_SYSTEST_SCL_I)
    {
        return(1);
    }
    else
    {
        return(0);
    }
}

int start_i2c(i2c_id_type i2c_id)
{ 
    int status = 0;
    //
    // first force the bus into the free state i.e. SCL = 1, SDA = 1
    //
    i2c_set_slc(i2c_id, 0); // make this low first before changing SDA
    i2c_set_sda(i2c_id, 1); // bring this high while SCL is low
    i2c_set_slc(i2c_id, 1); // now bring SCL high
    //
    // see if bus is free i.e. SDA, SCL = 1
    // if bus is not free then return error status
    if ((i2c_test_sda_pin_i(i2c_id))&&(i2c_test_scl_pin_i(i2c_id)))
    {
        status = 0;
    }
    else
    {
        status = -1;
    }
    //
    // create the start transition 
    //
    i2c_set_sda(i2c_id, 0); // while SCL is high this will be seen as a start
    return(status);
}

void stop_i2c(i2c_id_type i2c_id)
{
    //
    // first force the bus into the state SCL = 0, SDA = 1
    //
    i2c_set_slc(i2c_id, 0);     // make this low first before changing SDA

    i2c_set_sda(i2c_id, 0);     // bring this low while SCL is low

    i2c_set_slc(i2c_id, 1);     // now bring SCL high
    //
    // create the stop transition 
    //
    i2c_set_sda(i2c_id, 1); // while SCL is high this will be seen as a stop
}

u8 read_bit_i2c(i2c_id_type i2c_id)
{
    u8 bitValue;

    i2c_set_slc(i2c_id, 1); 

    if (i2c_test_sda_pin_i(i2c_id)== 1)         // read the bit while the
    {                                           // SCL line is high
        bitValue = 0x01;
    }
    else
    {
        bitValue = 0x00;
    }

    i2c_set_slc(i2c_id, 0);

    return(bitValue);
}

u32 wait_ack_i2c(i2c_id_type i2c_id)
{
    u32 sda;

    i2c_set_slc(i2c_id, 0);              // make SCL low so we can release SDA

    i2c_set_sda(i2c_id, 1);         // release SDA
    sda = i2c_test_sda_pin_i(i2c_id);   // read SDA after falling edge of 8th pulse

    i2c_set_slc(i2c_id, 1);              // provide 9th clock pulse

    i2c_set_slc(i2c_id, 0);
    if (sda & 0x0002)
    {
        // no ACK
        return(0);
    }
    else
    {
        // got an ACK
        return(1);
    }
}

u32 send_byte_i2c(i2c_id_type i2c_id, u32 byte)
{
    u32 i;
    u32 bits;

    for(i = 0; i < 8; i++)
    {
        bits = (byte >> (7-i)) & 0x01;
        i2c_send_bit(i2c_id, bits);
    }

    return(wait_ack_i2c(i2c_id));
}

u8 read_byte_i2c(i2c_id_type i2c_id)
{
    u32 i;
    u32 bit;
    u8 byte = 0;
    // release the SDA line first
    i2c_set_sda(i2c_id, 1);
    // read in a bit at time msb first
    for(i = 0; i < 8; i++)
    {
        bit = read_bit_i2c(i2c_id);
        byte = byte | (bit << (7-i));
    }
    return(byte);
}

void give_nack_i2c(i2c_id_type i2c_id)
{
    i2c_set_slc(i2c_id, 1); // do nothing to SDA line
    i2c_set_slc(i2c_id, 0);
}

void i2c_set_slc(i2c_id_type i2c_id, u32 val)
{
    u32 timeout;

    SCL = val;
    i2c_test_pinO(i2c_id, SCL, SDA);

    // if the clock is transitioning to high
    // check to see if slave is sychronizing with the master
    if (SCL == 1)
    {
        timeout = 0xfffff;
        while ((i2c_test_scl_pin_i(i2c_id) == 0) && (--timeout > 0));
        if (i2c_test_scl_pin_i(i2c_id) == 0)
        {
            printk("i2c_set_slc(): i2c clock line always low\n");
        }
    }
}

void i2c_set_sda(i2c_id_type i2c_id, u32 val)
{
    SDA = val;
    i2c_test_pinO(i2c_id, SCL, SDA);
}

void i2c_send_bit(i2c_id_type i2c_id, u32 bit)
{
    i2c_set_slc(i2c_id, 0);
    i2c_set_sda(i2c_id, bit);
    i2c_set_slc(i2c_id, 1);
}

int i2c_read_generic_reg(u8 i2c_device_addr, struct omap_i2c_dev *dev, int stop)
{
    int status = 0;                     // bus status
    u32 ack = 0;                        // acknowledgment

    I2C_BIG_DELAY();

    status = start_i2c(I2C_1);
    if (status == 0)                    // check to see if bus is free
    { 
        ack &= send_byte_i2c(I2C_1, (i2c_device_addr<<1)|READ_CYCLE);
        *dev->buf = read_byte_i2c(I2C_1);    
        give_nack_i2c(I2C_1);           // last byte give a NACK
        if (stop)
        {
            stop_i2c(I2C_1);
        }
        if (ack == 1)
        {
            status = 0;
        }
        else
        {
            status = -1;
        }
    }

    return(status);
}

int i2c_write_generic_reg(u8 i2c_device_addr, struct omap_i2c_dev *dev, int stop)
{
    int i;
    int status = 0;         // bus status
    u8* bufp;
    u32 ack;                // acknowledgment


    I2C_BIG_DELAY();
    status = start_i2c(I2C_1);
  
    if (status == 0)
    {
        ack = send_byte_i2c(I2C_1, (i2c_device_addr<<1)&WRITE_CYCLE);  

        i = 0;
        bufp = dev->buf;
        while ((ack) && (i++ < dev->buf_len))
        {
            ack &= send_byte_i2c(I2C_1, *bufp++);
        }
        if (ack == 1)
        {
            status = 0;
        }
        else
        {
            status = -1;
        }
    }

    if (stop)
    {
        stop_i2c(I2C_1);
    }

    return(status);
}

/*
 * Low level master read/write transaction.
 */
static int omap_i2c_xfer_msg(struct i2c_adapter *adap,
                 struct i2c_msg *msg, int stop)
{
    struct omap_i2c_dev *dev = i2c_get_adapdata(adap);
    u8 zero_byte = 0;

    dev_dbg(dev->dev, "addr: 0x%04x, len: %d, flags: 0x%x, stop: %d\n",
        msg->addr, msg->len, msg->flags, stop);

    /* Sigh, seems we can't do zero length transactions. Thus, we
     * can't probe for devices w/o actually sending/receiving at least
     * a single byte. So we'll set count to 1 for the zero length
     * transaction case and hope we don't cause grief for some
     * arbitrary device due to random byte write/read during
     * probes.
     */
    /* REVISIT: Could the STB bit of I2C_CON be used with probing? */
    if (msg->len == 0) {
        dev->buf = &zero_byte;
        dev->buf_len = 1;
    } else {
        dev->buf = msg->buf;
        dev->buf_len = msg->len;
    }

    if (msg->flags & I2C_M_RD)
    {
        i2c_read_generic_reg(msg->addr, dev, stop);
    }
    else
    {
        i2c_write_generic_reg(msg->addr, dev, stop);
    }

    return 0; // JBH ^^^^ no error for now
}

/*
 * Prepare controller for a transaction and call omap_i2c_xfer_msg
 * to do the work during IRQ processing.
 */
static int
omap_i2c_xfer(struct i2c_adapter *adap, struct i2c_msg msgs[], int num)
{
    struct omap_i2c_dev *dev = i2c_get_adapdata(adap);
    int i;
    int r = 0;

    if (num < 1 || num > MAX_MESSAGES)
        return -EINVAL;

    /* Check for valid parameters in messages */
    for (i = 0; i < num; i++)
        if (msgs[i].buf == NULL)
            return -EINVAL;

    omap_i2c_enable_clocks(dev);

    /* REVISIT: initialize and use adap->retries */
    if ((r = omap_i2c_wait_for_bb(dev)) < 0)
        goto out;

    for (i = 0; i < num; i++) {
        dev_dbg(dev->dev, "msg: %d, addr: 0x%04x, len: %d, flags: 0x%x\n",
            i, msgs[i].addr, msgs[i].len, msgs[i].flags);
        r = omap_i2c_xfer_msg(adap, &msgs[i], (i == (num - 1)));
        if (r != 0)
            break;
    }

    if (r == 0 && num > 1)
        r = num;
out:
    omap_i2c_disable_clocks(dev);
    return r;
}

#endif // #if (USE_BIG_BAD_BIT_BANG_HACK == 1)

static u32
omap_i2c_func(struct i2c_adapter *adap)
{
    return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;
}

static inline void
omap_i2c_complete_cmd(struct omap_i2c_dev *dev, u16 err)
{
    dev->cmd_err |= err;
    complete(&dev->cmd_complete);
}

static inline void
omap_i2c_ack_stat(struct omap_i2c_dev *dev, u16 stat)
{
    omap_i2c_write_reg(dev, OMAP_I2C_STAT_REG, stat);
}

#if (USE_BIG_BAD_BIT_BANG_HACK != 1)
static irqreturn_t
omap_i2c_isr(int this_irq, void *dev_id, struct pt_regs *regs)
{
    struct omap_i2c_dev *dev = dev_id;
    u16 bits;
    u16 stat, w;
    int count = 0;
    u16 iv_read;

    bits = omap_i2c_read_reg(dev, OMAP_I2C_IE_REG);
    while ((stat = (omap_i2c_read_reg(dev, OMAP_I2C_STAT_REG))) & bits) {
        dev_dbg(dev->dev, "IRQ (ISR = 0x%04x)\n", stat);
        if (count++ == 100) {
            dev_warn(dev->dev, "Too much work in one IRQ\n");
            break;
        }

        omap_i2c_write_reg(dev, OMAP_I2C_STAT_REG, stat);

        if (stat & OMAP_I2C_STAT_ARDY) {
            omap_i2c_complete_cmd(dev, 0);
            if (dev->rev1)
                iv_read = omap_i2c_read_reg(dev, OMAP_I2C_IV_REG);
            continue;
        }
        if (stat & OMAP_I2C_STAT_RRDY) {
            w = omap_i2c_read_reg(dev, OMAP_I2C_DATA_REG);
            if (dev->buf_len) {
                *dev->buf++ = w;
                dev->buf_len--;
                if (dev->buf_len) {
#if !defined(CONFIG_MACH_ITOMAP2430)
                    *dev->buf++ = w >> 8;
#else
                    *dev->buf++ = omap_i2c_read_reg(dev, OMAP_I2C_DATA_REG);
#endif
                    dev->buf_len--;
                }
                if (dev->rev1 && !dev->buf_len)
                    omap_i2c_complete_cmd(dev, 0);
            } else
                dev_err(dev->dev, "RRDY IRQ while no data requested\n");
            omap_i2c_ack_stat(dev, OMAP_I2C_STAT_RRDY);
            if (dev->rev1)
                iv_read = omap_i2c_read_reg(dev, OMAP_I2C_IV_REG);
            continue;
        }
        if (stat & OMAP_I2C_STAT_XRDY) {
            int bail_out = 0;

            w = 0;
            if (dev->buf_len) {
                w = *dev->buf++;
                dev->buf_len--;
                if (dev->buf_len) {
                    w |= *dev->buf++ << 8;
                    dev->buf_len--;
                }
            } else
                dev_err(dev->dev, "XRDY IRQ while no data to send\n");
#if 0
            if (!(stat & OMAP_I2C_STAT_BB)) {
                dev_warn(dev->dev, "XRDY while bus not busy\n");
                bail_out = 1;
            }
#endif
#if !defined(CONFIG_MACH_ITOMAP2430)
            // [SPP] Fill the data register outside the interrupt
            omap_i2c_write_reg(dev, OMAP_I2C_DATA_REG, w);
#endif
            omap_i2c_ack_stat(dev, OMAP_I2C_STAT_XRDY);
            if (dev->rev1) {
                iv_read = omap_i2c_read_reg(dev, OMAP_I2C_IV_REG);
                if (!dev->buf_len)
                    omap_i2c_complete_cmd(dev, 0);
            }
            if (bail_out)
                omap_i2c_complete_cmd(dev, 1 << 15);
            continue;
        }
        if (stat & OMAP_I2C_STAT_ROVR) {
            dev_err(dev->dev, "Receive overrun\n");
            dev->cmd_err |= OMAP_I2C_STAT_ROVR;
        }
        if (stat & OMAP_I2C_STAT_XUDF) {
            dev_err(dev->dev, "Transmit overflow\n");
            dev->cmd_err |= OMAP_I2C_STAT_XUDF;
        }
        if (stat & OMAP_I2C_STAT_NACK) {
            omap_i2c_complete_cmd(dev, OMAP_I2C_STAT_NACK);
            omap_i2c_write_reg(dev, OMAP_I2C_CON_REG, OMAP_I2C_CON_STP);
        }
        if (stat & OMAP_I2C_STAT_AL) {
            dev_err(dev->dev, "Arbitration lost\n");
            omap_i2c_complete_cmd(dev, OMAP_I2C_STAT_AL);
        }
        if (dev->rev1)
            iv_read = omap_i2c_read_reg(dev, OMAP_I2C_IV_REG);
    }

    return count ? IRQ_HANDLED : IRQ_NONE;
}
#endif // #if (USE_BIG_BAD_BIT_BANG_HACK != 1)

static struct i2c_algorithm omap_i2c_algo = {
    .master_xfer    = omap_i2c_xfer,
    .functionality    = omap_i2c_func,
};

static int
omap_i2c_probe(struct platform_device *pdev)
{
    struct omap_i2c_dev    *dev;
    struct i2c_adapter    *adap;
    struct resource        *mem, *irq;
    int r;

    /* NOTE: driver uses the static register mapping */
    mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    if (!mem) {
        dev_err(&pdev->dev, "no mem resource?\n");
        return -ENODEV;
    }
    irq = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
    if (!irq) {
        dev_err(&pdev->dev, "no irq resource?\n");
        return -ENODEV;
    }

    r = (int) request_mem_region(mem->start, (mem->end - mem->start) + 1,
            driver_name);
    if (!r) {
        dev_err(&pdev->dev, "I2C region already claimed\n");
        return -EBUSY;
    }

    if (clock > 200)
        clock = 400;    /* Fast mode */
    else
        clock = 100;    /* Standard mode */

    dev = kzalloc(sizeof(struct omap_i2c_dev), GFP_KERNEL);
    if (!dev) {
        r = -ENOMEM;
        goto do_release_region;
    }

    /* FIXME: Get own address from platform_data */
    if (own >= 1 && own < 0x7f)
        dev->own_address = own;
    else
        own = DEFAULT_OWN;

    dev->dev = &pdev->dev;
    dev->irq = irq->start;
    dev->base = (void __iomem *) IO_ADDRESS(mem->start);
    platform_set_drvdata(pdev, dev);

    if ((r = omap_i2c_get_clocks(dev, pdev->id)) != 0)
        goto do_free_mem;

    omap_i2c_enable_clocks(dev);

#ifdef CONFIG_ARCH_OMAP15XX
    dev->rev1 = omap_i2c_read_reg(dev, OMAP_I2C_REV_REG) < 0x20;
#endif

    /* reset ASAP, clearing any IRQs */
    omap_i2c_reset(dev);

#if (USE_BIG_BAD_BIT_BANG_HACK != 1)
    r = request_irq(dev->irq, omap_i2c_isr, 0,
            driver_name, dev);
    if (r) {
        dev_err(dev->dev, "failure requesting irq %i\n", dev->irq);
        goto do_unuse_clocks;
    }
#endif
    r = omap_i2c_read_reg(dev, OMAP_I2C_REV_REG) & 0xff;
    dev_info(dev->dev, "bus %d rev%d.%d at %d kHz\n",
         pdev->id - 1, r >> 4, r & 0xf, clock);

    adap = &dev->adapter;
    i2c_set_adapdata(adap, dev);
    adap->owner = THIS_MODULE;
    adap->class = I2C_CLASS_HWMON;
    strncpy(adap->name, "OMAP I2C adapter", sizeof(adap->name));
    adap->algo = &omap_i2c_algo;
    adap->dev.parent = &pdev->dev;

    /* i2c device drivers may be active on return from add_adapter() */
    r = i2c_add_adapter(adap);
    if (r) {
        dev_err(dev->dev, "failure adding adapter\n");
        goto do_free_irq;
    }

    omap_i2c_disable_clocks(dev);

    return 0;

do_free_irq:
    free_irq(dev->irq, dev);
#if (USE_BIG_BAD_BIT_BANG_HACK != 1)
do_unuse_clocks:
#endif
    omap_i2c_enable_clocks(dev);
    omap_i2c_put_clocks(dev);
do_free_mem:
    kfree(dev);
do_release_region:
    omap_i2c_write_reg(dev, OMAP_I2C_CON_REG, 0);
    release_mem_region(mem->start, (mem->end - mem->start) + 1);

    return r;
}

static int
omap_i2c_remove(struct platform_device *pdev)
{
    struct omap_i2c_dev    *dev = platform_get_drvdata(pdev);
    struct resource        *mem;

    free_irq(dev->irq, dev);
    i2c_del_adapter(&dev->adapter);
    omap_i2c_write_reg(dev, OMAP_I2C_CON_REG, 0);
    omap_i2c_put_clocks(dev);
    kfree(dev);
    mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    release_mem_region(mem->start, (mem->end - mem->start) + 1);
    return 0;
}

static struct platform_driver omap_i2c_driver = {
    .probe      = omap_i2c_probe,
    .remove     = omap_i2c_remove,
    .driver     = {
    .name       = (char *)driver_name,
    },
};

/* I2C may be needed to bring up other drivers */
static int __init omap_i2c_init_driver(void)
{

#if (USE_BIG_BAD_BIT_BANG_HACK == 1)
    // use test mode both high
    i2c_test_pinO(0, SCL, SDA);
    i2c_test_pinO(1, SCL, SDA);
#endif // #if (USE_BIG_BAD_BIT_BANG_HACK == 1)

    return platform_driver_register(&omap_i2c_driver);
}
subsys_initcall(omap_i2c_init_driver);

static void __exit omap_i2c_exit_driver(void)
{
    platform_driver_unregister(&omap_i2c_driver);
}
module_exit(omap_i2c_exit_driver);

MODULE_AUTHOR("MontaVista Software, Inc. (and others)");
MODULE_DESCRIPTION("TI OMAP I2C bus adapter");
MODULE_LICENSE("GPL");
