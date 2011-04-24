/*
 *  linux/drivers/mmc/mmci.h - ARM PrimeCell MMCI PL180/1 driver
 *
 *  Copyright (C) 2003 Deep Blue Solutions, Ltd, All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef DM320_MMC_H
#define DM320_MMC_H

#include <linux/mmc/machmmc.h>

#define DMA_TRANSFER 
#define DEBUG_MMC           0
#define DEBUG_MMC_RW        0
#define DEBUG_MMC_COMMAND   0
    
#if (DEBUG_MMC == 1)
    #define mmc_debug_msg(fmt, arg...) printk(KERN_DEBUG "%s:%d> " \
    fmt, __FUNCTION__, __LINE__ , ## arg)
#else
    #define mmc_debug_msg(fmt, arg...) do { /* NO OP */ } while(0)
#endif

#if (DEBUG_MMC_RW == 1)
    #define mmc_debug_msg_rw(fmt, arg...) printk(KERN_DEBUG "%s:%d> " \
    fmt, __FUNCTION__, __LINE__ , ## arg)
#else
    #define mmc_debug_msg_rw(fmt, arg...) do { /* NO OP */ } while(0)
#endif

/*
 * The size of the FIFO in bytes.
 */
#define MCI_FIFOSIZE    (16*4)
    
#define MCI_FIFOHALFSIZE (MCI_FIFOSIZE / 2)

#define NR_SG        16

/* Yes, this should probably go in asm/arch/gio.h */
#define IT_DMA_WRITE        1
#define IT_DMW_READ         0
#define SD_CARD_DETECT_BIT  2 
#define SD_CARD_WP_BIT      1

#define IO_CLK_MMCCLK       0x0482
#define MMC_CLK_ENABLE      (1 << 8)

/* just to be on the safe side, define some standard data types.  */
#define u8 unsigned char
#define u16 unsigned short
#define u32 unsigned long
#define s8 signed char
#define s16 signed short
#define s32 signed long

#define MMC_INT_DAT3                    (1 << 11)
#define MMC_INT_DAT_RCV_RDY             (1 << 10)
#define MMC_INT_DAT_TRX_RDY             (1 << 9)
#define MMC_INT_DMA_DONE                (1 << 8)    // see erata
#define MMC_INT_RSP_CRC_ERROR           (1 << 7)
#define MMC_INT_READ_CRC_ERROR          (1 << 6)
#define MMC_INT_WRITE_CRC_ERROR         (1 << 5)
#define MMC_INT_RSP_TIMEOUT             (1 << 4)
#define MMC_INT_READ_TIMEOUT            (1 << 3)
#define MMC_INT_RSP_DONE                (1 << 2)
#define MMC_INT_BUSY_DONE               (1 << 1)
#define MMC_INT_DATA_DONE               (1 << 0)

#define MMC_INT_CRC_ERROR (MMC_INT_WRITE_CRC_ERROR | \
    MMC_INT_READ_CRC_ERROR | MMC_INT_RSP_CRC_ERROR)

#define MMC_INT_TIME_OUT_ERROR (MMC_INT_READ_TIMEOUT | \
    MMC_INT_RSP_TIMEOUT)

#define MMC_INT_DAT_RCV_RDY_S1          (1 << 3)
#define MMC_INT_DAT_TRX_RDY_S1          (1 << 2)

#define MMC_DMA_STATUS_RUNNING          (1 << 12)
#define MMC_DMA_STATUS_TIMEOUT          (1 << 13)

/* Response Macros */
#define MMCSD_RSPNONE                   (0x0000)
#define MMCSD_RSP1                      (0x0200)
#define MMCSD_RSP2                      (0x0400)
#define MMCSD_RSP3                      (0x0600)
#define MMCSD_RSP4                      MMCSD_RSP1
#define MMCSD_RSP5                      MMCSD_RSP1
#define MMCSD_RSP6                      MMCSD_RSP1

/*Insert 80 CLK cycles*/

#define MMC_INITCK                      (0x4000)

/*Write enable */

#define MMC_WR_EN                       (0x0800)
    
/*data transfer*/

#define MMCSD_DATA_TRANS                (0x2000)

/* Command Macros */
#define MMCSD_CMD0                      (0x0000)
#define MMCSD_CMD1                      (0x0001)
#define MMCSD_CMD2                      (0x0002)
#define MMCSD_CMD3                      (0x0003)
#define MMCSD_CMD4                      (0x0004)
#define MMCSD_CMD5                      (0x0005)
#define MMCSD_CMD6                      (0x0006)
#define MMCSD_CMD7                      (0x0007)
#define MMCSD_CMD8                      (0x0008)
#define MMCSD_CMD9                      (0x0009)
#define MMCSD_CMD10                     (0x000A)
#define MMCSD_CMD11                     (0x000B)
#define MMCSD_CMD12                     (0x000C)
#define MMCSD_CMD13                     (0x000D)
#define MMCSD_CMD14                     (0x000E)
#define MMCSD_CMD15                     (0x000F)
#define MMCSD_CMD16                     (0x0010)
#define MMCSD_CMD17                     (0x0011)
#define MMCSD_CMD18                     (0x0012)
#define MMCSD_CMD19                     (0x0013)
#define MMCSD_CMD20                     (0x0014)
#define MMCSD_CMD21                     (0x0015)
#define MMCSD_CMD22                     (0x0016)
#define MMCSD_CMD23                     (0x0017)
#define MMCSD_CMD24                     (0x0018)
#define MMCSD_CMD25                     (0x0019)
#define MMCSD_CMD26                     (0x001A)
#define MMCSD_CMD27                     (0x001B)
#define MMCSD_CMD28                     (0x001C)
#define MMCSD_CMD29                     (0x001D)
#define MMCSD_CMD30                     (0x001E)
#define MMCSD_CMD31                     (0x001F)
#define MMCSD_CMD32                     (0x0020)
#define MMCSD_CMD33                     (0x0021)
#define MMCSD_CMD34                     (0x0022)
#define MMCSD_CMD35                     (0x0023)
#define MMCSD_CMD36                     (0x0024)
#define MMCSD_CMD37                     (0x0025)
#define MMCSD_CMD38                     (0x0026)
#define MMCSD_CMD39                     (0x0027)
#define MMCSD_CMD40                     (0x0028)
#define MMCSD_CMD41                     (0x0029)
#define MMCSD_CMD42                     (0x002A)
#define MMCSD_CMD43                     (0x002B)
#define MMCSD_CMD44                     (0x002C)
#define MMCSD_CMD45                     (0x002D)
#define MMCSD_CMD46                     (0x002E)
#define MMCSD_CMD47                     (0x002F)
#define MMCSD_CMD48                     (0x0030)
#define MMCSD_CMD49                     (0x0031)
#define MMCSD_CMD50                     (0x0032)
#define MMCSD_CMD51                     (0x0033)
#define MMCSD_CMD52                     (0x0034)
#define MMCSD_CMD53                     (0x0035)
#define MMCSD_CMD54                     (0x0036)
#define MMCSD_CMD55                     (0x0037)
#define MMCSD_CMD56                     (0x0038)
#define MMCSD_CMD57                     (0x0039)
#define MMCSD_CMD58                     (0x003A)
#define MMCSD_CMD59                     (0x003B)
#define MMCSD_CMD60                     (0x003C)
#define MMCSD_CMD61                     (0x003D)
#define MMCSD_CMD62                     (0x003E)
#define MMCSD_CMD63                     (0x003F)
#define MMCSD_CMD64                     (0x0040)

/*OCR request voltage*/
    
#define OCR_REQUEST                     (0x00100000)

/*Busy expected*/

#define MMCSD_BSYEXP                    (0x0100)

/* Commands and their responses */
/* MMC and SD */
#define MMCSD_GO_IDLE_STATE             (MMCSD_CMD0 | MMCSD_RSPNONE)
#define MMCSD_ALL_SEND_CID              (MMCSD_CMD2 | MMCSD_RSP2 )
#define MMCSD_ALL_SEND_CID_INITCK       (MMCSD_CMD2 | MMCSD_RSP2 | MMC_INITCK)
#define MMCSD_SEND_RELATIVE_ADDRESS     (MMCSD_CMD3 | MMCSD_RSP6)
#define SD_SEND_OP_COND                 (MMCSD_CMD41 | MMCSD_RSP3)
#define SD_SEND_SD_STATUS               (MMCSD_CMD41 | MMCSD_RSP1)
#define SD_SEND_SCR                     (MMCSD_CMD51 | MMCSD_RSP1)
#define MMCSD_SET_DSR                   (MMCSD_CMD4 | MMCSD_RSPNONE)
#define MMCSD_SELECT_CARD               (MMCSD_CMD7 | MMCSD_RSP1)
#define MMCSD_DESELECT_CARD             (MMCSD_CMD7 )
#define MMCSD_SEND_CSD                  (MMCSD_CMD9 | MMCSD_RSP2)
#define MMCSD_SEND_CID                  (MMCSD_CMD10| MMCSD_RSP2)
#define MMCSD_SEND_STATUS               (MMCSD_CMD13 | MMCSD_RSP1)
#define MMCSD_GO_INACTIVE_STATE         (MMCSD_CMD15 | MMCSD_RSPNONE)
#define MMCSD_APP_CMD                   (MMCSD_CMD55 | MMCSD_RSP1 )
#define MMCSD_STOP_TRANSMISSION         (MMCSD_CMD12 | MMCSD_RSP1 | MMCSD_BSYEXP)
#define MMCSD_READ_MULTIPLE_BLOCK       (MMCSD_CMD18 | MMCSD_RSP1)
#define MMCSD_WRITE_MULTIPLE_BLOCK      (MMCSD_CMD25 | MMCSD_RSP1 ) /*| MMCSD_BSYEXP)*/

/* Common to SPI & MMC */
#define MMCSD_SET_BLOCKLEN              (MMCSD_CMD16 | MMCSD_RSP1 )
#define MMCSD_PROGRAM_CSD               (MMCSD_CMD27 | MMCSD_RSP1 | MMCSD_BSYEXP) /* MMC-bsy, SPI-bsy optional */
#define MMCSD_SET_WRITE_PROT            (MMCSD_CMD28 | MMCSD_RSP1 | MMCSD_BSYEXP)
#define MMCSD_CLR_WRITE_PROT            (MMCSD_CMD29 | MMCSD_RSP1 | MMCSD_BSYEXP)
#define MMCSD_SEND_WRITE_PROT           (MMCSD_CMD30 | MMCSD_RSP1)
#define MMCSD_READ_SINGLE_BLOCK         (MMCSD_CMD17 | MMCSD_RSP1 )
#define MMCSD_WRITE_BLOCK               (MMCSD_CMD24 | MMCSD_RSP1 )/*| MMC_BSYEXP)*/
/* Yes, this should probably go in asm/arch/gio.h */



typedef enum {
    gio_output = 0,
    gio_input
    } gio_direction;

struct mmci_host {
    //void __iomem          *base;
    struct mmc_request      *mrq;
    struct mmc_command      *cmd;
    struct mmc_data         *data;
    struct mmc_host         *mmc;
//  void                    *clk;

    unsigned int            data_xfered;

    spinlock_t              lock;

    unsigned int            mclk;
    unsigned int            cclk;
    u32                     pwr;
    struct mmc_platform_data *plat;

    struct timer_list       timer;
    unsigned int            oldstat;

    unsigned int            sg_len;

    /* pio stuff */
    struct scatterlist      *sg_ptr;
    unsigned int            sg_off;
    unsigned int            size;
    struct work_struct      read_write;     //For read write calls instead of interrupts
};

static inline void mmci_init_sg(struct mmci_host *host, struct mmc_data *data)
{
    /*
     * Ideally, we want the higher levels to pass us a scatter list.
     */
    host->sg_len = data->sg_len;
    host->sg_ptr = data->sg;
    host->sg_off = 0;
}

static inline int mmci_next_sg(struct mmci_host *host)
{
    host->sg_ptr++;
    host->sg_off = 0;
    host->sg_len = host->sg_len -1;        
    return ((host->sg_len));
}

static inline char *mmci_kmap_atomic(struct mmci_host *host, unsigned long *flags)
{
    struct scatterlist *sg = host->sg_ptr;

    local_irq_save(*flags);
    return kmap_atomic(sg->page, KM_BIO_SRC_IRQ) + sg->offset;
}

static inline void mmci_kunmap_atomic(struct mmci_host *host, unsigned long *flags)
{
    kunmap_atomic(host->sg_ptr->page, KM_BIO_SRC_IRQ);
    local_irq_restore(*flags);
}

#endif // DM320_MMC_H
