#ifndef    DRIVERS_MEDIA_MMC_OMAP_H
#define    DRIVERS_MEDIA_MMC_OMAP_H

#define DRIVER_NAME    "omap-mmc"


#define MMC_REG_BASE            0xf809C000

#define PRCM_CLK_EN_PLL         0x500
#define PRCM_CLK_SEL1_PLL       0x540
#define PRCM_FCLK_EN1_CORE      0x200
#define PRCM_FCLK_EN2_CORE      0x204

#define OMAP_MMC_REG_CMD        0x00
#define OMAP_MMC_REG_ARGL       0x04
#define OMAP_MMC_REG_ARGH       0x08
#define OMAP_MMC_REG_CON        0x0c
#define OMAP_MMC_REG_STAT       0x10
#define OMAP_MMC_REG_IE         0x14
#define OMAP_MMC_REG_CTO        0x18
#define OMAP_MMC_REG_DTO        0x1c
#define OMAP_MMC_REG_DATA       0x20
#define OMAP_MMC_REG_BLEN       0x24
#define OMAP_MMC_REG_NBLK       0x28
#define OMAP_MMC_REG_BUF        0x2c
#define OMAP_MMC_REG_REV        0x3c
#define OMAP_MMC_REG_RSP0       0x40
#define OMAP_MMC_REG_RSP1       0x44
#define OMAP_MMC_REG_RSP2       0x48
#define OMAP_MMC_REG_RSP3       0x4c
#define OMAP_MMC_REG_RSP4       0x50
#define OMAP_MMC_REG_RSP5       0x54
#define OMAP_MMC_REG_RSP6       0x58
#define OMAP_MMC_REG_RSP7       0x5c

#define OMAP_MMC_STAT_CARD_ERR          (1 << 14)
#define OMAP_MMC_STAT_CARD_IRQ          (1 << 13)
#define OMAP_MMC_STAT_OCR_BUSY          (1 << 12)
#define OMAP_MMC_STAT_A_EMPTY           (1 << 11)
#define OMAP_MMC_STAT_A_FULL            (1 << 10)
#define OMAP_MMC_STAT_CMD_CRC           (1 <<  8)
#define OMAP_MMC_STAT_CMD_TOUT          (1 <<  7)
#define OMAP_MMC_STAT_DATA_CRC          (1 <<  6)
#define OMAP_MMC_STAT_DATA_TOUT         (1 <<  5)
#define OMAP_MMC_STAT_END_BUSY          (1 <<  4)
#define OMAP_MMC_STAT_END_OF_DATA       (1 <<  3)
#define OMAP_MMC_STAT_CARD_BUSY         (1 <<  2)
#define OMAP_MMC_STAT_END_OF_CMD        (1 <<  0)

/*
 * Command types
 */
#define OMAP_MMC_CMDTYPE_BC     0
#define OMAP_MMC_CMDTYPE_BCR    1
#define OMAP_MMC_CMDTYPE_AC     2
#define OMAP_MMC_CMDTYPE_ADTC   3


#define OMAP_MMC_BASE   0x4809C000

#define OMAP_USE_DMA               1
#define OMAP_MMC_IRQ              83
#define MMC_REF_CLOCK       96000000
#define MMC_FMAX            24000000
#define MMC_FMIN              400000
#define MMC_DMA_TX_REQUEST        60
#define MMC_DMA_RX_REQUEST        61
#define MMC_REG_SIZE      0x00001000
#define MMC_HP_IRQ                96
/* For Menelaus */
#define ENABLE_SLOT1 0x01
#define ENABLE_SLOT2 0x02
#define ENABLE_BOTH_SLOTS 0x03
static int MMC_CARDDETECT_IRQ = IH_MENELAUS_BASE + 0;


#define menelaus_mmcread_err(r, reg, act_if_fail) \
        do { \
           if ((*r = menelaus_read(reg)) == -1) { \
                       act_if_fail; \
            } \
        } while (0);

#define menelaus_mmcwrite_err(w, reg, act_if_fail) \
        do { \
           if (menelaus_write(w, reg) == -1) { \
                        act_if_fail; \
           } \
        } while (0);

struct mmc_omap_host {
    int                         suspended;
    struct mmc_host             *mmc;
    struct mmc_request *        mrq;
    struct mmc_command *        cmd;
    struct mmc_data *           data;
    struct timer_list           detect_timer;
    void __iomem *              base;
    int                         irq;
    unsigned char               bus_mode;
    struct semaphore            sem;
#define OMAP_MMC_DATADIR_NONE   0
#define OMAP_MMC_DATADIR_READ   1
#define OMAP_MMC_DATADIR_WRITE  2
    unsigned char               datadir;
    u16 *                       buffer;
    u32                         bytesleft;
    int                         use_dma, dma_ch;
};


/* Macro to write/ read to the MMC controller */
#define OMAP_MMC_READ(base, reg)    readw((base)+OMAP_MMC_REG_##reg)
#define OMAP_MMC_WRITE(base, reg, val)    writew(val, (base) + OMAP_MMC_REG_##reg)

#define omap_prcmreg_read(reg) readl(OMAP24XX_VA_PRCM_BASE + reg)
#define omap_prcmreg_write(data,reg) writel(data, OMAP24XX_VA_PRCM_BASE + reg)

#endif
