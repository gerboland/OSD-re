#ifndef    DRIVERS_MEDIA_MMC_OMAP_H
#define    DRIVERS_MEDIA_MMC_OMAP_H

#define DRIVER_NAME    "omap2430-mmc"

/* HSMMC slot1 definitions
 * */
#ifdef CONFIG_OMAP2430_MMC1
#define OMAP24XX_DMA_MMC_RX     OMAP24XX_DMA_MMC1_RX
#define OMAP24XX_DMA_MMC_TX     OMAP24XX_DMA_MMC1_TX
#define MMC_SRC                 (OMAP24XX_MMC_SD_1_BASE + 0x00000120)
#define OMAP_MMC_OCR            0x00ffc000
#define MMC_CARD_PRESENT        0x1
/* HSMMC slot2 definitions
 * */
#elif CONFIG_OMAP2430_MMC2
#define OMAP24XX_DMA_MMC_RX     OMAP24XX_DMA_MMC2_RX
#define OMAP24XX_DMA_MMC_TX     OMAP24XX_DMA_MMC2_TX
#define MMC_SRC                 (OMAP24XX_MMC_SD_2_BASE + 0x00000120)
#define OMAP_MMC_OCR            0x00ff8080
#define MMC_CARD_PRESENT        0x2
#endif

/* HSMMC local definitions
 * */
#define SOFTRST             (1<<1)
#define RSTDONE             0x1
#define CSS                 (1<<17)
#define VS18                (1<<26)
#define VS30                (1<<25)
#define SDVS18              (0x5<<9)
#define SDVS30              (0x6<<9)
#define SIDLE_MODE          (0x2<<3)
#define CDP                 (1<<7)
#define SDBP                (1<<8)
#define DTO                 0xe
#define ICE                 0x1
#define ICS                 0x2
#define CEN                 (1<<2)
#define CLKD_MASK           0x0000FFC0
#define INT_EN_MASK         0x00110033
#define INIT_STREAM         (1<<1)
#define DP_SELECT           (1<<21)
#define DDIR                (1<<4)
#define DMA_EN              0x1
#define MSBS                1<<5
#define BCE                 1<<1
#define ACEN                1<<2
#define ONE_BIT             ~(0x2)
#define EIGHT_BIT           ~(0x20)
#define CC                  0x1
#define TC                  0x02
#define OD                  0x1
#define CD_EN               0x00
#define CD_MASK             0x3
#define CARD_PRESENT        0x0
#define CARD_NOT_PRESENT    0x1
#define SLOT1_SELECT        0x01
#define SLOT1_DESELECT      0x00
#define SLOT1_POWERON       0x03
#define SLOT1_POWEROFF      0x00
#define SLOT2_SELECT        0x03
#define SLOT2_DESELECT      0x00
#define SLOT2_POWERON       0x0B
#define SLOT2_POWEROFF      0x33

/* Menelaus and T2 GPIO register definitions
 * */
#define MCT_CTRL3               0x38
#define LDO_CTRL7               0x10
#define DCDC_CTRL1              0x07
#define DCDC_CTRL3              0x09
#define TWL_GPIO_CTRL           0x0000002a
#define TWL_GPIO1_DIR           0x0000001B
#define TWL_GPIO1_DBN           0x00000027
#define TWL_GPIO1_EDR           0x00000028
#define TWL_GPIO1_IMR1A         0x0000001C
#define TWL_GPIO1_ISR1A         0x00000019
#define TWL_GPIO1_DATAIN1       0x00000000
#define TWL_GPIO_SIR            0x00000025
#define TWL_GPIO_SIH            0x0000002D
#define PBIAS_LITE              0x04A0
#define MMC_NAME                "omap2430hsmmc"
#define OMAP_MMC_MASTER_CLOCK   96000000


#define OMAP_USE_DMA            1
/*
 * Command types
 */
#define OMAP_MMC_CMDTYPE_BC     0
#define OMAP_MMC_CMDTYPE_BCR    1
#define OMAP_MMC_CMDTYPE_AC     2
#define OMAP_MMC_CMDTYPE_ADTC   3

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
    struct work_struct          mmc_carddetect_work;
};

#endif
