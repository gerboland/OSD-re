/*
 * linux/include/asm-arm/arch-omap/itmmc.h
 *
 */

#ifndef __ASM_ARCH_MMC_H
#define __ASM_ARCH_MMC_H

#define OMAP24XX_VA_SYSTEM_CONTROL_BASE IO_ADDRESS(OMAP24XX_SCM_BASE)

#if defined(CONFIG_OMAP2430_MMC1)
#define OMAP24XX_MMC_SD_START           OMAP24XX_MMC_SD_1_BASE
#define MMC_IRQ                         INT_24XX_MMC1_IRQ
#elif  defined(CONFIG_OMAP2430_MMC2)
#define OMAP24XX_MMC_SD_START           OMAP24XX_MMC_SD_2_BASE
#define MMC_IRQ                         INT_24XX_MMC2_IRQ
#endif

#define OMAP_HSMMC_BASE         IO_ADDRESS(OMAP24XX_MMC_SD_START)

#define OMAP_HSMMC_SYSCONFIG    (OMAP_HSMMC_BASE + 0x0010)
#define OMAP_HSMMC_SYSSTATUS    (OMAP_HSMMC_BASE + 0x0014)
#define OMAP_HSMMC_CSRE         (OMAP_HSMMC_BASE + 0x0024)
#define OMAP_HSMMC_SYSTEST      (OMAP_HSMMC_BASE + 0x0028)
#define OMAP_HSMMC_CON          (OMAP_HSMMC_BASE + 0x002C)
#define OMAP_HSMMC_BLK          (OMAP_HSMMC_BASE + 0x0104)
#define OMAP_HSMMC_ARG          (OMAP_HSMMC_BASE + 0x0108)
#define OMAP_HSMMC_CMD          (OMAP_HSMMC_BASE + 0x010C)
#define OMAP_HSMMC_RSP10        (OMAP_HSMMC_BASE + 0x0110)
#define OMAP_HSMMC_RSP32        (OMAP_HSMMC_BASE + 0x0114)
#define OMAP_HSMMC_RSP54        (OMAP_HSMMC_BASE + 0x0118)
#define OMAP_HSMMC_RSP76        (OMAP_HSMMC_BASE + 0x011C)
#define OMAP_HSMMC_DATA         (OMAP_HSMMC_BASE + 0x0120)
#define OMAP_HSMMC_PSTATE       (OMAP_HSMMC_BASE + 0x0124)
#define OMAP_HSMMC_HCTL         (OMAP_HSMMC_BASE + 0x0128)
#define OMAP_HSMMC_SYSCTL       (OMAP_HSMMC_BASE + 0x012C)
#define OMAP_HSMMC_STAT         (OMAP_HSMMC_BASE + 0x0130)
#define OMAP_HSMMC_IE           (OMAP_HSMMC_BASE + 0x0134)
#define OMAP_HSMMC_ISE          (OMAP_HSMMC_BASE + 0x0138)
#define OMAP_HSMMC_AC12         (OMAP_HSMMC_BASE + 0x013C)
#define OMAP_HSMMC_CAPA         (OMAP_HSMMC_BASE + 0x0140)
#define OMAP_HSMMC_CUR_CAPA     (OMAP_HSMMC_BASE + 0x0148)
#define OMAP_HSMMC_REV          (OMAP_HSMMC_BASE + 0x01FC)

#define OMAP_HSMMC_END_OF_CMD       (1 << 0)    // End of command phase
#define OMAP_HSMMC_BLOCK_RS         (1 << 1)    // Block received/sent 
#define OMAP_HSMMC_CMD_TIMEOUT      (1 << 16)   // Command response time-out
#define OMAP_HSMMC_DATA_TIMEOUT     (1 << 20)   // Data response time-out
#define OMAP_HSMMC_CMD_CRC          (1 << 17)   // Command CRC error
#define OMAP_HSMMC_DATA_CRC         (1 << 21)   // Date CRC error
#define OMAP_HSMMC_CARD_ERR         (1 << 28)   // Card status error in response
#define OMAP_HSMMC_A_FULL           (1 << 5)    // Buffer almost full
#define OMAP_HSMMC_A_EMPTY          (1 << 4)    // Buffer almost empty

#define OMAP_MMC_CARD_BUSY          (1 << 2)    // Card enter busy state
#define OMAP_MMC_EOF_BUSY           (1 << 4)    // Card exit busy state
#define OMAP_MMC_OCR_BUSY           (1 << 12)   // OCR busy
#define OMAP_MMC_CARD_IRQ           (1 << 13)   // Card IRQ received

#endif // __ASM_ARCH_MMC_H
