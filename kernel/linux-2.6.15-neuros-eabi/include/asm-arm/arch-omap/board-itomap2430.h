/*
 * linux/include/asm-arm/arch-omap/board-itomap2430.h
 *
 * Hardware definitions for IT OMAP2430 board.
 *
 * Initial creation by Dirk Behme <dirk.behme@de.bosch.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * THIS SOFTWARE IS PROVIDED ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN
 * NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 * USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#ifndef __ASM_ARCH_OMAP_ITOMAP2430_H
#define __ASM_ARCH_OMAP_ITOMAP2430_H

#include <asm/memory.h>
#include <asm/arch/memory.h>

extern void* high_memory;

#define ITOMAP2430_CS0_BASE     0x04000000
#define ITOMAP2430_CS1_BASE     0x08000000
#define ITOMAP2430_CS2_BASE     0x0a000000

// BEGIN changes for audio [SPP]
// These may need to be unified and then the mcbsp.c may need changed
#define ITOMAP2430_PERI_PADDR   0x40000000
#define ITOMAP2430_PERI_VADDR   0xd0000000
#define ITOMAP2430_PERI_ADDR(x)	(x - ITOMAP2430_PERI_PADDR + ITOMAP2430_PERI_VADDR)

#define ITOMAP2430_MCBSP_BASE_ADDR  0x48074000

#define MCBSP_REG_VBASE        ITOMAP2430_PERI_ADDR(ITOMAP2430_MCBSP_BASE_ADDR)
#define MCBSP_REG_PBASE        (ITOMAP2430_MCBSP_BASE_ADDR)
// END changes for audio [SPP]

#define OMAP24XX_ETHR_GPIO_IRQ  74
#define OMAP24XX_ETHR_START     (ITOMAP2430_CS2_BASE + 0x300)

#define OMAP24XX_HDD_USB_ENABLE 15
#define OMAP24XX_HDD_GPIO_IRQ   71
#define OMAP24XX_HDD_START      ITOMAP2430_CS1_BASE

#define OMAP24XX_UNUSED_GPIO66  66
#define OMAP24XX_BTN_SCAN0      67
#define OMAP24XX_BTN_SCAN1      68
#define OMAP24XX_BTN_SCAN2      69
#define OMAP24XX_BTN_SCAN3      70
#define OMAP24XX_BTN_SCAN4      85
#define OMAP24XX_BTN_SCAN_SEL   86

#define OMAP24XX_EXT_VID_DET    58

#define OMAP24XX_MMC_EN_PWR     63
#define OMAP24XX_MMC_CD_IRQ     62
#define OMAP24XX_MMC_WP         61

#define OMAP24XX_MMC_EN_PWR     63
#define OMAP24XX_MMC_CD_IRQ     62
#define OMAP24XX_MMC_WP         61

#define PHYS_SDRAM_1            PHYS_OFFSET

/* DSP internal memory */
#define IVA21_INTL_ADDR_PHY     0x5c000000
#define IVA21_INTL_ADDR_VIR     0xdc000000
#define IVA21_INTL_BASE         IVA21_ADDR_PHY
#define IVA21_INTL_START        IVA21_ADDR_VIR
#define IVA21_INTL_SIZE         SZ_16M

/* DSP MMU memory */
#define IVA21_MMU_ADDR_PHY      0x5d000000
#define IVA21_MMU_ADDR_VIR      0xdd000000
#define IVA21_MMU_BASE          IVA21_MMU_ADDR_PHY
#define IVA21_MMU_START         IVA21_MMU_ADDR_VIR
#define IVA21_MMU_SIZE          SZ_16M

/* DSP code memory */
#define IVA21_CODE_PHY     0x83000000
#define IVA21_CODE_VIR     0xda000000
#define IVA21_CODE_BASE    IVA21_CODE_PHY
#define IVA21_CODE_START   IVA21_CODE_VIR
#define IVA21_CODE_SIZE    SZ_16M

#define __idsp_code_v2p(a)           ((a) - IVA21_CODE_VIR + IVA21_CODE_PHY)
#define __idsp_code_p2v(a)           ((a) - IVA21_CODE_PHY + IVA21_CODE_VIR)

/* ARM/DSP message memory */
#define IPC_SHARE_PHY           (IVA21_CODE_PHY - SZ_1M)
#define IPC_SHARE_VIR           0xdb000000
#define IPC_SHARE_BASE          IPC_SHARE_PHY
#define IPC_SHARE_START         IPC_SHARE_VIR
#define IPC_SHARE_SIZE          SZ_1M

/* IMEM memory */
#define IMEM_ADDR_PHY           virt_to_phys(high_memory)
#define IMEM_ADDR_VIR           0xe0000000
#define IMEM_BASE               IMEM_ADDR_VIR
#define IMEM_START              IMEM_ADDR_PHY
#define IMEM_SIZE               (IPC_SHARE_PHY - IMEM_ADDR_PHY)

#define __imem(a)               ((a) + IMEM_ADDR_VIR)
#define __imem_v2p(a)           ((a) - IMEM_ADDR_VIR + IMEM_ADDR_PHY)
#define __imem_p2v(a)           ((a) - IMEM_ADDR_PHY + IMEM_ADDR_VIR)

/* use backwards naming */
#define DSP_IPC_BASE            IPC_SHARE_VIR
// use ioremap() and iounmap() #define DSP_BASE                IVA21_CODE_VIR
#define DSP_START               IVA21_CODE_PHY
#define DSP_SIZE                IVA21_CODE_SIZE
#define DSP_L1DSRAM_BASE        IVA21_ADDR_VIR + 0x00f04000
#define DSP_L1DSRAM_PADDR       IVA21_ADDR_PHY + 0x00f04000
#define DSP_L1DSRAM_SIZE        (SZ_16K * 3)
#define IRQ_CCD_VD0             INT_24XX_CAM_IRQ
#define IRQ_VENC                INT_24XX_DSS_IRQ
#define IRQ_PREVIEW0            0

/* HDD memory */
#define HDD_ADDR_PHYS           OMAP24XX_HDD_START
#define HDD_ADDR_VIRT           0xf9000000
#define HDD_SIZE                SZ_16M

#define __ide_hdd(a) (HDD_ADDR_VIRT + a)

/* Buttons */
#define GIO_NAV_SCANSEL         OMAP24XX_BTN_SCAN_SEL
#define GIO_NAV_RIGHT           OMAP24XX_BTN_SCAN2
#define GIO_NAV_UP              OMAP24XX_BTN_SCAN3
#define GIO_NAV_LEFT            OMAP24XX_BTN_SCAN0
#define GIO_NAV_DOWN            OMAP24XX_BTN_SCAN1
#define GIO_NAV_SEL             OMAP24XX_BTN_SCAN4

#endif /*  __ASM_ARCH_OMAP_ITOMAP2430_H */
