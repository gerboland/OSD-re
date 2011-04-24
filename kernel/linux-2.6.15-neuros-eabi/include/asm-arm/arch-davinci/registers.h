/*
 *  linux/include/asm-arm/arch-davinci/registers.h
 *
 *  BRIEF MODULE DESCRIPTION
 *      DAVINCI Virtual memofy definitions
 *
 *  Copyright (C) 2004 Texas Instruments.
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *  THIS  SOFTWARE  IS PROVIDED   ``AS  IS'' AND   ANY  EXPRESS OR IMPLIED
 *  WARRANTIES,   INCLUDING, BUT NOT  LIMITED  TO, THE IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN
 *  NO  EVENT  SHALL   THE AUTHOR  BE    LIABLE FOR ANY   DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 *  NOT LIMITED   TO, PROCUREMENT OF  SUBSTITUTE GOODS  OR SERVICES; LOSS OF
 *  USE, DATA,  OR PROFITS; OR  BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 *  ANY THEORY OF LIABILITY, WHETHER IN  CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 *  THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *  You should have received a copy of the  GNU General Public License along
 *  with this program; if not, write  to the Free Software Foundation, Inc.,
 *  675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */


#ifndef __ASM_ARCH_REGISTERS_H
#define __ASM_ARCH_REGISTERS_H

/**************************************************************************
 * Conditional Compilation
 **************************************************************************/

/**************************************************************************
 * Definitions
 **************************************************************************/
#ifndef __ASSEMBLY__
typedef volatile struct {
    unsigned int    pid;   // Peripheral ID
    unsigned int    pcr;   // Peripheral Control
    unsigned int    cfg;   // PWM configuration
    unsigned int    start; // PWM Start
    unsigned int    rpt;   // PWM Repeat count (for one-shot mode only)
    unsigned int    per;   // PWM Period
    unsigned int    ph1d;  // PWM phase 1 duration
} davinci_pwm_registers;

typedef volatile struct {
    unsigned int   pid;           // Peripheral ID
    unsigned int   pcr;           // Peripheral Control
    unsigned int   afpax1;        // Auto-Focus Paxel config 1
    unsigned int   afpax2;        // AF Paxel config 2
    unsigned int   afpaxstart;    // AF Paxel start position
    unsigned int   afiirsh;       // Start position for IIR filter
    unsigned int   afbufst;       // RAM start address for AF 
    unsigned int   afcoefset0[6]; // IIR coefficient set 0 for AF
    unsigned int   afcoefset1[6]; // IIR coefficient set 1 for AF
    unsigned int   aewwin1;       // Window config for Auto-Exposure/Auto White Balance
    unsigned int   aewinstart;    // Start position for AE/AWB 
    unsigned int   aewinblk;      // Black line window for AE/AWB 
    unsigned int   aewsubwin;     // Subsample window config for AE/AWB 
    unsigned int   aewbufst;      // RAM start address for AE/AWB  
} davinci_h3a_registers;
#endif

/**************************************************************************
 * Memory Map
 **************************************************************************/

#define DAVINCI_DMA_3PCC_BASE_ADDR              0x01C00000
#define DAVINCI_DMA_3PTC0_BASE_ADDR             0x01C10000
#define DAVINCI_DMA_3PTC1_BASE_ADDR             0x01C10400
#define DAVINCI_UART0_BASE_ADDR                 0x01C20000
#define DAVINCI_UART1_BASE_ADDR                 0x01C20400
#define DAVINCI_UART2_BASE_ADDR                 0x01C20800
#define DAVINCI_I2C_BASE_ADDR                   0x01C21000
#define DAVINCI_TIMER0_BASE_ADDR                0x01C21400
#define DAVINCI_TIMER1_BASE_ADDR                0x01C21800
#define DAVINCI_WDOG_BASE_ADDR                  0x01C21C00
#define DAVINCI_PWM0_BASE_ADDR                  0x01C22000
#define DAVINCI_PWM1_BASE_ADDR                  0x01C22400
#define DAVINCI_PWM2_BASE_ADDR                  0x01C22800
#define DAVINCI_SYSTEM_MODULE_BASE_ADDR         0x01C40000
#define DAVINCI_PLL_CNTRL0_BASE_ADDR            0x01C40800
#define DAVINCI_PLL_CNTRL1_BASE_ADDR            0x01C40C00
#define DAVINCI_PWR_SLEEP_CNTRL_BASE_ADDR       0x01C41000
#define DAVINCI_SYSTEM_DFT_BASE_ADDR            0x01C42000
#define DAVINCI_ARM_INTC_BASE_ADDR              0x01C48000
#define DAVINCI_IEEE1394_BASE_ADDR              0x01C60000
#define DAVINCI_USB_OTG_BASE_ADDR               0x01C64000
#define DAVINCI_CFC_ATA_BASE_ADDR               0x01C66000
#define DAVINCI_SPI_BASE_ADDR                   0x01C66800
#define DAVINCI_GPIO_BASE_ADDR                  0x01C67000
#define DAVINCI_UHPI_BASE_ADDR                  0x01C67800
#define DAVINCI_VPSS_REGS_BASE_ADDR             0x01C70000
#define DAVINCI_CCDC_REGS_BASE_ADDR             0x01C70400
#define DAVINCI_PREVIEW_REGS_BASE_ADDR          0x01C70800
#define DAVINCI_RESIZER_REGS_BASE_ADDR          0x01C70C00
#define DAVINCI_HIST_REGS_BASE_ADDR             0x01C71000
#define DAVINCI_H3A_REGS_BASE_ADDR              0x01C71400
#define DAVINCI_VFOCUS_REGS_BASE_ADDR           0x01C71800
#define DAVINCI_EMAC_CNTRL_REGS_BASE_ADDR       0x01C80000
#define DAVINCI_EMAC_WRAPPER_CNTRL_REGS_BASE_ADDR   0x01C81000
#define DAVINCI_EMAC_WRAPPER_RAM_BASE_ADDR      0x01C82000
#define DAVINCI_MDIO_CNTRL_REGS_BASE_ADDR       0x01C84000
#define DAVINCI_IMCOP_BASE_ADDR                 0x01CC0000
#define DAVINCI_ASYNC_EMIF_CNTRL_BASE_ADDR      0x01E00000
#define DAVINCI_VLYNQ_BASE_ADDR                 0x01E01000
#define DAVINCI_MCBSP_BASE_ADDR                 0x01E02000
#define DAVINCI_MMC_SD_BASE_ADDR                0x01E10000
#define DAVINCI_MS_BASE_ADDR                    0x01E20000
#define DAVINCI_ASYNC_EMIF_DATA_CE0_BASE_ADDR   0x02000000
#define DAVINCI_ASYNC_EMIF_DATA_CE1_BASE_ADDR   0x04000000
#define DAVINCI_ASYNC_EMIF_DATA_CE2_BASE_ADDR   0x06000000
#define DAVINCI_ASYNC_EMIF_DATA_CE3_BASE_ADDR   0x08000000
#define DAVINCI_VLYNQ_REMOTE_BASE_ADDR          0x0C000000

/* Somebody felt like removing the _ADDR suffix ...*/
#define DAVINCI_DMA_3PCC_BASE              0x01C00000
#define DAVINCI_DMA_3PTC0_BASE             0x01C10000
#define DAVINCI_DMA_3PTC1_BASE             0x01C10400
#define DAVINCI_UART0_BASE                 0x01C20000
#define DAVINCI_UART1_BASE                 0x01C20400
#define DAVINCI_UART2_BASE                 0x01C20800
#define DAVINCI_I2C_BASE                   0x01C21000
#define DAVINCI_TIMER0_BASE                0x01C21400
#define DAVINCI_TIMER1_BASE                0x01C21800
#define DAVINCI_WDOG_BASE                  0x01C21C00
#define DAVINCI_PWM0_BASE                  0x01C22000
#define DAVINCI_PWM1_BASE                  0x01C22400
#define DAVINCI_PWM2_BASE                  0x01C22800
#define DAVINCI_SYSTEM_MODULE_BASE         0x01C40000
#define DAVINCI_PLL_CNTRL0_BASE            0x01C40800
#define DAVINCI_PLL_CNTRL1_BASE            0x01C40C00
#define DAVINCI_PWR_SLEEP_CNTRL_BASE       0x01C41000
#define DAVINCI_SYSTEM_DFT_BASE            0x01C42000
#define DAVINCI_ARM_INTC_BASE              0x01C48000
#define DAVINCI_IEEE1394_BASE              0x01C60000
#define DAVINCI_USB_OTG_BASE               0x01C64000
#define DAVINCI_CFC_ATA_BASE               0x01C66000
#define DAVINCI_SPI_BASE                   0x01C66800
#define DAVINCI_GPIO_BASE                  0x01C67000
#define DAVINCI_UHPI_BASE                  0x01C67800
#define DAVINCI_VPSS_REGS_BASE             0x01C70000
#define DAVINCI_EMAC_CNTRL_REGS_BASE       0x01C80000
#define DAVINCI_EMAC_WRAPPER_CNTRL_REGS_BASE   0x01C81000
#define DAVINCI_EMAC_WRAPPER_RAM_BASE      0x01C82000
#define DAVINCI_MDIO_CNTRL_REGS_BASE       0x01C84000
#define DAVINCI_IMCOP_BASE                 0x01CC0000
#define DAVINCI_ASYNC_EMIF_CNTRL_BASE      0x01E00000
#define DAVINCI_VLYNQ_BASE                 0x01E01000
#define DAVINCI_MCBSP_BASE                 0x01E02000
#define DAVINCI_MMC_SD_BASE                0x01E10000
#define DAVINCI_MS_BASE                    0x01E20000
#define DAVINCI_ASYNC_EMIF_DATA_CE0_BASE   0x02000000
#define DAVINCI_ASYNC_EMIF_DATA_CE1_BASE   0x04000000
#define DAVINCI_ASYNC_EMIF_DATA_CE2_BASE   0x06000000
#define DAVINCI_ASYNC_EMIF_DATA_CE3_BASE   0x08000000
#define DAVINCI_VLYNQ_REMOTE_BASE          0x0C000000

#define PINMUX0 __REG(0x01c40000)
#define PINMUX1 __REG(0x01c40004)

/**************************************************************************
 * Selected Register Bit Settings
 **************************************************************************/

#endif /* __ASM_ARCH_REGISTERS_H */


