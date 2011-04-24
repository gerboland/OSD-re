/*
 *  linux/include/asm-arm/arch-davinci/hardware.h
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

#ifndef __ASM_ARCH_HARDWARE_H
#define __ASM_ARCH_HARDWARE_H

/**************************************************************************
 * Included Files
 **************************************************************************/

#include <linux/config.h>
#include <asm/sizes.h>
#include <asm/arch/io_registers.h>
#include <asm/arch/memory.h>
#include <asm/arch/io.h>

/* FLUSH_BASE.  This is a read only region is is read to flush D-Cache. 
 * Ideally, it should point at some fast-to-access region.  Some machines 
 * have a specific range of addresses that are designated for this purpose.
 *
 * The mapped size of the flush region will be PGDIR_SIZE (see
 * inclue/asm/pgtable.h and arch/arm/mm/mm-armv.c).  At present this is
 * 2Mb.  However, not all of this address space is used.  The actual
 * lush is performed in arch/arm/mm/cache-v4wb.S and a much smaller
 * memory access is made. So we can use still our smaller IRAM address
 * space for the D-Cache flush.
 *
 * FLUSH_BASE_MINICACHE. (Not used in the DAVINCI/ARM926 architecture)
 */

#define FLUSH_BASE                 DAVINCI_IRAM_VIRT
#define FLUSH_BASE_PHYS            DAVINCI_RESET_VECTOR_BASE

/* DAVINCI Register Definitions */
#include <asm/arch/registers.h>

/* Power and Sleep Controller (PSC) Domains */
#define DAVINCI_GPSC_ARMDOMAIN      0
#define DAVINCI_GPSC_DSPDOMAIN      1

/* PSC Domains */
#define DAVINCI_LPSC_VPSSMSTR       0       // VPSS Master LPSC
#define DAVINCI_LPSC_VPSSSLV        1       // VPSS Slave LPSC
#define DAVINCI_LPSC_TPCC           2       // TPCC LPSC
#define DAVINCI_LPSC_TPTC0          3       // TPTC0 LPSC
#define DAVINCI_LPSC_TPTC1          4       // TPTC1 LPSC
#define DAVINCI_LPSC_EMAC           5       // EMAC LPSC
#define DAVINCI_LPSC_EMAC_WRAPPER   6       // EMAC WRAPPER LPSC
#define DAVINCI_LPSC_MDIO           7       // MDIO LPSC
#define DAVINCI_LPSC_IEEE1394       8       // IEEE1394 LPSC
#define DAVINCI_LPSC_USB            9       // USB LPSC
#define DAVINCI_LPSC_ATA            10      // ATA LPSC
#define DAVINCI_LPSC_VLYNQ          11      // VLYNQ LPSC
#define DAVINCI_LPSC_UHPI           12      // UHPI LPSC
#define DAVINCI_LPSC_DDR_EMIF       13      // DDR_EMIF LPSC
#define DAVINCI_LPSC_AEMIF          14      // AEMIF LPSC
#define DAVINCI_LPSC_MMC_SD         15      // MMC_SD LPSC
#define DAVINCI_LPSC_MEMSTICK       16      // MEMSTICK LPSC
#define DAVINCI_LPSC_McBSP          17      // McBSP LPSC
#define DAVINCI_LPSC_I2C            18      // I2C LPSC
#define DAVINCI_LPSC_UART0          19      // UART0 LPSC
#define DAVINCI_LPSC_UART1          20      // UART1 LPSC
#define DAVINCI_LPSC_UART2          21      // UART2 LPSC
#define DAVINCI_LPSC_SPI            22      // SPI LPSC
#define DAVINCI_LPSC_PWM0           23      // PWM0 LPSC
#define DAVINCI_LPSC_PWM1           24      // PWM1 LPSC
#define DAVINCI_LPSC_PWM2           25      // PWM2 LPSC
#define DAVINCI_LPSC_GPIO           26      // GPIO LPSC
#define DAVINCI_LPSC_TIMER0         27      // TIMER0 LPSC
#define DAVINCI_LPSC_TIMER1         28      // TIMER1 LPSC
#define DAVINCI_LPSC_TIMER2         29      // TIMER2 LPSC
#define DAVINCI_LPSC_SYSTEM_SUBSYS  30      // SYSTEM SUBSYSTEM LPSC
#define DAVINCI_LPSC_ARM            31      // ARM LPSC
#define DAVINCI_LPSC_SCR2           32      // SCR2 LPSC
#define DAVINCI_LPSC_SCR3           33      // SCR3 LPSC
#define DAVINCI_LPSC_SCR4           34      // SCR4 LPSC
#define DAVINCI_LPSC_CROSSBAR       35      // CROSSBAR LPSC
#define DAVINCI_LPSC_CFG27          36      // CFG27 LPSC
#define DAVINCI_LPSC_CFG3           37      // CFG3 LPSC
#define DAVINCI_LPSC_CFG5           38      // CFG5 LPSC
#define DAVINCI_LPSC_GEM            39      // GEM LPSC
#define DAVINCI_LPSC_IMCOP          40      // IMCOP LPSC

/* DAVINCI Register Definitions */

# include <asm/arch/registers.h>

#define MCBSP_REG_VBASE        DAVINCI_PERI_ADDR(DAVINCI_MCBSP_BASE_ADDR)
#define MCBSP_REG_PBASE        (DAVINCI_MCBSP_BASE_ADDR)

#endif /* __ASM_ARCH_HARDWARE_H */
