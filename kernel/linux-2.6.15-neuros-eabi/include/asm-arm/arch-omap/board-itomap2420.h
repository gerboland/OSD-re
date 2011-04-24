/*
 * linux/include/asm-arm/arch-omap/board-itomap2420.h
 *
 * Hardware definitions for IT OMAP2420 board.
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

#ifndef __ASM_ARCH_OMAP_ITOMAP2420_H
#define __ASM_ARCH_OMAP_ITOMAP2420_H

#include <asm/memory.h>
#include <asm/arch/memory.h>

extern void* high_memory;

#define ITOMAP2420_CS0_BASE     0x04000000
#define ITOMAP2420_CS1_BASE     0x08000000
#define ITOMAP2420_CS2_BASE     0x0a000000

#define OMAP24XX_ETHR_GPIO_IRQ  74
#define OMAP24XX_ETHR_START     (ITOMAP2420_CS2_BASE + 0x300)

#define PHYS_SDRAM_1            PHYS_OFFSET

#define IVA_ADDR_PHY            0x5c000000
#define IVA_ADDR_VIR            IO_ADDRESS(IVA_ADDR_PHY)
#define IVA_BASE                IVA_ADDR_PHY
#define IVA_START               IVA_ADDR_VIR
#define IVA_SIZE                SZ_512K

#define IVA_MMU_ADDR_PHY        0x5d000000
#define IVA_MMU_ADDR_VIR        IO_ADDRESS(IVA_MMU_ADDR_PHY)
#define IVA_MMU_BASE            IVA_MMU_ADDR_PHY
#define IVA_MMU_START           IVA_MMU_ADDR_VIR
#define IVA_MMU_SIZE            SZ_4K

#define DSP_ADDR_PHY            0x58000000
#define DSP_ADDR_VIR            IO_ADDRESS(DSP_ADDR_PHY)
#define DSP_SIZE_160K           (SZ_128K + 2 * SZ_16K)
#define DSP_BASE                DSP_ADDR_PHY
#define DSP_START               DSP_ADDR_VIR
#define DSP_SIZE                DSP_SIZE_160K

#define DSP_MMU_ADDR_PHY        0x5a000000
#define DSP_MMU_ADDR_VIR        IO_ADDRESS(DSP_MMU_ADDR_PHY)
#define DSP_MMU_BASE            DSP_MMU_ADDR_PHY
#define DSP_MMU_START           DSP_MMU_ADDR_VIR
#define DSP_MMU_SIZE            SZ_4K

#define DSP_IPI_ADDR_PHY        0x59000000
#define DSP_IPI_ADDR_VIR        IO_ADDRESS(DSP_IPI_ADDR_PHY)
#define DSP_IPI_BASE            DSP_IPI_ADDR_PHY
#define DSP_IPI_START           DSP_IPI_ADDR_VIR
#define DSP_IPI_SIZE            SZ_4K

//IMEM_ADDR_PHY >= virt_to_phys(high_memory) = 0x80e00000
//IMEM_ADDR_VIR >= ((unsigned long)high_memory) = 0xc0e00000
#define IMEM_ADDR_PHY           0x81000000
#define IMEM_ADDR_VIR           0xc1000000
#define IMEM_BASE               IMEM_ADDR_VIR
#define IMEM_START              IMEM_ADDR_PHY
#define IMEM_SIZE               SZ_16M

#define __imem(a)               ((a) + IMEM_BASE)
#define __imem_v2p(a)           ((a) - IMEM_BASE + IMEM_START)
#define __imem_p2v(a)           ((a) - IMEM_START + IMEM_BASE)


#endif /*  __ASM_ARCH_OMAP_ITOMAP2420_H */

