/*
 *  linux/include/asm-arm/arch-davinci/io.h
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

#ifndef __ASM_ARCH_IO_H
#define __ASM_ARCH_IO_H

#include <asm/arch/hardware.h>

#define IO_SPACE_LIMIT 0xffffffff

/*
 * ----------------------------------------------------------------------------
 * I/O mapping
 * ----------------------------------------------------------------------------
 */
#define IO_PHYS	     DAVINCI_PERI_PADDR 
#define IO_VIRT	     DAVINCI_PERI_VADDR
#define IO_SIZE	     DAVINCI_PERI_SIZE

#define io_p2v(pa)   (((pa) & (IO_SIZE-1)) + IO_VIRT)
#define io_v2p(va)   (((va) & (IO_SIZE-1)) + IO_PHYS)
#define IO_ADDRESS(x) io_p2v(x)
#define __REG(x)	(*((volatile unsigned long *)io_p2v(x)))

/* We don't actually have real ISA nor PCI buses, but there is so many
 * drivers out there that might just work if we fake them...
 */

/* Original BSP passes virtual address to inb/inw/inl(), but */
/* our code passes an offset into peripheral space. */
/* Allow either style of access. */
#define __io(a)                 ((((unsigned long)a) < 0x1000000) ? (DAVINCI_PERI_VADDR+((unsigned long)a)) : (a))
#define __mem_pci(a)            ((unsigned long)(a))
#define __mem_isa(a)            ((unsigned long)(a))

#ifndef __ASSEMBLY__
static inline unsigned short __arch_getw(unsigned int addr)
{
	unsigned short retval;
  	__asm__ __volatile__("\tldrh %0, [%1]\n\t" : "=r"(retval) : "r"(addr));
  	return retval;
}
static inline void __arch_putw(unsigned short val, unsigned int addr)
{
 	__asm__ __volatile__("\tstrh %0, [%1]\n\t": : "r"(val), "r"(addr));
}
#endif

#define iomem_valid_addr(iomem,sz) (1)
#define iomem_to_phys(iomem)       (iomem)

#endif /* __ASM_ARCH_IO_H */


