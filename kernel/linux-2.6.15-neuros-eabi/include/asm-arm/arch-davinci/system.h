/*
 *  linux/include/asm-arm/arch-davinci/system.h
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

#ifndef __ASM_ARCH_SYSTEM_H
#define __ASM_ARCH_SYSTEM_H

/**************************************************************************
 * Included Files
 **************************************************************************/

#include <asm/io.h>
#include <asm/hardware.h>
#include <asm/arch/clockc.h>
#include <asm/arch/pinmux.h>

extern void arch_prepare_for_reset(void);

/**************************************************************************
 * Global Functions
 **************************************************************************/

static void arch_idle(void)
{
	if (!hlt_counter) {
	      	unsigned long flags;
	      	local_irq_save(flags);
	      	if (!need_resched())
			cpu_do_idle();
	      	local_irq_restore(flags);
    	}
}
#define ARCH_HANDLE_REBOOT_CMD
#define DAVINCI_REBOOT_MAGIC1 0x65676e49 
#define DAVINCI_REBOOT_MAGIC2 0x6e65696e 
#define DAVINCI_REBOOT_MAGIC3 0x74625274

static void inline arch_handle_reboot_cmd( char *cmd )
{
    u32  *pIRAM = (u32 *)(INGENIENT_REBOOT_FLAG_VADDR);

    if ( 0 == cmd ) {
        return;
    }
    
    *pIRAM++ = DAVINCI_REBOOT_MAGIC1;
    *pIRAM++ = DAVINCI_REBOOT_MAGIC2;
    *pIRAM++ = DAVINCI_REBOOT_MAGIC3;
    *pIRAM++ = strlen(cmd);
    strcpy( (char *)pIRAM, cmd );

    return;
}

static void inline arch_reset(char mode)
{
        /* Disable interrupts */
        local_irq_disable();

        /* Enable external memory interface */
        ( void ) pinmux_emif_save( );

        /* Powerdown modules in anticipation of warm reset */
        arch_prepare_for_reset();

        /* Code taken from bootloader/cpu/arm926ejs/start.S: reset_cpu */
	__asm__ __volatile__(					
	"mov     ip, #0\n"
	"mcr     p15, 0, ip, c7, c7, 0           @ invalidate cache\n"
	"mcr     p15, 0, ip, c8, c7, 0           @ flush TLB (v4)\n"
	"mrc     p15, 0, ip, c1, c0, 0           @ get ctrl register\n"
	"bic     ip, ip, #0x000f                 @ ............wcam\n"
	"bic     ip, ip, #0x2100                 @ ..v....s........\n"
	"mcr     p15, 0, ip, c1, c0, 0           @ ctrl register\n"
	"mov     ip, #0x02000000\n" /* jump to flash */
	"mov     pc, ip\n"
	:
	:
	: "cc" );
}

#endif /* __ASM_ARCH_SYSTEM_H */
