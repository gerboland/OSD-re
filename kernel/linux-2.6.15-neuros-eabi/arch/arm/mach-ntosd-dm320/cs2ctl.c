/*
 *  Copyright(C) 2007 Neuros Technology International LLC.
 *               <www.neurostechnology.com>
 *
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, version 2 of the License.
 *
 *
 *  This program is distributed in the hope that, in addition to its
 *  original purpose to support Neuros hardware, it will be useful
 *  otherwise, but WITHOUT ANY WARRANTY; without even the implied
 *  warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *  See the GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 *
 ****************************************************************************
 *
 * CS2 configuration.
 * nand flash and CF card use the same controler of DM320 
 *
 * REVISION:
 *
 *
 * 1) Initial creation. ---------------------------- 2007-08-27 TQ
 *
 */

#include <asm/uaccess.h>
#include <asm/io.h>
#include <asm/arch/io_registers.h>
#include <asm/arch/gio.h>

DECLARE_MUTEX(cs2controler_sem);
static int is_norflash = 0; /* 1 is nor, 0 is nand */

void dm320_controler_enable_nand(void)
{
    gio_set_bitclr(GIO_NAND_CF1);
    outw( 1, IO_EMIF_CFCTRL1 );
    outw( 0, IO_EMIF_CFCTRL2 );
}

void dm320_controler_enable_cfc(void)
{
    gio_set_bitset(GIO_NAND_CF1);
    outw(0x0000, IO_EMIF_CFCTRL1); /* Seltect CF card interface */
    outw(0x0011, IO_EMIF_CFCTRL2); /* Use dynamic bus sizing and output
				      high REG when accessing CFC common memory */
}

void dm320_controler_get_lock(void)
{
    //printk("dm320_controler_get_lock\n");
    up(&cs2controler_sem);
}

void dm320_controler_release_lock(void)
{
    //printk("dm320_controler_release_lock\n");
    down_interruptible(&cs2controler_sem);
}

void set_flash_type(void)
{
    is_norflash = 1;
}

int get_flash_type(void)
{
    return is_norflash;
}
