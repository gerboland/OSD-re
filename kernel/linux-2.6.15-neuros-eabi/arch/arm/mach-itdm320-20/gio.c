/*
 *
 * Copyright (C) 2004-2006 Ingenient Technologies
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <linux/errno.h>
#include <linux/module.h>

#include <asm/io.h>

#include <asm/arch/gio.h>
#include <asm/arch/io.h>

// Start GIO helper macros
#define __gio_get(gio, base_reg, val) \
	do {\
		unsigned long reg;\
\
		reg = base_reg + ((gio - (gio % 16)) / 8);\
\
		val = (inw(reg) >> (gio % 16)) & 0x0001;\
	} while(0)

#define __gio_set(gio, base_reg, val) \
	do {\
		unsigned long reg;\
\
		reg = base_reg + ((gio - (gio % 16)) / 8);\
\
		if (val)\
			outw(inw(reg) | (1 << (gio % 16)), reg);\
		else\
			outw(inw(reg) & ~(1 << (gio % 16)), reg);\
	} while(0)

#define __gio_overwrite(gio, base_reg, val) \
	do {\
		unsigned long reg;\
\
		reg = base_reg + ((gio - (gio % 16)) / 8);\
\
		outw((1 << (gio % 16)), reg);\
	} while(0)

#define __gio_set_fsel(gio, base_reg, val) \
	do {\
		unsigned long reg;\
		unsigned short result;\
\
		reg = base_reg + ((gio - (gio % 8)) / 4);\
\
		result = inw(reg);\
\
		result &= ~(0x03 << ((gio % 8) * 2));\
		result |= (val << ((gio % 8) * 2));\
\
		outw(result, reg);\
	} while (0)

#define __set_gio_register(gio, val) gio_register[gio] = (!val) ? 0 : 1

#define __is_gio_registered(gio) gio_register[gio]
// End GIO helper macros

static u8 gio_register[41];

int request_gio(unsigned char gio)
{
	if (__is_gio_registered(gio))
		return -EBUSY;

	__set_gio_register(gio, 1);

	return 0;
}
EXPORT_SYMBOL(request_gio);

void unrequest_gio(unsigned char gio)
{
	__set_gio_register(gio, 0);
}
EXPORT_SYMBOL(unrequest_gio);

unsigned char gio_get_bitset(unsigned char gio)
{
	unsigned char bit = 0;

	if (!__is_gio_registered(gio))
		return -1;

	if (gio > 40) return -1;

	__gio_get(gio, IO_GIO_BITSET0, bit);

	return bit;
}
EXPORT_SYMBOL(gio_get_bitset);

unsigned char gio_get_bitclr(unsigned char gio)
{
	unsigned char bit = 0;

	if (!__is_gio_registered(gio))
		return -1;

	if (gio > 40) return -1;

	__gio_get(gio, IO_GIO_BITCLR0, bit);

	return bit;
}
EXPORT_SYMBOL(gio_get_bitclr);

void gio_set_dir(unsigned char gio,
		 gio_bit_t dir)
{
	if (!__is_gio_registered(gio))
		return;

	if (gio > 40) return;

	__gio_set(gio, IO_GIO_DIR0, dir);
}
EXPORT_SYMBOL(gio_set_dir);

void gio_set_chat(unsigned char gio,
		 gio_bit_t inv)
{
	if (!__is_gio_registered(gio))
		return;

	if (gio > 40) return;

	__gio_set(gio, IO_GIO_CHAT0, inv);
}
EXPORT_SYMBOL(gio_set_chat);

void gio_set_fsel(unsigned char gio,
		  unsigned char value)
{
	if (!__is_gio_registered(gio))
		return;

	if ((gio < 9) || (gio > 40))
		return;

	gio -= 9;

	__gio_set_fsel(gio, IO_GIO_FSEL0, value);
}
EXPORT_SYMBOL(gio_set_fsel);

void gio_set_inv(unsigned char gio,
		 gio_bit_t inv)
{
	if (!__is_gio_registered(gio))
		return;

	if (gio > 40) return;

	__gio_set(gio, IO_GIO_INV0, inv);
}
EXPORT_SYMBOL(gio_set_inv);

void gio_set_bitset(unsigned char gio)
{
	if (!__is_gio_registered(gio))
		return;

	if (gio > 40) return;

	__gio_overwrite(gio, IO_GIO_BITSET0, set);
}
EXPORT_SYMBOL(gio_set_bitset);

void gio_set_bitclr(unsigned char gio)
{
	if (!__is_gio_registered(gio))
		return;

	if (gio > 40) return;

	__gio_overwrite(gio, IO_GIO_BITCLR0, clear);
}
EXPORT_SYMBOL(gio_set_bitclr);

void gio_enable_irq(unsigned char gio,
		   gio_edge_t edge)
{
	if (!__is_gio_registered(gio))
		return;

	if (gio > 15) return;

	gio_set_dir(gio, bit_hi);

	switch(edge) {
		case GIO_FALLING_EDGE:
			outw(inw(IO_GIO_IRQEDGE) & ~(1 << gio),
			     IO_GIO_IRQEDGE);

			gio_set_inv(gio, bit_low);

			break;
		case GIO_RISING_EDGE:
			outw(inw(IO_GIO_IRQEDGE) & ~(1 << gio),
			     IO_GIO_IRQEDGE);

			gio_set_inv(gio, bit_hi);

			break;
		case GIO_ANY_EDGE:
		default:
			outw(inw(IO_GIO_IRQEDGE) | (1 << gio),
			     IO_GIO_IRQEDGE);

			gio_set_inv(gio, bit_low);

			break;
	}

	outw(inw(IO_GIO_IRQPORT) | (1 << gio), IO_GIO_IRQPORT);
}
EXPORT_SYMBOL(gio_enable_irq);

void gio_disable_irq(unsigned char gio)
{
	if (!__is_gio_registered(gio))
		return;

	if (gio > 15) return;

	outw(inw(IO_GIO_IRQPORT) & ~(1 << gio), IO_GIO_IRQPORT);

	outw(inw(IO_GIO_IRQEDGE) & ~(1 << gio),
	     IO_GIO_IRQEDGE);

	gio_set_dir(gio, bit_low);

	gio_set_inv(gio, bit_low);
}
EXPORT_SYMBOL(gio_disable_irq);
