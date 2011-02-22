#ifndef _ASM_M32R_VGA_H
#define _ASM_M32R_VGA_H

/* $Id: vga.h,v 1.1.1.1 2006/01/06 00:51:59 jsantiago Exp $ */

/*
 *	Access to VGA videoram
 *
 *	(c) 1998 Martin Mares <mj@ucw.cz>
 */

/*
 *	On the PC, we can just recalculate addresses and then
 *	access the videoram directly without any black magic.
 */

#define VGA_MAP_MEM(x) (unsigned long)phys_to_virt(x)

#define vga_readb(x) (*(x))
#define vga_writeb(x,y) (*(y) = (x))

#endif  /* _ASM_M32R_VGA_H */
