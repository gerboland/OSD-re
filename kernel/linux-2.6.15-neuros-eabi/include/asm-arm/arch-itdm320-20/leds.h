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

#ifndef __ASM_ARCH_LEDS_H
#define __ASM_ARCH_LEDS_H

#include <asm/arch/gio.h>

#define IT_LED_IOC_MAGIC 'i'

#define IT_IOCLED_ON  _IO(IT_LED_IOC_MAGIC, 0)
#define IT_IOCLED_OFF _IO(IT_LED_IOC_MAGIC, 1)

#define IT_LED_IOC_NR 2

#define IT_LED_1 GIO_LED_1
#define IT_LED_2 GIO_LED_2
#define IT_LED_3 GIO_LED_3
#define IT_LED_4 GIO_LED_4 

#ifdef __KERNEL__
#ifdef CONFIG_INGENIENT_LED
extern void it_led_on(unsigned char led);
extern void it_led_off(unsigned char led);
#else
#define it_led_on(led) do {} while(0)
#define it_led_off(led) do {} while(0)
#endif
#endif

#endif
