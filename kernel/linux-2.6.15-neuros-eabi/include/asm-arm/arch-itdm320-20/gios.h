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

#ifndef __ASM_ARCH_GIOS_H
#define __ASM_ARCH_GIOS_H


#define GIO_LED_1 16
#define GIO_LED_2 17
#define GIO_LED_3 0
#define GIO_LED_4 0

//#ifdef CONFIG_INGENIENT_NAVIGATOR
#define GIO_NAV_SCANSEL 16
#define GIO_NAV_RIGHT 1
#define GIO_NAV_UP	  2
#define GIO_NAV_LEFT  3
#define GIO_NAV_DOWN  4
#define GIO_NAV_SEL	  5
//#endif
#define GIO_HDD_HOTPLUG 2
#define GIO_ETHER 6
#define GIO_HDD 7
#define GIO_USB_ENABLE 0
#define GIO_AIC23_FREQ 18
#define GIO_I2C_SCL 30
#define GIO_I2C_SDA 31
#define GIO_CFC_HOTPLUG 25
#define GIO_UART1_RXD	27
#define GIO_UART1_TXD	28
#define GIO_CFC_RESET   36
#define GIO_FIELD_ID 39

#define GIO_VIDEO_IN 10
#endif
