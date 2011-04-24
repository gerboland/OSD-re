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

#ifndef _MAILBOX_REGS_H
#define _MAILBOX_REGS_H

#include <asm/arch/omap24xx.h>

#define IO_MAILBOX_BASE             IO_ADDRESS(OMAP24XX_MAILBOX)
#define IO_MAILBOX_REVISION         (IO_MAILBOX_BASE+0x000)
#define IO_MAILBOX_SYSCONFIG        (IO_MAILBOX_BASE+0x010)
#define IO_MAILBOX_SYSSTATUS        (IO_MAILBOX_BASE+0x014)
#define IO_MAILBOX_MESSAGE_0        (IO_MAILBOX_BASE+0x040)
#define IO_MAILBOX_MESSAGE_1        (IO_MAILBOX_BASE+0x044)
#define IO_MAILBOX_MESSAGE_2        (IO_MAILBOX_BASE+0x048)
#define IO_MAILBOX_MESSAGE_3        (IO_MAILBOX_BASE+0x04c)
#define IO_MAILBOX_MESSAGE_4        (IO_MAILBOX_BASE+0x050)
#define IO_MAILBOX_MESSAGE_5        (IO_MAILBOX_BASE+0x054)
#define IO_MAILBOX_FIFOSTATUS_0     (IO_MAILBOX_BASE+0x080)
#define IO_MAILBOX_FIFOSTATUS_1     (IO_MAILBOX_BASE+0x084)
#define IO_MAILBOX_FIFOSTATUS_2     (IO_MAILBOX_BASE+0x088)
#define IO_MAILBOX_FIFOSTATUS_3     (IO_MAILBOX_BASE+0x08c)
#define IO_MAILBOX_FIFOSTATUS_4     (IO_MAILBOX_BASE+0x090)
#define IO_MAILBOX_FIFOSTATUS_5     (IO_MAILBOX_BASE+0x094)
#define IO_MAILBOX_MSGSTATUS_0      (IO_MAILBOX_BASE+0x0c0)
#define IO_MAILBOX_MSGSTATUS_1      (IO_MAILBOX_BASE+0x0c4)
#define IO_MAILBOX_MSGSTATUS_2      (IO_MAILBOX_BASE+0x0c8)
#define IO_MAILBOX_MSGSTATUS_3      (IO_MAILBOX_BASE+0x0cc)
#define IO_MAILBOX_MSGSTATUS_4      (IO_MAILBOX_BASE+0x0d0)
#define IO_MAILBOX_MSGSTATUS_5      (IO_MAILBOX_BASE+0x0d4)
#define IO_MAILBOX_IRQSTATUS_0      (IO_MAILBOX_BASE+0x100)
#define IO_MAILBOX_IRQENABLE_0      (IO_MAILBOX_BASE+0x104)
#define IO_MAILBOX_IRQSTATUS_1      (IO_MAILBOX_BASE+0x108)
#define IO_MAILBOX_IRQENABLE_1      (IO_MAILBOX_BASE+0x10C)
#define IO_MAILBOX_IRQSTATUS_2      (IO_MAILBOX_BASE+0x110)
#define IO_MAILBOX_IRQENABLE_2      (IO_MAILBOX_BASE+0x114)
#define IO_MAILBOX_IRQSTATUS_3      (IO_MAILBOX_BASE+0x118)
#define IO_MAILBOX_IRQENABLE_3      (IO_MAILBOX_BASE+0x11C)

#endif /* _MAILBOX_REGS_H */
