/******************************************************************
 * Copyright 2005 Mentor Graphics Corporation
 * Copyright (C) 2005 by Texas Instruments
 *
 * This file is part of the Inventra Controller Driver for Linux.
 * 
 * The Inventra Controller Driver for Linux is free software; you 
 * can redistribute it and/or modify it under the terms of the GNU 
 * General Public License version 2 as published by the Free Software 
 * Foundation.
 * 
 * The Inventra Controller Driver for Linux is distributed in 
 * the hope that it will be useful, but WITHOUT ANY WARRANTY; 
 * without even the implied warranty of MERCHANTABILITY or 
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public 
 * License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with The Inventra Controller Driver for Linux ; if not, 
 * write to the Free Software Foundation, Inc., 59 Temple Place, 
 * Suite 330, Boston, MA  02111-1307  USA
 * 
 * ANY DOWNLOAD, USE, REPRODUCTION, MODIFICATION OR DISTRIBUTION 
 * OF THIS DRIVER INDICATES YOUR COMPLETE AND UNCONDITIONAL ACCEPTANCE 
 * OF THOSE TERMS.THIS DRIVER IS PROVIDED "AS IS" AND MENTOR GRAPHICS 
 * MAKES NO WARRANTIES, EXPRESS OR IMPLIED, RELATED TO THIS DRIVER.  
 * MENTOR GRAPHICS SPECIFICALLY DISCLAIMS ALL IMPLIED WARRANTIES 
 * OF MERCHANTABILITY; FITNESS FOR A PARTICULAR PURPOSE AND 
 * NON-INFRINGEMENT.  MENTOR GRAPHICS DOES NOT PROVIDE SUPPORT 
 * SERVICES OR UPDATES FOR THIS DRIVER, EVEN IF YOU ARE A MENTOR 
 * GRAPHICS SUPPORT CUSTOMER. 
 ******************************************************************/

#ifndef __MUSB_LINUX_DEBUG_H__
#define __MUSB_LINUX_DEBUG_H__

/*
 * Linux HCD (Host Controller Driver) for HDRC and/or MHDRC.
 * Debug support routines
 */

#define MUSB_MONITOR_DATA

#define yprintk(facility, format, args...) do { printk(facility "%s %d: " format , \
	__FUNCTION__, __LINE__ , ## args); } while (0)
#define WARN(fmt, args...) yprintk(KERN_WARNING,fmt, ## args)
#define INFO(fmt,args...) yprintk(KERN_INFO,fmt, ## args)
#define ERR(fmt,args...) yprintk(KERN_ERR,fmt, ## args)

#if MUSB_DEBUG > 0

#define MGC_GetDebugLevel()	(MGC_DebugLevel)
#define MGC_SetDebugLevel(n)	do { MGC_DebugLevel = (n); } while(0)
#define MGC_EnableDebug()	do { MGC_DebugDisable=0; } while(0)
#define MGC_DisableDebug()	do { MGC_DebugDisable=1; } while(0)

#define xprintk(level, facility, format, args...) do { \
	if ( _dbg_level(level) ) { \
		printk(facility "%s %d: " format , \
				__FUNCTION__, __LINE__ , ## args); \
	} } while (0)

#define PARANOID( x )		do {}  while (0)
#define DBG(level,fmt,args...) xprintk(level,KERN_DEBUG,fmt, ## args)
#define DEBUG_CODE(level, code)	do { \
	if ( _dbg_level(level) ) \
		{ code } \
	} while (0)
#define TRACE(n) DEBUG_CODE(n, printk(KERN_INFO "%s:%s:%d: trace\n", \
	__FILE__, __FUNCTION__, __LINE__); )

#define ASSERT_SPINLOCK_LOCKED(_x)
#define ASSERT_SPINLOCK_UNLOCKED(_x)
/* #define ASSERT_SPINLOCK_LOCKED(_x) do { if (!spin_is_locked(_x)) \
	ERR("@pre clause failed, _x must be locked\n"); } while (0) 
#define ASSERT_SPINLOCK_UNLOCKED(_x) do { if (spin_is_locked(_x)) \
	ERR("@pre clause failed, _x must be unlocked\n"); } while (0) */

/* debug no defined */

#else
#define MGC_GetDebugLevel()	0
#define MGC_SetDebugLevel(n)	do {} while(0)
#define MGC_EnableDebug()
#define MGC_DisableDebug()

#define PARANOID( x )		do {}  while (0)
#define DBG(fmt,args...)	do {}  while (0)
#define DEBUG_CODE(x, y)	do {}  while (0)
#define TRACE(n)		do {}  while (0)

#define ASSERT_SPINLOCK_LOCKED(_x)
#define ASSERT_SPINLOCK_UNLOCKED(_x)

#endif

/*----------------------- DEBUG function/macros -----------------------------*/
struct usb_ep;
struct list_head;
struct usb_request;
struct usb_ctrlrequest;

extern int MGC_DebugLevel;
extern int MGC_DebugDisable;

static inline int _dbg_level(int l)
{
	return !MGC_DebugDisable && ((l >= -1 && MGC_DebugLevel >= l)
					|| MGC_DebugLevel == l);
}

extern void dump_urb(void *urb);
extern char *decode_csr0(u16 csr0);
extern char *decode_devctl(u16 devclt);
extern char *decode_ep0stage(u8 stage);
extern void MGC_HdrcDumpRegs(u8 * pBase, int multipoint, u8 bEnd);

#endif				//  __MUSB_LINUX_DEBUG_H__
