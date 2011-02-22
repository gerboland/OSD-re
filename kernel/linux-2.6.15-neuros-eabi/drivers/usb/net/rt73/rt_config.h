/***************************************************************************
 * RT2x00 SourceForge Project - http://rt2x00.serialmonkey.com             *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 *                                                                         *
 *   Licensed under the GNU GPL                                            *
 *   Original code supplied under license from RaLink Inc, 2004.           *
 ***************************************************************************/

/***************************************************************************
 *	Module Name:	rt_config.h
 *
 *	Abstract: Central header file to maintain all include files for all
 *		  driver routines.
 *
 *	Revision History:
 *	Who		When		What
 *	--------	----------	-----------------------------
 *	Nemo Tang	02-20-2005	created
 *
 ***************************************************************************/

#ifndef	__RT_CONFIG_H__
#define	__RT_CONFIG_H__

#define PROFILE_PATH                "/etc/Wireless/RT73STA/rt73sta.dat"
#define NIC_DEVICE_NAME             "RT73STA"
#define RT2573_IMAGE_FILE_NAME      "/lib/firmware/rt73/rt73.bin"
#define DRIVER_NAME                 "rt73"
#define DRIVER_VERSION		    "1.0.3.6 CVS"
#define DRIVER_RELDATE              "CVS"

// Query from UI
#define DRV_MAJORVERSION        1
#define DRV_MINORVERSION        0
#define DRV_SUBVERSION          3
#define DRV_TESTVERSION         6
#define DRV_YEAR                2006
#define DRV_MONTH               7
#define DRV_DAY                 4

/* Operational parameters that are set at compile time. */
#if !defined(__OPTIMIZE__)  ||  !defined(__KERNEL__)
#warning  You must compile this file with the correct options!
#warning  See the last lines of the source file.
#error  You must compile this driver with "-O".
#endif

#include <linux/module.h>
#include <linux/version.h>
#include <linux/kernel.h>

#include <linux/init.h>
#include <linux/string.h>
#include <linux/timer.h>
#include <linux/errno.h>
#include <linux/slab.h>
//#include <linux/interrupt.h>
#include <linux/pci.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <linux/wireless.h>
#include <linux/proc_fs.h>
#include <linux/delay.h>
#include <linux/if_arp.h>
#include <linux/ctype.h>
#include <linux/sched.h>
#include <linux/smp_lock.h>
#include <linux/kmod.h>


#include <linux/ioport.h>
//usb header files
#include <linux/usb.h>

#if LINUX_VERSION_CODE >= 0x20407
#include <linux/mii.h>
#endif
#include <asm/processor.h>      /* Processor type for cache alignment. */
#include <asm/bitops.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/uaccess.h>
#include <asm/atomic.h>
#include <asm/byteorder.h>

// load firmware
#define __KERNEL_SYSCALLS__
#include <linux/unistd.h>
#include <asm/uaccess.h>


#ifndef ULONG
#define CHAR            signed char
#define INT             int
#define SHORT           int
#define UINT            u32
#define ULONG           u32
#define USHORT          u16
#define UCHAR           u8

#define uint32			u32
#define uint8			u8


#define BOOLEAN         u8
//#define LARGE_INTEGER s64
#define VOID            void
#define LONG            int
#define LONGLONG        s64
#define ULONGLONG       u64
typedef VOID            *PVOID;
typedef CHAR            *PCHAR;
typedef UCHAR           *PUCHAR;
typedef USHORT          *PUSHORT;
typedef LONG            *PLONG;
typedef ULONG           *PULONG;

typedef union _LARGE_INTEGER {
    struct {
        ULONG LowPart;
        LONG HighPart;
    }vv;
    struct {
        ULONG LowPart;
        LONG HighPart;
    } u;
    s64 QuadPart;
} LARGE_INTEGER;

#endif


#define IN
#define OUT

#define TRUE        1
#define FALSE       0

#define ETH_LENGTH_OF_ADDRESS   6       // = MAC_ADDR_LEN

#define NDIS_STATUS                             INT
#define NDIS_STATUS_SUCCESS                     0x00
#define NDIS_STATUS_FAILURE                     0x01
#define NDIS_STATUS_RESOURCES                   0x03
#define NDIS_STATUS_MEDIA_DISCONNECT            0x04
#define NDIS_STATUS_MEDIA_CONNECT               0x05
#define NDIS_STATUS_RESET                       0x06
#define NDIS_STATUS_RINGFULL                    0x07 /* Thomas add */


// ** Wireless Extensions **
// 1. wireless events support        : v14 or newer
// 2. requirements of wpa-supplicant : v15 or newer
#if WIRELESS_EXT >= 15
#define WPA_SUPPLICANT_SUPPORT  1
#else
#define WPA_SUPPLICANT_SUPPORT  0
#endif


//
//	Hradware related header files
//
#include	"rt73.h"

//
//	Miniport defined header files
//
#include	"rt2x00debug.h"
#include	"rtmp_type.h"
#include	"rtmp_def.h"
#include    "oid.h"
#include	"mlme.h"
#include    "md5.h"
#include    "wpa.h"
#include	"rtmp.h"


// We can't use this flag on DM320, not sure exactly why. kmalloc always fails if we do. --Ugo R.
#ifdef PLAT_DM320
    #define MEM_ALLOC_FLAG      (GFP_ATOMIC)
    #define SKB_ALLOC_FLAG      (GFP_ATOMIC)
#else
    #define MEM_ALLOC_FLAG      (GFP_DMA | GFP_ATOMIC)
    #define SKB_ALLOC_FLAG      (GFP_DMA | GFP_ATOMIC)
#endif

#ifndef KERNEL_VERSION
#define KERNEL_VERSION(a,b,c) ((a)*65536+(b)*256+(c))
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,5,0)
#define rtusb_submit_urb(purb) usb_submit_urb(purb, GFP_KERNEL)
#else
#define rtusb_submit_urb(purb) usb_submit_urb(purb)
#endif


#ifndef USB_ST_NOERROR
#define  USB_ST_NOERROR     0
#endif

// We should not use this static config file right now, as it's better 
// to setup the interface using standard tools --nerochiaro
#undef READ_PROFILE_FROM_FILE      //read RaConfig profile parameters from rt73sta.dat
#define INIT_FROM_EEPROM


#endif	// __RT_CONFIG_H__
