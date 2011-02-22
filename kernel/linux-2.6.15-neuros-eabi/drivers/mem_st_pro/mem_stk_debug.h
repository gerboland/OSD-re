/*
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

//#define DMA_TRANSFER //Do not enable--Not working
#define DEBUG_MMC 0 
#define DEBUG_MMC_RW 0
#define DEBUG_MMC_COMMAND 0 

	
#if DEBUG_MMC
        #define ms_debug_msg(fmt, arg...) printk(KERN_DEBUG "%s:%d> " \
        fmt, __FUNCTION__, __LINE__ , ## arg)
#else
        #define ms_debug_msg(fmt, arg...) do { /* NO OP */ } while(0)
#endif

#if DEBUG_MMC_RW
        #define ms_debug_msg_rw(fmt, arg...) printk(KERN_DEBUG "%s:%d> " \
        fmt, __FUNCTION__, __LINE__ , ## arg)
#else
        #define ms_debug_msg_rw(fmt, arg...) do { /* NO OP */ } while(0)
#endif

#if DEBUG_MMC_COMMAND
        #define ms_debug_msg_command(fmt, arg...) printk(\
        fmt,## arg)
#else
        #define ms_debug_msg_command(fmt, arg...) do { /* NO OP */ } while(0)
#endif

