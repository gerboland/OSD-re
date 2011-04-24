#  Copyright(C) 2006-2007 Neuros Technology International LLC. 
#               <www.neurostechnology.com>
#
#
#  This program is free software; you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation, version 2 of the License.
#
#  This program is distributed in the hope that, in addition to its 
#  original purpose to support Neuros hardware, it will be useful 
#  otherwise, but WITHOUT ANY WARRANTY; without even the implied 
#  warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  
#  See the GNU General Public License for more details.
#
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software
#  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
#
#***************************************************************************
#
# KBuild Makefile for rt73 Wifi driver
#
# REVISION:
# 1) Initial creation. ------------------------------- 2007-04-20 nerochiaro


MODULE_NAME := rt73
obj-$(CONFIG_USB_RT73) += $(MODULE_NAME).o
$(MODULE_NAME)-objs :=  rtmp_main.o mlme.o connect.o rtusb_bulk.o rtusb_io.o sync.o assoc.o auth.o auth_rsp.o rtusb_data.o rtmp_init.o  sanity.o rtmp_wep.o rtmp_info.o rtmp_tkip.o wpa.o md5.o

EXTRA_CFLAGS += -DAGGREGATION_SUPPORT -DWMM_SUPPORT -DRTMP_EMBEDDED
#EXTRA_CFLAGS += -DRALINK_ATE
#EXTRA_CFLAGS += -DSINGLE_ADHOC_LINKUP
#EXTRA_CFLAGS += -DRT2X00DEBUGFS
EXTRA_CFLAGS += -DDBG

# Do not disable this or the driver will revert to vanilla rt73
EXTRA_CFLAGS += -DPLAT_DM320
