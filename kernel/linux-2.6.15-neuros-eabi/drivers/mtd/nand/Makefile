#
# linux/drivers/nand/Makefile
#
# $Id: Makefile,v 1.4 2006/06/16 15:35:19 jwhittington Exp $

obj-$(CONFIG_MTD_NAND)			+= nand.o nand_ecc.o
obj-$(CONFIG_MTD_NAND_IDS)		+= nand_ids.o

obj-$(CONFIG_MTD_NAND_SPIA)		+= spia.o
obj-$(CONFIG_MTD_NAND_TOTO)		+= toto.o
obj-$(CONFIG_MTD_NAND_AUTCPU12)		+= autcpu12.o
obj-$(CONFIG_MTD_NAND_EDB7312)		+= edb7312.o
obj-$(CONFIG_MTD_NAND_AU1550)		+= au1550nd.o
obj-$(CONFIG_MTD_NAND_PPCHAMELEONEVB)	+= ppchameleonevb.o
obj-$(CONFIG_MTD_NAND_S3C2410)		+= s3c2410.o
obj-$(CONFIG_MTD_NAND_DISKONCHIP)	+= diskonchip.o
obj-$(CONFIG_MTD_NAND_H1900)		+= h1910.o
obj-$(CONFIG_MTD_NAND_RTC_FROM4)	+= rtc_from4.o
obj-$(CONFIG_MTD_NAND_SHARPSL)		+= sharpsl.o
obj-$(CONFIG_MTD_NAND_NANDSIM)		+= nandsim.o
obj-$(CONFIG_MTD_NAND_OMAP) 		+= omap-nand-flash.o
obj-$(CONFIG_MTD_NAND_OMAP_HW)		+= omap-hw.o
obj-$(CONFIG_MTD_NAND_DAVINCI)          += davinci_nand.o llc_nand.o
obj-$(CONFIG_MTD_NAND_ITDM320)          += dm320-nand-flash.o

nand-objs = nand_base.o nand_bbt.o
