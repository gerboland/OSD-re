#
# Makefile for the kernel mmc device drivers.
#

#
# Core
#
obj-$(CONFIG_MMC)               += mmc_core.o

#
# Media drivers
#
obj-$(CONFIG_MMC_BLOCK)         += mmc_block.o

#
# Host drivers
#
obj-$(CONFIG_MMC_ARMMMCI)       += mmci.o
obj-$(CONFIG_MMC_PXA)           += pxamci.o
obj-$(CONFIG_MMC_WBSD)          += wbsd.o
obj-$(CONFIG_MMC_AU1X)          += au1xmmc.o
obj-$(CONFIG_MMC_OMAP)          += omap.o
obj-$(CONFIG_MMC_OMAP24XX)      += omap24xx_mmc.o
obj-$(CONFIG_MMC_OMAP243X)      += omap2430_hsmmc.o
obj-$(CONFIG_MMC_DAVINCI)       += davinci.o
obj-$(CONFIG_MMC_DM320)         += dm320_mmc.o

mmc_core-y := mmc.o mmc_queue.o mmc_sysfs.o

ifeq ($(CONFIG_MMC_DM320),m)
    obj-$(CONFIG_MMC_DM320)     += mmc.o mmc_queue.o mmc_sysfs.o
endif
