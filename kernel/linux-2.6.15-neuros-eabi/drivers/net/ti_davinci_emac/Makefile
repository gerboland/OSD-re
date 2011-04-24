# File: drivers/net/ti_cpmac/Makefile
#
# Makefile for the Linux network (CPMAC) device drivers.
#
# DDA
EXTRA_CFLAGS        += -DCPMAC_CACHE_WRITEBACK_MODE
EXTRA_CFLAGS        += -DCPMAC_DDA_DEBUG
EXTRA_CFLAGS        += -DCPMAC_DDA_CACHE_INVALIDATE_FIX

# DDC
EXTRA_CFLAGS        += -DCPMAC_DDC_DEBUG

# Common
EXTRA_CFLAGS        += -DCPMAC_MULTIPACKET_TX_COMPLETE_NOTIFY
EXTRA_CFLAGS        += -DCPMAC_MULTIPACKET_RX_COMPLETE_NOTIFY


obj-$(CONFIG_TI_DAVINCI_EMAC) += davinci_cpmac.o

davinci_cpmac-objs := cpmacNetLx.o ddc_cpmac.o ddc_cpmacTxRx.o  cpmacNetLxTxRx.o cpswhalcommon_miimdio.o

davinci_cpmac.o: $(davinci_cpmac-objs)
	$(LD) -r -o $@ $(davinci_cpmac-objs)

clean:
	rm -f core *.o *.a *.s
