#
# for USB OTG silicon based on Mentor Graphics INVENTRA designs
#

musb_hdrc-objs := plat_uds.o

obj-$(CONFIG_USB_MUSB_HDRC)	+= musb_hdrc.o

# FIXME remove all these extra "-DMUSB_* things, stick to CONFIG_*

ifeq ($(CONFIG_USB_MUSB_OTG),y)
	musb_hdrc-objs	+= otg.o
endif

ifeq ($(CONFIG_USB_GADGET_MUSB_HDRC),y)
	musb_hdrc-objs		+= musb_gadgetcommon.o g_ep0.o musb_gadget.o
endif

ifeq ($(CONFIG_USB_MUSB_HDRC_HCD),y)
	musb_hdrc-objs		+= virthub.o musb_host.o
endif

# Kconfig is supposed to guarantee us only one of the
# possible I/O schemes will be enabled at a time ...
# thoug of course PIO is always there, at least for ep0
ifneq ($(CONFIG_USB_INVENTRA_FIFO),y)

  ifeq ($(CONFIG_USB_INVENTRA_DMA),y)
    musb_hdrc-objs		+= musbhsdma.o

  else
    ifeq ($(CONFIG_USB_TI_CPPI_DMA),y)
      musb_hdrc-objs		+= cppi_dma.o

    endif
  endif
endif


################################################################################

ifeq ($(CONFIG_PROC_FS),y)
	musb_hdrc-objs		+= musb_procfs.o musb_ioctl.o
endif

ifeq ($(CONFIG_USB_INVENTRA_MUSB_HAS_AHB_ID),y)
	EXTRA_CFLAGS += -DMUSB_AHB_ID
endif

# Debugging

MUSB_DEBUG:=$(CONFIG_USB_INVENTRA_HCD_LOGGING)

ifeq ("$(strip $(MUSB_DEBUG))","")
    ifdef CONFIG_USB_DEBUG
	MUSB_DEBUG:=1
    else
	MUSB_DEBUG:=0
    endif
endif

ifneq ($(MUSB_DEBUG),0)
    musb_hdrc-objs += debug.o
    EXTRA_CFLAGS += -DDEBUG
endif

EXTRA_CFLAGS += -DMUSB_DEBUG=$(MUSB_DEBUG)
