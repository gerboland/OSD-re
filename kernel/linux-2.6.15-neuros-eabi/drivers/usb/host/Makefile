#
# Makefile for USB Host Controller Drivers
#

obj-$(CONFIG_PCI)		+= pci-quirks.o

#ISOFT_PORT
#obj-$(CONFIG_USB_DM320_HCD)     += dm320/dm320-hcd.o
obj-$(CONFIG_USB_DM320_HCD)     += dm320/
obj-$(CONFIG_USB_EHCI_HCD)	+= ehci-hcd.o
obj-$(CONFIG_USB_ISP116X_HCD)	+= isp116x-hcd.o
obj-$(CONFIG_USB_OHCI_HCD)	+= ohci-hcd.o
obj-$(CONFIG_USB_UHCI_HCD)	+= uhci-hcd.o
obj-$(CONFIG_USB_SL811_HCD)	+= sl811-hcd.o
obj-$(CONFIG_USB_SL811_CS)	+= sl811_cs.o
obj-$(CONFIG_ETRAX_ARCH_V10)	+= hc_crisv10.o
