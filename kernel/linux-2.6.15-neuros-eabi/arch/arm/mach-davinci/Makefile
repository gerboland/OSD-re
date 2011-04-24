#
# Makefile for the linux kernel.
#
#


# Object file lists.

obj-y 				:= time.o arch.o irq.o dma.o serial_davinci.o dma_osdep.o clock.o i2c-davinci-client.o

# Ingenient GIO functions
obj-y += gio.o

ifeq ($(CONFIG_LEDS),y)
obj-$(CONFIG_MACH_DAVINCI_EVM)		+= leds-evm.o
endif
obj-$(CONFIG_SPI)		+= spi/
