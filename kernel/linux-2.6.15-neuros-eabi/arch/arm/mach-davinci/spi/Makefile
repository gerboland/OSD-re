#
# Makefile for the spi core.
#

spi_davinci_test-objs :=	spi_davinci_client.o spi_davinci_lb_test.o

obj-$(CONFIG_SPI)                   += spi-core.o
obj-$(CONFIG_SPI_CHARDEV)           += spi-dev.o
obj-$(CONFIG_SPI_SENSOR)            += spi-sensor.o
obj-$(CONFIG_SPI_DAVINCI)           += spi_davinci.o
obj-$(CONFIG_SPI_DAVINCI_TEST)      += spi_davinci_test.o

ifeq ($(CONFIG_SPI_DEBUG_CORE),y)
EXTRA_CFLAGS += -DDEBUG
endif
