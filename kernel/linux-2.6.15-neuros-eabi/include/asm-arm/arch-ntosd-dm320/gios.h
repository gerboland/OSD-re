#ifndef __ASM_ARCH_GIOS_H
#define __ASM_ARCH_GIOS_H


#define GIO_LED_1 16
#define GIO_LED_2 17
//#define GIO_LED_3 0
//#define GIO_LED_4 0

#ifdef CONFIG_INGENIENT_NAVIGATOR
#define GIO_NAV_SCANSEL 16
#define GIO_NAV_RIGHT 1
#define GIO_NAV_UP	  2
#define GIO_NAV_LEFT  3
#define GIO_NAV_DOWN  4
#define GIO_NAV_SEL	  5
#endif
//#define GIO_HDD_HOTPLUG 2
#define GIO_ETHER 6
//#define GIO_HDD 7
//#define GIO_USB_ENABLE 0
#define GIO_AIC23_FREQ 12//18
#define GIO_PLL_FS1    12
//#define GIO_PLL_FS2    13
//#define GIO_PLL_SR     14
//#define GIO_AUD_ENA    21 /* Neuros DevB: Power up audio HW */

#define GIO_I2C_SCL 30
#define GIO_I2C_SDA 31
#define GIO_CFC_HOTPLUG 25
#define GIO_UART1_RXD	27
#define GIO_UART1_TXD	28
#define GIO_CFC_RESET   36
#define GIO_CFC_DETECT  9
//#define GIO_FIELD_ID 39

//#define GIO_VIDEO_IN 10

#define GIO_MS_CARDDETECT 5
#define GIO_SD_CARDDETECT 8
#define GIO_SDCARD_WP 35
#define GIO_NAND_CF1   1

#endif
