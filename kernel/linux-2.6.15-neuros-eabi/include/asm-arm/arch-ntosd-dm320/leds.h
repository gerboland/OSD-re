#ifndef __ASM_ARCH_LEDS_H
#define __ASM_ARCH_LEDS_H

#include <asm/arch/gio.h>

#define IT_LED_IOC_MAGIC 'i'

#define IT_IOCLED_ON  _IO(IT_LED_IOC_MAGIC, 0)
#define IT_IOCLED_OFF _IO(IT_LED_IOC_MAGIC, 1)

#define IT_LED_IOC_NR 2

#define IT_LED_1 GIO_LED_1
#define IT_LED_2 GIO_LED_2
//#define IT_LED_3 GIO_LED_3
//#define IT_LED_4 GIO_LED_4 

#ifdef __KERNEL__
#ifdef CONFIG_INGENIENT_LED
extern void it_led_on(unsigned char led);
extern void it_led_off(unsigned char led);
#else
#define it_led_on(led) do {} while(0)
#define it_led_off(led) do {} while(0)
#endif
#endif

#endif
