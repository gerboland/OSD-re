#ifndef __ASM_ARCH_GIO_H
#define __ASM_ARCH_GIO_H

#include <asm/arch/gios.h>

#ifdef __KERNEL__

typedef enum {
	GIO_FALLING_EDGE,
	GIO_RISING_EDGE,
	GIO_ANY_EDGE
} gio_edge_t;

typedef enum {
	bit_low,
	bit_hi
} gio_bit_t;

static inline int gio_irq_num(unsigned char gio)
{
	if (gio > 15) return -1;

	return (IRQ_GIO0 + gio);
}

extern int request_gio(unsigned char gio);
extern void unrequest_gio(unsigned char gio);

extern unsigned char gio_get_bitset(unsigned char gio);
extern unsigned char gio_get_bitclr(unsigned char gio);

extern void gio_set_dir(unsigned char gio, gio_bit_t dir);
extern void gio_set_fsel(unsigned char gio, unsigned char value);
extern void gio_set_inv(unsigned char gio, gio_bit_t inv);
extern void gio_set_bitset(unsigned char gio);
extern void gio_set_bitclr(unsigned char gio);
extern void gio_set_chat(unsigned char gio, gio_bit_t inv);

extern void gio_enable_irq(unsigned char gio, gio_edge_t edge);
extern void gio_disable_irq(unsigned char gio);

#endif //__KERNEL__

#endif
