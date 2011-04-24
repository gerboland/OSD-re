/*
 * linux/include/asm-arm/arch-itdm320-20/system.h
 *
 * Copyright (c) 1996-1999 Russell King.
 */
#ifndef __ASM_ARCH_SYSTEM_H
#define __ASM_ARCH_SYSTEM_H

static inline void arch_idle(void)
{
	cpu_do_idle();
}

static void arch_reset(char mode)
{
        /* Code taken from bootloader/cpu/arm926ejs/start.S: reset_cpu */
	__asm__ __volatile__(					
	"mov     ip, #0\n"
	"mcr     p15, 0, ip, c7, c7, 0           @ invalidate cache\n"
	"mcr     p15, 0, ip, c8, c7, 0           @ flush TLB (v4)\n"
	"mrc     p15, 0, ip, c1, c0, 0           @ get ctrl register\n"
	"bic     ip, ip, #0x000f                 @ ............wcam\n"
	"bic     ip, ip, #0x2100                 @ ..v....s........\n"
	"mcr     p15, 0, ip, c1, c0, 0           @ ctrl register\n"
	"mov     ip, #0xFF000000\n"
	"orr     ip, ip, #0xFF0000               @ ip = 0xFFFF0000 \n"  
	"mov     pc, ip\n"
	:
	:
	: "cc" );
}

#endif
