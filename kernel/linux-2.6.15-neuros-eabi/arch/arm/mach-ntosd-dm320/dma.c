/*
 *  linux/arch/arm/mach-itdm320-20/dma.c
 *
 *  by Ingenient Technologies
 *
 *  derived from:
 *  arch/arm/mach-shark/dma.c
 *  arch/arm/kernel/dma-ebsa285.c
 *  Copyright (C) 1998 Phil Blundell
 */

#include <linux/config.h>
#include <linux/init.h>

#include <asm/dma.h>
#include <asm/mach/dma.h>

void __init arch_dma_init(dma_t *dma)
{
#ifdef CONFIG_ISA_DMA
	isa_init_dma(dma);
#endif
}
