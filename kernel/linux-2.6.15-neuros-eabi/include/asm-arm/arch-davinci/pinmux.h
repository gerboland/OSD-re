#ifndef _ASM_ARCH_PINMUX_H
#define _ASM_ARCH_PINMUX_H

#include <linux/delay.h>
#include <asm/arch/hardware.h>

#define DAVINCI_SYSTEM_PINMUX0 ((volatile unsigned int*)DAVINCI_PERI_ADDR(0x01c40000))
#define DAVINCI_SYSTEM_PINMUX0_ENABLE_ATA 0x00030000 // (ATAEN | HDIREN)

static unsigned long inline pinmux_ata_save( void )
{
    unsigned long retval = *DAVINCI_SYSTEM_PINMUX0;

    (*DAVINCI_SYSTEM_PINMUX0) = retval | DAVINCI_SYSTEM_PINMUX0_ENABLE_ATA;

    return retval;
}

static unsigned long inline pinmux_emif_save( void )
{
    unsigned long retval = *DAVINCI_SYSTEM_PINMUX0;

    (*DAVINCI_SYSTEM_PINMUX0) = retval & ~DAVINCI_SYSTEM_PINMUX0_ENABLE_ATA;

    udelay(1);
    return retval;
}

static void inline pinmux_restore( unsigned long val )
{
    (*DAVINCI_SYSTEM_PINMUX0) = val;
}

static int inline pinmux_is_ata_enabled( unsigned long val )
{
    return ((val)&DAVINCI_SYSTEM_PINMUX0_ENABLE_ATA) ? 1 : 0;
}

static int inline pinmux_is_ata_enabled( unsigned long val )
{
    return ((val)&DAVINCI_SYSTEM_PINMUX0_ENABLE_ATA) ? 1 : 0;
}

#endif
