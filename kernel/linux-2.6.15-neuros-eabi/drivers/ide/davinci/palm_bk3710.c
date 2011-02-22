/*
 * linux/drivers/ide/davinci/palm_bk3710.c
 *
 * TI DaVinci Palm Chip 3710 IDE driver file
 *
 * Copyright (C) 2004 Texas Instruments.
 *
 * ----------------------------------------------------------------------------
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 * ----------------------------------------------------------------------------
 Modifications:
 ver. 1.0: Oct 2005, Swaminathan S
 -
 *
 */

#include <linux/config.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/ioport.h>
#include <linux/hdreg.h>
#include <linux/ide.h>
#include <linux/delay.h>
#include <linux/init.h>

#include <asm/io.h>
#include <asm/arch/registers.h>
#include <asm/arch/irqs.h>
#include <asm/arch/pinmux.h>
#include <asm/arch/i2c-expander.h>
#include <asm/hardware/clock.h>
#include <asm/mach-types.h>

#include "palm_bk3710.h"
#include "../ide-timing.h"


static ide_hwif_t             *palm_bk3710_hwif     = NULL;
struct ide_pci_device_s        palm_bk3710_dummydata;
palm_bk3710_ideregs           *palm_bk3710_base = NULL;
/*
 *
 *  Standard (generic) timings for Taskfile modes, from ATA2 specification.
 *        Some drives may specify a mode, while also specifying a different
 *        value for cycle_time (from drive identification data).
 */
const palm_bk3710_piotiming palm_bk3710_tasktimings[6] = {
	{290, 600},	/* PIO Mode 0 */
	{290, 383},	/* PIO Mode 1 */
	{290, 240},	/* PIO Mode 2 */
	{290, 240},	/* PIO Mode 2 */
	{290, 240},	/* PIO Mode 2 */
//	{80, 180},	    /* PIO Mode 3 with IORDY */
//	{70, 120} 	    /* PIO Mode 4 with IORDY */
};

/*
 *
 *  Standard (generic) timings for PIO modes, from ATA2 specification.
 *        Some drives may specify a mode, while also specifying a different
 *        value for cycle_time (from drive identification data).
 */
const palm_bk3710_piotiming palm_bk3710_piotimings[6] = {
	{165, 600},	/* PIO Mode 0 */
	{125, 383},	/* PIO Mode 1 */
	{100, 240},	/* PIO Mode 2 */
	{80, 180},	    /* PIO Mode 3 with IORDY */
	{70, 120}      /* PIO Mode 4 with IORDY */
};

/*
 *
 *  Standard (generic) timings for DMA modes, from ATA2 specification.
 *        Some drives may specify a mode, while also specifying a different
 *        value for cycle_time (from drive identification data).
 */
const palm_bk3710_dmatiming palm_bk3710_dmatimings[3] = {
	{215, 215, 480},	/* DMA Mode 0 */
	{80,  50,  150},  	/* DMA Mode 1 */
	{70,  25,  120}    	/* DMA Mode 2 */
};

/*
 *
 *  Standard (generic) timings for UDMA modes, from ATA2 specification.
 *        Some drives may specify a mode, while also specifying a different
 *        value for cycle_time (from drive identification data).
 */
  const palm_bk3710_udmatiming palm_bk3710_udmatimings[7] = {
        {45, 160, 114},         /* UDMA Mode 0 */
        {45, 125, 75},          /* UDMA Mode 1 */
        {45, 100, 55},          /* UDMA Mode 2 */
        {20, 100, 39},          /* UDMA Mode 3 */
        {20, 85,  25},          /* UDMA Mode 4 */
        {20, 85,  17}           /* UDMA Mode 5 */
  };

struct clk *ideclkp = NULL;


#undef PALM_BK3710_DEBUG 

#if defined( PALM_BK3710_DEBUG )
#define DBG( fmt, arg... ) printk( fmt, ## arg )
#else
#define DBG( fmt, arg... ) 
#endif

int palm_bk3710_chipinit (void);
int palm_bk3710_setdmamode (palm_bk3710_ideregs *, unsigned int, unsigned int, unsigned int);
int palm_bk3710_setpiomode (palm_bk3710_ideregs *, unsigned int, unsigned int, unsigned int);

static void palm_bk3710_tune_drive (ide_drive_t*, u8);

/* Ingenient - DaVinci pinmuxing */
#if defined( CONFIG_NET_ETHERNET ) && !defined( CONFIG_TI_DAVINCI_EMAC )
#define DAVINCI_NEED_IDE_PINMUX
#endif

#if defined( DAVINCI_NEED_IDE_PINMUX ) 

static spinlock_t palm_bk3710_spinlock = SPIN_LOCK_UNLOCKED;

#define PALM_BK3710_DECLARE( ) \
    unsigned long pinmux

#define PALM_BK3710_LOCK( )   \
    spin_lock( &palm_bk3710_spinlock ); \
    pinmux = pinmux_ata_save( ); \
    udelay( 1 );

#define PALM_BK3710_UNLOCK(  ) \
    pinmux_restore( pinmux ) ; \
    spin_unlock( &palm_bk3710_spinlock );

#else

#define PALM_BK3710_DECLARE( ) 
#define PALM_BK3710_LOCK( ) 
#define PALM_BK3710_UNLOCK( ) 

#endif

#ifndef CONFIG_DAVINCI_BLK_DEV_CF
#ifdef  CONFIG_BLK_DEV_IDEDMA
/**
 *	palm_bk3710_setudmamode		: Set the device UDMA mode on Palm Chip 3710
 *
 *  Handle [IN]                  : IDE Controller info
 *	Dev [IN]                     : drive to tune
 *	level [IN]                   : desired level
 *  int                         : level in UDMA Mode
 ******************************************************************************/
int palm_bk3710_setudmamode (palm_bk3710_ideregs *handle, unsigned int dev, unsigned int level)
{
	char            is_slave	= (dev == 1)? 1:0;
	unsigned char            ide_tenv, ide_trp, ide_t0;


	/* DMA Data Setup */
	ide_t0   = ((palm_bk3710_udmatimings [level] . cycletime +(IDE_PALM_CLK-1))/ IDE_PALM_CLK) - 1;
	ide_tenv = ((palm_bk3710_udmatimings [level] . envtime +(IDE_PALM_CLK-1)) / IDE_PALM_CLK) - 1;
	ide_trp  = ((palm_bk3710_udmatimings [level] . rptime +(IDE_PALM_CLK-1))/ IDE_PALM_CLK) - 1;

        DBG( "%s: level=%d ide_t0=0x%x ide_tenv=0x%x ide_trp=0x%x\n",
             __FUNCTION__, level, ide_t0, ide_tenv, ide_trp );

	if (!is_slave) {
		/* setup master device parameters */
		/* udmatim Register */
		palm_bk3710_base->config. udmatim &= 0xFFF0;
		palm_bk3710_base->config. udmatim |= level;
		/* udmastb Ultra DMA Access Strobe Width */
		palm_bk3710_base->config. udmastb =
                    (palm_bk3710_base->config. udmastb & ~0xff) | ide_t0;
		/* udmatrp Ultra DMA Ready to Pause Time */
		palm_bk3710_base->config. udmatrp =
                    (palm_bk3710_base->config. udmatrp & ~0xff) | ide_trp;
		/* udmaenv Ultra DMA envelop Time */
		palm_bk3710_base->config. udmaenv =
                    (palm_bk3710_base->config. udmaenv & ~0xff) | ide_tenv;
    		/* Enable UDMA for Device 0 */
		palm_bk3710_base->config. udmactl |= 1;
	} else {
	    	/* setup slave device parameters */
		/* udmatim Register */
		palm_bk3710_base->config. udmatim &= 0xFF0F;
		palm_bk3710_base->config. udmatim |= (((unsigned int)level) << 4);
		/* udmastb Ultra DMA Access Strobe Width */
		palm_bk3710_base->config. udmastb =
                    (palm_bk3710_base->config. udmastb & ~0xff00) | (((unsigned int)ide_t0) << 8);
		/* udmatrp Ultra DMA Ready to Pause Time */
		palm_bk3710_base->config. udmatrp =
                    (palm_bk3710_base->config. udmatrp & ~0xff00) | (((unsigned int)ide_trp) << 8);
		/* udmaenv Ultra DMA envelop Time */
		palm_bk3710_base->config. udmaenv =
                    (palm_bk3710_base->config. udmaenv & ~0xff00) | (((unsigned int)ide_tenv) << 8);
	    	/* Enable UDMA for Device 0 */
		palm_bk3710_base->config. udmactl |= (1 << 1);
	}

        DBG( "%s: udmatim=0x%x udmastb=0x%x udmatrp=0x%x udmaenv=0x%x udmactl=0x%x\n",
             __FUNCTION__, 
             palm_bk3710_base->config.udmatim,
             palm_bk3710_base->config.udmastb,
             palm_bk3710_base->config.udmatrp,
             palm_bk3710_base->config.udmaenv,
             palm_bk3710_base->config.udmactl );

	return level;
}

/**
 *	palm_bk3710_setdmamode		: Set the device DMA mode on Palm Chip 3710.
 *
 *  	Handle [IN]                  : IDE Controller info
 *	Dev [IN]                     : drive to tune
 *	level [IN]                   : desired level
 *  	int                         : level in DMA Mode
 ******************************************************************************/
int palm_bk3710_setdmamode(palm_bk3710_ideregs * handle, unsigned int dev,
			   unsigned int cycletime, unsigned int mode)
{
	char            is_slave	= (dev == 1)? 1:0;
	char            ide_td, ide_tkw, ide_t0;

	if (cycletime < palm_bk3710_dmatimings[mode].cycletime) {
	    	cycletime = palm_bk3710_dmatimings [mode] . cycletime;
	}

	/* DMA Data Setup */
	ide_t0 = cycletime / IDE_PALM_CLK;
	ide_td = palm_bk3710_dmatimings [mode] . activetime / IDE_PALM_CLK;
	ide_tkw = ide_t0 > ide_td ? ide_t0 - ide_td - 1 : 0;
	ide_td -= 1;

	if (!is_slave) {
 		/* setup master device parameters */
  		palm_bk3710_base->config. dmastb =
                    (palm_bk3710_base->config. dmastb & ~0xff) | ide_td;
		palm_bk3710_base->config. dmarcvr =
                    (palm_bk3710_base->config. dmarcvr & ~0xff) | ide_tkw;
		palm_bk3710_base -> dmaengine . bmisp |= 0x20;
		palm_bk3710_base->config. udmactl &= 0xFF02;
	} else {
    		/* setup slave device parameters */
  		palm_bk3710_base->config. dmastb =
                    (palm_bk3710_base->config. dmastb&~0xff00) | (((unsigned int)ide_td) << 8);
		palm_bk3710_base->config. dmarcvr =
                    (palm_bk3710_base->config. dmarcvr&~0xff00) | (((unsigned int)ide_tkw) << 8);
    		palm_bk3710_base -> dmaengine . bmisp |= 0x40;
    		/* Disable UDMA for Device 1 */
		palm_bk3710_base->config. udmactl &= 0xFF01;
	}

	return mode;
}
#endif
#endif

/**
 *	palm_bk3710_setpiomode		: Set the device PIO mode on Palm Chip 3710.
 *
 *  	Handle [IN]                  : IDE Controller info
 *	Dev [IN]                     : drive to tune
 *	level [IN]                   : desired level
 *  	int                         : level in PIO mode
 ******************************************************************************/
int palm_bk3710_setpiomode (palm_bk3710_ideregs *handle, unsigned int dev, unsigned int cycletime,
							unsigned int mode)
{
	int             is_slave	= (dev == 1)? 1:0;
	unsigned char            ide_t2, ide_t2i, ide_t0;

        DBG( "%s: Set PIO Mode %d, cycletime %d\n", 
                __FUNCTION__, mode, cycletime );

	if (cycletime < palm_bk3710_piotimings [mode] . cycletime)
	{
            cycletime = palm_bk3710_piotimings [mode] . cycletime;
            DBG( "%s: Cycletime adjusted to %d\n", __FUNCTION__, cycletime );
	}
	/* PIO Data Setup */
	ide_t0 = (cycletime+IDE_PALM_CLK-1) / IDE_PALM_CLK;
	ide_t2 = (palm_bk3710_piotimings [mode] . activetime +(IDE_PALM_CLK-1))/IDE_PALM_CLK;
	ide_t2i = (ide_t0 > ide_t2 ) ? (ide_t0 - ide_t2) - 1 : 0;
	ide_t2 -= 1;

        DBG( "%s: t0=%u t2=%u t2i=%u\n", __FUNCTION__, 
                (unsigned int)ide_t0,
                (unsigned int)ide_t2, 
                (unsigned int)ide_t2i );

	if (!is_slave) {
		/* setup master device parameters */
	    	palm_bk3710_base->config.datstb = 
                    (palm_bk3710_base->config.datstb & ~0x7f)
                    | ide_t2;
	    	palm_bk3710_base->config.datrcvr =
                    (palm_bk3710_base->config.datrcvr & ~0x7f)
                    | ide_t2i;
	    	/* Disable UDMA for Device 0 */
	} else {
	    	/* setup slave device parameters */
	    	palm_bk3710_base->config.datstb = 
                    (palm_bk3710_base->config.datstb & ~0x7f00)
                    | (((unsigned int)ide_t2) << 8);
	    	palm_bk3710_base->config.datrcvr = 
                    (palm_bk3710_base->config.datrcvr & ~0x7f00)
                    | (((unsigned int)ide_t2i) << 8);
	    	/* Disable UDMA for Device 1 */
	}

	/* TASKFILE Setup */
	ide_t2 = (palm_bk3710_tasktimings [mode] . activetime +IDE_PALM_CLK-1)/IDE_PALM_CLK;
	ide_t2i = (ide_t0 > ide_t2 ) ? (ide_t0 - ide_t2) - 1 : 0;
	ide_t2 -= 1;

	if (!is_slave) {
		/* setup master device parameters */
	    	palm_bk3710_base->config.regstb =
                    (palm_bk3710_base->config.regstb & ~0x7f)
                    | ide_t2;
	    	palm_bk3710_base->config.regrcvr =
                    (palm_bk3710_base->config.regrcvr & ~0x7f)
                    | ide_t2i;
	} else {
	    	/* setup slave device parameters */
	    	palm_bk3710_base->config.regstb = 
                    (palm_bk3710_base->config.regstb & ~0x7f00)
                    | (((unsigned int)ide_t2) << 8);
	    	palm_bk3710_base->config.regrcvr = 
                    (palm_bk3710_base->config.regrcvr & ~0x7f00)
                    | (((unsigned int)ide_t2i) << 8);
	}

        DBG( "%s: datastb=0x%04x datarcvr=0x%04x regstb=0x%04x regrcvr=0x%04x\n",
                __FUNCTION__, 
                palm_bk3710_base->config.datstb,
                palm_bk3710_base->config.datrcvr,
                palm_bk3710_base->config.regstb,
                palm_bk3710_base->config.regrcvr );
                

	return mode;
}

#ifndef CONFIG_DAVINCI_BLK_DEV_CF
#ifdef  CONFIG_BLK_DEV_IDEDMA
/**
 *	palm_bk3710_hostdma	-
 *	@drive: IDE drive to tune
 *	@xferspeed: speed to configure
 *
 *	Set a Palm Chip 3710 interface channel to the desired speeds. This involves
 *	requires the right timing data into the 3710 timing override registers.
 */

static int palm_bk3710_hostdma (ide_drive_t *drive, u8 xferspeed)
{
	ide_hwif_t *hwif	    = HWIF(drive);
	u8          speed	    = (XFER_UDMA_4 < xferspeed)? XFER_UDMA_4 : xferspeed;
	int         is_slave	= (&hwif->drives[1] == drive);
	char        ide_cycle;
  	struct      hd_driveid *id	= drive->id;
	int 	    nspeed = -1;

        DBG( "%s: Set DMA Mode %u\n", __FUNCTION__, xferspeed );

	switch(speed) {
	case XFER_UDMA_4:
		nspeed = 2;
		break;
	case XFER_UDMA_3:
		nspeed = 3;
		break;
	case XFER_UDMA_2:
                nspeed = 4; 
		break;
	case XFER_UDMA_1:
		nspeed = 5;
		break;
	case XFER_UDMA_0:
		nspeed = 6;
		break;
	case XFER_MW_DMA_2:
		nspeed = 8;
		break;
	case XFER_MW_DMA_1:
		nspeed = 9;
		break;
	case XFER_MW_DMA_0:
		nspeed = 10;
		break;
	default:
		return -1;
	}

	if (nspeed != -1) {
      		ide_cycle = (ide_timing [nspeed] . cycle < id -> eide_dma_min) ?
                		id -> eide_dma_min : ide_timing [nspeed] . cycle;
		if ((speed <= XFER_UDMA_4) && (speed >= XFER_UDMA_0)) {
	        	palm_bk3710_setudmamode (NULL, is_slave, 6 - nspeed);
		} else {
			palm_bk3710_setdmamode(NULL, is_slave, ide_cycle,
					       10 - nspeed);
   		}

		return (ide_config_drive_speed(drive, speed));
	} else {
		return 0;
	}
}


/**
 *	palm_bk3710_ratemask	-	determine DMA type for hardware
 *	@drive: IDE drive to configure
 *
 */
static u8 palm_bk3710_ratemask (ide_drive_t *drive)
{
        unsigned long ide_clk_rate = clk_get_rate(ideclkp);
	u8 mode;

        if ( ide_clk_rate >= 66000000 ) {
            mode = 2; /* UDMA 3-4: ATA-5 standard */
        } else {
            mode = 1; /* UDMA 0-2: ATA-4 standard */
        }

	if (!eighty_ninty_three(drive)) {
		mode = min(mode, (u8)1);
        }

        /* UDMA has problems on ITDM644X */
        if ( machine_is_itdm644x_10() ) {
            mode = 0; /* Multi-word DMA */
        }

	return mode;
}

/**
 *	palm_bk3710_drivedma	-	configure drive for DMA
 *	@drive: IDE drive to configure
 *
 *	Set up a Palm Chip 3710 interface channel for the best available speed.
 *	We prefer UDMA if it is available and then MWDMA. If DMA is
 *	not available we switch to PIO and return 0.
 */

static inline int palm_bk3710_drivedma(ide_drive_t * pDrive)
{
	u8 speed = ide_dma_speed(pDrive, palm_bk3710_ratemask(pDrive));
	
	/* If no DMA/single word DMA was available or the chipset has DMA bugs
	   then disable DMA and use PIO */
	if (!speed) {
		palm_bk3710_tune_drive(pDrive, 255);
	} else {
		palm_bk3710_hostdma(pDrive, speed);
		return ide_dma_enable(pDrive);
	}
}


/**
 *	palm_bk3710_checkdma	-	set up an IDE device
 *	@drive: IDE drive to configure
 *
 *	Set up the Palm Chip 3710 interface for the best available speed on this
 *	interface, preferring DMA to PIO.
 */

static int palm_bk3710_checkdma (ide_drive_t *drive)
{
	ide_hwif_t *hwif	= HWIF(drive);
	struct hd_driveid *id	= drive->id;

	drive->init_speed = 0;

	if ((id->capability & 1) && drive->autodma) {
		if (id->field_valid & 4) {
			if (id->dma_ultra & hwif->ultra_mask) {
				/* Force if Capable UltraDMA */
				if ((id->field_valid & 2) &&
	    				(!palm_bk3710_drivedma(drive)))
					goto try_dma_modes;
			}
		} else if (id->field_valid & 2) {
try_dma_modes:
			if (id->dma_mword & hwif->mwdma_mask) {
				/* Force if Capable regular DMA modes */
				if (!palm_bk3710_drivedma(drive))
					goto no_dma_set;
			}
		} else {
				goto fast_ata_pio;
		}
		return hwif->ide_dma_on(drive);
	} else if ((id->capability & 8) || (id->field_valid & 2)) {
fast_ata_pio:
no_dma_set:
		hwif->tuneproc(drive, 255);
		return hwif->ide_dma_off_quietly(drive);
	}

	return 0;
}
#endif
#endif

/**
 *	palm_bk3710_tune_drive		-	tune a drive attached to a Palm Chip 3710
 *	@drive: drive to tune
 *	@pio: desired PIO mode
 *
 *	Set the interface and device PIO mode
 *
 */
static void palm_bk3710_tune_drive (ide_drive_t *drive, u8 pio)
{
	ide_hwif_t     *hwif	= HWIF(drive);
	ide_pio_data_t  piodata;
	int             is_slave = (&hwif->drives[1] == drive);

	/* Get the best PIO Mode supported by the drive
	 * Obtain the drive PIO data for tuning the Palm Chip registers
	   */
	ide_get_best_pio_mode (drive, pio, 5, &piodata);
        DBG( "%s: best_pio_mode=%u use_iordy=%u overridden=%u blk=%u cycle=%u\n",
                __FUNCTION__, 
                piodata.pio_mode,
                piodata.use_iordy,
                piodata.overridden,
                piodata.blacklisted,
                piodata.cycle_time );

	  /* Check for IORDY here */
	if (piodata . cycle_time < ide_pio_timings [piodata . pio_mode]
		. cycle_time)
	{
		piodata . cycle_time = ide_pio_timings [piodata . pio_mode]
	        			. cycle_time;
	}
	palm_bk3710_setpiomode (NULL, is_slave, piodata . cycle_time,
						piodata . pio_mode);
}

#if defined( DAVINCI_NEED_IDE_PINMUX ) 

/*
 *	Conventional PIO operations for ATA devices
 */

static u8 palm_bk3710_inb (unsigned long port)
{
    u8 retval;
    PALM_BK3710_DECLARE( );

    PALM_BK3710_LOCK( );
    retval = inb(port);
    PALM_BK3710_UNLOCK( );

    return retval;
}

static u16 palm_bk3710_inw (unsigned long port)
{
    u16 retval;
    PALM_BK3710_DECLARE( );

    PALM_BK3710_LOCK();
    retval = inw(port);
    PALM_BK3710_UNLOCK();

    return retval;
}

static void palm_bk3710_insw (unsigned long port, void *addr, u32 count)
{
    PALM_BK3710_DECLARE( );

    PALM_BK3710_LOCK();
    insw(port, addr, count);
    PALM_BK3710_UNLOCK();
}

static u32 palm_bk3710_inl (unsigned long port)
{
    u32 retval;
    PALM_BK3710_DECLARE( );

    PALM_BK3710_LOCK();
    retval = (u32) inl(port);
    PALM_BK3710_UNLOCK();

    return retval;
}

static void palm_bk3710_insl (unsigned long port, void *addr, u32 count)
{
    PALM_BK3710_DECLARE( );

    PALM_BK3710_LOCK();
    insl(port, addr, count);
    PALM_BK3710_UNLOCK();
}

static void palm_bk3710_outb (u8 val, unsigned long port)
{
    PALM_BK3710_DECLARE( );

    PALM_BK3710_LOCK();
    outb(val, port);
    PALM_BK3710_UNLOCK();
}

static void palm_bk3710_outbsync (ide_drive_t *drive, u8 addr, unsigned long port)
{
    PALM_BK3710_DECLARE( );

    PALM_BK3710_LOCK();
	outb(addr, port);
    PALM_BK3710_UNLOCK();
}

static void palm_bk3710_outw (u16 val, unsigned long port)
{
    PALM_BK3710_DECLARE( );

    PALM_BK3710_LOCK();
    outw(val, port);
    PALM_BK3710_UNLOCK();
}

static void palm_bk3710_outsw (unsigned long port, void *addr, u32 count)
{
    PALM_BK3710_DECLARE( );

    PALM_BK3710_LOCK();
    outsw(port, addr, count);
    PALM_BK3710_UNLOCK();
}

static void palm_bk3710_outl (u32 val, unsigned long port)
{
    PALM_BK3710_DECLARE( );

    PALM_BK3710_LOCK();
    outl(val, port);
    PALM_BK3710_UNLOCK();
}

static void palm_bk3710_outsl (unsigned long port, void *addr, u32 count)
{
    PALM_BK3710_DECLARE( );

    PALM_BK3710_LOCK();
    outsl(port, addr, count);
    PALM_BK3710_UNLOCK();
}

#endif /* DAVINCI_NEED_IDE_PINMUX */

/**
 *	palm_bk3710_init		-	Init Palm Chip 3710
 *
 *	Initialize the Palm Chip 3710 IDE controller to default conditions.
 *
 */
int palm_bk3710_init(void)
{
	int ret = 0;
	hw_regs_t ide_ctlr_info;
	int index = 0;
	int pribase = IO_ADDRESS(IDE_PALM_REG_MMAP_BASE) +
	    IDE_PALM_ATA_PRI_REG_OFFSET;
	struct clk *clkp;

        printk( "Palm Chip BK3710 IDE Initializing: base 0x%08x\n",
                (unsigned int) pribase );

	clkp = clk_get (NULL, "IDECLK");
	if (clkp != 0)
	{
		ideclkp = clkp;
		clk_use (ideclkp);
		clk_enable (ideclkp);

		/* ATA_SEL is 1 -> Disable 0 -> Enable
		 * CF_SEL  is 1 -> Disable 0 -> Enable
		 * 
		 * Ensure both are not Enabled.
		 */
#ifdef CONFIG_DAVINCI_BLK_DEV_CF
		davinci_i2c_expander_op (0x3A, ATA_SEL, 1);
		davinci_i2c_expander_op (0x3A, CF_RESET, 1);
		davinci_i2c_expander_op (0x3A, CF_SEL, 0);
#else
		davinci_i2c_expander_op (0x3A, CF_SEL, 1);
		davinci_i2c_expander_op (0x3A, ATA_SEL, 0);
#endif
		/* Register the IDE interface with Linux ATA Interface */
		memset(&ide_ctlr_info, 0, sizeof(ide_ctlr_info));


		palm_bk3710_base =
		    (palm_bk3710_ideregs *) IO_ADDRESS(IDE_PALM_REG_MMAP_BASE);
		/* Configure the Palm Chip controller */
		palm_bk3710_chipinit();

		for (index = 0; index < IDE_NR_PORTS - 2; index++) {
			ide_ctlr_info.io_ports[index] = pribase + index;
		}
		ide_ctlr_info.io_ports[IDE_CONTROL_OFFSET] =
		    IO_ADDRESS(IDE_PALM_REG_MMAP_BASE)
		    + IDE_PALM_ATA_PRI_CTL_OFFSET;
		ide_ctlr_info.irq = IRQ_IDE;
		ide_ctlr_info.chipset = ide_palm3710;
		ide_ctlr_info.ack_intr = NULL;
		if (ide_register_hw(&ide_ctlr_info, &palm_bk3710_hwif) < 0) {
			printk("Palm Chip BK3710 IDE Register Fail\n");
			return -1;
		}

#if defined( DAVINCI_NEED_IDE_PINMUX ) 
                palm_bk3710_hwif->OUTB	    = palm_bk3710_outb;
                palm_bk3710_hwif->OUTBSYNC  = palm_bk3710_outbsync;
                palm_bk3710_hwif->OUTW	    = palm_bk3710_outw;
                palm_bk3710_hwif->OUTL	    = palm_bk3710_outl;
                palm_bk3710_hwif->OUTSW	    = palm_bk3710_outsw;
                palm_bk3710_hwif->OUTSL	    = palm_bk3710_outsl;
                palm_bk3710_hwif->INB	    = palm_bk3710_inb;
                palm_bk3710_hwif->INW       = palm_bk3710_inw;
                palm_bk3710_hwif->INL	    = palm_bk3710_inl;
                palm_bk3710_hwif->INSW	    = palm_bk3710_insw;
                palm_bk3710_hwif->INSL	    = palm_bk3710_insl;
#endif                
		palm_bk3710_hwif->tuneproc = &palm_bk3710_tune_drive;
		palm_bk3710_hwif->noprobe = 0;
#ifndef CONFIG_DAVINCI_BLK_DEV_CF
#ifdef  CONFIG_BLK_DEV_IDEDMA
		/* Just put this for using the ide-dma.c init code */
		palm_bk3710_dummydata.extra = 0;
		palm_bk3710_hwif->cds = &palm_bk3710_dummydata;
	
		/* Setup up the memory map base for this instance of hwif */
		palm_bk3710_hwif->mmio = 0;
		palm_bk3710_hwif->ide_dma_check = palm_bk3710_checkdma;
		palm_bk3710_hwif->ultra_mask = 0x1f; // Ultra DMA Mode 4 Max
                                                     // (input clk 99MHz)
		palm_bk3710_hwif->mwdma_mask = 0x7;
		palm_bk3710_hwif->swdma_mask = 0;
		palm_bk3710_hwif->dma_command =
		    IO_ADDRESS(IDE_PALM_REG_MMAP_BASE);
		palm_bk3710_hwif->dma_status =
		    IO_ADDRESS(IDE_PALM_REG_MMAP_BASE) + 2;
		palm_bk3710_hwif->dma_prdtable =
		    IO_ADDRESS(IDE_PALM_REG_MMAP_BASE) + 4;
		palm_bk3710_hwif->drives[0].autodma = 1;
		palm_bk3710_hwif->drives[1].autodma = 1;
		ide_setup_dma(palm_bk3710_hwif,
			      IO_ADDRESS(IDE_PALM_REG_MMAP_BASE), 8);
#ifdef MODULE
		palm_bk3710_checkdma (&palm_bk3710_hwif->drives[0]);
		palm_bk3710_checkdma (&palm_bk3710_hwif->drives[1]);
#endif
#endif
#endif
		ret = 0;
	} else {
		printk ("IDE DEV FAIKL\n");
		ret = -ENODEV;
	}

	return ret;
}

/*
 *
 * palm_bk3710_chipinit ()  : Configures the Palm Chip Controller in the
 *                            desired default operating mode
 *
 ******************************************************************************/
int palm_bk3710_chipinit (void)
{
	int atacount = 0;

	/* enable the reset_en of ATA controller so that when ata signals are brought
	 * out , by writing into device config. at that time por_n signal should not be
	 * 'Z' and have a stable value.
	 */
 	palm_bk3710_base->config.miscctl = 0x0300;

	/* wait for some time and deassert the reset of ATA Device. */
	for(atacount = 0; atacount < 5; atacount++);

	/* Deassert the Reset */
	palm_bk3710_base->config.miscctl = 0x0200;

	/* Program the IDETIMP Register Value based on the following assumptions
	 *
	 * (ATA_IDETIMP_IDEEN      ,ENABLE )  |
	 * (ATA_IDETIMP_SLVTIMEN   , DISABLE) |
	 * (ATA_IDETIMP_RDYSMPL    , 70NS) |
	 * (ATA_IDETIMP_RDYRCVRY   , 50NS) |
	 * (ATA_IDETIMP_DMAFTIM1   , PIOCOMP) |
	 * (ATA_IDETIMP_PREPOST1   , DISABLE) |
	 * (ATA_IDETIMP_RDYSEN1    , DISABLE) |
	 * (ATA_IDETIMP_PIOFTIM1   , DISABLE) |
	 * (ATA_IDETIMP_DMAFTIM0   , PIOCOMP) |
	 * (ATA_IDETIMP_PREPOST0   , DISABLE) |
	 * (ATA_IDETIMP_RDYSEN0    , DISABLE) |
	 * (ATA_IDETIMP_PIOFTIM0   , DISABLE)
	 */

	palm_bk3710_base->config.idetimp  = 0xb388;

	/* Configure  SIDETIM  Register
	 * (ATA_SIDETIM_RDYSMPS1     ,120NS ) |
	 * (ATA_SIDETIM_RDYRCYS1     ,120NS )
	 */
	palm_bk3710_base->config.sidetim = 0;

	/* UDMACTL Ultra-ATA DMA Control
	 * (ATA_UDMACTL_UDMAP1      , 0 ) |
	 * (ATA_UDMACTL_UDMAP0      , 0 )
	 *
	 */
	palm_bk3710_base->config.udmactl  = 0;

	/* MISCCTL Miscellaneous Conrol Register
	 * (ATA_MISCCTL_RSTMODEP    , 1) |
	 * (ATA_MISCCTL_RESETP        , 0) |
	 * (ATA_MISCCTL_TIMORIDE      , 1)
	 */
        // XXX: extended data hold on WE
	palm_bk3710_base->config.miscctl = 0x201;

	/* IORDYTMP IORDY Timer for Primary Register
	 * (ATA_IORDYTMP_IORDYTMP     , 0xffff  )
	 */

	palm_bk3710_base->config.iordytmp   =   0xffff;

	/*Configure BMISP Register
	 * (ATA_BMISP_DMAEN1        , DISABLE  ) |
	 * (ATA_BMISP_DMAEN0	 , DISABLE  ) |
	 * (ATA_BMISP_IORDYINT	 , CLEAR) |
	 * (ATA_BMISP_INTRSTAT	 , CLEAR) |
	 * (ATA_BMISP_DMAERROR	 , CLEAR)
	 */

	palm_bk3710_base->dmaengine.bmisp = 0;

	palm_bk3710_setpiomode (NULL, 0, 0, 0);
	palm_bk3710_setpiomode (NULL, 1, 0, 0);

		return 1;
}

#ifdef MODULE
static void __exit  palm_bk3710_exit (void)
{
	palm_bk3710_hwif = NULL;
	ide_unregister (0);
	clk_disable (ideclkp);
	clk_unuse (ideclkp);
}

module_init(palm_bk3710_init);
module_exit(palm_bk3710_exit);
MODULE_LICENSE("GPL");
#endif
