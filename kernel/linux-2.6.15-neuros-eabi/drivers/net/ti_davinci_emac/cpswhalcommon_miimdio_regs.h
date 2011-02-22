/*
 * linux/drivers/net/ti_davinci_emac/cpswhalcommon_miimdio_regs.h
 *
 * MDIO Polling State Machine API. Functions will enable mii-Phy
 * negotiation.
 *
 * Copyright (C) 2005 Texas Instruments.
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
 * Modifications:
 *  HISTORY:
 *  Date      Modifier         Ver    Notes
 *      27Mar02 Michael Hanrahan Original (modified from emacmdio.h)
 *      01Apr02 Michael Hanrahan Modified to include all regs. in spec
 *      03Apr02 Michael Hanrahan Updated to Version 0.6 of spec
 *      05Apr02 Michael Hanrahan Moved Phy Mode values into here
 *      30Apr02 Michael Hanrahan Updated to Version 0.8 of spec
 *      30Apr02 Michael Hanrahan Updated to recommended format
 *      10May02 Michael Hanrahan Updated to Version 0.9 of spec
*****************************************************************************/

#ifndef _CPSWHALCOMMON_MIIMDIO_REGS_H
#define _CPSWHALCOMMON_MIIMDIO_REGS_H

/***************************************************************************
**                                                                         
**         M D I O  M E M O R Y  M A P 
**                                                                         
***************************************************************************/

#define pMDIO_VER(base)                 ((volatile bit32u *)(base+0x00))
#define pMDIO_CONTROL(base)             ((volatile bit32u *)(base+0x04))
#define pMDIO_ALIVE(base)               ((volatile bit32u *)(base+0x08))
#define pMDIO_LINK(base)                ((volatile bit32u *)(base+0x0C))
#define pMDIO_LINKINTRAW(base)          ((volatile bit32u *)(base+0x10))
#define pMDIO_LINKINTMASKED(base)       ((volatile bit32u *)(base+0x14))
#define pMDIO_USERINTRAW(base)          ((volatile bit32u *)(base+0x20))
#define pMDIO_USERINTMASKED(base)       ((volatile bit32u *)(base+0x24))
#define pMDIO_USERINTMASKED_SET(base)   ((volatile bit32u *)(base+0x28))
#define pMDIO_USERINTMASKED_CLR(base)   ((volatile bit32u *)(base+0x2C))
#define pMDIO_USERACCESS(base, channel) ((volatile bit32u *)(base+(0x80+(channel*8))))
#define pMDIO_USERPHYSEL(base, channel) ((volatile bit32u *)(base+(0x84+(channel*8))))

/***************************************************************************
**                                                                         
**         M D I O  R E G I S T E R  A C C E S S  M A C R O S 
**                                                                         
***************************************************************************/

#define MDIO_ALIVE(base)                    (*(pMDIO_ALIVE(base)))
#define MDIO_CONTROL(base)                  (*(pMDIO_CONTROL(base)))
#define         MDIO_CONTROL_IDLE             (1 << 31)
#define         MDIO_CONTROL_ENABLE           (1 << 30)
#define         MDIO_CONTROL_PREAMBLE             (1 << 20)
#define         MDIO_CONTROL_FAULT            (1 << 19)
#define         MDIO_CONTROL_FAULT_DETECT_ENABLE  (1 << 18)
#define         MDIO_CONTROL_INT_TEST_ENABLE      (1 << 17)
#define         MDIO_CONTROL_HIGHEST_USER_CHANNEL (0x1F << 8)
#define         MDIO_CONTROL_CLKDIV           (0xFF)
#define MDIO_LINK(base)                     (*(pMDIO_LINK(base)))
#define MDIO_LINKINTRAW(base)               (*(pMDIO_LINKINTRAW(base)))
#define MDIO_LINKINTMASKED(base)            (*(pMDIO_LINKINTMASKED(base)))
#define MDIO_USERINTRAW(base)               (*(pMDIO_USERINTRAW(base)))
#define MDIO_USERINTMASKED(base)            (*(pMDIO_USERINTMASKED(base)))
#define MDIO_USERINTMASKED_CLR(base)        (*(pMDIO_USERINTMASKED_CLR(base)))
#define MDIO_USERINTMASKED_SET(base)        (*(pMDIO_USERINTMASKED_SET(base)))
#define MDIO_USERINTRAW(base)               (*(pMDIO_USERINTRAW(base)))
#define MDIO_USERACCESS(base, channel)      (*(pMDIO_USERACCESS(base, channel)))
#define         MDIO_USERACCESS_GO     (1 << 31)
#define         MDIO_USERACCESS_WRITE  (1 << 30)
#define         MDIO_USERACCESS_READ   (0 << 30)
#define         MDIO_USERACCESS_ACK    (1 << 29)
#define         MDIO_USERACCESS_REGADR (0x1F << 21)
#define         MDIO_USERACCESS_PHYADR (0x1F << 16)
#define         MDIO_USERACCESS_DATA   (0xFFFF)
#define MDIO_USERPHYSEL(base, channel)      (*(pMDIO_USERPHYSEL(base, channel)))
#define         MDIO_USERPHYSEL_LINKSEL         (1 << 7)
#define         MDIO_USERPHYSEL_LINKINT_ENABLE  (1 << 6)
#define         MDIO_USERPHYSEL_PHYADR_MON      (0x1F)
#define MDIO_VER(base)                      (*(pMDIO_VER(base)))
#define         MDIO_VER_MODID         (0xFFFF << 16)
#define         MDIO_VER_REVMAJ        (0xFF   << 8)
#define         MDIO_VER_REVMIN        (0xFF)

/****************************************************************************/
/*                                                                          */
/*         P H Y   R E G I S T E R  D E F I N I T I O N S                   */
/*                                                                          */
/****************************************************************************/

#define PHY_CONTROL_REG       0
#define MII_PHY_RESET           (1<<15)
#define MII_PHY_LOOP            (1<<14)
#define MII_PHY_100             (1<<13)
#define MII_AUTO_NEGOTIATE_EN   (1<<12)
#define MII_PHY_PDOWN           (1<<11)
#define MII_PHY_ISOLATE         (1<<10)
#define MII_RENEGOTIATE         (1<<9)
#define MII_PHY_FD              (1<<8)

#define PHY_STATUS_REG        1
#define MII_NWAY_COMPLETE       (1<<5)
#define MII_NWAY_CAPABLE        (1<<3)
#define MII_PHY_LINKED          (1<<2)

#define NWAY_ADVERTIZE_REG    4
#define NWAY_REMADVERTISE_REG 5
#define MII_NWAY_FD100          (1<<8)
#define MII_NWAY_HD100          (1<<7)
#define MII_NWAY_FD10           (1<<6)
#define MII_NWAY_HD10           (1<<5)
#define MII_NWAY_SEL            (1<<0)

#endif				/* _INC_MDIO_REG */
