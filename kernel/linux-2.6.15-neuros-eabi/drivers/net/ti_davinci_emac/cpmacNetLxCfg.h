/*
 * linux/drivers/net/ti_davinci_emac/cpmacNetLxCfg.h
 *
 * Linux DDA Configuration Header file
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
 Modifications:
 ver. 0.1 Anant Gole - Created
      0.2 Sharath Kumar - Incorporated review comments
      0.3 - Update for DaVinci
 */

#ifndef _CPMAC_LX_DDA_CFG_H_
#define _CPMAC_LX_DDA_CFG_H_

#define EGRESS_TRAILOR_LEN                  0

#define CFG_START_LINK_SPEED                (SNWAY_AUTOALL)	/* auto nego */

/* Defaut Configuration values required for passing on to DDC */
#define CPMAC_DEFAULT_MLINK_MASK                        0
#define CPMAC_DEFAULT_PASS_CRC                          FALSE
#define CPMAC_DEFAULT_QOS_ENABLE                        FALSE
#define CPMAC_DEFAULT_NO_BUFFER_CHAINING                FALSE
#define CPMAC_DEFAULT_COPY_MAC_CONTROL_FRAMES_ENABLE    FALSE
#define CPMAC_DEFAULT_COPY_SHORT_FRAMES_ENABLE          FALSE
#define CPMAC_DEFAULT_COPY_ERROR_FRAMES_ENABLE          FALSE
#define CPMAC_DEFAULT_PROMISCOUS_CHANNEL                0
#define CPMAC_DEFAULT_BROADCAST_CHANNEL                 0
#define CPMAC_DEFAULT_MULTICAST_CHANNEL                 0
#define CPMAC_DEFAULT_BUFFER_OFFSET                     0
#define CPMAC_DEFAULT_TX_PRIO_TYPE                      CPMAC_TXPRIO_FIXED
#define CPMAC_DEFAULT_TX_SHORT_GAP_ENABLE               FALSE
#define CPMAC_DEFAULT_TX_PACING_ENABLE                  FALSE
#define CPMAC_DEFAULT_MII_ENABLE                        TRUE
#define CPMAC_DEFAULT_TX_FLOW_ENABLE                    FALSE
#define CPMAC_DEFAULT_RX_FLOW_ENABLE                    FALSE
#define CPMAC_DEFAULT_LOOPBACK_ENABLE                   FALSE
#define CPMAC_DEFAULT_FULL_DUPLEX_ENABLE                TRUE
#define CPMAC_DEFAULT_TX_INTERRUPT_DISABLE              TRUE
#define CONFIG_CPMAC_MIB_TIMER_TIMEOUT                  5000	/* 5 seconds should be enough */

#define CPMAC_DEFAULT_PROMISCOUS_ENABLE                 0
#define CPMAC_DEFAULT_BROADCAST_ENABLE                  1
#define CPMAC_DEFAULT_MULTICAST_ENABLE                  1

/* Linux invalidate function. This macro is provided for easy configurability */
#define CPMAC_DDA_CACHE_INVALIDATE(addr, size)      PAL_osCacheInvalidate(PAL_OSMEM_ADDR_DAT, (Uint32)addr, size)

/* Linux writeback function. This macro is provided for easy configurability */
#define CPMAC_DDA_CACHE_WRITEBACK(addr, size)       PAL_osCacheWb(PAL_OSMEM_ADDR_DAT, (Uint32)addr, size)

/* NOT EXPLICIT SUPPORT PROVIDED AS OF NOW - Vlan support in the driver */
#define CPMAC_DDA_DEFAULT_VLAN_ENABLE       FALSE

/* System value for ticks per seconds */
#define CPMAC_DDA_TICKS_PER_SEC             HZ

/* CPMAC new Ioctl's created - using a value with a base that can be adjusted as per needs */
#define CPMAC_DDA_IOCTL_BASE                0

/* Filtering IOCTL */
#define CPMAC_DDA_PRIV_FILTERING            (CPMAC_DDA_IOCTL_BASE + 1)

/* Read/Write MII */
#define CPMAC_DDA_PRIV_MII_READ             (CPMAC_DDA_IOCTL_BASE + 2)
#define CPMAC_DDA_PRIV_MII_WRITE            (CPMAC_DDA_IOCTL_BASE + 3)

/* Get/Clear Statistics */
#define CPMAC_DDA_PRIV_GET_STATS            (CPMAC_DDA_IOCTL_BASE + 4)
#define CPMAC_DDA_PRIV_CLR_STATS            (CPMAC_DDA_IOCTL_BASE + 5)

/* External Switch configuration */
#define CPMAC_DDA_EXTERNAL_SWITCH           (CPMAC_DDA_IOCTL_BASE + 6)

/* Extra bytes for Cache alignment of skbuf - should be equal to processor
 * cache line size - in case of ARM926 its 32 bytes
 */
#define CPMAC_DDA_DEFAULT_EXTRA_RXBUF_SIZE  32

/* Default max frame size = 1522 = 1500 byte data + 14 byte eth header + 4 byte checksum + 4 byte Vlan tag  + 32 bytes for Cache alignment*/
#define CPMAC_DDA_DEFAULT_MAX_FRAME_SIZE    (1500 + 14 + 4 + 4 + EGRESS_TRAILOR_LEN + CPMAC_DDA_DEFAULT_EXTRA_RXBUF_SIZE)

/* Default number of TX channels */
#define CPMAC_DDA_DEFAULT_NUM_TX_CHANNELS   1

/* Default TX channel number */
#define CPMAC_DDA_DEFAULT_TX_CHANNEL        0

/* Default TX number of BD's/Buffers */
#define CPMAC_DDA_DEFAULT_TX_NUM_BD         128

/* Default TX max service BD's */
#define CPMAC_DDA_DEFAULT_TX_MAX_SERVICE    32

/* Default number of RX channels */
#define CPMAC_DDA_DEFAULT_NUM_RX_CHANNELS   1

/* Default RX channel number */
#define CPMAC_DDA_DEFAULT_RX_CHANNEL        0

#define CPMAC_DDA_DEFAULT_RX_NUM_BD         128

/* Default RX max service BD's */
#define CPMAC_DDA_DEFAULT_RX_MAX_SERVICE    32	/* This should be equal to netdev->weight parameter */

#if ((CPMAC_DDA_DEFAULT_TX_NUM_BD +CPMAC_DDA_DEFAULT_RX_NUM_BD)  > 256)
#error "Error. DaVinci has space for no more than 256 TX+RX BD's"
#endif				/* 
				 */

/* 
 * Size of CPMAC peripheral footprint in memory that needs to be reserved in Linux 
 * Note that this value is actually a hardware memory footprint value taken from the specs
 * and ideally should have been in the csl files. Keeping it for convinience since CPMAC
 * peripheral footprint will not change unless the peripheral itself changes drastically 
 * and it will be called with a different name and will have a different driver anyway
 * 
 * For Davinci size = control regs (4k) + wrapper regs (4k) + wrapper RAM (8k) + mdio regs (2k)
 */
#define CPMAC_DDA_DEFAULT_CPMAC_SIZE        0x4800

/* ENV variable names for obtaining MAC Addresses */
#define CPMAC_DDA_MAC_ADDR_A    "maca"
#define CPMAC_DDA_MAC_ADDR_B    "macb"
#define CPMAC_DDA_MAC_ADDR_C    "macc"
#define CPMAC_DDA_MAC_ADDR_D    "macd"
#define CPMAC_DDA_MAC_ADDR_E    "mace"
#define CPMAC_DDA_MAC_ADDR_F    "macf"

/* Maximum multicast addresses list to be handled by the driver - If this is not restricted
 * then the driver will spend considerable time in handling multicast lists 
 */
#define CPMAC_DDA_DEFAULT_MAX_MULTICAST_ADDRESSES   64

#endif				/* _CPMAC_LX_DDA_CFG_H_ */
