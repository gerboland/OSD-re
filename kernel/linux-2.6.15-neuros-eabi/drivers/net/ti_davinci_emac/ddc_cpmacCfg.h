/*
 * linux/drivers/net/ti_davinci_emac/ddc_cpmacCfg.h
 *
 * EMAC Driver Core configuration header file
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
 ver  0.1 Anant Gole - Created
 */

#ifndef __DDC_CPMAC_CFG_H__
#define __DDC_CPMAC_CFG_H__

/* DaVinci specific configuration */
#define DAVINCI_CPMAC_BASE_ADDR             IO_ADDRESS(0x01C80000)
#define DAVINCI_CPMAC_WRAPPER_REGS_ADDR     IO_ADDRESS(0x01C81000)
#define DAVINCI_CPMAC_WRAPPER_RAM_ADDR      IO_ADDRESS(0x01C82000)
#define DAVINCI_CPMAC_WRAPPER_RAM_SIZE      0x2000	/* 8K */
#define DAVINCI_CPMAC_MDIO_BASE_ADDR        IO_ADDRESS(0x01C84000)

#define DAVINCI_CPMAC_INTERRUPT             13
#define DAVINCI_CPMAC_BUS_FREQUENCY         76500000	/* PLL/6 i.e 76.5 MHz for normal operation */
#define DAVINCI_CPMAC_MDIO_FREQUENCY        2200000	/* PHY bus frequency */
#define DAVINCI_CPMAC_PHY_MASK              0x2	/* PHY chip is located at address 1 on DaVinci EVM */

/* Note: For DaVinci, Buffer Descriptors are located in Wrapper RAM (4K).
 * Half of the Wrapper memory is for RX BD's and other half for TX BD's
 */
#define DAVINCI_CPMAC_TX_BD_MEM             DAVINCI_CPMAC_WRAPPER_RAM_ADDR
#define DAVINCI_CPMAC_RX_BD_MEM             (DAVINCI_CPMAC_WRAPPER_RAM_ADDR + (DAVINCI_CPMAC_WRAPPER_RAM_SIZE >> 1))

/* Feature macros here */

/* If multi packet Tx complete notifications is enabled (via CPMAC_MULTIPACKET_TX_COMPLETE_NOTIFY), 
   Max number of Tx packets that can be notified - the actual number will depend upon 
   user configuration for parameter "maxPktsToProcess" */
#define CPMAC_DDC_MAX_TX_COMPLETE_PKTS_TO_NOTIFY    8

/* If multi packet Rx indication is enabled (via CPMAC_MULTIPACKET_RX_COMPLETE_NOTIFY)
   Max number of Rx packets that can be notified - the actual number will depend upon 
   user configuration for parameter "maxPktsToProcess" */
#define CPMAC_DDC_MAX_RX_COMPLETE_PKTS_TO_NOTIFY    8

/* Config macros */
#define CPMAC_MAX_INSTANCES                     1	/**< Max CPMAC instances */
#define CPMAC_MIN_ETHERNET_PKT_SIZE             60	/**< Minimum Ethernet packet size */

/* Max RX fragments calculation - 1500 byte packet and 64 byte buffer. Fragments=1500/64=24 */
#define CPMAC_MAX_RX_FRAGMENTS                  24	/**< Maximum RX fragments supported */

/* Theoratically TX max fragments are equal to 24 */
#define CPMAC_MAX_TX_FRAGMENTS                  8	/**< Maximum TX fragments supported */

/* CPMAC hardware specific */
#define CPMAC_RESET_CLOCKS_WAIT                 64	/**< Clocks to wait for reset operation */
#define CPMAC_MAX_TX_CHANNELS                   8	/**< Maximum TX Channels supported by the DDC */
#define CPMAC_MAX_RX_CHANNELS                   8	/**< Maximum RX Channels supported by the DDC */
#define CPMAC_MIN_FREQUENCY_FOR_10MBPS          5500000	    /**< Minimum CPMAC bus frequency for 10   Mbps operation is 5.5 MHz (as per specs) */
#define CPMAC_MIN_FREQUENCY_FOR_100MBPS         55000000    /**< Minimum CPMAC bus frequency for 100  Mbps operation is 55  MHz (as per specs) */
#define CPMAC_MIN_FREQUENCY_FOR_1000MBPS        125000000   /**< Minimum CPMAC bus frequency for 1000 Mbps operation is 125 MHz (as per specs) */

/* Macros for Address conversions */
#define PAL_CPMAC_VIRT_2_PHYS(addr)             PAL_osVirtToPhys((unsigned int)addr)
#define PAL_CPMAC_VIRT_NOCACHE(addr)            (addr)

/* For DaVinci since BD's are in Non-Cached Wrapper RAM, and since DDC uses 
 * these macros for BD's only, they are stubbed out - they will be required
 * if BD's are moved to Cached memory in future
 */
#define PAL_CPMAC_CACHE_INVALIDATE(addr, size)
#define PAL_CPMAC_CACHE_WRITEBACK(addr, size)
#define PAL_CPMAC_CACHE_WRITEBACK_INVALIDATE(addr, size)

/* Old functions if BD's are in main memory -
#define PAL_CPMAC_CACHE_INVALIDATE(addr, size)  PAL_osCacheInvalidate(PAL_OSMEM_ADDR_DAT, (Uint32)addr, size)
#define PAL_CPMAC_CACHE_WRITEBACK(addr, size)   PAL_osCacheWb(PAL_OSMEM_ADDR_DAT, (Uint32)addr, size)
#define PAL_CPMAC_CACHE_WRITEBACK_INVALIDATE(addr, size)    PAL_osCacheWbInv(PAL_OSMEM_ADDR_DAT, (Uint32)addr, size)
*/

/* Define LOCAL as static - within file */
#ifndef LOCAL
#define LOCAL   static
#endif				/* 
				 */

#endif				/* __DDC_CPMAC_CFG_H__ */
