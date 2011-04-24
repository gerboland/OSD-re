/*
 * linux/drivers/net/ti_davinci_emac/cpmacNetLx.h
 *
 * EMAC Network Driver Linux adaptation header file
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
 ver. 0.0 Suraj Iyer - Original Linux drive
      0.1 Anant Gole - Recoded as per TI PSPF architecture (converted to DDA)
      2.0 Suraj Iyer, Sharath Kumar, Ajay Singh - Completed for TI BCG
      3.0 Anant Gole - Modified and ported for DaVinci
 */

#ifndef _CPMAC_LX_DDA_H_
#define _CPMAC_LX_DDA_H_

#include <linux/version.h>
#include "ddc_cpmac.h"
#include "cpmacNetLxCfg.h"

extern int cpmac_debug_mode;

#define NETDEV_PRIV(net_dev)  netdev_priv(net_dev)
#define FREE_NETDEV(net_dev)  free_netdev(net_dev)

#define dbgPrint if (cpmac_debug_mode) printk
#define errPrint printk

/* Misc Error codes */
#define CPMAC_DDA_INTERNAL_FAILURE  -1

/* LED codes required for controlling LED's */
#define CPMAC_LINK_OFF          0
#define CPMAC_LINK_ON           1
#define CPMAC_SPEED_100         2
#define CPMAC_SPEED_10          3
#define CPMAC_FULL_DPLX         4
#define CPMAC_HALF_DPLX         5
#define CPMAC_TX_ACTIVITY       6
#define CPMAC_RX_ACTIVITY       7

/**
 * \brief  CPMAC (DDA) Private Ioctl Structure  
 *
 * Private Ioctl commands provided by the CPMAC Linux Driver use this structure
 */
typedef struct {

	unsigned int cmd;
		   /**< Command */

	void *data; /**< Data provided with the command - depending upon command */

} CpmacDrvPrivIoctl;

/**
 * \brief  CPMAC DDA maintained statistics 
 *
 * Driver maintained statistics (apart from Hardware statistics)
 */
typedef struct {

	unsigned long tx_discards;
			   /**< TX Discards */

	unsigned long rx_discards;
			   /**< RX Discards */

	unsigned long start_tick;
			   /**< Start tick */

} CpmacDrvStats;

/**
 * \brief CPMAC Private data structure
 * 
 * Each CPMAC device maintains its own private data structure and has a pointer 
 * to the net_device data structure representing the instance with the kernel. 
 * The private data structure contains a "owner" member pointing to the net_device
 * structure and the net_device data structure's "priv" member points back to this
 * data structure.
 */
typedef struct {

	void *owner;		    /**< Pointer to the net_device structure */

	unsigned int instanceNum;   /**< Instance Number of the device */

	struct net_device *nextDevice;
				   /**< Next device pointer - for internal use */

	unsigned int linkSpeed;
			    /**< Link Speed */

	unsigned int linkMode;
			    /**< Link Mode */

	unsigned long setToClose;   /**< Flag to indicate closing of device */

	void *ledHandle;	    /**< Handle for LED control */

	/* DDC related parameters */
	CpmacDDCObj *hDDC;		    /**< Handle (pointer) to Cpmac DDC object */

	CpmacDDCIf *ddcIf;	    /**< Handle (pointer) to Cpmac DDC function table */

	CpmacDDCStatus ddcStatus;   /**< Cpmac DDC data structure */

	/* Configuration parameters */
	unsigned char macAddr[6];	   /**< Mac (ethernet) address */

	CpmacInitConfig initCfg;    /**< Init cfg parameters - contains rx and mac cfg structures */

	unsigned int rxBufSize;	    /**< RX Buffer Size - skb size to be allocated for RX */

	unsigned int rxBufOffset;   /**> RX Buffer Offset - extra bytes to be reserved before RX data begins in a skb */

	/* TODO: VLAN TX not supported as of now */
	Bool vlanEnable;		    /**< Vlan enable (TX: 8 priority ch's, RX: 1514 byte frames accepted) */

	/* Channel configuration - though only 1 TX/RX channel is supported, provision is made for max */
	CpmacChInfo txChInfo[CPMAC_MAX_TX_CHANNELS];	    /**< Tx Channel configuration */

	CpmacChInfo rxChInfo[CPMAC_MAX_RX_CHANNELS];/**< Rx Channel configuration */

	/* Periodic Timer required for DDC (MDIO) polling */
	struct timer_list periodicTimer;    /**< Periodic timer required for DDC (MDIO) polling */

	Uint32 periodicTicks;	    /**< Ticks for this timer */

	Bool timerActive;	    /**< Periodic timer active ??? */

	struct timer_list mibTimer;
			     /**< Periodic timer required for 64 bit MIB counter support */

	Uint32 mibTicks;	 /**< Ticks for this timer */

	Bool mibTimerActive;	       /**< Periodic timer active ??? */

	/* Statistics */
	CpmacHwStatistics deviceMib;	    /**< Device MIB - CPMAC hardware statistics counters */

	CpmacDrvStats deviceStats;  /**< Device Statstics */

	struct net_device_stats netDevStats;
					    /**< Linux Network Device statistics */

	/* Statistics counters for debugging */
	Uint32 isrCount;		    /**< Number of interrupts */

	/* TxRxParam struct added */
	RxTxParams napiRxTx;

	/* TX Lock */
	spinlock_t lock;
			/**< Tx lock */

} CpmacNetDevice;

/* Function prototypes - only those functions that are in different C files and need
 * to be referenced from the other sources\
 */
int cpmac_dev_tx(struct sk_buff *skb, struct net_device *p_dev);

irqreturn_t cpmac_hal_isr(int irq, void *dev_id, struct pt_regs *p_cb_param);

Ptr DDA_cpmac_net_alloc_rx_buf(CpmacNetDevice * hDDA, Int bufSize,
			       DDC_NetDataToken * dataToken, Uint32 channel,
			       Ptr allocArgs);

PAL_Result DDA_cpmac_net_free_rx_buf(CpmacNetDevice * hDDA, Ptr buffer,
				     DDC_NetDataToken dataToken,
				     Uint32 channel, Ptr freeArgs);

PAL_Result DDA_cpmac_net_rx(CpmacNetDevice * hDDA, DDC_NetPktObj * pkt,
			    Ptr rxArgs, Ptr arg);

PAL_Result DDA_cpmac_net_tx_complete(CpmacNetDevice * hDDA,
				     DDC_NetDataToken * netDataTokens,
				     Int numTokens, Uint32 channel);

#ifdef CPMAC_MULTIPACKET_RX_COMPLETE_NOTIFY
PAL_Result DDA_cpmac_net_rx_multiple_cb(CpmacNetDevice * hDDA,
					DDC_NetPktObj * netPktList,
					Int numPkts, Ptr rxArgs);

#endif				/* 
				 */

#endif				/* _CPMAC_LX_DDA_H_ */
