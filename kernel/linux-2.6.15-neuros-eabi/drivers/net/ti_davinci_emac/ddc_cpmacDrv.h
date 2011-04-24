/*
 * linux/drivers/net/ti_davinci_emac/ddc_cpmacDrv.h
 *
 * EMAC Driver Core internal header file
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
 ver  0.1 PSP architecture team
 */

#ifndef __DDC_CPMAC_DRV_H__
#define __DDC_CPMAC_DRV_H__

/* This file will be included ONLY by ALL CPMAC DDC "implementation" files and hence
   should define CPMAC_DDC macro to get the right definition of CpmacDDCObj
*/
#define CPMAC_DDC

#include "cslr_cpmac.h"		/* Include CPMAC CSL headers */
#include "cslr_ewrap.h"
#include "ddc_cpmacCfg.h"	/* CPMAC DDC driver configuration parameters */

/**
 * \defgroup CPMAC_DDC_Internal CPMAC DDC Internal
 * 
 *  CPMAC DDC Layer Internal Data Structures/Functions
 */
/*@{*/

/* Required for hiding internal object name behind the public name */
typedef struct _CpmacDDCObj_t CpmacDDCObj;

#include "ddc_cpmac.h"		/* DDC CPMAC Interface */

/* Debug Mechanism */
/* To be defined by the makefile - #define CPMAC_DDC_DEBUG */

/* Debug flags 
 *
 * IMPORTANT NOTE: The debug flags need to be enabled carefully as it could flood the console/sink point
 * of the debug traces and also affect the functionality of the overall system
 */
#define CPMAC_DEBUG_FUNCTION_ENTRY          (0x1 << 1)	/* Almost All functions entry/exit */
#define CPMAC_DEBUG_FUNCTION_EXIT           (0x1 << 2)
#define CPMAC_DEBUG_BUSY_FUNCTION_ENTRY     (0x1 << 3)	/* Busy functions - frequently called entry/exit */
#define CPMAC_DEBUG_BUSY_FUNCTION_EXIT      (0x1 << 4)
#define CPMAC_DEBUG_TX                      (0x1 << 6)	/* Transmit functionality */
#define CPMAC_DEBUG_RX                      (0x1 << 7)	/* Receive functionality */
#define CPMAC_DEBUG_PORT_UPDATE             (0x1 << 8)	/* Port status updates */
#define CPMAC_DEBUG_MII                     (0x1 << 9)	/* MII debug */
#define CPMAC_DEBUG_TEARDOWN                (0x1 << 10)	/* Teardown debug */

#ifdef CPMAC_DDC_DEBUG
#define LOGERR(format, args...)         hDDC->ddaIf->ddaErrLog(format, ##args);
#define LOGMSG(flag, format, args... )  { if (flag & CpmacDDCDebug) hDDC->ddaIf->ddaPrintf(format, ## args); }
#else				/* 
				 */
#define LOGERR(format, args...)
#define LOGMSG(flag, format, args... )
#endif				/* 
				 */

/* DDC Internal macros */
#define CPMAC_RX_BD_BUF_SIZE                    0xFFFF;
#define CPMAC_BD_LENGTH_FOR_CACHE               16	/* Only CPPI specified bytes to be invalidated */
#define CPMAC_RX_BD_PKT_LENGTH_MASK             0xFFFF

/**
 *  \brief TX Buffer Descriptor 
 *
 *  CPPI 3.0 TX BD structure specific to CPMAC.
 */
typedef struct {

	Int32 hNext;
		/**< next (hardware) buffer descriptor pointer */

	Int32 buffPtr;
		/**< data buffer pointer */

	Int32 off_bLen;
	       /**< (buffer_offset_16)(buffer_length_16) */

	Int32 mode;
		/**< SOP, EOP, Ownership, EOQ, Teardown, Q Starv, Length */

	Ptr next;
		/**< Pointer to the next TX buffer descriptor (linked list) */

	DDC_NetDataToken bufToken;
			   /**< Data Buffer (OS) Token */

	Ptr eopBD;
		/**< Pointer to end of packet BD */

} CpmacTxBD;

/* Forward declaration */
typedef struct _CpmacRxCppiCh_t CpmacRxCppiCh;

/**
 *  \brief RX Buffer Descriptor 
 *
 *  CPPI 3.0 RX BD structure specific to CPMAC.
 */
typedef volatile struct {

	Int32 hNext;
		/**< next (hardware) buffer descriptor pointer */

	Int32 buffPtr;
		/**< data buffer pointer */

	Int32 off_bLen;
	       /**< (buffer_offset_16)(buffer_length_16) */

	Int32 mode;
		/**< SOP, EOP, Ownership, EOQ, Teardown, Q Starv, Length */

	Ptr next;
		/**< Pointer to the next RX buffer in BD queue */

	Ptr dataPtr;
		/**< DataPtr (virtual address) of the buffer allocated */

	DDC_NetDataToken bufToken;
			   /**< Data Buffer (OS) Token */

	CpmacRxCppiCh *rxCppi;
			   /**< RX CPPI channel owning this BD */

} CpmacRxBD;

/**
 *  \brief TX Channel Control Structure 
 *
 *  Used by CPMAC DDC code to process TX Buffer Descriptors
 */
typedef struct {

    /** Configuration info */
	CpmacChInfo chInfo;		/**< Channel config/info */

	/* CPPI specific */
	Uint32 allocSize;		/**< Buffer Descriptor pool allocated memory size */

	Char *bdMem;		/**< Buffer Descriptor Memory pointer */

	CpmacTxBD *bdPoolHead;
			/**< Free BD Pool Head */

	CpmacTxBD *activeQueueHead;
				/**< Head of active packet queue */

	CpmacTxBD *activeQueueTail;
				/**< Last hardware buffer descriptor written */

	CpmacTxBD *lastHwBDProcessed;
				/**< Last hardware buffer descriptor processed */

	Bool queueActive;	/**< Queue Active ? TRUE/FALSE */

#ifdef CPMAC_MULTIPACKET_TX_COMPLETE_NOTIFY
	Uint32 *txComplete;		/**< Tx complete notification queue */

#endif				/* 
				 */

    /** Statistics */
	Uint32 procCount;		/**< TX packet processing count - number of times CpmacTxBDProc is called */

	Uint32 misQueuedPackets;/**< Misqueued packets */

	Uint32 queueReinit;	/**< Queue reinit - Head ptr reinit */

	Uint32 endOfQueueAdd;	/**< Packet added to end of queue in Send */

	Uint32 outOfTxBD;	/**< out of tx bd errors */

	Uint32 noActivePkts;	/**< Interrupt raised when there were no packets to process */

	Uint32 activeQueueCount;/**< Active tx bd count */

	Uint32 numMultiFragPkts;/**< Number of packets with multiple fragments */

} CpmacTxCppiCh;

/**
 *  \brief RX Channel Control Structure 
 *
 *  Used by CPMAC DDC code to process RX Buffer Descriptors
 */
typedef struct _CpmacRxCppiCh_t {

	/* Back pointer to the DDC instance owning this Channel structure.
	 * Useful for returning a buffer to a given RX pool - saves on Buffer descriptor space
	 */
	CpmacDDCObj *hDDC;

	/* Configuration info */
	CpmacChInfo chInfo;		/**< Channel config/info */

	/* CPMAC (ethernet) specific configuration info */
	Char macAddr[6];		/**< Ethernet MAC address */

    /** CPPI specific */
	Uint32 allocSize;		/**< Buffer Descriptor pool allocated memory size */

	Char *bdMem;		/**< Buffer Descriptor Memory pointer */

	CpmacRxBD *bdPoolHead;
			/**< Free BD Pool Head */

	CpmacRxBD *activeQueueHead;
				/**< Head of active packet queue - next BD processed from here */

	CpmacRxBD *activeQueueTail;
				/**< Active BD Queue Tail */

	Bool queueActive;	/**< Queue Active ? TRUE/FALSE */

	/* Packet and buffer objects required for passing up to DDA layer for the given instance */
#ifdef CPMAC_MULTIPACKET_RX_COMPLETE_NOTIFY
	DDC_NetPktObj *pktQueue;			/**< Packet queue */

	DDC_NetBufObj *bufQueue;		/**< Buffer queue */

#else				/* 
				 */
	DDC_NetPktObj pktQueue;				/**< Single packet */

	DDC_NetBufObj bufQueue[CPMAC_MAX_RX_FRAGMENTS];
					       /**< Buffers for a single packet */

#endif				/* 
				 */

#ifdef CPMAC_MULTIFRAGMENT
	Uint32 rxBufferPtr[CPMAC_MAX_RX_FRAGMENTS];	    /**< RX Buffer pointer local storage */

	Uint32 rxDataToken[CPMAC_MAX_RX_FRAGMENTS]; /**< Buffer token local storage */

#endif				/* 
				 */
    /** Statistics */
	Uint32 procCount;		/**< RX packet processing count - number of times CpmacRxBDProc is called */

	Uint32 processedBD;	/**< Number of BD's processed */

	Uint32 recycledBD;	/**< Number of recycled BD's */

	Uint32 outOfRxBD;	/**< NO BD's available */

	Uint32 outOfRxBuffers;	/**< NO buffers available */

	Uint32 queueReinit;	/**< Queue re-init condition when recycling buffers */

	Uint32 endOfQueueAdd;	/**< End of queue condition - when adding BD at end */

	Uint32 endOfQueue;	/**< End of queue condition */

	Uint32 misQueuedPackets;/**< Mis-queued packet condition */

	Uint32 numMultiFragPkts;/**< Number of packets with multiple fragments */

} _CpmacRxCppiCh;

/* Data structures and header files required for MII-MDIO module  */
typedef struct _phy_device PHY_DEVICE;

#include "cpswhalcommon_stddef.h"	/* Required for MII module defns */
#include "cpswhalcommon_miimdio.h"

/**
 *  \brief CPMAC DDC object 
 *
 *  CPMAC DDC layer Object - encapsulates all bookeeping and data structure for CPMAC DDC
 */
typedef struct _CpmacDDCObj_t {

    /** DDC generic parameters */
	DDC_Obj ddcObj;		/* DDC Object containing version, instance id and driver state */

    /** CPMAC specific parameters - DDC device specifics */
	CpmacInitConfig initCfg;			    /**< Initialization Configuration */

	CpmacTxCppiCh *txCppi[CPMAC_MAX_TX_CHANNELS];
						    /**< Tx Control structure pointers */

	CpmacRxCppiCh *rxCppi[CPMAC_MAX_RX_CHANNELS];
						    /**< Rx Control structure pointers */

	Bool txIsCreated[CPMAC_MAX_TX_CHANNELS];    /**< TX Channel created ? */

	Bool rxIsCreated[CPMAC_MAX_RX_CHANNELS];    /**< RX Channel created ? */

	Bool txIsOpen[CPMAC_MAX_TX_CHANNELS];	    /**< TX channel opened ? */

	Bool rxIsOpen[CPMAC_MAX_RX_CHANNELS];	    /**< RX channel opened ? */

	Bool txTeardownPending[CPMAC_MAX_TX_CHANNELS];	     /**< IS TX Teardown pending ? */

	Bool rxTeardownPending[CPMAC_MAX_RX_CHANNELS];	     /**< IS RX Teardown pending ? */

	Int txIntThreshold[CPMAC_MAX_TX_CHANNELS];	  /**< TX  Completion Threshold count */

	Bool txInterruptDisable;	/* Is Tx completion interrupt disabled? */

	/* Register Mirror values - maintained to avoid costly register access for reads */
	Uint32 Rx_Unicast_Set;		    /**< Unicast Set Register */

	Uint32 Rx_Unicast_Clear;    /**< Unicast Clear Register */

	Uint32 Rx_MBP_Enable;	    /**< RX MBP Register */

	Uint32 MacHash1;	    /**< MAC Hash 1 Register */

	Uint32 MacHash2;	    /**< MAC Hash 2 Register */

	Uint32 MacControl;	    /**< MACControl Register */

	CpmacDDCStatus status;
			    /**< Structure to capture hardware status */

	Uint32 multicastHashCnt[CPMAC_NUM_MULTICAST_BITS];  /**< Number of multicast hash bits used in hardware */

	/* CPMAC/CPGMAC addressing mechanism */
	Uint32 RxAddrType;	       /**< Address Type: 0 (CPMAC), 1 or 2 (CPGMAC) * MAC Config type */

	PHY_DEVICE *PhyDev;	/**< MII-MDIO module device structure */

	CpmacRegsOvly regs;	/**< Pointer points to CPMAC Base address - Register overlay */

	EwrapRegs *eWrapRegs;	/**< Pointer to base address of CPMAC Wrapper for DaVinci */

	struct mib2_ifHCCounters Mib2IfHCCounter;

	CpmacDDACbIf *ddaIf;   /**< DDA provided callback functions */

	CpmacDDCIf *ddcIf;     /**< DDC implemented functions */

} _CpmacDDCObj;

/* Function prototypes */
PAL_Result DDC_cpmacSend(CpmacDDCObj * hDDC, DDC_NetPktObj * pkt,
			 Int channel, Ptr sendArgs);

PAL_Result cpmacTick(CpmacDDCObj * hDDC, Ptr tickArgs);

Int cpmacPktProcess(CpmacDDCObj * hDDC, Int * pktsPending, Ptr pktArgs);

Int cpmacPktProcessEnd(CpmacDDCObj * hDDC, Ptr procArgs);

Int cpmacTxBDProc(CpmacDDCObj * hDDC, Uint32 channel, Uint32 * morePkts,
		  Bool * isEOQ);

Int CpmacRxBDProc(CpmacDDCObj * hDDC, Uint32 channel, Int32 * morePkts);

#ifdef CPMAC_MULTIFRAGMENT
void cpmacAddBDToRxQueue(CpmacDDCObj * hDDC, CpmacRxCppiCh * rxCppi,
			 CpmacRxBD * sopBD, CpmacRxBD * eopBD, Uint32 * buffer,
			 DDC_NetDataToken * bufToken, Uint32 numBD);

#else				/* 
				 */
void cpmacAddBDToRxQueue(CpmacDDCObj * hDDC, CpmacRxCppiCh * rxCppi,
			 CpmacRxBD * currBD, Char * buffer,
			 DDC_NetDataToken bufToken);

#endif				/* 
				 */
Int cpmacUpdatePhyStatus(CpmacDDCObj * hDDC);

void cpmacGetInterruptCause(CpmacDDCObj * hDDC, Bool * rxPending,
			    Bool * txPending, Ptr causeArgs);

Int cpmacTxPktCompletionProcess(CpmacDDCObj * hDDC, Int * pktsPending,
				Ptr pktArgs);

Int cpmacRxPktProcess(CpmacDDCObj * hDDC, Int * pktsPending, Ptr pktArgs);

/*@}*/

#endif				/* __DDC_CPMAC_DRV_H__ */
