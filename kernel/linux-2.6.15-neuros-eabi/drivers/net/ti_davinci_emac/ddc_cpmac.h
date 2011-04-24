/*
 * linux/drivers/net/ti_davinci_emac/ddc_cpmac.h
 *
 * EMAC Driver Core header file
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
      0.2 Sharath Kumar - Incorporated review comments
      0.3 Anant Gole - Modified and ported for DaVinci
 */

#ifndef __DDC_CPMAC_H__
#define __DDC_CPMAC_H__

#include "ddc_netdev.h"		/* Inherit from Network device */
#include "ddc_cpmacCfg.h"	/* DDC Configuration file for ready reference to some HW cfg macros */
#include "cslr_cpmac.h"		/* CSL Header file */
#include "ddc_cpmac_ioctl.h"
#include "ioctl_api.h"

/**
 * \defgroup CPMAC_DDC_Interface    CPMAC DDC Interface
 * 
 *  CPMAC DDC Layer Interface
 */
/*@{*/

/**
 *  \brief DDC CPMAC Error Codes
 *
 *  Refer to DDC Error codes description.
 */

/* Defining the macro CPMAC_INSTANCE_CODE to 0 so that it can be usable in DDA */
#define CPMAC_INSTANCE_CODE                     0	/*hDDC->initCfg.instId */
#define CPMAC_ERROR_CODE                        ((DDC_ERROR | (CPMAC_INSTANCE_CODE << 16)) + DDC_NETDEV_ERROR_MAX)
#define CPMAC_ERROR_INFO                        (CPMAC_ERROR_CODE)
#define CPMAC_ERROR_WARNING                     (CPMAC_ERROR_CODE | 0x10000000)
#define CPMAC_ERROR_MINOR                       (CPMAC_ERROR_CODE | 0x20000000)
#define CPMAC_ERROR_MAJOR                       (CPMAC_ERROR_CODE | 0x30000000)
#define CPMAC_ERROR_CRITICAL                    (CPMAC_ERROR_CODE | 0x40000000)

/* CPMAC Success code */
#define CPMAC_SUCCESS                           PAL_SOK

/* CPMAC Error codes */
#define CPMAC_ERR_DEV_ALREADY_INSTANTIATED(instID) (0x30000000 + DDC_ERROR + DDC_NETDEV_ERROR_MAX + ((instId) << 16) )
#define CPMAC_ERR_DEV_NOT_INSTANTIATED          (CPMAC_ERROR_MAJOR + 1)
#define CPMAC_INVALID_PARAM                     (CPMAC_ERROR_MAJOR + 2)
#define CPMAC_ERR_TX_CH_INVALID                 (CPMAC_ERROR_CRITICAL + 3)
#define CPMAC_ERR_TX_CH_ALREADY_INIT            (CPMAC_ERROR_MAJOR + 4)
#define CPMAC_ERR_TX_CH_ALREADY_CLOSED          (CPMAC_ERROR_MAJOR + 5)
#define CPMAC_ERR_TX_CH_NOT_OPEN                (CPMAC_ERROR_MAJOR + 6)
#define CPMAC_ERR_TX_NO_LINK                    (CPMAC_ERROR_MAJOR + 7)
#define CPMAC_ERR_TX_OUT_OF_BD                  (CPMAC_ERROR_MAJOR + 8)
#define CPMAC_ERR_RX_CH_INVALID                 (CPMAC_ERROR_CRITICAL + 9)
#define CPMAC_ERR_RX_CH_ALREADY_INIT            (CPMAC_ERROR_MAJOR + 10)
#define CPMAC_ERR_RX_CH_ALREADY_CLOSED          (CPMAC_ERROR_MAJOR + 11)
#define CPMAC_ERR_RX_CH_NOT_OPEN                (CPMAC_ERROR_MAJOR + 12)
#define CPMAC_ERR_DEV_ALREADY_CREATED           (CPMAC_ERROR_MAJOR + 13)
#define CPMAC_ERR_DEV_NOT_OPEN                  (CPMAC_ERROR_MAJOR + 14)
#define CPMAC_ERR_DEV_ALREADY_CLOSED            (CPMAC_ERROR_MAJOR + 15)
#define CPMAC_ERR_DEV_ALREADY_OPEN              (CPMAC_ERROR_MAJOR + 16)
#define CPMAC_ERR_RX_BUFFER_ALLOC_FAIL          (CPMAC_ERROR_CRITICAL + 17)
#define CPMAC_INTERNAL_FAILURE                  (CPMAC_ERROR_MAJOR + 18)

/**
 *  \brief CPMAC DDC Ioctl's
 *
 */
#define DDC_NET_CPMAC_IOCTL_BASE                0	/* Arbitrary base */
#define CPMAC_DDC_IOCTL_GET_SWVER                DDC_IOCTL(DDC_NET_IOCTL_MIN, 2)
#define CPMAC_DDC_IOCTL_GET_HWVER                DDC_IOCTL(DDC_NET_IOCTL_MIN, 3)
#define CPMAC_DDC_IOCTL_SET_RXCFG                DDC_IOCTL(DDC_NET_IOCTL_MIN, 4)
#define CPMAC_DDC_IOCTL_SET_MACCFG               DDC_IOCTL(DDC_NET_IOCTL_MIN, 5)
#define CPMAC_DDC_IOCTL_GET_STATUS               DDC_IOCTL(DDC_NET_IOCTL_MIN, 6)
#define CPMAC_DDC_IOCTL_READ_PHY_REG            DDC_IOCTL(DDC_NET_IOCTL_MIN, 7)
#define CPMAC_DDC_IOCTL_WRITE_PHY_REG           DDC_IOCTL(DDC_NET_IOCTL_MIN, 8)
#define CPMAC_DDC_IOCTL_GET_STATISTICS          DDC_NET_IOCTL_GET_NET_STATS
#define CPMAC_DDC_IOCTL_CLR_STATISTICS          DDC_NET_IOCTL_CLR_NET_STATS
#define CPMAC_DDC_IOCTL_MULTICAST_ADDR          DDC_IOCTL(DDC_NET_IOCTL_MIN, 9)	/* Add/Delete */
#define CPMAC_DDC_IOCTL_ALL_MULTI               DDC_IOCTL(DDC_NET_IOCTL_MIN, 10)	/* Set/Clear */
#define CPMAC_DDC_IOCTL_TYPE2_3_FILTERING       DDC_IOCTL(DDC_NET_IOCTL_MIN, 11)
#define CPMAC_DDC_IOCTL_SET_MAC_ADDRESS       DDC_IOCTL(DDC_NET_IOCTL_MIN, 12)
#define CPMAC_DDC_IOCTL_IF_COUNTERS		DDC_IOCTL(DDC_NET_IOCTL_MIN,13)
#define CPMAC_DDC_IOCTL_ETHER_COUNTERS     	DDC_IOCTL(DDC_NET_IOCTL_MIN,14)
#define CPMAC_DDC_IOCTL_IF_PARAMS_UPDT  	DDC_IOCTL(DDC_NET_IOCTL_MIN,15)

/**
 *  \brief CPMAC DDA Ioctl's
 *
 *  Called by the DDC layer, implemented by DDA layer in Control function
 */
#define DDA_NET_CPMAC_IOCTL_BASE              0	/* Arbitrary Base */
#define CPMAC_DDA_IOCTL_TIMER_START           (DDA_NET_CPMAC_IOCTL_BASE + 1)
#define CPMAC_DDA_IOCTL_TIMER_STOP            (DDA_NET_CPMAC_IOCTL_BASE + 2)
#define CPMAC_DDA_IOCTL_STATUS_UPDATE         (DDA_NET_CPMAC_IOCTL_BASE + 3)
#define CPMAC_DDA_IOCTL_MIB64_CNT_TIMER_START (DDA_NET_CPMAC_IOCTL_BASE + 4)
#define CPMAC_DDA_IOCTL_MIB64_CNT_TIMER_STOP (DDA_NET_CPMAC_IOCTL_BASE + 5)

/* 
 * \brief MII module port settings
 *
 * DDA sets the Phy mode as a combination of the following in "phyMode" parameter 
 * in the init configuration structure
 */
#define SNWAY_AUTOMDIX      (1<<16)	/* Bit 16 and above not used by MII register */
#define SNWAY_FD1000        (1<<13)
#define SNWAY_HD1000        (1<<12)
#define SNWAY_NOPHY         (1<<10)
#define SNWAY_LPBK          (1<<9)
#define SNWAY_FD100         (1<<8)
#define SNWAY_HD100         (1<<7)
#define SNWAY_FD10          (1<<6)
#define SNWAY_HD10          (1<<5)
#define SNWAY_AUTO          (1<<0)
#define SNWAY_AUTOALL       (SNWAY_AUTO|SNWAY_FD100|SNWAY_FD10|SNWAY_HD100|SNWAY_HD10)

/**
 *  DDC Status Ioctl - Error status 
 *  
 *  Note that each error code is a bit position so that multiple errors can be
 *  clubbed together and passed in a integer value
 */
#define CPMAC_DDC_NO_ERROR          0	/**< Ioctl Success */
#define CPMAC_DDC_TX_HOST_ERROR     0x1	/**< TX Host Error - "hwErrInfo" MSB 8 bits indicate "error code""channel no" */
#define CPMAC_DDC_RX_HOST_ERROR     0x2	/**< RX Host Error - "hwErrInfo" LSB 8 bits indicate "error code""channel no" */

/**
 *  \brief DDC Status values
 * 
 * Provides status of the device - error status, phy status etc
 *
 */
typedef struct {

	Uint32 hwStatus;	/**< Either NO_ERROR or combination of error bits from above status codes */

	Uint32 hwErrInfo;	/**< If error, then additional info about the error */

	Uint32 PhyLinked;	/**< Link status: 1=Linked, 0=No link */

	Uint32 PhyDuplex;	/**< Duplex status: 1=Full Duplex, 0=Half Duplex */

	Uint32 PhySpeed;	/**< Link Speed = 1=100 mbps, 0=10 Mbbs. (In future this is expected: 10/100/1000 mbps) */

	Uint32 PhyNum;		/**< Phy number - useful if phy number is discovered */

} CpmacDDCStatus;

/**
 *  \brief CPMAC Channel Config Info
 *
 *  Common to both TX/RX
 * Used to pass channel config info from DDA to DDC for CPMAC channels
 */
typedef struct {

	Int chNum;		/**< DDC_NetChInfo: Channel number */

	DDC_NetChDir chDir;	/**< DDC_NetChInfo: Channel direction */

	DDC_NetChState chState;
			/**< DDC_NetChInfo: Channel state */

	Int numBD;		/**< Number of BD (& buffers for RX) */

	Int serviceMax;	/**< Maximum BD's processed in one go */

	Int bufSize;		/**< Buffer Size (applicable for RX only) */

} CpmacChInfo;

/**
 *  \brief CPMAC RX configuration
 *
 *  This data structure configures the RX module of the device
 */
typedef struct {

	Bool passCRC;		    /**< Pass CRC bytes to the packet memory */

	Bool qosEnable;	    /**< Receive QoS enable ? */

	Bool noBufferChaining;	    /**< KEPT FOR DEBUGGING ONLY - ALWAYS SET TO FALSE */

	Bool copyMACControlFramesEnable;
				   /**< Copy MAC Control frames to packet memory */

	Bool copyShortFramesEnable; /**< Copy Short frames to packet memory */

	Bool copyErrorFramesEnable; /**< Copy Errored frames to packet memory */

	Bool promiscousEnable;	    /**< Copy ALL (Promiscous) frames to packet memory */

	Uint32 promiscousChannel;   /**< Promiscous receive channel */

	Bool broadcastEnable;	    /**< Receive broadcast frames ? */

	Uint32 broadcastChannel;    /**< Broadcast receive channel */

	Bool multicastEnable;	    /**< Receive multicast frames ? */

	Uint32 multicastChannel;    /**< Multicast receive channel */

	Uint32 maxRxPktLength;	    /**< Max receive packet length */

	Uint32 bufferOffset;	    /**< Buffer offset for all RX channels */

} CpmacRxConfig;

/**
 *  \brief Transmit Queue priority type
 *
 *  Enums for transmit queue priority type - fixed/round robin available in hardware
 */
typedef enum {
	CPMAC_TXPRIO_ROUND_ROBIN = 0,
			      /**< Round Robin priority mechanism between TX channels */
	CPMAC_TXPRIO_FIXED = 1
	      /**< Fixed priority mechanism between TX channels */
} CpmacTxQueuePriorityType;

/**
 *  \brief CPMAC MAC configuration
 *
 *  This data structure configures the MAC module parameters of the device
 */
typedef struct {

	CpmacTxQueuePriorityType pType;
				   /**< Transmit Queue priority type */

	Bool txShortGapEnable;	    /**< Short Gap enable on transmit - CP(G)MAC only */

	Bool gigaBitEnable;	    /**< Gigabit mode -CP(G)MAC only */

	Bool txPacingEnable;	    /**< Transmit pacing enabled ? */

	Bool miiEnable;	    /**< KEPT FOR DEBUGGING ONLY - ALWAYS SET TO TRUE */

	Bool txFlowEnable;	    /**< TX Flow Control enabled ? */

	Bool rxFlowEnable;	    /**< TX Flow Control enabled ? */

	Bool loopbackEnable;	    /**< Loopback mode enabled ? */

	Bool fullDuplexEnable;	    /**< KEPT FOR DEBUGGING ONLY - Will be set based upon phyMode */

	Bool txInterruptDisable;    /**< To allow Disabling of Tx Completion interrupts */

} CpmacMacConfig;

/**
 *  \brief CPMAC Init Configuration
 *
 *  Configuration information provided to DDC layer during initialization.
 *  DDA gets the config information from the OS/PAL layer and passes the relevant
 *  config to the DDC during initialization. The config info can come from various
 *  sources - static compiled in info, boot time (ENV, Flash) info etc.
 */
typedef struct {

	Uint32 instId;	    /**< DDC Init Cfg: Instance number */

	Uint32 numTxChannels;
			    /**< DDCNet Init Cfg: Number of Tx Channels to be supported */

	Uint32 numRxChannels;
			    /**< DDCNet Init Cfg: Number of Rx Channels to be supported */

	Uint32 cpmacBusFrequency;
			   /**< Bus frequency at which this module is operating */

	Uint32 baseAddress; /**< CPMAC peripheral device's register overlay address */

	Uint32 eWrapBaseAddress;
			   /**< DaVinci module control wrapper base address */

	Uint32 intrLine;    /**< CPMAC Device Interrupt Line Number within the system */

	Uint32 resetLine;   /**< CPMAC Reset Line Number within the system */

	Uint32 mdioBaseAddress;
			   /**< MDIO Device base address */

	Uint32 mdioResetLine;
			    /**< MDIO Device Reset line number within the system */

	Uint32 mdioIntrLine;/**< MDIO Device Interrupt line number within the system */

	Uint32 PhyMask;
		    /**< Phy Mask for this CPMAC Phy  */

	Uint32 MLinkMask;   /**< MLink Mask for this CPMAC Phy  */

	Uint32 MdioBusFrequency;
			   /**< Bus frequency for the MII module */

	Uint32 MdioClockFrequency;
			   /**< Clock frequency for MDIO link */

	Uint32 MdioTickMSec;/**< DDC MDIO Tick count in milliSeconds */

	Uint32 Mib64CntMsec;

	Uint32 phyMode;
		    /**< Phy mode settings - Speed,Duplex */

	CpmacRxConfig rxCfg;/**< RX common configuration */

	CpmacMacConfig macCfg;
			    /**< MAC common configuration */

} CpmacInitConfig;

/* The CPMAC DDC object internals are not important from the DDA layer. It just 
 * needs a handle to the object. Hence the internal data structure for CpmacDDCObj
 * is in the DDC internal header and the DDA layer see it only as a DDC_Handle
 * When the DDC files are compiled they dont see this definition as CPMAC_DDC is 
 * already defined for DDC internal files implementation
 */
#ifndef CPMAC_DDC
typedef DDC_Handle CpmacDDCObj;

#endif				/* 
				 */

/* CPMAC specific function prototypes here */

/** 
 *  \brief CPMAC periodic tick function
 *
 *  Used to check the device status and report state changes.
 *  DDC may use ControlCb() to set the tick time period based upon its needs.
 *  The existing CPMAC hardware supports MDIO interrupt, but is not working and hence a 
 *  periodic poll function needs to check MDIO state and status. The DDA may provide a 
 *  periodic timeout using OS services. 
 *  Note: This can be done using PAL software timer service.
 *
 *  @param  hDDC        DDC Handle
 *  @param  tickArgs    Arguments if any, else NULL. Not used in current implementation.
 *  @return PAL_SUCCESS or PAL Error code
 */
typedef PAL_Result(*DDC_CpmacTick) (CpmacDDCObj * hDDC, Ptr tickArgs);

/** 
 *  \brief CPMAC function indicating the cause of interrupt (to be called by the DDA)
 *
 *  DDA calls this function on occurance of interrupt to know the cause for the interrupt(Rx/Tx).
 * 
 *  @param  hDDC       DDC Handle
 *  @param  rxPending  Placeholder to return TRUE if rx packets are pending to be processed
 *  @param  txPending  Placeholder to return TRUE if tx packets are pending to be processed
 *  @param  causeArgs  Arguments if any, else NULL. Not used in current implementation. 
 *  @return None
 */
typedef void (*DDC_CpmacGetInterruptCause) (CpmacDDCObj * hDDC,
					    Bool * rxPending,
					    Bool * txPending, Ptr causeArgs);

/** 
 *  \brief CPMAC Packet Processing function (to be called by the DDA)
 *
 *  DDA calls the packet packet processing function when the device interrupt is pending.
 *  DDA typically calls this function in a thread context. 
 *
 *  @param  hDDC        DDC Handle
 *  @param  pktsPending Placeholder to return TRUE if packets are still pending to be processed
 *  @param  pktArgs     Arguments if any, else NULL. Kept for future enhancement
 *  @return Number of packets processed
 */
typedef Int(*DDC_CpmacPktProcess) (CpmacDDCObj * hDDC,
				   Int * pktsPending, Ptr pktArgs);

/** 
 *  \brief CPMAC Tx Packet Processing function (to be called by the DDA)
 *
 *  DDA calls the Tx packet processing function when there are pending Tx packets 
 *  to be processed.
 *  DDA typically calls this function in a thread context. 
 *
 *  @param  hDDC        DDC Handle
 *  @param  pktsPending Placeholder to return TRUE if packets are still pending to be processed
 *  @param  pktArgs     Arguments if any, else NULL. Kept for future enhancement
 *  @return Number of packets processed
 */
typedef Int(*DDC_CpmacPktTxCompletionProcess) (CpmacDDCObj * hDDC,
					       Int * pktsPending, Ptr pktArgs);

/** 
 *  \brief CPMAC Rx Packet Processing function (to be called by the DDA)
 *
 *  DDA calls the Rx packet processing function when there are pending Rx packets 
 *  to be processed.
 *  DDA typically calls this function in a thread context. 
 *
 *  @param  hDDC        DDC Handle
 *  @param  pktsPending Placeholder to return TRUE if packets are still pending to be processed
 *  @param  pktArgs     Arguments if any, else NULL. Kept for future enhancement
 *  @return Number of packets processed
 */
typedef Int(*DDC_CpmacPktRxProcess) (CpmacDDCObj * hDDC,
				     Int * pktsPending, Ptr pktArgs);

/** 
 *  \brief CPMAC Packet Processing End Function (to be called by the DDA)
 *
 *  DDA processes packets using the TX and RX packets processing functions. When no packets are
 *  pending or the desired number of packets is processed, device level interrupt must be enabled.
 *  This call triggers the device level interrupt logic to raise an interrupt when new  packets have
 *  arrived or are still pending to be processed.
 *
 *  @param  hDDC        DDC Handle
 *  @param  procArgs    Arguments if any, else NULL. Kept for future enhancement.
 *  @return Success or Failure. Currently since this will be a reg write only, success is returned.
 */
typedef Int(*DDC_CpmacPktProcessEnd) (CpmacDDCObj * hDDC, Ptr procArgs);

/** 
 *  \brief CPMAC Buffer-BD recycling function
 *
 *  When CPMAC_RX_RECYCLE_BUFFER is defined, DDC allocates RX buffer only at init time (during Open)
 *  When a buffer is allocated, the BD ptr is passed along to the DDA RX buffer allocation function
 *  which makes sure that when the received packet is processed, the buffer is freed using this function
 *
 *  @param  hDDC        DDC Handle
 *  @param  channel     Channel number
 *  @param  param       RX Buffer Descriptor pointer passed as parameter
 *  @return Success or Failure. Currently since this will be a reg write only, success is returned.
 */
typedef Int(*DDC_CpmacRecycleBuffer) (CpmacDDCObj * hDDC,
				      Int channel, Ptr param);

/**
 * \brief CPMAC DDC Interface Structure
 *
 * CPMAC DDC Interface object - inherits interfaces from network device 
 */
typedef struct {

	/* Always super class members first */
	DDC_NetFuncTable ddcNetIf;	/* DDC and Net function table */

    /** DDC CPMAC specific functions */
#ifdef CPMAC_RX_RECYCLE_BUFFER
	DDC_CpmacRecycleBuffer ddcRecycleBuffer;/**< DDC CPMAC Buffer recycle function */

#endif				/* 
				 */
	DDC_CpmacTick ddctick;			   /**< DDC CPMAC Tick function */

	DDC_CpmacPktProcess pktProcess;
				   /**< DDC CPMAC packet processing function */

	DDC_CpmacPktProcessEnd pktProcessEnd;
					   /**< DDC CPMAC End of packet processing function */

	DDC_CpmacGetInterruptCause getInterruptCause;
						   /**< DDC CPMAC function returning interrupt cause */

	DDC_CpmacPktTxCompletionProcess pktTxCompletionProcess;
							 /**< DDC CPMAC Tx packet processing function */

	DDC_CpmacPktRxProcess pktRxProcess;/**< DDC CPMAC Rx packet processing function */

} CpmacDDCIf;

/** 
 *  \brief Debug printf / Error log s function (provided by DDA, used by DDC for debug prints
 *
 *  DDA provides this callback function to be used by DDC layer for debug printing and error logging.
 */
typedef Int(*DDA_Printf) (const char *format, ...);

typedef Int(*DDA_ErrLog) (const char *format, ...);

/** 
 * \brief DDA Callback Interface Structure 
 *
 * Callback functions provided by the DDA layer to be called by DDC.
 * Inherits from Network device callback functions
 */
typedef struct {

	/* Always super class members first */
	DDA_NetFuncTable ddaNetIf;	    /**< Net function table - includes DDC level DDA Cb table */

	/* Debug printf and error functions */
	DDA_Printf ddaPrintf;		    /**< DDA provided debug printing function */

	DDA_ErrLog ddaErrLog;	    /**< DDA provided error logging function */

} CpmacDDACbIf;

/**
 *  \brief CPMAC Get DDC Version Info
 *
 *  This function returns the version information of the DDC implementation for CPMAC
 *  device. The version number is returned in the pointer parameter passed to the function
 *  and the string is returned as the return value of the function.
 *
 *  @param  swVer       Placeholder for returning Version Id (major/minor version)
 *                      Major Version Id (MSB 16 bits), minor version Id (LSB 16 bits)
 *  @return Returns version string (array of char's stored in DDC)
 */
String DDC_cpmacGetVersionInfo(Uint32 * swVer);

/**
 * TODO:
 * \brief CPMAC DDC Version Compatibility Check
 *
 * This function provides a runtime usable version compat check function as specified in 
 * PSP framework guidelines.
 */

/**
 * \brief CPMAC DDC Create Instance
 * 
 * CPMAC Instance creation function. DDA/DDC Handle and interface pointer exchange happens here
 *
 *  @param  instId          Instance identifier
 *  @param  hDDA            Handle to the DDA layer
 *  @param  hDDA            Handle to the DDA layer
 *  @param  hDDACbIf        Handle to the DDA Callback interface structure
 *  @param  hDDC            Placeholder for Handle to the DDC layer
 *  @param  hDDCIf          Placeholder for Handle to the DDC interface structure
 *  @param  param           Not used in this implementation. For future use.
 *  @return Success or failure (PAL success/error code)
 */
PAL_Result DDC_cpmacCreateInstance(Uint32 instId, DDA_Handle hDDA,
				   CpmacDDACbIf * hDDACbIf,
				   DDC_Handle ** hDDC,
				   CpmacDDCIf ** hDDCIf, Ptr param);

/*@}*/
Ptr cpmacGetPhyDev(CpmacDDCObj * hDDC);

typedef struct {

	Uint32 rxPkts;		/* Number of rx pkts to be processed */

	Uint32 txPkts;		/* Number of tx pkts to be processed */

	Uint32 retRxPkts;	/* Number of rx pkts processed */

	Uint32 retTxPkts;	/* Number of tx pkts processed */

} RxTxParams;

#endif				/* __DDC_CPMAC_H__ */
