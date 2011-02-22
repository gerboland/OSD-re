/*
 * linux/drivers/net/ti_davinci_emac/ddc_cpmac.c
 *
 * EMAC Driver Core file
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
      0.4 Anant Gole - Update for DaVinci - code cleanup
      0.5 Anant Gole - Added DaVinci EVM fix for low PHY signal strength in 
                       mdio file and changed version in this file to reflect that
 */

/* Notes;
    This file contains the device driver core for CPMAC/CPGMAC device based upon
    PSP Framework architecture.

    Acknowledgements: This DDC implementation for CP(G)MAC device is based upon
    HAL 2.0 based device driver for CPMAC from BSTC. To benefit from the concepts 
    and implementation experience of HAL development, some code snippets have been 
    ported from the BSTC HAL implementation.
    
    Notes:
    DDA/DDC Common features:
    (1) CPMAC_CACHE_WRITEBACK_MODE should be defined by the make file to support
        write back cache mode
    (2) CPMAC_MULTIFRAGMENT should be defined by the make file to support
        multifragments
    (3) CPMAC_RX_RECYCLE_BUFFER - should be defined by the makefile for Receive 
        buffer recycling mechanism (as explained below)
        At initialization, DDC requests buffers from DDA layer and attaches them to RD BD's.
        When a packet is received, the buffer can be recycled along with the BD or a new buffer
        can be allocated. If buffer recycling is used, the upper layer has to call the DDC Buffer
        Return function and return the buffer/BD back to the RX pool.
        This macro controls this feature. If defined the upper layer will call the DDC return
        function and the BD/buffer will always be connected untill freed.
        If no defined, DDC will get a new buffer from the DDA layer.
    (4) CPMAC_MULTIPACKET_TX_COMPLETE_NOTIFY - should be defined by the makefile for 
        support of multiple Tx complete notifications. If this is defined the Tx complete
        DDA callback function contains multiple packet Tx complete events. 
        IMPORTANT NOTE: The configuration define CPMAC_DDC_MAX_TX_COMPLETE_PKTS_TO_NOTIFY should
        be greater than max tx service pkts.
    (5) CPMAC_MULTIPACKET_RX_COMPLETE_NOTIFY - should be defined by the makefile for 
        support of multiple Receive packet. If this is defined the multiple packet receive 
        DDA callback function contains multiple packet Rx packets. 
        IMPORTANT NOTE: The configuration define CPMAC_DDC_MAX_RX_COMPLETE_PKTS_TO_NOTIFY should
        be greater than max rx service pkts.

    DDC specific features:
    (1) NOT FOR DAVINCI - CPMAC_DDC_USE_ASSEMBLY should be defined by the makefile if the compiler supports assembly
        code in the DDC implementation. 
        Note that this is not used in DaVinci CPMAC driver

    (2) NOT FOR DAVINCI - CPMAC_DDC_MIPS_OPTIMIZED should be defined by the makefile if the "MIPS" architectuer optimized 
        versions of  VIRT_2_PHYS and other time critical macros are desired to be used. If not 
        defined ten PAL_sysXXX functions are used.
        Note that this is not used in DaVinci CPMAC driver

    (3) CPMAC_DDC_DEBUG should be defined by the makefile for compiling in debug statements

 */

#include "cpmac_palOsMem.h"	/* CPMAC port for PAL OS Mem inline functions */
#include "cpmac_palOsCache.h"	/* CPMAC port for PAL OS Cache inline functions */
#include "cpmac_palOsProtect.h"	/* CPMAC port for PAL OS Protect inline functions */
#include "ddc_netdev.h"		/* Network Device Interface - includes DDC Interface */
#include "ddc_cpmacDrv.h"	/* CPMAC Driver DDC Internal Data Structures */

/* Version macro */
#define CPMAC_DDC_MAJOR_VERSION         0
#define CPMAC_DDC_MINOR_VERSION         5

const static Char CpmacDDCVersionString[] = "EMAC DDC version 0.5";

/* Static Global Instance Variable */
static Bool CpmacDDCInstCreated[CPMAC_MAX_INSTANCES] = { False };

static CpmacDDCObj *CpmacDDCObject[CPMAC_MAX_INSTANCES] = { NULL };

static Uint32 CpmacDDCNumInst = 0;

Uint32 CpmacDDCDebug = 0x0;	/* No debug flags by default */

Uint32 CpmacWrapperPtr = DAVINCI_CPMAC_WRAPPER_RAM_ADDR;

/* Local Declarations */
LOCAL PAL_Result DDC_cpmacInit(CpmacDDCObj * hDDC, CpmacInitConfig * initCfg);

LOCAL PAL_Result DDC_cpmacDeInit(CpmacDDCObj * hDDC, Ptr param);

LOCAL PAL_Result DDC_cpmacOpen(CpmacDDCObj * hDDC, Ptr param);

LOCAL PAL_Result DDC_cpmacClose(CpmacDDCObj * hDDC, Ptr param);

LOCAL PAL_Result DDC_cpmacControl(CpmacDDCObj * hDDC, Int cmd, Ptr cmdArg,
				  Ptr param);

LOCAL PAL_Result DDC_cpmacChOpen(CpmacDDCObj * hDDC, CpmacChInfo * chInfo,
				 Ptr chOpenArgs);

LOCAL PAL_Result DDC_cpmacChClose(CpmacDDCObj * hDDC, Int channel,
				  Int direction, Ptr chCloseArgs);

LOCAL PAL_Result cpmacWaitForTeardownComplete(CpmacDDCObj * hDDC,
					      Uint32 channel,
					      DDC_NetChDir direction,
					      Bool blocking);

LOCAL PAL_Result cpmacEnableChannel(CpmacDDCObj * hDDC, Uint32 channel,
				    Uint32 direction);

LOCAL PAL_Result cpmacDisableChannel(CpmacDDCObj * hDDC, Uint32 channel,
				     DDC_NetChDir direction);

LOCAL PAL_Result cpmacInitTxChannel(CpmacDDCObj * hDDC, CpmacChInfo * chInfo,
				    Ptr chOpenArgs);

LOCAL PAL_Result cpmacInitRxChannel(CpmacDDCObj * hDDC, CpmacChInfo * chInfo,
				    Ptr chOpenArgs);

LOCAL PAL_Result cpmacUnInitTxChannel(CpmacDDCObj * hDDC, Uint32 channel,
				      Ptr chCloseArgs);

LOCAL PAL_Result cpmacUnInitRxChannel(CpmacDDCObj * hDDC, Uint32 channel,
				      Ptr chCloseArgs);

PAL_Result DDC_cpmacDeleteInstance(CpmacDDCObj * hDDC, Ptr param);

LOCAL void cpmacSetMacAddress(CpmacDDCObj * hDDC, Uint32 channel,
			      String macAddr);

LOCAL void cpmacDDCIfcntClear(CpmacDDCObj * hDDC);

LOCAL void cpmacDDCIfcntUpdt(CpmacDDCObj * hDDC);

LOCAL void cpmacDDCPhycnt(CpmacDDCObj * hDDC, Uint32 * cmdArg);

/* Local functions */

/************************ HASH SUPPORT FUNCTIONS ************************/

/* Get hash value using mechainsm in specs */
LOCAL Uint32 hashGet(Uint8 * addr)
{

	Uint32 hash;

	Uint8 tmpval;

	Int cnt;

	hash = 0;

	for (cnt = 0; cnt < 2; cnt++) {

		tmpval = *addr++;

		hash ^= (tmpval >> 2) ^ (tmpval << 4);

		tmpval = *addr++;

		hash ^= (tmpval >> 4) ^ (tmpval << 2);

		tmpval = *addr++;

		hash ^= (tmpval >> 6) ^ (tmpval);

	}

	return (hash & 0x3F);

}

/**
 * Hash Table Add 
 *  - Adds mac address to hash table and upates hash bits in hardware
 *  - Returns negative if error, 0 if no change to registers,  >0 if hash registers need to change  
 */
LOCAL Int hashAdd(CpmacDDCObj * hDDC, Uint8 * macAddress)
{

	Uint32 hashValue;

	Uint32 hashBit;

	Uint32 status = 0;

	hashValue = hashGet(macAddress);

	if (hashValue >= CPMAC_NUM_MULTICAST_BITS) {

		LOGERR
		    ("\nERROR:DDC: hashAdd:%d: Invalid Hash Value=%d. Should not be greater than %d",
		     hDDC->ddcObj.instId, hashValue,
		     (CPMAC_NUM_MULTICAST_BITS - 1));

		return (CPMAC_INVALID_PARAM);

	}

	/* Set the hash bit only if not previously set */
	if (hDDC->multicastHashCnt[hashValue] == 0) {

		status = 1;

		if (hashValue < 32) {

			hashBit = (1 << hashValue);

			hDDC->MacHash1 |= hashBit;

		}

		else {

			hashBit = (1 << (hashValue - 32));

			hDDC->MacHash2 |= hashBit;

		}

	}

	/* Increment counter to maintain number of multicast address that map to this hash bit  */
	++hDDC->multicastHashCnt[hashValue];

	return (status);

}

/**
 * Hash Table Del 
 *  - Deletes a mac address from hash table and updates hash register bits
 *  - Returns negative if error, 0 if no change to registers, >0 if hash registers need to change 
 */
LOCAL Int hashDel(CpmacDDCObj * hDDC, Uint8 * macAddress)
{

	Uint32 hashValue;

	Uint32 hashBit;

	hashValue = hashGet(macAddress);

	if (hDDC->multicastHashCnt[hashValue] > 0) {

		/* Decrement counter to reduce number of multicast address that map to this hash bit  */
		--hDDC->multicastHashCnt[hashValue];

	}

	/* If counter still > 0, at least one multicast address refers to this hash bit. So return 0 */
	if (hDDC->multicastHashCnt[hashValue] > 0) {

		return (0);

	}

	if (hashValue < 32) {

		hashBit = (1 << hashValue);

		hDDC->MacHash1 &= ~hashBit;

	}

	else {

		hashBit = (1 << (hashValue - 32));

		hDDC->MacHash2 &= ~hashBit;

	}

	/* Return 1 to indicate change in MacHash registers reqd */
	return (1);

}

/* Updates hash register bits with single multicast address add/delete operation */
void cpmacSingleMulti(CpmacDDCObj * hDDC, CpmacSingleMultiOper oper,
		      Uint8 * addr)
{

	Int32 status = -1;

	switch (oper) {

	case CPMAC_MULTICAST_ADD:

		status = hashAdd(hDDC, addr);

		break;

	case CPMAC_MULTICAST_DEL:

		status = hashDel(hDDC, addr);

		break;

	default:

		LOGERR
		    ("\nWARN: cpmacSingleMulti:%d: Unhandled Single Multicast operation %d",
		     hDDC->ddcObj.instId, oper);

		break;

	}

	/* Write to the hardware only if the register status chances */
	if (status > 0) {

		hDDC->regs->MacHash1 = hDDC->MacHash1;

		hDDC->regs->MacHash2 = hDDC->MacHash2;

	}

}

/* Updates hash register bits for all multi operation (set/clear) */
void cpmacAllMulti(CpmacDDCObj * hDDC, CpmacAllMultiOper oper)
{

	switch (oper) {

	case CPMAC_ALL_MULTI_SET:

		hDDC->MacHash1 = CPMAC_ALL_MULTI_REG_VALUE;

		hDDC->MacHash2 = CPMAC_ALL_MULTI_REG_VALUE;

		break;

	case CPMAC_ALL_MULTI_CLR:

		hDDC->MacHash1 = 0;

		hDDC->MacHash2 = 0;

		break;

	default:

		LOGERR
		    ("\nWARN: cpmacAllMulti:%d: Unhandled All multi operation %d",
		     hDDC->ddcObj.instId, oper);

		break;

	}

	hDDC->regs->MacHash1 = hDDC->MacHash1;

	hDDC->regs->MacHash2 = hDDC->MacHash2;

}

/************************ PHY related functions ************************/

/* Cpmac Update Phy Status - updates phy status variables in hDDC->status "CpmacDDCStatus" structure */
Int cpmacUpdatePhyStatus(CpmacDDCObj * hDDC)
{

	PHY_DEVICE *PhyDev = hDDC->PhyDev;

	Uint32 setPhyMode;

	LOGMSG(CPMAC_DEBUG_BUSY_FUNCTION_ENTRY, "\n+ cpmacUpdatePhyStatus %d",
	       hDDC->ddcObj.instId);

	/* Verify proper device state */
	if (hDDC->ddcObj.state != DDC_OPENED) {

		LOGERR("\nERROR:DDC: cpmacUpdatePhyStatus:%d: Device NOT Open",
		       hDDC->ddcObj.instId);

		return (CPMAC_ERR_DEV_NOT_OPEN);

	}

	setPhyMode = hDDC->initCfg.phyMode;

	/* No Phy Condition */
	if (setPhyMode & SNWAY_NOPHY) {

		/*  No Phy condition, always linked */
		hDDC->status.PhyLinked = 1;

		hDDC->status.PhySpeed = 1;

		hDDC->status.PhyDuplex = 1;

		hDDC->status.PhyNum = 0xFFFFFFFF;	/* No Phy */

		hDDC->MacControl |= (1 << CPMAC_MACCONTROL_FULLDUPLEXEN_SHIFT);

		/* Write mac control register from stored value */
		hDDC->regs->MacControl = hDDC->MacControl;

		goto cpmacUpdatePhyStatus_Exit;

	}

	/* If loopback set in hardware, set link to ON */
	if (hDDC->MacControl & CPMAC_MACCONTROL_LOOPBKEN_MASK) {

		hDDC->status.PhyLinked = 1;

		goto cpmacUpdatePhyStatus_Exit;

	}

	if (setPhyMode & SNWAY_LPBK) {
		hDDC->status.PhyLinked =
		    cpswHalCommonMiiMdioGetLoopback(PhyDev);
	}

	else {
		hDDC->status.PhyLinked = cpswHalCommonMiiMdioGetLinked(PhyDev);
	}

	if (hDDC->status.PhyLinked) {

		/*  Retreive Duplex and Speed and the Phy Number  */
		if (setPhyMode & SNWAY_LPBK) {
			hDDC->status.PhyDuplex = 1;
		}

		else {
			hDDC->status.PhyDuplex =
			    cpswHalCommonMiiMdioGetDuplex(PhyDev);
		}

		hDDC->status.PhySpeed = cpswHalCommonMiiMdioGetSpeed(PhyDev);

		hDDC->status.PhySpeed = hDDC->status.PhySpeed >> 10;

		hDDC->status.PhyNum = cpswHalCommonMiiMdioGetPhyNum(PhyDev);

		/* Set the duplex bit in maccontrol */
		if (hDDC->status.PhyDuplex) {
			hDDC->MacControl |=
			    (1 << CPMAC_MACCONTROL_FULLDUPLEXEN_SHIFT);
		}

		else {
			hDDC->MacControl &=
			    ~(1 << CPMAC_MACCONTROL_FULLDUPLEXEN_SHIFT);
		}

	}

	/* Write mac control register from stored value */
	hDDC->regs->MacControl = hDDC->MacControl;

      cpmacUpdatePhyStatus_Exit:
	LOGMSG(CPMAC_DEBUG_PORT_UPDATE,
	       "\ncpmacUpdatePhyStatus:%d: MacControl=%08X, Status: Phy=%d, Speed=%s, Duplex=%s",
	       hDDC->ddcObj.instId, hDDC->MacControl, hDDC->status.PhyNum,
	       (hDDC->status.PhySpeed) ? "100" : "10",
	       (hDDC->status.PhyDuplex) ? "Full" : "Half");

	LOGMSG(CPMAC_DEBUG_BUSY_FUNCTION_EXIT, "\n- cpmacUpdatePhyStatus:%d:",
	       hDDC->ddcObj.instId);

	return (CPMAC_SUCCESS);

}

Ptr cpmacGetPhyDev(CpmacDDCObj * hDDC)
{

	return (Ptr) hDDC->PhyDev;

}

/* Set Phy Mode */
LOCAL Int cpmacSetPhyMode(CpmacDDCObj * hDDC)
{

	Uint32 setPhyMode;

	Uint32 PhyMode;

	LOGMSG(CPMAC_DEBUG_BUSY_FUNCTION_ENTRY, "\n+ cpmacSetPhyMode:%d:",
	       hDDC->ddcObj.instId);

	/* Verify proper device state */
	if (hDDC->ddcObj.state != DDC_OPENED) {

		LOGERR("\nERROR:DDC: cpmacSetPhyMode:%d: Device NOT Open",
		       hDDC->ddcObj.instId);

		return (CPMAC_ERR_DEV_NOT_OPEN);

	}

	setPhyMode = hDDC->initCfg.phyMode;

	PhyMode = 0;

	if (setPhyMode & SNWAY_AUTO)
		PhyMode |= NWAY_AUTO;

	if (setPhyMode & SNWAY_FD10)
		PhyMode |= NWAY_FD10;

	if (setPhyMode & SNWAY_FD100)
		PhyMode |= NWAY_FD100;

	if (setPhyMode & SNWAY_HD10)
		PhyMode |= NWAY_HD10;

	if (setPhyMode & SNWAY_HD100)
		PhyMode |= NWAY_HD100;

	if (setPhyMode & SNWAY_LPBK)
		PhyMode |= NWAY_LPBK;

	if (setPhyMode & SNWAY_AUTOMDIX)
		PhyMode |= NWAY_AUTOMDIX;

	/* Check for CPMAC Bus frequency for correct speed operation */
	if ((setPhyMode & SNWAY_FD10) || (setPhyMode & SNWAY_HD10)) {

		if (hDDC->initCfg.cpmacBusFrequency <=
		    CPMAC_MIN_FREQUENCY_FOR_10MBPS)

			LOGERR
			    ("\nERROR: Bus speedcpmacSetPhyMode:%d: CpmacFreq(%d) is less than required %d freq for 10Mbps support. CANNOT SUPPORTING 10Mbps",
			     hDDC->ddcObj.instId,
			     hDDC->initCfg.cpmacBusFrequency,
			     CPMAC_MIN_FREQUENCY_FOR_10MBPS);

	}

	else if ((setPhyMode & SNWAY_FD100) || (setPhyMode & SNWAY_HD100)) {

		if (hDDC->initCfg.cpmacBusFrequency <=
		    CPMAC_MIN_FREQUENCY_FOR_100MBPS)

			LOGERR
			    ("\nERROR: Bus speedcpmacSetPhyMode:%d: CpmacFreq(%d) is less than required %d freq for 100Mbps support. CANNOT SUPPORTING 100Mbps",
			     hDDC->ddcObj.instId,
			     hDDC->initCfg.cpmacBusFrequency,
			     CPMAC_MIN_FREQUENCY_FOR_100MBPS);

	}

	/* TODO: Check for Gigabit mode when PHY mode defines for gigabit are available */

	LOGMSG(CPMAC_DEBUG_PORT_UPDATE,
	       "\ncpmacSetPhyMode:%d: MdioPhyMode=%08X, PhyMode=%08d, Auto:%d, FD10:%d, HD10:%d, FD100:%d, HD100:%d",
	       hDDC->ddcObj.instId, setPhyMode, PhyMode,
	       (PhyMode & NWAY_AUTO), (PhyMode & NWAY_FD10),
	       (PhyMode & NWAY_HD10),
	       (PhyMode & NWAY_FD100), (PhyMode & NWAY_HD100));

	cpswHalCommonMiiMdioSetPhyMode(hDDC->PhyDev, PhyMode);

	cpmacUpdatePhyStatus(hDDC);

	LOGMSG(CPMAC_DEBUG_BUSY_FUNCTION_EXIT, "\n- cpmacSetPhyMode:%d:",
	       hDDC->ddcObj.instId);

	return (CPMAC_SUCCESS);

}

/************************ MAC ADDRESSING MODE SUPPORT FUNCTIONS ************************/

/* This function sets / clears the unicast flag in hardware */
LOCAL void cpmacRxUniCast(CpmacDDCObj * hDDC, Uint32 channel, Bool enable)
{

	/* Update local copy of register to save cycles in reading the register */
	if (enable == True) {

		hDDC->Rx_Unicast_Set |= (1 << channel);

		hDDC->Rx_Unicast_Clear &= ~(1 << channel);

	}

	else {

		/* Disable Unicast Channel Setting */
		hDDC->Rx_Unicast_Clear |= (1 << channel);

		hDDC->Rx_Unicast_Set &= ~(1 << channel);

	}

	/* Write to hardware if device is open */
	if (hDDC->ddcObj.state == DDC_OPENED) {

		hDDC->regs->Rx_Unicast_Set = hDDC->Rx_Unicast_Set;

		hDDC->regs->Rx_Unicast_Clear = hDDC->Rx_Unicast_Clear;

	}

}

/**
 * CPMAC Add Type 0 Address 
 *  - set mac address for type 0 addressing (CPMAC)
 *
 * \note This is an internal function of the DDC called from channel enable 
 * API which does channel number range checking and hence its not required.
 * It is assumed that this function will get the correct channel number always 
 */
LOCAL void cpmacAddType0Addr(CpmacDDCObj * hDDC, Uint32 channel,
			     String macAddress)
{

	hDDC->regs->MacSrcAddr_Lo = (macAddress[0] << 8) | (macAddress[1]);	/* bytes 0, 1 */

	hDDC->regs->MacSrcAddr_Hi = (macAddress[2] << 24) | (macAddress[3] << 16) | (macAddress[4] << 8) | (macAddress[5]);	/* bytes 2-5 */

	/* Enable Unicast */
	cpmacRxUniCast(hDDC, channel, True);

}

/**
 * CPMAC Add Type 1 Address
 *  - set mac address for type 1 addressing (CPMAC)
 * 
 * \note This is an internal function of the DDC called from channel enable 
 * API which does channel number range checking and hence its not required.
 * It is assumed that this function will get the correct channel number always 
 */
LOCAL void cpmacAddType1Addr(CpmacDDCObj * hDDC, Uint32 channel,
			     String macAddress)
{

	/* Set MacIndex register with channel number */
	hDDC->regs->MacIndex = channel;

	/* Set MacAddr_Hi register */
	hDDC->regs->MacAddr_Hi =
	    (macAddress[3] << 24) | (macAddress[2] << 16) |
	    (macAddress[1] << 8) | (macAddress[0]);

	/* Set MacAddr_Lo register */
	hDDC->regs->MacAddr_Lo = ((macAddress[5] << 8) | macAddress[4]);

	/* Set Mac Hash */
	hDDC->regs->MacHash1 = 0;

	hDDC->regs->MacHash2 = 0;

	/* As per discussion with hardware folks, it is mandatory to set the source
	   address of the mac, else correct behaviour is not guaranteed */
	cpmacAddType0Addr(hDDC, channel, macAddress);

	/* Enable Unicast */
	cpmacRxUniCast(hDDC, channel, True);

}

/* CPGMAC CFIG 2/3 Type addressing - Filtering */
LOCAL void cpmacAddType2Addr(CpmacDDCObj * hDDC, Uint32 channel,
			     String macAddress,
			     Int index, Bool valid, Int match)
{

	/* Not supported in DaVinci */
}

/************************ HARDWARE CONFIGURATION SUPPORT FUNCTIONS ************************/

/* Set RX Hardware configuration */
void cpmacSetRxHwCfg(CpmacDDCObj * hDDC)
{

	CpmacRxConfig *rxCfg;

	Uint32 rxMbpEnable;

	if (hDDC->ddcObj.state != DDC_OPENED) {

		LOGERR
		    ("\nWARN: cpmacSetRxHwCfg:%d: Function called when device is NOT in open state",
		     hDDC->ddcObj.instId);

		return;

	}

	rxCfg = &hDDC->initCfg.rxCfg;

	/* Set RX MBP Enable register */
	rxMbpEnable =
	    ((rxCfg->passCRC & 0x1) << CPMAC_RXMBP_PASSCRC_SHIFT) |
	    ((rxCfg->qosEnable & 0x1) << CPMAC_RXMBP_QOSEN_SHIFT)
	    |
	    ((rxCfg->noBufferChaining & 0x1) << CPMAC_RXMBP_NOCHAIN_SHIFT) |
	    ((rxCfg->
	      copyMACControlFramesEnable & 0x1) << CPMAC_RXMBP_CMFEN_SHIFT) |
	    ((rxCfg->
	      copyShortFramesEnable & 0x1) << CPMAC_RXMBP_CSFEN_SHIFT) |
	    ((rxCfg->
	      copyErrorFramesEnable & 0x1) << CPMAC_RXMBP_CEFEN_SHIFT) |
	    ((rxCfg->
	      promiscousEnable & 0x1) << CPMAC_RXMBP_CAFEN_SHIFT) |
	    ((rxCfg->promiscousChannel & CPMAC_RXMBP_CHMASK)
	     << CPMAC_RXMBP_PROMCH_SHIFT)
	    |
	    ((rxCfg->broadcastEnable & 0x1) << CPMAC_RXMBP_BROADEN_SHIFT) |
	    ((rxCfg->
	      broadcastChannel & CPMAC_RXMBP_CHMASK) <<
	     CPMAC_RXMBP_BROADCH_SHIFT) |
	    ((rxCfg->
	      multicastEnable & 0x1) <<
	     CPMAC_RXMBP_MULTIEN_SHIFT) |
	    ((rxCfg->
	      multicastChannel & CPMAC_RXMBP_CHMASK) <<
	     CPMAC_RXMBP_MULTICH_SHIFT);

	if (rxCfg->promiscousEnable) {

		/* disable mcast bcast and unicast:  H/W limitation */
		rxMbpEnable &= ~(0x1 << CPMAC_RXMBP_BROADEN_SHIFT);

		rxMbpEnable &= ~(0x1 << CPMAC_RXMBP_MULTIEN_SHIFT);

		/* disable unicast - Warning!! Assuming only one channel open */
		cpmacRxUniCast(hDDC, (hDDC->rxCppi[0])->chInfo.chNum, False);

	} else {

		/* enable unicast - Warning!! Assuming only one channel open */
		cpmacRxUniCast(hDDC, (hDDC->rxCppi[0])->chInfo.chNum, True);

	}

	if (hDDC->Rx_MBP_Enable != rxMbpEnable) {

		hDDC->Rx_MBP_Enable = rxMbpEnable;

		hDDC->regs->Rx_MBP_Enable = rxMbpEnable;

	}

	/* Set max Rx packet length */
	hDDC->regs->Rx_Maxlen = (rxCfg->maxRxPktLength & CPMAC_RX_MAX_LEN_MASK);

	/* Set rx buffer offset */
	hDDC->regs->Rx_Buffer_Offset =
	    (rxCfg->bufferOffset & CPMAC_RX_BUFFER_OFFSET_MASK);

	LOGMSG(CPMAC_DEBUG_FUNCTION_EXIT,
	       "- cpmacSetRxHwCfg: Rx_MBP_Enable = 0x%08x\n", rxMbpEnable);

}

/* Set MAC Configuration - MACControl register */
void cpmacSetMacHwCfg(CpmacDDCObj * hDDC)
{

	CpmacMacConfig *macCfg;

	Uint32 macControl;

	if (hDDC->ddcObj.state != DDC_OPENED) {

		LOGERR
		    ("\nWARN: cpmacSetMacHwCfg:%d: Function called when device is NOT in open state",
		     hDDC->ddcObj.instId);

		return;

	}

	macCfg = &hDDC->initCfg.macCfg;

	macControl =
	    ((macCfg->
	      txShortGapEnable & 0x1) << CPMAC_MACCONTROL_TXSHORTGAPEN_SHIFT) |
	    (((macCfg->pType ==
	       CPMAC_TXPRIO_FIXED) ? 0x1 : 0) <<
	     CPMAC_MACCONTROL_TXPTYPE_SHIFT) |
	    ((macCfg->gigaBitEnable & 0x1) << CPMAC_MACCONTROL_GIGABITEN_SHIFT)
	    |
	    ((macCfg->txPacingEnable & 0x1) <<
	     CPMAC_MACCONTROL_TXPACEEN_SHIFT) |
	    /* THIS LINE FOR REFERENCE ONLY ((macCfg->miiEnable & 0x1) << CPMAC_MACCONTROL_MIIEN_SHIFT) | */
	    (hDDC->MacControl & CPMAC_MACCONTROL_MIIEN_MASK) |
	    ((macCfg->
	      txFlowEnable & 0x1) << CPMAC_MACCONTROL_TXFLOWEN_SHIFT) |
	    ((macCfg->
	      rxFlowEnable & 0x1) << CPMAC_MACCONTROL_RXFLOWEN_SHIFT) |
	    ((macCfg->
	      loopbackEnable & 0x1) << CPMAC_MACCONTROL_LOOPBKEN_SHIFT) |
	    /* THIS LINE FOR REFERENCE ONLY ((macCfg->fullDuplexEnable & 0x1) << CPMAC_MACCONTROL_FULLDUPLEXEN_SHIFT); */
	    (hDDC->MacControl & CPMAC_MACCONTROL_FULLDUPLEXEN_MASK);

	if (hDDC->MacControl != macControl) {

		hDDC->MacControl = macControl;

		hDDC->regs->MacControl = macControl;

	}

}

/************************ CPMAC DDC FUNCTIONS ************************/

/* CPMAC DDC Get Version information */
String DDC_cpmacGetVersionInfo(Uint32 * swVer)
{

	if (swVer != NULL)

		*swVer =
		    ((CPMAC_DDC_MAJOR_VERSION << 16) | CPMAC_DDC_MINOR_VERSION);

	return ((String) & CpmacDDCVersionString[0]);

}

/* Static CPMAC DDC function table */
static CpmacDDCIf CpmacDDCInterface = {
	{			/* DDC Net Class functions */
	 {			/*   DDC Class functions */
	  (DDC_Init) DDC_cpmacInit,	/*      CPMAC Init function */
	  (DDC_DeInit) DDC_cpmacDeInit,	/*      CPMAC DeInit function */
	  (DDC_Open) DDC_cpmacOpen,	/*      CPMAC Open function */
	  (DDC_Close) DDC_cpmacClose,	/*      CPMAC Close function */
	  (DDC_Control) DDC_cpmacControl,	/*      CPMAC Control function */
	  (DDC_DeleteInstance)
	  DDC_cpmacDeleteInstance
	  /*  CPMAC Delete function */
	  },
	 (DDC_NetSend) DDC_cpmacSend,	/*   CPMAC Send function */
	 NULL,			/*   Multiple packet send not supported */
	 NULL,			/*   Poll RX not supported */
	 NULL,			/*   Rx Return not supported */
	 NULL,			/*   DDC_cpmacIsr - NOT REQUIRED - taken care by PktProcess */
	 (DDC_NetChOpen) DDC_cpmacChOpen,	/*   CPMAC channel open function */
	 (DDC_NetChClose) DDC_cpmacChClose
	 /*   CPMAC channel close function */
	 },
	(DDC_CpmacTick) cpmacTick,	/* CPMAC Tick function */
	(DDC_CpmacPktProcess) cpmacPktProcess,	/* CPMAC Packet processing function */
	(DDC_CpmacPktProcessEnd) cpmacPktProcessEnd,	/* CPMAC Packet processing End function */
	(DDC_CpmacGetInterruptCause) cpmacGetInterruptCause,
							 /**< DDC CPMAC function returning interrupt cause */
	(DDC_CpmacPktTxCompletionProcess) cpmacTxPktCompletionProcess,
								   /**< DDC CPMAC Tx packet processing function */
	(DDC_CpmacPktRxProcess) cpmacRxPktProcess
    /**< DDC CPMAC Rx packet processing function */
};

/**
 * CPMAC DDC Instance Create 
 *  - exchange handles and function pointers between DDA/DDC
 * 
 * \note 1. Unless function pointers are exchanged, error log and debug printf statements 
 *          cannot be used in this function. Hence we dont log anything in this function.
 *       2. "param" is not used in this implementation
 */
PAL_Result DDC_cpmacCreateInstance(Uint32 instId, DDA_Handle hDDA,
				   CpmacDDACbIf * hDDACbIf,
				   DDC_Handle ** hDDC,
				   CpmacDDCIf ** hDDCIf, Ptr param)
{

	CpmacDDCObj *cpmacDDCHandle;

	PAL_Result retCode;

	/* Check CPMAC instance */
	if (CpmacDDCInstCreated[instId] == True) {

		return (CPMAC_ERR_DEV_ALREADY_INSTANTIATED(instId));

	}

	/* Allocate memory for CPMAC DDC Instance Object and set to 0 */
	retCode =
	    PAL_osMemAlloc(0, sizeof(CpmacDDCObj), 0, (Ptr *) & cpmacDDCHandle);

	if (retCode != PAL_SOK) {

		return (retCode);

	}

	PAL_osMemSet(cpmacDDCHandle, 0, sizeof(CpmacDDCObj));

	/* Set CPMAC object variables */
	cpmacDDCHandle->ddcObj.versionId =
	    (CPMAC_DDC_MAJOR_VERSION << 16) | CPMAC_DDC_MINOR_VERSION;

	cpmacDDCHandle->ddcObj.instId = instId;

	cpmacDDCHandle->ddcObj.state = DDC_CREATED;

	/* Populate CPMAC DDC Interface Table */
	cpmacDDCHandle->ddcIf = &CpmacDDCInterface;

	/* Save DDA handle and interface table */
	cpmacDDCHandle->ddcObj.hDDA = hDDA;

	cpmacDDCHandle->ddaIf = hDDACbIf;

	/* Pass back DDC Object handle and Interface Table handle */
	*hDDC = (DDC_Handle) cpmacDDCHandle;

	*hDDCIf = cpmacDDCHandle->ddcIf;

	/* Update "instance created" variable */
	CpmacDDCInstCreated[instId] = True;

	CpmacDDCObject[instId] = cpmacDDCHandle;

	++CpmacDDCNumInst;

	return (CPMAC_SUCCESS);

}

/**
 * Delete CPMAC DDC Instance 
 *  - delete the "instance" created via CreateInstance
 * 
 * \note "param" is not used in this implementation
 */
PAL_Result DDC_cpmacDeleteInstance(CpmacDDCObj * hDDC, Ptr param)
{

	PAL_Result retCode = CPMAC_SUCCESS;

	Uint32 instId = hDDC->initCfg.instId;

	LOGMSG(CPMAC_DEBUG_FUNCTION_ENTRY, "\n+ DDC_cpmacDeleteInstance %d",
	       instId);

	/* Check instance created global variable */
	if (CpmacDDCInstCreated[instId] == False) {

		LOGERR
		    ("\nERROR:DDC: DDC_cpmacDeleteInstance:%d: Instance NOT created / Already deleted",
		     instId);

		return (CPMAC_ERR_DEV_NOT_INSTANTIATED);

	}

	/* Update instance created global variable */
	CpmacDDCInstCreated[instId] = False;

	CpmacDDCObject[instId] = NULL;

	--CpmacDDCNumInst;

	/* Free resources allocated */
	if (hDDC != NULL) {

		retCode = PAL_osMemFree(0, hDDC, sizeof(CpmacDDCObj));

		if (retCode != PAL_SOK) {

			LOGERR
			    ("\nERROR:DDC: DDC_cpmacDeleteInstance:%d: Failed to free memory of DDC Instance Object. Error=%08X",
			     instId, retCode);

			hDDC->ddcIf = NULL;

		}

		hDDC = NULL;

	}

	LOGMSG(CPMAC_DEBUG_FUNCTION_EXIT, "\n- DDC_cpmacDeleteInstance:%d:",
	       instId);

	return (retCode);

}

/**
 * CPMAC DDC Init 
 *  - validates max TX/RX channels and stores initial configuration
 * 
 * \note Initial configuration passed by DDA via the "initCfg" parameter 
 */
LOCAL PAL_Result DDC_cpmacInit(CpmacDDCObj * hDDC, CpmacInitConfig * initCfg)
{

	LOGMSG(CPMAC_DEBUG_FUNCTION_ENTRY, "\n+ DDC_cpmacInit %d",
	       hDDC->ddcObj.instId);

	/* Validate numTx and numRx channels */
	if ((initCfg->numTxChannels > CPMAC_MAX_TX_CHANNELS) ||
	    (initCfg->numRxChannels > CPMAC_MAX_RX_CHANNELS)) {

		LOGERR
		    ("\nERROR:DDC: DDC_cpmacInit:%d: Invalid number of TX/RX channels",
		     hDDC->ddcObj.instId);

		return (CPMAC_INVALID_PARAM);

	}

	/* Save config info for later use */
	hDDC->initCfg = *initCfg;	/* Structure copy */

	LOGMSG(CPMAC_DEBUG_FUNCTION_EXIT, "\n- DDC_cpmacInit:%d:",
	       hDDC->ddcObj.instId);

	return (CPMAC_SUCCESS);

}

/* CPMAC DDC DeInit 
 * \note Stub function - no functionality required as per this implementation 
 */
LOCAL PAL_Result DDC_cpmacDeInit(CpmacDDCObj * hDDC, Ptr param)
{

	LOGMSG(CPMAC_DEBUG_FUNCTION_ENTRY,
	       "\n- DDC_cpmacDeInit %d : NULL FUNCTION", hDDC->ddcObj.instId);

	LOGMSG(CPMAC_DEBUG_FUNCTION_EXIT, "\n- DDC_cpmacDeInit:%d:",
	       hDDC->ddcObj.instId);

	return (CPMAC_SUCCESS);

}

/**
 * CPMAC DDC Open 
 *  - Brings module out of reset
 *  - Open's CSL, programs mandatory hardware init registers
 *  - Open's MII_MDIO module and enable poll timer via DDA
 *  - Enables earlier created TX/RX channels
 *  - Enables TX/RX operation in hardware
 * 
 * \note "param" not used in this implementation
 */
LOCAL PAL_Result DDC_cpmacOpen(CpmacDDCObj * hDDC, Ptr param)
{

	PAL_Result retCode;

	Uint32 channel;

	Uint32 miiModId, miiRevMaj, miiRevMin;

	PAL_Result retVal;

	CpmacInitConfig *initCfg;

	LOGMSG(CPMAC_DEBUG_FUNCTION_ENTRY, "\n+ DDC_cpmacOpen:%d:",
	       hDDC->ddcObj.instId);

	if (hDDC->ddcObj.state == DDC_OPENED) {

		LOGERR("\nERROR:DDC: DDC_cpmacOpen:%d: Device already open",
		       hDDC->ddcObj.instId);

		return (CPMAC_ERR_DEV_ALREADY_OPEN);

	}

	/* Get init config info structure pointer for easy access */
	initCfg = &hDDC->initCfg;

	hDDC->regs = (CpmacRegsOvly) initCfg->baseAddress;

	hDDC->eWrapRegs = (EwrapRegs *) initCfg->eWrapBaseAddress;

	/* Set the BD memory pointer */
	CpmacWrapperPtr = DAVINCI_CPMAC_WRAPPER_RAM_ADDR;

	/* Bring CPMAC out of reset - for clean implementation, reset and then unreset the module */
	/* For CPMAC 2.6 and beyond, reset is internal to the module */
	hDDC->regs->Soft_Reset = 1;

	while (hDDC->regs->Soft_Reset) {

		/* wait for reset to complete - do nothing */
	}

	/* Program TX/RX HDP's to 0 */
	for (channel = 0; channel < CPMAC_MAX_TX_CHANNELS; channel++) {

		hDDC->regs->Tx_HDP[channel] = 0;

		/* Initialize the completion pointers to 0 */
		hDDC->regs->Tx_CP[channel] = 0;

	}

	for (channel = 0; channel < CPMAC_MAX_RX_CHANNELS; channel++) {

		hDDC->regs->Rx_HDP[channel] = 0;

		/* Initialize the completion pointers to 0 */
		hDDC->regs->Rx_CP[channel] = 0;

	}

	/* Enable TX/RX DMA */
	hDDC->regs->Tx_Control |= CPMAC_TX_CONTROL_TX_ENABLE_VAL;

	hDDC->regs->Rx_Control |= CPMAC_RX_CONTROL_RX_ENABLE_VAL;

	/* Enable Adapter check interrupts - disable stats interupt */
	hDDC->regs->Mac_IntMask_Set = CPMAC_MAC_HOST_ERR_INTMASK_VAL;

	/* Set device state - Opened - useful when opening channels */
	hDDC->ddcObj.state = DDC_OPENED;

	/* Set the MacControl register */
	cpmacSetMacHwCfg(hDDC);

	/* Start MDIO Autonegotiation and set Phy mode */
	cpswHalCommonMiiMdioGetVer(initCfg->mdioBaseAddress, &miiModId,
				   &miiRevMaj, &miiRevMin);

	LOGMSG(CPMAC_DEBUG_PORT_UPDATE,
	       "\nDDC_cpmacOpen:%d: MII Module Id=%d, MII Base Address=%08X, Major Rev=%d, Minor Rev=%d",
	       hDDC->ddcObj.instId, miiModId, initCfg->mdioBaseAddress,
	       miiRevMaj, miiRevMin);

	/* Allocate memory for the PHY device and initialize the module */
	retCode =
	    PAL_osMemAlloc(0, cpswHalCommonMiiMdioGetPhyDevSize(), 0,
			   (Ptr *) & hDDC->PhyDev);

	if (retCode != PAL_SOK) {

		LOGERR
		    ("\nERROR:DDC: DDC_cpmacOpen:%d: Failed to allocate memory for Phy Device",
		     hDDC->ddcObj.instId);

		return (retCode);

	}

	/* \note No failure code returned from this function */
	cpswHalCommonMiiMdioInit(hDDC->PhyDev,
				 initCfg->mdioBaseAddress,
				 hDDC->ddcObj.instId,
				 initCfg->PhyMask, initCfg->MLinkMask, 0,
				 /* ResetBase param not required for this driver */
				 initCfg->mdioResetLine,	/* Reset line is also not required as this is handled outside this module */
				 initCfg->MdioBusFrequency,
				 initCfg->MdioClockFrequency,
				 (CpmacDDCDebug & CPMAC_DEBUG_MII),
				 /* previously HalDev->debug */
				 (OsPrintf) hDDC->ddaIf->ddaPrintf);	/* previously HalDev->OsFunc->Printf */

	/* Set the PHY to a given mode - as per config parameters and update DDA layer */
	cpmacSetPhyMode(hDDC);

	hDDC->ddaIf->ddaNetIf.ddaFuncTable.ddaControlCb(hDDC->ddcObj.hDDA,
							CPMAC_DDA_IOCTL_STATUS_UPDATE,
							(Ptr) & hDDC->status,
							NULL);

	/* Start the tick timer via DDA */
	hDDC->ddaIf->ddaNetIf.ddaFuncTable.ddaControlCb(hDDC->ddcObj.hDDA,
							CPMAC_DDA_IOCTL_TIMER_START,
							(Ptr) initCfg->
							MdioTickMSec, NULL);

	/* Enable Opened TX Channels */
	for (channel = 0; channel < CPMAC_MAX_TX_CHANNELS; channel++) {

		if (hDDC->txCppi[channel] != NULL) {

			retVal =
			    cpmacEnableChannel(hDDC, channel,
					       DDC_NET_CH_DIR_TX);

			if (retVal != CPMAC_SUCCESS) {

				LOGERR
				    ("\nERROR:DDC: DDC_cpmacOpen:%d: Error enabling TX channel %d",
				     hDDC->ddcObj.instId, channel);

				/* TODECIDE: Should we return from here or continue enabling other channels */
				return (retVal);

			}

		}

	}

	/* Set filter low threshold - Not supported, hence set to 0 */
	hDDC->regs->Rx_FilterLowThresh = 0;

	/* Disable unicast on all channels first - enabled if channel is configured & enabled below */
	hDDC->regs->Rx_Unicast_Clear = CPMAC_RX_UNICAST_CLEAR_ALL;

	/* Set MAC Hash register */
	hDDC->regs->MacHash1 = hDDC->MacHash1;

	hDDC->regs->MacHash2 = hDDC->MacHash2;

	/* RX MBP, RX pkt length and RX buffer offset registers taken care by this function */
	cpmacSetRxHwCfg(hDDC);

	/* Read RX Address matching/filtering Type (0/1/2) */
	hDDC->RxAddrType = (hDDC->regs->Mac_Cfig >> 8) & 0xFF;

	/* Enable Opened RX Channels */
	for (channel = 0; channel < CPMAC_MAX_RX_CHANNELS; channel++) {

		if (hDDC->rxCppi[channel] != NULL) {

			retVal =
			    cpmacEnableChannel(hDDC, channel,
					       DDC_NET_CH_DIR_RX);

			if (retVal != CPMAC_SUCCESS) {

				LOGERR
				    ("\nERROR:DDC: DDC_cpmacOpen:%d: Error enabling RX channel %d",
				     hDDC->ddcObj.instId, channel);

				/* TODECIDE: Should we return from here or continue enabling other channels */
				return (retVal);

			}

		}

		/* Since flow threshold and free buffer feature is not supported, set it to 0 */
		hDDC->regs->Rx_FlowThresh[channel] = 0;

		hDDC->regs->Rx_FreeBuffer[channel] = 0;

	}

	/* Finally Set MAC Control register, Enable MII */
	hDDC->MacControl |= (1 << CPMAC_MACCONTROL_MIIEN_SHIFT);

	hDDC->regs->MacControl = hDDC->MacControl;

	/* Start the MIB cnt tick timer via DDA */
	hDDC->ddaIf->ddaNetIf.ddaFuncTable.ddaControlCb(hDDC->ddcObj.hDDA,
							CPMAC_DDA_IOCTL_MIB64_CNT_TIMER_START,
							(Ptr) initCfg->
							Mib64CntMsec, NULL);

	/* Enable interrupts via Module control (wrapper) */
	hDDC->eWrapRegs->EWCTL = 0x1;

	LOGMSG(CPMAC_DEBUG_FUNCTION_EXIT, "\n- DDC_cpmacOpen:%d:",
	       hDDC->ddcObj.instId);

	return (CPMAC_SUCCESS);

}

/**
 * CPMAC DDC Close 
 *  - Disables poll timer via DDA
 *  - Disable and Close all open TX/RX channels
 *  - Closes CSL
 *  - Puts module in reset
 * 
 * \note "param" not used in this implementation
 */
LOCAL PAL_Result DDC_cpmacClose(CpmacDDCObj * hDDC, Ptr param)
{

	PAL_Result retVal;

	PAL_Result errVal = CPMAC_SUCCESS;

	Uint32 channel;

	LOGMSG(CPMAC_DEBUG_FUNCTION_ENTRY, "\n+ DDC_cpmacClose:%d:",
	       hDDC->ddcObj.instId);

	if (hDDC->ddcObj.state == DDC_CLOSED) {

		LOGERR("\nERROR:DDC: DDC_cpmacClose:%d: Device already closed",
		       hDDC->ddcObj.instId);

		return (CPMAC_ERR_DEV_ALREADY_CLOSED);

	}

	/* Stop the tick timer via DDA */
	hDDC->ddaIf->ddaNetIf.ddaFuncTable.ddaControlCb(hDDC->ddcObj.hDDA,
							CPMAC_DDA_IOCTL_TIMER_STOP,
							NULL, NULL);

	/* Stop the mib timer via DDA */
	hDDC->ddaIf->ddaNetIf.ddaFuncTable.ddaControlCb(hDDC->ddcObj.hDDA,
							CPMAC_DDA_IOCTL_MIB64_CNT_TIMER_STOP,
							NULL, NULL);

	/* Close TX Channels */
	for (channel = 0; channel < CPMAC_MAX_TX_CHANNELS; channel++) {

		if (hDDC->txCppi[channel] != NULL) {

			retVal =
			    DDC_cpmacChClose(hDDC, channel, DDC_NET_CH_DIR_TX,
					     NULL);

			if (retVal != CPMAC_SUCCESS) {

				LOGERR
				    ("\nERROR:DDC: DDC_cpmacOpen:%d: Error closing TX channel %d",
				     hDDC->ddcObj.instId, channel);

				/* \note Instead of returning immediatley on error, we close all possible channels - return (retVal); */
				errVal = retVal;

			}

		}

	}

	/* Close RX Channels */
	for (channel = 0; channel < CPMAC_MAX_RX_CHANNELS; channel++) {

		if (hDDC->rxCppi[channel] != NULL) {

			retVal =
			    DDC_cpmacChClose(hDDC, channel, DDC_NET_CH_DIR_RX,
					     NULL);

			if (retVal != CPMAC_SUCCESS) {

				LOGERR
				    ("\nERROR:DDC: DDC_cpmacOpen:%d: Error closing RX channel %d",
				     hDDC->ddcObj.instId, channel);

				/* \note Instead of returning immediatley on error, we close all possible channels */
				errVal = retVal;	/* return (retVal); */

			}

		}

	}

	/* Disable interrupts via Module control (wrapper) */
	hDDC->eWrapRegs->EWCTL = 0x0;

	/* Put CPMAC in reset */
	hDDC->regs->Soft_Reset = 1;

	/* Put MDIO in reset - not required for Davinci */

	LOGMSG(CPMAC_DEBUG_FUNCTION_EXIT, "\n- DDC_cpmacOpen:%d:",
	       hDDC->ddcObj.instId);

	if (errVal == CPMAC_SUCCESS) {	/* closed all channels successfully. Mark the DDC as closed */

		hDDC->ddcObj.state = DDC_CLOSED;

	}

	return (errVal);

}

/**
 * CPMAC DDC Ioctl 
 *  - Get Software (DDC) and Hardware Versions
 *  - Set/Modify RX and MAC configuration
 *  - Get DDC/module status
 *  - Read/Write MII registers (via PHY)
 *  - Get/Clr Statistics (hardware counters)
 *  - Add/Del/ Multicast operations AllMulti Set/Clear operations
 *  - Type2/3 Filtering operation
 * 
 * \note "param" not used in this implementation
 */
LOCAL PAL_Result DDC_cpmacControl(CpmacDDCObj * hDDC, Int cmd, Ptr cmdArg,
				  Ptr param)
{

	switch (cmd) {

	case CPMAC_DDC_IOCTL_GET_SWVER:

		/* cmdArg is an ptr to an integer that will contain the integer version id and 
		   param is a double ptr to a string which will point to the static version string */
		*((Uint32 *) cmdArg) =
		    (Uint32) ((CPMAC_DDC_MAJOR_VERSION << 16) |
			      CPMAC_DDC_MINOR_VERSION);

		*((Char **) param) = (Char *) & CpmacDDCVersionString[0];

		break;

	case CPMAC_DDC_IOCTL_GET_HWVER:

		/* Read hardware versions only if device is in open state */
		/* cmdArg is a ptr to an integer that will contain Tx Id ver 
		   and param is a pointer to an integer that will contain Rx Id ver after this call */
		if (hDDC->ddcObj.state == DDC_OPENED) {

			*((Uint32 *) cmdArg) = hDDC->regs->Tx_IdVer;

			*((Uint32 *) param) = hDDC->regs->Rx_IdVer;

		}

		else {
			return (CPMAC_ERR_DEV_NOT_OPEN);
		}

		break;

	case CPMAC_DDC_IOCTL_SET_RXCFG:

		/* Rx configuration structure passed in structure pointed by cmdArg, params not used */
		if (cmdArg != NULL) {

			hDDC->initCfg.rxCfg = *((CpmacRxConfig *) cmdArg);

			cpmacSetRxHwCfg(hDDC);	/* Set the RX_MBP, Maxlen, BufferOffset registers */

		}

		else {
			return (CPMAC_INVALID_PARAM);
		}

		break;

	case CPMAC_DDC_IOCTL_SET_MACCFG:

		/* Mac configuration structure passed in a structure pointed by cmdArg, params not used */
		if (cmdArg != NULL) {

			hDDC->initCfg.macCfg = *((CpmacMacConfig *) cmdArg);

			cpmacSetMacHwCfg(hDDC);	/* Set the MAC Control register */

		}

		else {
			return (CPMAC_INVALID_PARAM);
		}

		break;

	case CPMAC_DDC_IOCTL_GET_STATUS:

		/* Returns CpmacDDCStatus structure back in cmdArg pointer pointed structure */
		if (hDDC->ddcObj.state != DDC_OPENED) {

			LOGERR
			    ("\nWARN: DDC_cpmacControl:%d: DDC_CPMAC_IOCTL_GET_STATUS Ioctl called when device is NOT in open state",
			     hDDC->ddcObj.instId);

			return (CPMAC_ERR_DEV_NOT_OPEN);

		}

		{		/* Scope {} introduced here to declare status as local scope variable */

			CpmacDDCStatus *status = (CpmacDDCStatus *) cmdArg;

			*status = hDDC->status;	/* structure copy */

		}

		break;

	case CPMAC_DDC_IOCTL_READ_PHY_REG:

		/* cmdArg = pointer to CpmacPhyParams struct. data read back into "data" parameter in the structure */
		if (hDDC->ddcObj.state != DDC_OPENED) {

			LOGERR
			    ("\nWARN: DDC_cpmacControl:%d: CPMAC_DDC_IOCTL_READ_PHY_REG Ioctl called when device is NOT in open state",
			     hDDC->ddcObj.instId);

			return (CPMAC_ERR_DEV_NOT_OPEN);

		}

		{

			/* \warning: Read to the phy registers - Note that this code loops 
			   on a completion bit in the phy so there are chances of hanging" */
			CpmacPhyParams *phyParams = (CpmacPhyParams *) cmdArg;

			phyParams->data =
			    _cpswHalCommonMiiMdioUserAccessRead(hDDC->PhyDev,
								phyParams->
								regAddr,
								phyParams->
								phyNum);

		}

		break;

	case CPMAC_DDC_IOCTL_WRITE_PHY_REG:

		/* cmdArg = pointer to CpmacPhyParams struct. data to be written is in "data" parameter in the structure */
		if (hDDC->ddcObj.state != DDC_OPENED) {

			LOGERR
			    ("\nWARN: DDC_cpmacControl:%d: CPMAC_DDC_IOCTL_WRITE_PHY_REG Ioctl called when device is NOT in open state",
			     hDDC->ddcObj.instId);

			return (CPMAC_ERR_DEV_NOT_OPEN);

		}

		{

			CpmacPhyParams *phyParams = (CpmacPhyParams *) cmdArg;

			/* \warning: Write to the phy registers - Note that this code loops 
			   on a completion bit in the phy so there are chances of hanging" */
			_cpswHalCommonMiiMdioUserAccessWrite(hDDC->PhyDev,
							     phyParams->
							     regAddr,
							     phyParams->
							     phyNum,
							     phyParams->data);

		}

		break;

	case CPMAC_DDC_IOCTL_GET_STATISTICS:

		/* cmdArg points to the user provided structure for statistics which match with hardware 36 regs, param is not used */
		if (hDDC->ddcObj.state != DDC_OPENED) {

			LOGERR
			    ("\nWARN: DDC_cpmacControl:%d: CPMAC_DDC_IOCTL_GET_STATISTICS Ioctl called when device is NOT in open state",
			     hDDC->ddcObj.instId);

			return (CPMAC_ERR_DEV_NOT_OPEN);

		}

		{

			Uint32 cnt;

			Uint32 *userStats = (Uint32 *) cmdArg;

			volatile Uint32 *addr =
			    (Uint32 *) & hDDC->regs->RxGoodFrames;

			for (cnt = 0; cnt < CPMAC_NUM_STAT_REGS;
			     cnt++, userStats++, addr++) {

				*userStats = *addr;

			}

		}

		break;

	case CPMAC_DDC_IOCTL_CLR_STATISTICS:

		/* cmdArg or param is not used */
		if (hDDC->ddcObj.state != DDC_OPENED) {

			LOGERR
			    ("\nWARN: DDC_cpmacControl:%d: CPMAC_DDC_IOCTL_CLR_STATISTICS Ioctl called when device is NOT in open state",
			     hDDC->ddcObj.instId);

			return (CPMAC_ERR_DEV_NOT_OPEN);

		}

		{

			Uint32 cnt;

			volatile Uint32 *addr =
			    (Uint32 *) & hDDC->regs->RxGoodFrames;

			for (cnt = 0; cnt < CPMAC_NUM_STAT_REGS; cnt++, addr++) {

				*addr = CPMAC_STAT_CLEAR;	/* 0xFFFFFFFF value */

			}

			cpmacDDCIfcntClear(hDDC);

		}

		break;

	case CPMAC_DDC_IOCTL_MULTICAST_ADDR:

		/* cmdArg= CpmacMulticastOper enum, param = pointer to multicast address - Uint8 */
		if (hDDC->ddcObj.state != DDC_OPENED) {

			LOGERR
			    ("\nWARN: DDC_cpmacControl:%d: CPMAC_DDC_IOCTL_MULTICAST_ADDR Ioctl called when device is NOT in open state",
			     hDDC->ddcObj.instId);

			return (CPMAC_ERR_DEV_NOT_OPEN);

		}

		{

			Uint8 *addr = (Uint8 *) param;

			cpmacSingleMulti(hDDC, (CpmacSingleMultiOper) cmdArg,
					 addr);

		}

		break;

	case CPMAC_DDC_IOCTL_ALL_MULTI:

		/* cmdArg= CpmacAllMultiOper enum, param=not used */
		if (hDDC->ddcObj.state != DDC_OPENED) {

			LOGERR
			    ("\nWARN: DDC_cpmacControl:%d: CPMAC_DDC_IOCTL_MULTICAST_ADDR Ioctl called when device is NOT in open state",
			     hDDC->ddcObj.instId);

			return (CPMAC_ERR_DEV_NOT_OPEN);

		}

		cpmacAllMulti(hDDC, (CpmacAllMultiOper) cmdArg);

		break;

	case CPMAC_DDC_IOCTL_TYPE2_3_FILTERING:

		{

			/* cmdArg = Pointer to CpmacType2_3_AddrFilterParams structure, param=not used */
			CpmacType2_3_AddrFilterParams *addrParams;

			if (hDDC->ddcObj.state != DDC_OPENED) {

				LOGERR
				    ("\nWARN: DDC_cpmacControl:%d: CPMAC_DDC_IOCTL_TYPE2_3_FILTERING Ioctl called when device is NOT in open state",
				     hDDC->ddcObj.instId);

				return (CPMAC_ERR_DEV_NOT_OPEN);

			}

			addrParams = (CpmacType2_3_AddrFilterParams *) cmdArg;

			cpmacAddType2Addr(hDDC, addrParams->channel,
					  addrParams->macAddress,
					  addrParams->index,
					  addrParams->valid, addrParams->match);

		}

		break;

	case CPMAC_DDC_IOCTL_SET_MAC_ADDRESS:

		{

			/* cmdArg = Pointer to CpmacType2_3_AddrFilterParams structure, param=not used */
			CpmacAddressParams *addrParams;

			CpmacRxCppiCh *rxCppi;

			int cnt;

			if (hDDC->ddcObj.state != DDC_OPENED) {

				LOGERR
				    ("\nWARN: DDC_cpmacControl:%d: CPMAC_DDC_IOCTL_TYPE2_3_FILTERING Ioctl called when device is NOT in open state",
				     hDDC->ddcObj.instId);

				return (CPMAC_ERR_DEV_NOT_OPEN);

			}

			addrParams = (CpmacAddressParams *) cmdArg;

			rxCppi = hDDC->rxCppi[addrParams->channel];

			if (rxCppi == NULL) {

				LOGERR
				    ("\nERROR:DDC: cpmacEnableChannel:%d: Invalid Channel %d. RX CPPI structure NULL",
				     hDDC->ddcObj.instId, addrParams->channel);

				return (CPMAC_ERR_RX_CH_INVALID);

			}

			for (cnt = 0; cnt < 6; cnt++)

				rxCppi->macAddr[cnt] =
				    addrParams->macAddress[cnt];

			/* Set interface MAC address */
			cpmacSetMacAddress(hDDC, addrParams->channel,
					   addrParams->macAddress);

		}

		break;

	case CPMAC_DDC_IOCTL_IF_COUNTERS:

		if (hDDC->ddcObj.state != DDC_OPENED) {

			LOGERR
			    ("\nWARN: DDC_cpmacControl:%d: CPMAC_DDC_IOCTL_IF_COUNTERS Ioctl called when device is NOT in open state",
			     hDDC->ddcObj.instId);

			return (CPMAC_ERR_DEV_NOT_OPEN);

		}

		cpmacDDCIfcntUpdt(hDDC);

		PAL_osMemCopy((char *)cmdArg,
			      (char *)&hDDC->Mib2IfHCCounter.Mib2IfCounter,
			      sizeof(struct mib2_ifCounters));

		break;

	case CPMAC_DDC_IOCTL_ETHER_COUNTERS:

		if (hDDC->ddcObj.state != DDC_OPENED) {

			LOGERR
			    ("\nWARN: DDC_cpmacControl:%d: CPMAC_DDC_IOCTL_ETHER_COUNTERS Ioctl called when device is NOT in open state",
			     hDDC->ddcObj.instId);

			return (CPMAC_ERR_DEV_NOT_OPEN);

		}

		cpmacDDCPhycnt(hDDC, cmdArg);

		break;

	case CPMAC_DDC_IOCTL_IF_PARAMS_UPDT:

		if (hDDC->ddcObj.state != DDC_OPENED) {

			LOGERR
			    ("\nWARN: DDC_cpmacControl:%d: CPMAC_DDC_IOCTL_IF_PARAMS_UPDT Ioctl called when device is NOT in open state",
			     hDDC->ddcObj.instId);

			return (CPMAC_ERR_DEV_NOT_OPEN);

		}

		cpmacDDCIfcntUpdt(hDDC);

		break;

	default:

		LOGERR("\nWARN: DDC_cpmacControl:%d: Unhandled ioctl code %d",
		       hDDC->ddcObj.instId, cmd);

		break;

	}

	return (CPMAC_SUCCESS);

}

/************************ DDC NET DEVICE INTERFACE TABLE FUNCTIONS ************************/

/**
 * CPMAC DDC Channel Open
 *  - Verify channel info (range checking etc)
 *  - Allocate memory for the channel
 *  - Book-keep operations for the channel - ready to be enabled in hardware
 * 
 * \note 1. If DDC instance is in "Opened" state, the channel is enabled in hardware
 *       2. "chOpenArgs" is used only for opening RX channel
 */
LOCAL PAL_Result DDC_cpmacChOpen(CpmacDDCObj * hDDC, CpmacChInfo * chInfo,
				 Ptr chOpenArgs)
{

	PAL_Result retVal;

	LOGMSG(CPMAC_DEBUG_FUNCTION_ENTRY,
	       "\n+ DDC_cpmacChOpen:%d: ChannelNo=%d, Dir=%s",
	       hDDC->ddcObj.instId, chInfo->chNum,
	       ((chInfo->chDir == DDC_NET_CH_DIR_TX) ? "TX" : "RX"));

	/* If the channel state is not DDC_NET_CH_UNINITIALIZED, return error */
	if (chInfo->chState != DDC_NET_CH_UNINITIALIZED) {

		LOGERR
		    ("\nERROR:DDC: DDC_cpmacChOpen:%d: %s channel %d  should be in DDC_NET_CH_UNINITIALIZED state",
		     hDDC->ddcObj.instId,
		     ((chInfo->chDir == DDC_NET_CH_DIR_TX) ? "TX" : "RX"),
		     chInfo->chNum);

		return (CPMAC_INVALID_PARAM);

	}

	/* Init channel */
	if (chInfo->chDir == DDC_NET_CH_DIR_TX) {

		if (chInfo->chNum >= hDDC->initCfg.numTxChannels) {

			LOGERR
			    ("\nERROR:DDC: DDC_cpmacChOpen:%d: Invalid TX Channel=%d specified",
			     hDDC->ddcObj.instId, chInfo->chNum);

			return (CPMAC_ERR_TX_CH_INVALID);

		}

		if (hDDC->txIsCreated[chInfo->chNum] == True) {

			LOGERR
			    ("\nERROR:DDC: DDC_cpmacChOpen:%d: TX Channel %d already open",
			     hDDC->ddcObj.instId, chInfo->chNum);

			return (CPMAC_ERR_TX_CH_ALREADY_INIT);

		}

		/* Allocate channel memory and perform other book-keep functions for the channel */
		retVal = cpmacInitTxChannel(hDDC, chInfo, chOpenArgs);

		if (retVal != CPMAC_SUCCESS) {

			LOGERR
			    ("\nERROR:DDC: DDC_cpmacChOpen:%d: Error in initializing TX channel %d",
			     hDDC->ddcObj.instId, chInfo->chNum);

			return (retVal);

		}

	}

	else if (chInfo->chDir == DDC_NET_CH_DIR_RX) {

		if (chInfo->chNum >= hDDC->initCfg.numRxChannels) {

			LOGERR
			    ("\nERROR:DDC: DDC_cpmacChOpen:%d: Invalid RX Channel=%d specified",
			     hDDC->ddcObj.instId, chInfo->chNum);

			return (CPMAC_ERR_RX_CH_INVALID);

		}

		if (hDDC->rxIsCreated[chInfo->chNum] == True) {

			LOGERR
			    ("\nERROR:DDC: DDC_cpmacChOpen:%d: RX Channel %d already open",
			     hDDC->ddcObj.instId, chInfo->chNum);

			return (CPMAC_ERR_RX_CH_ALREADY_INIT);

		}

		/* Allocate channel memory and perform other book-keep functions for the channel */
		retVal = cpmacInitRxChannel(hDDC, chInfo, chOpenArgs);

		if (retVal != CPMAC_SUCCESS) {

			LOGERR
			    ("\nERROR:DDC: DDC_cpmacChOpen:%d: Error in initializing RX channel %d",
			     hDDC->ddcObj.instId, chInfo->chNum);

			return (retVal);

		}

	}

	/* If device is opened already, enable this channel for use */
	if (hDDC->ddcObj.state == DDC_OPENED) {

		retVal = cpmacEnableChannel(hDDC, chInfo->chNum, chInfo->chDir);

		if (retVal != CPMAC_SUCCESS) {

			LOGERR
			    ("\nERROR:DDC: DDC_cpmacChOpen:%d: Error enabling channel %d in %d direction",
			     hDDC->ddcObj.instId, chInfo->chNum, chInfo->chDir);

			return (retVal);

		}

	}

	LOGMSG(CPMAC_DEBUG_FUNCTION_EXIT,
	       "\n- DDC_cpmacChOpen:%d: ChannelNo=%d, Dir=%s",
	       hDDC->ddcObj.instId, chInfo->chNum,
	       ((chInfo->chDir == DDC_NET_CH_DIR_TX) ? "TX" : "RX"));

	return (CPMAC_SUCCESS);

}

/**
 * CPMAC DDC Channel Close
 *  - If DDC instance is in "Opened" state, disable the channel in hardware
 *  - Un-initialize the channel (free memory previously allocated)
 */
LOCAL PAL_Result DDC_cpmacChClose(CpmacDDCObj * hDDC, Int channel,
				  Int direction, Ptr chCloseArgs)
{

	PAL_Result retVal;

	LOGMSG(CPMAC_DEBUG_FUNCTION_ENTRY,
	       "\n+ DDC_cpmacChClose:%d: ChannelNo=%d, Dir=%s",
	       hDDC->ddcObj.instId, channel,
	       ((direction == DDC_NET_CH_DIR_TX) ? "TX" : "RX"));

	/* Disable this channel */
	if (hDDC->ddcObj.state == DDC_OPENED) {

		retVal = cpmacDisableChannel(hDDC, channel, direction);

		if (retVal != CPMAC_SUCCESS) {

			LOGERR
			    ("\nERROR:DDC: DDC_cpmacChClose:%d: Error disabling channel %d in %d direction",
			     hDDC->ddcObj.instId, channel,
			     ((direction == DDC_NET_CH_DIR_TX) ? "TX" : "RX"));

			return (retVal);

		}

	}

	/* UnInit channel */
	if (direction == DDC_NET_CH_DIR_TX) {

		retVal = cpmacUnInitTxChannel(hDDC, channel, chCloseArgs);

		if (retVal != CPMAC_SUCCESS) {

			LOGERR
			    ("\nERROR:DDC: DDC_cpmacChClose:%d: Error in UnInit of TX channel %d",
			     hDDC->ddcObj.instId, channel);

			return (retVal);

		}

	}

	else if (direction == DDC_NET_CH_DIR_RX) {

		retVal = cpmacUnInitRxChannel(hDDC, channel, chCloseArgs);

		if (retVal != CPMAC_SUCCESS) {

			LOGERR
			    ("\nERROR:DDC: DDC_cpmacChClose:%d: Error in UnInit of TX channel %d",
			     hDDC->ddcObj.instId, channel);

			return (retVal);

		}

	}

	LOGMSG(CPMAC_DEBUG_FUNCTION_EXIT,
	       "\n- DDC_cpmacChClose:%d: ChannelNo=%d, Dir=%s",
	       hDDC->ddcObj.instId, channel,
	       ((direction == DDC_NET_CH_DIR_TX) ? "TX" : "RX"));

	return (CPMAC_SUCCESS);

}

/**
 * Init Tx Channel
 *  - Allocates memory for TX Ch Control structure, Buffer descriptors 
 *  - Initialize the above data structures as per channel configuration
 *  - Chain the TX BD list ready to be given to hardware
 * 
 * \note 1. "chOpenArgs" not used in this implementation
 *       2. This function assumes that the channel number passed is valid and the
 *       hDDC->txCppi[channel] pointer is NULL. This function will not do any error 
 *       check on these parameters to avoid duplicate error checks (done in caller function).
 */
LOCAL PAL_Result cpmacInitTxChannel(CpmacDDCObj * hDDC,
				    CpmacChInfo * chInfo, Ptr chOpenArgs)
{

	PAL_Result retCode;

	Uint32 cnt, bdSize;

	Char *allocMem;

	CpmacTxBD *currBD;

	CpmacTxCppiCh *txCppi = NULL;

	LOGMSG(CPMAC_DEBUG_FUNCTION_ENTRY,
	       "\n+ cpmacInitTxChannel:%d: ChannelNo=%d",
	       hDDC->ddcObj.instId, chInfo->chNum);

	/* Allocate memory for TX CPPI Channel and set to 0 */
	retCode = PAL_osMemAlloc(0, sizeof(CpmacTxCppiCh), 0, (Ptr *) & txCppi);

	if (retCode != PAL_SOK) {

		LOGERR
		    ("\nERROR:DDC: cpmacInitTxChannel:%d: Failed to allocate memory for TX CPPI Channel %d",
		     hDDC->ddcObj.instId, chInfo->chNum);

		return (retCode);

	}

	PAL_osMemSet(txCppi, 0, sizeof(CpmacTxCppiCh));

	/* Update the channel control structure in DDC */
	hDDC->txCppi[chInfo->chNum] = txCppi;

	/* Populate channel info */
	txCppi->chInfo = *chInfo;	/* Structure copy */

	txCppi->chInfo.chState = DDC_NET_CH_INITIALIZED;

	txCppi->activeQueueHead = 0;

	txCppi->activeQueueTail = 0;

	txCppi->queueActive = False;

	hDDC->txTeardownPending[chInfo->chNum] = False;	/* Clear the TX teardown pending flag */

#ifdef CPMAC_MULTIPACKET_TX_COMPLETE_NOTIFY
	/* Allocate memory for TX CPPI Channel on a 4 byte boundry */
	retCode =
	    PAL_osMemAlloc(0, (chInfo->serviceMax * sizeof(Uint32)), 4,
			   (Ptr *) & txCppi->txComplete);

	if (retCode != PAL_SOK) {

		LOGERR
		    ("\nERROR:DDC: cpmacInitTxChannel:%d: Failed to allocate memory for TX Complete queue %d",
		     hDDC->ddcObj.instId, chInfo->chNum);

		/* free tx cppi channel memory */
		PAL_osMemFree(0, txCppi, sizeof(CpmacTxCppiCh));

		return (retCode);

	}

	/* Set memory to 0 */
	PAL_osMemSet(txCppi->txComplete, 0,
		     (chInfo->serviceMax * sizeof(Uint32)));

#endif				/* 
				 */

	/* Allocate buffer descriptor pool align every BD on four word boundry for future requirements */
	bdSize = (sizeof(CpmacTxBD) + 0xF) & ~0xF;

	txCppi->allocSize = (((bdSize * chInfo->numBD) + 0xF) & ~0xF);

	/* Allocate BD Pool memory on 4 word boundry - 16 bytes below */

	/* NOTE: For DaVinci TX BD memory is from Wrapper RAM and hence this is not required
	 * 
	 * retCode = PAL_osMemAlloc(0, txCppi->allocSize, 16, (Ptr *)&txCppi->bdMem);  
	 * if (retCode != PAL_SOK)
	 * {
	 *      LOGERR("\nERROR:DDC: cpmacInitTxChannel:%d: Failed to allocated %d bytes for BD pool memory for ch  %d", 
	 *          hDDC->ddcObj.instId, txCppi->allocSize, chInfo->chNum);
	 *      return (retCode);
	 * }
	 */

	/* Check Wrapper Memory pointer to see if we exceeded available memory */
	if (CpmacWrapperPtr >
	    (DAVINCI_CPMAC_WRAPPER_RAM_ADDR +
	     DAVINCI_CPMAC_WRAPPER_RAM_SIZE - 1)) {

		LOGERR
		    ("\nERROR:DDC: cpmacInitTxChannel:%d: Failed to allocated %d bytes for BD pool memory for ch  %d",
		     hDDC->ddcObj.instId, txCppi->allocSize, chInfo->chNum);

		return (retCode);

	}

	/* Alloc TX BD memory */
	txCppi->bdMem = (Char *) DAVINCI_CPMAC_TX_BD_MEM;

	/* Set memory to 0 */
	PAL_osMemSet(txCppi->bdMem, 0, txCppi->allocSize);

	/* Initialize the BD linked list */
	allocMem = (char *)(((Uint32) txCppi->bdMem + 0xF) & ~0xF);

	txCppi->bdPoolHead = 0;

	for (cnt = 0; cnt < chInfo->numBD; cnt++) {

		currBD = (CpmacTxBD *) (allocMem + (cnt * bdSize));

		currBD->next = txCppi->bdPoolHead;

		txCppi->bdPoolHead = currBD;

	}

	/* Reset Statistics counters */
	txCppi->outOfTxBD = 0;

	txCppi->noActivePkts = 0;

	txCppi->activeQueueCount = 0;

	hDDC->txIsCreated[chInfo->chNum] = True;

	LOGMSG(CPMAC_DEBUG_FUNCTION_EXIT,
	       "\n- cpmacInitTxChannel:%d: ChannelNo=%d",
	       hDDC->ddcObj.instId, chInfo->chNum);

	return (CPMAC_SUCCESS);

}

/**
 * Un-Init Tx Channel
 *  - Frees memory previously allocated for Ch Control structure, Buffer descriptors 
 * 
 * \note 1. "chCloseArgs" not used in this implementation
 *       2. This function assumes that the channel number passed is valid and this function will 
 *          not do any error check to avoid duplicate error checks (done in caller function).
 */
LOCAL PAL_Result cpmacUnInitTxChannel(CpmacDDCObj * hDDC, Uint32 channel,
				      Ptr chCloseArgs)
{

	PAL_Result retCode;

	CpmacTxCppiCh *txCppi;

	LOGMSG(CPMAC_DEBUG_FUNCTION_ENTRY,
	       "\n+ cpmacUnInitTxChannel:%d: ChannelNo=%d",
	       hDDC->ddcObj.instId, channel);

	/* Check if channel structure is already de-allocated */
	if (hDDC->txIsCreated[channel] == False) {

		LOGERR
		    ("\nERROR:DDC: cpmacUnInitTxChannel:%d: TX CPPI Channel %d structure already freed",
		     hDDC->ddcObj.instId, channel);

		return (CPMAC_ERR_TX_CH_ALREADY_CLOSED);

	}

	txCppi = hDDC->txCppi[channel];

	/* Free the buffer descriptors memory */
	if (txCppi->bdMem != NULL) {

		/* NOTE: For DaVinci TX BD memory is from Wrapper RAM and hence this is not required
		 * 
		 *   retCode = PAL_osMemFree(0, txCppi->bdMem, txCppi->allocSize);
		 *   if (retCode != PAL_SOK)
		 *   {
		 *       LOGERR("\nERROR:DDC: cpmacUnInitTxChannel:%d: Failed to free memory for BD Pool for Ch %d",
		 *           hDDC->ddcObj.instId, channel );
		 *   }
		 */
		txCppi->bdMem = NULL;

	}
#ifdef CPMAC_MULTIPACKET_TX_COMPLETE_NOTIFY
	/* Free the TX complete queue */
	retCode =
	    PAL_osMemFree(0, txCppi->txComplete,
			  (txCppi->chInfo.serviceMax * sizeof(Uint32)));

	if (retCode != PAL_SOK) {

		LOGERR
		    ("\nERROR:DDC: cpmacUnInitTxChannel:%d: Failed to free TX complete queue for Ch %d",
		     hDDC->ddcObj.instId, channel);

	}
#endif				/* 
				 */

	/* Free the TX Channel structure */
	retCode = PAL_osMemFree(0, txCppi, sizeof(CpmacTxCppiCh));

	if (retCode != PAL_SOK) {

		LOGERR
		    ("\nERROR:DDC: cpmacUnInitTxChannel:%d: Failed to free TX CPPI channel structure for Ch %d",
		     hDDC->ddcObj.instId, channel);

	}

	hDDC->txCppi[channel] = NULL;

	hDDC->txIsCreated[channel] = False;

	LOGMSG(CPMAC_DEBUG_FUNCTION_EXIT,
	       "\n- cpmacUnInitTxChannel:%d: ChannelNo=%d",
	       hDDC->ddcObj.instId, channel);

	return (CPMAC_SUCCESS);

}

/**
 * Init Rx Channel
 *  - Allocates memory for RX Ch Control structure, Buffer descriptors 
 *  - Initialize the above data structures as per channel configuration
 *  - Allocate receive buffers from DDA and chain the RX BD list ready to be given to hardware
 * 
 * \note 1. "chOpenArgs" Points to MAC address for this channel
 *       2. This function assumes that the channel number passed is valid and the
 *       hDDC->rxCppi[channel] pointer is NULL. This function will not do any error 
 *       check on these parameters to avoid duplicate error checks (done in caller function).
 */
LOCAL PAL_Result cpmacInitRxChannel(CpmacDDCObj * hDDC,
				    CpmacChInfo * chInfo, Ptr chOpenArgs)
{

	PAL_Result retCode;

	Uint32 cnt, bdSize;

	Char *allocMem;

	CpmacRxBD *currBD;

	CpmacRxCppiCh *rxCppi = NULL;

	LOGMSG(CPMAC_DEBUG_FUNCTION_ENTRY,
	       "\n+ cpmacInitRxChannel:%d: ChannelNo=%d",
	       hDDC->ddcObj.instId, chInfo->chNum);

	/* Allocate memory for RX CPPI Channel */
	retCode = PAL_osMemAlloc(0, sizeof(CpmacRxCppiCh), 0, (Ptr *) & rxCppi);

	if (retCode != PAL_SOK) {

		LOGERR
		    ("\nERROR:DDC: cpmacInitRxChannel:%d: Failed to allocate memory for RX CPPI Channel %d",
		     hDDC->ddcObj.instId, chInfo->chNum);

		return (retCode);

	}

	/* Set memory to 0 */
	PAL_osMemSet(rxCppi, 0, sizeof(CpmacRxCppiCh));

	/* Update the channel control structure in DDC */
	hDDC->rxCppi[chInfo->chNum] = rxCppi;

	/* Populate channel info */
	rxCppi->hDDC = hDDC;

	rxCppi->chInfo = *chInfo;	/* Structure copy */

	rxCppi->chInfo.chState = DDC_NET_CH_INITIALIZED;

	hDDC->rxTeardownPending[chInfo->chNum] = False;	/* Clear the RX teardown pending flag */

	/* Save mac address */
	allocMem = (Char *) chOpenArgs;	/* reusing allocMem local variable as char pointer */

	for (cnt = 0; cnt < 6; cnt++)

		rxCppi->macAddr[cnt] = allocMem[cnt];

	/* Allocate buffer descriptor pool align every BD on four word boundry for future requirements */
	bdSize = (sizeof(CpmacRxBD) + 0xF) & ~0xF;

	rxCppi->allocSize = (((bdSize * chInfo->numBD) + 0xF) & ~0xF);

	/* Allocate BD Pool on 4 word boundry - 16 bytes below */

	/* NOTE: For DaVinci RX BD memory is from Wrapper RAM and hence this is not required
	 * 
	 * retCode = PAL_osMemAlloc(0, rxCppi->allocSize, 16, (Ptr *)&rxCppi->bdMem);  
	 * if (retCode != PAL_SOK)
	 * {
	 *      LOGERR("\nERROR:DDC: cpmacInitRxChannel:%d: Failed to allocated %d bytes for BD pool memory for ch  %d", 
	 *          hDDC->ddcObj.instId, rxCppi->allocSize, chInfo->chNum);
	 *      return (retCode);
	 * }
	 */

	/* Check Wrapper Memory pointer to see if we exceeded available memory */
	if (CpmacWrapperPtr >
	    (DAVINCI_CPMAC_WRAPPER_RAM_ADDR +
	     DAVINCI_CPMAC_WRAPPER_RAM_SIZE - 1)) {

		LOGERR
		    ("\nERROR:DDC: cpmacInitRxChannel:%d: Failed to allocated %d bytes for BD pool memory for ch  %d",
		     hDDC->ddcObj.instId, rxCppi->allocSize, chInfo->chNum);

		return (retCode);

	}

	/* Alloc RX BD memory */
	rxCppi->bdMem = (Char *) DAVINCI_CPMAC_RX_BD_MEM;

	/* Set memory to 0 */
	PAL_osMemSet(rxCppi->bdMem, 0, rxCppi->allocSize);

#ifdef CPMAC_MULTIPACKET_RX_COMPLETE_NOTIFY
	/* Allocate memory for packet queue on a 4 byte boundry and set to 0 */
	retCode =
	    PAL_osMemAlloc(0, (chInfo->serviceMax * sizeof(DDC_NetPktObj)), 4,
			   (Ptr *) & rxCppi->pktQueue);

	if (retCode != PAL_SOK) {

		LOGERR
		    ("\nERROR:DDC: cpmacInitRxChannel:%d: Failed to allocate memory for RX packet queue %d",
		     hDDC->ddcObj.instId, chInfo->chNum);

		/* Free tx cppi channel memory */
		PAL_osMemFree(0, rxCppi, sizeof(CpmacRxCppiCh));

		return (retCode);

	}

	PAL_osMemSet(rxCppi->pktQueue, 0,
		     (chInfo->serviceMax * sizeof(DDC_NetPktObj)));

	/* Allocate memory for buffer queue on a 4 byte boundry and set to 0 */
	retCode =
	    PAL_osMemAlloc(0,
			   (chInfo->serviceMax * sizeof(DDC_NetBufObj) *
			    CPMAC_MAX_RX_FRAGMENTS), 4,
			   (Ptr *) & rxCppi->bufQueue);

	if (retCode != PAL_SOK) {

		LOGERR
		    ("\nERROR:DDC: cpmacInitRxChannel:%d: Failed to allocate memory for RX buffer queue %d",
		     hDDC->ddcObj.instId, chInfo->chNum);

		/* Free pkt queue */
		PAL_osMemFree(0, rxCppi,
			      (chInfo->serviceMax * sizeof(DDC_NetPktObj)));

		/* Free tx cppi channel memory */
		PAL_osMemFree(0, rxCppi, sizeof(CpmacRxCppiCh));

		return (retCode);

	}

	PAL_osMemSet(rxCppi->bufQueue, 0,
		     (chInfo->serviceMax * sizeof(DDC_NetBufObj) *
		      CPMAC_MAX_RX_FRAGMENTS));

	/* Build the packet-buffer structures */
	{

		DDC_NetPktObj *currPkt = &rxCppi->pktQueue[0];

		DDC_NetBufObj *currBuf = &rxCppi->bufQueue[0];

		/* Bind pkt and buffer queue data structures */
		for (cnt = 0; cnt < chInfo->serviceMax; cnt++) {

			currPkt->bufList = currBuf;

			++currPkt;

			currBuf += CPMAC_MAX_RX_FRAGMENTS;

			/*currBuf = (DDC_NetBufObj *) ( ((Uint32)currBuf) +  (sizeof(DDC_NetBufObj) * CPMAC_MAX_RX_FRAGMENTS) ); */
		}

	}

#else				/* 
				 */
	rxCppi->pktQueue.bufList = &rxCppi->bufQueue[0];

#endif				/* 
				 */

	/* Allocate RX buffer and initialize the BD linked list */
	allocMem = (char *)(((Uint32) rxCppi->bdMem + 0xF) & ~0xF);

	rxCppi->activeQueueHead = 0;

	rxCppi->activeQueueTail = (CpmacRxBD *) allocMem;

	for (cnt = 0; cnt < chInfo->numBD; cnt++) {

		currBD = (CpmacRxBD *) (allocMem + (cnt * bdSize));

		/* for potential future use the last parameter contains the BD ptr */
		currBD->dataPtr =
		    (Ptr) (hDDC->ddaIf->ddaNetIf.
			   ddaNetAllocRxBufCb(hDDC->ddcObj.hDDA,
					      chInfo->bufSize,
					      (DDC_NetDataToken *) & currBD->
					      bufToken, (Ptr) currBD));

		if (currBD->dataPtr == NULL) {

			LOGERR
			    ("\nERROR:DDC: cpmacInitRxChannel: Error in RX Buffer allocation for channel %d",
			     chInfo->chNum);

			return (CPMAC_ERR_RX_BUFFER_ALLOC_FAIL);

		}

		/* Populate the hardware descriptor */
		currBD->hNext = PAL_CPMAC_VIRT_2_PHYS(rxCppi->activeQueueHead);

		currBD->buffPtr = PAL_CPMAC_VIRT_2_PHYS(currBD->dataPtr);

		currBD->off_bLen = chInfo->bufSize;

		currBD->mode = CPMAC_CPPI_OWNERSHIP_BIT;

		/* Write back to hardware memory */
		PAL_CPMAC_CACHE_WRITEBACK_INVALIDATE((Uint32) currBD,
						     CPMAC_BD_LENGTH_FOR_CACHE);

		currBD->next = (Ptr) rxCppi->activeQueueHead;

		rxCppi->activeQueueHead = currBD;

	}

	/* At this point rxCppi->activeQueueHead points to the first RX BD ready to be given to RX HDP
	   and rxCppi->activeQueueTail points to the last RX BD */

	hDDC->rxIsCreated[chInfo->chNum] = True;

	LOGMSG(CPMAC_DEBUG_FUNCTION_EXIT,
	       "\n- cpmacInitRxChannel:%d: ChannelNo=%d",
	       hDDC->ddcObj.instId, chInfo->chNum);

	return (CPMAC_SUCCESS);

}

/**
 * Un-Init Rx Channel
 *  - Frees memory previously allocated for Ch Control structure, Buffer descriptors 
 *  - Returns (Frees) back receive buffers to DDA layer
 * 
 * \note 1. "chCloseArgs" not used in this implementation
 *       2. This function assumes that the channel number passed is valid and this function will 
 *          not do any error check to avoid duplicate error checks (done in caller function).
 */
LOCAL PAL_Result cpmacUnInitRxChannel(CpmacDDCObj * hDDC, Uint32 channel,
				      Ptr chCloseArgs)
{

	PAL_Result retCode;

	CpmacRxCppiCh *rxCppi;

	CpmacRxBD *currBD;

	LOGMSG(CPMAC_DEBUG_FUNCTION_ENTRY,
	       "\n+ cpmacUnInitRxChannel:%d: ChannelNo=%d",
	       hDDC->ddcObj.instId, channel);

	/* Check if channel structure is already de-allocated */
	if (hDDC->rxIsCreated[channel] == False) {

		LOGERR
		    ("\nERROR:DDC: cpmacUnInitRxChannel:%d: RX CPPI Channel %d structure already freed",
		     hDDC->ddcObj.instId, channel);

		return (CPMAC_ERR_RX_CH_ALREADY_CLOSED);

	}

	rxCppi = hDDC->rxCppi[channel];

	/* Free the Receive buffers previously allocated */
	currBD = rxCppi->activeQueueHead;

	while (currBD) {

		if (hDDC->ddaIf->ddaNetIf.
		    ddaNetFreeRxBufCb(hDDC->ddcObj.hDDA, currBD->dataPtr,
				      (DDC_NetDataToken) currBD->bufToken,
				      NULL) != CPMAC_SUCCESS) {

			LOGERR
			    ("\nERROR:DDC: cpmacUnInitRxChannel:%d: Failed to free RX buffer Ch %d",
			     hDDC->ddcObj.instId, channel);

		}

		currBD = currBD->next;

	}

	/* Free the buffer descriptors memory */
	if (rxCppi->bdMem != NULL) {

		/* NOTE: For DaVinci RX BD memory is from Wrapper RAM and hence this is not required
		 * 
		 *   retCode = PAL_osMemFree(0, rxCppi->bdMem, rxCppi->allocSize);
		 *   if (retCode != PAL_SOK)
		 *   {
		 *       LOGERR("\nERROR:DDC: cpmacUnInitRxChannel:%d: Failed to free memory for BD Pool for Ch %d",
		 *           hDDC->ddcObj.instId, channel );
		 *   }
		 */
		rxCppi->bdMem = NULL;

	}

	/* Free the RX Channel structure */
	retCode = PAL_osMemFree(0, rxCppi, sizeof(CpmacRxCppiCh));

	if (retCode != PAL_SOK) {

		LOGERR
		    ("\nERROR:DDC: cpmacUnInitRxChannel:%d: Failed to free RX CPPI channel structure for Ch %d",
		     hDDC->ddcObj.instId, channel);

	}

	hDDC->rxCppi[channel] = NULL;

	hDDC->rxIsCreated[channel] = False;

	LOGMSG(CPMAC_DEBUG_FUNCTION_EXIT,
	       "\n- cpmacUnInitRxChannel:%d: ChannelNo=%d",
	       hDDC->ddcObj.instId, channel);

	return (CPMAC_SUCCESS);

}

/**
 * Set CPMAC Mac address 
 * Functionality provided:
 *  - CPMAC address is set in the hardware based on the address type 
 * 
 * \note 1. It is assumed that the channel is already "initialized"
  */
LOCAL void cpmacSetMacAddress(CpmacDDCObj * hDDC, Uint32 channel,
			      String macAddr)
{

	/* Enable Unicast on this channel */
	hDDC->regs->Rx_Unicast_Set = (1 << channel);

	/* Program MAC address for the channel depending upon cpmac/cpgmac */
	if (hDDC->RxAddrType == RX_ADDR_TYPE0)

		cpmacAddType0Addr(hDDC, channel, macAddr);

	else if (hDDC->RxAddrType == RX_ADDR_TYPE1)

		cpmacAddType1Addr(hDDC, channel, macAddr);

	else if (hDDC->RxAddrType == RX_ADDR_TYPE2)

		cpmacAddType2Addr(hDDC, channel, macAddr, 0, 1, 1);

	else

		LOGERR
		    ("\nWARN: cpmacEnableChannel:%d: Wrong Rx Addressing Type - (Type2) detected in hardware",
		     hDDC->ddcObj.instId);

}

/**
 * Enable TX/RX Channel
 * Functionality provided:
 *  - Channel is enabled in hardware. Data transfer can occur on this channel after this.
 * 
 * \note 1. It is assumed that the channel is already "initialized"
 *       2. To enable a channel after its disabled, it needs to be initialized again
 */
LOCAL PAL_Result cpmacEnableChannel(CpmacDDCObj * hDDC, Uint32 channel,
				    Uint32 direction)
{

	LOGMSG(CPMAC_DEBUG_FUNCTION_ENTRY,
	       "\n+ cpmacEnableChannel:%d: ChannelNo=%d, Direction=%s",
	       hDDC->ddcObj.instId, channel,
	       ((direction == DDC_NET_CH_DIR_TX) ? "TX" : "RX"));

	if (direction == DDC_NET_CH_DIR_TX) {

		CpmacTxCppiCh *txCppi;

		txCppi = hDDC->txCppi[channel];

		if (txCppi == NULL) {

			LOGERR
			    ("\nERROR:DDC: cpmacEnableChannel:%d: Invalid Channel %d. TX CPPI structure NULL",
			     hDDC->ddcObj.instId, channel);

			return (CPMAC_ERR_TX_CH_INVALID);

		}

		/* Init Head Descriptor pointer */
		hDDC->regs->Tx_HDP[channel] = 0;

		{

			CpmacMacConfig *macCfg;

			macCfg = &hDDC->initCfg.macCfg;

			if (macCfg->txInterruptDisable == True) {

				/* Disable Channel interrupt */
				hDDC->regs->Tx_IntMask_Clear = (1 << channel);

				hDDC->txInterruptDisable = True;

				hDDC->txIntThreshold[channel] =
				    hDDC->txCppi[channel]->chInfo.serviceMax;

			}

			else {

				/* Enable Channel interrupt */
				hDDC->regs->Tx_IntMask_Set = (1 << channel);

				hDDC->txInterruptDisable = False;

			}

		}

		/* Mark channel open */
		hDDC->txIsOpen[channel] = True;

		txCppi->chInfo.chState = DDC_NET_CH_OPENED;

	}

	else if (direction == DDC_NET_CH_DIR_RX) {

		CpmacRxCppiCh *rxCppi;

		rxCppi = hDDC->rxCppi[channel];

		if (rxCppi == NULL) {

			LOGERR
			    ("\nERROR:DDC: cpmacEnableChannel:%d: Invalid Channel %d. RX CPPI structure NULL",
			     hDDC->ddcObj.instId, channel);

			return (CPMAC_ERR_RX_CH_INVALID);

		}

		/* Set interface MAC address */
		cpmacSetMacAddress(hDDC, channel, rxCppi->macAddr);

		/* Enable Channel Interrupt */
		hDDC->regs->Rx_IntMask_Set = (1 << channel);

		/* Mark queue active */
		rxCppi->queueActive = True;

		/* Enable DMA */
		hDDC->regs->Rx_HDP[channel] =
		    PAL_CPMAC_VIRT_2_PHYS(rxCppi->activeQueueHead);

		/* Mark channel open */
		hDDC->rxIsOpen[channel] = True;

		rxCppi->chInfo.chState = DDC_NET_CH_OPENED;

	}

	LOGMSG(CPMAC_DEBUG_FUNCTION_EXIT,
	       "\n- cpmacEnableChannel:%d: ChannelNo=%d, Direction=%s",
	       hDDC->ddcObj.instId, channel,
	       ((direction == DDC_NET_CH_DIR_TX) ? "TX" : "RX"));

	return (CPMAC_SUCCESS);

}

/**
 * Disable TX/RX Channel
 * Functionality provided:
 *  - Channel is disabled in hardware. No data transfer can occur on this channel after this.
 * 
 * \note 1. It is assumed that the channel number passed is valid
 *       2. Resources for the channel will be released only when its closed
 */
LOCAL PAL_Result cpmacDisableChannel(CpmacDDCObj * hDDC, Uint32 channel,
				     DDC_NetChDir direction)
{

	LOGMSG(CPMAC_DEBUG_FUNCTION_ENTRY,
	       "\n+ cpmacDisableChannel:%d: ChannelNo=%d, Direction=%s",
	       hDDC->ddcObj.instId, channel,
	       ((direction == DDC_NET_CH_DIR_TX) ? "TX" : "RX"));

	if (direction == DDC_NET_CH_DIR_TX) {

		hDDC->txTeardownPending[channel] = True;	/* Set the TX teardown pending flag */

		/* Initiate teardown of TX channel */
		hDDC->regs->Tx_Teardown = channel;

		/* Wait for teardown complete */
		if (cpmacWaitForTeardownComplete
		    (hDDC, channel, direction, True) != CPMAC_SUCCESS) {

			LOGERR
			    ("\nERROR:DDC: cpmacDisableChannel:%d: Failed to teardown TX channel %d",
			     hDDC->ddcObj.instId, channel);

			/* Instead of quitting on error immediately, we continue so as to cleanup the channel */
		}

		hDDC->txTeardownPending[channel] = False;	/* Clear the TX teardown pending flag */

		/* Disable Interrupt */
		hDDC->regs->Tx_IntMask_Clear = (1 << channel);

		/* Disable DMA */

		/* Mark channel closed */
		hDDC->txIsOpen[channel] = False;

	}

	else if (direction == DDC_NET_CH_DIR_RX) {

		hDDC->rxTeardownPending[channel] = True;	/* Set the RX teardown pending flag */

		/* Initiate teardown of TX channel */
		hDDC->regs->Rx_Teardown = channel;

		/* Wait for teardown complete */
		if (cpmacWaitForTeardownComplete
		    (hDDC, channel, direction, True) != CPMAC_SUCCESS) {

			LOGERR
			    ("\nERROR:DDC: cpmacDisableChannel:%d: Failed to teardown RX channel %d",
			     hDDC->ddcObj.instId, channel);

		}

		hDDC->rxTeardownPending[channel] = False;	/* Clear the RX teardown pending flag */

		/* Disable Interrupt */
		hDDC->regs->Rx_IntMask_Clear = (1 << channel);

		/* Mark channel closed */
		hDDC->rxIsOpen[channel] = False;

	}

	LOGMSG(CPMAC_DEBUG_FUNCTION_EXIT,
	       "\n- cpmacDisableChannel:%d: ChannelNo=%d, Direction=%s",
	       hDDC->ddcObj.instId, channel,
	       ((direction == DDC_NET_CH_DIR_TX) ? "TX" : "RX"));

	return (CPMAC_SUCCESS);

}

/**
 * Wait for Teardown Complete
 *  - This function waits (blocking mode) for teardown completion.
 *  - blocking = True ( waits on OS timer wait untill teardown complete), 
 *             = False (returns immediately) - NOT SUPPORTED
 * As of now this function supports blocking mode in polled mode only
 */
LOCAL PAL_Result cpmacWaitForTeardownComplete(CpmacDDCObj * hDDC,
					      Uint32 channel,
					      DDC_NetChDir direction,
					      Bool blocking)
{

	volatile unsigned int teardown_cnt = 0xFFFFFFF0;

	if (direction == DDC_NET_CH_DIR_TX) {

		CpmacTxBD *currBD;

		CpmacTxCppiCh *txCppi;

		LOGMSG(CPMAC_DEBUG_FUNCTION_ENTRY,
		       "\nENTRY: cpmacWaitForTeardownComplete:%d: ChannelNo=%d, Direction=%s",
		       hDDC->ddcObj.instId, channel,
		       ((direction == DDC_NET_CH_DIR_TX) ? "TX" : "RX"));

		while ((hDDC->regs->Tx_CP[channel] & CPMAC_TEARDOWN_VALUE) !=
		       CPMAC_TEARDOWN_VALUE) {

			/* Wait here for Tx teardown completion interrupt to occur */

			/* A task delay can be called here to pend rather than occupying 
			 * CPU cycles - anyway it has been found that the teardown takes 
			 * very few cpu cycles and does not affect functionality 
			 */
			--teardown_cnt;

			if (teardown_cnt) {

				printk("Tx teardown aborted\n");

				break;

			}

		}

		/* Write to the completion pointer */
		hDDC->regs->Tx_CP[channel] = CPMAC_TEARDOWN_VALUE;

		/* TX teardown complete - process sent packets and return sent packets to DDA */
		txCppi = hDDC->txCppi[channel];

		if (txCppi->queueActive == True) {

			currBD = txCppi->activeQueueHead;

			while (currBD != NULL) {

				hDDC->ddaIf->ddaNetIf.ddaNettxCompleteCb(hDDC->
									 ddcObj.
									 hDDA,
									 &
									 (currBD->
									  bufToken),
									 1,
									 (Ptr)
									 channel);

				if (currBD != txCppi->activeQueueTail) {

					currBD = currBD->next;

				} else {

					break;

				}

			}

			txCppi->bdPoolHead = txCppi->activeQueueHead;

			txCppi->activeQueueHead = txCppi->activeQueueTail = 0;

		}

		/* At this stage all TX BD's are available linked with "bdPoolHead" and can be freed */
		LOGMSG(CPMAC_DEBUG_FUNCTION_EXIT,
		       "\nEXIT: cpmacWaitForTeardownComplete:%d: ChannelNo=%d, Direction=%s",
		       hDDC->ddcObj.instId, channel,
		       ((direction == DDC_NET_CH_DIR_TX) ? "TX" : "RX"));

	}

	else if (direction == DDC_NET_CH_DIR_RX) {

		LOGMSG(CPMAC_DEBUG_FUNCTION_ENTRY,
		       "\nENTRY: cpmacWaitForTeardownComplete:%d: ChannelNo=%d, Direction=%s",
		       hDDC->ddcObj.instId, channel,
		       ((direction == DDC_NET_CH_DIR_TX) ? "TX" : "RX"));

		while ((hDDC->regs->Rx_CP[channel] & CPMAC_TEARDOWN_VALUE) !=
		       CPMAC_TEARDOWN_VALUE) {

			/* Wait here for Rx teardown completion interrupt to occur */

			/* A task delay can be called here to pend rather than occupying 
			 * CPU cycles - anyway it has been found that the teardown takes 
			 * very few cpu cycles and does not affect functionality 
			 */
			--teardown_cnt;

			if (teardown_cnt) {

				printk("Rx teardown aborted\n");

				break;

			}

		}

		/* Write to the completion pointer */
		hDDC->regs->Rx_CP[channel] = CPMAC_TEARDOWN_VALUE;

		/* At this stage all TX BD's are available linked with "activeQueueHead" and can be freed */
		LOGMSG(CPMAC_DEBUG_FUNCTION_EXIT,
		       "\nEXIT: cpmacWaitForTeardownComplete:%d: ChannelNo=%d, Direction=%s",
		       hDDC->ddcObj.instId, channel,
		       ((direction == DDC_NET_CH_DIR_TX) ? "TX" : "RX"));

	}

	return (CPMAC_SUCCESS);

}

LOCAL void cpmacDDCPhycnt(CpmacDDCObj * hDDC, Uint32 * cmdArg)
{

	int result;

	CpmacHwStatistics stats;

	struct mib2_phyCounters *mib2PhyCounters =
	    (struct mib2_phyCounters *)cmdArg;

	result =
	    DDC_cpmacControl(hDDC, CPMAC_DDC_IOCTL_GET_STATISTICS,
			     (Uint32 *) & stats, NULL);

	if (result != 0) {

		LOGERR
		    ("\ncpmacStats: Error from ioctl for DDC CPMAC_DDC_IOCTL_GET_STATISTICS \n");

		return;

	}

	mib2PhyCounters->ethAlignmentErrors = stats.ifInAlignCodeErrors;

	mib2PhyCounters->ethFCSErrors = stats.ifInCRCErrors;

	mib2PhyCounters->ethSingleCollisions = stats.ifSingleCollisionFrames;

	mib2PhyCounters->ethMultipleCollisions =
	    stats.ifMultipleCollisionFrames;

	mib2PhyCounters->ethSQETestErrors = 0;	/*Hardware doesn't support this */

	mib2PhyCounters->ethDeferredTxFrames = stats.ifDeferredTransmissions;

	mib2PhyCounters->ethLateCollisions = stats.ifLateCollisions;

	mib2PhyCounters->ethExcessiveCollisions =
	    stats.ifExcessiveCollisionFrames;

	mib2PhyCounters->ethInternalMacTxErrors = 0;	/*Hardware doesn't support this */

	mib2PhyCounters->ethCarrierSenseErrors = stats.ifCarrierSenseErrors;

	mib2PhyCounters->ethTooLongRxFrames = stats.ifInOversizedFrames;

	mib2PhyCounters->ethInternalMacRxErrors = 0;	/*Hardware doesn't support this */

	mib2PhyCounters->ethSymbolErrors = 0;	/*Hardware doesn't support this */

	return;

}

unsigned int ts_ddc_stat_dbg = 0;

LOCAL void cpmacDDCIfcntClear(CpmacDDCObj * hDDC)
{

	PAL_osMemSet((char *)&hDDC->Mib2IfHCCounter, 0,
		     sizeof(hDDC->Mib2IfHCCounter));

}

LOCAL void cpmacDDCIfcntUpdt(CpmacDDCObj * hDDC)
{

	int result;

	CpmacHwStatistics stats;

	result =
	    DDC_cpmacControl(hDDC, CPMAC_DDC_IOCTL_GET_STATISTICS,
			     (Uint32 *) & stats, NULL);

	if (result != 0) {

		LOGERR
		    ("\ncpmacStats: Error from ioctl for DDC CPMAC_DDC_IOCTL_GET_STATISTICS \n");

		return;

	}

	if (stats.ifInOctets >= hDDC->Mib2IfHCCounter.inBytes) {

		hDDC->Mib2IfHCCounter.inBytesHC +=
		    (stats.ifInOctets - hDDC->Mib2IfHCCounter.inBytes);

	}

	else {

		hDDC->Mib2IfHCCounter.inBytesHC +=
		    0xffffffff - (hDDC->Mib2IfHCCounter.inBytes -
				  stats.ifInOctets);

	}

	hDDC->Mib2IfHCCounter.inBytes = stats.ifInOctets;

	if (stats.ifInGoodFrames >=
	    hDDC->Mib2IfHCCounter.inMulticastPkts +
	    hDDC->Mib2IfHCCounter.inBroadcastPkts +
	    hDDC->Mib2IfHCCounter.inUnicastPkts) {

		hDDC->Mib2IfHCCounter.inUnicastPktsHC +=
		    ((stats.ifInGoodFrames -
		      (stats.ifInBroadcasts + stats.ifInMulticasts))
		     - hDDC->Mib2IfHCCounter.inUnicastPkts);

	}

	else {

		hDDC->Mib2IfHCCounter.inUnicastPktsHC +=
		    0xffffffff - (hDDC->Mib2IfHCCounter.inUnicastPkts -
				  (stats.ifInGoodFrames -
				   (stats.ifInBroadcasts +
				    stats.ifInMulticasts)));

	}

	hDDC->Mib2IfHCCounter.inUnicastPkts = (stats.ifInGoodFrames -
					       (stats.ifInBroadcasts +
						stats.ifInMulticasts));

	if (stats.ifInMulticasts >= hDDC->Mib2IfHCCounter.inMulticastPkts) {

		hDDC->Mib2IfHCCounter.inMulticastPktsHC +=
		    (stats.ifInMulticasts -
		     hDDC->Mib2IfHCCounter.inMulticastPkts);

	}

	else {

		hDDC->Mib2IfHCCounter.inMulticastPktsHC +=
		    0xffffffff - (hDDC->Mib2IfHCCounter.inMulticastPkts -
				  stats.ifInMulticasts);

	}

	hDDC->Mib2IfHCCounter.inMulticastPkts = stats.ifInMulticasts;

	if (stats.ifInBroadcasts >= hDDC->Mib2IfHCCounter.inBroadcastPkts) {

		hDDC->Mib2IfHCCounter.inBroadcastPktsHC +=
		    (stats.ifInBroadcasts -
		     hDDC->Mib2IfHCCounter.inBroadcastPkts);

	}

	else {

		hDDC->Mib2IfHCCounter.inBroadcastPktsHC +=
		    0xffffffff - (hDDC->Mib2IfHCCounter.inBroadcastPkts -
				  stats.ifInBroadcasts);

	}

	hDDC->Mib2IfHCCounter.inBroadcastPkts = stats.ifInBroadcasts;

	if (stats.ifOutOctets >= hDDC->Mib2IfHCCounter.outBytes) {

		hDDC->Mib2IfHCCounter.outBytesHC +=
		    (stats.ifOutOctets - hDDC->Mib2IfHCCounter.outBytes);

	}

	else {

		hDDC->Mib2IfHCCounter.outBytesHC +=
		    0xffffffff - (hDDC->Mib2IfHCCounter.outBytes -
				  stats.ifOutOctets);

	}

	hDDC->Mib2IfHCCounter.outBytes = stats.ifOutOctets;

	if (stats.ifOutGoodFrames >=
	    hDDC->Mib2IfHCCounter.outMulticastPkts +
	    hDDC->Mib2IfHCCounter.outBroadcastPkts +
	    hDDC->Mib2IfHCCounter.outUnicastPkts) {

		hDDC->Mib2IfHCCounter.outUnicastPktsHC +=
		    ((stats.ifOutGoodFrames -
		      (stats.ifOutBroadcasts + stats.ifOutMulticasts))
		     - hDDC->Mib2IfHCCounter.outUnicastPkts);

	}

	else {

		hDDC->Mib2IfHCCounter.outUnicastPktsHC +=
		    0xffffffff - (hDDC->Mib2IfHCCounter.outUnicastPkts -
				  (stats.ifOutGoodFrames -
				   (stats.ifOutBroadcasts +
				    stats.ifOutMulticasts)));

	}

	hDDC->Mib2IfHCCounter.outUnicastPkts = (stats.ifOutGoodFrames -
						(stats.ifOutBroadcasts +
						 stats.ifOutMulticasts));

	if (stats.ifOutMulticasts >= hDDC->Mib2IfHCCounter.outMulticastPkts) {

		hDDC->Mib2IfHCCounter.outMulticastPktsHC +=
		    (stats.ifOutMulticasts -
		     hDDC->Mib2IfHCCounter.outMulticastPkts);

	}

	else {

		hDDC->Mib2IfHCCounter.outMulticastPktsHC +=
		    0xffffffff - (hDDC->Mib2IfHCCounter.outMulticastPkts -
				  stats.ifOutMulticasts);

	}

	hDDC->Mib2IfHCCounter.outMulticastPkts = stats.ifOutMulticasts;

	if (stats.ifOutBroadcasts >= hDDC->Mib2IfHCCounter.outBroadcastPkts) {

		hDDC->Mib2IfHCCounter.outBroadcastPktsHC +=
		    (stats.ifOutBroadcasts -
		     hDDC->Mib2IfHCCounter.outBroadcastPkts);

	}

	else {

		hDDC->Mib2IfHCCounter.outBroadcastPktsHC +=
		    0xffffffff - (hDDC->Mib2IfHCCounter.outBroadcastPkts -
				  stats.ifOutBroadcasts);

	}

	hDDC->Mib2IfHCCounter.outBroadcastPkts = stats.ifOutBroadcasts;

	/* low 32-bit of total octets received from media */
	hDDC->Mib2IfHCCounter.Mib2IfCounter.inBytesLow =
	    (unsigned long)hDDC->Mib2IfHCCounter.inBytesHC;

	/* high 32-bit of total octets received from media */
	hDDC->Mib2IfHCCounter.Mib2IfCounter.inBytesHigh =
	    (hDDC->Mib2IfHCCounter.inBytesHC >> 32);

	/* low 32-bit of total octets received from media */
	hDDC->Mib2IfHCCounter.Mib2IfCounter.inUnicastPktsLow =
	    (unsigned long)hDDC->Mib2IfHCCounter.inUnicastPktsHC;

	/* high 32-bit of total octets received from media */
	hDDC->Mib2IfHCCounter.Mib2IfCounter.inUnicastPktsHigh =
	    (hDDC->Mib2IfHCCounter.inUnicastPktsHC >> 32);

	/* low 32-bit of total octets received from media */
	hDDC->Mib2IfHCCounter.Mib2IfCounter.inMulticastPktsLow =
	    (unsigned long)hDDC->Mib2IfHCCounter.inMulticastPktsHC;

	/* high 32-bit of total octets received from media */
	hDDC->Mib2IfHCCounter.Mib2IfCounter.inMulticastPktsHigh =
	    hDDC->Mib2IfHCCounter.inMulticastPktsHC >> 32;

	/* low 32-bit of total octets received from media */
	hDDC->Mib2IfHCCounter.Mib2IfCounter.inBroadcastPktsLow =
	    (unsigned long)hDDC->Mib2IfHCCounter.inBroadcastPktsHC;

	/* high 32-bit of total octets received from media */
	hDDC->Mib2IfHCCounter.Mib2IfCounter.inBroadcastPktsHigh =
	    hDDC->Mib2IfHCCounter.inBroadcastPktsHC >> 32;

	/* packets discarded due to resource limit */
	hDDC->Mib2IfHCCounter.Mib2IfCounter.inDiscardPkts =
	    stats.ifRxDMAOverruns
	    + stats.ifRxMofOverruns
	    + stats.
	    ifRxSofOverruns
	    + stats.ifInCRCErrors
	    + stats.
	    ifInAlignCodeErrors
	    + stats.ifInJabberFrames
	    + stats.
	    ifInFragments
	    + stats.ifInOversizedFrames
	    + stats.
	    ifInUndersizedFrames
	    + stats.ifInFilteredFrames + stats.ifInQosFilteredFrames;

	/* packets discarded due to format errors */
	hDDC->Mib2IfHCCounter.Mib2IfCounter.inErrorPkts =
	    stats.ifInCRCErrors
	    + stats.ifInAlignCodeErrors
	    + stats.ifInJabberFrames + stats.ifInFragments;

	hDDC->Mib2IfHCCounter.Mib2IfCounter.inUnknownProtPkts = 0;	/* Hardware doesn't support this.
									   packets for unknown protocols */

	/* low 32-bit of total octets received from media */
	hDDC->Mib2IfHCCounter.Mib2IfCounter.outBytesLow =
	    (unsigned long)hDDC->Mib2IfHCCounter.outBytesHC;

	/* high 32-bit of total octets received from media */
	hDDC->Mib2IfHCCounter.Mib2IfCounter.outBytesHigh =
	    hDDC->Mib2IfHCCounter.outBytesHC >> 32;

	/* low 32-bit of total octets received from media */
	hDDC->Mib2IfHCCounter.Mib2IfCounter.outUnicastPktsLow =
	    (unsigned long)hDDC->Mib2IfHCCounter.outUnicastPktsHC;

	/* high 32-bit of total octets received from media */
	hDDC->Mib2IfHCCounter.Mib2IfCounter.outUnicastPktsHigh =
	    hDDC->Mib2IfHCCounter.outUnicastPktsHC >> 32;

	/* low 32-bit of total octets received from media */
	hDDC->Mib2IfHCCounter.Mib2IfCounter.outMulticastPktsLow =
	    (unsigned long)hDDC->Mib2IfHCCounter.outMulticastPktsHC;

	/* high 32-bit of total octets received from media */
	hDDC->Mib2IfHCCounter.Mib2IfCounter.outMulticastPktsHigh =
	    hDDC->Mib2IfHCCounter.outMulticastPktsHC >> 32;

	/* low 32-bit of total octets received from media */
	hDDC->Mib2IfHCCounter.Mib2IfCounter.outBroadcastPktsLow =
	    (unsigned long)hDDC->Mib2IfHCCounter.outBroadcastPktsHC;

	/* high 32-bit of total octets received from media */
	hDDC->Mib2IfHCCounter.Mib2IfCounter.outBroadcastPktsHigh =
	    hDDC->Mib2IfHCCounter.outBroadcastPktsHC >> 32;

	/* packets discarded due to format errors */
	hDDC->Mib2IfHCCounter.Mib2IfCounter.outErrorPkts =
	    (stats.ifExcessiveCollisionFrames
	     + stats.ifLateCollisions + stats.ifCarrierSenseErrors);

	/* packets discarded due to resource limit */
	hDDC->Mib2IfHCCounter.Mib2IfCounter.outDiscardPkts =
	    stats.ifOutUnderrun +
	    hDDC->Mib2IfHCCounter.Mib2IfCounter.outErrorPkts;

	return;

}

/* Debug functions for VxWorks only */
#ifdef TORNADO2_2

extern int printf();

/* DDC specific debug functions */
void cpmacDDCStats(Uint32 instId)
{

	CpmacDDCObj *hDDC;

	CpmacRxCppiCh *rxCppi;

	CpmacTxCppiCh *txCppi;

	if (instId >= CpmacDDCNumInst) {

		printf
		    ("\ncpmacDumpRxStat: Invalid instance id %d. Currently only %d instances available",
		     instId, CpmacDDCNumInst);

		return;

	}

	hDDC = CpmacDDCObject[instId];

	if (hDDC == NULL) {

		printf
		    ("\ncpmacDumpRxStat: Invalid DDC handle for instance id %d.",
		     instId);

		return;

	}

	rxCppi = hDDC->rxCppi[0];

	txCppi = hDDC->txCppi[0];

	printf("\nCPMAC Instance %d Statistics", instId);

	if (hDDC->txTeardownPending[0] == True)

		printf("\nTX Teardown pending ...");

	else {

		printf("\n");

		printf("\nTX Proc Count                 = %u",
		       txCppi->procCount);

		printf("\nTX Queue Reinit               = %u",
		       txCppi->queueReinit);

		printf("\nTX End of Queue Add in Send   = %u",
		       txCppi->endOfQueueAdd);

		printf("\nTX Active Queue Count         = %u",
		       txCppi->activeQueueCount);

		printf("\nTX Out of BD's                = %u",
		       txCppi->outOfTxBD);

		printf("\nTX Int when no active pkts    = %u",
		       txCppi->noActivePkts);

		printf("\nPkts with multifragments      = %u",
		       txCppi->numMultiFragPkts);

	}

	if (hDDC->rxTeardownPending[0] == True)

		printf("\nRX Teardown pending ...");

	else {

		printf("\n");

		printf("\nRX Proc Count                 = %u",
		       rxCppi->procCount);

		printf("\nRX Processed BD's             = %u",
		       rxCppi->processedBD);

		printf("\nRX Recycled BD's              = %u",
		       rxCppi->recycledBD);

		printf("\nRX Out of BD's                = %u",
		       rxCppi->outOfRxBD);

		printf("\nRX Out of Buffers             = %u",
		       rxCppi->outOfRxBuffers);

		printf("\nRX Queue reinit               = %u",
		       rxCppi->queueReinit);

		printf("\nRX End of queue Add           = %u",
		       rxCppi->endOfQueueAdd);

		printf("\nRX End of queue               = %u",
		       rxCppi->endOfQueue);

		printf("\nRX Misqueued packets          = %u",
		       rxCppi->misQueuedPackets);

		printf("\nPkts with multifragments      = %u",
		       rxCppi->numMultiFragPkts);

	}

	printf("\n");

}

#endif				/* Tornado 2.2 */
