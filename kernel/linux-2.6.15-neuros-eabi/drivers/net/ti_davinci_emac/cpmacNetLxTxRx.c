/*
 * linux/drivers/net/ti_davinci_emac/cpmacNetLxTxRx.c
 *
 * Linux DDA Send/Receive Source file
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
 ver. 0.1 Suraj Iyer - original linux driver
      0.2 Anant Gole - recoded as per PSPF architecture
      0.3 Sharath Kumar - Incorporated review comments
 */

/*
    Notes:
    DDA/DDC Common features:

    The following flags should be defined by the make file for support of the features:

    (1) CPMAC_CACHE_WRITEBACK_MODE to support write back cache mode.

    (2) CPMAC_MULTIPACKET_TX_COMPLETE_NOTIFY - to  support of multiple Tx complete 
        notifications. If this is defined the Tx complete DDA callback function 
        contains multiple packet Tx complete events.
        Note: BY DEFAULT THIS DRIVER HANDLES MULTIPLE TX COMPLETE VIA ITS CALLBACK IN THE SAME
        FUNCTION FOR SINGLE PACKET COMPLETE NOTIFY.

    (3) CPMAC_MULTIPACKET_RX_COMPLETE_NOTIFY - to support multiple Rx packets to be given
        to DDA layer. If this is defined, DDA should provide the callback function for
        multiple packets too.

    DDA specific features:

    (4) CPMAC_USE_CONFIG_SERVICE - Not supported by this driver version

    (5) CPMAC_USE_ENV - Not supported by this driver version

    (6) CONFIG_CPMAC_INIT_BUF_MALLOC - Not required for DaVinci driver - feature was added for another TI platform

    (7) CPMAC_DDA_CACHE_INVALIDATE_FIX - to use the fix of invalidating the receive buffer before
        providing it to the DMA (to avoid possible data cache corruption if a dirty cache line is
        evicted and gets written after the RX DMA has written to the memory). If not defined, the receive
        buffer is invalidated after the DMA has been done (with possible data corruption). Since there
        is less probability of this condition occuring, the user of the driver can choose to use this
        fix or not. Note that by using this fix, the whole buffer is being invalidated rather than just
        the size of received packet, there is a performance hit if this fix is used.
 */

#include <linux/config.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/string.h>
#include <linux/timer.h>
#include <linux/errno.h>
#include <linux/in.h>
#include <linux/ioport.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <linux/ethtool.h>
#include <linux/highmem.h>
#include <linux/proc_fs.h>
#include <linux/ctype.h>
#include <asm/irq.h>		/* For NR_IRQS only. */
#include <asm/bitops.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/semaphore.h>

#include "cpmac_palOsMem.h"	/* CPMAC port for PAL OS Mem inline functions */
#include "cpmac_palOsCache.h"	/* CPMAC port for PAL OS Cache inline functions */
#include "cpmac_palOsProtect.h"	/* CPMAC port for PAL OS Protect inline functions */

#include "cpmacNetLx.h"		/* This will include required DDC headers */
#include "cpmacNetLxCfg.h"

/* Linux 2.6 Kernel Ethernet Poll function 
 * Call only RX processing in the poll function - TX is taken care of in interrupt context 
 */
int cpmac_poll(struct net_device *p_dev, int *budget)
{

	CpmacNetDevice *hDDA = netdev_priv(p_dev);

	unsigned int work = min(p_dev->quota, *budget);

	unsigned int pkts_pending = 0;

	/* This is used to pass the rx packets to be processed and return the number of rx packets processed */
	RxTxParams *napi_params = &hDDA->napiRxTx;

	if (!hDDA->setToClose) {

		napi_params->rxPkts = work;

		napi_params->txPkts = CPMAC_DDA_DEFAULT_TX_MAX_SERVICE;

		/* Process packets - Call the DDC packet processing function */
		hDDA->ddcIf->pktProcess(hDDA->hDDC, &pkts_pending, napi_params);	/* work returns the number of rx packets processed */

		/* If more packets reschedule the tasklet or call pktProcessEnd */
		if (!pkts_pending) {

			netif_rx_complete(p_dev);

			hDDA->ddcIf->pktProcessEnd(hDDA->hDDC, NULL);

			return 0;

		}

		else if (!test_bit(0, &hDDA->setToClose)) {

			*budget -= napi_params->retRxPkts;

			p_dev->quota -= napi_params->retRxPkts;

			return 1;

		}

	}

	return 0;		/* We are closing down, so dont process anything */

}

/* Allocate RX buffer */
Ptr DDA_cpmac_net_alloc_rx_buf(CpmacNetDevice * hDDA, Int bufSize,
			       DDC_NetDataToken * dataToken,
			       Uint32 channel, Ptr allocArgs)
{

	struct net_device *p_dev = hDDA->owner;

	struct sk_buff *p_skb;

	p_skb = dev_alloc_skb(hDDA->rxBufSize);

	if (p_skb == NULL) {

#ifdef CPMAC_DDA_DEBUG		/* We dont want the error printf to appear on screen as it clogs the serial port */
		errPrint
		    ("DDA_cpmac_net_alloc_rx_buf:Failed to allocate skb for %s.\n",
		     p_dev->name);

#endif				/* 
				 */
		return (NULL);

	}

	/* Set device pointer in skb and reserve space for extra bytes */
	p_skb->dev = p_dev;

	skb_reserve(p_skb, hDDA->rxBufOffset);

	/* Set the data token */
	*dataToken = (DDC_NetDataToken) p_skb;

#ifdef CPMAC_DDA_CACHE_INVALIDATE_FIX
	/* Invalidate buffer */
	CPMAC_DDA_CACHE_INVALIDATE((unsigned long)p_skb->data, bufSize);

#endif

	return p_skb->data;

}

/* Free RX buffer */
PAL_Result DDA_cpmac_net_free_rx_buf(CpmacNetDevice * hDDA, Ptr buffer,
				     DDC_NetDataToken dataToken,
				     Uint32 channel, Ptr freeArgs)
{

	dev_kfree_skb_any((struct sk_buff *)dataToken);

	return (CPMAC_SUCCESS);

}

#if 0
#define isprint(a) ((a >=' ')&&(a<= '~'))
void xdump(unsigned char *cp, int length, char *prefix)
{

	int col, count;

	u_char prntBuf[120];

	u_char *pBuf = prntBuf;

	count = 0;

	while (count < length) {

		pBuf += sprintf(pBuf, "%s", prefix);

		for (col = 0; count + col < length && col < 16; col++) {

			if (col != 0 && (col % 4) == 0)

				pBuf += sprintf(pBuf, " ");

			pBuf += sprintf(pBuf, "%02X ", cp[count + col]);

		}

		while (col++ < 16) {	/* pad end of buffer with blanks */

			if ((col % 4) == 0)

				sprintf(pBuf, " ");

			pBuf += sprintf(pBuf, "   ");

		}

		pBuf += sprintf(pBuf, "  ");

		for (col = 0; count + col < length && col < 16; col++) {

			if (isprint((int)cp[count + col]))

				pBuf += sprintf(pBuf, "%c", cp[count + col]);

			else

				pBuf += sprintf(pBuf, ".");

		}

		sprintf(pBuf, "\n");

		// SPrint(prntBuf);
		printk(prntBuf);

		count += col;

		pBuf = prntBuf;

	}

}				/* close xdump(... */

#endif				/* 
				 */

/* Receive Packet */
PAL_Result DDA_cpmac_net_rx(CpmacNetDevice * hDDA, DDC_NetPktObj * pkt,
			    Ptr rxArgs, Ptr arg)
{

	struct sk_buff *p_skb = (struct sk_buff *)pkt->pktToken;

	skb_put(p_skb, pkt->pktLength);

#ifndef CPMAC_DDA_CACHE_INVALIDATE_FIX
	/* Invalidate cache */
	CPMAC_DDA_CACHE_INVALIDATE((unsigned long)p_skb->data, pkt->pktLength);

#endif

	p_skb->protocol = eth_type_trans(p_skb, hDDA->owner);

	p_skb->dev->last_rx = jiffies;

	netif_receive_skb(p_skb);

	hDDA->netDevStats.rx_packets++;

	hDDA->netDevStats.rx_bytes += pkt->pktLength;

	return (0);

}

#ifdef CPMAC_MULTIPACKET_RX_COMPLETE_NOTIFY
/* 
 * Multiple packet receive 
 * 
 * This function get multiple received packets via the netPktList and it queues these packets into 
 * the higher layer queue
 *
 * Note that rxArgs contains "channel" and is ignored for this implementation
*/
PAL_Result DDA_cpmac_net_rx_multiple_cb(CpmacNetDevice * hDDA,
					DDC_NetPktObj * netPktList,
					Int numPkts, Ptr rxArgs)
{

	Uint32 cnt;

	for (cnt = 0; cnt < numPkts; cnt++) {

		struct sk_buff *p_skb = (struct sk_buff *)netPktList->pktToken;

		/* Set length of packet */
		skb_put(p_skb, netPktList->pktLength);

#ifndef CPMAC_DDA_CACHE_INVALIDATE_FIX
		/* Invalidate cache */
		CPMAC_DDA_CACHE_INVALIDATE((unsigned long)p_skb->data,
					   p_skb->len);

#endif

		p_skb->protocol = eth_type_trans(p_skb, hDDA->owner);

		p_skb->dev->last_rx = jiffies;

		netif_receive_skb(p_skb);

		hDDA->netDevStats.rx_bytes += netPktList->pktLength;

		++netPktList;

	}

	hDDA->netDevStats.rx_packets += numPkts;

	return (0);

}

#endif				/*  CPMAC_MULTIPACKET_RX_COMPLETE_NOTIFY */

/* Transmit Complete Callback */
PAL_Result DDA_cpmac_net_tx_complete(CpmacNetDevice * hDDA,
				     DDC_NetDataToken * netDataTokens,
				     Int numTokens, Uint32 channel)
{

	Uint32 cnt;

	if (numTokens && netif_queue_stopped(hDDA->owner)) {

		printk("EMAC: TX Complete: Starting queue\n");

		netif_start_queue(hDDA->owner);

	}

	for (cnt = 0; cnt < numTokens; cnt++) {

		struct sk_buff *skb = (struct sk_buff *)netDataTokens[cnt];

		if (skb == NULL)
			continue;

		hDDA->netDevStats.tx_packets++;

		hDDA->netDevStats.tx_bytes += skb->len;

		dev_kfree_skb_any(skb);

	}

	return (0);

}

/******************************************************************************
 *  Interrupt functions
 *****************************************************************************/

/* DDA ISR */
irqreturn_t cpmac_hal_isr(int irq, void *dev_id, struct pt_regs * regs)
{

	CpmacNetDevice *hDDA = (CpmacNetDevice *) dev_id;

	++hDDA->isrCount;

	if (!hDDA->setToClose) {

		/* NAPI support */
		netif_rx_schedule(hDDA->owner);

	}

	else {

		/* We are closing down, so dont process anything */
	}

	return IRQ_HANDLED;

}

/* Transmit Function - Only single fragment supported */
int cpmac_dev_tx(struct sk_buff *skb, struct net_device *p_dev)
{

	PAL_Result retCode;

	DDC_NetBufObj txBuf;	/* Buffer object - Only single frame support */

	DDC_NetPktObj txPacket;	/* Packet object */

	CpmacNetDevice *hDDA = NETDEV_PRIV(p_dev);

	/* ANANT_HACK: unsigned long flags; */

	/* Build the buffer and packet objects - Since only single fragment is 
	 * supported, need not set length and token in both packet & object. 
	 * Doing so for completeness sake & to show that this needs to be done 
	 * in multifragment case 
	 */
	txPacket.bufList = &txBuf;

	txPacket.numBufs = 1;	/* Only single fragment supported */

	txPacket.pktLength = skb->len;

	txPacket.pktToken = (DDC_NetDataToken) skb;

	txBuf.length = skb->len;

	txBuf.bufToken = (DDC_NetDataToken) skb;

	txBuf.dataPtr = skb->data;

	/* Flush data buffer if write back mode */
	CPMAC_DDA_CACHE_WRITEBACK((unsigned long)skb->data, skb->len);

	p_dev->trans_start = jiffies;

	/* ANANT_HACK: Need to lock TX so that there is no contention 
	   spin_lock_irqsave(&hDDA->lock, flags);
	 */

	/* DDC Send : last param FALSE so that hardware calculates CRC */
	retCode =
	    hDDA->ddcIf->ddcNetIf.ddcNetSend(hDDA->hDDC, &txPacket,
					     CPMAC_DDA_DEFAULT_TX_CHANNEL,
					     False);

	/* ANANT_HACK: Need to un-lock TX so that there is no contention between two processes 
	   spin_unlock_irqrestore(&hDDA->lock, flags);
	 */

	if (retCode != CPMAC_SUCCESS) {

		if (retCode == CPMAC_ERR_TX_OUT_OF_BD) {

			errPrint("WARN: cpmac_dev_tx: Out of TX BD's\n");
			netif_stop_queue(hDDA->owner);
		}

		hDDA->netDevStats.tx_dropped++;

		goto cpmac_dev_tx_drop_pkt;

	}

	return (0);

      cpmac_dev_tx_drop_pkt:
	hDDA->netDevStats.tx_dropped++;

	return (-1);

}
