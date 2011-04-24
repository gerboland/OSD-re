/*
 * linux/drivers/net/ti_davinci_emac/cpmacNetLx.c
 *
 * EMAC Network Driver Linux adaptation file
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
#include <linux/version.h>
#include <asm/irq.h>		/* For NR_IRQS only. */
#include <asm/bitops.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/semaphore.h>
#include <asm/hardware/clock.h>

#include "cpmac_palOsMem.h"	/* CPMAC port for PAL OS Mem inline functions */
#include "cpmac_palOsCache.h"	/* CPMAC port for PAL OS Cache inline functions */
#include "cpmac_palOsProtect.h"	/* CPMAC port for PAL OS Protect inline functions */
#include "cpmacNetLx.h"		/* This will include required DDC headers */
#include "mib_ioctl.h"

/* NO PHY used in case of external ethernet switches */
#define CONFIG_CPMAC_NOPHY                                    9999

MODULE_LICENSE("GPL");

MODULE_AUTHOR("Davinci CPMAC Maintainer: Anant Gole <anantgole@ti.com>");

MODULE_DESCRIPTION("DaVinci Ethernet driver - CPMAC (EMAC)");

static int cfg_link_speed = 0;

MODULE_PARM(cfg_link_speed, "i");

MODULE_PARM_DESC(cfg_link_speed, "Fixed speed of the Link: <100/10>");

static char *cfg_link_mode = NULL;

MODULE_PARM(cfg_link_mode, "1-3s");

MODULE_PARM_DESC(cfg_link_mode, "Fixed mode of the Link: <fd/hd>");

int debug_mode = 0;

MODULE_PARM(debug_mode, "i");

MODULE_PARM_DESC(debug_mode,
		 "Turn on the debug info: <0/1>. Default is 0 (off)");

/* Version Info */
#define CPMAC_DDA_MAJOR_VERSION         3
#define CPMAC_DDA_MINOR_VERSION         0

#define CPMAC_MODULE_VERSION "3.0"
MODULE_VERSION(CPMAC_MODULE_VERSION);

const char cpmac_DDA_version_string[] = "TI DaVinci EMAC Linux version 3.0";

/* Debug tracing */
int cpmac_link_status = 1;

int cpmac_debug_mode = 0;

/* Global variables required during initialization */
static int g_cfg_start_link_params = CFG_START_LINK_SPEED;

static int g_init_enable_flag = 0;

/* Global device array */
static struct net_device *cpmac_net_dev[CPMAC_MAX_INSTANCES] = { NULL };

/* Lock to be used in protection functions */
spinlock_t *cpmac_lock;

/* ANANT HACK */

/* Last cpmac device accessed */
static struct net_device *last_cpmac_device = NULL;

/* Number of CPMAC instances */
static int cpmac_devices_installed = 0;

extern int cpmac_poll(struct net_device *p_dev, int *budget);

/* Proc entries */
static struct proc_dir_entry *gp_stats_file = NULL;

/* The real device and driver matching will be done by the 
 * match routine of the platform bus. It is necessary 
 * for the probe function to be non null though. 
 * We have a function that just returns zero. "All matched."
 */
static int __devinit cpmac_probe(struct device *dev)
{

	return 0;

}

/* structure describing the CPMAC driver */
static struct device_driver cpmac_driver = {
	.name = "cpmac",
	.bus = &platform_bus_type,
	.probe = cpmac_probe,
	.remove = NULL,		/* TODO: Findout when probe would be called. */
	.suspend = NULL,
	.resume = NULL,
};

static char cpmac_cfg[CPMAC_MAX_INSTANCES][200];

/* Clock frequency for EMAC */
static struct clk *cpmac_clk;
static unsigned long cpmac_bus_frequency;

static int cpmac_cfg_build(int connect, int external_switch)
{

	static int cfg_instance = 0;

	unsigned int speed = 0;

	speed = (external_switch) ? CONFIG_CPMAC_NOPHY : 0;

	sprintf(cpmac_cfg[cfg_instance],
		"%d:%x:%d:%d:%u:%d:%d:%d:%d:%d:%d:%d:%d:%d:%d:%d:%x:%d:%d:%u:%u:%x:%d",
		cfg_instance, DAVINCI_CPMAC_BASE_ADDR,
		DAVINCI_CPMAC_INTERRUPT, 0, DAVINCI_CPMAC_BUS_FREQUENCY,
		speed, 0, CPMAC_DEFAULT_PROMISCOUS_ENABLE,
		CPMAC_DEFAULT_BROADCAST_ENABLE,
		CPMAC_DEFAULT_MULTICAST_ENABLE,
		CPMAC_DDA_DEFAULT_MAX_FRAME_SIZE,
		CPMAC_DDA_DEFAULT_TX_NUM_BD,
		CPMAC_DDA_DEFAULT_TX_MAX_SERVICE, CPMAC_DDA_DEFAULT_RX_NUM_BD,
		CPMAC_DDA_DEFAULT_RX_MAX_SERVICE, 0,
		DAVINCI_CPMAC_MDIO_BASE_ADDR, 0, 0,
		DAVINCI_CPMAC_BUS_FREQUENCY,
		DAVINCI_CPMAC_MDIO_FREQUENCY, DAVINCI_CPMAC_PHY_MASK, 10);

	dbgPrint("TI DAVINCI EMAC Driver Config:\n\n%s\n\n",
		 cpmac_cfg[cfg_instance]);

	cfg_instance++;

	return (0);

}

/* Get Ethernet address from kernel boot params */
static unsigned char cpmac_eth_string[20] = "deadbeaf";

static int cpmac_eth_setup(char *str)
{

	strcpy(&cpmac_eth_string[0], &str[1]);	/* The first char passed from the bootloader is '=', so ignore it */

	printk("TI DaVinci EMAC: Kernel Boot params Eth address: %s\n",
	       cpmac_eth_string);

	return (1);

}

__setup("eth", cpmac_eth_setup);

static int cpmac_cfg_probe(void)
{

	/* For DaVinci there is only 1 CPMAC instance */
	if (cpmac_cfg_build(0, 0))

		return (-1);

	else

		return (0);

}

/* 
 * L3 Alignment mechanism:
 * The below given macro returns the number of bytes required to align the given size
 * to a L3 frame 4 byte boundry. This is typically required to add 2 bytes to the ethernet
 * frame start to make sure the IP header (L3) is aligned on a 4 byte boundry
 */
static char cpmac_L3_align[] = { 0x02, 0x01, 0x00, 0x03 };

#define CPMAC_L3_ALIGN(size)    cpmac_L3_align[(size) & 0x3]

/* 4 Byte alignment for skb's:
 * Currently Skb's dont need to be on a 4 byte boundry, but other OS's have such requirements
 * Just to make sure we comply if there is any such requirement on SKB's in future, we align
 * it on a 4 byte boundry.
 */
char cpmac_4byte_align[] = { 0x0, 0x03, 0x02, 0x1 };

#define CPMAC_4BYTE_ALIGN(size) cpmac_4byte_align[(size) & 0x3]

/* Net device related private function prototypes */
static int cpmac_dev_init(struct net_device *p_dev);

static int cpmac_dev_open(struct net_device *dev);

static int cpmac_ioctl(struct net_device *p_dev, struct ifreq *rq, int cmd);

static int cpmac_dev_close(struct net_device *p_dev);

static void cpmac_dev_mcast_set(struct net_device *p_dev);

static void cpmac_tx_timeout(struct net_device *p_dev);

static struct net_device_stats *cpmac_dev_get_net_stats(struct net_device
							*dev);

/* Internal Function prototypes */
static int __init cpmac_p_detect_manual_cfg(int, char *, int);

static int cpmac_p_read_stats(char *buf, char **start, off_t offset, int count,
			      int *eof, void *data);

static int cpmac_p_write_stats(struct file *fp, const char *buf,
			       unsigned long count, void *data);

static int cpmac_p_read_link(char *buf, char **start, off_t offset, int count,
			     int *eof, void *data);

static int cpmac_dump_config(char *buf, char **start, off_t offset, int count,
			     int *eof, void *data);

static int cpmac_p_get_version(char *buf, char **start, off_t offset,
			       int count, int *eof, void *data);

static int cpmac_p_update_statistics(struct net_device *p_dev, char *buf,
				     int limit, int *p_len);

static int cpmac_p_reset_statistics(struct net_device *p_dev);

static int cpmac_p_read_rfc2665_stats(char *buf, char **start, off_t offset,
				      int count, int *eof, void *data);

static int cpmac_p_dev_enable(CpmacNetDevice * hDDA);

static int cpmac_p_dev_disable(CpmacNetDevice * hDDA);

static void cpmac_p_tick_timer_expiry(CpmacNetDevice * hDDA);

static int cpmac_dev_set_mac_addr(struct net_device *p_dev, void *addr);

/* Function prototype for cpmac_p_tick_timer_expiry() function as per Linux Timer API */
typedef void (*timer_tick_func) (unsigned long);

/* DDA function table */
static PAL_Result DDA_cpmac_control_cb(CpmacNetDevice * hDDA, Int cmd,
				       Ptr cmdArg, Ptr param);

/* Static CPMAC DDA function table */
static CpmacDDACbIf cpmac_DDA_cb_interface = {
	{			/* DDC Net Class functions */
	 {			/*   DDC Class functions */
	  (DDA_ControlCb)
	  DDA_cpmac_control_cb
	  /*     Control Callback function */
	  },
	 NULL,			/*   DDA_NetRxNotifyCb function - Not supported/required */
	 (DDA_NetAllocRxBufCb)
	 DDA_cpmac_net_alloc_rx_buf,
	 /*   DDA_NetAllocRxBufCb function */

	 (DDA_NetFreeRxBufCb)
	 DDA_cpmac_net_free_rx_buf,
	 /*   DDA_NetFreeRxBufCb function */

	 (DDA_NetRxCb)
	 DDA_cpmac_net_rx,
	 /*   DDA_NetRxCb function */
#ifdef CPMAC_MULTIPACKET_RX_COMPLETE_NOTIFY
	 (DDA_NetRxMultipleCb)
	 DDA_cpmac_net_rx_multiple_cb,
	 /*   DDA_NetRxMultipleCb */
#else				/* 
				 */
	 NULL,			/*   DDA_NetRxMultipleCb */
#endif				/* 
				 */
	 (DDA_NetTxCompleteCb)
	 DDA_cpmac_net_tx_complete
	 /*   DDA_NetTxCompleteCb function */
	 },
	(DDA_Printf) printk,	/* DDA printf function */
	(DDA_ErrLog) printk,	/* DDA error log function */
};

/******************************************************************************
 *  Internal Utility functions
 *****************************************************************************/

/* String to Hex conversion */
static unsigned char cpmac_str_to_hexnum(unsigned char c)
{

	if (c >= '0' && c <= '9')
		return c - '0';

	if (c >= 'a' && c <= 'f')
		return c - 'a' + 10;

	if (c >= 'A' && c <= 'F')
		return c - 'A' + 10;

	return 0;

}

/* String to ethernet address conversion */
static void cpmac_str_to_ethaddr(unsigned char *ea, unsigned char *str)
{

	int i;

	unsigned char num;

	for (i = 0; i < 6; i++) {

		if ((*str == '.') || (*str == ':')) {

			str++;

		}

		num = cpmac_str_to_hexnum(*str) << 4;

		++str;

		num |= (cpmac_str_to_hexnum(*str));

		++str;

		ea[i] = num;

	}

}

/******************************************************************************
 *  DDA Callback functions
 *****************************************************************************/

/* DDA_cpmac_control_cb - ioctl function to be called by the DDC */
static PAL_Result DDA_cpmac_control_cb(CpmacNetDevice * hDDA, Int cmd,
				       Ptr cmdArg, Ptr param)
{

	switch (cmd) {

	case CPMAC_DDA_IOCTL_TIMER_START:

		{

			/* cmdArg will directly have the timer period of the periodic
			 * timer, param not used */
			/* DDC asks for milliSecs. So calculate ticks from ticks per 
			 * 1000 mSec
			 */
			struct timer_list *p_timer = &hDDA->periodicTimer;

			hDDA->periodicTicks =
			    (CPMAC_DDA_TICKS_PER_SEC * (Int) cmdArg) / 1000;

			p_timer->expires = jiffies + hDDA->periodicTicks;

			add_timer(&hDDA->periodicTimer);

			hDDA->timerActive = TRUE;

		}
		break;

	case CPMAC_DDA_IOCTL_TIMER_STOP:

		{

			/* cmdArg and param not used */
			if (hDDA->timerActive == TRUE) {

				del_timer_sync(&hDDA->periodicTimer);

				hDDA->timerActive = FALSE;

			}

		}

		break;

	case CPMAC_DDA_IOCTL_STATUS_UPDATE:

		{		/* cmdArg will point to CpmacDDCStatus structure, param not 
				 * used 
				 */

			struct net_device *p_dev = hDDA->owner;

			CpmacDDCStatus *status = &hDDA->ddcStatus;

			hDDA->ddcStatus = *((CpmacDDCStatus *) cmdArg);

			if ((status->hwStatus & CPMAC_DDC_TX_HOST_ERROR) ==
			    CPMAC_DDC_TX_HOST_ERROR) {

				errPrint
				    ("DDA_cpmac_control_cb: TX Host Error. Transmit Stopped %s\n",
				     p_dev->name);

			}

			if ((status->hwStatus & CPMAC_DDC_RX_HOST_ERROR) ==
			    CPMAC_DDC_RX_HOST_ERROR) {

				errPrint
				    ("DDA_cpmac_control_cb: RX Host Error. Receive Stopped %s\n",
				     p_dev->name);

			}

			if (status->PhyLinked) {

				/* Link ON */
				if (!netif_carrier_ok(p_dev)) {

					netif_carrier_on(p_dev);

				}

				hDDA->linkSpeed =
				    ((status->PhySpeed) ? 100000000 : 10000000);

				hDDA->linkMode = ((status->PhyDuplex) ? 3 : 2);

				/* Reactivate the transmit queue if it is stopped */
				if (netif_running(p_dev)
				    && netif_queue_stopped(p_dev)) {

					netif_wake_queue(p_dev);

				}

			}

			else {

				/* Link OFF */
				if (netif_carrier_ok(p_dev)) {

					/* do we need to register synchronization issues with stats here. */
					hDDA->linkSpeed = 100000000;

					hDDA->linkMode = 1;

					netif_carrier_off(p_dev);

				}

				if (!netif_queue_stopped(p_dev)) {

					netif_stop_queue(p_dev);	/* So that kernel does not keep on xmiting pkts. */

				}

			}

			if (cpmac_link_status)

				dbgPrint
				    ("TI DaVinci EMAC: %s, PhyNum %d,  %s, %s, %s\n",
				     ((struct net_device *)hDDA->owner)->name,
				     status->PhyNum,
				     ((status->
				       PhyDuplex) ? "Full Duplex" :
				      "Half Duplex"),
				     ((status->
				       PhySpeed) ? "100 Mbps" : "10 Mbps"),
				     ((status->
				       PhyLinked) ? "Linked" : "NO LINK"));

		}
		break;

	case CPMAC_DDA_IOCTL_MIB64_CNT_TIMER_START:

		{

			/* cmdArg will directly have the timer period of the 
			 * periodic timer, param not used 
			 */
			/* DDC asks for milliSecs. So calculate ticks from ticks per 1000 mSec */
			struct timer_list *p_timer = &hDDA->mibTimer;

			hDDA->mibTicks =
			    (CPMAC_DDA_TICKS_PER_SEC * (Int) cmdArg) / 1000;

			p_timer->expires = jiffies + hDDA->mibTicks;

			add_timer(p_timer);

			hDDA->mibTimerActive = TRUE;

		}
		break;

	case CPMAC_DDA_IOCTL_MIB64_CNT_TIMER_STOP:

		{

			/* cmdArg and param not used */
			if (hDDA->mibTimerActive == TRUE) {

				del_timer_sync(&hDDA->mibTimer);

				hDDA->mibTimerActive = FALSE;

			}

		}

		break;

	default:

		dbgPrint
		    ("TI DAVINCI EMAC: Unhandled ioctl code %d in DDA_cpmac_control_cb\n",
		     cmd);

		break;

	}

	return (CPMAC_SUCCESS);

}

/******************************************************************************
 *  Internal functions
 *****************************************************************************/

/*******************************************************************************
 *
 * cpmacEndGetConfig - Extract configuration for given unit number/instance
 *
 * This function gets the configuration information from the configuration service
 * or by some means for the given unit number/cpmac instance
 *
 * Note: For debug/default, static information is obtained from the header file
 *
 * RETURNS: OK or ERROR.
 */
static int cpmac_net_get_config(CpmacNetDevice * hDDA)
{

#if defined (CPMAC_USE_CONFIG_SERVICE)
#error " CPMAC_USE_CONFIG_SERVICE Not Supported"
#else				/* 
				 */

#define CPMAC_TOKEN_PARSE(str)       { if ((tok = (char *)strsep ((str), ":")) == NULL) return -1; }
#define CPMAC_TOKEN_GET_INTEGER simple_strtoul (tok, NULL, 10)
#define CPMAC_TOKEN_GET_HEX     simple_strtoul (tok, NULL, 16)

	{

		CpmacInitConfig *iCfg = &hDDA->initCfg;

		CpmacChInfo *txChCfg = &hDDA->txChInfo[0];

		CpmacChInfo *rxChCfg = &hDDA->rxChInfo[0];

		int speed, duplex, extra;

		char localStringVal[200];

		char *localString = NULL;

		char *tok;

		char *pHolder = NULL;

		/* Use static config string */
		switch (hDDA->instanceNum) {

		case 0:
			localString = cpmac_cfg[0];
			break;

		case 1:
			localString = cpmac_cfg[1];
			break;

		default:
			localString = cpmac_cfg[0];
			break;

		}

		strcpy(&localStringVal[0], localString);

		localString = &localStringVal[0];

		pHolder = NULL;

		tok = (char *)strsep(&localString, ":");

		if (tok == NULL)
			return (-1);

		iCfg->instId = CPMAC_TOKEN_GET_INTEGER;
		CPMAC_TOKEN_PARSE(&localString);

		dbgPrint("iCfg->instId=%d", iCfg->instId);

		iCfg->baseAddress = CPMAC_TOKEN_GET_HEX;
		CPMAC_TOKEN_PARSE(&localString);

		dbgPrint("\niCfg->baseAddress=%08X", iCfg->baseAddress);

		iCfg->intrLine = CPMAC_TOKEN_GET_INTEGER;
		CPMAC_TOKEN_PARSE(&localString);

		dbgPrint("\niCfg->intrLine=%d", iCfg->intrLine);

		iCfg->resetLine = CPMAC_TOKEN_GET_INTEGER;
		CPMAC_TOKEN_PARSE(&localString);

		dbgPrint("\niCfg->resetLine=%d", iCfg->resetLine);

		iCfg->cpmacBusFrequency = CPMAC_TOKEN_GET_INTEGER;
		CPMAC_TOKEN_PARSE(&localString);

		dbgPrint("\niCfg->cpmacBusFrequency=%d",
			 iCfg->cpmacBusFrequency);

		speed = CPMAC_TOKEN_GET_INTEGER;
		CPMAC_TOKEN_PARSE(&localString);

		dbgPrint("\nspeed=%d", speed);

		duplex = CPMAC_TOKEN_GET_INTEGER;
		CPMAC_TOKEN_PARSE(&localString);

		dbgPrint("\nduplex=%d", duplex);

		iCfg->rxCfg.promiscousEnable = CPMAC_TOKEN_GET_INTEGER;
		CPMAC_TOKEN_PARSE(&localString);

		dbgPrint("\niCfg->rxCfg.promiscousEnable=%d",
			 iCfg->rxCfg.promiscousEnable);

		iCfg->rxCfg.broadcastEnable = CPMAC_TOKEN_GET_INTEGER;
		CPMAC_TOKEN_PARSE(&localString);

		dbgPrint("\niCfg->rxCfg.broadcastEnable=%d",
			 iCfg->rxCfg.broadcastEnable);

		iCfg->rxCfg.multicastEnable = CPMAC_TOKEN_GET_INTEGER;
		CPMAC_TOKEN_PARSE(&localString);

		dbgPrint("\niCfg->rxCfg.multicastEnable=%d",
			 iCfg->rxCfg.multicastEnable);

		iCfg->rxCfg.maxRxPktLength = CPMAC_TOKEN_GET_INTEGER;
		CPMAC_TOKEN_PARSE(&localString);

		dbgPrint("\niCfg->rxCfg.maxRxPktLength=%d",
			 iCfg->rxCfg.maxRxPktLength);

		txChCfg->numBD = CPMAC_TOKEN_GET_INTEGER;
		CPMAC_TOKEN_PARSE(&localString);

		dbgPrint("\ntxChCfg->numBD=%d", txChCfg->numBD);

		txChCfg->serviceMax = CPMAC_TOKEN_GET_INTEGER;
		CPMAC_TOKEN_PARSE(&localString);

		dbgPrint("\ntxChCfg->serviceMax=%d", txChCfg->serviceMax);

		rxChCfg->numBD = CPMAC_TOKEN_GET_INTEGER;
		CPMAC_TOKEN_PARSE(&localString);

		dbgPrint("\nrxChCfg->numBD=%d", rxChCfg->numBD);

		rxChCfg->serviceMax = CPMAC_TOKEN_GET_INTEGER;
		CPMAC_TOKEN_PARSE(&localString);

		dbgPrint("\nrxChCfg->serviceMax=%d", rxChCfg->serviceMax);

		extra = CPMAC_TOKEN_GET_INTEGER;
		CPMAC_TOKEN_PARSE(&localString);

		dbgPrint("\nextra=%d", extra);

		iCfg->mdioBaseAddress = CPMAC_TOKEN_GET_HEX;
		CPMAC_TOKEN_PARSE(&localString);

		dbgPrint("\niCfg->mdioBaseAddress=%08X", iCfg->mdioBaseAddress);

		iCfg->mdioIntrLine = CPMAC_TOKEN_GET_INTEGER;
		CPMAC_TOKEN_PARSE(&localString);

		dbgPrint("\niCfg->mdioIntrLine=%d", iCfg->mdioIntrLine);

		iCfg->mdioResetLine = CPMAC_TOKEN_GET_INTEGER;
		CPMAC_TOKEN_PARSE(&localString);

		dbgPrint("\niCfg->mdioResetLine=%d", iCfg->mdioResetLine);

		iCfg->MdioBusFrequency = CPMAC_TOKEN_GET_INTEGER;
		CPMAC_TOKEN_PARSE(&localString);

		dbgPrint("\niCfg->MdioBusFrequency=%d", iCfg->MdioBusFrequency);

		iCfg->MdioClockFrequency = CPMAC_TOKEN_GET_INTEGER;
		CPMAC_TOKEN_PARSE(&localString);

		dbgPrint("\niCfg->MdioClockFrequency=%d",
			 iCfg->MdioClockFrequency);

		iCfg->PhyMask = CPMAC_TOKEN_GET_HEX;
		CPMAC_TOKEN_PARSE(&localString);

		dbgPrint("\niCfg->PhyMask=%08X", iCfg->PhyMask);

		iCfg->MdioTickMSec = CPMAC_TOKEN_GET_INTEGER;

		dbgPrint("\niCfg->MdioTickMSec=%d", iCfg->MdioTickMSec);

		dbgPrint("\n");

		iCfg->Mib64CntMsec = CONFIG_CPMAC_MIB_TIMER_TIMEOUT;

		rxChCfg->bufSize = iCfg->rxCfg.maxRxPktLength;

		hDDA->rxBufOffset =
		    CPMAC_L3_ALIGN(extra) + CPMAC_DDA_DEFAULT_EXTRA_RXBUF_SIZE;
		printk(KERN_ERR "CPMAC: rxBufOffset=%d\n", hDDA->rxBufOffset);

		hDDA->rxBufSize = (rxChCfg->bufSize + hDDA->rxBufOffset);

		/* align skb's on 4 byte boundry - no hard requirement currently - done for future use */
		hDDA->rxBufSize += CPMAC_4BYTE_ALIGN(hDDA->rxBufSize);

		/* Determine Phy speed/duplex mode - to be built as per MDIO module requirements */
		if (speed == CONFIG_CPMAC_NOPHY) {

			iCfg->phyMode = SNWAY_NOPHY;

		}

		else {

			if ((speed == 0) && (duplex == 0)) {

				iCfg->phyMode = SNWAY_AUTOALL;	/* Auto detection */

			}

			else if (speed == 10) {

				if (duplex == 1) {
					iCfg->phyMode = SNWAY_HD10;
				}

				else if (duplex == 2) {
					iCfg->phyMode = SNWAY_FD10;
				}

				else {
					iCfg->phyMode = SNWAY_HD10 | SNWAY_FD10;
				}

			}

			else if (speed == 100) {

				if (duplex == 1) {
					iCfg->phyMode = SNWAY_HD100;
				}

				else if (duplex == 2) {
					iCfg->phyMode = SNWAY_FD100;
				}

				else {
					iCfg->phyMode =
					    SNWAY_HD100 | SNWAY_FD100;
				}

			}

			else {

				if (duplex == 1) {
					iCfg->phyMode =
					    SNWAY_HD10 | SNWAY_HD100;
				}

				else {
					iCfg->phyMode =
					    SNWAY_FD10 | SNWAY_FD100;
				}

			}

		}

		hDDA->vlanEnable = CPMAC_DDA_DEFAULT_VLAN_ENABLE;

		iCfg->numTxChannels = CPMAC_DDA_DEFAULT_NUM_TX_CHANNELS;

		iCfg->numRxChannels = CPMAC_DDA_DEFAULT_NUM_RX_CHANNELS;

		iCfg->MLinkMask = CPMAC_DEFAULT_MLINK_MASK;

		iCfg->rxCfg.passCRC = CPMAC_DEFAULT_PASS_CRC;

		iCfg->rxCfg.qosEnable = CPMAC_DEFAULT_QOS_ENABLE;

		iCfg->rxCfg.noBufferChaining = CPMAC_DEFAULT_NO_BUFFER_CHAINING;

		iCfg->rxCfg.copyMACControlFramesEnable =
		    CPMAC_DEFAULT_COPY_MAC_CONTROL_FRAMES_ENABLE;

		iCfg->rxCfg.copyShortFramesEnable =
		    CPMAC_DEFAULT_COPY_SHORT_FRAMES_ENABLE;

		iCfg->rxCfg.copyErrorFramesEnable =
		    CPMAC_DEFAULT_COPY_ERROR_FRAMES_ENABLE;

		iCfg->rxCfg.promiscousChannel =
		    CPMAC_DEFAULT_PROMISCOUS_CHANNEL;

		iCfg->rxCfg.broadcastChannel = CPMAC_DEFAULT_BROADCAST_CHANNEL;

		iCfg->rxCfg.multicastChannel = CPMAC_DEFAULT_MULTICAST_CHANNEL;

		iCfg->rxCfg.bufferOffset = CPMAC_DEFAULT_BUFFER_OFFSET;

		iCfg->macCfg.pType = CPMAC_TXPRIO_FIXED;

		iCfg->macCfg.txShortGapEnable = FALSE;

		if (speed == 1000)

			iCfg->macCfg.gigaBitEnable = TRUE;

		else

			iCfg->macCfg.gigaBitEnable = FALSE;

		iCfg->macCfg.txPacingEnable = CPMAC_DEFAULT_TX_PACING_ENABLE;

		iCfg->macCfg.miiEnable = CPMAC_DEFAULT_MII_ENABLE;

		iCfg->macCfg.txFlowEnable = CPMAC_DEFAULT_TX_FLOW_ENABLE;

		iCfg->macCfg.rxFlowEnable = CPMAC_DEFAULT_RX_FLOW_ENABLE;

		iCfg->macCfg.loopbackEnable = CPMAC_DEFAULT_LOOPBACK_ENABLE;

		iCfg->macCfg.fullDuplexEnable =
		    CPMAC_DEFAULT_FULL_DUPLEX_ENABLE;

		iCfg->macCfg.txInterruptDisable =
		    CPMAC_DEFAULT_TX_INTERRUPT_DISABLE;

		txChCfg->chNum = CPMAC_DDA_DEFAULT_TX_CHANNEL;

		txChCfg->chDir = DDC_NET_CH_DIR_TX;

		txChCfg->chState = DDC_NET_CH_UNINITIALIZED;

		rxChCfg->chNum = CPMAC_DDA_DEFAULT_RX_CHANNEL;

		rxChCfg->chDir = DDC_NET_CH_DIR_RX;

		rxChCfg->chState = DDC_NET_CH_UNINITIALIZED;

		/* Module control EWrap base address for DaVinci */
		iCfg->eWrapBaseAddress = DAVINCI_CPMAC_WRAPPER_REGS_ADDR;

		dbgPrint("\n");

	}

#endif				/* Config service or other mechanism */

	return (0);

}

/* Detect Manual config */
static int __init cpmac_p_detect_manual_cfg(int linkSpeed, char *linkMode,
					    int debug)
{

	char *pSpeed = NULL;

	if (debug == 1) {

		cpmac_debug_mode = 1;

		dbgPrint("TI DAVINCI EMAC: Enabled debug print.\n");

	}

	if (!linkSpeed && !linkMode) {

		dbgPrint
		    ("TI DAVINCI EMAC: No manual link params, defaulting to auto negotiation.\n");

		return (0);

	}

	if (!linkSpeed || (linkSpeed != 10 && linkSpeed != 100)) {

		dbgPrint
		    ("TI DAVINCI EMAC: Invalid or No value of link speed specified, defaulting to auto speed.\n");

		pSpeed = "auto";

	}

	if (!linkMode || (!strcmp(linkMode, "fd") && !strcmp(linkMode, "hd")))

		dbgPrint
		    ("TI DaVinci EMAC: Invalid or No value of link mode specified, defaulting to auto mode.\n");

	if ((linkSpeed == 10) && (strcmp(linkMode, "fd"))) {

		g_cfg_start_link_params = SNWAY_FD10;

	}

	else if ((linkSpeed == 10) && (strcmp(linkMode, "hd"))) {

		g_cfg_start_link_params = SNWAY_HD10;

	}

	else if ((linkSpeed == 100) && (strcmp(linkMode, "hd"))) {

		g_cfg_start_link_params = SNWAY_HD100;

	}

	else if ((linkSpeed == 100) && (strcmp(linkMode, "fd"))) {

		g_cfg_start_link_params = SNWAY_FD100;

	}

	dbgPrint
	    ("TI DaVinci EMAC: Link is manually set to the speed of %s speed and %s mode.\n",
	     pSpeed, linkMode ? linkMode : "auto");

	return (0);

}

/* Link read support */
static int cpmac_p_read_link(char *buf, char **start, off_t offset, int count,
			     int *eof, void *data)
{

	int len = 0;

	struct net_device *p_dev;

	CpmacNetDevice *hDDA;

	struct net_device *cpmac_dev_list[cpmac_devices_installed];

	int i;

	len +=
	    sprintf(buf + len, "EMAC devices = %d\n", cpmac_devices_installed);

	p_dev = last_cpmac_device;

	/* Reverse the the device link list to list eth0,eth1...in correct order */
	for (i = 0; i < cpmac_devices_installed; i++) {

		cpmac_dev_list[cpmac_devices_installed - (i + 1)] = p_dev;

		hDDA = NETDEV_PRIV(p_dev);

		p_dev = hDDA->nextDevice;

	}

	for (i = 0; i < cpmac_devices_installed; i++) {

		p_dev = cpmac_dev_list[i];

		hDDA = NETDEV_PRIV(p_dev);

		/*  This prints them out from high to low because of how the devices are linked */
		if (netif_carrier_ok(p_dev)) {

			len +=
			    sprintf(buf + len,
				    "eth%d: Link State: %s    Phy %d, Speed = %s, Duplex = %s\n",
				    hDDA->instanceNum, "UP",
				    hDDA->ddcStatus.PhyNum,
				    (hDDA->linkSpeed ==
				     100000000) ? "100" : "10",
				    (hDDA->linkMode == 2) ? "Half" : "Full");

		}

		else {

			len +=
			    sprintf(buf + len, "eth%d: Link State: DOWN\n",
				    hDDA->instanceNum);

		}

		p_dev = hDDA->nextDevice;

	}

	return len;

}

/* Dump Configuration information for debug purposes */
static int cpmac_dump_config(char *buf, char **start, off_t offset, int count,
			     int *eof, void *data)
{

	int len = 0;

	struct net_device *p_dev;

	struct net_device *cpmac_dev_list[cpmac_devices_installed];

	int i;

	CpmacNetDevice *hDDA;

	len +=
	    sprintf(buf + len, "EMAC devices = %d\n", cpmac_devices_installed);

	p_dev = last_cpmac_device;

	/* Reverse the the device link list to list eth0,eth1...in correct order */
	for (i = 0; i < cpmac_devices_installed; i++) {

		cpmac_dev_list[cpmac_devices_installed - (i + 1)] = p_dev;

		hDDA = NETDEV_PRIV(p_dev);

		p_dev = hDDA->nextDevice;

	}

	for (i = 0; i < cpmac_devices_installed; i++) {

		p_dev = cpmac_dev_list[i];

		hDDA = NETDEV_PRIV(p_dev);

		len +=
		    sprintf(buf + len,
			    "\nCPMAC Driver Internal Config Info for Unit %d\n",
			    hDDA->instanceNum);

		len +=
		    sprintf(buf + len, "vlanEnable         = %d\n",
			    hDDA->vlanEnable);

		len +=
		    sprintf(buf + len, "rxBufSize          = %d\n",
			    hDDA->rxBufSize);

		len +=
		    sprintf(buf + len, "rxBufOffset        = %d\n",
			    hDDA->rxBufOffset);

		len +=
		    sprintf(buf + len, "instId             = %d\n",
			    hDDA->initCfg.instId);

		len +=
		    sprintf(buf + len, "numTxChannels      = %d\n",
			    hDDA->initCfg.numTxChannels);

		len +=
		    sprintf(buf + len, "numRxChannels      = %d\n",
			    hDDA->initCfg.numRxChannels);

		len +=
		    sprintf(buf + len, "cpmacBusFrequency  = %d\n",
			    hDDA->initCfg.cpmacBusFrequency);

		len +=
		    sprintf(buf + len, "baseAddress        = %08X\n",
			    hDDA->initCfg.baseAddress);

		len +=
		    sprintf(buf + len, "intrLine           = %d\n",
			    hDDA->initCfg.intrLine);

		len +=
		    sprintf(buf + len, "resetLine          = %d\n",
			    hDDA->initCfg.resetLine);

		len +=
		    sprintf(buf + len, "mdioBaseAddress    = %08X\n",
			    hDDA->initCfg.mdioBaseAddress);

		len +=
		    sprintf(buf + len, "mdioResetLine      = %d\n",
			    hDDA->initCfg.mdioResetLine);

		len +=
		    sprintf(buf + len, "mdioIntrLine       = %d\n",
			    hDDA->initCfg.mdioIntrLine);

		len +=
		    sprintf(buf + len, "PhyMask            = %08X\n",
			    hDDA->initCfg.PhyMask);

		len +=
		    sprintf(buf + len, "MLinkMask          = %08X\n",
			    hDDA->initCfg.MLinkMask);

		len +=
		    sprintf(buf + len, "MdioBusFrequency   = %d\n",
			    hDDA->initCfg.MdioBusFrequency);

		len +=
		    sprintf(buf + len, "MdioClockFrequency = %d\n",
			    hDDA->initCfg.MdioClockFrequency);

		len +=
		    sprintf(buf + len, "MdioTickMSec       = %d\n",
			    hDDA->initCfg.MdioTickMSec);

		len +=
		    sprintf(buf + len, "phyMode            = %d\n",
			    hDDA->initCfg.phyMode);

		len +=
		    sprintf(buf + len, "passCRC            = %d\n",
			    hDDA->initCfg.rxCfg.passCRC);

		len +=
		    sprintf(buf + len, "qosEnable          = %d\n",
			    hDDA->initCfg.rxCfg.qosEnable);

		len +=
		    sprintf(buf + len, "noBufferChaining   = %d\n",
			    hDDA->initCfg.rxCfg.noBufferChaining);

		len +=
		    sprintf(buf + len, "copyMACCntrlFrsEne = %d\n",
			    hDDA->initCfg.rxCfg.copyMACControlFramesEnable);

		len +=
		    sprintf(buf + len, "copyShortFramesEn  = %d\n",
			    hDDA->initCfg.rxCfg.copyShortFramesEnable);

		len +=
		    sprintf(buf + len, "copyErrorFramesEn  = %d\n",
			    hDDA->initCfg.rxCfg.copyErrorFramesEnable);

		len +=
		    sprintf(buf + len, "promiscousEnable   = %d\n",
			    hDDA->initCfg.rxCfg.promiscousEnable);

		len +=
		    sprintf(buf + len, "promiscousChannel  = %d\n",
			    hDDA->initCfg.rxCfg.promiscousChannel);

		len +=
		    sprintf(buf + len, "broadcastEnable    = %d\n",
			    hDDA->initCfg.rxCfg.broadcastEnable);

		len +=
		    sprintf(buf + len, "broadcastChannel   = %d\n",
			    hDDA->initCfg.rxCfg.broadcastChannel);

		len +=
		    sprintf(buf + len, "multicastEnable    = %d\n",
			    hDDA->initCfg.rxCfg.multicastEnable);

		len +=
		    sprintf(buf + len, "multicastChannel   = %d\n",
			    hDDA->initCfg.rxCfg.multicastChannel);

		len +=
		    sprintf(buf + len, "maxRxPktLength     = %d\n",
			    hDDA->initCfg.rxCfg.maxRxPktLength);

		len +=
		    sprintf(buf + len, "bufferOffset       = %d\n",
			    hDDA->initCfg.rxCfg.bufferOffset);

		len +=
		    sprintf(buf + len, "pType              = %d\n",
			    hDDA->initCfg.macCfg.pType);

		len +=
		    sprintf(buf + len, "txShortGapEnable   = %d\n",
			    hDDA->initCfg.macCfg.txShortGapEnable);

		len +=
		    sprintf(buf + len, "gigaBitEnable      = %d\n",
			    hDDA->initCfg.macCfg.gigaBitEnable);

		len +=
		    sprintf(buf + len, "txPacingEnable     = %d\n",
			    hDDA->initCfg.macCfg.txPacingEnable);

		len +=
		    sprintf(buf + len, "miiEnable          = %d\n",
			    hDDA->initCfg.macCfg.miiEnable);

		len +=
		    sprintf(buf + len, "txFlowEnable       = %d\n",
			    hDDA->initCfg.macCfg.txFlowEnable);

		len +=
		    sprintf(buf + len, "rxFlowEnable       = %d\n",
			    hDDA->initCfg.macCfg.rxFlowEnable);

		len +=
		    sprintf(buf + len, "loopbackEnable     = %d\n",
			    hDDA->initCfg.macCfg.loopbackEnable);

		len +=
		    sprintf(buf + len, "fullDuplexEnable   = %d\n",
			    hDDA->initCfg.macCfg.fullDuplexEnable);

		p_dev = hDDA->nextDevice;

	}

	return len;

}

/* Read stats */
static int cpmac_p_read_stats(char *buf, char **start, off_t offset, int count,
			      int *eof, void *data)
{

	struct net_device *p_dev = last_cpmac_device;

	int len = 0;

	int limit = count - 80;

	int i;

	struct net_device *cpmac_dev_list[cpmac_devices_installed];

	CpmacNetDevice *hDDA;

	CpmacHwStatistics *p_deviceMib;

	/* Reverse the the device link list to list eth0,eth1...in correct order */
	for (i = 0; i < cpmac_devices_installed; i++) {

		cpmac_dev_list[cpmac_devices_installed - (i + 1)] = p_dev;

		hDDA = NETDEV_PRIV(p_dev);

		p_dev = hDDA->nextDevice;

	}

	for (i = 0; i < cpmac_devices_installed; i++) {

		p_dev = cpmac_dev_list[i];

		if (!p_dev)

			goto proc_error;

		/* Get Stats */
		cpmac_p_update_statistics(p_dev, NULL, 0, NULL);

		hDDA = NETDEV_PRIV(p_dev);

		p_deviceMib = &hDDA->deviceMib;

		/* Transmit stats */
		if (len <= limit)

			len +=
			    sprintf(buf + len, "\nCpmac %d, Address %lx\n",
				    i + 1, p_dev->base_addr);

		if (len <= limit)

			len += sprintf(buf + len, " Transmit Stats\n");

		if (len <= limit)

			len +=
			    sprintf(buf + len,
				    "   Tx Valid Bytes Sent        :%u\n",
				    p_deviceMib->ifOutOctets);

		if (len <= limit)

			len +=
			    sprintf(buf + len,
				    "   Good Tx Frames (Hardware)  :%u\n",
				    p_deviceMib->ifOutGoodFrames);

		if (len <= limit)

			len +=
			    sprintf(buf + len,
				    "   Good Tx Frames (Software)  :%lu\n",
				    hDDA->netDevStats.tx_packets);

		if (len <= limit)

			len +=
			    sprintf(buf + len,
				    "   Good Tx Broadcast Frames   :%u\n",
				    p_deviceMib->ifOutBroadcasts);

		if (len <= limit)

			len +=
			    sprintf(buf + len,
				    "   Good Tx Multicast Frames   :%u\n",
				    p_deviceMib->ifOutMulticasts);

		if (len <= limit)

			len +=
			    sprintf(buf + len,
				    "   Pause Frames Sent          :%u\n",
				    p_deviceMib->ifOutPauseFrames);

		if (len <= limit)

			len +=
			    sprintf(buf + len,
				    "   Collisions                 :%u\n",
				    p_deviceMib->ifCollisionFrames);

		if (len <= limit)

			len +=
			    sprintf(buf + len,
				    "   Tx Error Frames            :%lu\n",
				    hDDA->netDevStats.tx_errors);

		if (len <= limit)

			len +=
			    sprintf(buf + len,
				    "   Carrier Sense Errors       :%u\n",
				    p_deviceMib->ifCarrierSenseErrors);

		if (len <= limit)

			len += sprintf(buf + len, "\n");

		/* Receive Stats */
		if (len <= limit)

			len +=
			    sprintf(buf + len, "\nCpmac %d, Address %lx\n",
				    i + 1, p_dev->base_addr);

		if (len <= limit)

			len += sprintf(buf + len, " Receive Stats\n");

		if (len <= limit)

			len +=
			    sprintf(buf + len,
				    "   Rx Valid Bytes Received    :%u\n",
				    p_deviceMib->ifInOctets);

		if (len <= limit)

			len +=
			    sprintf(buf + len,
				    "   Good Rx Frames (Hardware)  :%u\n",
				    p_deviceMib->ifInGoodFrames);

		if (len <= limit)

			len +=
			    sprintf(buf + len,
				    "   Good Rx Frames (Software)  :%lu\n",
				    hDDA->netDevStats.rx_packets);

		if (len <= limit)

			len +=
			    sprintf(buf + len,
				    "   Good Rx Broadcast Frames   :%u\n",
				    p_deviceMib->ifInBroadcasts);

		if (len <= limit)

			len +=
			    sprintf(buf + len,
				    "   Good Rx Multicast Frames   :%u\n",
				    p_deviceMib->ifInMulticasts);

		if (len <= limit)

			len +=
			    sprintf(buf + len,
				    "   Pause Frames Received      :%u\n",
				    p_deviceMib->ifInPauseFrames);

		if (len <= limit)

			len +=
			    sprintf(buf + len,
				    "   Rx CRC Errors              :%u\n",
				    p_deviceMib->ifInCRCErrors);

		if (len <= limit)

			len +=
			    sprintf(buf + len,
				    "   Rx Align/Code Errors       :%u\n",
				    p_deviceMib->ifInAlignCodeErrors);

		if (len <= limit)

			len +=
			    sprintf(buf + len,
				    "   Rx Jabbers                 :%u\n",
				    p_deviceMib->ifInOversizedFrames);

		if (len <= limit)

			len +=
			    sprintf(buf + len,
				    "   Rx Filtered Frames         :%u\n",
				    p_deviceMib->ifInFilteredFrames);

		if (len <= limit)

			len +=
			    sprintf(buf + len,
				    "   Rx Fragments               :%u\n",
				    p_deviceMib->ifInFragments);

		if (len <= limit)

			len +=
			    sprintf(buf + len,
				    "   Rx Undersized Frames       :%u\n",
				    p_deviceMib->ifInUndersizedFrames);

		if (len <= limit)

			len +=
			    sprintf(buf + len,
				    "   Rx Overruns                :%u\n",
				    p_deviceMib->ifRxDMAOverruns);

	}

	return len;

      proc_error:
	*eof = 1;

	return len;

}

/* Write stats */
static int cpmac_p_write_stats(struct file *fp, const char *buf,
			       unsigned long count, void *data)
{

	char local_buf[31];

	int ret_val = 0;

	if (count > 30) {

		printk("Error : Buffer Overflow\n");

		printk("Use \"echo 0 > cpmac_stat\" to reset the statistics\n");

		return -EFAULT;

	}

	copy_from_user(local_buf, buf, count);

	local_buf[count - 1] = '\0';	/* Ignoring last \n char */

	ret_val = count;

	if (strcmp("0", local_buf) == 0) {

		struct net_device *p_dev = last_cpmac_device;

		int i;

		struct net_device *cpmac_dev_list[cpmac_devices_installed];

		CpmacNetDevice *hDDA;

		/* Valid command */
		printk("Resetting statistics for CPMAC interface.\n");

		/* Reverse the the device link list to list eth0,eth1...in correct order */
		for (i = 0; i < cpmac_devices_installed; i++) {

			cpmac_dev_list[cpmac_devices_installed - (i + 1)] =
			    p_dev;

			hDDA = NETDEV_PRIV(p_dev);

			p_dev = hDDA->nextDevice;

		}

		for (i = 0; i < cpmac_devices_installed; i++) {

			p_dev = cpmac_dev_list[i];

			if (!p_dev) {

				ret_val = -EFAULT;

				break;

			}

			cpmac_p_reset_statistics(p_dev);

		}

	}

	else {

		printk("Error: Unknown operation on cpmac statistics\n");

		printk("Use \"echo 0 > emac_stats\" to reset the statistics\n");

		return -EFAULT;

	}

	return ret_val;

}

/* Update RFC2665 Statistics */
static int cpmac_p_read_rfc2665_stats(char *buf, char **start, off_t offset,
				      int count, int *eof, void *data)
{

	int limit = count - 80;

	int len = 0;

	struct net_device *p_dev = (struct net_device *)data;

	cpmac_p_update_statistics(p_dev, buf, limit, &len);

	*eof = 1;

	return len;

}

/* Reset Statistics */
static int cpmac_p_reset_statistics(struct net_device *p_dev)
{

	CpmacNetDevice *hDDA = NETDEV_PRIV(p_dev);

	memset(&hDDA->deviceMib, 0, sizeof(CpmacHwStatistics));

	memset(&hDDA->deviceStats, 0, sizeof(CpmacDrvStats));

	memset(&hDDA->netDevStats, 0, sizeof(struct net_device_stats));

	/* Clear statistics */
	if (hDDA->ddcIf->ddcNetIf.ddcFuncTable.
	    ddcControl(hDDA->hDDC, CPMAC_DDC_IOCTL_CLR_STATISTICS, NULL, NULL)
	    != CPMAC_SUCCESS) {

		errPrint
		    ("cpmac_p_reset_statistics: Error clearing statistics in DDC for %s\n",
		     p_dev->name);

		return (-1);

	}

	return (0);

}

/* Update Statistics */
static int cpmac_p_update_statistics(struct net_device *p_dev, char *buf,
				     int limit, int *p_len)
{

	unsigned long rx_hal_errors = 0;

	unsigned long rx_hal_discards = 0;

	unsigned long tx_hal_errors = 0;

	unsigned long ifOutDiscards = 0;

	unsigned long ifInDiscards = 0;

	unsigned long ifOutErrors = 0;

	unsigned long ifInErrors = 0;

	CpmacNetDevice *hDDA = NETDEV_PRIV(p_dev);

	CpmacHwStatistics *p_deviceMib = &hDDA->deviceMib;

	CpmacDrvStats *p_stats = &hDDA->deviceStats;

	CpmacHwStatistics local_mib;

	CpmacHwStatistics *p_local_mib = &local_mib;

	struct net_device_stats *p_netDevStats = &hDDA->netDevStats;

	int len = 0;

	int dev_mib_elem_count = 0;

	/* do not access the hardware if it is in the reset state. */
	if (!test_bit(0, &hDDA->setToClose)) {

		/* Get hardware statistics from DDC */
		if (hDDA->ddcIf->ddcNetIf.ddcFuncTable.
		    ddcControl(hDDA->hDDC, CPMAC_DDC_IOCTL_GET_STATISTICS,
			       (Ptr) p_local_mib, NULL)
		    != CPMAC_SUCCESS) {

			errPrint("Error getting statistics from DDC for %s\n",
				 p_dev->name);

			return (-1);

		}

		dev_mib_elem_count =
		    sizeof(CpmacHwStatistics) / sizeof(unsigned long);

		/* Update the history of the stats. This takes care of any reset of the 
		 * device and stats that might have taken place during the life time of 
		 * the driver. 
		 */
		while (dev_mib_elem_count--) {

			*((unsigned long *)p_deviceMib + dev_mib_elem_count) =
			    *((unsigned long *)p_local_mib +
			      dev_mib_elem_count);

		}
	}

	/* RFC2665, section 3.2.7, page 9 */
	rx_hal_errors =
	    p_deviceMib->ifInFragments +
	    p_deviceMib->ifInCRCErrors +
	    p_deviceMib->ifInAlignCodeErrors + p_deviceMib->ifInJabberFrames;

	/* RFC2233 */
	rx_hal_discards = p_deviceMib->ifRxDMAOverruns;

	/* RFC2665, section 3.2.7, page 9 */
	tx_hal_errors =
	    p_deviceMib->ifExcessiveCollisionFrames +
	    p_deviceMib->ifLateCollisions +
	    p_deviceMib->ifCarrierSenseErrors + p_deviceMib->ifOutUnderrun;

	/* if not set, the short frames (< 64 bytes) are considered as errors */
	if (hDDA->initCfg.rxCfg.copyShortFramesEnable == FALSE)

		rx_hal_errors += p_deviceMib->ifInUndersizedFrames;

	/* All frames greater than max rx frame length set in hardware should be considered error frames
	 * RFC2665, section 3.2.7, page 9. */
	rx_hal_errors += p_deviceMib->ifInOversizedFrames;

	/* if not in promiscous, then non addr matching frames are discarded */
	/* CPMAC 2.0 Manual Section 2.8.1.14 */
	if (hDDA->initCfg.rxCfg.promiscousEnable == FALSE) {

		ifInDiscards += p_deviceMib->ifInFilteredFrames;

	}

	/* total rx discards = hal discards */
	ifInDiscards = rx_hal_discards;
	p_netDevStats->rx_dropped = rx_hal_discards;
	p_netDevStats->rx_crc_errors = p_deviceMib->ifInCRCErrors;
	p_netDevStats->rx_frame_errors = p_deviceMib->ifInAlignCodeErrors;

	p_netDevStats->multicast = p_deviceMib->ifInMulticasts;

	ifInErrors = rx_hal_errors;

	ifOutErrors = tx_hal_errors;

	ifOutDiscards = p_netDevStats->tx_dropped;

	/* Let us update the net device stats struct. To be updated in the later releases. */
	hDDA->netDevStats.rx_errors = ifInErrors;

	hDDA->netDevStats.collisions = p_deviceMib->ifCollisionFrames;

	hDDA->netDevStats.tx_carrier_errors = p_deviceMib->ifCarrierSenseErrors;

	if (buf == NULL || limit == 0) {

		return (0);

	}

	if (len <= limit)

		len +=
		    sprintf(buf + len, "%-35s: %u\n", "ifSpeed",
			    hDDA->linkSpeed);

	if (len <= limit)

		len +=
		    sprintf(buf + len, "%-35s: %u\n", "dot3StatsDuplexStatus",
			    hDDA->linkMode);

	if (len <= limit)

		len +=
		    sprintf(buf + len, "%-35s: %u\n", "ifAdminStatus",
			    (p_dev->flags & IFF_UP ? 1 : 2));

	if (len <= limit)

		len +=
		    sprintf(buf + len, "%-35s: %u\n", "ifOperStatus",
			    (((p_dev->flags & IFF_UP)
			      && netif_carrier_ok(p_dev)) ? 1 : 2));

	if (len <= limit)

		len +=
		    sprintf(buf + len, "%-35s: %lu\n", "ifLastChange",
			    p_stats->start_tick);

	if (len <= limit)

		len +=
		    sprintf(buf + len, "%-35s: %lu\n", "ifInDiscards",
			    ifInDiscards);

	if (len <= limit)

		len +=
		    sprintf(buf + len, "%-35s: %lu\n", "ifInErrors",
			    ifInErrors);

	if (len <= limit)

		len +=
		    sprintf(buf + len, "%-35s: %lu\n", "ifOutDiscards",
			    ifOutDiscards);

	if (len <= limit)

		len +=
		    sprintf(buf + len, "%-35s: %lu\n", "ifOutErrors",
			    ifOutErrors);

	if (len <= limit)

		len +=
		    sprintf(buf + len, "%-35s: %u\n", "ifInGoodFrames",
			    p_deviceMib->ifInGoodFrames);

	if (len <= limit)

		len +=
		    sprintf(buf + len, "%-35s: %u\n", "ifInBroadcasts",
			    p_deviceMib->ifInBroadcasts);

	if (len <= limit)

		len +=
		    sprintf(buf + len, "%-35s: %u\n", "ifInMulticasts",
			    p_deviceMib->ifInMulticasts);

	if (len <= limit)

		len +=
		    sprintf(buf + len, "%-35s: %u\n", "ifInPauseFrames",
			    p_deviceMib->ifInPauseFrames);

	if (len <= limit)

		len +=
		    sprintf(buf + len, "%-35s: %u\n", "ifInCRCErrors",
			    p_deviceMib->ifInCRCErrors);

	if (len <= limit)

		len +=
		    sprintf(buf + len, "%-35s: %u\n", "ifInAlignCodeErrors",
			    p_deviceMib->ifInAlignCodeErrors);

	if (len <= limit)

		len +=
		    sprintf(buf + len, "%-35s: %u\n", "ifInOversizedFrames",
			    p_deviceMib->ifInOversizedFrames);

	if (len <= limit)

		len +=
		    sprintf(buf + len, "%-35s: %u\n", "ifInJabberFrames",
			    p_deviceMib->ifInJabberFrames);

	if (len <= limit)

		len +=
		    sprintf(buf + len, "%-35s: %u\n", "ifInUndersizedFrames",
			    p_deviceMib->ifInUndersizedFrames);

	if (len <= limit)

		len +=
		    sprintf(buf + len, "%-35s: %u\n", "ifInFragments",
			    p_deviceMib->ifInFragments);

	if (len <= limit)

		len +=
		    sprintf(buf + len, "%-35s: %u\n", "ifInFilteredFrames",
			    p_deviceMib->ifInFilteredFrames);

	if (len <= limit)

		len +=
		    sprintf(buf + len, "%-35s: %u\n", "ifInQosFilteredFrames",
			    p_deviceMib->ifInQosFilteredFrames);

	if (len <= limit)

		len +=
		    sprintf(buf + len, "%-35s: %u\n", "ifInOctets",
			    p_deviceMib->ifInOctets);

	if (len <= limit)

		len +=
		    sprintf(buf + len, "%-35s: %u\n", "ifOutGoodFrames",
			    p_deviceMib->ifOutGoodFrames);

	if (len <= limit)

		len +=
		    sprintf(buf + len, "%-35s: %u\n", "ifOutBroadcasts",
			    p_deviceMib->ifOutBroadcasts);

	if (len <= limit)

		len +=
		    sprintf(buf + len, "%-35s: %u\n", "ifOutMulticasts",
			    p_deviceMib->ifOutMulticasts);

	if (len <= limit)

		len +=
		    sprintf(buf + len, "%-35s: %u\n", "ifOutPauseFrames",
			    p_deviceMib->ifOutPauseFrames);

	if (len <= limit)

		len +=
		    sprintf(buf + len, "%-35s: %u\n", "ifDeferredTransmissions",
			    p_deviceMib->ifDeferredTransmissions);

	if (len <= limit)

		len +=
		    sprintf(buf + len, "%-35s: %u\n", "ifCollisionFrames",
			    p_deviceMib->ifCollisionFrames);

	if (len <= limit)

		len +=
		    sprintf(buf + len, "%-35s: %u\n", "ifSingleCollisionFrames",
			    p_deviceMib->ifSingleCollisionFrames);

	if (len <= limit)

		len +=
		    sprintf(buf + len, "%-35s: %u\n",
			    "ifMultipleCollisionFrames",
			    p_deviceMib->ifMultipleCollisionFrames);

	if (len <= limit)

		len +=
		    sprintf(buf + len, "%-35s: %u\n",
			    "ifExcessiveCollisionFrames",
			    p_deviceMib->ifExcessiveCollisionFrames);

	if (len <= limit)

		len +=
		    sprintf(buf + len, "%-35s: %u\n", "ifLateCollisions",
			    p_deviceMib->ifLateCollisions);

	if (len <= limit)

		len +=
		    sprintf(buf + len, "%-35s: %u\n", "ifOutUnderrun",
			    p_deviceMib->ifOutUnderrun);

	if (len <= limit)

		len +=
		    sprintf(buf + len, "%-35s: %u\n", "ifCarrierSenseErrors",
			    p_deviceMib->ifCarrierSenseErrors);

	if (len <= limit)

		len +=
		    sprintf(buf + len, "%-35s: %u\n", "ifOutOctets",
			    p_deviceMib->ifOutOctets);

	if (len <= limit)

		len +=
		    sprintf(buf + len, "%-35s: %u\n", "if64OctetFrames",
			    p_deviceMib->if64OctetFrames);

	if (len <= limit)

		len +=
		    sprintf(buf + len, "%-35s: %u\n", "if65To127POctetFrames",
			    p_deviceMib->if65To127OctetFrames);

	if (len <= limit)

		len +=
		    sprintf(buf + len, "%-35s: %u\n", "if128To255OctetFrames",
			    p_deviceMib->if128To255OctetFrames);

	if (len <= limit)

		len +=
		    sprintf(buf + len, "%-35s: %u\n", "if256To511OctetFrames",
			    p_deviceMib->if256To511OctetFrames);

	if (len <= limit)

		len +=
		    sprintf(buf + len, "%-35s: %u\n", "if512To1023OctetFrames",
			    p_deviceMib->if512To1023OctetFrames);

	if (len <= limit)

		len +=
		    sprintf(buf + len, "%-35s: %u\n", "if1024ToUpOctetFrames",
			    p_deviceMib->if1024ToUPOctetFrames);

	if (len <= limit)

		len +=
		    sprintf(buf + len, "%-35s: %u\n", "ifNetOctets",
			    p_deviceMib->ifNetOctets);

	if (len <= limit)

		len +=
		    sprintf(buf + len, "%-35s: %u\n", "ifRxSofOverruns",
			    p_deviceMib->ifRxSofOverruns);

	if (len <= limit)

		len +=
		    sprintf(buf + len, "%-35s: %u\n", "ifRxMofOverruns",
			    p_deviceMib->ifRxMofOverruns);

	if (len <= limit)

		len +=
		    sprintf(buf + len, "%-35s: %u\n", "ifRxDMAOverruns",
			    p_deviceMib->ifRxDMAOverruns);

	*p_len = len;

	return (0);

}

/* Version info */
static int cpmac_p_get_version(char *buf, char **start, off_t offset, int count,
			       int *eof, void *data)
{

	int len = 0;

	int limit = count - 80;

	unsigned int ddc_version = 0;

	char *ddc_version_string = NULL;

	ddc_version_string = DDC_cpmacGetVersionInfo(&ddc_version);

	len +=
	    sprintf(buf + len, "Texas Instruments : %s\n",
		    cpmac_DDA_version_string);

	if (len <= limit && ddc_version_string)

		len +=
		    sprintf(buf + len, "Texas Instruments : %s\n",
			    ddc_version_string);

	return len;

}

/* Tick Timer */
static void cpmac_p_tick_timer_expiry(CpmacNetDevice * hDDA)
{

	struct timer_list *p_timer = &hDDA->periodicTimer;

	if (test_bit(0, &hDDA->setToClose)) {

		return;

	}

	if (hDDA->timerActive == TRUE) {

		hDDA->ddcIf->ddctick(hDDA->hDDC, NULL);

		/* Restart the timer */
		p_timer->expires = jiffies + hDDA->periodicTicks;

		add_timer(p_timer);

	}

}

/* Mib Timer */
static void cpmac_p_mib_timer_expiry(CpmacNetDevice * hDDA)
{

	struct timer_list *p_timer = &hDDA->mibTimer;

	if (test_bit(0, &hDDA->setToClose)) {

		return;

	}

	if (hDDA->mibTimerActive == TRUE) {

		hDDA->ddcIf->ddcNetIf.ddcFuncTable.ddcControl(hDDA->hDDC,
							      CPMAC_DDC_IOCTL_IF_PARAMS_UPDT,
							      NULL, NULL);

		/* Restart the timer */
		p_timer->expires = jiffies + hDDA->mibTicks;

		add_timer(p_timer);

	}

}

/******************************************************************************
 *  Device enable/disable functions
 *****************************************************************************/

/* Enable the device - Init TX/RX Channels and Open DDC */
static int cpmac_p_dev_enable(CpmacNetDevice * hDDA)
{

	PAL_Result retCode;

	struct net_device *p_dev = hDDA->owner;

	hDDA->setToClose = 0;

	/* Create a TX channel */
	retCode =
	    hDDA->ddcIf->ddcNetIf.ddcNetChOpen((DDC_Handle *) hDDA->hDDC,
					       (DDC_NetChInfo *) & hDDA->
					       txChInfo[0], NULL);

	if (retCode != CPMAC_SUCCESS) {

		errPrint
		    ("%s error: Error %08X from CPMAC DDC TX Channel Open()\n",
		     p_dev->name, retCode);

		return (-1);

	}

	/* Create a RX channel - Note that last param contains mac address */
	retCode =
	    hDDA->ddcIf->ddcNetIf.ddcNetChOpen((DDC_Handle *) hDDA->hDDC,
					       (DDC_NetChInfo *) & hDDA->
					       rxChInfo[0],
					       (Ptr) & hDDA->macAddr[0]);

	if (retCode != CPMAC_SUCCESS) {

		errPrint
		    ("%s error: Error %08X from CPMAC DDC RX Channel Open()\n",
		     p_dev->name, retCode);

		return (-1);

	}

	/* Open DDC instance */
	retCode = hDDA->ddcIf->ddcNetIf.ddcFuncTable.ddcOpen(hDDA->hDDC, NULL);

	if (retCode != CPMAC_SUCCESS) {

		errPrint("%s error: Error %08X from CPMAC DDC Open()\n",
			 p_dev->name, retCode);

		return (-1);

	}

	return (0);

}

/* Disable the device - teardown chanels and close DDC */
static int cpmac_p_dev_disable(CpmacNetDevice * hDDA)
{

	PAL_Result retCode;

	struct net_device *p_dev = hDDA->owner;

	/* inform the upper layers. */
	netif_stop_queue(hDDA->owner);

	/* Prepare to close */
	set_bit(0, &hDDA->setToClose);

	/* Closing the DDC instance will close all channels also */
	retCode = hDDA->ddcIf->ddcNetIf.ddcFuncTable.ddcClose(hDDA->hDDC, NULL);

	if (retCode != CPMAC_SUCCESS) {

		errPrint("%s error: Error %08X from CPMAC DDC Close()\n",
			 p_dev->name, retCode);

		return (-1);

	}

	else {

		/* DDC should turn off the timer, but just in case */
		if (hDDA->timerActive != FALSE) {

			del_timer_sync(&hDDA->periodicTimer);

			hDDA->timerActive = FALSE;

		}

		dbgPrint("TI DaVinci EMAC: Device %s Closed.\n", p_dev->name);

		hDDA->deviceStats.start_tick = jiffies;

		hDDA->linkSpeed = 100000000;

		hDDA->linkMode = 1;

		netif_carrier_off(p_dev);

	}

	return (0);

}

/******************************************************************************
 *  Net Device functions
 *****************************************************************************/

/* Get statistics */
static struct net_device_stats *cpmac_dev_get_net_stats(struct net_device
							*p_dev)
{

	CpmacNetDevice *hDDA = NETDEV_PRIV(p_dev);

	cpmac_p_update_statistics(p_dev, NULL, 0, NULL);

	return &hDDA->netDevStats;

}

/* Set multicast address in the driver */
static void cpmac_dev_mcast_set(struct net_device *p_dev)
{

	CpmacNetDevice *hDDA = NETDEV_PRIV(p_dev);

	if (p_dev->flags & IFF_PROMISC) {

		/* Enable promiscous mode */
		hDDA->initCfg.rxCfg.promiscousEnable = TRUE;

		hDDA->ddcIf->ddcNetIf.ddcFuncTable.ddcControl(hDDA->hDDC,
							      CPMAC_DDC_IOCTL_SET_RXCFG,
							      (Ptr) & hDDA->
							      initCfg.rxCfg,
							      NULL);

	}

	else if ((p_dev->flags & IFF_ALLMULTI)
		 || (p_dev->mc_count >
		     CPMAC_DDA_DEFAULT_MAX_MULTICAST_ADDRESSES)) {

		/* Enable multicast - disable promiscous */
		hDDA->initCfg.rxCfg.promiscousEnable = FALSE;

		hDDA->initCfg.rxCfg.multicastEnable = TRUE;

		hDDA->ddcIf->ddcNetIf.ddcFuncTable.ddcControl(hDDA->hDDC,
							      CPMAC_DDC_IOCTL_SET_RXCFG,
							      (Ptr) & hDDA->
							      initCfg.rxCfg,
							      NULL);

		/* Enable all multicast addresses */
		hDDA->ddcIf->ddcNetIf.ddcFuncTable.ddcControl(hDDA->hDDC,
							      CPMAC_DDC_IOCTL_ALL_MULTI,
							      (Ptr)
							      CPMAC_ALL_MULTI_SET,
							      NULL);

	}

	else if (p_dev->mc_count == 0) {

		/* Only unicast mode to be set - clear promiscous and clear multicast modes */
		hDDA->ddcIf->ddcNetIf.ddcFuncTable.ddcControl(hDDA->hDDC,
							      CPMAC_DDC_IOCTL_ALL_MULTI,
							      (Ptr)
							      CPMAC_ALL_MULTI_CLR,
							      NULL);

		/* Disable promiscous and multicast modes */
		hDDA->initCfg.rxCfg.promiscousEnable = FALSE;

		hDDA->initCfg.rxCfg.multicastEnable = FALSE;

		hDDA->ddcIf->ddcNetIf.ddcFuncTable.ddcControl(hDDA->hDDC,
							      CPMAC_DDC_IOCTL_SET_RXCFG,
							      (Ptr) & hDDA->
							      initCfg.rxCfg,
							      NULL);

	}

	else if (p_dev->mc_count) {

		struct dev_mc_list *mc_ptr;

		/* Clear multicast list first */
		hDDA->ddcIf->ddcNetIf.ddcFuncTable.ddcControl(hDDA->hDDC,
							      CPMAC_DDC_IOCTL_ALL_MULTI,
							      (Ptr)
							      CPMAC_ALL_MULTI_CLR,
							      NULL);

		/* Enable multicast - disable promiscous */
		hDDA->initCfg.rxCfg.promiscousEnable = FALSE;

		hDDA->initCfg.rxCfg.multicastEnable = TRUE;

		hDDA->ddcIf->ddcNetIf.ddcFuncTable.ddcControl(hDDA->hDDC,
							      CPMAC_DDC_IOCTL_SET_RXCFG,
							      (Ptr) & hDDA->
							      initCfg.rxCfg,
							      NULL);

		/* Program multicast address list into CPMAC hardware using DDC */
		for (mc_ptr = p_dev->mc_list; mc_ptr; mc_ptr = mc_ptr->next) {

			/* Add ethernet address to hardware list via DDC ioctl */
			hDDA->ddcIf->ddcNetIf.ddcFuncTable.ddcControl(hDDA->
								      hDDC,
								      CPMAC_DDC_IOCTL_MULTICAST_ADDR,
								      (Ptr)
								      CPMAC_MULTICAST_ADD,
								      (Ptr)
								      mc_ptr->
								      dmi_addr);

		}

	}

	else {

		dbgPrint("TI DaVinci EMAC: %s:No Multicast address to set.\n",
			 p_dev->name);

	}

}

static int cpmac_dev_set_mac_addr(struct net_device *p_dev, void *addr)
{

	int retCode;

	CpmacAddressParams AddressParams;

	struct sockaddr *sa = addr;

	CpmacNetDevice *hDDA = NETDEV_PRIV(p_dev);

	AddressParams.channel = CPMAC_DDA_DEFAULT_RX_CHANNEL;

	AddressParams.macAddress = sa->sa_data;

	retCode =
	    hDDA->ddcIf->ddcNetIf.ddcFuncTable.ddcControl(hDDA->hDDC,
							  CPMAC_DDC_IOCTL_SET_MAC_ADDRESS,
							  (CpmacAddressParams
							   *) & AddressParams,
							  NULL);

	if (retCode != CPMAC_SUCCESS) {

		errPrint
		    ("%s error: Error %08X from CPMAC DDC TX Channel Open()\n",
		     p_dev->name, retCode);

		return -EIO;

	}

	memcpy(hDDA->macAddr, sa->sa_data, p_dev->addr_len);

	memcpy(p_dev->dev_addr, sa->sa_data, p_dev->addr_len);

	return 0;

}

static void cpmac_tx_timeout(struct net_device *p_dev)
{

	int retCode;

	CpmacNetDevice *hDDA = NETDEV_PRIV(p_dev);

	printk("EMAC Tx Timeout: Closing TX channel\n");

	hDDA->ddcIf->ddcNetIf.ddcNetChClose(hDDA->hDDC,
					    hDDA->txChInfo[0].chNum,
					    hDDA->txChInfo[0].chDir, 0);

	printk("EMAC Tx Timeout: Opening TX channel\n");

	retCode =
	    hDDA->ddcIf->ddcNetIf.ddcNetChOpen((DDC_Handle *) hDDA->hDDC,
					       (DDC_NetChInfo *) & hDDA->
					       txChInfo[0], NULL);

	if (retCode != CPMAC_SUCCESS) {

		errPrint
		    ("%s error: Error %08X from CPMAC DDC TX Channel Open()\n",
		     p_dev->name, retCode);

	}

	else {

		errPrint
		    ("EMAC Tx Timeout: successfully closed and opened channels\n");

	}
	
	/* Update interface statistics */
	hDDA->netDevStats.tx_errors++;

}

/***************************************************************
 *  cpmac_dev_init
 *
 *  Returns:
 *      0 on success, error code otherwise.
 *  Parms:
 *      dev The structure of the device to be
 *          init'ed.
 *
 *  This function completes the initialization of the
 *  device structure and driver.  It reserves the IO
 *  addresses and assignes the device's methods.
 * 
 **************************************************************/
static int cpmac_dev_init(struct net_device *p_dev)
{

	int cnt, ddcInitStatus = 0;

	PAL_Result retCode;

	char *mac_name = NULL;

	char *mac_string = NULL;

	char *default_mac_string = NULL;

	CpmacNetDevice *hDDA = NETDEV_PRIV(p_dev);

	int instanceNum = hDDA->instanceNum;

	/* Create mac name */
	switch (instanceNum) {

	case 0:
		mac_name = CPMAC_DDA_MAC_ADDR_A;

		/* We are getting default MAC address from Bootloader */
		if (strcmp(cpmac_eth_string, "deadbeaf") == 0) {

			default_mac_string = "08.00.28.32.06.08";

		} else {

			default_mac_string = &cpmac_eth_string[0];

		}

		break;

	default:
		mac_name = "";

		default_mac_string = "08.00.28.32.06.08";

		break;

	}

	mac_string = default_mac_string;

	cpmac_str_to_ethaddr(hDDA->macAddr, mac_string);

	for (cnt = 0; cnt <= ETH_ALEN; cnt++) {

		p_dev->dev_addr[cnt] = hDDA->macAddr[cnt];	/* This sets the hardware address */

	}

	hDDA->setToClose = 1;

	/* Get configuration information for this instance */
	/* When Config service is available, use it */

	if (cpmac_net_get_config(hDDA) != 0) {

		errPrint
		    ("TI DAVINCI EMAC: Could not fetch configuration information for instance %d\n",
		     instanceNum);

		goto cpmac_dev_init_exit;

	}

	/* Create DDC Instance */
	retCode = DDC_cpmacCreateInstance(instanceNum,	/* Instance Id */
					  hDDA,	/* DDA Handle */
					  &cpmac_DDA_cb_interface,	/* Pointer to DDA Callback function table */
					  (DDC_Handle **) & hDDA->hDDC,	/* Address of pointer to DDC Handle */
					  &hDDA->ddcIf,	/* Address of pointer to DDC function table */
					  NULL);	/* Param not used */

	if (retCode != CPMAC_SUCCESS) {

		errPrint
		    ("TI DAVINCI EMAC: cpmac_dev_init:%d: Error %08X from DDC_cpmacCreateInstance()\n",
		     instanceNum, retCode);

		goto cpmac_dev_init_exit;

	}

	ddcInitStatus = 1;	/* Instance created */

	/* Initialize DDC Instance by passing initial configuration structure */
	retCode =
	    hDDA->ddcIf->ddcNetIf.ddcFuncTable.ddcInit(hDDA->hDDC,
						       &hDDA->initCfg);

	if (retCode != CPMAC_SUCCESS) {

		errPrint
		    ("TI DAVINCI EMAC: cpmac_dev_init:%d: Error %08X from DDC Init()\n",
		     instanceNum, retCode);

		goto cpmac_dev_init_exit;

	}

	ddcInitStatus = 2;	/* Instance initialized */

	/* Init spin lock */
	spin_lock_init(&hDDA->lock);

	/* Spin lock for protect macro */
	cpmac_lock = &hDDA->lock;

	/* Set as per RFC 2665 */
	hDDA->linkSpeed = 100000000;

	hDDA->linkMode = 1;

	/* initialize the timers for the net device - the timer will be started by DDC calling the Ioctl on DDA */
	init_timer(&hDDA->periodicTimer);

	hDDA->periodicTicks = 0;

	hDDA->periodicTimer.expires = 0;

	hDDA->timerActive = FALSE;

	hDDA->periodicTimer.data = (unsigned long)hDDA;

	hDDA->periodicTimer.function =
	    (timer_tick_func) cpmac_p_tick_timer_expiry;

	init_timer(&hDDA->mibTimer);

	hDDA->mibTimerActive = FALSE;

	hDDA->mibTimer.data = (unsigned long)hDDA;

	hDDA->mibTimer.function = (timer_tick_func) cpmac_p_mib_timer_expiry;

	/* Populate the device structure */
	p_dev->addr_len = 6;

	p_dev->open = cpmac_dev_open;	/*  i.e. Start Device  */

	p_dev->do_ioctl = cpmac_ioctl;

	p_dev->hard_start_xmit = cpmac_dev_tx;

	p_dev->stop = cpmac_dev_close;

	p_dev->get_stats = cpmac_dev_get_net_stats;

	p_dev->set_multicast_list = cpmac_dev_mcast_set;

	p_dev->tx_timeout = cpmac_tx_timeout;

	p_dev->set_mac_address = cpmac_dev_set_mac_addr;

	p_dev->poll = cpmac_poll;

	p_dev->weight = CPMAC_DDA_DEFAULT_RX_MAX_SERVICE;

	/* Reset the broadcast and multicast flags and enable them based upon configuration of driver */
	p_dev->flags &= ~(IFF_PROMISC | IFF_BROADCAST | IFF_MULTICAST);

	if (hDDA->initCfg.rxCfg.broadcastEnable == TRUE)

		p_dev->flags |= IFF_BROADCAST;

	if (hDDA->initCfg.rxCfg.multicastEnable == TRUE)

		p_dev->flags |= IFF_MULTICAST;

	netif_carrier_off(p_dev);

	p_dev->irq = hDDA->initCfg.intrLine;

	/* Request memory region from the kernel */
	p_dev->base_addr = hDDA->initCfg.baseAddress;

	request_mem_region(p_dev->base_addr, CPMAC_DDA_DEFAULT_CPMAC_SIZE,
			   p_dev->name);

	/* If following flag ON then Open DDC */
	if (g_init_enable_flag) {

		if (cpmac_p_dev_enable(hDDA)) {

			errPrint
			    ("%s error: cpmac_dev_init: device could not OPEN DDC\n",
			     p_dev->name);

			goto cpmac_dev_init_exit;

		}

	}

	return (0);

      cpmac_dev_init_exit:
	/* All resources allocated are freed - call the un-init sequence on DDC */
	switch (ddcInitStatus) {

	case 2:		/* Deinit DDC */

		retCode =
		    hDDA->ddcIf->ddcNetIf.ddcFuncTable.ddcDeinit(hDDA->hDDC,
								 NULL);

		if (retCode != CPMAC_SUCCESS)

			errPrint
			    ("TI DAVINCI EMAC: cpmac_dev_init_exit:%s: Error %08X from DDC Deinit()\n",
			     p_dev->name, retCode);

		/* Follow through to delete the instance also */
	case 1:		/* Delete DDC Instance */

		retCode =
		    hDDA->ddcIf->ddcNetIf.ddcFuncTable.ddcDelInst(hDDA->hDDC,
								  NULL);

		if (retCode != CPMAC_SUCCESS)

			errPrint
			    ("TI DAVINCI EMAC: cpmac_dev_init_exit:%s: Error %08X from DDC Delete Instance()\n",
			     p_dev->name, retCode);

		break;

	default:

		break;

	}

	return (-1);

}				/* cpmac_dev_init */

/******************************************************************************
 *  Device Open/Close functions
 *****************************************************************************/

/* Open the Adapter */
static int cpmac_dev_open(struct net_device *p_dev)
{

	CpmacNetDevice *hDDA = NETDEV_PRIV(p_dev);

	if (!g_init_enable_flag) {

		if (cpmac_p_dev_enable(hDDA)) {

			errPrint
			    ("%s error: cpmac_dev_open: device could not OPEN DDC\n",
			     p_dev->name);

			return (-1);

		}

	}

	if (request_irq
	    (hDDA->initCfg.intrLine, cpmac_hal_isr, SA_INTERRUPT,
	     "TI DAVINCI EMAC Driver", hDDA)) {

		errPrint
		    ("Failed to register the irq %d for TI DAVINCI EMAC %s.\n",
		     hDDA->initCfg.intrLine, p_dev->name);

		return (-1);

	}

	if (netif_carrier_ok(p_dev))

		netif_start_queue(p_dev);

	else

		netif_stop_queue(p_dev);

	hDDA->deviceStats.start_tick = jiffies;

	dbgPrint("TI DaVinci EMAC: Started the network queue for %s.\n",
		 p_dev->name);

	return (0);

}

/* Close the adapter */
static int cpmac_dev_close(struct net_device *p_dev)
{

	CpmacNetDevice *hDDA = NETDEV_PRIV(p_dev);

	if (!g_init_enable_flag)

		cpmac_p_dev_disable(hDDA);

	/* Free ISR */
	free_irq(hDDA->initCfg.intrLine, hDDA);

	return (0);

}

/* Ioctl function */
static int cpmac_ioctl(struct net_device *p_dev, struct ifreq *rq, int cmd)
{

	CpmacDrvPrivIoctl privIoctl;

	CpmacNetDevice *hDDA = NETDEV_PRIV(p_dev);

	if (cmd == SIOCDEVPRIVATE) {

		/* Copy user data */
		if (copy_from_user
		    ((char *)&privIoctl, (char *)rq->ifr_data,
		     sizeof(CpmacDrvPrivIoctl)))

			return -EFAULT;

		switch (privIoctl.cmd) {

			/* Program Type 2/3 Address Filter */
		case CPMAC_DDA_PRIV_FILTERING:

			{

				CpmacType2_3_AddrFilterParams filterParams;

				if (copy_from_user
				    ((char *)&filterParams,
				     (char *)privIoctl.data,
				     sizeof(CpmacType2_3_AddrFilterParams)))

					return -EFAULT;

				if (hDDA->ddcIf->ddcNetIf.ddcFuncTable.
				    ddcControl(hDDA->hDDC,
					       CPMAC_DDC_IOCTL_TYPE2_3_FILTERING,
					       (CpmacType2_3_AddrFilterParams
						*) & filterParams, NULL)
				    != CPMAC_SUCCESS) {

					errPrint
					    ("Failed to read params (CPMAC_DDA_PRIV_FILTERING) from DDC for  %s.\n",
					     p_dev->name);

					return -EFAULT;

				}

				break;

			}

			/* Read PHY register via MII interface */
		case CPMAC_DDA_PRIV_MII_READ:

			{

				CpmacPhyParams phyParams;

				unsigned long irq_flags;

				/* Copy user data into local variable */
				if (copy_from_user
				    ((char *)&phyParams,
				     (char *)privIoctl.data,
				     sizeof(CpmacPhyParams)))

					return -EFAULT;

				/* Make sure this function does not clash with mii access during tick function */
				save_and_cli(irq_flags);

				if (hDDA->ddcIf->ddcNetIf.ddcFuncTable.
				    ddcControl(hDDA->hDDC,
					       CPMAC_DDC_IOCTL_READ_PHY_REG,
					       (Ptr) & phyParams,
					       NULL) != CPMAC_SUCCESS) {

					errPrint
					    ("Failed to read params (CPMAC_DDA_PRIV_MII_READ) from DDC for  %s.\n",
					     p_dev->name);

					return -EFAULT;

				}

				/* Copy the local data to user space */
				if (copy_to_user
				    ((char *)privIoctl.data,
				     (char *)&phyParams,
				     sizeof(CpmacPhyParams)))

					return -EFAULT;

				/* Enable tick timer to access phy now if required */
				restore_flags(irq_flags);

			}

			break;

			/* Write PHY register via MII interface */
		case CPMAC_DDA_PRIV_MII_WRITE:

			{

				CpmacPhyParams phyParams;

				unsigned long irq_flags;

				/* Copy user data into local variable */
				if (copy_from_user
				    ((char *)&phyParams,
				     (char *)privIoctl.data,
				     sizeof(CpmacPhyParams)))

					return -EFAULT;

				/* Make sure this function does not clash with mii access during tick function */
				save_and_cli(irq_flags);

				if (hDDA->ddcIf->ddcNetIf.ddcFuncTable.
				    ddcControl(hDDA->hDDC,
					       CPMAC_DDC_IOCTL_WRITE_PHY_REG,
					       (Ptr) & phyParams,
					       NULL) != CPMAC_SUCCESS) {

					errPrint
					    ("Failed to read params (CPMAC_DDA_PRIV_MII_READ) from DDC for  %s.\n",
					     p_dev->name);

					return -EFAULT;

				}

				/* Enable tick timer to access phy now if required */
				restore_flags(irq_flags);

			}

			break;

			/* Get Statistics */
		case CPMAC_DDA_PRIV_GET_STATS:

			{

				CpmacHwStatistics stats;

				/* Caller provides memory for CpmacHwStatistics structure via "data" pointer */
				if (hDDA->ddcIf->ddcNetIf.ddcFuncTable.
				    ddcControl(hDDA->hDDC,
					       CPMAC_DDC_IOCTL_GET_STATISTICS,
					       (Ptr) & stats, NULL)
				    != CPMAC_SUCCESS) {

					errPrint
					    ("Failed to get statistics (CPMAC_DDA_PRIV_GET_STATS) from DDC for  %s.\n",
					     p_dev->name);

					return (CPMAC_DDA_INTERNAL_FAILURE);

				}

				/* Copy the local data to user space */
				if (copy_to_user
				    ((char *)privIoctl.data, (char *)&stats,
				     sizeof(CpmacHwStatistics)))

					return -EFAULT;

				break;

			}

			/* Clear Statistics */
		case CPMAC_DDA_PRIV_CLR_STATS:

			{

				if (hDDA->ddcIf->ddcNetIf.ddcFuncTable.
				    ddcControl(hDDA->hDDC,
					       CPMAC_DDC_IOCTL_CLR_STATISTICS,
					       NULL, NULL)
				    != CPMAC_SUCCESS) {

					errPrint
					    ("Failed to clear statistics (CPMAC_DDA_PRIV_CLR_STATS) in DDC for  %s.\n",
					     p_dev->name);

					return (CPMAC_DDA_INTERNAL_FAILURE);

				}

				break;

			}

		default:

			return -EFAULT;

			break;

		}		/* End of switch for CpmacDrvPrivIoctl command */

	}

	else if (cmd == SIOTIMIB2) {

		TI_SNMP_CMD_T ti_snmp_cmd;

		/* Now copy the user data */
		if (copy_from_user
		    ((char *)&ti_snmp_cmd, (char *)rq->ifr_data,
		     sizeof(TI_SNMP_CMD_T)))

			return -EFAULT;

		switch (ti_snmp_cmd.cmd) {

		case TI_SIOCGINTFCOUNTERS:

			{

				struct mib2_ifCounters mib_counter;

				/* Caller provides memory for CpmacHwStatistics structure via "data" pointer */
				if (hDDA->ddcIf->ddcNetIf.ddcFuncTable.
				    ddcControl(hDDA->hDDC,
					       CPMAC_DDC_IOCTL_IF_COUNTERS,
					       (Ptr) & mib_counter, NULL)
				    != CPMAC_SUCCESS) {

					errPrint
					    ("Failed to get statistics (TI_SIOCGINTFCOUNTERS) from DDC for  %s.\n",
					     p_dev->name);

					return (CPMAC_DDA_INTERNAL_FAILURE);

				}

				/* Copy the local data to user space */
				if (copy_to_user
				    ((char *)ti_snmp_cmd.data,
				     (char *)&mib_counter,
				     sizeof(struct mib2_ifCounters)))

					return -EFAULT;

				break;

			}

		case TI_SIOCGINTFPARAMS:

			{

				struct mib2_ifParams localParams;

				localParams.ifSpeed = hDDA->linkSpeed;

				localParams.ifHighSpeed =
				    (localParams.ifSpeed) / 1000000;

				localParams.ifOperStatus =
				    ((p_dev->
				      flags & IFF_UP) ? MIB2_STATUS_UP :
				     MIB2_STATUS_DOWN);

				localParams.ifPromiscuousMode =
				    ((p_dev->
				      flags & IFF_PROMISC) ? TRUE : FALSE);

				/* Now copy the counters to the user data */
				if (copy_to_user
				    ((char *)ti_snmp_cmd.data,
				     (char *)&localParams,
				     sizeof(struct mib2_ifParams)))

					return -EFAULT;

			}

			break;

		case TI_SIOCGETHERCOUNTERS:

			{

				struct mib2_phyCounters phy_counter;

				/* Caller provides memory for CpmacHwStatistics structure via "data" pointer */
				if (hDDA->ddcIf->ddcNetIf.ddcFuncTable.
				    ddcControl(hDDA->hDDC,
					       CPMAC_DDC_IOCTL_ETHER_COUNTERS,
					       (Ptr) & phy_counter, NULL)
				    != CPMAC_SUCCESS) {

					errPrint
					    ("Failed to get statistics (TI_SIOCGETHERCOUNTERS) from DDC for  %s.\n",
					     p_dev->name);

					return (CPMAC_DDA_INTERNAL_FAILURE);

				}

				/* Copy the local data to user space */
				if (copy_to_user
				    ((char *)ti_snmp_cmd.data,
				     (char *)&phy_counter,
				     sizeof(struct mib2_phyCounters)))

					return -EFAULT;

				break;

			}

		case TI_SIOCGETHERPARAMS:

			{

				struct mib2_ethParams localParams;

				localParams.ethDuplexStatus =
				    ((hDDA->linkMode ==
				      2) ? MIB2_FULL_DUPLEX : MIB2_HALF_DUPLEX);

				/* Now copy the counters to the user data */
				if (copy_to_user
				    ((char *)ti_snmp_cmd.data,
				     (char *)&localParams,
				     sizeof(struct mib2_ethParams)))

					return -EFAULT;

				break;

			}

		default:

			return -EFAULT;

		}

	}

	else {

		return -EFAULT;

	}

	return (0);

}

/******************************************************************************
 *  Linux Module Init/Exit
 *****************************************************************************/

static struct platform_device *cpmac_dev;

static ssize_t cpmac_show_version(struct device_driver *drv, char *buf)
{

	return cpmac_p_get_version(buf, NULL, 0, 4096, NULL, NULL);

}

static DRIVER_ATTR(version, S_IRUGO, cpmac_show_version, NULL);

/* Probe number of CPMAC instances and register net_device structure */
static int __init cpmac_dev_probe(void)
{

	int retVal = 0;

	int unit;

	int instance_count = CPMAC_MAX_INSTANCES;

	/* Obtain clock rate from kernel clock API's */
	cpmac_clk = clk_get(0, "EMACCLK");
	if (IS_ERR(cpmac_clk)) {
		printk("TI DAVINCI EMAC: Failed to get clock. Exiting\n");
		return (-1);
	}
	clk_use(cpmac_clk);
	clk_enable(cpmac_clk);
	cpmac_bus_frequency = clk_get_rate(cpmac_clk);

	if (cpmac_cfg_probe()) {

		printk
		    ("TI DAVINCI EMAC: Failed to probe for configuration, Exiting.\n");
		/* If error, free EMAC clock */
		clk_disable(cpmac_clk);
		clk_unuse(cpmac_clk);
		return (-1);

	}

	cpmac_dev =
	    platform_device_register_simple("ti_davinci_emac", -1, NULL, 0);

	if (IS_ERR(cpmac_dev)) {

		/* If error, free EMAC clock */
		clk_disable(cpmac_clk);
		clk_unuse(cpmac_clk);
		return -1;

	}

	if (driver_register(&cpmac_driver)) {

		platform_device_unregister(cpmac_dev);

		/* If error, free EMAC clock */
		clk_disable(cpmac_clk);
		clk_unuse(cpmac_clk);
		return -1;

	}

	driver_create_file(&cpmac_driver, &driver_attr_version);

	for (unit = 0; unit < instance_count; unit++) {

		struct net_device *p_dev;

		CpmacNetDevice *hDDA;

		int failed;

		if (!(p_dev = alloc_etherdev(sizeof(CpmacNetDevice)))) {

			printk
			    ("TI DaVinci EMAC: Etherdev alloc failed for device inst %d.\n",
			     unit);

			retVal = -ENOMEM;
			/* If error, free EMAC clock */
			clk_disable(cpmac_clk);
			clk_unuse(cpmac_clk);

			break;

		}

		hDDA = NETDEV_PRIV(p_dev);

		hDDA->owner = p_dev;

		hDDA->instanceNum = unit;

		p_dev->init = cpmac_dev_init;	/* Set the initialization function */

		SET_NETDEV_DEV(p_dev, &(cpmac_dev->dev));

		cpmac_net_dev[hDDA->instanceNum] = p_dev;

#if defined CONFIG_CPMAC_INIT_BUF_MALLOC
		g_init_enable_flag = 1;

#endif				/* 
				 */

		cpmac_p_detect_manual_cfg(cfg_link_speed, cfg_link_mode,
					  debug_mode);

		/* Register the network device with Linux */
		failed = register_netdev(p_dev);

		if (failed) {

			printk
			    ("TI DAVINCI EMAC: Could not register device for inst %d because of reason code %d.\n",
			     unit, failed);

			retVal = -1;
			/* If error, free EMAC clock */
			clk_disable(cpmac_clk);
			clk_unuse(cpmac_clk);

			FREE_NETDEV(p_dev);

			break;

		}

		else {

			hDDA->nextDevice = last_cpmac_device;

			last_cpmac_device = p_dev;

			dbgPrint("TI DAVINCI EMAC: %s irq=%2d io=%04x\n",
				 p_dev->name, (int)p_dev->irq,
				 (int)p_dev->base_addr);

			create_proc_read_entry("net/emac_rfc2665_stats", 0, NULL, cpmac_p_read_rfc2665_stats, p_dev);

		}
	}

	if (retVal == 0) {

		/* To maintain backward compatibility with NSP. */
		gp_stats_file = create_proc_entry("net/emac_stats", 0644, NULL);

		if (gp_stats_file) {

			gp_stats_file->read_proc = cpmac_p_read_stats;

			gp_stats_file->write_proc = cpmac_p_write_stats;

		}

		create_proc_read_entry("net/emac_link", 0, NULL,
				       cpmac_p_read_link, NULL);

		create_proc_read_entry("net/emac_ver", 0, NULL,
				       cpmac_p_get_version, NULL);

		create_proc_read_entry("net/emac_config", 0, NULL,
				       cpmac_dump_config, NULL);

	}

	cpmac_devices_installed = unit;

	printk("%s - %s\n", cpmac_DDA_version_string,
	       DDC_cpmacGetVersionInfo(NULL));

	printk("TI DaVinci EMAC: Installed %d instances.\n", unit);

#if defined CONFIG_CPMAC_INIT_BUF_MALLOC
	printk
	    ("TI DAVINCI EMAC driver is allocating buffer memory at init time.\n");

#endif				/* 
				 */

	return ((unit >= 0) ? 0 : -ENODEV);

}

/* Frees the CPMAC device structures */
void cpmac_exit(void)
{

	struct net_device *p_dev;

	CpmacNetDevice *hDDA;

	PAL_Result retCode;

	while (cpmac_devices_installed) {

		char proc_name[100];

		int proc_category_name_len = 0;

		p_dev = last_cpmac_device;

		hDDA = NETDEV_PRIV(p_dev);

		dbgPrint("TI DAVINCI EMAC: Unloading %s irq=%2d io=%04x\n",
			 p_dev->name, (int)p_dev->irq, (int)p_dev->base_addr);

		/* free EMAC clock */
		clk_disable(cpmac_clk);
		clk_unuse(cpmac_clk);

		if (g_init_enable_flag) {

			cpmac_p_dev_disable(hDDA);

		}

		/* Deinit DDC */
		retCode =
		    hDDA->ddcIf->ddcNetIf.ddcFuncTable.ddcDeinit(hDDA->hDDC,
								 NULL);

		if (retCode != CPMAC_SUCCESS) {

			errPrint
			    ("TI DAVINCI EMAC: cpmac_exit:%s: Error %08X from DDC Deinit()\n",
			     p_dev->name, retCode);

			/* We dont want to quit from here, lets delete the instance also */
		}

		/* Delete DDC Instance */
		retCode =
		    hDDA->ddcIf->ddcNetIf.ddcFuncTable.ddcDelInst(hDDA->hDDC,
								  NULL);

		if (retCode != CPMAC_SUCCESS) {

			errPrint
			    ("TI DAVINCI EMAC: cpmac_exit:%s: Error %08X from DDC Delete Instance()\n",
			     p_dev->name, retCode);

			/* We dont want to quit from here, lets continue the exit procedure for the driver */
		}

		/* Delete the proc entry */
		strcpy(proc_name, "davinci/");

		strcat(proc_name, p_dev->name);

		proc_category_name_len = strlen(proc_name);

		strcpy(proc_name + proc_category_name_len, "_rfc2665_stats");

		remove_proc_entry(proc_name, NULL);

		/* Release memory region and unregister the device */
		release_mem_region(p_dev->base_addr,
				   CPMAC_DDA_DEFAULT_CPMAC_SIZE);

		unregister_netdev(p_dev);

		last_cpmac_device = hDDA->nextDevice;

		if (p_dev)

			FREE_NETDEV(p_dev);

		cpmac_devices_installed--;

	}

	if (gp_stats_file)

		remove_proc_entry("net/emac_stats", NULL);

	remove_proc_entry("net/emac_link", NULL);

	remove_proc_entry("net/emac_ver", NULL);

	remove_proc_entry("net/emac_config", NULL);

	platform_device_unregister(cpmac_dev);

	printk("TI DAVINCI EMAC: platform device unregistered.\n");

	driver_remove_file(&cpmac_driver, &driver_attr_version);

	printk("TI DAVINCI EMAC: driver file removed.\n");

	driver_unregister(&cpmac_driver);

	printk("TI DAVINCI EMAC: driver unregistered.\n");

}

module_init(cpmac_dev_probe);

module_exit(cpmac_exit);
