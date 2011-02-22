/* this file includes DMA configuration related definitions */

#ifndef _CPPI_DMA_CFG_H_
#define _CPPI_DMA_CFG_H_

/* FIXME import #channels from struct musb */

/* macro definitions for maximum bounds */
#define OTG_MAX_TX_CHANNELS   4
#define OTG_MAX_RX_CHANNELS   4

/* REVISIT now we can avoid preallocating these descriptors;
 * also the freelist could vanish.
 */
#define OTG_TXCHAN_BD_NUM       65
#define OTG_RXCHAN_BD_NUM       65

#endif
