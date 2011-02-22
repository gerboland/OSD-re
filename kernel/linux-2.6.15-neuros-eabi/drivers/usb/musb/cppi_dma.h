/* Copyright (C) 2005 by Texas Instruments */

#ifndef _CPPI_DMA_H_
#define _CPPI_DMA_H_

#include <linux/slab.h>
#include <linux/list.h>
#include <linux/smp_lock.h>
#include <linux/errno.h>
#include <linux/dmapool.h>

#include "cppi_dma_cfg.h"
#include "dma.h"
#include "musbdefs.h"
#include "musb_hdrdf.h"

extern MGC_DmaControllerFactory MGC_CppiDmaControllerFactory;

/* bit masks for CPPI BDs */
#define CPPI_SOP_SET   ((u32)(1 << 31))
#define CPPI_EOP_SET   ((u32)(1 << 30))
#define CPPI_OWN_SET  ((u32)(1 << 29))
#define CPPI_EOQ_MASK ((u32)(1 << 28))
#define CPPI_RXABT_SET ((u32)(1<<19))
#define CPPI_RECV_PKTLEN_MASK 0xFFFF
#define CPPI_BUFFER_LEN_MASK 0xFFFF

#define CPPI_TEAR_READY ((u32)(1 << 31))
#define CPPI_CHNUM_BITS_MASK  0x3

/* CPPI data structure definitions */

/** 
 *  CPPI  Buffer Descriptor 
 *
 *   Buffer Descriptor structure for USB OTG Module CPPI.Using the same across Tx/Rx
 */
struct cppi_descriptor {
	/* Hardware Overlay */
	u32 hNext;     /**< Next(hardware) Buffer Descriptor Pointer */
	u32 buffPtr;	   /**<Buffer Pointer (dma_addr_t) */
	u32 bOffBLen;	    /**<Buffer_offset16,buffer_length16 */
	u32 hOptions;	    /**<Option fields for SOP,EOP etc*/

	struct cppi_descriptor *next; /**<Next(software) Buffer Descriptor pointer*/
	dma_addr_t dma;		/* address of this descriptor */

	/* for Rx Desc, keep track of enqueued Buffer len to detect short packets */
	u32 enqBuffLen;
} __attribute__ ((aligned(16)));

typedef struct cppi_descriptor OtgBufferDesc;

/** TBD: BD alignment restriction */

/* Forward Declaration */
struct __OtgCppiChannel;

/**
 *  Dummy buffer descriptor
 *
 *  Dummy Buffer descriptor to workaround minimum(3) BD restriction
 *  on Rx CPPI Chain.Need to have to such structures for every RX CPPI Channel
 */
struct dummy_bd {
	/* Hardware overlay fields */
	u32 *hNext;	       /**< Next(hardware) Buffer Descriptor Pointer */
	u32 *buffPtr;	    /**<Buffer Pointer */
	u32 *bOffBLen;	     /**<Buffer_offset16,buffer_length16 */
	u32 *hOptions;	     /**<Option fields for SOP,EOP etc*/
} __attribute__ ((aligned(16)));

typedef struct dummy_bd DummyBD;

/**
 *  OTG CPPI channel Config info
 */
typedef struct {
	int channelNum;
} OtgChannelConfig;

/**
 *  CPPI Channel state
 */
typedef enum {
	CPPI_CH_UNINITIALIZED = 0,
	CPPI_CH_INITIALIZED,
	CPPI_CH_OPENED,
	CPPI_CH_CLOSE_IN_PROGRESS,
	CPPI_CH_CLOSED
} OtgCppiChannelState;

/* forward declaration for CppiDmaController structure */
struct _CppiDmaController;

/**
 *  Channel Control Structure
 *
 * CPPI  Channel Control structure. Using he same for Tx/Rx. If need be
 * derive out of this later.
 */
typedef struct __OtgCppiChannel {
	/* First field must be MGC_DmaChannel for easy type casting
	 * dont change the position of this field*/
	MGC_DmaChannel Channel;

	/* back pointer to the Dma Controller structure */
	struct _CppiDmaController *pController;

	/* flag for tx/rx */
	u8 bTransmit;
	/*channel number for reference, access as bTransmit,chNo tupple */
	u8 chNo;

	/* flag to avoid reprogramming */
	u8 bLastModeRndis;

	/*flag to address non_rndis mode last packet ReqPkt handling */
	u8 bLastPktLeft;

	/* private pointer passed during channel program.In this implementation
	 * used as a reference to EndpointResource structure so as to enable
	 * completion callback invocation 
	 */

	MGC_LinuxLocalEnd *pEndPt;
	/* Params from channel config */
	u32 numBd;	      /**<Number of BDs */
	OtgCppiChannelState chState;	/**<Channel state */

	/* book keeping for current Irp request */
	u32 startAddr;
	u32 currOffset;
	u32 transferSize;
	u32 pktSize;
	u32 actualLen;

	void *stateRam;		/*pointer to StateRam for the Channel */

	/* BD management fields */
	OtgBufferDesc *bdPoolHead;		/* Free BD Pool head pointer */
	OtgBufferDesc *activeQueueHead;		/* Head of active packet queue */
	OtgBufferDesc *activeQueueTail;		/* Last hardware buffer descriptor written */
	OtgBufferDesc *lastHwBDProcessed;	/* Last hardware BD Processed */
	u8 queueActive;				/* Queue Active True/False */

} OtgCppiChannel;

/**
 *  CPPI Dma Controller Object
 *
 *  CPPI Dma controller object.Encapsulates all bookeeping and Data
 *  structures pertaining to the CPPI Dma Controller.
 */
typedef struct _CppiDmaController {
	MGC_DmaController Controller;
	MGC_LinuxCd *pDmaPrivate;
	u8 *pCoreBase;
	int dmaStarted;

	/* book-keeping information */
	OtgCppiChannel txCppi[OTG_MAX_TX_CHANNELS];
	OtgCppiChannel rxCppi[OTG_MAX_RX_CHANNELS];

	struct dma_pool *pool;

#if 0
	/* maintain a reference to MGC_Port structure */
	MGC_Port *pPortPtr;
	/* maintain a reference to System services structure */
	MGC_SystemServices *pSystemServices;
#endif

	/* check if the following are reqd */
	u8 txIsInit[OTG_MAX_TX_CHANNELS];
	u8 rxIsInit[OTG_MAX_RX_CHANNELS];
	u8 txTeardownPending[OTG_MAX_TX_CHANNELS];

	/*TBD:Check whether H/w allows Rx teardown */
	u8 rxTeardownPending[OTG_MAX_RX_CHANNELS];

} CppiDmaController;

#endif				/* end of ifndef _CPPI_DMA_H_ */
