/* Copyright (C) 2005 by Texas Instruments */

/* This file defines the CPPI DMA implementation in conformance with
 * DMA Controller Interface(DCI) defined by Mentor Graphics
 */

#include <linux/usb.h>
#include "cppi_dma.h"
#define TRUE 1
#define FALSE 0

/*Compile time option for debug*/
#if MUSB_DEBUG > 0
#define dbgPrint(args...) printk(args)
#else
#define dbgPrint(args...)
#endif

#include "musb_host.h"

/* currently maintaining a global pointer to the first DmaController
 * instance instantiated, completion ISR can refer to bookkeeping info
 * thru this structure. If PrOS supports ISR with parama then this can go away
 */
static CppiDmaController *cppiCB;

extern void MGC_LinuxStartNextUrb(MGC_LinuxCd *, u8);

static MGC_DmaController *funcNewDmaController(
		MGC_pfDmaChannelStatusChanged pfDmaChannelStatusChanged,
		void *pDmaPrivate, u8 * pCoreBase);

static void funcDestroyDmaController(MGC_DmaController * pController);

static u8 funcCppiDmaControllerIsr(void *pPrivateData);

static u8 funcCppiAbortDma(MGC_DmaChannel * pChannel);

//static u8 funcCppiResetDmaController(void *pPrivateData);

/* Define all Global,Instance Variables*/
MGC_DmaControllerFactory MGC_CppiDmaControllerFactory = {
	.wCoreRegistersExtent = 0x300,
	.pfNewDmaController = funcNewDmaController,
	.pfDestroyDmaController = funcDestroyDmaController,
};

/* prototypes */
void funcCppiCompletionIsr(void);

/*
 *  Start Dma controller
 *
 *  Initialize the Dma Controller as necessary.
 */

static u8 funcDmaStartController(void *pPrivateData)
{
	CppiDmaController *pController = (CppiDmaController *) pPrivateData;
	u32 regBase, bufCnt;
	int i, j;
	TxCppiStateRam *txState;
	RxCppiStateRam *rxState;

	/* do whatever is necessary to start controller */
	for (i = 0; i < OTG_MAX_TX_CHANNELS; i++) {
		pController->txCppi[i].bTransmit = TRUE;
		pController->txCppi[i].chNo = i;
	}
	for (i = 0; i < OTG_MAX_RX_CHANNELS; i++) {
		pController->rxCppi[i].bTransmit = FALSE;
		pController->rxCppi[i].chNo = i;
	}

	/* setup BD list on a per channel basis */

	for (i = 0; i < OTG_MAX_TX_CHANNELS; i++) {
		OtgCppiChannel *txChannel = &(pController->txCppi[i]);
		OtgBufferDesc *txBd;

		/* initialize the txChannel fields */
		txChannel->activeQueueHead = (OtgBufferDesc *) 0;
		txChannel->activeQueueTail = (OtgBufferDesc *) 0;
		txChannel->lastHwBDProcessed = (OtgBufferDesc *) 0;
		txChannel->queueActive = FALSE;
		txChannel->pController = pController;
		txChannel->bLastModeRndis = 0;
		txChannel->Channel.pPrivateData = txChannel;
		txChannel->bdPoolHead = NULL;

		/* build the BD Free list for the channel */
		for (j = 0; j < (OTG_TXCHAN_BD_NUM - 1); j++) {
			dma_addr_t dma;

			txBd = dma_pool_alloc(pController->pool, SLAB_KERNEL, &dma);
			txBd->next = txChannel->bdPoolHead;
			txBd->dma = dma;

			txChannel->bdPoolHead = txBd;
		}
	}

	for (i = 0; i < OTG_MAX_RX_CHANNELS; i++) {
		OtgCppiChannel *rxChannel = &(pController->rxCppi[i]);
		OtgBufferDesc *rxBd;

		/* initialize the txChannel fields */
		rxChannel->activeQueueHead = (OtgBufferDesc *) 0;
		rxChannel->activeQueueTail = (OtgBufferDesc *) 0;
		rxChannel->lastHwBDProcessed = (OtgBufferDesc *) 0;
		rxChannel->queueActive = FALSE;
		rxChannel->bLastModeRndis = 0;
		rxChannel->pController = pController;
		rxChannel->Channel.pPrivateData = rxChannel;
		rxChannel->bdPoolHead = NULL;

		/*dbgPrint("pController %x in start controller RX= %x %x\n", (u32) pController, (u32) rxChannel,(u32) rxChannel->Channel.pPrivateData); */

		/* build the BD Free list for the channel */
		for (j = 0; j < (OTG_RXCHAN_BD_NUM - 1); j++) {
			dma_addr_t dma;

			rxBd = dma_pool_alloc(pController->pool, SLAB_KERNEL, &dma);
			rxBd->next = rxChannel->bdPoolHead;
			rxBd->dma = dma;

			rxChannel->bdPoolHead = rxBd;
		}
	}

	/* Do Necessary configuartion in H/w to get started */
	regBase = ((u32) pController->pCoreBase) - MENTOR_BASE_OFFSET;

	/* initialise tx/rx channel head pointers to zero */
	for (i = 0; i < OTG_MAX_TX_CHANNELS; i++) {
		OtgCppiChannel *txChannel = &(pController->txCppi[i]);
		txState = (TxCppiStateRam *) (regBase + TXCPPI_STATERAM_OFFSET(i));
		txChannel->stateRam = (void *)txState;
		/*txState->headPtr = 0; */
		/* zero out entire state RAM entry for the channel */
		txState->headPtr = 0;
		txState->sopDescPtr = 0;
		txState->currDescPtr = 0;
		txState->currBuffPtr = 0;
		txState->flags = 0;
		txState->remLength = 0;
		/*txState->dummy = 0; */
		txState->completionPtr = 0;

	}
	for (i = 0; i < OTG_MAX_RX_CHANNELS; i++) {
		OtgCppiChannel *rxChannel = &(pController->rxCppi[i]);
		rxState = (RxCppiStateRam *) (regBase + RXCPPI_STATERAM_OFFSET(i));
		rxChannel->stateRam = (void *)rxState;
		/*rxState->headPtr = 0; */
		/* zero out entire state RAM entry for the channel */
		rxState->buffOffset = 0;
		rxState->headPtr = 0;
		rxState->sopDescPtr = 0;
		rxState->currDescPtr = 0;
		rxState->currBuffPtr = 0;
		rxState->pktLength = 0;
		rxState->byteCount = 0;
		rxState->completionPtr = 0;
	}

	/* enable individual cppi channels */
	MGC_Write32(regBase, USB_OTG_TXCPPI_INTENAB_REG, DMA_ALL_CHANNELS_ENABLE);
	MGC_Write32(regBase, USB_OTG_RXCPPI_INTENAB_REG, DMA_ALL_CHANNELS_ENABLE);

	/* enable tx/rx CPPI control */
	MGC_Write32(regBase, USB_OTG_TXCPPI_CTRL_REG, DMA_CTRL_ENABLE);
	MGC_Write32(regBase, USB_OTG_RXCPPI_CTRL_REG, DMA_CTRL_ENABLE);

	/* Fake - dummy buffers to get around Rx BufCount Min 3 Buffers restriction */
	for (i = 0; i < OTG_MAX_RX_CHANNELS; i++) {
		bufCnt = MGC_Read32(regBase, (USB_OTG_RXCPPI_BUFCNT0_REG + i * 4));
		bufCnt &= 0xFFFF;
		if (0 == bufCnt) {
			/* add two dummy buffers by writing 2 to channel buffer count */
			MGC_Write32(regBase, (USB_OTG_RXCPPI_BUFCNT0_REG + i * 4), 2);
		} else {
			dbgPrint("Rx channel %d Buffcount left at : %d\n", i, bufCnt);
		}
	}
	pController->dmaStarted = TRUE;
	return TRUE;
}				/*end of function funcDmaStartController() */

/*
 *  Stop Dma controller
 *
 *  De-Init the Dma Controller as necessary.
 */

static u8 funcDmaStopController(void *pPrivateData)
{
	CppiDmaController *pController = (CppiDmaController *) pPrivateData;
	u32 regBase;
	int i;
	MGC_DmaChannel *pChannel;

	pController->dmaStarted = FALSE;
	regBase = ((u32) pController->pCoreBase) - MENTOR_BASE_OFFSET;
	/* DISABLE INDIVIDUAL CHANNEL Interrupts */
	MGC_Write32(regBase, USB_OTG_TXCPPI_INTCLR_REG, DMA_ALL_CHANNELS_ENABLE);
	MGC_Write32(regBase, USB_OTG_RXCPPI_INTCLR_REG, DMA_ALL_CHANNELS_ENABLE);

	/*TODO: should we teardown channels */
	dbgPrint("Tearing down Tx Channels\n");
	for (i = 0; i < OTG_MAX_TX_CHANNELS; i++) {
		pChannel = (MGC_DmaChannel *) & pController->txCppi[i];
		funcCppiAbortDma(pChannel);
	}

	/* in Tx Case proper teardown is supported. We resort to disabling
	 * Tx/Rx CPPI after cleanup of Tx channels. Before TX teardown is complete
	 * TX CPPI cannot be disabled. With Rx we only do s/w clean up,reset of state ram,
	 * better to disable Rx CPPI before that */
	/*disable tx/rx cppi */
	MGC_Write32(regBase, USB_OTG_TXCPPI_CTRL_REG, DMA_CTRL_DISABLE);
	MGC_Write32(regBase, USB_OTG_RXCPPI_CTRL_REG, DMA_CTRL_DISABLE);

#if MUSB_DEBUG > 0
	printk("cleaning up Rx Channels \n");
#endif

	for (i = 0; i < OTG_MAX_RX_CHANNELS; i++) {
		pChannel = (MGC_DmaChannel *) & pController->rxCppi[i];
		/* deactivate the queue */
		pController->rxCppi[i].queueActive = FALSE;
		funcCppiAbortDma(pChannel);
	}

	return TRUE;
}				/*end of function funcDmaStopController() */

/**
 *  Allocate DMA Channel
 *
 *  Allocate a CPPI Channel for DMA
 */

static MGC_DmaChannel *funcDmaAllocateChannel(void *pPrivateData, u8 bLocalEnd,
					      u8 bTransmit, u8 bProtocol, u16 wMaxPacketSize)
{
	CppiDmaController *pController = (CppiDmaController *) pPrivateData;
	u8 chNum;
	MGC_DmaChannel *dmaCh;
	OtgCppiChannel *otgCh;
	MGC_LinuxCd *pThis = pController->pDmaPrivate;
	MGC_LinuxLocalEnd *pLocalEnd = &(pThis->aLocalEnd[bLocalEnd]);
#if 0
	MGC_Controller *pControllerImpl = (MGC_Controller *) MGC_pProsSystem->pController->pPrivateData;
	MGC_Port *pPort = pControllerImpl->pPort;
	MGC_EndpointResource *pEnd;
	pEnd = (MGC_EndpointResource *) MGC_ArrayFetch(&(pPort->LocalEnds), bLocalEnd);
#endif
	/* If channel had not been allocated before, do allocate and Init */
	/*start with sanity checking of arguments */
	if (bTransmit) {
		if (bLocalEnd > OTG_MAX_TX_CHANNELS) {
			dbgPrint
			    ("Local Endpoint %d does not have a corresponding DMA Channel in Tx direction\n",
			     bLocalEnd);
			return ((MGC_DmaChannel *) 0);
		}
	} else {
		if (bLocalEnd > OTG_MAX_RX_CHANNELS) {
			dbgPrint
			    ("Local Endpoint %d does not have a corresponding DMA Channel in Rx direction\n",
			     bLocalEnd);
			return ((MGC_DmaChannel *) 0);
		}
	}

	/* remember bLocalEnd: 1..Max_EndPt, and cppi ChNum:0..Max_EndPt-1 */
	chNum = bLocalEnd - 1;

	/* as of now, just return the corresponding CPPI Channel Handle */
	if (bTransmit) {
		otgCh = (OtgCppiChannel *) & (pController->txCppi[chNum]);
	} else {
		otgCh = (OtgCppiChannel *) & (pController->rxCppi[chNum]);
	}

	otgCh->pEndPt = pLocalEnd;
	dbgPrint
	    ("\nAllocate channel for chnum:%d setting  EPtr:%x ep->bEnd:%d inputEP:%d\n",
	     chNum, (u32) pLocalEnd, pLocalEnd->bEnd, bLocalEnd);
	dmaCh = (MGC_DmaChannel *) (&otgCh->Channel);
	dmaCh->pPrivateData = otgCh;
	return (dmaCh);

}				/*End of function funcDmaAllocateChannel() */

/**
 *  Release DMA Channel
 *
 *  Release a CPPI Channel. This functions is stubbed out because CPPI channels
 *  are bound to Endpoint resource in H/w
 */

static void funcDmaReleaseChannel(MGC_DmaChannel * pChannel)
{
}				/*end of function funcDmaReleaseChannel() */

int dmaProgFail = 0;

/* Buffer enqueuing Logic:
 * we create a static Pool(a free list [linked through software next ptr]
 * out of this) of BDs on a per channel basis. Everytime a New IRP is started
 * and ProgramChannel invoked, the same list is re-used. By not adding
 * BDs to the queue when the queue is active, we ensure that when queue is
 * completely processed all BDs in the pool are in a ready to reclaim state.
 * Hence when we re-construct the queue to satisfy the same IRP we can start
 * from scratch with the same Free list of BDs
 */

/**
 *  Program DMA Channel
 *
 *  Program an allocated CPPI channel for Data transfer.
 */

static u8 funcDmaProgramChannel(MGC_DmaChannel * pChannel,
		u16 wPacketSz, u8 bRndisMode,
		dma_addr_t dma_addr, u32 dwLength)
{
	OtgCppiChannel *otgChannel = (OtgCppiChannel *) pChannel->pPrivateData;
	int numBds = 0, i;
	OtgBufferDesc *pBD;
	TxCppiStateRam *txState;
	RxCppiStateRam *rxState;
	CppiDmaController *pController;
	u32 regOffset, regBase, regVal, temp;
	u16 bufferCount;

	BUG_ON(!otgChannel);
	pController = otgChannel->pController;

	if (!otgChannel->bTransmit)
		printk("ProgramChannel called with pktSz:%d mode:%d dma:0x%x len:%u tx:%d\n",
		       wPacketSz, bRndisMode, dma_addr, dwLength, otgChannel->bTransmit);

	if (0 == dwLength) {
		dbgPrint("Zero byte DMA transfer request \n");

	}
	if (otgChannel->queueActive) {
		/* we shouldnt be here. Program channel must be called only
		 * when Channel is not active
		 */
		dbgPrint("DmaProgram channel when channel is Busy? \n");
		dmaProgFail = 1;
		return FALSE;
	}

	regBase = ((u32) pController->pCoreBase) - MENTOR_BASE_OFFSET;
	printk("In CppiProg regbase=0x%x \n",regBase);
	if (bRndisMode) {
		/* override packetsize with total Length */
		/*TODO:check whether 16 /32 bit limit mismatch */
		if (dwLength & 0xFFFF0000) {
			dbgPrint("DmaProgramChannel overflow, length > 16 bits\n");
		} else {
			wPacketSz = dwLength;
		}

	}

	/* end of if bRndisMode conditional block */
	if (otgChannel->bLastModeRndis != bRndisMode) {
		if (bRndisMode) {
			/* enable rndis mode for the cppi channel */
			regVal = MGC_Read32(regBase, USB_OTG_RNDIS_REG);
			temp = (otgChannel->bTransmit)
				? (1 << (otgChannel->chNo))
				: (1 << (otgChannel->chNo + 16));
			regVal |= temp;
			MGC_Write32(regBase, USB_OTG_RNDIS_REG, regVal);
		} else {
			/* ensure that rndis mode bit for the channel is cleared */
			regVal = MGC_Read32(regBase, USB_OTG_RNDIS_REG);
			temp = (otgChannel->bTransmit)
				? (1 << otgChannel->chNo)
				: (1 << (otgChannel->chNo + 16));
			regVal &= ~temp;
			MGC_Write32(regBase, USB_OTG_RNDIS_REG, regVal);
		}
		/* update our internal flag to new state */
		otgChannel->bLastModeRndis = bRndisMode;
	}

	/* end of if lastModeRndis != bRndisMode */
	/* we are operating the CPPI DMA in transparent mode, hence every BD must correspond to
	 * USB Packet size and every BD must be SOP/EOP
	 * When we operate in Rndis mode also, we create one BD for entire application request, hence
	 * always every BD is SOP/EOP
	 */
	numBds = ((dwLength - 1) / wPacketSz) + 1;
	otgChannel->pktSize = wPacketSz;

/**
 * Host mode Autoreq programming logic:
 * When in rndis Mode just set the AutoReq on all but EOP option.
 * When in Non_rndis mode, its a little tricky as we dont want to set
 * ReqPkt in RxCsr after the transfer gets over. So we break the request
 * of N packets(last pkt may be less than USB Packet size) into N-1 packets
 * for which we turn on AutoReq always option and a an additional last
 * packet for which we turn off AutoReq option and program the DMA to
 * recieve just the last packet.Though the procedure is tedious, at least
 * we ensure that the ReqPkt bit in RxCSr is not set at the end of the transfer.
 *
 * Pitfall: We assume that Non_rndis mode is to be used when the amount of data
 * to be transfered is known beforehand. If the transfer is terminated
 * prematurely with a short packet, the strategy of breaking every transfer
 * to [(N-1),1] will fail
 */

	/* REVISIT:  for TX, "rndis" mode should almost always be used; we know
	 * the lengths in advance, including whether or not a ZLP terminates.
	 * For RX ... check it, but RNDIS mode _should_ again be the default
	 * (given the DMA terminates on short read).
	 */

/* program autoreq if Host mode and Rx */
	if ((!otgChannel->bTransmit)
	    && ((MGC_LinuxCd *) (otgChannel->pController->pDmaPrivate))->bIsHost) {
		/* we need to program the autoReq register in Wrapper register space according
		 * to the DMA mode */
		if (bRndisMode) {
			/* enable AutoReq on all but EOP for the channel */
			regVal = MGC_Read32(regBase, USB_OTG_AUTOREQ_REG);
			regVal |= ((0x1) << (otgChannel->chNo * 2));
			MGC_Write32(regBase, USB_OTG_AUTOREQ_REG, regVal);
		} else {

			if (numBds > 1) {	/* turn on AutoReq always only if more than one packet expected */
				/* enable AutoReq always for the channel */
				regVal = MGC_Read32(regBase, USB_OTG_AUTOREQ_REG);
				regVal |= ((0x3) << (otgChannel->chNo * 2));
				MGC_Write32(regBase, USB_OTG_AUTOREQ_REG, regVal);
			}
		}
	}


	/* REVISIT dma descriptors can now be dynamically allocated */

	if (otgChannel->bTransmit) {	/*program Tx CPPI corresponding Channel */
		numBds = (numBds > OTG_TXCHAN_BD_NUM) ? OTG_TXCHAN_BD_NUM : numBds;

		/* assuming here that DmaProgramChannel is called during
		 * IRP initiation
		 * Current code maintains state for one outstanding IRP only */

		pBD = otgChannel->bdPoolHead;
		otgChannel->activeQueueHead = otgChannel->bdPoolHead;
		otgChannel->lastHwBDProcessed = (OtgBufferDesc *) NULL;
		otgChannel->startAddr = dma_addr;
		/* bytesLeft to be updated as reclaim buffers incompletion handling */
		otgChannel->transferSize = dwLength;
		otgChannel->currOffset = 0;


		printk("Before BD loop\n");

		/* Include code here to enqueue Buffer,BD pair with H/w .All BDs
		 * except for the last should be of full packet size*/
		for (i = 0; i < numBds; i++) {
			pBD->hNext = (u32)pBD->next->dma;

			pBD->buffPtr = otgChannel->startAddr + otgChannel->currOffset;

			if ((otgChannel->currOffset + wPacketSz) <= otgChannel->transferSize) {
				otgChannel->currOffset += wPacketSz;
				pBD->bOffBLen = wPacketSz;
				pBD->hOptions = CPPI_SOP_SET | CPPI_EOP_SET | CPPI_OWN_SET | wPacketSz;
			} else {	/*last buffer may not be a complete USB Packet */
				u32 buffSz;

				buffSz = otgChannel->transferSize - otgChannel->currOffset;
				otgChannel->currOffset = otgChannel->transferSize;
				pBD->bOffBLen = buffSz;
				pBD->hOptions = CPPI_SOP_SET | CPPI_EOP_SET | CPPI_OWN_SET | buffSz;
			}

			dbgPrint("Next:%08x buff:%08x Off:%08x Opt:%08x\n",
				 pBD->hNext, pBD->buffPtr, pBD->bOffBLen, pBD->hOptions);

			/* update the last BD enqueued to the list */
			otgChannel->activeQueueTail = pBD;
			pBD = pBD->next;
		}

		/* ensure that for the last BD to be enqueued the next pointer is NULL */
		otgChannel->activeQueueTail->hNext = (u32) (NULL);

		/* BDs live in DMA-coherent memory, but writes might be pending */
		wmb();

		/* update Channel state to queue active */
		otgChannel->queueActive = TRUE;

		/* Write to the HeadPtr in StateRam to trigger */
		txState = (TxCppiStateRam *) (otgChannel->stateRam);
		dbgPrint("Txstate pointer:0x%x , head set to 0x%x\n",
				(u32) txState,
				(u32) otgChannel->bdPoolHead->dma);
		txState->headPtr = (u32)otgChannel->bdPoolHead->dma;

	} else {		/* program Rx CPPI Corresponding channel */
		numBds = (numBds > OTG_RXCHAN_BD_NUM) ? OTG_RXCHAN_BD_NUM : numBds;

		if ((numBds > 1) && (!bRndisMode)
		    && ((MGC_LinuxCd *) (otgChannel->pController->pDmaPrivate))->bIsHost) {
			/* case of non-rndis mode , host receive - address the AutoReq/ReqPkt issue */
			numBds -= 1;
			otgChannel->bLastPktLeft = 1;
		} else {
			otgChannel->bLastPktLeft = 0;
		}
		/* assuming here that DmaProgramChannel is called during IRP initiation
		 * Current code maintains state for one outstanding IRP only */

		pBD = otgChannel->bdPoolHead;
		otgChannel->activeQueueHead = otgChannel->bdPoolHead;
		otgChannel->lastHwBDProcessed = (OtgBufferDesc *) NULL;
		otgChannel->startAddr = dma_addr;
		/* bytesLeft to be updated as we reclaim buffers in completion handling */
		otgChannel->transferSize = dwLength;
		otgChannel->currOffset = 0;
		otgChannel->actualLen = 0;

		/* Include code here to enqueue Buffer,BD pair with H/w */
		for (i = 0; i < numBds; i++) {
			pBD->hNext = (u32)pBD->next->dma;

			pBD->buffPtr = otgChannel->startAddr + otgChannel->currOffset;

			if ((otgChannel->currOffset + wPacketSz) <= otgChannel->transferSize) {
				otgChannel->currOffset += wPacketSz;
				/* buffer length can be overwritten by the port on reception */
				pBD->bOffBLen = wPacketSz & 0xFFFF;
				/* keep track of enqueued length */
				pBD->enqBuffLen = wPacketSz & 0xFFFF;
				/* Packet size to be updated by the Port only, just set ownership bit */
				pBD->hOptions = CPPI_OWN_SET;
			} else {	/*last buffer may not be a complete USB Packet */
				u32 buffSz;

				buffSz = otgChannel->transferSize - otgChannel->currOffset;
				otgChannel->currOffset = otgChannel->transferSize;
				/* buffer length can be overwritten by the port on reception */
				pBD->bOffBLen = buffSz & 0xFFFF;
				/* keep track of enqueued length */
				pBD->enqBuffLen = buffSz & 0xFFFF;
				/* Packet size to be updated by the Port only, just set Ownership bit */
				pBD->hOptions = CPPI_OWN_SET;
			}

			/* update the last BD enqueued to the list */
			otgChannel->activeQueueTail = pBD;
			pBD = pBD->next;
		}		/* end of for loop over BDs */

		/* ensure that for the last BD to be enqueued the next pointer is NULL */
		otgChannel->activeQueueTail->hNext = (u32) NULL;

		/* BDs live in DMA-coherent memory, but writes might be pending */
		wmb();

		/* update Buff Count by writing to the register */

		regOffset = USB_OTG_RXCPPI_BUFCNT0_REG + (otgChannel->chNo * 4);
		bufferCount = (MGC_Read32(regBase, regOffset)) & 0xFFFF;
		/*dbgPrint("BuffCntReg = %d enqCount=%d  channelNum=%d\n",bufferCount,numBds,otgChannel->chNo ); */
		/*if(bufferCount < 3)
		   {
		   dbgPrint("Unexpected: BuffCount on channel %d is = %d\n",otgChannel->chNo,bufferCount);
		   }
		 */

		/* Update channel status to active */
		otgChannel->queueActive = TRUE;

		/* Prepare to write the HeadPtr in StateRam to trigger */
		rxState = (RxCppiStateRam *) (otgChannel->stateRam);
		/*dbgPrint("Writing Rx HeadPtr  %x\n",((u32) otgChannel->bdPoolHead)); */
		rxState->headPtr = (u32)otgChannel->bdPoolHead->dma;
		/*dbgPrint(" SopDesc=%x Head=%x Desc=%x BuffPtr=%x\n",rxState->sopDescPtr,
		   rxState->headPtr,rxState->currDescPtr,rxState->currBuffPtr); */

		if (numBds > (bufferCount - 2)) {
			/*dbgPrint("writing to BuffCnt %d channel:%d\n",(numBds -(bufferCount -2)) ,otgChannel->chNo); */
			MGC_Write32(regBase, regOffset, (numBds - (bufferCount - 2)));
			/*dbgPrint("readback buffcnt =%d, chn:%d\n",(MGC_Read32(regBase, regOffset)),otgChannel->chNo ) */
			;
		}

		/* TBD: For every Rx BD enqueued: Fill in Next descriptor pointer,Buffer
		 * ptr with byte aligned addr,clear Offset field, Buffer length with number
		 * of bytes in buffer,clear SOP,EOP,EOQ and set Ownership bit */

		/*TBD: With Rx remember to write to buffCount for every
		 * Buffer/BD thats enqueued */

	}

	return TRUE;

}				/*End of function funcDmaProgramChannel() */

/**
 *  Return Channel Status
 *
 *  Get the current status of CPPI DMA channel.Currently stubbed out
 */

static MGC_DmaChannelStatus funcDmaGetChannelStatus(MGC_DmaChannel * pChannel)
{
	OtgCppiChannel *cppiCh = (OtgCppiChannel *) pChannel->pPrivateData;
	if (cppiCh->queueActive) {
		return MGC_DMA_STATUS_BUSY;
	} else {
		return MGC_DMA_STATUS_FREE;
	}
}				/*end of funcDmaGetChannelStatus() */

extern union sch gschflg;
u32 lastWrittenCompletion, nullWrittenCompletion, irpWrittenCompletion;
u32 lastCompletionPtr, lastBdPtr;

/**
 *  CPPI Completion ISR
 *
 *  CPPI completion ISR
 */
#define MUSB_STATUS_OK 0
void funcCppiCompletionIsr(void)
{
	u32 regBase, regVal, intStatus, regOffset;
	int i, chanNum, numBds, numCompleted;
	OtgCppiChannel *txChannel, *rxChannel;
	OtgBufferDesc *bdPtr, *pBD;
	TxCppiStateRam *txState;
	RxCppiStateRam *rxState;
	/*
	   unsigned int  item;
	   u32 status =MUSB_STATUS_OK;
	   u8 bResult;
	   u16 wval=0;
	   unsigned long flags;
	 */
	u8 bEnqueuedBckgd = 0, bReqComplete;
	CppiDmaController *cppidma;
	u16 bdRecvLen;
	u16 bufferCount;
	MGC_LinuxCd *pThis = NULL;
	MGC_LinuxLocalEnd *pEnd = NULL;
	int nPipe;
	struct urb *pUrb = NULL;

	/*TBD: S/W must inspect Rx SOP packet for Rx Abort bit and handle
	 *  accordingly- ignore all buffers as pkt is incomplete
	 */
	regBase = ((u32) cppiCB->pCoreBase) - MENTOR_BASE_OFFSET;
	intStatus = MGC_Read32(regBase, USB_OTG_TXCPPI_MASKED_REG);

	chanNum = 0;
	while (intStatus) {	/*process all Tx channels completion */
		if (intStatus & 1) {
			txChannel = &cppiCB->txCppi[chanNum];
			txState = (TxCppiStateRam *) txChannel->stateRam;
			/*dbgPrint("Tx Channel %d Completion:%x \n",txChannel->chNo,txState->completionPtr); */
			lastCompletionPtr = txState->completionPtr;

			if (NULL == txChannel->lastHwBDProcessed) {
				bdPtr = txChannel->activeQueueHead;
			} else
				bdPtr = txChannel->lastHwBDProcessed->next;

			if (NULL == bdPtr) {

				dbgPrint("Bdptr NUll.Possible error condition \n");

				intStatus = intStatus >> 1;
				chanNum++;
				continue;	/*continue with next Tx Channel */
			}

			i = 0;
			bReqComplete = 0;

			dbgPrint("Tx cppi comp Isr start Bd:%x\n", (u32) bdPtr);
			numCompleted = 0;

			while (1) {	/* run through the list of completed BDs */

				dbgPrint
				    ("Bd hNext:%x BuffPtr:%x  offlen:%x Options:0x%x\n",
				     bdPtr->hNext, bdPtr->buffPtr, bdPtr->bOffBLen, bdPtr->hOptions);

				if (bdPtr->hOptions & CPPI_OWN_SET)
					break;

				numCompleted++;

				if ((bdPtr->hOptions & CPPI_EOQ_MASK)
				    || (bdPtr->hNext == (u32) NULL)) {
					txChannel->lastHwBDProcessed = bdPtr;
					bReqComplete = 1;
					break;
				}
				txChannel->lastHwBDProcessed = bdPtr;
				bdPtr = (OtgBufferDesc *) bdPtr->next;
				/*bdPtr = (OtgBufferDesc*) bdPtr->hNext; */

				if (++i > OTG_TXCHAN_BD_NUM) {
					dbgPrint
					    ("possible runaway condition, processing beyound maximum BD number possible \n");
				}
			}	/* end of while 1 loop */

			/* write to the completion register to acknowledge processing of completed BDs */
			txState->completionPtr = txChannel->lastHwBDProcessed->dma;
			dbgPrint("Update completion Ptr:%x\n",
				 (u32) txChannel->lastHwBDProcessed->dma);

#if MUSB_DEBUG >1
			printk("Num of Tx Bds serviced:%d\n", numCompleted);
#endif

			lastWrittenCompletion = (u32) txChannel->lastHwBDProcessed->dma;
			lastBdPtr = (u32) txChannel->lastHwBDProcessed;

			/* for now we can check any of EOQ,Next being NULL,lastHwProcessed being activeQueueTail
			 *  to detect end of list processing */
			if (bReqComplete) {
				nullWrittenCompletion = (u32) txChannel->lastHwBDProcessed;

				/* if more data for current IRP re-program the channel */
				if (txChannel->currOffset >= (txChannel->transferSize)) {

					irpWrittenCompletion = (u32) txChannel->lastHwBDProcessed;
					/* transfer for current IRP complete */

					/* set queue to inactive until next IRP fired, reclaim all BDs */
					txChannel->activeQueueHead = (OtgBufferDesc *) 0;
					txChannel->activeQueueTail = (OtgBufferDesc *) 0;
					txChannel->lastHwBDProcessed = (OtgBufferDesc *) 0;
					txChannel->queueActive = FALSE;

					cppidma = txChannel->pController;
					pThis = (MGC_LinuxCd *) cppidma->pDmaPrivate;
					pEnd = txChannel->pEndPt;
					pUrb = MGC_GetCurrentUrb(pEnd);
					/* set status */
					pUrb->status = 0;
					pUrb->actual_length = txChannel->currOffset;
					dbgPrint
					    ("Irp completed. pend:%x pUrb:%x Act Len=%d\n",
					     (u32) pEnd, (u32) pUrb, pUrb->actual_length);
					nPipe = pUrb ? pUrb->pipe : 0;

#if 0
				/** dont bother about re-use now */
					if (usb_pipebulk(nPipe)) {
						/* we re-use bulk, so re-programming required */
						pEnd->bIsReady = FALSE;
						/* release claim if borrowed */
						if ((pEnd->bLocalEnd != pThis->bBulkTxEnd)
						    && (pThis->bBulkTxEnd != pThis->bBulkRxEnd)) {
							pEnd->bIsClaimed = FALSE;
						}

						spin_lock_irqsave(&pThis->Lock, flags);
						MGC_SelectEnd(pThis->pRegs, pEnd->bLocalEnd);
						wval = MGC_ReadCsr16(pThis->pRegs, MGC_O_HDRC_TXCSR, pEnd->bLocalEnd);
						spin_unlock_irqrestore(&pThis->Lock, flags);

						dbgPrint("Save toggle TxCsr:0x%x \n", wval);

						/* save data toggle */
						usb_settoggle(pUrb->dev,
							      pEnd->bEnd, 1, (wval & MGC_M_TXCSR_H_DATATOGGLE)
							      ? 1 : 0);
					}
#endif

					DBG(3,
					    "completing Tx URB=%p, status=%d, len=%x\n",
					    pUrb, pUrb->status, pUrb->actual_length);
					dbgPrint
					    ("calling Next URb from Tx Cppi complete args: %x %x \n",
					     (u32) pThis, pEnd->bEnd);
					/*MGC_LinuxStartNextUrb((MGC_LinuxCd*)pThis, pEnd->bEnd); */
					spin_lock_irq(&pEnd->Lock);
					MGC_LinuxStartNextUrb((MGC_LinuxCd *)
							      pThis, pEnd->bLocalEnd);
					spin_unlock_irq(&pEnd->Lock);
					dbgPrint("Back from startNextUrb\n");
				} else {
					/* more data to be moved for the current IRP , re-program */
					numBds =
					    ((txChannel->transferSize -
					      txChannel->currOffset - 1) / txChannel->pktSize) + 1;
					numBds = (numBds > OTG_TXCHAN_BD_NUM) ? OTG_TXCHAN_BD_NUM : numBds;

					pBD = txChannel->bdPoolHead;
					txChannel->activeQueueHead = txChannel->bdPoolHead;
					txChannel->lastHwBDProcessed = (OtgBufferDesc *) NULL;

					/* Include code here to enqueue Buffer,BD pair with H/w */
					for (i = 0; i < numBds; i++) {
						pBD->hNext = (u32)pBD->next->dma;
						pBD->buffPtr = txChannel->startAddr + txChannel->currOffset;

						if ((txChannel->currOffset +
						     txChannel->pktSize) <= txChannel->transferSize) {
							txChannel->currOffset += txChannel->pktSize;
							pBD->bOffBLen = txChannel->pktSize;
							pBD->hOptions =
							    CPPI_SOP_SET |
							    CPPI_EOP_SET | CPPI_OWN_SET | txChannel->pktSize;
						} else {	/* last buffer may not be a complet USB packet */
							u32 buffSz;
							buffSz = txChannel->transferSize - txChannel->currOffset;
							txChannel->currOffset = txChannel->transferSize;
							pBD->bOffBLen = buffSz;
							pBD->hOptions =
							    CPPI_SOP_SET | CPPI_EOP_SET | CPPI_OWN_SET | buffSz;
						}

						/* update the last BD enqueued to the list */
						txChannel->activeQueueTail = pBD;
						pBD = pBD->next;
					}	/* end of for loop over numBds */

					/* ensure that for the last BD to be enqueued the next pointer is NULL */
					txChannel->activeQueueTail->hNext = (u32) NULL;

					/* BDs live in DMA-coherent memory, but writes might be pending */
					wmb();

					/* Write to the HeadPtr in StateRam to trigger */
					txState = (TxCppiStateRam *) (txChannel->stateRam);
					txState->headPtr = (u32)txChannel->bdPoolHead->dma;

				}	/* end of Else block - for more outstanding data to be moved with current IRP */

			}
			/* end of if NULL== bdPtr->hNext */
		}

		intStatus = intStatus >> 1;
		chanNum++;
	}

	/** Start processing the RX block **/
	intStatus = MGC_Read32(regBase, USB_OTG_RXCPPI_MASKED_REG);
	chanNum = 0;
	while (intStatus) {
		/* process all Rx Channels completion */
		if (intStatus & 1) {
			rxChannel = &cppiCB->rxCppi[chanNum];
			rxState = (RxCppiStateRam *) rxChannel->stateRam;

			/*dbgPrint("Rx Channel %d completion %x \n", rxChannel->chNo,rxState->completionPtr); */
			lastCompletionPtr = rxState->completionPtr;

			if (NULL == rxChannel->lastHwBDProcessed)
				bdPtr = rxChannel->activeQueueHead;
			else
				bdPtr = rxChannel->lastHwBDProcessed->next;

			/* handle case of stray interrupt ? */
			if (NULL == bdPtr) {
				dbgPrint("bdPtr is NULL in rx path of cppi completion\n");
				intStatus = intStatus >> 1;
				chanNum++;
				continue;	/* continue with next Rx channel */
			}

			i = 0;
			bReqComplete = 0;

			while (1) {	/* run through the list of completed BDs */

				if (bdPtr->hOptions & CPPI_RXABT_SET) {
					dbgPrint("Rx Abt set \n");
				}

				/* lets try a polling scenario here, instead of break issues continue at this point */
				if (bdPtr->hOptions & CPPI_OWN_SET)
					break;	/* continue; */

				/* actual packet received length, in our case each BD is SOP/EOP
				 * hence no explicit checking being done */
				bdRecvLen = (u16) (bdPtr->hOptions & CPPI_RECV_PKTLEN_MASK);
				rxChannel->actualLen += bdRecvLen;

#if 0
				/* include an appropriate condition to terminate short transfers */
				if (bdRecvLen < (bdPtr->bOffBLen & CPPI_BUFFER_LEN_MASK)) {
					/* TODO:short packet , terminate transfer */
				}
#endif

				if ((bdPtr->hOptions & CPPI_EOQ_MASK)
				    || (bdPtr->hNext == (u32) NULL)) {
					rxChannel->lastHwBDProcessed = bdPtr;
					bReqComplete = 1;
					break;
				}

				rxChannel->lastHwBDProcessed = bdPtr;
				bdPtr = (OtgBufferDesc *) bdPtr->next;
				/*bdPtr = (OtgBufferDesc*) bdPtr->hNext; */

				if (++i > OTG_RXCHAN_BD_NUM) {
					dbgPrint("Possible runaway condition.Processing BD past max RxBd count \n");
				}

			}	/* end of while 1 loop over list of completed BDs */

			/* write to the completion register to acknowledge processing of completed BDs */
			rxState->completionPtr = (u32)rxChannel->lastHwBDProcessed->dma;
			dbgPrint("Update completion Ptr:%x\n",
				(u32) (rxChannel->lastHwBDProcessed->dma));
			lastBdPtr = (u32) rxChannel->lastHwBDProcessed;

			/* for now we can check any of EOQ,Next being NULL,lastHwProcessed being activeQueueTail
			 *  to detect end of list processing */
			if (bReqComplete) {
				/* if more data for current IRP re-program the channel */
				if (rxChannel->currOffset >= (rxChannel->transferSize)) {

					/* transfer for current IRP complete */
					/*dbgPrint("Before clearing Head RxCSR in CppiComplete:0x%x\n",MGC_ReadCsr16(0x37400, 0x06, 2)); */
					/* set queue to inactive until next IRP fired, reclaim all BDs */
					rxChannel->activeQueueHead = (OtgBufferDesc *) 0;
					rxChannel->activeQueueTail = (OtgBufferDesc *) 0;
					rxChannel->lastHwBDProcessed = (OtgBufferDesc *) 0;
					rxChannel->queueActive = FALSE;
					cppidma = rxChannel->pController;
					pThis = (MGC_LinuxCd *) cppidma->pDmaPrivate;
					pEnd = rxChannel->pEndPt;
					pUrb = MGC_GetCurrentUrb(pEnd);
					/* set status */
					pUrb->status = 0;
					pUrb->actual_length = rxChannel->actualLen;
					nPipe = pUrb ? pUrb->pipe : 0;

#if 0
					/** for now dont bother about endpoint reuse */
					if (usb_pipebulk(nPipe)) {
						/* we re-use bulk, so re-programming required */
						pEnd->bIsReady = FALSE;
						/* release claim if borrowed */
						if ((pEnd->bLocalEnd != pThis->bBulkRxEnd)
						    && (pThis->bBulkTxEnd != pThis->bBulkRxEnd)) {
							pEnd->bIsClaimed = FALSE;
						}

						/* save data toggle */
						spin_lock_irqsave(&pThis->Lock, flags);
						MGC_SelectEnd(pThis->pRegs, pEnd->bLocalEnd);
						wval = MGC_ReadCsr16(pThis->pRegs, MGC_O_HDRC_RXCSR, pEnd->bLocalEnd);
						spin_unlock_irqrestore(&pThis->Lock, flags);

						usb_settoggle(pUrb->dev,
							      pEnd->bEnd, 0, (wval & MGC_M_RXCSR_H_DATATOGGLE)
							      ? 1 : 0);
					}
#endif

					DBG(3,
					    "completing Tx URB=%p, status=%d, len=%x\n",
					    pUrb, pUrb->status, pUrb->actual_length);
					dbgPrint("calling NextUrb from Rx Cppi complete \n");
					/*MGC_LinuxStartNextUrb((MGC_LinuxCd*)pThis, pEnd->bEnd); */
					spin_lock_irq(&pEnd->Lock);
					MGC_LinuxStartNextUrb((MGC_LinuxCd *)
							      pThis, pEnd->bLocalEnd);
					spin_unlock_irq(&pEnd->Lock);

				} /* end of if currOffset > conditional block */
				else {
					/* more data to be moved for the current IRP , re-program */
					numBds =
					    ((rxChannel->transferSize -
					      rxChannel->currOffset - 1) / rxChannel->pktSize) + 1;
					numBds = (numBds > OTG_RXCHAN_BD_NUM) ? OTG_RXCHAN_BD_NUM : numBds;

					if (rxChannel->bLastPktLeft) {
						if (numBds > 1) {
							numBds -= 1;
						} else {
							/* this should be the last packet,turn off autoReq and program one BD */
							regVal = MGC_Read32(regBase, USB_OTG_AUTOREQ_REG);
							regVal &= (~((0x3) << (rxChannel->chNo * 2)));
							MGC_Write32(regBase, USB_OTG_AUTOREQ_REG, regVal);
							/* clear off bLastPktLeft flag */
							rxChannel->bLastPktLeft = 0;
						}	/* end of else conditional block */

					}
					/* end of if lastPacketLeft conditional block */
					pBD = rxChannel->bdPoolHead;
					rxChannel->activeQueueHead = rxChannel->bdPoolHead;
					rxChannel->lastHwBDProcessed = (OtgBufferDesc *) NULL;

					/* Include code here to enqueueBuffer,BD pair with H/w */
					for (i = 0; i < numBds; i++) {
						pBD->hNext = (u32)pBD->next->dma;
						pBD->buffPtr = rxChannel->startAddr + rxChannel->currOffset;

						if ((rxChannel->currOffset +
						     rxChannel->pktSize) <= rxChannel->transferSize) {
							rxChannel->currOffset += rxChannel->pktSize;
							/* buffer length can be overwritten by Port on reception */
							pBD->bOffBLen = rxChannel->pktSize & 0xFFFF;
							/* keep of enqueued length */
							pBD->enqBuffLen = rxChannel->pktSize & 0xFFFF;
							/* packet size to be updated by port only, just set OWN bit */
							pBD->hOptions = CPPI_OWN_SET;
						} else {	/* last buffer may not be a complete USB packet */
							u32 buffSz;

							buffSz = rxChannel->transferSize - rxChannel->currOffset;
							rxChannel->currOffset = rxChannel->transferSize;

							/* buffer length can be overwritten by the port on reception */
							pBD->bOffBLen = buffSz & 0xFFFF;
							/* keep track of enqueued length */
							pBD->enqBuffLen = buffSz & 0xFFFF;
							/* Packet size to be updated by the Port only, just set Ownership bit */
							pBD->hOptions = CPPI_OWN_SET;
						}	/* end of else block covering last buffer descriptor case */

						/* update the last BD enqueued to the list */
						rxChannel->activeQueueTail = pBD;
						pBD = pBD->next;
					}	/* end of for loop over numBds */

					/* ensure that for the last BD to be enqueued the next pointer is NULL */
					rxChannel->activeQueueTail->hNext = (u32) NULL;

					/* update Buff Count by writing to the register */
					regOffset = USB_OTG_RXCPPI_BUFCNT0_REG + (rxChannel->chNo * 4);
					bufferCount = (MGC_Read32(regBase, regOffset)) & 0xFFFF;

					/* Write to the HeadPtr in StateRam to trigger */
					rxState = (RxCppiStateRam *) (rxChannel->stateRam);
					rxState->headPtr = (u32)rxChannel->bdPoolHead->dma;

					if (numBds > (bufferCount - 2)) {
						/*dbgPrint("writing to BuffCnt %d channel:%d\n",(numBds -(bufferCount -2)) ,otgChannel->chNo); */
						MGC_Write32(regBase, regOffset, (numBds - (bufferCount - 2)));
						/*dbgPrint("readback buffcnt =%d, chn:%d\n",(MGC_Read32(regBase, regOffset)),otgChannel->chNo ) */
						;
					}

				}	/* end of Else block - for more outstanding data to be moved with current IRP */

			}
			/* end of if bReqComplete conditional block */
		}

		/* end of if intStatus & 1 conditional block */
		intStatus = intStatus >> 1;
		chanNum++;
	}			/* end of while intStatus loop */

	/* no processing in Background thread,process all enqueued reqs  and then  re-enable interrupts right here */
	if (bEnqueuedBckgd) {

	}
	/* write to CPPI EOI register to re-enable interrupts */
	MGC_Write32(regBase, USB_OTG_CPPI_EOI_REG, 0);

}				/* end of function funcCppiCompletionIsr() */

/**
 * Instantiate a DMA Controller
 *
 * Instantiate a software object representing a DMA controller.
 */

static MGC_DmaController *funcNewDmaController(MGC_pfDmaChannelStatusChanged
					       pfDmaChannelStatusChanged, void *pDmaPrivate, u8 * pCoreBase)
{
	int i;
	MGC_DmaController *pResult = NULL;
	CppiDmaController *pController;

	pController = kzalloc(sizeof *pController, GFP_KERNEL);
	if (!pController)
		return NULL;

	/* Initialize the Cppi DmaController  structure */
	cppiCB = pController;

	pController->dmaStarted = FALSE;

	/*check if following init for CB is reqd */
	for (i = 0; i < OTG_MAX_TX_CHANNELS; i++) {
		pController->txIsInit[i] = FALSE;
		pController->txTeardownPending[i] = FALSE;
	}
	for (i = 0; i < OTG_MAX_RX_CHANNELS; i++) {
		/*pController->rxIsCreated[i] = FALSE; */
		pController->rxTeardownPending[i] = FALSE;
	}

	pController->pCoreBase = pCoreBase;
	pController->pDmaPrivate = pDmaPrivate;
	pController->Controller.pPrivateData = pController;
	pController->Controller.pfDmaStartController = funcDmaStartController;
	pController->Controller.pfDmaStopController = funcDmaStopController;
	pController->Controller.pfDmaAllocateChannel = funcDmaAllocateChannel;
	pController->Controller.pfDmaReleaseChannel = funcDmaReleaseChannel;
	pController->Controller.pfDmaProgramChannel = funcDmaProgramChannel;
	pController->Controller.pfDmaGetChannelStatus = funcDmaGetChannelStatus;
	pController->Controller.pfDmaControllerIsr = funcCppiDmaControllerIsr;
	/*pController->Controller.pfDmaResetController =  funcCppiResetDmaController; */
	pResult = &(pController->Controller);

	/* setup BufferPool */
	pController->pool = dma_pool_create("cppi",
					    pController->pDmaPrivate->
					    controller, sizeof(struct cppi_descriptor), 16, 0);
	if (!pController->pool) {
		kfree(pController);
		pResult = NULL;
	}

	return pResult;
}				/* end of funcNewDmaController() */

/**
 *  Destroy DMA Controller.
 *
 *  Destroy a previously-instantiated DMA controller.
 */

static void funcDestroyDmaController(MGC_DmaController * pController)
{
	CppiDmaController *cpController = (CppiDmaController *) pController->pPrivateData;

	/* FIXME first free all the descriptors allocated from this pool
	 */
	dma_pool_destroy(cpController->pool);

	if (cpController) {
		cpController->Controller.pPrivateData = NULL;
		kfree(cpController);
	}

}				/* end of funcDestroyDmaController() */

static u8 funcCppiAbortDma(MGC_DmaChannel * pChannel)
{
	u32 regBase;
	u32 regVal, enable;
	int chNum;
	OtgCppiChannel *otgCh = (OtgCppiChannel *) pChannel->pPrivateData;
	CppiDmaController *pController = otgCh->pController;
	TxCppiStateRam *txState;
	RxCppiStateRam *rxState;

	regBase = ((u32) pController->pCoreBase) - MENTOR_BASE_OFFSET;
	chNum = otgCh->chNo;
	if (!otgCh->queueActive)
		return TRUE;

	if (otgCh->bTransmit) {
		/* If channel is Tx set the TearDown Bit, wait and reclaim enqueued BDs */
		do {
			regVal = MGC_Read32(regBase, USB_OTG_TXCPPI_TEAR_REG);
		} while (!(regVal & CPPI_TEAR_READY));

		/*  we want to mask completion interrupts that are raised to signal teardown complete
		 *  Since this function may be called from thread context as well, clear the corresponding
		 *  Tx Channel interrupt Mask and re-enable before exit */

		enable = (MGC_Read32(regBase, USB_OTG_TXCPPI_INTENAB_REG)) & (1 << otgCh->chNo);
		MGC_Write32(regBase, USB_OTG_TXCPPI_INTCLR_REG, (1 << otgCh->chNo));

		MGC_Write32(regBase, USB_OTG_TXCPPI_TEAR_REG, chNum & CPPI_CHNUM_BITS_MASK);

		/* wait for teardown completion to be signalled */
		txState = (TxCppiStateRam *) otgCh->stateRam;
		do {
			regVal = txState->completionPtr;
		} while (0xFFFFFFFC != regVal);
		/* TODO: acknowledge write to CompletionPtr ? */
		txState->completionPtr = 0xFFFFFFFC;

		/*re-enable interrupt  if it was enabled earlier */
		if (enable)
			MGC_Write32(regBase, USB_OTG_TXCPPI_INTENAB_REG, (1 << otgCh->chNo));

		txState->headPtr = 0;
		txState->sopDescPtr = 0;
		txState->currBuffPtr = 0;
		txState->currDescPtr = 0;
		txState->flags = 0;
		txState->remLength = 0;

		/* Ensure that we clean up any Interrupt asserted
		 * 1. Write to completion Ptr value 0x1(bit 0 set) - write back mode
		 * 2. Write to completion Ptr value 0x0(bit 0 cleared) - compare mode
		 * Value written is compared(for bits 31:2) nad being equal interrupt deasserted?
		 */
		/* write back mode, bit 0 set, hence completion Ptr must be updated */
		txState->completionPtr = 0x1;
		/* compare mode, write back zero now */
		txState->completionPtr = 0;
	} /* if transmit channel */
	else {
		/* for recv channel, turn of autoreq always until next req is fired */
		if (((MGC_LinuxCd *) (otgCh->pController->pDmaPrivate))->bIsHost) {
			regVal = MGC_Read32(regBase, USB_OTG_AUTOREQ_REG);
			regVal &= (~((0x3) << (otgCh->chNo * 2)));
			MGC_Write32(regBase, USB_OTG_AUTOREQ_REG, regVal);

			/* keep away from clearing ReqPkt for now
			 * rxCsr = MGC_ReadCsr16(((u32) pController->pCoreBaseIsr),MGC_O_HDRC_RXCSR, (otgCh->chNo +1));
			 * rxCsr &= (~(MGC_M_RXCSR_H_REQPKT));
			 * MGC_WriteCsr16(((u32) pController->pCoreBaseIsr),MGC_O_HDRC_RXCSR, (otgCh->chNo+1),rxCsr);
			 * */
		}
		if ((otgCh->queueActive)
		    && (!((MGC_LinuxCd *) (otgCh->pController->pDmaPrivate))->bIsHost)) {
			dbgPrint("cannot abort Rx Channel when active, H/W does not support  teardown \n");
			return FALSE;
		} else {

			rxState = (RxCppiStateRam *) otgCh->stateRam;

			/* zero out stateRam */
			rxState->headPtr = 0;
			rxState = (RxCppiStateRam *) otgCh->stateRam;
			rxState->buffOffset = 0;
			rxState->byteCount = 0;

			/* Ensure that we clean up any Interrupt asserted
			 * 1. Write to completion Ptr value 0x1(bit 0 set) - write back mode
			 * 2. Write to completion Ptr value 0x0(bit 0 cleared) - compare mode
			 * Value written is compared(for bits 31:2) nad being equal interrupt deasserted?
			 */
			/* write back mode, bit 0 set, hence completion Ptr must be updated */
			rxState->completionPtr = 0x1;
			/* compare mode, write back zero now */
			rxState->completionPtr = 0;

			rxState->currBuffPtr = 0;
			rxState->currDescPtr = 0;
			rxState->pktLength = 0;
			rxState->sopDescPtr = 0;
		}		/* end of Rx Channel not active */
	}

	/* TODO: why no teardown for RX ? */
	/* reclaim BDs part is same for rx/tx */
	otgCh->activeQueueHead = (OtgBufferDesc *) 0;
	otgCh->activeQueueTail = (OtgBufferDesc *) 0;
	otgCh->lastHwBDProcessed = (OtgBufferDesc *) 0;
	otgCh->queueActive = FALSE;
	otgCh->startAddr = 0;
	otgCh->currOffset = 0;
	otgCh->transferSize = 0;
	otgCh->pktSize = 0;
	return TRUE;
/* TODO: may be called from Interrupt Context, do we want to wait until teardown complete */
}

static u8 funcCppiDmaControllerIsr(void *pPrivateData)
{
	/* As Per DCI documentation this function needs to be implemented if DMA
	 * interrupts are multiplexed/shared with other Core interrupts.Hence
	 * its just stubbed out.Findout whether functionPtr in MGC_DmaController
	 * must be set to NULL*/
	return FALSE;
}

#if 0
/* new function to re-start DMA*/
static u8 funcCppiResetDmaController(void *pPrivateData)
{
	CppiDmaController *pController = (CppiDmaController *) pPrivateData;
	if (pController->dmaStarted)
		funcDmaStopController(pController);

	funcDmaStartController(pController);
	return TRUE;
}
#endif

/* TBD Queries:
 *
 * 1. DMA ISR - completion callback
 * 2. DMA ISR - registration and IRQ enable
 * 3. Buffer Allocation and management
 * 	--> RESOLVED:  wrong API for Linux.
 * 	    dma channels take pre-mapped dma addresses.
 * 4. Function for Dma Channel Teardown/flush
 * 5. Keep tab on EndPoint resource allocation policy
 * 6. Mentor Stack ISR would need modification as EndPt Interrupts are raised
 *    on error. DMA mode is programmed in accordance with BCG wrapper
 * 7. Interrupt Vector interpretation and EOI
 * 8. Verify whether One URB<-> One Buffer Descriptor holds.Esp, for ISO and Int
 *    endpoints.
 * 9. Modify TXCSR,RXCSR programming in the stack -
 *    TXCSR - Set DMAReqEnab,DMAReqMode and clear Autoset
 *    RXCSR - Set DMAReqEnab.Clear DMAReqMode and AutoClear
 *            Further if Host mode clear AutoReq(once when we enter Host mode)
 *
 * 10. External API to CPPI configuration outside the scope of stack/DCI
 * 11. USB BUS Reset- Teardown of TX DMA channels must.(then setup DMa,core
 *     again).Hence introduce DCI API for channel teardown?
 * 12. For complete TX(no RX) teardown set Cppi teardown bit, then in TXCSR
 *     set FlushFIFO(twice if double buffered)
 * 13. Power Management as is necessary
 *
 *
 */

/* Assumptions
 * 1. Each URB Buffer can be mapped directly with one BD
 * 2.
 *
 */

/* Misc
 * 1.Find information about system RESET/CLOCK etc reqd for initialization
 * 2. These line lengths are way too long; 80 characters.
 * 3. finish removing the virtual_to_bus stuff
 */
