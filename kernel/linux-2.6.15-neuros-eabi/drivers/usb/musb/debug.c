/******************************************************************
 * Copyright 2005 Mentor Graphics Corporation
 * Copyright (C) 2005 by Texas Instruments
 *
 * This file is part of the Inventra Controller Driver for Linux.
 * 
 * The Inventra Controller Driver for Linux is free software; you 
 * can redistribute it and/or modify it under the terms of the GNU 
 * General Public License version 2 as published by the Free Software 
 * Foundation.
 * 
 * The Inventra Controller Driver for Linux is distributed in 
 * the hope that it will be useful, but WITHOUT ANY WARRANTY; 
 * without even the implied warranty of MERCHANTABILITY or 
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public 
 * License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with The Inventra Controller Driver for Linux ; if not, 
 * write to the Free Software Foundation, Inc., 59 Temple Place, 
 * Suite 330, Boston, MA  02111-1307  USA
 * 
 * ANY DOWNLOAD, USE, REPRODUCTION, MODIFICATION OR DISTRIBUTION 
 * OF THIS DRIVER INDICATES YOUR COMPLETE AND UNCONDITIONAL ACCEPTANCE 
 * OF THOSE TERMS.THIS DRIVER IS PROVIDED "AS IS" AND MENTOR GRAPHICS 
 * MAKES NO WARRANTIES, EXPRESS OR IMPLIED, RELATED TO THIS DRIVER.  
 * MENTOR GRAPHICS SPECIFICALLY DISCLAIMS ALL IMPLIED WARRANTIES 
 * OF MERCHANTABILITY; FITNESS FOR A PARTICULAR PURPOSE AND 
 * NON-INFRINGEMENT.  MENTOR GRAPHICS DOES NOT PROVIDE SUPPORT 
 * SERVICES OR UPDATES FOR THIS DRIVER, EVEN IF YOU ARE A MENTOR 
 * GRAPHICS SUPPORT CUSTOMER. 
 ******************************************************************/

 /* Inventra Controller Driver (ICD) for Linux.
  *
  * Debug general code.
  */

#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/completion.h>
#include <linux/interrupt.h>

#include <linux/usb.h>

#include "debug.h"
#include "musbdefs.h"

#define IPRINTF(_f, _m)	printk(KERN_INFO "%s"_f, indent, _m)
#define isspace(c) 	(c==' ' || c=='\t')
#define LABEL KERN_INFO "dump: "

/******************************************************************/

int MGC_DebugLevel = MUSB_DEBUG;
int MGC_DebugDisable = 0;

static char *dump_node(struct list_head *node);

int mgc_is_corrupt(struct musb *pThis, const char *function,
				int line)
{
#ifdef CONFIG_USB_MUSB_HDRC_HCD
	u8 bEnd;
	MGC_LinuxLocalEnd *pEnd;
#endif
	u8 bResult = FALSE;

	if (MGC_PAD_FRONT != pThis->dwPadFront) {
		bResult = TRUE;
		printk(KERN_INFO "musb %s:%d: pThis front pad corrupted (%x)\n",
		       function, line, pThis->dwPadFront);
	}

	if (MGC_PAD_BACK != pThis->dwPadBack) {
		bResult = TRUE;
		printk(KERN_INFO "musb %s:%d: pThis back pad corrupted (%x)\n",
		       function, line, pThis->dwPadBack);
	}
#ifdef CONFIG_USB_MUSB_HDRC_HCD
	for (bEnd = 0; bEnd < pThis->bEndCount; bEnd++) {
		pEnd = &(pThis->aLocalEnd[bEnd]);

		if (MGC_PAD_FRONT != pEnd->dwPadFront) {
			bResult = TRUE;
			printk(KERN_INFO
			       "musb %s:%d: end %d front pad corrupted (%x)\n",
			       function, line, bEnd, pEnd->dwPadFront);
		}

		if (MGC_PAD_BACK != pEnd->dwPadBack) {
			bResult = TRUE;
			printk(KERN_INFO
			       "musb %s:%d: end %d back pad corrupted (%x)\n",
			       function, line, bEnd, pEnd->dwPadBack);
		}
	}
#endif

	return bResult;
}

/******************************************************************/

/* Decode CSR0 value to a string.
 */
char *decode_csr0(u16 csr0)
{
	static char buf[64];
	sprintf(buf, "(%s%s%s%s)",
		csr0 & MGC_M_CSR0_TXPKTRDY ? "[TXPKTRDY]" : "",
		csr0 & MGC_M_CSR0_P_SVDRXPKTRDY ? "[SVDRXPKTRDY]" : "",
		csr0 & MGC_M_CSR0_P_SENDSTALL ? "[stalled]" : "",
		csr0 & MGC_M_CSR0_P_DATAEND ? "[dataend]" : "");
	return buf;
}

#if 0
/* Decode a value to binary.
 */
static char *decode_bits(u16 value)
{
	int i = 0;
	static char buf[64];

	for (; i < 16; i++) {
		buf[15 - i] = (value & (1 << i)) ? '1' : '0';
	}

	return buf;
}

/* Decode TXCSR register.
 */
static char *decode_txcsr(u16 txcsr)
{
	static char buf[256];
	sprintf(buf, "%s (%s%s%s%s)",
		decode_bits(txcsr),
		txcsr & MGC_M_TXCSR_TXPKTRDY ? "[TXPKTRDY]" : "",
		txcsr & MGC_M_TXCSR_AUTOSET ? "[MGC_M_TXCSR_AUTOSET]" : "",
		txcsr & MGC_M_TXCSR_DMAENAB ? "[MGC_M_TXCSR_DMAENAB]" : "",
		txcsr & MGC_M_TXCSR_DMAMODE ? "[MGC_M_TXCSR_DMAMODE]" : "");
	return buf;
}
#endif

/*
 */
char *decode_devctl(u16 devctl)
{
	return (devctl & MGC_M_DEVCTL_HM) ? "host" : "function";
}

/*
 */
char *decode_ep0stage(u8 stage)
{
	static char buff[64];
	u8 stallbit = stage & MGC_END0_STAGE_STALL_BIT;

	stage = stage & ~stage & MGC_END0_STAGE_STALL_BIT;
	sprintf(buff, "%s%s", (stallbit) ? "stall-" : "",
		(stage == MGC_END0_STAGE_SETUP)
		? "setup" : (stage == MGC_END0_STAGE_TX)
		? "tx" : (stage == MGC_END0_STAGE_RX)
		? "rx" : (stage == MGC_END0_STAGE_STATUSIN)
		? "statusin" : (stage == MGC_END0_STAGE_STATUSOUT)
		? "statusout" : "error");
	return buff;
}

/*
 */
void dump_urb(void *pUrb)
{
	struct urb *purb = (struct urb *)pUrb;

	printk(LABEL "urb                   :%p\n", purb);
	printk(LABEL "urb_list				 :%s\n",
	       dump_node(&purb->urb_list));
	printk(LABEL "dev                   :%p\n", purb->dev);
	printk(LABEL "pipe                  :%08X\n", purb->pipe);
	printk(LABEL "status                :%d\n", purb->status);
	printk(LABEL "transfer_flags        :%08X\n", purb->transfer_flags);
	printk(LABEL "transfer_buffer       :%p\n", purb->transfer_buffer);
	printk(LABEL "transfer_buffer_length:%d\n",
	       purb->transfer_buffer_length);
	printk(LABEL "actual_length         :%d\n", purb->actual_length);
	printk(LABEL "setup_packet          :%p\n", purb->setup_packet);
	printk(LABEL "start_frame           :%d\n", purb->start_frame);
	printk(LABEL "number_of_packets     :%d\n", purb->number_of_packets);
	printk(LABEL "interval              :%d\n", purb->interval);
	printk(LABEL "error_count           :%d\n", purb->error_count);
	printk(LABEL "context               :%p\n", purb->context);
	printk(LABEL "complete              :%p\n", purb->complete);
}

/**
 * Dump core registers whose reads are non-destructive.
 * @param pThis
 * @param bEnd 
 */
void MGC_HdrcDumpRegs(u8 * pBase, int multipoint, u8 bEnd)
{
	MGC_SelectEnd(pBase, bEnd);

	if (!bEnd) {
		printk(KERN_INFO
		       " 0: CSR0=%04x, Count0=%02x, Type0=%02x, NAKlimit0=%02x\n",
		       MGC_ReadCsr16(pBase, MGC_O_HDRC_CSR0, 0),
		       MGC_ReadCsr8(pBase, MGC_O_HDRC_COUNT0, 0),
		       MGC_ReadCsr8(pBase, MGC_O_HDRC_TYPE0, 0),
		       MGC_ReadCsr8(pBase, MGC_O_HDRC_NAKLIMIT0, 0));
	} else {
		printk(KERN_INFO
		       "%2d: TxCSR=%04x, TxMaxP=%04x, TxType=%02x, TxInterval=%02x\n",
		       bEnd, MGC_ReadCsr16(pBase, MGC_O_HDRC_TXCSR, bEnd),
		       MGC_ReadCsr16(pBase, MGC_O_HDRC_TXMAXP, bEnd),
		       MGC_ReadCsr8(pBase, MGC_O_HDRC_TXTYPE, bEnd),
		       MGC_ReadCsr8(pBase, MGC_O_HDRC_TXINTERVAL, bEnd));
		printk(KERN_INFO
		       "    RxCSR=%04x, RxMaxP=%04x, RxType=%02x, RxInterval=%02x, RxCount=%04x\n",
		       MGC_ReadCsr16(pBase, MGC_O_HDRC_TXCSR, bEnd),
		       MGC_ReadCsr16(pBase, MGC_O_HDRC_TXMAXP, bEnd),
		       MGC_ReadCsr8(pBase, MGC_O_HDRC_TXTYPE, bEnd),
		       MGC_ReadCsr8(pBase, MGC_O_HDRC_TXINTERVAL, bEnd),
		       MGC_ReadCsr16(pBase, MGC_O_HDRC_RXCOUNT, bEnd));
	}

	if (multipoint) {
		printk(KERN_INFO
		       "    TxAddr=%02x, TxHubAddr=%02x, TxHubPort=%02x\n",
		       MGC_Read8(pBase,
				 MGC_BUSCTL_OFFSET(bEnd,
						   MGC_O_HDRC_TXFUNCADDR)),
		       MGC_Read8(pBase,
				 MGC_BUSCTL_OFFSET(bEnd, MGC_O_HDRC_TXHUBADDR)),
		       MGC_Read8(pBase,
				 MGC_BUSCTL_OFFSET(bEnd,
						   MGC_O_HDRC_TXHUBPORT)));
		printk(KERN_INFO
		       "    RxAddr=%02x, RxHubAddr=%02x, RxHubPort=%02x\n",
		       MGC_Read8(pBase,
				 MGC_BUSCTL_OFFSET(bEnd,
						   MGC_O_HDRC_RXFUNCADDR)),
		       MGC_Read8(pBase,
				 MGC_BUSCTL_OFFSET(bEnd, MGC_O_HDRC_RXHUBADDR)),
		       MGC_Read8(pBase,
				 MGC_BUSCTL_OFFSET(bEnd,
						   MGC_O_HDRC_RXHUBPORT)));
	}
}

/* list related */

/*
 * NOT REENTRANT!
 */
static char *dump_node(struct list_head *node)
{
	static char buf[64];
	sprintf(buf, "[n=%p,p=%p]", node->next, node->prev);
	return buf;
}

/*
 */
int queue_length(struct list_head *lh)
{
	int count = 0;
	struct list_head *p = lh;

	while (p && (p->next != lh)) {
		count++;
		p = p->next;
	}

	return count;
}
