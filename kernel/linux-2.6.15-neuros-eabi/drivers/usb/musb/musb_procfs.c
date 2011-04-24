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

/*
 * Inventra Controller Driver (ICD) for Linux.
 *
 * The code managing debug files (currently in procfs).
 */

#include <linux/config.h>
#include <linux/kernel.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/usb.h>
#include <asm/uaccess.h>	/* FIXME remove procfs writes */

#include "musbdefs.h"

#include "musb_hdrdf.h"

#if MUSB_DEBUG > 0
static int atoi(char *buffer, int base, int len)
{
	int result = 0, digit = 0;

	while (len-- > 0 && (*buffer)) {
		digit = ((*buffer >= '0') && (*buffer <= '9'))
		    ? *buffer - '0' : ((*buffer >= 'a') && (*buffer <= 'f'))
		    ? *buffer - 'a' : -1;

		if (digit < 0) {
			break;
		}

		buffer++;
		result = result * base + digit;
	}

	return result;
}

static const char *decode_address(int index)
{
	static const char *COMMON_REGISTER_MAP[] = {
		"FAddr", "Power", "IntrTx", "IntrRx",
		"IntrTxE", "IntrRxE", "IntrUSB", "IntrUSBE",
		"Frame", "Index", "TestMode"
	};
	return (index < 11) ? COMMON_REGISTER_MAP[index] : NULL;
}
#endif

//#ifdef CONFIG_USB_MUSB_HDRC_HCD
/**
 * Dump statistics for a local end (driver operaiting in host mode).
 * @param pThis the device driver instance  
 * @param bEnd
 * @param aBuffer the buffer to print the report to
 */
static int dump_end_stats(MGC_LinuxCd * pThis, u8 bEnd, char *aBuffer)
{
	int code, count = 0;
	MGC_LinuxLocalEnd *pEnd = &pThis->aLocalEnd[bEnd];

	spin_lock(&pEnd->Lock);

	do {
		code = snprintf(aBuffer, 256 - count,
				"End-%01x: %s, %s, %s, proto=%s, pkt size=%04x, address=%02x, end=%02x\n",
				bEnd,
//				(MGC_GetCurrentUrb(pEnd) ? "Busy" : "Idle"),
"(state tbd)",
				(list_empty(&(pEnd->urb_list)) ? "Q Empty" :
				 " Q Full"), (pEnd->bIsTx ? "Tx" : "Rx"),
// FIXME pull that code into THIS file,
// and make peripheral mode dump itself too
				"?",	//decode_protocol(pThis, bEnd), 
				pEnd->wPacketSize, pEnd->bAddress, pEnd->bEnd);
		if (code < 0) {
			break;
		} else {
			count += code;
		}

		if (MUSB_IS_HST(pThis)) {
			code = snprintf(&aBuffer[count], 256 - count,
					"  %10ld bytes Rx in %10ld pkts; %10ld errs, %10ld overruns\n",
					pEnd->dwTotalRxBytes,
					pEnd->dwTotalRxPackets,
					pEnd->dwErrorRxPackets,
					pEnd->dwMissedRxPackets);
			if (code < 0) {
				break;
			} else {
				count += code;
			}

			code = snprintf(&aBuffer[count], 256 - count,
					"  %10ld bytes Tx in %10ld pkts; %10ld errs, %10ld underruns\n",
					pEnd->dwTotalTxBytes,
					pEnd->dwTotalTxPackets,
					pEnd->dwErrorTxPackets,
					pEnd->dwMissedTxPackets);
			if (code < 0) {
				break;
			} else {
				count += code;
			}
		} else {
			/* no stats for gadget, yet! */
		}
	} while (0);

	spin_unlock(&pEnd->Lock);
	if (code < 0) {
		ERR("An error generating the report");
		return code;
	}

	return count;
}
// #endif

/** Dump the current status and compile options.
 * @param pThis the device driver instance  
 * @param buffer where to dump the status; it must be big enough hold the
 * result otherwise "BAD THINGS HAPPENS(TM)".
 */
static int dump_header_stats(MGC_LinuxCd * pThis, char *buffer)
{
	int code, count = 0;
	const u8 *pBase = pThis->pRegs;

	*buffer = 0;
	count = sprintf(buffer,
			"Current Status: %sDRC, Mode=%s (addr=%d) (Power=%02x, DevCtl=%02x)\n",
			(pThis->bIsMultipoint ? "MH" : "H"), MUSB_MODE(pThis),
#ifdef CONFIG_USB_GADGET_MUSB_HDRC
			MUSB_IS_HST(pThis) ? 0 : pThis->bAddress,
#else
			-1,
#endif
			MGC_Read8(pBase, MGC_O_HDRC_POWER),
			MGC_Read8(pBase, MGC_O_HDRC_DEVCTL));
	if (count < 0)
		return count;

	code = sprintf(&buffer[count],
			"Options: "
#ifdef CONFIG_USB_INVENTRA_FIFO
			"[pio]"
#elif defined(CONFIG_USB_TI_CPPI_DMA)
			"[cppi-DMA]"
#elif defined(CONFIG_USB_INVENTRA_DMA)
			"[musb-DMA]"
#else
			"[?]"
#endif
			" "
#ifdef CONFIG_USB_MUSB_OTG
			"[otg: peripheral+host]"
#elif defined(CONFIG_USB_GADGET_MUSB_HDRC)
			"[peripheral]"
#elif defined(CONFIG_USB_MUSB_HDRC_HCD)
			"[host]"
#endif
			" [debug=%d] [eps=%d]\n",
		MGC_GetDebugLevel(), pThis->bEndCount);
	if (code < 0)
		return code;
	count += code;

#ifdef	CONFIG_ARCH_DAVINCI
	code = sprintf(&buffer[count],
			"DaVinci: ctrl=%02x stat=%1x rndis=%05x auto=%02x "
			"intsrc=%08x intmsk=%08x"
			"\n",
			MGC_Read32(pBase - MENTOR_BASE_OFFSET,
				USB_OTG_CTRL_REG),
			MGC_Read32(pBase - MENTOR_BASE_OFFSET,
				USB_OTG_STAT_REG),
			MGC_Read32(pBase - MENTOR_BASE_OFFSET,
				USB_OTG_RNDIS_REG),
			MGC_Read32(pBase - MENTOR_BASE_OFFSET,
				USB_OTG_AUTOREQ_REG),
			MGC_Read32(pBase - MENTOR_BASE_OFFSET,
				USB_OTG_INT_SOURCE_REG),
			MGC_Read32(pBase - MENTOR_BASE_OFFSET,
				USB_OTG_INT_MASK_REG));
	if (code < 0)
		return count;
	count += code;
#endif

#ifdef	CONFIG_USB_GADGET_MUSB_HDRC
	if (pThis->pGadgetDriver) {
		code = sprintf(&buffer[count], "Gadget driver: %s\n",
				pThis->pGadgetDriver->driver.name);
		if (code < 0)
			return code;
		count += code;
	}
#endif

	return count;
}

/* Write to ProcFS
 *
 * C soft-connect
 * c soft-disconnect
 * I enable HS
 * i disable HS
 * R resume bus
 * S start session (OTG-friendly when OTG-compiled)
 * s stop session
 * F force session (OTG-unfriendly)
 * E rElinquish bus (OTG)
 * H request host mode
 * h cancel host request
 * P disable the low-power mode that kills us in peripheral mode
 * D<num> set/query the debug level
 * Z zap
 */
static int MGC_ProcWrite(struct file *file, const char *buffer,
			 unsigned long count, void *data)
{
	char cmd;
	u8 bReg;
	u8 *pBase = ((MGC_LinuxCd *) data)->pRegs;

	/* MOD_INC_USE_COUNT; */

	copy_from_user(&cmd, buffer, 1);
	switch (cmd) {
	case 'C':
		if (pBase) {
			bReg =
			    MGC_Read8(pBase,
				      MGC_O_HDRC_POWER) | MGC_M_POWER_SOFTCONN;
			MGC_Write8(pBase, MGC_O_HDRC_POWER, bReg);
		}
		break;

	case 'c':
		if (pBase) {
			bReg =
			    MGC_Read8(pBase,
				      MGC_O_HDRC_POWER) & ~MGC_M_POWER_SOFTCONN;
			MGC_Write8(pBase, MGC_O_HDRC_POWER, bReg);
		}
		break;

	case 'I':
		if (pBase) {
			bReg =
			    MGC_Read8(pBase,
				      MGC_O_HDRC_POWER) | MGC_M_POWER_HSENAB;
			MGC_Write8(pBase, MGC_O_HDRC_POWER, bReg);
		}
		break;

	case 'i':
		if (pBase) {
			bReg =
			    MGC_Read8(pBase,
				      MGC_O_HDRC_POWER) & ~MGC_M_POWER_HSENAB;
			MGC_Write8(pBase, MGC_O_HDRC_POWER, bReg);
		}
		break;

	case 'R':
		if (pBase) {
			bReg = MGC_Read8(pBase, MGC_O_HDRC_POWER);
			MGC_Write8(pBase, MGC_O_HDRC_POWER,
				   bReg | MGC_M_POWER_RESUME);
			mdelay(10);
			MGC_Write8(pBase, MGC_O_HDRC_POWER, bReg);
			WARN("Power Resumed\n");
		}
		break;

	case 'S':
		MGC_Session((MGC_LinuxCd *) data);
		break;

#ifdef CONFIG_USB_MUSB_OTG
	case 'E':
		MGC_OtgMachineRequest(&(((MGC_LinuxCd *) data)->OtgMachine),
				      MGC_OTG_REQUEST_DROP_BUS);
		break;
#endif

	case 's':
		bReg = MGC_Read8(pBase, MGC_O_HDRC_DEVCTL);
		bReg &= ~MGC_M_DEVCTL_SESSION;
		MGC_Write8(pBase, MGC_O_HDRC_DEVCTL, bReg);
		break;

	case 'F':
		bReg = MGC_Read8(pBase, MGC_O_HDRC_DEVCTL);
		bReg |= MGC_M_DEVCTL_SESSION;
		MGC_Write8(pBase, MGC_O_HDRC_DEVCTL, bReg);
		break;

	case 'H':
		if (pBase) {
			bReg = MGC_Read8(pBase, MGC_O_HDRC_DEVCTL);
			bReg |= MGC_M_DEVCTL_HR;
			MGC_Write8(pBase, MGC_O_HDRC_DEVCTL, bReg);
			//MUSB_HST_MODE( ((MGC_LinuxCd*)data) );
			//WARN("Host Mode\n");
		}
		break;

	case 'h':
		if (pBase) {
			bReg = MGC_Read8(pBase, MGC_O_HDRC_DEVCTL);
			bReg &= ~MGC_M_DEVCTL_HR;
			MGC_Write8(pBase, MGC_O_HDRC_DEVCTL, bReg);
		}
		break;

#if (MUSB_DEBUG>0)
		/* read & write registers */
	case 'r':
	case 'w':{
			u8 index = 0;
			u32 value = 0;
			char command[64];

			memset(command, 0, sizeof(command));
			copy_from_user(command, buffer,
				       min(count, (unsigned long)63));

			/* detrermine the index, 
			 * only the adrress now */
			index = atoi(&command[2], 16, count - 2);
			if (index > 0 && pBase) {
				const char *address = decode_address(index);

				if (buffer[0] == 'r') {
					value = (command[1] == '8')
					    ? MGC_Read8(pBase, index)
					    : (command[1] == 'f')
					    ? MGC_Read16(pBase, index)
					    : 0;
				} else {
					// not write, not yet...
					index = -1;
				}

				if (address) {
					INFO("%s=0x%x\n", address, value);
				} else {
					INFO("0x%x=0x%x\n", index, value);
				}
			}
		}
		break;

		/* set/read debug level */
	case 'D':{
			if (count > 1) {
				char digits[8], *p = digits;
				int i = 0, level = 0, sign = 1, len =
				    min(count - 1, (unsigned long)8);

				copy_from_user(&digits, &buffer[1], len);

				/* optional sign */
				if (*p == '-') {
					len -= 1;
					sign = -sign;
					p++;
				}

				/* read it */
				while (i++ < len && *p > '0' && *p < '9') {
					level = level * 10 + (*p - '0');
					p++;
				}

				MGC_SetDebugLevel(sign * level);
			} else {
				INFO("MGC_DebugLevel=%d\n", MGC_DebugLevel);
				/* & dump the status to syslog */
			}
		}
		break;

		/* display queue status */
	case 'Q':{
			int index = -1;
			char endb[256];
			MGC_LinuxCd *pThis = (MGC_LinuxCd *) data;

			if (count > 2) {
				char digits[8];
				int len = min(count, (unsigned long)8);
				copy_from_user(&digits, &buffer[1], len);
				index = atoi(digits, 10, len);
			}

			if (dump_header_stats(pThis, endb) > 0) {
				printk(KERN_INFO "%s", endb);
			}
#ifdef CONFIG_USB_MUSB_HDRC_HCD
			if (MUSB_IS_HST(pThis)) {
				if (index < 0) {
					u8 bEnd;

					/* generate the report for the end points */
					for (bEnd = 0; bEnd < pThis->bEndCount;
					     bEnd++) {
						if (dump_end_stats
						    (pThis, bEnd, endb) > 0) {
							printk(KERN_INFO "%s",
							       endb);
						}
					}

				} else {
					if (dump_end_stats(pThis, index, endb) >
					    0) {
						printk(KERN_INFO "%s", endb);
					}
				}
			}
#endif

#ifdef CONFIG_USB_GADGET_MUSB_HDRC
			if (MUSB_IS_DEV(pThis)) {
				// erm normal debug dump should show queues
			}
#endif

		}
		break;

	case '?':
		INFO("?: you are seeing it\n");
		INFO("C/c: soft connect enable/disable\n");
		INFO("I/i: hispeed enable/disable\n");
		INFO("S/s: session set/clear\n");
		INFO("F: \n");
		INFO("H: host mode\n");
		INFO("r/w: read write register\n");
		INFO("D: set/read dbug level\n");
		INFO("Q: show queue status\n");
		break;
#endif

	default:
		ERR("Command %c not implemented\n", cmd);
		break;
	}

	return count;
}

/**
 *
 *
 */
static int MGC_ProcRead(char *page, char **start,
			off_t off, int count, int *eof, void *data)
{
	off_t len = 0;
	char *buffer;
	int rc = 0, code = 0;
	unsigned long flags;
	MGC_LinuxCd *pThis = (MGC_LinuxCd *) data;

	spin_lock_irqsave(&pThis->Lock, flags);

	buffer = kmalloc(4 * 1024, GFP_USER);
	if (!buffer) {
		ERR("Out of memory\n");
		return -1;
	}

	/* generate the report for the end points */
	code = dump_header_stats(pThis, buffer);
	if (code > 0) {
		len += code;
	}
#ifdef CONFIG_USB_MUSB_HDRC_HCD
	if (MUSB_IS_HST(pThis)) {
		u8 bEnd;

		for (bEnd = 0; bEnd < pThis->bEndCount; bEnd++) {
			code = dump_end_stats(pThis, bEnd, &buffer[len]);
			if (code > 0) {
				len += code;
			}
		}
	}
#endif

#ifdef	CONFIG_USB_GADGET_MUSB_HDRC
	if (MUSB_IS_DEV(pThis)) {
		u8 bEnd;

		// FIXME dump each endpoint's request queue,
		// display by endpoint name, etc

		for (bEnd = 0; bEnd < pThis->bEndCount; bEnd++) {
			code = dump_end_stats(pThis, bEnd, &buffer[len]);
			if (code > 0) {
				len += code;
			}
		}
	}
#endif

	if (off < len) {
		int i = 0, togo = len - off;
		char *s = &buffer[off], *d = page;

		if (togo > count) {
			togo = count;
		}

		while (i++ < togo) {
			*d++ = *s++;
		}

		rc = togo;
	} else {
		*buffer = 0;
		*eof = 1;
	}

	kfree(buffer);
	spin_unlock_irqrestore(&pThis->Lock, flags);
	return rc;
}

void MGC_LinuxDeleteProcFs(MGC_LinuxCd * data)
{
	remove_proc_entry(data->pProcEntry->name, NULL);
}

struct proc_dir_entry *MGC_LinuxCreateProcFs(char *name, MGC_LinuxCd * data)
{
	/* FIXME convert everything to seq_file; then later, debugfs */

	if (!name) {
		name = data->aName;
	}

	data->pProcEntry = create_proc_entry(name,
					     S_IFREG | S_IRUGO | S_IWUSR, NULL);
	if (data->pProcEntry) {
		data->pProcEntry->data = data;
		data->pProcEntry->owner = THIS_MODULE;
		// pProcEntry->proc_fops = &MGC_ProcFileOperations;

		data->pProcEntry->read_proc = MGC_ProcRead;
		data->pProcEntry->write_proc = MGC_ProcWrite;

		data->pProcEntry->size = 0;

		pr_debug("Registered /proc/%s\n", name);
	} else {
		pr_debug("Cannot create a valid proc file entry");
	}

	return data->pProcEntry;
}
