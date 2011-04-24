/*
 * linux/drivers/net/ti_davinci_emac/mib_ioctl.h
 *
 * EMAC Driver Core mib ioctl interfaces
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

#ifndef _MIB_IOCTL_H_
#define _MIB_IOCTL_H_

#include "ioctl_api.h"

typedef struct {
	unsigned long cmd;
	unsigned long port;
	void *data;
} TI_SNMP_CMD_T;

/* Ioctl/Cmd value to be used by snmpd like applications */
#define SIOTIMIB2   SIOCDEVPRIVATE + 1

#endif				/* _MIB_IOCTL_H_ */
