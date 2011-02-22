/*
 * linux/drivers/net/ti_davinci_emac/cpmac_palOsProtect.h
 *
 * EMAC driver Protection abstraction
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
 ver. 0.1 PSP Architecture Team
      0.2 Ajay Singh, Anant Gole - ported for linux 2.6 and DaVinci
 */

#ifndef __CPMAC_PAL_OSPROTECT_H__
#define __CPMAC_PAL_OSPROTECT_H__

#include "_tistdtypes.h"
#include <linux/spinlock.h>
#include <linux/slab.h>

extern spinlock_t *cpmac_lock;	  /**< Tx lock to be used in protection functions */

#define PAL_OSPROTECT_INTERRUPT (-1)
#define PAL_OSPROTECT_SCHEDULER (-2)

/**
 * \brief   PAL OS Protect Entry
 */
static inline void PAL_osProtectEntry(Int level, Uint32 * cookie)
{

	unsigned long flags;

	if (level == PAL_OSPROTECT_INTERRUPT) {

#ifndef CONFIG_PREEMPT_RT
		local_irq_save(flags);

#else
		spin_lock(cpmac_lock);

#endif
		*cookie = flags;

	}

}

/**
 * \brief   PAL OS Protect Exit
 */
static inline void PAL_osProtectExit(Int level, Uint32 cookie)
{

	if (level == PAL_OSPROTECT_INTERRUPT) {

#ifndef CONFIG_PREEMPT_RT
		local_irq_restore(cookie);

#else
		spin_unlock(cpmac_lock);

#endif
	}

}

#endif				/* __CPMAC_PAL_OSPROTECT_H__ */
