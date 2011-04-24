/*
 * File: drivers/video/omap/lcd-itomap2430-casio01.c
 *
 * LCD panel support for the itomap2430 board
 *
 * Copyright (C) 2004 Nokia Corporation
 * Author: Imre Deak <imre.deak@nokia.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 */

#include <linux/module.h>

#include <asm/arch/omapfb.h>

/* #define OMAPFB_DBG 1 */

#include "debug.h"

static int itomap2430_casio01_panel_init(struct omapfb_device *fbdev)
{
    DBGENTER(1);
    DBGLEAVE(1);
    return 0;
}

static void itomap2430_casio01_panel_cleanup(void)
{
    DBGENTER(1);
    DBGLEAVE(1);
}

static int itomap2430_casio01_panel_enable(void)
{

    DBGENTER(1);
    DBGLEAVE(1);
    return 0;
}

static void itomap2430_casio01_panel_disable(void)
{
    DBGENTER(1);
    DBGLEAVE(1);
}

static unsigned long itomap2430_casio01_panel_get_caps(void)
{
    return 0;
}

struct lcd_panel itomap2430_casio01_panel = {
    .name           = "itomap2430",
    .display_type   = OMAP_LCDC_TYPE,
    .config         = OMAP_LCDC_PANEL_TFT,

    .bpp            = 16,
    .data_lines     = 18,
    .x_res          = 320,
    .y_res          = 240,
    .pixel_clock    = 6000,

    .hbp            = 1,
    .hfp            = 54,
    .hsw            = 8,
    .vbp            = 0,
    .vfp            = 20,
    .vsw            = 2,

    .init           = itomap2430_casio01_panel_init,
    .cleanup        = itomap2430_casio01_panel_cleanup,
    .enable         = itomap2430_casio01_panel_enable,
    .disable        = itomap2430_casio01_panel_disable,
    .get_caps       = itomap2430_casio01_panel_get_caps,
};

