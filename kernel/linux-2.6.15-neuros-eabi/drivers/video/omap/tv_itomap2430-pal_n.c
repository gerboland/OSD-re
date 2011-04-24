/*
 * File: drivers/video/omap/lcd-itomap2430-pal_n.c
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
#include "venc.h"

static int write_venc(struct tv_standard_config* ptv)
{
    if (ptv == NULL)
        return(-1);

    outl(0, VIDENC_F_CONTROL);
    outl(0x00001040, VIDENC_SYNC_CTRL);
    outl(ptv->llen, VIDENC_LLEN);
    outl(ptv->flens, VIDENC_FLENS);
    outl(ptv->hfltr_ctrl, VIDENC_HFLTR_CTRL);
    outl(ptv->cc_carr_wss_carr , VIDENC_CC_CARR_WSS_CARR);
    outl(ptv->c_phase , VIDENC_C_PHASE);
    outl(ptv->gain_u, VIDENC_GAIN_U);
    outl(ptv->gain_v, VIDENC_GAIN_V);
    outl(ptv->gain_y, VIDENC_GAIN_Y);
    outl(ptv->black_level, VIDENC_BLACK_LEVEL);
    outl(ptv->blank_level, VIDENC_BLANK_LEVEL);
    outl(ptv->x_color, VIDENC_X_COLOR);
    outl(ptv->m_control, VIDENC_M_CONTROL);
    outl(ptv->bstamp_wss_data, VIDENC_BSTAMP_WSS_DATA);
    outl(ptv->s_carr, VIDENC_S_CARR);
    outl(ptv->line21 , VIDENC_LINE21);
    outl(ptv->ln_sel , VIDENC_LN_SEL);
    outl(ptv->l21_wc_ctl, VIDENC_L21_WC_CTRL);
    outl(ptv->htrigger_vtrigger , VIDENC_HTRIGGER_VTRIGGER);
    outl(ptv->savid_eavid, VIDENC_SAVID_EAVID);
    outl(ptv->flen_fal, VIDENC_FLEN_FAL);
    outl(ptv->lal_phase_reset, VIDENC_LAL_PHASE_RESET);
    outl(ptv->hs_int_start_stop_x, VIDENC_HS_INT_START_STOP_X);
    outl(ptv->hs_ext_start_stop_x, VIDENC_HS_EXT_START_STOP_X);
    outl(ptv->vs_int_start_x, VIDENC_VS_INT_START_X);
    outl(ptv->vs_int_stop_x_vs_int_start_y, VIDENC_VS_INT_STOP_X_VS_INT_START_Y);
    outl(ptv->vs_int_stop_y_vs_ext_start_x, VIDENC_VS_INT_STOP_Y_VS_EXT_START_X);
    outl(ptv->vs_ext_stop_x_vs_ext_start_y, VIDENC_VS_EXT_STOP_X_VS_EXT_START_Y);
    outl(ptv->vs_ext_stop_y, VIDENC_VS_EXT_STOP_Y);
    outl(ptv->avid_start_stop_x, VIDENC_AVID_START_STOP_X);
    outl(ptv->avid_start_stop_y, VIDENC_AVID_START_STOP_Y);
    outl(ptv->fid_int_start_x_fid_int_start_y, VIDENC_FID_INT_START_X_FID_INT_START_Y);
    outl(ptv->fid_int_offset_y_fid_ext_start_x, VIDENC_FID_INT_OFFSET_Y_FID_EXT_START_X);
    outl(ptv->fid_ext_start_y_fid_ext_offset_y, VIDENC_FID_EXT_START_Y_FID_EXT_OFFSET_Y);
    outl(ptv->tvdetgp_int_start_stop_x, VIDENC_TVDETGP_INT_START_STOP_X);
    outl(ptv->tvdetgp_int_start_stop_y, VIDENC_TVDETGP_INT_START_STOP_Y);
    outl(ptv->gen_ctrl, VIDENC_GEN_CTRL);
    outl(ptv->dac_tst, VIDENC_DAC_TST);

    return(0);
}

static int itomap2430_pal_n_panel_init(struct omapfb_device *fbdev)
{
    int status;
    DBGENTER(1);
    status = write_venc(&pal_n_cfg);
    DBGLEAVE(1);
    return(status);
}

static void itomap2430_pal_n_panel_cleanup(void)
{
    DBGENTER(1);
    DBGLEAVE(1);
}

static int itomap2430_pal_n_panel_enable(void)
{

    DBGENTER(1);
    DBGLEAVE(1);
    return 0;
}

static void itomap2430_pal_n_panel_disable(void)
{
    DBGENTER(1);
    DBGLEAVE(1);
}

static unsigned long itomap2430_pal_n_panel_get_caps(void)
{
    return 0;
}

struct lcd_panel itomap2430_pal_n_panel = {
    .name           = "itomap2430",
    .display_type   = OMAP_TVC_TYPE,
    .config         = OMAP_LCDC_PANEL_TFT,

    .bpp            = 16,
    .data_lines     = 24,
    .x_res          = 640,
    .y_res          = 480,
    .pixel_clock    = 16000,

    .hbp            = 40,
    .hfp            = 4,
    .hsw            = 4,
    .vbp            = 8,
    .vfp            = 2,
    .vsw            = 2,

    .init           = itomap2430_pal_n_panel_init,
    .cleanup        = itomap2430_pal_n_panel_cleanup,
    .enable         = itomap2430_pal_n_panel_enable,
    .disable        = itomap2430_pal_n_panel_disable,
    .get_caps       = itomap2430_pal_n_panel_get_caps,
};

