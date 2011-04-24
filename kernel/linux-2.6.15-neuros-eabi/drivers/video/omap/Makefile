#
# Makefile for the new OMAP framebuffer device driver
#

obj-$(CONFIG_FB_OMAP) += omapfb.o

objs-yy := omapfb_main.o

objs-y$(CONFIG_ARCH_OMAP2) += dispc.o

objs-$(CONFIG_ARCH_OMAP1)$(CONFIG_FB_OMAP_LCDC_INTERNAL) += lcdc.o

objs-$(CONFIG_ARCH_OMAP1)$(CONFIG_FB_OMAP_LCDC_EXTERNAL) += sossi.o
objs-$(CONFIG_ARCH_OMAP2)$(CONFIG_FB_OMAP_LCDC_EXTERNAL) += rfbi.o

objs-y$(CONFIG_MACH_OMAP_H4) += lcd_h4.o
objs-y$(CONFIG_MACH_OMAP_H3) += lcd_h3.o
objs-y$(CONFIG_MACH_OMAP_H2) += lcd_h2.o
objs-y$(CONFIG_MACH_OMAP_PALMTE) += lcd_palmte.o
objs-$(CONFIG_ARCH_OMAP16XX)$(CONFIG_MACH_OMAP_INNOVATOR) += lcd_inn1610.o
objs-$(CONFIG_ARCH_OMAP15XX)$(CONFIG_MACH_OMAP_INNOVATOR) += lcd_inn1510.o
objs-y$(CONFIG_MACH_OMAP_OSK) += lcd_osk.o
objs-y$(CONFIG_MACH_OMAP_PERSEUS2) += lcd_p2.o
objs-y$(CONFIG_FB_OMAP_LCDC_SHARP01) += lcd_itomap2430-sharp01.o
objs-y$(CONFIG_FB_OMAP_LCDC_CASIO01) += lcd_itomap2430-casio01.o
objs-y$(CONFIG_FB_OMAP_TVC_NTSC_M_PANEL) += tv_itomap2430-ntsc_m.o
objs-y$(CONFIG_FB_OMAP_TVC_NTSC_J_PANEL) += tv_itomap2430-ntsc_j.o
objs-y$(CONFIG_FB_OMAP_TVC_NTSC_443_PANEL) += tv_itomap2430-ntsc_443.o
objs-y$(CONFIG_FB_OMAP_TVC_PAL_BDGHI_PANEL) += tv_itomap2430-pal_bdghi.o
objs-y$(CONFIG_FB_OMAP_TVC_PAL_NC_PANEL) += tv_itomap2430-pal_nc.o
objs-y$(CONFIG_FB_OMAP_TVC_PAL_N_PANEL) += tv_itomap2430-pal_n.o
objs-y$(CONFIG_FB_OMAP_TVC_PAL_M_PANEL) += tv_itomap2430-pal_m.o
objs-y$(CONFIG_FB_OMAP_TVC_PAL_60_PANEL) += tv_itomap2430-pal_60.o

omapfb-objs := $(objs-yy)

