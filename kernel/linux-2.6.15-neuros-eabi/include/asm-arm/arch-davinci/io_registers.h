/** Register offset definitions for the TI DM420 */

#ifndef __ASM_ARCH_IO_REGISTERS1_H__
#define __ASM_ARCH_IO_REGISTERS1_H__


#define VPSS_OFFSET (DAVINCI_VPSS_REGS_BASE_ADDR-DAVINCI_PERI_PADDR)

#define IO_OSD_MODE               (VPSS_OFFSET + 0x2600)
#define IO_OSD_VIDWINMD           (VPSS_OFFSET + 0x2604)
#define IO_OSD_OSDWINMD0          (VPSS_OFFSET + 0x2608)
#define IO_OSD_OSDWINMD1          (VPSS_OFFSET + 0x260C)
#define IO_OSD_RECTCUR            (VPSS_OFFSET + 0x2610)
#define IO_OSD_RESERVED           (VPSS_OFFSET + 0x2614)
#define IO_OSD_VIDWIN0OFST        (VPSS_OFFSET + 0x2618)
#define IO_OSD_VIDWIN1OFST        (VPSS_OFFSET + 0x261C)
#define IO_OSD_OSDWIN0OFST        (VPSS_OFFSET + 0x2620)
#define IO_OSD_OSDWIN1OFST        (VPSS_OFFSET + 0x2624)

#define IO_OSD_VIDWIN0ADR         (VPSS_OFFSET + 0x262C)
#define IO_OSD_VIDWIN1ADR         (VPSS_OFFSET + 0x2630)

#define IO_OSD_OSDWIN0ADR         (VPSS_OFFSET + 0x2638)
#define IO_OSD_OSDWIN1ADR         (VPSS_OFFSET + 0x263C)
#define IO_OSD_BASEPX             (VPSS_OFFSET + 0x2640)
#define IO_OSD_BASEPY             (VPSS_OFFSET + 0x2644)
#define IO_OSD_VIDWIN0XP          (VPSS_OFFSET + 0x2648)
#define IO_OSD_VIDWIN0YP          (VPSS_OFFSET + 0x264C)
#define IO_OSD_VIDWIN0XL          (VPSS_OFFSET + 0x2650)
#define IO_OSD_VIDWIN0YL          (VPSS_OFFSET + 0x2654)
#define IO_OSD_VIDWIN1XP          (VPSS_OFFSET + 0x2658)
#define IO_OSD_VIDWIN1YP          (VPSS_OFFSET + 0x265C)
#define IO_OSD_VIDWIN1XL          (VPSS_OFFSET + 0x2660)
#define IO_OSD_VIDWIN1YL          (VPSS_OFFSET + 0x2664)

#define IO_OSD_OSDWIN0XP          (VPSS_OFFSET + 0x2668)
#define IO_OSD_OSDWIN0YP          (VPSS_OFFSET + 0x266C)
#define IO_OSD_OSDWIN0XL          (VPSS_OFFSET + 0x2670)
#define IO_OSD_OSDWIN0YL          (VPSS_OFFSET + 0x2674)
#define IO_OSD_OSDWIN1XP          (VPSS_OFFSET + 0x2678)
#define IO_OSD_OSDWIN1YP          (VPSS_OFFSET + 0x267C)
#define IO_OSD_OSDWIN1XL          (VPSS_OFFSET + 0x2680)
#define IO_OSD_OSDWIN1YL          (VPSS_OFFSET + 0x2684)
#define IO_OSD_CURXP          	  (VPSS_OFFSET + 0x2688)
#define IO_OSD_CURYP          	  (VPSS_OFFSET + 0x268C)
#define IO_OSD_CURXL          	  (VPSS_OFFSET + 0x2690)
#define IO_OSD_CURYL          	  (VPSS_OFFSET + 0x2694)

#define IO_OSD_W0BMP01		  (VPSS_OFFSET + 0x26A0)
#define IO_OSD_W0BMP23		  (VPSS_OFFSET + 0x26A4)
#define IO_OSD_W0BMP45		  (VPSS_OFFSET + 0x26A8)
#define IO_OSD_W0BMP67		  (VPSS_OFFSET + 0x26AC)
#define IO_OSD_W0BMP89		  (VPSS_OFFSET + 0x26B0)
#define IO_OSD_W0BMPAB	          (VPSS_OFFSET + 0x26B4)
#define IO_OSD_W0BMPCD		  (VPSS_OFFSET + 0x26B8)
#define IO_OSD_W0BMPEF		  (VPSS_OFFSET + 0x26BC)

#define IO_OSD_W1BMP01		  (VPSS_OFFSET + 0x26C0)
#define IO_OSD_W1BMP23		  (VPSS_OFFSET + 0x26C4)
#define IO_OSD_W1BMP45		  (VPSS_OFFSET + 0x26C8)
#define IO_OSD_W1BMP67		  (VPSS_OFFSET + 0x26CC)
#define IO_OSD_W1BMP89		  (VPSS_OFFSET + 0x26D0)
#define IO_OSD_W1BMPAB		  (VPSS_OFFSET + 0x26D4)
#define IO_OSD_W1BMPCD		  (VPSS_OFFSET + 0x26D8)
#define IO_OSD_W1BMPEF		  (VPSS_OFFSET + 0x26DC)
#define	IO_OSD_VBNDRY		  (VPSS_OFFSET + 0x26E0)	// Vertical Boundary Display Control
#define	IO_OSD_EXTENDMD		  (VPSS_OFFSET + 0x26E4)        // Extend Mode
#define IO_OSD_MISCCTL		  (VPSS_OFFSET + 0x26E8)
#define IO_OSD_CLUTRAMYCB	  (VPSS_OFFSET + 0x26EC)
#define IO_OSD_CLUTRAMCR	  (VPSS_OFFSET + 0x26F0)
#define	IO_OSD_TRANSPL		  (VPSS_OFFSET + 0x26F4)	// Bitmap Window Transparent Color Code - Low
#define	IO_OSD_TRANSPH		  (VPSS_OFFSET + 0x26F6)	// Bitmap Window Transparent Color Code - High
#define IO_OSD_PPVWIN0ADR         (VPSS_OFFSET + 0x26FC)


#endif
