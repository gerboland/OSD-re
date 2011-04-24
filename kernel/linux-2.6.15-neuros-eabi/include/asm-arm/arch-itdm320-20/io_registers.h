/** All register offset definitions for the TI DM320 */

#ifndef __ASM_ARCH_IO_REGISTERS1_H__
#define __ASM_ARCH_IO_REGISTERS1_H__

#define PHY_IO_BASE 0x00030000

/* Timer 0-3 */
#define IO_TIMER0_TMMD            0x0000
#define IO_TIMER0_TMRSV0          0x0002
#define IO_TIMER0_TMPRSCL         0x0004
#define IO_TIMER0_TMDIV           0x0006
#define IO_TIMER0_TMTRG           0x0008
#define IO_TIMER0_TMCNT           0x000A

#define IO_TIMER1_TMMD            0x0080
#define IO_TIMER1_TMRSV0          0x0082
#define IO_TIMER1_TMPRSCL         0x0084
#define IO_TIMER1_TMDIV           0x0086
#define IO_TIMER1_TMTRG           0x0088
#define IO_TIMER1_TMCNT           0x008A

#define IO_TIMER2_TMMD            0x0100
#define IO_TIMER2_TMVDCLR         0x0102
#define IO_TIMER2_TMPRSCL         0x0104
#define IO_TIMER2_TMDIV           0x0106
#define IO_TIMER2_TMTRG           0x0108
#define IO_TIMER2_TMCNT           0x010A

#define IO_TIMER3_TMMD            0x0180
#define IO_TIMER3_TMVDCLR         0x0182
#define IO_TIMER3_TMPRSCL         0x0184
#define IO_TIMER3_TMDIV           0x0186
#define IO_TIMER3_TMTRG           0x0188
#define IO_TIMER3_TMCNT           0x018A

/* Serial 0/1 */
#define IO_SERIAL0_TX_DATA        0x0200
#define IO_SERIAL0_RX_DATA        0x0202
#define IO_SERIAL0_TX_ENABLE      0x0204
#define IO_SERIAL0_MODE           0x0206
#define IO_SERIAL0_DMA_TRIGGER    0x0208
#define IO_SERIAL0_DMA_MODE       0x020A
#define IO_SERIAL0_DMA_SDRAM_LOW  0x020C
#define IO_SERIAL0_DMA_SDRAM_HI   0x020E
#define IO_SERIAL0_DMA_STATUS     0x0210

#define IO_SERIAL1_TX_DATA        0x0280
#define IO_SERIAL1_RX_DATA        0x0282
#define IO_SERIAL1_TX_ENABLE      0x0284
#define IO_SERIAL1_MODE           0x0286

/* UART 0/1 */
#define IO_UART0_DTRR             0x0300
#define IO_UART0_BRSR             0x0302
#define IO_UART0_MSR              0x0304
#define IO_UART0_RFCR             0x0306
#define IO_UART0_TFCR             0x0308
#define IO_UART0_LCR              0x030A
#define IO_UART0_SR               0x030C

#define IO_UART1_DTRR             0x0380
#define IO_UART1_BRSR             0x0382
#define IO_UART1_MSR              0x0384
#define IO_UART1_RFCR             0x0386
#define IO_UART1_TFCR             0x0388
#define IO_UART1_LCR              0x038A
#define IO_UART1_SR               0x038C

/* Watchdog Timer */
#define IO_WATCHDOG_MODE          0x0400
#define IO_WATCHDOG_RESET         0x0402
#define IO_WATCHDOG_PRESCALAR     0x0404
#define IO_WATCHDOG_DIVISOR       0x0406
#define IO_WATCHDOG_EXT_RESET     0x0408

/* MMC/SD Controller */
#define IO_MMC_CONTROL            0x0480
#define IO_MMC_MEM_CLK_CONTROL    0x0482
#define IO_MMC_STATUS0            0x0484
#define IO_MMC_STATUS1            0x0486
#define IO_MMC_INT_ENABLE         0x0488
#define IO_MMC_RESPONSE_TIMEOUT   0x048A
#define IO_MMC_READ_TIMEOUT       0x048C
#define IO_MMC_BLOCK_LENGTH       0x048E
#define IO_MMC_NR_BLOCKS          0x0490
#define IO_MMC_NR_BLOCKS_COUNT    0x0492
#define IO_MMC_RX_DATA            0x0494
#define IO_MMC_TX_DATA            0x0496
#define IO_MMC_COMMAND            0x0498
#define IO_MMC_ARG_LOW            0x049A
#define IO_MMC_ARG_HI             0x049C
#define IO_MMC_RESPONSE0          0x049E
#define IO_MMC_RESPONSE1          0x04A0
#define IO_MMC_RESPONSE2          0x04A2
#define IO_MMC_RESPONSE3          0x04A4
#define IO_MMC_RESPONSE4          0x04A6
#define IO_MMC_RESPONSE5          0x04A8
#define IO_MMC_RESPONSE6          0x04AA
#define IO_MMC_RESPONSE7          0x04AC
#define IO_MMC_SPI_DATA           0x04AE
#define IO_MMC_SPI_ERR            0x04B0
#define IO_MMC_DMASIZE            0x04B0 // do not use DMASIZE see erata
#define IO_MMC_COMMAND_INDEX      0x04B2
#define IO_MMC_CLK_START_PHASE    0x04B4
#define IO_MMC_RESPONSE_TOUT_CNT  0x04B6
#define IO_MMC_READ_TOUT_CNT      0x04B8
#define IO_MMC_BLOCK_LENGTH_CNT   0x04BA

#define IO_MMC_SD_DMA_TRIGGER     0x04BC
#define IO_MMC_SD_DMA_MODE        0x04BE
#define IO_MMC_SD_DMA_ADDR_LOW    0x04C0
#define IO_MMC_SD_DMA_ADDR_HI     0x04C2
#define IO_MMC_SD_DMA_STATUS0     0x04C4
#define IO_MMC_SD_DMA_STATUS1     0x04C6
#define IO_MMC_SD_DMA_TIMEOUT     0x04C8

#define IO_SDIO_CONTROL           0x04CA
#define IO_SDIO_STATUS0           0x04CC
#define IO_SDIO_INT_ENABLE        0x04CE
#define IO_SDIO_INT_STATUS        0x04D0

/* Interrupt Controller */
#define IO_INTC_FIQ0              0x0500
#define IO_INTC_FIQ1              0x0502
#define IO_INTC_FIQ2              0x0504
#define IO_INTC_IRQ0              0x0508
#define IO_INTC_IRQ1              0x050A
#define IO_INTC_IRQ2              0x050C
#define IO_INTC_FIQENTRY0         0x0510
#define IO_INTC_FIQENTRY1         0x0512
#define IO_INTC_FIQ_LOCK_ADDR0    0x0514
#define IO_INTC_FIQ_LOCK_ADDR1    0x0516 
#define IO_INTC_IRQENTRY0         0x0518
#define IO_INTC_IRQENTRY1         0x051A
#define IO_INTC_IRQ_LOCK_ADDR0    0x051C
#define IO_INTC_IRQ_LOCK_ADDR1    0x051E
#define IO_INTC_FISEL0            0x0520
#define IO_INTC_FISEL1            0x0522
#define IO_INTC_FISEL2            0x0524
#define IO_INTC_EINT0             0x0528
#define IO_INTC_EINT1             0x052A
#define IO_INTC_EINT2             0x052C
#define IO_INTC_RAW               0x0530
#define IO_INTC_ENTRY_TBA0        0x0538
#define IO_INTC_ENTRY_TBA1        0x053A
#define IO_INTC_PRIORITY0         0x0540
#define IO_INTC_PRIORITY1         0x0542
#define IO_INTC_PRIORITY2         0x0544
#define IO_INTC_PRIORITY3         0x0546
#define IO_INTC_PRIORITY4         0x0548
#define IO_INTC_PRIORITY5         0x054A
#define IO_INTC_PRIORITY6         0x054C
#define IO_INTC_PRIORITY7         0x054E
#define IO_INTC_PRIORITY8         0x0550
#define IO_INTC_PRIORITY9         0x0552
#define IO_INTC_PRIORITY10        0x0554
#define IO_INTC_PRIORITY11        0x0556
#define IO_INTC_PRIORITY12        0x0558
#define IO_INTC_PRIORITY13        0x055A
#define IO_INTC_PRIORITY14        0x055C
#define IO_INTC_PRIORITY15        0x055E
#define IO_INTC_PRIORITY16        0x0560
#define IO_INTC_PRIORITY17        0x0562
#define IO_INTC_PRIORITY18        0x0564
#define IO_INTC_PRIORITY19        0x0566
#define IO_INTC_PRIORITY20        0x0568
#define IO_INTC_PRIORITY21        0x056A
#define IO_INTC_PRIORITY22        0x056C

/* GIO Controller */
#define IO_GIO_DIR0               0x0580
#define IO_GIO_DIR1               0x0582
#define IO_GIO_DIR2               0x0584
#define IO_GIO_INV0               0x0586
#define IO_GIO_INV1               0x0588
#define IO_GIO_INV2               0x058A
#define IO_GIO_BITSET0            0x058C
#define IO_GIO_BITSET1            0x058E
#define IO_GIO_BITSET2            0x0590
#define IO_GIO_BITCLR0            0x0592
#define IO_GIO_BITCLR1            0x0594
#define IO_GIO_BITCLR2            0x0596
#define IO_GIO_IRQPORT            0x0598
#define IO_GIO_IRQEDGE            0x059A
#define IO_GIO_CHAT0              0x059C
#define IO_GIO_CHAT1              0x059E
#define IO_GIO_CHAT2              0x05A0
#define IO_GIO_NCHAT              0x05A2
#define IO_GIO_FSEL0              0x05A4
#define IO_GIO_FSEL1              0x05A6
#define IO_GIO_FSEL2              0x05A8
#define IO_GIO_FSEL3              0x05AA
#define IO_GIO_FSEL4              0x05AC
#define IO_GIO_CARD_SET           0x05AE
#define IO_GIO_CARD_ST            0x05B0

/* DSP Controller */
#define IO_DSPC_HPIB_CONTROL      0x0600
#define IO_DSPC_HPIB_STATUS       0x0602

/* OSD Controller */
#define IO_OSD_MODE               0x0680
#define IO_OSD_VIDWINMD           0x0682
#define IO_OSD_OSDWINMD0             0x0684
#define IO_OSD_OSDWINMD1             0x0686
//#define IO_OSD_ATRMD              0x0688
#define IO_OSD_RECTCUR            0x0688
#define IO_OSD_RESERVED           0x068A
#define IO_OSD_VIDWIN0OFST        0x068C
#define IO_OSD_VIDWIN1OFST        0x068E
#define IO_OSD_OSDWIN0OFST         0x0690
#define IO_OSD_OSDWIN1OFST         0x0692
#define IO_OSD_VIDWINADH          0x0694
#define IO_OSD_VIDWIN0ADL         0x0696
#define IO_OSD_VIDWIN1ADL         0x0698
#define IO_OSD_OSDWINADH             0x069A
#define IO_OSD_OSDWIN0ADL            0x069C
#define IO_OSD_OSDWIN1ADL            0x069E
#define IO_OSD_BASEPX             0x06A0
#define IO_OSD_BASEPY             0x06A2
#define IO_OSD_VIDWIN0XP          0x06A4
#define IO_OSD_VIDWIN0YP          0x06A6
#define IO_OSD_VIDWIN0XL          0x06A8
#define IO_OSD_VIDWIN0YL          0x06AA
#define IO_OSD_VIDWIN1XP          0x06AC
#define IO_OSD_VIDWIN1YP          0x06AE
#define IO_OSD_VIDWIN1XL          0x06B0
#define IO_OSD_VIDWIN1YL          0x06B2

#define IO_OSD_OSDWIN0XP          0x06B4
#define IO_OSD_OSDWIN0YP          0x06B6
#define IO_OSD_OSDWIN0XL          0x06B8
#define IO_OSD_OSDWIN0YL          0x06BA
#define IO_OSD_OSDWIN1XP          0x06BC
#define IO_OSD_OSDWIN1YP          0x06BE
#define IO_OSD_OSDWIN1XL          0x06C0
#define IO_OSD_OSDWIN1YL          0x06C2
#define IO_OSD_CURXP          	  0x06C4
#define IO_OSD_CURYP          	  0x06C6
#define IO_OSD_CURXL          	  0x06C8
#define IO_OSD_CURYL          	  0x06CA

#define IO_OSD_W0BMP01			  0x06D0
#define IO_OSD_W0BMP23			  0x06D2
#define IO_OSD_W0BMP45			  0x06D4
#define IO_OSD_W0BMP67			  0x06D6
#define IO_OSD_W0BMP89			  0x06D8
#define IO_OSD_W0BMPAB			  0x06DA
#define IO_OSD_W0BMPCD			  0x06DC
#define IO_OSD_W0BMPEF			  0x06DE

#define IO_OSD_W1BMP01			  0x06E0
#define IO_OSD_W1BMP23			  0x06E2
#define IO_OSD_W1BMP45			  0x06E4
#define IO_OSD_W1BMP67			  0x06E6
#define IO_OSD_W1BMP89			  0x06E8
#define IO_OSD_W1BMPAB			  0x06EA
#define IO_OSD_W1BMPCD			  0x06EC
#define IO_OSD_W1BMPEF			  0x06EE

#define IO_OSD_MISCCTL			  0x06F4
#define IO_OSD_CLUTRAMYCB		  0x06F6
#define IO_OSD_CLUTRAMCR		  0x06F8

#define IO_OSD_PPWIN0ADH		  0x06FC
#define IO_OSD_PPWIN0ADL		  0x06FE


/* CCD Controller */
#define IO_CCD_SYNCEN             0x0700
#define IO_CCD_MODESET            0x0702
#define IO_CCD_HDWIDTH            0x0704
#define IO_CCD_VDWIDTH            0x0706
#define IO_CCD_PPLN               0x0708
#define IO_CCD_LPFR               0x070A
#define IO_CCD_SPH                0x070C
#define IO_CCD_NPH                0x070E
#define IO_CCD_SLV0               0x0710
#define IO_CCD_SLV1               0x0712
#define IO_CCD_NLV                0x0714
#define IO_CCD_CULH               0x0716
#define IO_CCD_CULV               0x0718
#define IO_CCD_HSIZE              0x071A
#define IO_CCD_SDOFST             0x071C
#define IO_CCD_STADRH             0x071E
#define IO_CCD_STADRL             0x0720
#define IO_CCD_CLAMP              0x0722
#define IO_CCD_DCSUB              0x0724
#define IO_CCD_COLPTN             0x0726
#define IO_CCD_BLKCMP0            0x0728
#define IO_CCD_BLKCMP1            0x072A
#define IO_CCD_MEDFILT            0x072C
#define IO_CCD_RYEGAN             0x072E /* this is kept on the odd chance that some code is using the misspelled reg */
#define IO_CCD_RYEGAIN            0x072E
#define IO_CCD_GRCYGAIN           0x0730
#define IO_CCD_GBGGAIN            0x0732
#define IO_CCD_BMGGAIN            0x0734
#define IO_CCD_OFFSET             0x0736
#define IO_CCD_OUTCLP             0x0738
#define IO_CCD_VDINT0             0x073A
#define IO_CCD_VDINT1             0x073C
#define IO_CCD_RSV0               0x073E
#define IO_CCD_GAMMAWD            0x0740
#define IO_CCD_REC656IF           0x0742
#define IO_CCD_CCDFG              0x0744
#define IO_CCD_FMTCFG             0x0746
#define IO_CCD_FMTSPH             0x0748
#define IO_CCD_FMTLNH             0x074A
#define IO_CCD_FMTSLV             0x074C
#define IO_CCD_FMTSNV             0x074E
#define IO_CCD_FMTOFST            0x0750
#define IO_CCD_FMTRLEN            0x0752
#define IO_CCD_FMTHCNT            0x0754
#define IO_CCD_FMTPTNA            0x0756
#define IO_CCD_FMTPTNB            0x0758

/* NTSC/PAL Encoder */
#define IO_VID_ENC_VMOD           0x0800
#define IO_VID_ENC_VDCTL          0x0802
#define IO_VID_ENC_VDPRO          0x0804
#define IO_VID_ENC_SYNCCTL        0x0806
#define IO_VID_ENC_HSPLS          0x0808
#define IO_VID_ENC_VSPLS          0x080A
#define IO_VID_ENC_HINT           0x080C
#define IO_VID_ENC_HSTART         0x080E
#define IO_VID_ENC_HVALID         0x0810
#define IO_VID_ENC_VINT           0x0812
#define IO_VID_ENC_VSTART         0x0814
#define IO_VID_ENC_VVALID         0x0816
#define IO_VID_ENC_HSDLY          0x0818
#define IO_VID_ENC_VSDLY          0x081A
#define IO_VID_ENC_YCCTL          0x081C
#define IO_VID_ENC_RGBCTL         0x081E
#define IO_VID_ENC_RGBCLP         0x0820
#define IO_VID_ENC_LNECTL         0x0822
#define IO_VID_ENC_CULLLNE        0x0824
#define IO_VID_ENC_LCDOUT         0x0826
#define IO_VID_ENC_BRTS           0x0828
#define IO_VID_ENC_BRTW           0x082A
#define IO_VID_ENC_ACCTL          0x082C
#define IO_VID_ENC_PWMP           0x082E
#define IO_VID_ENC_PWMW           0x0830
#define IO_VID_ENC_DCLKCTL        0x0832
#define IO_VID_ENC_DCLKPTN0       0x0834
#define IO_VID_ENC_DCLKPTN1       0x0836
#define IO_VID_ENC_DCLKPTN2       0x0838
#define IO_VID_ENC_DCLKPTN3       0x083A
#define IO_VID_ENC_DCLKPTN0A      0x083C
#define IO_VID_ENC_DCLKPTN1A      0x083E
#define IO_VID_ENC_DCLKPTN2A      0x0840
#define IO_VID_ENC_DCLKPTN3A      0x0842
#define IO_VID_ENC_DCLKHS         0x0844
#define IO_VID_ENC_DCLKHSA        0x0846
#define IO_VID_ENC_DCLKHR         0x0848
#define IO_VID_ENC_DCLKVS         0x084A
#define IO_VID_ENC_DCLKVR         0x084C
#define IO_VID_ENC_CAPCTL         0x084E
#define IO_VID_ENC_CAPDO          0x0850
#define IO_VID_ENC_CAPDE          0x0852
#define IO_VID_ENC_ATR0           0x0854

/* Clock Controller */
#define IO_CLK_PLLA               0x0880
#define IO_CLK_PLLB               0x0882
#define IO_CLK_SEL0               0x0884
#define IO_CLK_SEL1               0x0886
#define IO_CLK_SEL2               0x0888
#define IO_CLK_DIV0               0x088A
#define IO_CLK_DIV1               0x088C
#define IO_CLK_DIV2               0x088E
#define IO_CLK_DIV3               0x0890
#define IO_CLK_DIV4               0x0892
#define IO_CLK_BYP                0x0894
#define IO_CLK_INV                0x0896
#define IO_CLK_MOD0               0x0898
#define IO_CLK_MOD1               0x089A
#define IO_CLK_MOD2               0x089C
#define IO_CLK_LPCTL0             0x089E
#define IO_CLK_LPCTL1             0x08A0
#define IO_CLK_OSEL               0x08A2
#define IO_CLK_00DIV              0x08A4
#define IO_CLK_O1DIV              0x08A6
#define IO_CLK_02DIV              0x08A8
#define IO_CLK_PWM0C              0x08AA
#define IO_CLK_PWM0H              0x08AC
#define IO_CLK_PWM1C              0x08AE
#define IO_CLK_PWM1H              0x08B0

/* Bus Controller */
#define IO_BUSC_ECR               0x0900
#define IO_BUSC_EBYTER            0x0902
#define IO_BUSC_EBITR             0x0904
#define IO_BUSC_REVR              0x0906

/* SDRAM Controller */
#define IO_SDRAM_SDBUFD0L         0x0980
#define IO_SDRAM_SDBUFD0H         0x0982
#define IO_SDRAM_SDBUFD1L         0x0984
#define IO_SDRAM_SDBUFD1H         0x0986
#define IO_SDRAM_SDBUFD2L         0x0988
#define IO_SDRAM_SDBUFD2H         0x098A
#define IO_SDRAM_SDBUFD3L         0x098C
#define IO_SDRAM_SDBUFD3H         0x098E
#define IO_SDRAM_SDBUFD4L         0x0990
#define IO_SDRAM_SDBUFD4H         0x0992
#define IO_SDRAM_SDBUFD5L         0x0994
#define IO_SDRAM_SDBUFD5H         0x0996
#define IO_SDRAM_SDBUFD6L         0x0998
#define IO_SDRAM_SDBUFD6H         0x099A
#define IO_SDRAM_SDBUFD7L         0x099C
#define IO_SDRAM_SDBUFD7H         0x099E
#define IO_SDRAM_SDBUFAD1         0x09A0
#define IO_SDRAM_SDBUFAD2         0x09A2
#define IO_SDRAM_SDBUFCTL         0x09A4
#define IO_SDRAM_SDMODE           0x09A6
#define IO_SDRAM_REFCTL           0x09A8
#define IO_SDRAM_SDPRTY1          0x09AA
#define IO_SDRAM_SDPRTY2          0x09AC
#define IO_SDRAM_SDPRTY3          0x09AE
#define IO_SDRAM_SDPRTY4          0x09B0
#define IO_SDRAM_SDPRTY5          0x09B2
#define IO_SDRAM_SDPRTY6          0x09B4
#define IO_SDRAM_SDPRTY7          0x09B6
#define IO_SDRAM_SDPRTY8          0x09B8
#define IO_SDRAM_SDPRTY9          0x09BA
#define IO_SDRAM_SDPRTY10         0x09BC
#define IO_SDRAM_SDPRTY11         0x09BE
#define IO_SDRAM_SDPRTY12         0x09C0
#define IO_SDRAM_RSV              0x09C2
#define IO_SDRAM_SDPRTYON         0x09C4
#define IO_SDRAM_SDDMASEL         0x09C6

/* EMIF Controller */
#define IO_EMIF_CS0CTRL1          0x0A00
#define IO_EMIF_CS0CTRL2          0x0A02
#define IO_EMIF_CS0CTRL3          0x0A04
#define IO_EMIF_CS1CTRL1A         0x0A06
#define IO_EMIF_CS1CTRL1B         0x0A08
#define IO_EMIF_CS1CTRL2          0x0A0A
#define IO_EMIF_CS2CTRL1          0x0A0C
#define IO_EMIF_CS2CTRL2          0x0A0E
#define IO_EMIF_CS3CTRL1          0x0A10
#define IO_EMIF_CS3CTRL2          0x0A12
#define IO_EMIF_CS4CTRL1          0x0A14
#define IO_EMIF_CS4CTRL2          0x0A16
#define IO_EMIF_BUSCTRL           0x0A18
#define IO_EMIF_BUSRLS            0x0A1A
#define IO_EMIF_CFCTRL1           0x0A1C
#define IO_EMIF_CFCTRL2           0x0A1E
#define IO_EMIF_SMCTRL            0x0A20
#define IO_EMIF_BUSINTEN          0x0A22
#define IO_EMIF_BUSSTS            0x0A24
#define IO_EMIF_BUSWAITMD         0x0A26
#define IO_EMIF_ECC1CP            0x0A28
#define IO_EMIF_ECC1LP            0x0A2A
#define IO_EMIF_ECC2CP            0x0A2C
#define IO_EMIF_ECC2LP            0x0A2E
#define IO_EMIF_ECC3CP            0x0A30
#define IO_EMIF_ECC3LP            0x0A32
#define IO_EMIF_ECC4CP            0x0A34
#define IO_EMIF_ECC4LP            0x0A36
#define IO_EMIF_ECC5CP            0x0A38
#define IO_EMIF_ECC5LP            0x0A3A
#define IO_EMIF_ECC6CP            0x0A3C
#define IO_EMIF_ECC6LP            0x0A3E 
#define IO_EMIF_ECC7CP            0x0A40
#define IO_EMIF_ECC7LP            0x0A42
#define IO_EMIF_ECC8CP            0x0A44
#define IO_EMIF_ECC8LP            0x0A46
#define IO_EMIF_ECCCLR            0x0A48
#define IO_EMIF_PAGESZ            0x0A4A
#define IO_EMIF_PRIORCTL          0x0A4C
#define IO_EMIF_MGDSPDEST         0x0A4E
#define IO_EMIF_MGDSPADDH         0x0A50
#define IO_EMIF_MGDSPADDL         0x0A52
#define IO_EMIF_AHBADDH           0x0A54
#define IO_EMIF_AHBADDL           0x0A56
#define IO_EMIF_MTCADDH           0x0A58
#define IO_EMIF_MTCADDL           0x0A5A
#define IO_EMIF_DMASIZE           0x0A5C
#define IO_EMIF_DMAMTCSEL         0x0A5E
#define IO_EMIF_DMACTL            0x0A60

/* Preivew Engine */
#define IO_PREV_ENG_PVEN          0x0A80
#define IO_PREV_ENG_PVSET1        0x0A82
#define IO_PREV_ENG_RADRH         0x0A84
#define IO_PREV_ENG_RADRL         0x0A86
#define IO_PREV_ENG_WADRH         0x0A88
#define IO_PREV_ENG_WADRL         0x0A8A
#define IO_PREV_ENG_HSTART        0x0A8C
#define IO_PREV_ENG_HSIZE         0x0A8E
#define IO_PREV_ENG_VSTART        0x0A90
#define IO_PREV_ENG_VSIZE         0x0A92
#define IO_PREV_ENG_PVSET2        0x0A94
#define IO_PREV_ENG_NFILT         0x0A96
#define IO_PREV_ENG_DGAIN         0x0A98
#define IO_PREV_ENG_WBGAIN0       0x0A9A
#define IO_PREV_ENG_WBGAIN1       0x0A9C
#define IO_PREV_ENG_SMTH          0x0A9E
#define IO_PREV_ENG_HRSZ          0x0AA0
#define IO_PREV_ENG_VRSZ          0x0AA2
#define IO_PREV_ENG_BLOFST0       0x0AA4
#define IO_PREV_ENG_BLOFST1       0x0AA6
#define IO_PREV_ENG_MTXGAIN0      0x0AA8 
#define IO_PREV_ENG_MTXGAIN1      0x0AAA
#define IO_PREV_ENG_MTXGAIN2      0x0AAC
#define IO_PREV_ENG_MTXGAIN3      0x0AAE
#define IO_PREV_ENG_MTXGAIN4      0x0AB0
#define IO_PREV_ENG_MTXGAIN5      0x0AB2
#define IO_PREV_ENG_MTXGAIN6      0x0AB4
#define IO_PREV_ENG_MTXGAIN7      0x0AB6
#define IO_PREV_ENG_MTXGAIN8      0x0AB8
#define IO_PREV_ENG_MTXOFST0      0x0ABA
#define IO_PREV_ENG_MTXOFST1      0x0ABC
#define IO_PREV_ENG_MTXOFST2      0x0ABE
#define IO_PREV_ENG_GAMTBYP       0x0AC0
#define IO_PREV_ENG_CSC0          0x0AC2
#define IO_PREV_ENG_CSC1          0x0AC4
#define IO_PREV_ENG_CSC2          0x0AC6
#define IO_PREV_ENG_CSC3          0x0AC8
#define IO_PREV_ENG_CSC4          0x0ACA
#define IO_PREV_ENG_YOFST         0x0ACC
#define IO_PREV_ENG_COFST         0x0ACE
#define IO_PREV_ENG_CNTBRT        0x0AD0
#define IO_PREV_ENG_CSUP0         0x0AD2
#define IO_PREV_ENG_CSUP1         0x0AD4
#define IO_PREV_ENG_SETUPY        0x0AD4
#define IO_PREV_ENG_SETUPC        0x0AD8
#define IO_PREV_ENG_TABLE_ADDR    0x0ADA
#define IO_PREV_ENG_TABLE_DATA    0x0ADC
#define IO_PREV_ENG_HG_CTL        0x0ADE
#define IO_PREV_ENG_HG_R0_HSTART  0x0AE0
#define IO_PREV_ENG_HG_R0_HSIZE   0x0AE2
#define IO_PREV_ENG_HG_R0_VSTART  0x0AE4
#define IO_PREV_ENG_HR_R0_VSIZE   0x0AE6
#define IO_PREV_ENG_HG_R1_HSTART  0x0AE8
#define IO_PREV_ENG_HG_R1_HSIZE   0x0AEA
#define IO_PREV_ENG_HG_R1_VSTART  0x0AEC
#define IO_PREV_ENG_HG_R1_VSIZE   0x0AEE
#define IO_PREV_ENG_HG_R2_HSTART  0x0AF0
#define IO_PREV_ENG_HG_R2_HSIZE   0x0AF2
#define IO_PREV_ENG_HG_R2_VSTART  0x0AF4
#define IO_PREV_ENG_HG_R2_VSIZE   0x0AF6
#define IO_PREV_ENG_HG_R3_HSTART  0x0AF8
#define IO_PREV_ENG_HG_R3_HSIZE   0x0AFA 
#define IO_PREV_ENG_HG_R3_VSTART  0x0AFC
#define IO_PREV_ENG_HG_R3_VSIZE   0x0AFE
#define IO_PREV_ENG_HG_ADDR       0x0B00
#define IO_PREV_ENG_HG_DATA       0x0B02

/* H3A Hardware */
#define IO_H3A_H3ACTRL            0x0B80
#define IO_H3A_AFCTRL             0x0B82
#define IO_H3A_AFPAX1             0x0B84
#define IO_H3A_AFPAX2             0x0B86
#define IO_H3A_AFPAX3             0x0B88
#define IO_H3A_AFPAX4             0x0B8A
#define IO_H3A_AFIRSH             0x0B8C
#define IO_H3A_AFPAX5             0x0B8E
#define IO_H3A_AFSDRA1            0x0B90
#define IO_H3A_AFSDRA2            0x0B92
#define IO_H3A_AFSDRFLG           0x0B94
#define IO_H3A_AFCOEFF10          0x0B96
#define IO_H3A_AFCOEFF11          0x0B98
#define IO_H3A_AFCOEFF12          0x0B9A
#define IO_H3A_AFCOEFF13          0x0B9C
#define IO_H3A_AFCOEFF14          0x0B9E
#define IO_H3A_AFCOEFF15          0x0BA0
#define IO_H3A_AFCOEFF16          0x0BA2
#define IO_H3A_AFCOEFF17          0x0BA4
#define IO_H3A_AFCOEFF18          0x0BA6
#define IO_H3A_AFCOEFF19          0x0BA8
#define IO_H3A_AFCOEFF110         0x0BAA
#define IO_H3A_AFCOEFF20          0x0BAC
#define IO_H3A_AFCOEFF21          0x0BAE
#define IO_H3A_AFCOEFF22          0x0BB0
#define IO_H3A_AFCOEFF23          0x0BB2
#define IO_H3A_AFCOEFF24          0x0BB4
#define IO_H3A_AFCOEFF25          0x0BB6
#define IO_H3A_AFCOEFF26          0x0BB8
#define IO_H3A_AFCOEFF27          0x0BBA
#define IO_H3A_AFCOEFF28          0x0BBC
#define IO_H3A_AFCOEFF29          0x0BBE
#define IO_H3A_AFCOEFF210         0x0BC0
#define IO_H3A_AEWCTRL            0x0BC2
#define IO_H3A_AEWWIN1            0x0BC4
#define IO_H3A_AEWWIN2            0x0BC6
#define IO_H3A_AEWWIN3            0x0BC8
#define IO_H3A_AEWWIN4            0x0BCA
#define IO_H3A_AEWWIN5            0x0BCC
#define IO_H3A_AEWSDRA1           0x0BCE
#define IO_H3A_AEWSDRA2           0x0BD0
#define IO_H3A_AEWSDRFLG          0x0BD2

/* Reserved 0x0C00 - 0x0CCFF */

/* Memory Stick Controller : */
#define IO_MEM_STICK_MODE       0x0C80
#define IO_MEM_STICK_CMD        0x0C82
#define IO_MEM_STICK_DATA       0x0C84
#define IO_MEM_STICK_STATUS     0x0C86
#define IO_MEM_STICK_SYS        0x0C88
#define IO_MEM_STICK_ENDIAN     0x0C8A
#define IO_MEM_STICK_INT_STATUS 0x0C8C
#define IO_MEM_STICK_DMA_TRG    0x0C8E
#define IO_MEM_STICK_DMA_MODE   0x0C90
#define IO_MEM_STICK_SDRAM_ADDL 0x0C92
#define IO_MEM_STICK_SDRAM_ADDH 0x0C94
#define IO_MEM_STICK_DMA_STATUS 0x0C96

/* ATM : WBB Need to find these Register values */
#define IO_ATM_                   0x0D00

/* I2C */
#define IO_I2C_TXDATA             0x0D80
#define IO_I2C_RXDATA             0x0D82
#define IO_I2C_SCS                0x0D84

#endif
