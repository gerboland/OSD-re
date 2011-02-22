/*
 *  linux/include/asm/arch/edma.h
 *
 *  BRIEF MODULE DESCRIPTION
 *      TI DAVINCI dma definitions
 *
 *  Copyright (C) 2004 Texas Instruments.
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *  THIS  SOFTWARE  IS PROVIDED   ``AS  IS'' AND   ANY  EXPRESS OR IMPLIED
 *  WARRANTIES,   INCLUDING, BUT NOT  LIMITED  TO, THE IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN
 *  NO  EVENT  SHALL   THE AUTHOR  BE    LIABLE FOR ANY   DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 *  NOT LIMITED   TO, PROCUREMENT OF  SUBSTITUTE GOODS  OR SERVICES; LOSS OF
 *  USE, DATA,  OR PROFITS; OR  BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 *  ANY THEORY OF LIABILITY, WHETHER IN  CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 *  THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *  You should have received a copy of the  GNU General Public License along
 *  with this program; if not, write  to the Free Software Foundation, Inc.,
 *  675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */
/******************************************************************************
 * DMA driver for DaVinci
 * DMA driver for Davinci abstractes each ParamEntry as a Logical DMA channel
 * for the user.So on Davinci the user can request 128 DAM channels
 *
 * Actual Physical DMA channels = 64 EDMA channels + 8 QDMA channels
 *
 * On davinci user can request for two kinds of Logical DMA channels
 * DMA MasterChannel -> ParamEntry which is associated with a DMA channel.
 *                      On Davinci there are (64 + 8) MasterChanneles
 *                      MasterChannel can be triggered by an event or manually
 *
 * DMA SlaveChannel  -> ParamEntry which is not associated with DMA cahnnel but
 *                      which can be used to associate with MasterChannel.
 *                      On Davinci there are (128-(64 + 8)) SlaveChannels
 *                      SlaveChannel can only be triggered by a MasterChannel
 *
 */

#ifndef EDMA_H_
#define EDMA_H_

/*Used by driver*/

/**************************************************************************\
* Register Overlay Structure for DRA
\**************************************************************************/
typedef struct  {
	volatile unsigned int drae;
	volatile unsigned int draeh;
} edmacc_dra_regs;

/**************************************************************************\
* Register Overlay Structure for QUEEVTENTRY
\**************************************************************************/
typedef struct  {
    volatile unsigned int evt_entry;
} edmacc_que_evtentry_regs;

/**************************************************************************\
* Register Overlay Structure for SHADOW
\**************************************************************************/
typedef struct  {
    	volatile unsigned int er;
	volatile unsigned int erh;
	volatile unsigned int ecr;
	volatile unsigned int ecrh;
	volatile unsigned int esr;
	volatile unsigned int esrh;
	volatile unsigned int cer;
	volatile unsigned int cerh;
	volatile unsigned int eer;
	volatile unsigned int eerh;
	volatile unsigned int eecr;
	volatile unsigned int eecrh;
	volatile unsigned int eesr;
	volatile unsigned int eesrh;
	volatile unsigned int ser;
	volatile unsigned int serh;
	volatile unsigned int secr;
	volatile unsigned int secrh;
	volatile unsigned char rsvd0[8];
	volatile unsigned int ier;
	volatile unsigned int ierh;
	volatile unsigned int iecr;
	volatile unsigned int iecrh;
	volatile unsigned int iesr;
	volatile unsigned int iesrh;
	volatile unsigned int ipr;
	volatile unsigned int iprh;
	volatile unsigned int icr;
	volatile unsigned int icrh;
	volatile unsigned int ieval;
	volatile unsigned char rsvd1[4];
	volatile unsigned int qer;
	volatile unsigned int qeer;
	volatile unsigned int qeecr;
	volatile unsigned int qeesr;
	volatile unsigned int qser;
	volatile unsigned int qsecr;
	volatile unsigned char rsvd2[360];
} edmacc_shadow_regs;

typedef volatile edmacc_shadow_regs  *edmacc_shadow_regs_ovly;
/**************************************************************************\
* Register Overlay Structure for PARAMENTRY
\**************************************************************************/
typedef struct  {
	volatile unsigned int opt;
	volatile unsigned int src;
	volatile unsigned int a_b_cnt;
	volatile unsigned int dst;
	volatile unsigned int src_dst_bidx;
	volatile unsigned int link_bcntrld;
	volatile unsigned int src_dst_cidx;
	volatile unsigned int ccnt;
} edmacc_paramentry_regs;
typedef volatile edmacc_paramentry_regs  *edmacc_paramentry_regs_ovly;
/**************************************************************************\
* Register Overlay Structure
\**************************************************************************/
typedef struct  {
	volatile unsigned int rev;
	volatile unsigned int cccfg;
	volatile unsigned char rsvd0[504];
	volatile unsigned int qchmap[8];
	volatile unsigned char rsvd1[32];
	volatile unsigned int dmaqnum[8];
	volatile unsigned int qdmaqnum;
	volatile unsigned char rsvd2[28];
	volatile unsigned int quetcmap;
	volatile unsigned int quepri;
	volatile unsigned char rsvd3[120];
	volatile unsigned int emr;
	volatile unsigned int emrh;
	volatile unsigned int emcr;
	volatile unsigned int emcrh;
	volatile unsigned int qemr;
	volatile unsigned int qemcr;
	volatile unsigned int ccerr;
	volatile unsigned int ccerrclr;
	volatile unsigned int eeval;
	volatile unsigned char rsvd4[28];
	edmacc_dra_regs dra[4];
	volatile unsigned char rsvd5[32];
	volatile unsigned int qrae[4];
	volatile unsigned char rsvd6[112];
	edmacc_que_evtentry_regs queevtentry[2][16];
	volatile unsigned char rsvd7[384];
	volatile unsigned int qstat[2];
	volatile unsigned char rsvd8[24];
	volatile unsigned int qwmthra;
	volatile unsigned int qwmthrb;
	volatile unsigned char rsvd9[24];
	volatile unsigned int ccstat;
	volatile unsigned char rsvd10[188];
	volatile unsigned int aetctl;
	volatile unsigned int aetstat;
	volatile unsigned int aetcmd;
	volatile unsigned char rsvd11[2292];
	volatile unsigned int er;
	volatile unsigned int erh;
	volatile unsigned int ecr;
	volatile unsigned int ecrh;
	volatile unsigned int esr;
	volatile unsigned int esrh;
	volatile unsigned int cer;
	volatile unsigned int cerh;
	volatile unsigned int eer;
	volatile unsigned int eerh;
	volatile unsigned int eecr;
	volatile unsigned int eecrh;
	volatile unsigned int eesr;
	volatile unsigned int eesrh;
	volatile unsigned int ser;
	volatile unsigned int serh;
	volatile unsigned int secr;
	volatile unsigned int secrh;
	volatile unsigned char rsvd12[8];
	volatile unsigned int ier;
	volatile unsigned int ierh;
	volatile unsigned int iecr;
	volatile unsigned int iecrh;
	volatile unsigned int iesr;
	volatile unsigned int iesrh;
	volatile unsigned int ipr;
	volatile unsigned int iprh;
	volatile unsigned int icr;
	volatile unsigned int icrh;
	volatile unsigned int ieval;
	volatile unsigned char rsvd13[4];
	volatile unsigned int qer;
	volatile unsigned int qeer;
	volatile unsigned int qeecr;
	volatile unsigned int qeesr;
	volatile unsigned int qser;
	volatile unsigned int qsecr;
	volatile unsigned char rsvd14[3944];
	edmacc_shadow_regs shadow[4];
	volatile unsigned char rsvd15[6144];
	edmacc_paramentry_regs paramentry[128];
} edmacc_regs;

#define CCINT0_INTERRUPT     16
#define CCERRINT_INTERRUPT   17
#define TCERRINT0_INTERRUPT   18
#define TCERRINT1_INTERRUPT   19

//#define EINVAL -1
//#define EBUSY  -2

#define DAM (1)
#define SAM (1<<1)
#define SYNCDIM (1<<2)
#define STATIC (1<<3)
#define EDMA_FWID (0x7<<8)
#define TCCMODE (0x1<<11)
#define TCC (0x3f<<12)
#define WIMODE (0x1<<19)
#define TCINTEN (0x1<<20)
#define ITCINTEN (0x1<<21)
#define TCCHEN (0x1<<22)
#define ITCCHEN (0x1<<23)
#define SECURE (0x1<<30)
#define PRIV (0x1<<31)

#define TRWORD (0x7<<2)
#define PAENTRY (0x1ff<<5)
/*if changing the QDMA_TRWORD do appropriate change in davinci_start_dma */
#define QDMA_TRWORD (7 & 0x7)

/*Used by driver*/

#define DAVINCI_EDMA_NUM_DMACH           64
#define DAVINCI_EDMA_NUM_QDMACH           8
#define DAVINCI_EDMA_NUM_PARAMENTRY     128
#define DAVINCI_EDMA_NUM_EVQUE            2
#define DAVINCI_EDMA_CHMAPEXIST           0
#define DAVINCI_EDMA_NUM_REGIONS          4
#define DAVINCI_EDMA_MEMPROTECT           0

#define DAVINCI_NUM_UNUSEDCH             21

#define TCC_ANY    -1

#define DAVINCI_EDMA_PARAM_ANY            -2
#define DAVINCI_DMA_CHANNEL_ANY           -1
#define DAVINCI_DMA_MCBSP_TX              2
#define DAVINCI_DMA_MCBSP_RX              3
#define DAVINCI_DMA_VPSS_HIST             4
#define DAVINCI_DMA_VPSS_H3A              5
#define DAVINCI_DMA_VPSS_PRVU             6
#define DAVINCI_DMA_VPSS_RSZ              7
#define DAVINCI_DMA_IMCOP_IMXINT          8
#define DAVINCI_DMA_IMCOP_VLCDINT         9
#define DAVINCI_DMA_IMCO_PASQINT         10
#define DAVINCI_DMA_IMCOP_DSQINT         11
#define DAVINCI_DMA_SPI_SPIX             16
#define DAVINCI_DMA_SPI_SPIR             17
#define DAVINCI_DMA_UART0_URXEVT0        18
#define DAVINCI_DMA_UART0_UTXEVT0        19
#define DAVINCI_DMA_UART1_URXEVT1        20
#define DAVINCI_DMA_UART1_UTXEVT1        21
#define DAVINCI_DMA_UART2_URXEVT2        22
#define DAVINCI_DMA_UART2_UTXEVT2        23
#define DAVINCI_DMA_MEMSTK_MSEVT         24
#define DAVINCI_DMA_MMCRXEVT             26
#define DAVINCI_DMA_MMCTXEVT             27
#define DAVINCI_DMA_I2C_ICREVT           28
#define DAVINCI_DMA_I2C_ICXEVT           29
#define DAVINCI_DMA_GPIO_GPINT0          32
#define DAVINCI_DMA_GPIO_GPINT1          33
#define DAVINCI_DMA_GPIO_GPINT2          34
#define DAVINCI_DMA_GPIO_GPINT3          35
#define DAVINCI_DMA_GPIO_GPINT4          36
#define DAVINCI_DMA_GPIO_GPINT5          37
#define DAVINCI_DMA_GPIO_GPINT6          38
#define DAVINCI_DMA_GPIO_GPINT7          39
#define DAVINCI_DMA_GPIO_GPBNKINT0       40
#define DAVINCI_DMA_GPIO_GPBNKINT1       41
#define DAVINCI_DMA_GPIO_GPBNKINT2       42
#define DAVINCI_DMA_GPIO_GPBNKINT3       43
#define DAVINCI_DMA_GPIO_GPBNKINT4       44
#define DAVINCI_DMA_TIMER0_TINT0         48
#define DAVINCI_DMA_TIMER1_TINT1         49
#define DAVINCI_DMA_TIMER2_TINT2         50
#define DAVINCI_DMA_TIMER3_TINT3         51
#define DAVINCI_DMA_PWM0                 52
#define DAVINCI_DMA_PWM1                 53
#define DAVINCI_DMA_PWM2                 54
#define DAVINCI_DMA_QDMA0                64
#define DAVINCI_DMA_QDMA1                65
#define DAVINCI_DMA_QDMA2                66
#define DAVINCI_DMA_QDMA3                67
#define DAVINCI_DMA_QDMA4                68
#define DAVINCI_DMA_QDMA5                69
#define DAVINCI_DMA_QDMA6                71
#define DAVINCI_DMA_QDMA7                72

/*ch_status paramater of callback function possible values*/
#define DMA_COMPLETE 1
#define DMA_CC_ERROR 2
#define DMA_TC1_ERROR 3
#define DMA_TC2_ERROR 4

/* ERROR FLAGS */
#define ENOPARAM    -1 /* Error flag for no param are free */
#define EDMAINUSE   -2 /* Error flag for DMA channel in use*/
#define EREQDMA     -3 /* Error flag if request dma failed */

enum  address_mode {
        INCR = 0,
        FIFO = 1
    };

enum fifo_width {
W8BIT  = 0,
W16BIT = 1,
W32BIT = 2,
W64BIT = 3,
W128BIT= 4,
W256BIT=  5
};

enum dma_event_q {
EVENTQ_0 = 0,
EVENTQ_1 = 1,
EVENTQ_DEFAULT = -1
};

enum sync_dimension{
    Async =0 ,
    ABsync =1
};

#define PSP_SOK             (0)
#define PSP_SINPROGRESS     (1)
#define PSP_E_DRIVER_INIT   (-1)
#define PSP_E_NO_MEMORY     (-2)
#define PSP_E_RESOURCES     (-3)
#define PSP_E_INVAL_STATE   (-4)
#define PSP_E_INVAL_PARAM   (-5)
#define PSP_E_NOT_SUPPORTED (-6)

/******************************************************************************
 * davinci_request_dma - request for the Davinci DMA channel
 *
 * dev_id - DMA channel number
 *
 * EX: DAVINCI_DMA_MCBSP_TX - For requesting a DMA MasterChannel with MCBSP_TX
 *     event association
 *
 *     DAVINCI_DMA_ANY - For requesting a DMA Masterchannel which does not has
 *     event association
 *
 *     DAVINCI_DMA_LINK - for requesting a DMA SlaveChannel
 *
 * dev_name   - name of the dma channel in human readable format
 * callback   - channel callback function (valied only if you are requesting
 *              for a DMA MasterChannel)
 * data       - private data for the channel to be requested
 * lch        - contains the device id allocated
 * tcc        - specifies the channel number on which the interrupt is
 *              generated
 *              Valied for QDMA and PARAM channes
 * eventq_no  - Event Queue no to which the channel will be associated with
 *              (valied only if you are requesting for a DMA MasterChannel)
 *              Values : EVENTQ_0/EVENTQ_1 for event queue 0/1.
 *                       EVENTQ_DEFAULT for Default queue
 *
 * Return: zero on success,
 *         -EINVAL - if the requested channel is not supported on the ARM side events
 *         -EBUSY - if the requested channel is already in use
 *          EREQDMA - if failed to request the dma channel
 *
 *****************************************************************************/
int davinci_request_dma(int dev_id,
    const char *dev_name,
			void (*callback) (int lch, unsigned short ch_status,
					  void *data), void *data, int *lch,
			int *tcc, enum dma_event_q
    );

/******************************************************************************
 * davinci_set_dma_src_params - DMA source parameters setup
 *
 * lch         - channel for which the source parameters to be configured
 * src_port    - Source port address
 * addressMode - indicates whether the address mode is FIFO or not
 * fifoWidth   - valied only if addressMode is FIFO, indicates the vidth of
 *                FIFO
 *             0 - 8 bit
 *             1 - 16 bit
 *             2 - 32 bit
 *             3 - 64 bit
 *             4 - 128 bit
 *             5 - 256 bit
 *****************************************************************************/
void davinci_set_dma_src_params(int lch, unsigned long src_port,
    enum address_mode mode, enum fifo_width);

/******************************************************************************
 * davinci_set_dma_dest_params - DMA destination parameters setup
 *
 * lch         - channel or param device for destination parameters to be
 *               configured
 * dest_port   - Destination port address
 * addressMode - indicates whether the address mode is FIFO or not
 * fifoWidth   - valied only if addressMode is FIFO,indicates the vidth of FIFO
 *             0 - 8 bit
 *             1 - 16 bit
 *             2 - 32 bit
 *             3 - 64 bit
 *             4 - 128 bit
 *             5 - 256 bit
 *
 *****************************************************************************/
void davinci_set_dma_dest_params(int lch, unsigned long dest_port,
    enum address_mode mode, enum fifo_width);

/******************************************************************************
 * davinci_set_dma_src_index - DMA source index setup
 *
 * lch     - channel or param device for configuration of source index
 * srcbidx - source B-register index
 * srccidx - source C-register index
 *
 *****************************************************************************/
void davinci_set_dma_src_index (int lch, short srcbidx, short srccidx);

/******************************************************************************
 * davinci_set_dma_dest_index - DMA destination index setup
 *
 * lch      - channel or param device for configuration of destination index
 * destbidx - dest B-register index
 * destcidx - dest C-register index
 *
 *****************************************************************************/
void davinci_set_dma_dest_index (int lch, short destbidx, short destcidx);

/******************************************************************************
 * davinci_set_dma_transfer_params -  DMA transfer parameters setup
 *
 * lch  - channel or param device for configuration of aCount, bCount and
 *        cCount regs.
 * aCnt - aCnt register value to be configured
 * bCnt - bCnt register value to be configured
 * cCnt - cCnt register value to be configured
 *
 *****************************************************************************/
void davinci_set_dma_transfer_params(int lch, unsigned short acnt,
				     unsigned short bcnt, unsigned short ccnt,
				     unsigned short bcntrld,
    enum sync_dimension sync_mode);

/******************************************************************************
 *
 * davinci_set_dma_params -
 * ARGUMENTS:
 *      lch - logical channel number
 *
 *****************************************************************************/
void davinci_set_dma_params(int lch, edmacc_paramentry_regs *temp);

/******************************************************************************
 *
 * davinci_get_dma_params -
 * ARGUMENTS:
 *      lch - logical channel number
 *
 *****************************************************************************/
void davinci_get_dma_params(int lch, edmacc_paramentry_regs *temp);

/******************************************************************************
 * davinci_start_dma -  Starts the dma on the channel passed
 *
 * lch - logical channel number
 *
 * Note:    This API can be used only on DMA MasterChannel
 *
 * Return: zero on success
 *        -EINVAL on failure, i.e if requested for the slave channels
 *
 *****************************************************************************/
int davinci_start_dma (int lch);

/******************************************************************************
 * davinci_stop_dma -  Stops the dma on the channel passed
 *
 * lch - logical channel number
 *
 * Note:    This API can be used on MasterChannel and SlaveChannel
 *****************************************************************************/
void davinci_stop_dma (int lch);

/******************************************************************************
 * davinci_dma_link_lch - Link two Logical channels
 *
 * lch_head  - logical channel number, in which the link field is linked to the
 *             the param pointed to by lch_queue
 *             Can be a MasterChannel or SlaveChannel
 * lch_queue - logical channel number or the param entry number, which is to be
 *             linked to the lch_head
 *             Must be a SlaveChannel
 *
 *                     |---------------|
 *                     v               |
 *      Ex:    ch1--> ch2-->ch3-->ch4--|
 *
 *             ch1 must be a MasterChannel
 *
 *             ch2, ch3, ch4 must be SlaveChannels
 *
 * Note:       After channel linking,the user should not update any PaRam entry
 *             of MasterChannel ( In the above example ch1 )
 *
 *****************************************************************************/
void davinci_dma_link_lch (int lch_head, int lch_queue);

/******************************************************************************
 * davinci_dma_unlink_lch - unlink the two logical channels passed through by
 *                          setting the link field of head to 0xffff.
 *
 * lch_head  - logical channel number, from which the link field is to be
 *             removed
 * lch_queue - logical channel number or the param entry number,which is to be
 *             unlinked from lch_head
 *
 *****************************************************************************/
void davinci_dma_unlink_lch (int lch_head, int lch_queue);

/******************************************************************************
 *
 * DMA channel chain - chains the two logical channels passed through by
 * ARGUMENTS:
 * lch_head - logical channel number, from which the link field is to be removed
 * lch_queue - logical channel number or the param entry number, which is to be
 *             unlinked from lch_head
 *
 *****************************************************************************/
void davinci_dma_chain_lch(int lch_head, int lch_queue);

/******************************************************************************
 *
 * DMA channel unchain - unchain the two logical channels passed through by
 * ARGUMENTS:
 * lch_head - logical channel number, from which the link field is to be removed
 * lch_queue - logical channel number or the param entry number, which is to be
 *             unlinked from lch_head
 *
 *****************************************************************************/
void davinci_dma_unchain_lch(int lch_head, int lch_queue);

/******************************************************************************
 *
 * Free DMA channel - Free the dma channel number passed
 *
 * ARGUMENTS:
 * lch - dma channel number to get free
 *
 *****************************************************************************/
void davinci_free_dma (int lch);

#endif
