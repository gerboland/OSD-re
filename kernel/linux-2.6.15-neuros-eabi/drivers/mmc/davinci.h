/*
 *  linux/drivers/mmc/davinci.h
 *
 *  BRIEF MODULE DESCRIPTION
 *      DAVINCI MMC register and other definitions
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

#ifndef DAVINCI_MMC_H_
#define DAVINCI_MMC_H_

/**************************************************************************\
* Register Overlay Structure
\**************************************************************************/
typedef struct {
	volatile unsigned short mmcctl;
	volatile unsigned char rsvd0[2];
	volatile unsigned short mmcclk;
	volatile unsigned char rsvd1[2];
	volatile unsigned short mmcst0;
	volatile unsigned char rsvd2[2];
	volatile unsigned short mmcst1;
	volatile unsigned char rsvd3[2];
	volatile unsigned short mmcim;
	volatile unsigned char rsvd4[2];
	volatile unsigned short mmctor;
	volatile unsigned char rsvd5[2];
	volatile unsigned short mmctod;
	volatile unsigned char rsvd6[2];
	volatile unsigned short mmcblen;
	volatile unsigned char rsvd7[2];
	volatile unsigned short mmcnblk;
	volatile unsigned char rsvd8[2];
	volatile unsigned short mmcnblc;
	volatile unsigned char rsvd9[2];
	volatile unsigned int mmcdrr;
	volatile unsigned int mmcdxr;
	volatile unsigned int mmccmd;
	volatile unsigned int mmcarghl;
	volatile unsigned int mmcrsp01;
	volatile unsigned int mmcrsp23;
	volatile unsigned int mmcrsp45;
	volatile unsigned int mmcrsp67;
	volatile unsigned short mmcdrsp;
	volatile unsigned char rsvd10[2];
	volatile unsigned short mmcetok;
	volatile unsigned char rsvd11[2];
	volatile unsigned short mmccidx;
	volatile unsigned char rsvd12[2];
	volatile unsigned short mmcckc;
	volatile unsigned char rsvd13[2];
	volatile unsigned short mmctorc;
	volatile unsigned char rsvd14[2];
	volatile unsigned short mmctodc;
	volatile unsigned char rsvd15[2];
	volatile unsigned short mmcblnc;
	volatile unsigned char rsvd16[2];
	volatile unsigned short sdioctl;
	volatile unsigned char rsvd17[2];
	volatile unsigned short sdiost0;
	volatile unsigned char rsvd18[2];
	volatile unsigned short sdioen;
	volatile unsigned char rsvd19[2];
	volatile unsigned short sdiost;
	volatile unsigned char rsvd20[2];
	volatile unsigned short mmcfifoctl;
} mmcsd_regs;

/**************************************************************************\
* Overlay structure typedef definition
\**************************************************************************/
typedef volatile mmcsd_regs *CSL_MmcsdRegsOvly;

/*
 * Command types
 */
#define DAVINCI_MMC_CMDTYPE_BC	0
#define DAVINCI_MMC_CMDTYPE_BCR	1
#define DAVINCI_MMC_CMDTYPE_AC	2
#define DAVINCI_MMC_CMDTYPE_ADTC	3
#define EDMA_MAX_LOGICAL_CHA_ALLOWED 1

typedef struct {
	unsigned char cntChanel;
    /**< Number of logical channel allocated */
	unsigned int chanelNum[EDMA_MAX_LOGICAL_CHA_ALLOWED];
    /**< channel numbers */
} edmaChMmcsd;

struct mmc_davinci_host {

	int initialized;
	int suspended;
	struct mmc_request *req;
	struct mmc_command *cmd;
	struct mmc_data *data;
	struct mmc_host *mmc;
	struct device *dev;
	unsigned char id;	/* 1610 has 2 MMC blocks */
	struct clk *clk;
	u32 base;
	int irq;
	unsigned char bus_mode;

#define DAVINCI_MMC_DATADIR_NONE	0
#define DAVINCI_MMC_DATADIR_READ	1
#define DAVINCI_MMC_DATADIR_WRITE	2
	unsigned char datadir;
	u32 *buffer;
	u32 bytesleft;
	int power_pin;

	int use_dma;
	/*int                 dma_ch; */
	struct completion dma_completion;

	/*   int                           switch_pin;
	   struct work_struct  switch_work;
	   struct timer_list   switch_timer; */

	unsigned char sd_support;

	edmaChMmcsd edmaChDetails;

};
typedef struct {
	unsigned short rwThreshold;
		/**< minimum data requirement in bytes for a read or
		 * write request to be sent								*/
	unsigned short use_dma;
		/**< DMA to be used or not								*/
} mmcsd_configdef;

typedef enum {

	MMCSD_EVENT_EOFCMD = (1 << 2),
	/**< for commands with response, an end of
	 * Command + Response; for commands without response,
	 * an end of Command												*/
	MMCSD_EVENT_READ = (1 << 10),
	/**< data available with controller for reading						*/

	MMCSD_EVENT_WRITE = (1 << 9),
	/**< data required by controller for writing						*/

	MMCSD_EVENT_ERROR_CMDCRC = (1 << 7),
	/**< Error detected in the CRC during commannd
	 * - response phase													*/

	MMCSD_EVENT_ERROR_DATACRC = ((1 << 6) | (1 << 5)),
	/**< Error detected in the CRC during data transfer					*/

	MMCSD_EVENT_ERROR_CMDTIMEOUT = (1 << 4),
	/**< Timeout detected during commannd - response phase				*/

	MMCSD_EVENT_ERROR_DATATIMEOUT = (1 << 3),
	/**< Timeout detected during data transfer							*/

	MMCSD_EVENT_CARD_EXITBUSY = (1 << 1),
	/**< Card has exited busy state										*/

	MMCSD_EVENT_BLOCK_XFERRED = (1 << 0)
	/**< block transfer done											*/
} mmcsdevent;

#define MMCSD_EVENT_TIMEOUT_ERROR  (MMCSD_EVENT_ERROR_DATATIMEOUT | \
			    		MMCSD_EVENT_ERROR_CMDTIMEOUT )
#define MMCSD_EVENT_CRC_ERROR  ( MMCSD_EVENT_ERROR_DATACRC  | MMCSD_EVENT_ERROR_CMDCRC)

#define MMCSD_EVENT_ERROR  ( MMCSD_EVENT_ERROR_DATATIMEOUT | \
				MMCSD_EVENT_ERROR_CMDTIMEOUT | \
				MMCSD_EVENT_ERROR_DATACRC  | \
				MMCSD_EVENT_ERROR_CMDCRC)
#endif
