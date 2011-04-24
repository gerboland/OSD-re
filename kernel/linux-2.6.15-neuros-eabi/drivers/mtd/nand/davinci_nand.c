/*
 * linux/drivers/mtd/nand/davinci_nand.c
 *
 * Flash Driver
 *
 * Copyright (C) 2004 Texas Instruments.
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
 *
 *  Overview:
 *   This is a device driver for the NAND flash device found on the
 *   DaVinci board which utilizes the Samsung k9k2g08 part.
 *
 Modifications:
 ver. 1.0: March 2005, Vinod Mistral
 -
 *
 */



#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/delay.h>

#include "llc_nandIf.h"
#include "llc_nandTypes.h"


/*
 * MTD structure for DaVinici board
 */
static struct mtd_info *davinci_mtd = NULL;

LLC_NandObj                      nandObj;
LLC_NandHandle                   hNand;

#define DAVINCI_CSL_NAND_REG1   	0x01E00000
#define DAVINCI_NAND_IO_ADDR_OFFSET     0x100
#define DAVINCI_NAND_IO_ADDR		\
	(DAVINCI_PERI_ADDR(DAVINCI_CSL_NAND_REG1) + DAVINCI_NAND_IO_ADDR_OFFSET)

#define NAND_READ_START    0x00
#define NAND_READ_END      0x30
#define NAND_STATUS        0x70


#define NAND_Ecc_P1e		(1 << 0)
#define NAND_Ecc_P2e		(1 << 1)
#define NAND_Ecc_P4e		(1 << 2)
#define NAND_Ecc_P8e		(1 << 3)
#define NAND_Ecc_P16e		(1 << 4)
#define NAND_Ecc_P32e		(1 << 5)
#define NAND_Ecc_P64e		(1 << 6)
#define NAND_Ecc_P128e		(1 << 7)
#define NAND_Ecc_P256e		(1 << 8)
#define NAND_Ecc_P512e		(1 << 9)
#define NAND_Ecc_P1024e		(1 << 10)
#define NAND_Ecc_P2048e		(1 << 11)
 
#define NAND_Ecc_P1o		(1 << 16)
#define NAND_Ecc_P2o		(1 << 17)
#define NAND_Ecc_P4o		(1 << 18)
#define NAND_Ecc_P8o		(1 << 19)
#define NAND_Ecc_P16o		(1 << 20)
#define NAND_Ecc_P32o		(1 << 21)
#define NAND_Ecc_P64o		(1 << 22)
#define NAND_Ecc_P128o		(1 << 23)
#define NAND_Ecc_P256o		(1 << 24)
#define NAND_Ecc_P512o		(1 << 25)
#define NAND_Ecc_P1024o		(1 << 26)
#define NAND_Ecc_P2048o		(1 << 27)
 
#define TF(value)	(value ? 1 : 0)
 
#define P2048e(a)	(TF(a & NAND_Ecc_P2048e)	<< 0 )
#define P2048o(a)	(TF(a & NAND_Ecc_P2048o)	<< 1 )
#define P1e(a)		(TF(a & NAND_Ecc_P1e)		<< 2 )
#define P1o(a)		(TF(a & NAND_Ecc_P1o)		<< 3 )
#define P2e(a)		(TF(a & NAND_Ecc_P2e)		<< 4 )
#define P2o(a)		(TF(a & NAND_Ecc_P2o)		<< 5 )
#define P4e(a)		(TF(a & NAND_Ecc_P4e)		<< 6 )
#define P4o(a)		(TF(a & NAND_Ecc_P4o)		<< 7 )

#define P8e(a)		(TF(a & NAND_Ecc_P8e)		<< 0 )
#define P8o(a)		(TF(a & NAND_Ecc_P8o)		<< 1 )
#define P16e(a)		(TF(a & NAND_Ecc_P16e)		<< 2 )
#define P16o(a)		(TF(a & NAND_Ecc_P16o)		<< 3 )
#define P32e(a)		(TF(a & NAND_Ecc_P32e)		<< 4 )
#define P32o(a)		(TF(a & NAND_Ecc_P32o)		<< 5 )
#define P64e(a)		(TF(a & NAND_Ecc_P64e)		<< 6 )
#define P64o(a)		(TF(a & NAND_Ecc_P64o)		<< 7 )

#define P128e(a)	(TF(a & NAND_Ecc_P128e)		<< 0 )
#define P128o(a)	(TF(a & NAND_Ecc_P128o)		<< 1 )
#define P256e(a)	(TF(a & NAND_Ecc_P256e)		<< 2 )
#define P256o(a)	(TF(a & NAND_Ecc_P256o)		<< 3 )
#define P512e(a)	(TF(a & NAND_Ecc_P512e)		<< 4 )
#define P512o(a)	(TF(a & NAND_Ecc_P512o)		<< 5 )
#define P1024e(a)	(TF(a & NAND_Ecc_P1024e)	<< 6 )
#define P1024o(a)	(TF(a & NAND_Ecc_P1024o)	<< 7 )

#define P8e_s(a)	(TF(a & NAND_Ecc_P8e)		<< 0 )
#define P8o_s(a)	(TF(a & NAND_Ecc_P8o)		<< 1 )
#define P16e_s(a)	(TF(a & NAND_Ecc_P16e)		<< 2 )
#define P16o_s(a)	(TF(a & NAND_Ecc_P16o)		<< 3 )
#define P1e_s(a)	(TF(a & NAND_Ecc_P1e)		<< 4 )
#define P1o_s(a)	(TF(a & NAND_Ecc_P1o)		<< 5 )
#define P2e_s(a)	(TF(a & NAND_Ecc_P2e)		<< 6 )
#define P2o_s(a)	(TF(a & NAND_Ecc_P2o)		<< 7 )
 
#define P4e_s(a)	(TF(a & NAND_Ecc_P4e)		<< 0 )
#define P4o_s(a)	(TF(a & NAND_Ecc_P4o)		<< 1 )



/*
 * Define partitions for flash device
 */
static struct mtd_partition partition_info[] = {
	/* BOOTLOADER + KERNEL */
	{ name: "Flash partition 0",
	  offset: 0,
	  size:   1 * SZ_1M},

	{ name: "Flash partition 1",
	  offset: 1 * SZ_1M,
	  size:   3 * SZ_1M},

	{ name: "Flash partition 2",
	  offset: 4 * SZ_1M,	/*Block Number 4 * 64 = 256 */	
	  size:   4 * SZ_1M},

	/* INIT CRAMFS           */
	{ name: "Flash partition 3",
	  offset: 8 * SZ_1M,       /* Block Number 8 * 64 = 512 */
	  size:   2 * SZ_1M},

	/* ROOT CRAMFS           */	
	{ name: "Flash partition 4",
	  offset: 10 * SZ_1M,     /* Block Number 10 * 64 = 640 */
	  size:   4  * SZ_1M},
	
	/* USR CRAMFS            */
	{ name: "Flash partition 5",
	  offset: 14 * SZ_1M,     /* Block number 14 * 64 = 896 */
	  size:   10 * SZ_1M},
	
	/* EXT2 FILE SYSTEM     */
	{ name: "Flash partition 6",
	  offset: 24 * SZ_1M,     /* Block number 24 * 64 = 1536 */
	  size:   24 * SZ_1M},

	/*  JFFS FILE SYSTEM    */
	{ name: "Flash partition 7",
	  offset: 48 * SZ_1M,    /* Block number 48 * 64 = 3072 */
	  size:   16 * SZ_1M},

};


#define NUM_PARTITIONS           8

/*
 *      hardware specific access to control-lines
*/
static void davinci_hwcontrol(struct mtd_info *mtd, int cmd){
}


static void davinci_nand_write_byte(struct mtd_info *mtd, u8 byte)
{
    struct nand_chip *this = mtd->priv;
    LLC_NandHandle hnand;
    
    hnand = (LLC_NandHandle)this->priv;
    LLC_nandWriteByte(hnand, le16_to_cpu((u16) byte));
}

static u8 davinci_nand_read_byte(struct mtd_info *mtd)
{
    struct nand_chip *this = mtd->priv;
    LLC_NandHandle hnand;

    hnand = (LLC_NandHandle)this->priv;
    return (u_char) cpu_to_le16(LLC_nandReadByte(hnand));
}

static int nand_write_command(struct mtd_info *mtd, u8 cmd, u32 addr, int addr_valid)
{
    register struct nand_chip *this = mtd->priv;
    LLC_NandHandle hnand;
    
    hnand = (LLC_NandHandle)this->priv;
    if (addr_valid) {
        LLC_nandSendCommand(hnand, cmd);
        LLC_nandSetAddr(hnand, addr);
    } else {
        LLC_nandSendCommand(hnand, cmd);
        LLC_nandSetAddr(hnand, 0x00);
    }

    return 0;
}

/**
 * nand_command - [DEFAULT] Send command to NAND device
 * @mtd:	MTD device structure
 * @command:	the command to be sent
 * @column:	the column address for this command, -1 if none
 * @page_addr:	the page address for this command, -1 if none
 *
 * Send command to NAND device. This function is used for small page
 * devices (256/512 Bytes per page)
 */
static void davinci_nand_command (struct mtd_info *mtd, unsigned command, int column, int page_addr)
{
	register struct nand_chip *this = mtd->priv;
	LLC_NandHandle hnand;
    
	hnand = (LLC_NandHandle)this->priv;
         /* Emulate NAND_CMD_READOOB */
         if (command == NAND_CMD_READOOB) {
                 column += mtd->oobblock;
                 command = NAND_CMD_READ0;
         }


        if ( command == NAND_CMD_READID) {
            nand_write_command(mtd, LLC_NAND_READ_DEVICEID, 0, 0);
            return;
        }


         /* Begin command latch cycle */
         this->hwcontrol(mtd, NAND_CTL_SETCLE);
         /* Write out the command to the device. */
          LLC_nandSendCommand(hnand, command);
         /* End command latch cycle */
         this->hwcontrol(mtd, NAND_CTL_CLRCLE);

         if (column != -1 || page_addr != -1) {
                 this->hwcontrol(mtd, NAND_CTL_SETALE);

                 /* Serially input address */
                 if (column != -1) {
                         /* Adjust columns for 16 bit buswidth */
                         if (this->options & NAND_BUSWIDTH_16)
                                 column >>= 1;
                         LLC_nandSetAddr(hnand, (unsigned char) (column & 0xff));
                         LLC_nandSetAddr(hnand, (unsigned char) ( column >> 8));
                 }
                 if (page_addr != -1) {
                         LLC_nandSetAddr(hnand, (unsigned char) (page_addr & 0xff));
                         LLC_nandSetAddr(hnand,(unsigned char) ((page_addr >> 8) & 0xff));
                         /* One more address cycle for devices > 128MiB */
                         if (this->chipsize > (128 << 20))
                             LLC_nandSetAddr(hnand,(unsigned char) ((page_addr >> 16) & 0xff));
                 }
                 /* Latch in address */
                 this->hwcontrol(mtd, NAND_CTL_CLRALE);
         }
         /*
          * program and erase have their own busy handlers
          * status and sequential in needs no delay
         */
         switch (command) {

         case NAND_CMD_CACHEDPROG:
         case NAND_CMD_PAGEPROG:
         case NAND_CMD_ERASE1:
         case NAND_CMD_ERASE2:
         case NAND_CMD_SEQIN:
         case NAND_CMD_STATUS:
                 return;


         case NAND_CMD_RESET:
                 if (this->dev_ready)
                         break;
                 udelay(this->chip_delay);
                 this->hwcontrol(mtd, NAND_CTL_SETCLE);
                 LLC_nandSendCommand(hnand, NAND_CMD_STATUS);
                 this->hwcontrol(mtd, NAND_CTL_CLRCLE);
                 while ( !(this->read_byte(mtd) & 0x40));
                 return;

         case NAND_CMD_READ0:
                 /* Begin command latch cycle */
                 this->hwcontrol(mtd, NAND_CTL_SETCLE);
                 /* Write out the start read command */
                 LLC_nandSendCommand(hnand, NAND_CMD_READSTART);
                 /* End command latch cycle */
                 this->hwcontrol(mtd, NAND_CTL_CLRCLE);
                 /* Fall through into ready check */

         /* This applies to read commands */
         default:
                 /*
                  * If we don't have access to the busy pin, we apply the given
                  * command delay
                 */
                 if (!this->dev_ready) {
                         udelay (this->chip_delay);
                         return;
                 }
         }

         /* Apply this short delay always to ensure that we do wait tWB in
          * any case on any machine. */
         ndelay (100);
         /* wait until command is processed */

}			

static void davinci_nand_enable_hwecc(struct mtd_info *mtd, int mode)
{
        /* Enable Hw ecc */
}


static Uint32 nandReadEcc(struct mtd_info *mtd,u32  Reg)
{
       u32      l = 0;
       struct nand_chip *this = mtd->priv;
       LLC_NandHandle hnand;
    
       hnand = (LLC_NandHandle)this->priv;
       if (Reg == 1)
           l = LLC_nandReadEcc1(hnand);
       else if (Reg == 2)
           l = LLC_nandReadEcc2(hnand);
       else if (Reg == 3)
           l = LLC_nandReadEcc3(hnand);
       else if (Reg == 4)
           l = LLC_nandReadEcc4(hnand);

       return l;
}

static int davinci_nand_calculate_ecc(struct mtd_info *mtd, const u_char *dat, u_char *ecc_code)
{
 	unsigned int      l;
 	int               reg;
 	int               n;
 	struct nand_chip *this = mtd->priv;
 
 	if (this->eccmode == NAND_ECC_HW12_2048)
 		n = 4;
 	else
 		n = 1;

 	reg = 1;
 	while (n--) {
 		l = nandReadEcc(mtd, reg);
 		*ecc_code++ = l;          // P128e, ..., P1e
 		*ecc_code++ = l >> 16;    // P128o, ..., P1o
 		// P2048o, P1024o, P512o, P256o, P2048e, P1024e, P512e, P256e
 		*ecc_code++ = ((l >> 8) & 0x0f) | ((l >> 20) & 0xf0);
 		reg++;
 	}
 	return 0;
}

static void gen_true_ecc(u8 *ecc_buf)
{
 	u32 tmp = ecc_buf[0] | (ecc_buf[1] << 16) | ((ecc_buf[2] & 0xF0) << 20) | ((ecc_buf[2] & 0x0F) << 8);
 
 	ecc_buf[0] = ~(P64o(tmp) | P64e(tmp) | P32o(tmp) | P32e(tmp) | P16o(tmp) | P16e(tmp) | P8o(tmp) | P8e(tmp) );
 	ecc_buf[1] = ~(P1024o(tmp) | P1024e(tmp) | P512o(tmp) | P512e(tmp) | P256o(tmp) | P256e(tmp) | P128o(tmp) | P128e(tmp));
 	ecc_buf[2] = ~( P4o(tmp) | P4e(tmp) | P2o(tmp) | P2e(tmp) | P1o(tmp) | P1e(tmp) | P2048o(tmp) | P2048e(tmp));
}



static int davinci_nand_compare_ecc(u8 *     ecc_data1,   /* read from NAND memory */
 				    u8 *     ecc_data2,   /* read from register */
 				    u8 *     page_data)
{
 	u32    i;
 	u8     tmp0_bit[8], tmp1_bit[8], tmp2_bit[8];
 	u8     comp0_bit[8], comp1_bit[8], comp2_bit[8];
 	u8     ecc_bit[24];
 	u8     ecc_sum = 0;
 	u8     find_bit = 0;
 	u32    find_byte = 0;
 	int    isEccFF;

 	isEccFF = ((*(u32 *)ecc_data1 & 0xFFFFFF) == 0xFFFFFF);
 
 	gen_true_ecc(ecc_data1);
 	gen_true_ecc(ecc_data2);

 	for (i = 0; i <= 2; i++) {
 		*(ecc_data1 + i) = ~(*(ecc_data1 + i));
 		*(ecc_data2 + i) = ~(*(ecc_data2 + i));
 	}
 
 	for (i = 0; i < 8; i++) {
 		tmp0_bit[i]      = *ecc_data1 % 2;
 		*ecc_data1       = *ecc_data1 / 2;
 	}
   
 	for (i = 0; i < 8; i++) {
 		tmp1_bit[i]      = *(ecc_data1 + 1) % 2;
 		*(ecc_data1 + 1) = *(ecc_data1 + 1) / 2;
        }

 	for (i = 0; i < 8; i++) {
 		tmp2_bit[i]      = *(ecc_data1 + 2) % 2;
 		*(ecc_data1 + 2) = *(ecc_data1 + 2) / 2;
 	}
 
 	for (i = 0; i < 8; i++) {
 		comp0_bit[i]     = *ecc_data2 % 2;
 		*ecc_data2       = *ecc_data2 / 2;
 	}

 	for (i = 0; i < 8; i++) {
 		comp1_bit[i]     = *(ecc_data2 + 1) % 2;
 		*(ecc_data2 + 1) = *(ecc_data2 + 1) / 2;
 	}

 	for (i = 0; i < 8; i++) {
 		comp2_bit[i]     = *(ecc_data2 + 2) % 2;
 		*(ecc_data2 + 2) = *(ecc_data2 + 2) / 2;
 	}

 	for (i = 0; i< 6; i++ )
 		ecc_bit[i] = tmp2_bit[i + 2] ^ comp2_bit[i + 2];
 
 	for (i = 0; i < 8; i++)
 		ecc_bit[i + 6] = tmp0_bit[i] ^ comp0_bit[i];

 	for (i = 0; i < 8; i++)
 		ecc_bit[i + 14] = tmp1_bit[i] ^ comp1_bit[i];

 	ecc_bit[22] = tmp2_bit[0] ^ comp2_bit[0];
 	ecc_bit[23] = tmp2_bit[1] ^ comp2_bit[1];

 	for (i = 0; i < 24; i++)
 		ecc_sum += ecc_bit[i];

 	switch (ecc_sum) {
 	case 0:
 		/* Not reached because this function is not called if
 		   ECC values are equal */
 		return 0;
 
 	case 1:
 		/* Uncorrectable error */
 		DEBUG (MTD_DEBUG_LEVEL0, "ECC UNCORRECTED_ERROR 1\n");
 		return -1;

 	case 12:
 		/* Correctable error */
 		find_byte = (ecc_bit[23] << 8) + 
 			    (ecc_bit[21] << 7) + 
 			    (ecc_bit[19] << 6) +
 			    (ecc_bit[17] << 5) +
 			    (ecc_bit[15] << 4) +
 			    (ecc_bit[13] << 3) +
 			    (ecc_bit[11] << 2) +
 			    (ecc_bit[9]  << 1) +
 			    ecc_bit[7];

 		find_bit = (ecc_bit[5] << 2) + (ecc_bit[3] << 1) + ecc_bit[1];
 
 		DEBUG (MTD_DEBUG_LEVEL0, "Correcting single bit ECC error at offset: %d, bit: %d\n", find_byte, find_bit);
 
 		page_data[find_byte] ^= (1 << find_bit);
 
 		return 0;
 	default:
 		if (isEccFF) {
 			if (ecc_data2[0] == 0 && ecc_data2[1] == 0 && ecc_data2[2] == 0)
 				return 0;
 		} 
 		DEBUG (MTD_DEBUG_LEVEL0, "UNCORRECTED_ERROR default\n");
 		return -1;
 	}
}

static int davinci_nand_correct_data(struct mtd_info *mtd, u_char *dat, u_char *read_ecc, u_char *calc_ecc)
{
 	struct nand_chip *this;
 	int block_count = 0, i, r;
 
 	this = mtd->priv;
 	if (this->eccmode == NAND_ECC_HW12_2048)
 		block_count = 4;
 	else
 		block_count = 1;
 	for (i = 0; i < block_count; i++) {
 		if (memcmp(read_ecc, calc_ecc, 3) != 0) {
 			r = davinci_nand_compare_ecc(read_ecc, calc_ecc, dat);
 			if (r < 0)
 				return r;
 		}
 		read_ecc += 3;
 		calc_ecc += 3;
 		dat += 512;
 	}
 	return 0;
}


/*
 * Main initialization routine
 */
int __init davinci_init (void)
{
        struct nand_chip  *this;
        u32                nand_rev_code;

        /* Allocate memory for MTD device structure and private data */
        davinci_mtd = kmalloc (sizeof(struct mtd_info) + sizeof (struct nand_chip),
                                GFP_KERNEL);
        if (!davinci_mtd) {
                printk ("Unable to allocate davinci NAND MTD device structure.\n");
                return -ENOMEM;
        }

        /* Get pointer to private data */
        this = (struct nand_chip *) (&davinci_mtd[1]);

        /* Initialize structures */
        memset((char *) davinci_mtd, 0, sizeof(struct mtd_info));
        memset((char *) this, 0, sizeof(struct nand_chip));
   
        /* Link the private data with the MTD structure */
        davinci_mtd->priv = this;

        LLC_nandInit(&nandObj,CSL_NAND);

	this->priv       = (void*) &nandObj;
        hNand = (LLC_NandHandle) &nandObj;

	nand_rev_code = LLC_nandReadRevision(hNand);

        printk("DaVinci NAND Controller rev. %d.%d\n", 
                     (nand_rev_code >>8) & 0xff, nand_rev_code & 0xff);

	this->IO_ADDR_R = (void  __iomem*)DAVINCI_NAND_IO_ADDR;
	this->IO_ADDR_W = (void  __iomem*)DAVINCI_NAND_IO_ADDR;

        this->chip_delay  = 25;
        this->cmdfunc     = davinci_nand_command;
        this->read_byte  = davinci_nand_read_byte;
        this->write_byte = davinci_nand_write_byte;
        this->options |= NAND_BUSWIDTH_16; 
        this->eccmode  = NAND_ECC_SOFT; // NAND_ECC_HW12_2048;
        /* Set address of hardware control function */

        this->calculate_ecc = davinci_nand_calculate_ecc;
 	this->correct_data = davinci_nand_correct_data;
 	this->enable_hwecc = davinci_nand_enable_hwecc;

        this->hwcontrol = davinci_hwcontrol;

        /* Scan to find existence of the device */
        if (nand_scan (davinci_mtd, 1)) {
                kfree (davinci_mtd);
                return -ENXIO;
        }
        /* Register the partitions */
        add_mtd_partitions(davinci_mtd, partition_info, NUM_PARTITIONS);

        /* Return happy */
        return 0;
}
module_init(davinci_init);

/*
 * Clean up routine
 */
#ifdef MODULE
static void __exit davinci_cleanup (void)
{
        /* Release resources, unregister device */
        nand_release (davinci_mtd);

        /* Free the MTD device structure */
        kfree (davinci_mtd);
}
module_exit(davinci_cleanup);
#endif

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Texas Instruments");
MODULE_DESCRIPTION("Board-specific glue layer for NAND flash on davinci board");
