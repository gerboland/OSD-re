/*

  dm9ks.c: Version 2.03 2005/10/17 
  
        A Davicom DM9000A/DM9010 ISA NIC fast Ethernet driver for Linux.

	This program is free software; you can redistribute it and/or
	modify it under the terms of the GNU General Public License
	as published by the Free Software Foundation; either version 2
	of the License, or (at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.


  (C)Copyright 1997-2005 DAVICOM Semiconductor,Inc. All Rights Reserved.

	
V1.00	10/13/2004	Add new function Early transmit & IP/TCP/UDP Checksum
			offload enable & flow control is default
V1.1	12/29/2004	Add Two packet mode & modify RX function
V1.2	01/14/2005	Add Early transmit mode 
V1.3	03/02/2005	Support kernel 2.6
v1.33   06/08/2005	#define DM9KS_MDRAL		0xf4
			#define DM9KS_MDRAH		0xf5
			
V2.00 Spenser - 01/10/2005
			- Modification for PXA270 MAINSTONE.
			- Modified dmfe_tx_done().
			- Add dmfe_timeout().
V2.01	10/07/2005	Modified dmfe_timer()
			Dected network speed 10/100M
V2.02	10/12/2005	Use link change to chage db->Speed
			dmfe_open() wait for Link OK  
V2.03	11/22/2005	Power-off and Power-on PHY in dmfe_init()
			support IOL

	06/17/2006	Butchered for NTR3 OSD by neuros

V2.04	12/13/2005	-delay 1.6s between power-on and power-off in 
			 dmfe_init_dm9000()
			-set LED mode 1 in dmfe_init_dm9000()
			-add data bus driving capability in dmfe_init_dm9000()
			 (optional)
	10/3/2006	-Add DM8606 read/write function by MDC and MDIO
V2.06	01/03/2007	-CONT_RX_PKT_CNT=0xFFFF
			-modify dmfe_tx_done function
			-check RX FIFO pointer
			-if using physical address, re-define I/O function
			-add db->cont_rx_pkt_cnt=0 at the front of dmfe_packet_receive()

        03/15/2007      integrate the 2.06 version into neuros bsp
	                Patch partially from Chris <chris@edesix.com>
	                    -Removed ioctl handler which just said OK!
			    -Removed a timer process which no need now.
*/

#if defined(MODVERSIONS)
#include <linux/modversions.h>
#endif

#include <linux/config.h>
#include <linux/init.h>				
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/ioport.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <linux/version.h>
#include <asm/dma.h>
#include <linux/spinlock.h>
#include <linux/crc32.h>
#include <linux/platform_device.h>
#include <linux/mii.h>

#ifdef CONFIG_ARCH_NTOSD_DM320
#include <asm/io.h>
#include <asm/hardware.h>
#include <asm/irq.h>
#include <asm/arch/gio.h>
#include <asm/arch/irqs.h>
#endif

/* Board/System/Debug information/definition ---------------- */

#define DM9KS_ID		0x90000A46
#define DM9010_ID		0x90100A46
/*-------register name-----------------------*/
#define DM9KS_NCR		0x00	/* Network control Reg.*/
#define DM9KS_NSR		0x01	/* Network Status Reg.*/
#define DM9KS_TCR		0x02	/* TX control Reg.*/
#define DM9KS_RXCR		0x05	/* RX control Reg.*/
#define DM9KS_BPTR		0x08
#define DM9KS_EPCR		0x0b
#define DM9KS_EPAR		0x0c
#define DM9KS_EPDRL		0x0d
#define DM9KS_EPDRH		0x0e
#define DM9KS_GPR		0x1f	/* General purpose register */
#define DM9KS_TCR2		0x2d
#define DM9KS_SMCR		0x2f 	/* Special Mode Control Reg.*/
#define DM9KS_ETXCSR		0x30	/* Early Transmit control/status Reg.*/
#define	DM9KS_TCCR		0x31	/* Checksum cntrol Reg. */
#define DM9KS_RCSR		0x32	/* Receive Checksum status Reg.*/
#define DM9KS_MRCMDX		0xf0
#define DM9KS_MRCMD		0xf2
#define DM9KS_MDRAL		0xf4
#define DM9KS_MDRAH		0xf5
#define DM9KS_MWCMD		0xf8
#define DM9KS_TXPLL		0xfc
#define DM9KS_TXPLH		0xfd
#define DM9KS_ISR		0xfe
#define DM9KS_IMR		0xff
/*---------------------------------------------*/
#define DM9KS_REG05		0x30	/* SKIP_CRC/SKIP_LONG */ 
#define DM9KS_REGFF		0xA3	/* IMR */
#define DM9KS_DISINTR		0x80

#define DM9KS_PHY		0x40	/* PHY address 0x01 */
#define DM9KS_PKT_RDY		0x01	/* Packet ready to receive */

#define DM9KS_VID_L		0x28
#define DM9KS_VID_H		0x29
#define DM9KS_PID_L		0x2A
#define DM9KS_PID_H		0x2B

#define DM9KS_RX_INTR		0x01
#define DM9KS_TX_INTR		0x02
#define DM9KS_LINK_INTR		0x20

#define DM9KS_DWORD_MODE	1
#define DM9KS_BYTE_MODE		2
#define DM9KS_WORD_MODE		0

#define TRUE			1
#define FALSE			0

#define DATAOFST1               2       /* OSD:use A0 as CMD line */
#define DATAOFST2               0x10000 /* OSD:use A15 as CMD line */

/* Number of continuous Rx packets */
#define CONT_RX_PKT_CNT	        0xFFFF 

#define CARDNAME "dm9ks"

#define	MEM_MAPPED_IO	1
#ifdef	MEM_MAPPED_IO
#define	GETB(a)		*((volatile unsigned char *) (a))
#define	GETW(a)		*((volatile unsigned short *) (a))
#define	GETL(a)		*((volatile unsigned long *) (a))
#define	PUTB(d,a)	*((volatile unsigned char *) (a)) = d
#define	PUTW(d,a)	*((volatile unsigned short *) (a)) = d
#define	PUTL(d,a)	*((volatile unsigned long *) (a)) = d
#else
#define	GETB(a)		inb(a)
#define	GETW(a)		inw(a)
#define	GETL(a)		inl(a)
#define	PUTB(d,a)	outb(d,a)
#define	PUTW(d,a)	outw(d,a)
#define	PUTL(d,a)	outl(d,a)
#endif

#define CONFIG_DM9KS_DMA 1
//#define DM9KS_DEBUG 1

#if defined(DM9KS_DEBUG)
#define DMFE_DBUG(dbug_now, msg, vaule)\
if (dmfe_debug||dbug_now) printk(KERN_ERR "dmfe: %s %x\n", msg, vaule)
#else
#define DMFE_DBUG(dbug_now, msg, vaule)\
if (dbug_now) printk(KERN_ERR "dmfe: %s %x\n", msg, vaule)
#endif

#ifndef CONFIG_ARCH_MAINSTONE
#pragma pack(push, 1)
#endif

typedef struct _RX_DESC
{
	u8 rxbyte;
	u8 status;
	u16 length;
}RX_DESC;

typedef union{
	u8 buf[4];
	RX_DESC desc;
} rx_t;
#ifndef CONFIG_ARCH_MAINSTONE
#pragma pack(pop)
#endif

enum DM9KS_PHY_mode {
	DM9KS_10MHD   = 0, 
	DM9KS_100MHD  = 1, 
	DM9KS_10MFD   = 4,
	DM9KS_100MFD  = 5, 
	DM9KS_AUTO    = 8, 
};

/* Structure/enum declaration ------------------------------- */
typedef struct board_info {
 
	u32 reset_counter;		/* counter: RESET */ 
	u32 reset_tx_timeout;		/* RESET caused by TX Timeout */ 

	u32 io_addr;			/* Register I/O base address */
	u32 io_data;			/* Data I/O address */
	int tx_pkt_cnt;

	u8 op_mode;			/* PHY operation mode */
	u8 io_mode;			/* 0:word, 2:byte */
	u8 device_wait_reset;		/* device state */
	u8 Speed;			/* current speed */

	int cont_rx_pkt_cnt;/* current number of continuos rx packets  */
	struct timer_list timer;
	struct net_device_stats stats;
	unsigned char srom[128];
	spinlock_t lock;
	struct mii_if_info mii;

} board_info_t;

/* Global variable declaration ----------------------------- */
/* static int dmfe_debug = 0; */
static struct net_device * dmfe_dev = NULL;

/* For module input parameter */
static int mode       = DM9KS_AUTO;
static int media_mode = DM9KS_AUTO;
static int dma_mode   = 0; /* use dma or not */

/* function declaration ------------------------------------- */
int dm9k_probe(struct net_device *, unsigned long);
static int dmfe_open(struct net_device *);
static int dmfe_start_xmit(struct sk_buff *, struct net_device *);
static void dmfe_tx_done(unsigned long);
static void dmfe_packet_receive(struct net_device *);
static int dmfe_stop(struct net_device *);
static struct net_device_stats * dmfe_get_stats(struct net_device *); 
static irqreturn_t dmfe_interrupt(int , void *, struct pt_regs *);
static void dmfe_init_dm9000(struct net_device *);
static unsigned long cal_CRC(unsigned char *, unsigned int, u8);
static u8 ior(board_info_t *, int);
static void iow(board_info_t *, int, u8);
static u16 phy_read(board_info_t *, int);
static void phy_write(board_info_t *, int, u16);
#ifdef CURRENTLY_UNUSED
static u16 read_srom_word(board_info_t *, int);
#endif
static void dm9000_hash_table(struct net_device *);
static void dmfe_timeout(struct net_device *);
static void dmfe_reset(struct net_device *);


#if defined(CHECKSUM)
static u8 check_rx_ready(u8);
#endif

static int getdm9kid(unsigned long addr, u32 offset)
{
	u32 id_val;

        /* Search for DM9000 serial NIC */
        PUTB(DM9KS_VID_L, addr);
	id_val = GETB(addr + offset);
	PUTB(DM9KS_VID_H, addr);
	id_val |= GETB(addr + offset) << 8;
	PUTB(DM9KS_PID_L, addr);
	id_val |= GETB(addr + offset) << 16;
	PUTB(DM9KS_PID_H, addr);
	id_val |= GETB(addr + offset) << 24;

	if (id_val != DM9KS_ID && id_val != DM9010_ID)
	   return -1; 

	//printk("<DM9KS> I/O: %lx, offset: %x,  VID: %x \n",addr, offset, id_val);
	return 0;
}

/*
 *   Read a word from phyxcer
 */
static int
dm9000_phy_read(struct net_device *dev, int phy_reg_unused, int reg)
{
	board_info_t *db = (board_info_t *) dev->priv;
	unsigned long flags;
	unsigned int reg_save;
	int ret;

	spin_lock_irqsave(&db->lock,flags);

	/* Save previous register address */
	reg_save = readb(db->io_addr);

	/* Fill the phyxcer register into REG_0C */
	iow(db, DM9KS_EPAR, DM9KS_PHY | reg);

	iow(db, DM9KS_EPCR, 0xc);	/* Issue phyxcer read command */
	udelay(100);		/* Wait read complete */
	iow(db, DM9KS_EPCR, 0x0);	/* Clear phyxcer read command */

	/* The read data keeps on REG_0D & REG_0E */
	ret = (ior(db, DM9KS_EPDRH) << 8) | ior(db, DM9KS_EPDRL);

	/* restore the previous address */
	writeb(reg_save, db->io_addr);

	spin_unlock_irqrestore(&db->lock,flags);

	return ret;
}

/*
 *   Write a word to phyxcer
 */
static void
dm9000_phy_write(struct net_device *dev, int phyaddr_unused, int reg, int value)
{
	board_info_t *db = (board_info_t *) dev->priv;
	unsigned long flags;
	unsigned long reg_save;

	spin_lock_irqsave(&db->lock,flags);

	/* Save previous register address */
	reg_save = readb(db->io_addr);

	/* Fill the phyxcer register into REG_0C */
	iow(db, DM9KS_EPAR, DM9KS_PHY | reg);

	/* Fill the written data into REG_0D & REG_0E */
	iow(db, DM9KS_EPDRL, (value & 0xff));
	iow(db, DM9KS_EPDRH, ((value >> 8) & 0xff));

	iow(db, DM9KS_EPCR, 0xa);	/* Issue phyxcer write command */
	udelay(500);		/* Wait write complete */
	iow(db, DM9KS_EPCR, 0x0);	/* Clear phyxcer write command */

	/* restore the previous address */
	writeb(reg_save, db->io_addr);

	spin_unlock_irqrestore(&db->lock,flags);
}

static int dmfe_do_ioctl(struct net_device *netdev, struct ifreq *ifr, int cmd)
{
	struct board_info *db = netdev_priv(netdev);
	return generic_mii_ioctl(&db->mii, if_mii(ifr), cmd, NULL);
}

/* DM9KS network baord routine ---------------------------- */
int __init dm9k_probe(struct net_device *dev, unsigned long addr)
{
	struct board_info *db;    /* Point a board information structure */
	u16 i, j;
	int retval;
	u32 offset;

	DMFE_DBUG(0, "dmfe_probe(): addr=",(unsigned)addr);

	if(getdm9kid(addr, DATAOFST2) == 0)
	  {
	    printk("DM9000 Ethernet chip uses A15\n");
	    offset = DATAOFST2;
	    dma_mode = 0;
	  }
	else if (getdm9kid(addr, DATAOFST1) == 0)
	  {
	    printk("DM9000 Ethernet chip uses A0\n");
	    offset = DATAOFST1;
	    dma_mode = 0;
	  }
	else 
	  {
	    /* Dm9k chip not found */
	    printk("dmfe_probe(): NO DM9000 founded\n");
	    return -ENODEV;
	  }

	/* Allocated board information structure */
	memset(dev->priv, 0, sizeof(struct board_info));
	db = (board_info_t *)dev->priv;
	dmfe_dev    = dev;
	db->io_addr  = addr;
	db->io_data = addr + offset;
	db->mii.phy_id_mask  = 0x1f;
	db->mii.reg_num_mask = 0x1f;
	db->mii.force_media  = 0;
	db->mii.full_duplex  = 0;
	db->mii.dev	     = dev;
	db->mii.mdio_read    = dm9000_phy_read;
	db->mii.mdio_write   = dm9000_phy_write;
	/* driver system function */
				
	dev->base_addr 		= addr;
	dev->irq 		= IRQ_GIO6;
	dev->open 		= &dmfe_open;
	dev->hard_start_xmit 	= &dmfe_start_xmit;
	dev->watchdog_timeo	= 5*HZ;	
	dev->tx_timeout		= dmfe_timeout;
	dev->stop 		= &dmfe_stop;
	dev->get_stats 		= &dmfe_get_stats;
	dev->set_multicast_list = &dm9000_hash_table;
	dev->do_ioctl = dmfe_do_ioctl;

#if defined(CHECKSUM)
	dev->features = dev->features | NETIF_F_NO_CSUM;
#endif
#if 0
	/* Read SROM content */
	for (i=0; i<64; i++)
		((u16 *)db->srom)[i] = read_srom_word(db, i);
#else 
	for(i=0,j=0x10; i<6; i++,j++)
	  {
	        db->srom[i] = ior(db, j);
	  }
#endif

	/* Set Node Address */
	for (i=0; i<6; i++)
		dev->dev_addr[i] = db->srom[i];

	retval = register_netdev(dev);
	if (retval == 0) {
		/* now, print out the card info, in a short format.. */
		printk("%s: at %#lx IRQ %d\n",
			dev->name, dev->base_addr, dev->irq);

		if (dev->dma != (unsigned char)-1)
			printk(" DMA %d\n", dev->dma);

		if (!is_valid_ether_addr(dev->dev_addr)) {
			printk("%s: Invalid ethernet MAC address.  Please "
			       "set using ifconfig\n", dev->name);
		} else {
			/* Print the Ethernet address */
			printk("%s: Ethernet addr: ", dev->name);
			for (i = 0; i < 5; i++)
				printk("%2.2x:", dev->dev_addr[i]);
			printk("%2.2x\n", dev->dev_addr[5]);
		}
	}

	return 0;
}


/*
  Open the interface.
  The interface is opened whenever "ifconfig" actives it.
*/
static int dmfe_open(struct net_device *dev)
{
	board_info_t *db = (board_info_t *)dev->priv;

	DMFE_DBUG(0, "dmfe_open", 0);

	if (request_irq(dev->irq,&dmfe_interrupt,SA_SHIRQ,dev->name,dev)) 
		return -EAGAIN;

	/* Grab the IRQ */
	gio_enable_irq(GIO_ETHER, GIO_RISING_EDGE);

	/* Initilize DM910X board */
	dmfe_init_dm9000(dev);
 
	/* Init driver variable */
	db->reset_counter 	= 0;
	db->reset_tx_timeout 	= 0;
	db->cont_rx_pkt_cnt	= 0;

	db->Speed = 10; /* default value */
 	
	netif_start_queue(dev);

	return 0;
}

/* Set PHY operationg mode
*/
static void set_PHY_mode(board_info_t *db)
{
	u16 phy_reg0 = 0x1200;		/* Auto-negotiation & Restart Auto-negotiation */
	u16 phy_reg4 = 0x01e1;		/* Default flow control disable*/

	if ( !(db->op_mode & DM9KS_AUTO) ) // op_mode didn't auto sense */
	{ 
		switch(db->op_mode) {
			case DM9KS_10MHD:  phy_reg4 = 0x21; 
                        	           phy_reg0 = 0x1000;
					   break;
			case DM9KS_10MFD:  phy_reg4 = 0x41; 
					   phy_reg0 = 0x1100;
                                	   break;
			case DM9KS_100MHD: phy_reg4 = 0x81; 
					   phy_reg0 = 0x3000;
				    	   break;
			case DM9KS_100MFD: phy_reg4 = 0x101; 
					   phy_reg0 = 0x3100;
				   	   break;
			default: 
					   break;
		} // end of switch
	} // end of if
	phy_write(db, 0, phy_reg0);
	phy_write(db, 4, phy_reg4);
}

/* 
	Initilize dm9000 board
*/
static void dmfe_init_dm9000(struct net_device *dev)
{
	board_info_t *db = (board_info_t *)dev->priv;
	DMFE_DBUG(0, "dmfe_init_dm9000()", 0);

	/* set the internal PHY power-on, GPIOs normal, and wait 2ms */
	iow(db, DM9KS_GPR, 1); 	/* Power-Down PHY */
	udelay(500);
	iow(db, DM9KS_GPR, 0);	/* GPR (reg_1Fh)bit GPIO0=0 pre-activate PHY */
	udelay(20);		/* wait 2ms for PHY power-on ready */

	/* do a software reset and wait 20us */
	iow(db, DM9KS_NCR, 3);
	udelay(20);		/* wait 20us at least for software reset ok */
	iow(db, DM9KS_NCR, 3);	/* NCR (reg_00h) bit[0] RST=1 & Loopback=1, reset on */
	udelay(20);		/* wait 20us at least for software reset ok */

	/* I/O mode */
	db->io_mode = ior(db, DM9KS_ISR) >> 6; /* ISR bit7:6 keeps I/O mode */

	/* Set PHY */
	db->op_mode = media_mode;
	set_PHY_mode(db);

	/* Program operating register */
	iow(db, DM9KS_NCR, 0);
	iow(db, DM9KS_TCR, 0);		/* TX Polling clear */
	iow(db, DM9KS_BPTR, 0x3f);	/* Less 3kb, 600us */
	iow(db, DM9KS_SMCR, 0);		/* Special Mode */
	iow(db, DM9KS_NSR, 0x2c);	/* clear TX status */
	iow(db, DM9KS_ISR, 0x0f); 	/* Clear interrupt status */
	iow(db, DM9KS_TCR2, 0x80);	/* Set LED mode 1 */

	/* Added by jackal at 03/29/2004 */
#if defined(CHECKSUM)
	iow(db, DM9KS_TCCR, 0x07);	/* TX UDP/TCP/IP checksum enable */
	iow(db, DM9KS_RCSR, 0x02);	/*Receive checksum enable */
#endif

#if defined(ETRANS)
	iow(db, DM9KS_ETXCSR, 0x83);
#endif
 
	/* Set address filter table */
	dm9000_hash_table(dev);

	/* Activate DM9000A/DM9010 */
	iow(db, DM9KS_RXCR, DM9KS_REG05 | 1);	/* RX enable */
	iow(db, DM9KS_IMR, DM9KS_REGFF); 	// Enable TX/RX interrupt mask
 
	/* Init Driver variable */
	db->tx_pkt_cnt 		= 0;
		
	netif_carrier_on(dev);
	spin_lock_init(&db->lock);
}

/*
  Hardware start transmission.
  Send a packet to media from the upper layer.
*/
static int dmfe_start_xmit(struct sk_buff *skb, struct net_device *dev)
{
	board_info_t *db = (board_info_t *)dev->priv;
	u16 * data_ptr;
	int tmplen;
	if(db->Speed == 10)
		{if (db->tx_pkt_cnt >= 1) return 1;}
	else
		{if (db->tx_pkt_cnt >= 2) return 1;}
	
	/* packet counting */
	db->tx_pkt_cnt++;

	db->stats.tx_packets++;
	db->stats.tx_bytes+=skb->len;
	if (db->Speed == 10)
		{if (db->tx_pkt_cnt >= 1) netif_stop_queue(dev);}
	else
		{if (db->tx_pkt_cnt >= 2) netif_stop_queue(dev);}

	/* Disable all interrupt */
	iow(db, DM9KS_IMR, DM9KS_DISINTR);

	/* Set TX length to reg. 0xfc & 0xfd */
	iow(db, DM9KS_TXPLL, (skb->len & 0xff));
	iow(db, DM9KS_TXPLH, (skb->len >> 8) & 0xff);

	/* Move data to TX SRAM */
	data_ptr = (u16 *)skb->data;
	
	PUTB(DM9KS_MWCMD, db->io_addr); // Write data into SRAM trigger

	if(dma_mode)
	  {
	    arm926_dma_flush_range((u32) skb->data, (u32) (skb->data + skb->len+4));
	    
	    PUTW(*data_ptr++, db->io_data); /* Make it always 4-byte aligned */
	    tmplen = skb->len - 2;
	    
	    {
	      u_long dest_addr, net_addr;
	      
	      net_addr = virt_to_phys((void *)data_ptr);
	      dest_addr =  db->io_data - db->io_addr;
	      
	      outw(0x0820, IO_SDRAM_SDDMASEL);/* MTC 1 burst DMA */
	      outw(0x0004, IO_EMIF_DMAMTCSEL);/* DMA MTC Select: CS4*/
	      
	      outw(((dest_addr & 0x0FFF0000) >> 16) | 0x8000 , IO_EMIF_MTCADDH);
	      outw(dest_addr & 0x0000FFFF, IO_EMIF_MTCADDL);
	      
	      outw((net_addr & 0x7FFF0000) >> 16, IO_EMIF_AHBADDH);
	      outw(net_addr & 0x0000FFFF, IO_EMIF_AHBADDL);
	      
	      outw((tmplen+1) >> 2, IO_EMIF_DMASIZE);
	      
	      /* Start the DMA transfer */
	      outw(0x0000, IO_EMIF_DMACTL);
	      outw(0x0001, IO_EMIF_DMACTL);
	      
	      while(inw(IO_EMIF_DMACTL) & 1) ;	/* Wait for DMA finished */
	      if ((tmplen & 0x3)==0x1 || (tmplen & 0x3)==0x2)
		PUTW(data_ptr[(tmplen & ~0x3) >> 1], db->io_data);
	    }
	  }
	else	/* PIO transfer */
	  {
	    int i;
	    tmplen = (skb->len + 1) / 2;
	    for (i = 0; i < tmplen; i++)
	      PUTW(*data_ptr++, db->io_data);
	  }
	
#if !defined(ETRANS)
	/* Issue TX polling command */
	iow(db, DM9KS_TCR, 0x1); /* Cleared after TX complete*/
#endif

	/* Saved the time stamp */
	dev->trans_start = jiffies;
	db->cont_rx_pkt_cnt =0;

	/* Free this SKB */
	dev_kfree_skb(skb);

	/* Re-enable interrupt */
	iow(db, DM9KS_IMR, DM9KS_REGFF);

	return 0;
}

/*
  Stop the interface.
  The interface is stopped when it is brought.
*/
static int dmfe_stop(struct net_device *dev)
{
	board_info_t *db = (board_info_t *)dev->priv;
	DMFE_DBUG(0, "dmfe_stop", 0);

	netif_stop_queue(dev); 

	/* free interrupt */
	free_irq(dev->irq, dev);

	/* RESET devie */
	phy_write(db, 0x00, 0x8000);	/* PHY RESET */
	iow(db, DM9KS_GPR, 0x01); 	/* Power-Down PHY */
	iow(db, DM9KS_IMR, DM9KS_DISINTR);	/* Disable all interrupt */
	iow(db, DM9KS_RXCR, 0x00);	/* Disable RX */

	/* Dump Statistic counter */
#if FALSE
	printk("\nRX FIFO OVERFLOW %lx\n", db->stats.rx_fifo_errors);
	printk("RX CRC %lx\n", db->stats.rx_crc_errors);
	printk("RX LEN Err %lx\n", db->stats.rx_length_errors);
	printk("RESET %x\n", db->reset_counter);
	printk("RESET: TX Timeout %x\n", db->reset_tx_timeout);
	printk("g_TX_nsr %x\n", g_TX_nsr);
#endif

	return 0;
}

static void dmfe_tx_done(unsigned long unused)
{
	struct net_device *dev = dmfe_dev;
	board_info_t *db = (board_info_t *)dev->priv;
	int  nsr;

	DMFE_DBUG(0, "dmfe_tx_done()", 0);
	
	nsr = ior(db, DM9KS_NSR);
	if (nsr & 0x0c)
	  {
	      if(nsr & 0x04) db->tx_pkt_cnt--;
	      if(nsr & 0x08) db->tx_pkt_cnt--;
	      if(db->tx_pkt_cnt < 0)
		{
		    printk("[dmfe_tx_done] tx_pkt_cnt ERROR!!\n");
		    while(ior(db,DM9KS_TCR) & 0x1){}
		    db->tx_pkt_cnt = 0;
		}
	  }
	else
	  {
	      while(ior(db,DM9KS_TCR) & 0x1){}
	      db->tx_pkt_cnt = 0;
	  }
		
	netif_wake_queue(dev);
	
	return;
}

/*
  DM9000 insterrupt handler
  receive the packet to upper layer, free the transmitted packet
*/
static irqreturn_t dmfe_interrupt(int irq, void *dev_id, struct pt_regs *regs)
{
	struct net_device *dev = dev_id;
	board_info_t *db;
	int int_status,i;
	u8 reg_save;

	DMFE_DBUG(0, "dmfe_interrupt()", 0);

	/* A real interrupt coming */
	db = (board_info_t *)dev->priv;
	spin_lock(&db->lock);

	/* Save previous register address */
	reg_save = GETB(db->io_addr);

	/* Disable all interrupt */
	iow(db, DM9KS_IMR, DM9KS_DISINTR); 

	/* Got DM9000A/DM9010 interrupt status */
	int_status = ior(db, DM9KS_ISR);		/* Got ISR */
	iow(db, DM9KS_ISR, int_status);		/* Clear ISR status */ 

	/* Link status change */
	if (int_status & DM9KS_LINK_INTR) 
	{
		netif_stop_queue(dev);
		for(i=0; i<500; i++) /*wait link OK, waiting time =0.5s */
		{
			phy_read(db,0x1);
			if(phy_read(db,0x1) & 0x4) /*Link OK*/
			{
				/* wait for detected Speed */
				for(i=0; i<200;i++)
					udelay(1000);
				/* set media speed */
				if(phy_read(db,0)&0x2000) db->Speed =100;
				else db->Speed =10;
				break;
			}
			udelay(1000);
		}
		netif_wake_queue(dev);
		//printk("[INTR]i=%d speed=%d\n",i, (int)(db->Speed));	
	}
	/* Received the coming packet */
	if (int_status & DM9KS_RX_INTR) 
		dmfe_packet_receive(dev);

	/* Trnasmit Interrupt check */
	if (int_status & DM9KS_TX_INTR)
		dmfe_tx_done(0);
	
	if (db->cont_rx_pkt_cnt>=CONT_RX_PKT_CNT)
	{
		iow(db, DM9KS_IMR, 0xa2);
	}
	else
	{
		/* Re-enable interrupt mask */ 
		iow(db, DM9KS_IMR, DM9KS_REGFF);
	}
	
	/* Restore previous register address */
	PUTB(reg_save, db->io_addr); 

	spin_unlock(&db->lock); 

	return IRQ_HANDLED;
}

/*
  Get statistics from driver.
*/
static struct net_device_stats * dmfe_get_stats(struct net_device *dev)
{
	board_info_t *db = (board_info_t *)dev->priv;
	DMFE_DBUG(0, "dmfe_get_stats", 0);
	return &db->stats;
}

/* Our watchdog timed out. Called by the networking layer */
static void
dmfe_timeout(struct net_device *dev)
{
	board_info_t *db = (board_info_t *)dev->priv;

	DMFE_DBUG(0, "dmfe_TX_timeout()", 0);
	printk("TX time-out -- dmfe_timeout().\n");
	db->reset_tx_timeout++;
	db->stats.tx_errors++;
#if FALSE
	printk("TX packet count = %d\n", db->tx_pkt_cnt);	
	printk("TX timeout = %d\n", db->reset_tx_timeout);	
	printk("22H=0x%02x  23H=0x%02x\n",ior(db,0x22),ior(db,0x23));
	printk("faH=0x%02x  fbH=0x%02x\n",ior(db,0xfa),ior(db,0xfb));
#endif
	dmfe_reset(dev);
}

static void dmfe_reset(struct net_device * dev)
{
	board_info_t *db = (board_info_t *)dev->priv;
	u8 reg_save;
	int i;
	/* Save previous register address */
	reg_save = GETB(db->io_addr);

	netif_stop_queue(dev); 
	db->reset_counter++;
	dmfe_init_dm9000(dev);
	
	db->Speed =10;
	for(i=0; i<1000; i++) /*wait link OK, waiting time=1 second */
	{
		if(phy_read(db,0x1) & 0x4) /*Link OK*/
		{
			if(phy_read(db,0)&0x2000) db->Speed =100;
			else db->Speed =10;
			break;
		}
		udelay(1000);
	}
	
	netif_wake_queue(dev);
	
	/* Restore previous register address */
	PUTB(reg_save, db->io_addr);
}

#if !defined(CHECKSUM)
#define check_rx_ready(a)	((a) == 0x01)
#else
inline u8 check_rx_ready(u8 rxbyte)
{
	if (!(rxbyte & 0x01))
		return 0;
	return ((rxbyte >> 4) | 0x01);
}
#endif

/*
  Received a packet and pass to upper layer
*/
static void dmfe_packet_receive(struct net_device *dev)
{
	board_info_t *db = (board_info_t *)dev->priv;
	struct sk_buff *skb;
	u8 rxbyte, val;
	u16 i, GoodPacket, tmplen = 0, MDRAH, MDRAL;

	rx_t rx;

	u16 * ptr = (u16*)&rx;
	u16 *rdptr16;

	DMFE_DBUG(0, "dmfe_packet_receive()", 0);

	db->cont_rx_pkt_cnt=0;

	do {
		/*store the value of Memory Data Read address register*/
		MDRAH=ior(db, DM9KS_MDRAH);
		MDRAL=ior(db, DM9KS_MDRAL);
		
		ior(db, DM9KS_MRCMDX);		/* Dummy read */
		rxbyte = GETB(db->io_data);	/* Got most updated data */

		/* packet ready to receive check */
		if(!(val = check_rx_ready(rxbyte))) break;

		/* A packet ready now  & Get status/length */
		GoodPacket = TRUE;
		PUTB(DM9KS_MRCMD, db->io_addr);

		/* Read packet status & length */
		*ptr = GETW(db->io_data);
		*(ptr+1)    = GETW(db->io_data);
		//printk("rx.desc.length=%d\n", rx.desc.length);

		/* Packet status check */
		if (rx.desc.status & 0xbf)
		{
			GoodPacket = FALSE;
			if (rx.desc.status & 0x01) 
			{
				db->stats.rx_fifo_errors++;
				printk("<RX FIFO error>\n");
			}
			if (rx.desc.status & 0x02) 
			{
				db->stats.rx_crc_errors++;
				printk("<RX CRC error>\n");
			}
			if (rx.desc.status & 0x80) 
			{
				db->stats.rx_length_errors++;
				printk("<RX Length error>\n");
			}
			if (rx.desc.status & 0x08)
				printk("<Physical Layer error>\n");
		}

		if (unlikely(!GoodPacket))
		{
			// drop this packet!!!
		        tmplen = (rx.desc.length + 1) / 2;
			for (i = 0; i < tmplen; i++)
			       GETW(db->io_data);

			continue;/*next the packet*/
		}
		
		skb = dev_alloc_skb(rx.desc.length+4);
		if (skb == NULL )
		{	
			printk(KERN_INFO "%s: Memory squeeze.\n", dev->name);
			/*re-load the value into Memory data read address register*/
			iow(db,DM9KS_MDRAH,MDRAH);
			iow(db,DM9KS_MDRAL,MDRAL);
			return;
		}
		else
		{
			arm926_dma_flush_range((int) skb->tail, (int) (skb->tail + (rx.desc.length+8) ));

			/* Move data from DM9000 */
			skb->dev = dev;
			skb_reserve(skb, 2);
			rdptr16 = (u16*)skb_put(skb, rx.desc.length - 4);

			if(dma_mode)
			  {
			    *rdptr16++ = GETW(db->io_data);		/* Make it always 4-byte aligned */
			    tmplen = rx.desc.length -2;
			    
			    {	/* Setup DMA transfer */
			      u_long dest_addr, net_addr;
			      
			      dest_addr = virt_to_phys((void*) rdptr16);
			      net_addr =  db->io_data - db->io_addr;
			      
			      outw(0x0820, IO_SDRAM_SDDMASEL);	/* MTC 1 burst DMA */
			      outw(0x0004, IO_EMIF_DMAMTCSEL);	/* DMA MTC Select: CS4*/
			      
			      outw(((net_addr & 0x0FFF0000) >> 16) | 0x8000, IO_EMIF_MTCADDH);
			      outw(net_addr & 0x0000FFFF, IO_EMIF_MTCADDL);
			      
			      outw((dest_addr & 0x7FFF0000) >> 16, IO_EMIF_AHBADDH);
			      outw(dest_addr & 0x0000FFFF, IO_EMIF_AHBADDL);
			      
			      outw((tmplen+1) >> 2, IO_EMIF_DMASIZE);
			      
			      /* Start the DMA transfer */
			      outw(0x0002, IO_EMIF_DMACTL);
			      outw(0x0003, IO_EMIF_DMACTL);
			      
			      while(inw(IO_EMIF_DMACTL) & 1) ;	/* Wait for DMA finished */
			      
			      if ( (tmplen & 0x3)==0x1 || (tmplen & 0x3)==0x2 )
				rdptr16[(tmplen & ~0x3) >> 1] = GETW(db->io_data);
			    }
			  }
			else	/* PIO transfer */
			  {
			    /* Read received packet from RX SRAM */
			    tmplen = (rx.desc.length + 1) / 2;
			    for (i=0; i<tmplen; i++)
			      *rdptr16++ = GETW(db->io_data);
			  }

			/* Pass to upper layer */
			skb->protocol = eth_type_trans(skb,dev);
#if defined(CHECKSUM)
			if (val == 0x01)
				skb->ip_summed = CHECKSUM_UNNECESSARY;
#endif
			netif_rx(skb);
			dev->last_rx=jiffies;
			db->stats.rx_packets++;
			db->stats.rx_bytes += rx.desc.length;
			db->cont_rx_pkt_cnt++;
				
			if (db->cont_rx_pkt_cnt>=CONT_RX_PKT_CNT)
			{
				dmfe_tx_done(0);
				break;
			}
		}
			
	}while((rxbyte & 0x01) == DM9KS_PKT_RDY);
	DMFE_DBUG(0, "[END]dmfe_packet_receive()", 0);
}

#ifdef CURRENTLY_UNUSED
/*
  Read a word data from SROM
*/
static u16 read_srom_word(board_info_t *db, int offset)
{
	iow(db, DM9KS_EPAR, offset);
	iow(db, DM9KS_EPCR, 0x4);
	udelay(200);
	iow(db, DM9KS_EPCR, 0x0);
	return (ior(db, DM9KS_EPDRL) + (ior(db, DM9KS_EPDRH) << 8) );
}
#endif

/*
  Set DM9000A/DM9010 multicast address
*/
static void dm9000_hash_table(struct net_device *dev)
{
	board_info_t *db = (board_info_t *)dev->priv;
	struct dev_mc_list *mcptr = dev->mc_list;
	int mc_cnt = dev->mc_count;
	u32 hash_val;
	u16 i, oft, hash_table[4];

	DMFE_DBUG(0, "dm9000_hash_table()", 0);

	/* Set Node address */
	for (i = 0, oft = 0x10; i < 6; i++, oft++)
		iow(db, oft, dev->dev_addr[i]);

	/* Clear Hash Table */
	for (i = 0; i < 4; i++)
		hash_table[i] = 0x0;

	/* broadcast address */
	hash_table[3] = 0x8000;

	/* the multicast address in Hash Table : 64 bits */
	for (i = 0; i < mc_cnt; i++, mcptr = mcptr->next) {
		hash_val = cal_CRC((char *)mcptr->dmi_addr, 6, 0) & 0x3f; 
		hash_table[hash_val / 16] |= (u16) 1 << (hash_val % 16);
	}

	/* Write the hash table to MAC MD table */
	for (i = 0, oft = 0x16; i < 4; i++) {
		iow(db, oft++, hash_table[i] & 0xff);
		iow(db, oft++, (hash_table[i] >> 8) & 0xff);
	}
}

/*
  Calculate the CRC valude of the Rx packet
  flag = 1 : return the reverse CRC (for the received packet CRC)
         0 : return the normal CRC (for Hash Table index)
*/
static unsigned long cal_CRC(unsigned char * Data, unsigned int Len, u8 flag)
{
	u32 crc = ether_crc_le(Len, Data);

	if (flag) 
		return ~crc;
		
	return crc;
}

/*
   Read a byte from I/O port
*/
static u8 ior(board_info_t *db, int reg)
{
	PUTB(reg, db->io_addr);
	return GETB(db->io_data);
}

/*
   Write a byte to I/O port
*/
static void iow(board_info_t *db, int reg, u8 value)
{
	PUTB(reg, db->io_addr);
	PUTB(value, db->io_data);
}

/*
   Read a word from phyxcer
*/
static u16 phy_read(board_info_t *db, int reg)
{
	/* Fill the phyxcer register into REG_0C */
	iow(db, DM9KS_EPAR, DM9KS_PHY | reg);

	iow(db, DM9KS_EPCR, 0xc); 	/* Issue phyxcer read command */
	udelay(100);			/* Wait read complete */
	iow(db, DM9KS_EPCR, 0x0); 	/* Clear phyxcer read command */

	/* The read data keeps on REG_0D & REG_0E */
	return ( ior(db, DM9KS_EPDRH) << 8 ) | ior(db, DM9KS_EPDRL);
}

/*
   Write a word to phyxcer
*/
static void phy_write(board_info_t *db, int reg, u16 value)
{
	/* Fill the phyxcer register into REG_0C */
	iow(db, DM9KS_EPAR, DM9KS_PHY | reg);

	/* Fill the written data into REG_0D & REG_0E */
	iow(db, DM9KS_EPDRL, (value & 0xff));
	iow(db, DM9KS_EPDRH, ( (value >> 8) & 0xff));

	iow(db, DM9KS_EPCR, 0xa);	/* Issue phyxcer write command */
	udelay(500);			/* Wait write complete */
	iow(db, DM9KS_EPCR, 0x0);	/* Clear phyxcer write command */
}

/*
 * probe the driver
 */
static int dm9k_drv_probe(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct net_device *ndev;
	unsigned long base;
	unsigned int *addr = NULL;
	int ret = -ENODEV;

	ndev = alloc_etherdev(sizeof(struct board_info));
	if (!ndev) {
		printk("%s: could not allocate device.\n", CARDNAME);
		return -ENOMEM;
	}
	SET_MODULE_OWNER(ndev);
	SET_NETDEV_DEV(ndev, dev);

	ndev->dma = (unsigned char)-1;

	if (pdev->num_resources < 2 || pdev->num_resources > 3) {
		printk("DM9000: Wrong num of resources %d\n", pdev->num_resources);
		ret = -ENODEV;
		goto out;
	}

	base = pdev->resource[0].start;
	ndev->irq = pdev->resource[1].start;

	/*
	 * Request the regions.
	 */
	if (!request_mem_region(base, DATAOFST2, ndev->name)) {
		ret = -EBUSY;
		goto out;
	}


	addr = ioremap(base, DATAOFST2);
	if (!addr) {
		ret = -ENOMEM;
		goto release_mem;
	}

	dev_set_drvdata(dev, ndev);
	ret = dm9k_probe(ndev, (unsigned long)addr);
	if (ret != 0) {
		dev_set_drvdata(dev, NULL);
		iounmap(addr);
 release_mem:
		release_mem_region(base, DATAOFST2);
 out:
		printk("%s: not found (%d).\n", CARDNAME, ret);
		kfree(ndev);
	}

	return ret;
}

static int dm9k_drv_remove(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct net_device *ndev = dev_get_drvdata(dev);

	dev_set_drvdata(dev, NULL);

	unregister_netdev(ndev);

	gio_disable_irq(GIO_ETHER);
	free_irq(ndev->irq, ndev);

	iounmap((void *)ndev->base_addr);
	release_mem_region(pdev->resource[0].start, 4);

	kfree(ndev);
	return 0;
}

static int dm9k_drv_suspend(struct device *dev, pm_message_t state)
{
	return 0;
}

static int dm9k_drv_resume(struct device *dev)
{
	return 0;
}

static struct device_driver dm9k_driver = {
	.name		= CARDNAME,
	.bus		= &platform_bus_type,
	.probe		= dm9k_drv_probe,
	.remove		= dm9k_drv_remove,
	.suspend	= dm9k_drv_suspend,
	.resume		= dm9k_drv_resume,
};

/* Description: 
   when user used insmod to add module, system invoked init_module()
   to initilize and register.
*/
int dm9k_init_module(void)
{
	switch(mode) {
		case DM9KS_10MHD:
		case DM9KS_100MHD:
		case DM9KS_10MFD:
		case DM9KS_100MFD:
			media_mode = mode;
			break;
		default:
			media_mode = DM9KS_AUTO;
	}

	if (request_gio(GIO_ETHER))
		return -ENODEV;

	return driver_register(&dm9k_driver);
}

/* Description: 
   when user used rmmod to delete module, system invoked clean_module()
   to  un-register DEVICE.
*/
void dm9k_cleanup_module(void)
{
	struct net_device *dev = dmfe_dev;
	DMFE_DBUG(0, "clean_module()", 0);

	unregister_netdev(dmfe_dev);
	release_region(dev->base_addr, 2);
	free_netdev(dev);
	
	DMFE_DBUG(0, "clean_module() exit", 0);
}

module_init(dm9k_init_module);
module_exit(dm9k_cleanup_module);
