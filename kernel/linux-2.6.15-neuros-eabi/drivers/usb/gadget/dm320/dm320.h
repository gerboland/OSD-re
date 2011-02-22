/* DM320 high/full speed USB device controller
 * 
 * Copyright (C) 2005 Integrated Softtech solutions.
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
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#ifdef __KERNEL__



#ifndef __DM320_H__
#define __DM320_H_

#include "dm320_usb_config.h"
//#include <asm/barrier.h>
#define USB_ENDPOINT_HALT 0    //Found in Linux 2.6.8 kernel onwards.




#ifndef container_of
#define container_of    list_entry
#endif

#ifndef likely
#define likely(x)       (x)
#define unlikely(x)     (x)
#endif

#ifndef BUG_ON
#define BUG_ON(condition)       \
	        do { if (unlikely((condition)!=0)) BUG(); } while (0)
#endif

#ifndef WARN_ON
#define WARN_ON(x)      do { } while (0)
#endif

#ifndef IRQ_NONE
typedef void irqreturn_t;
#define IRQ_NONE
#define IRQ_HANDLED
#define IRQ_RETVAL(x)
#endif
#define NUM_OF_ENDPOINTS 3

typedef struct usbEndpointInfo
{
    char            fAddr;
    unsigned short  nSize;
    char            fBufMode;
    char            fTransMode;

} USB_EP_INFO;



USB_EP_INFO     stEndpointInfo[ 3 ]     =
{       // fAddr                // nSize            // fBufMode     // fTransMode
    {   BULK_IN_EP | 0x80,             BULK_TRANS_SIZE,    EP_BUF_SINGLE,  BULK_EP },
    {   BULK_OUT_EP ,     BULK_TRANS_SIZE,    EP_BUF_SINGLE,  BULK_EP },      // DM320 output transfer endpoint
    {   INTR_IN_EP| 0x80,             INTR_TRANS_SIZE,    EP_BUF_SINGLE,  INTR_EP }
};


struct dm320_ep {
	struct usb_ep			ep;
	struct dm320			*dev;
	unsigned long			irqs;
	int pending;

	/* analogous to a host-side qh */
	struct list_head		queue;
	const struct			usb_endpoint_descriptor *desc;
	unsigned			num:8,
					fifo_size:12,
					stopped:1, 
					is_in:1, 
					is_iso:1,
					dma:1,
					not_empty:1;
};


struct dm320 {
        /* each device provides one gadget, several endpoints */
        struct usb_gadget               gadget;
        spinlock_t                      lock;
        struct dm320_ep		            ep[NUM_OF_ENDPOINTS + 1]; // 1*EP0 +2( EP1 + EP2 + EP3 + EP4 ) 
        struct usb_gadget_driver        *driver;
        unsigned                        enabled:1,
                                        protocol_stall:1,
                                        softconnect:1,
                                        is_selfpowered:1,
                                        wakeup:1,
                                        got_irq:1;
        u16                             chiprev;

        unsigned int                    indexed_threshold;    /* Vishal: Net2272 specific */
        u8                              pagesel;	      /* Vishal: used to select one of the end point register pages */	
        unsigned int                    irq;
        unsigned int                    attach_irq;
        unsigned int                    dma_irq;
        unsigned int                    core_irq;
		unsigned char					paddr_status;    // Peripheral address assigned?
		unsigned char					peripheral_addr;
		int								last_event;
        struct platform_device          *pdev;
		u8								status;
};


extern struct dm320	*the_controller;


static inline void
dm320_ep_write (struct dm320_ep *ep, unsigned int reg, u8 value)
{
	writeb( ep->num , USB_INDEX );
	writeb( value ,reg );
}



static inline u8
dm320_ep_read (struct dm320_ep *ep, unsigned int reg)
{
	struct dm320			*dev = ep->dev;
	writeb( ep->num , USB_INDEX );
	return readb(reg);
}

int     USB_WriteEP0( void *pvBuffer, int iSize )
{
    unsigned char   *pucBuf         = (unsigned char *)pvBuffer;
    int             iCount;
    unsigned char   fCSR0RegVal     = USB_CSR0_TXPKTRDY;        // transmit packet ready flag
    int             nWritten;
	core_debug("dm320: %s: write EPO: %d bytes\n", __FUNCTION__,  iSize);
    if( EP0_FIFO_SIZE >= iSize )
	    {
    	    iCount       = iSize;
	        fCSR0RegVal |= USB_CSR0_DATAEND;                        // set trasaction end flag
	    }
	    else
	        iCount       = EP0_FIFO_SIZE;
	
	    nWritten = iCount;
	    while( iCount > 0 )
	    {
	        writeb( *pucBuf++, USB_FIFO0 );
	        iCount--;
	    }
   	writeb( fCSR0RegVal, USB_PER_CSR0 );


    if( nWritten > 0 )
        core_debug( KERN_INFO "dm320: %s : EP0 : %d bytes written\n",__FUNCTION__, nWritten );

    return nWritten;
}

int flush_ep_fifo(int ep_num)
{

    writeb( ep_num, USB_INDEX );
    writeb( USB_CSR2_FLFIFO, USB_CSR2 );            // flush FIFO
    writeb( USB_CSR2_FLFIFO, USB_CSR2 );            // flush FIFO for double buffer
//	if(ep_num == EP_BULK_IN_NUM )
//	    writeb( readb( USB_PER_TXCSR1 ) | USB_TXCSR1_TXPKTRDY, USB_PER_TXCSR1 );
//	if(ep_num == EP_BULK_OUT_NUM )
//	    writeb( readb( USB_PER_RXCSR1 ) & 0xFE, USB_PER_RXCSR1 );   // manually clear RXPKTRDY bit in PER_RXCSR1 register
}


int     USB_WriteEP( int iEndpoint, void *pvBuffer, int iSize )
{
    volatile unsigned char      *pucFIFO;
    unsigned char               *pucBuf;
    int                         iRet            = -1;
    int                         iCount;
	
    if( ( iEndpoint < USB_EP0_SELECT ) || ( iEndpoint >= TX_EP_MAX ) )
        return iRet;

    writeb( iEndpoint, USB_INDEX );

    if( iEndpoint == USB_EP0_SELECT )
    {
        iRet = USB_WriteEP0( pvBuffer, iSize );
        return iRet;
    }

    iCount = BULK_TRANS_SIZE;
    if( iCount > iSize )
        iCount = iSize;

    iRet    = iCount;
    pucBuf  = (unsigned char *)pvBuffer;
    pucFIFO = (unsigned char *)USB_FIFO0;
    pucFIFO = pucFIFO + ( iEndpoint * 4 );

	if((readb(USB_PER_TXCSR1) & USB_TXCSR1_FIFOEMP) )
	{
    		writeb( readb( USB_PER_TXCSR1 ) | USB_TXCSR1_TXPKTRDY, USB_PER_TXCSR1 );
// 		mdelay(5);
		while( readb( USB_PER_TXCSR1 )& USB_TXCSR1_TXPKTRDY);
		writeb((readb(USB_PER_TXCSR1) | USB_TXCSR1_FLFIFO) , USB_PER_TXCSR1);
// 		mdelay(5);
	}
	
#if 0
	if(!(readb(USB_PER_TXCSR1) & USB_TXCSR1_FIFOEMP))
    {
			printk( "Data Tx: Tx  FIFO Contains Data. Flushing \n");
            writeb((readb(USB_PER_TXCSR1) | USB_TXCSR1_FLFIFO) , USB_PER_TXCSR1);
		//	while(!(readb(USB_PER_TXCSR1) & USB_TXCSR1_FIFOEMP));
			udelay(1);
    }
#endif
	
	
    while( iCount > 0 )
    {
        *pucFIFO = *pucBuf++;
        iCount--;
    }
    writeb( readb( USB_PER_TXCSR1 ) | USB_TXCSR1_TXPKTRDY, USB_PER_TXCSR1 );
    return iRet;
}

int bytes_in_fifo(int iEndpoint)
{

    int iCount;
    writeb( iEndpoint, USB_INDEX );
	iCount = readb( USB_RXCOUNT2 );
    iCount = ( iCount << 8 ) + readb( USB_RXCOUNT1 );
	return iCount;

}


void    dm320_set_CSR0_reg( unsigned char fFlag )
{
    writeb( fFlag, USB_PER_CSR0 );
}

void clear_rxpktrdy(int iEndpoint)
{
	if( ( iEndpoint < USB_EP0_SELECT ) || ( iEndpoint >= RX_EP_MAX ) )
        return ;
    writeb( iEndpoint, USB_INDEX );

	if( iEndpoint == USB_EP0_SELECT )
      dm320_set_CSR0_reg( USB_CSR0_CLRRXRDY);
    else
	{
      writeb( readb( USB_PER_RXCSR1 ) & 0xFE, USB_PER_RXCSR1 );   // manually clear RXPKTRDY bit in PER_RXCSR1 register
	}

}


int     USB_ReadEP( int iEndpoint, void *pvBuffer, int iSize )
{
    int                     iRet        = -1;
    int                     iCount;
    volatile unsigned char  *pucFIFO;
    unsigned char           *pucBuf;
//	printk("%s: reading from endpoint %d\n", __FUNCTION__, iEndpoint);
    if( ( iEndpoint < USB_EP0_SELECT ) || ( iEndpoint >= RX_EP_MAX ) )
        return iRet;
    writeb( iEndpoint, USB_INDEX );
    if( iEndpoint == USB_EP0_SELECT )
    {
        iCount = readb( USB_COUNT0 );
        if( iCount > iSize )
            iCount = iSize;
    }
    else
    {
		if(iSize < BULK_TRANS_SIZE)
			printk("%s: reading the last %d  bytes\n", __FUNCTION__, iSize);

        iCount = readb( USB_RXCOUNT2 );
        iCount = ( iCount << 8 ) + readb( USB_RXCOUNT1 );
        if( iCount > iSize )
            iCount = iSize;
    }

#if 0
	if(readb(USB_PER_RXCSR1) & USB_RXCSR1_RXPKTRDY)
		printk("RX: Data Recieved: \n");
	if(readb(USB_PER_RXCSR1) & USB_RXCSR1_OVERRUN )//USB_RXCSR1_FIFOFUL)
		printk("RX: fifo OVERRUN OCCURED!!\n");
	if(readb(USB_PER_RXCSR1) & USB_RXCSR1_FIFOFUL)
		printk("RX: fifo FULL!!\n");
	if(readb(USB_PER_RXCSR1) & USB_RXCSR1_DATERR)
		printk("RX: Data Error!!\n");
	if(readb(USB_PER_RXCSR1) & USB_RXCSR1_DATERR)
		printk("RX: Data Error!!\n");
//			printk( "###############   Rx  FIFO FULL .####################\n");
//            writeb((readb(USB_PER_TXCSR1) | USB_TXCSR1_FLFIFO) , USB_PER_TXCSR1);
#endif
#if 0
	if(readb(USB_PER_RXCSR1) & USB_RXCSR1_FIFOFUL)
   		writeb((readb(USB_PER_RXCSR1) | USB_RXCSR1_FLFIFO) , USB_PER_RXCSR1);
#endif


    iRet    = iCount;
    pucBuf  = (unsigned char *)pvBuffer;
    pucFIFO = (unsigned char *)USB_FIFO0;
    pucFIFO = pucFIFO + ( iEndpoint * 4 );
    while( iCount > 0 )
    {
      *pucBuf++ = *pucFIFO;
       iCount--;
    }

		
    
	writeb( readb( USB_PER_RXCSR1 ) & ~(USB_RXCSR1_RXPKTRDY), USB_PER_RXCSR1 );//manually clear RXPKTRDY bit in PER_RXCSR1 register
    return iRet;
}




static inline void allow_status (struct dm320_ep *ep)
{
	core_debug("dm320 : %s\n", __FUNCTION__);
}

static inline void set_halt (struct dm320_ep *ep)
{
	/* ep0 and bulk/intr endpoints */
	core_debug("dm320 : %s\n", __FUNCTION__);

}

static inline void clear_halt (struct dm320_ep *ep)
{
	core_debug("dm320 : %s\n", __FUNCTION__);
}

/* count (<= 4) bytes in the next fifo write will be valid */
static inline void set_fifo_bytecount (struct dm320_ep *ep, unsigned count)
{
  core_debug("dm320: %s: dummy function call\n",__FUNCTION__ );
}

struct dm320_request {
	struct usb_request		req;
	struct list_head		queue;
	unsigned			mapped:1, 
					valid:1;
};

#define xprintk(dev,level,fmt,args...) \
	printk(level "dm320 : " fmt , ## args)

#ifdef DEBUG
#undef DEBUG
#define DEBUG(dev,fmt,args...) \
	xprintk(dev, KERN_DEBUG , fmt , ## args)
#else
#define DEBUG(dev,fmt,args...) \
	do { } while (0)
#endif			/* DEBUG */

#ifdef VERBOSE
#define VDEBUG DEBUG
#else
#define VDEBUG(dev,fmt,args...) \
	do { } while (0)
#endif			/* VERBOSE */

#define ERROR(dev,fmt,args...) \
	xprintk(dev, KERN_ERR , fmt , ## args)
#define WARN(dev,fmt,args...) \
	xprintk(dev, KERN_WARNING , fmt , ## args)
#define INFO(dev,fmt,args...) \
	xprintk(dev, KERN_INFO , fmt , ## args)


static inline void stop_out_naking (struct dm320_ep *ep)
{
	printk("dm320: %s .\n", __FUNCTION__);
}
/*---------------------------------------------------------------------------*/

#endif //__IFNDEF __DM320_H__
#endif  /* __KERNEL__ */

