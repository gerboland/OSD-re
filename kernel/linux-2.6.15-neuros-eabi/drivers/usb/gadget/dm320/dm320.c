/*
 * linux/drivers/usb/gadget/dm320/dm320.c
 * Texas Instruments DM320 on-chip full speed USB device controllers
 * Author : Vishal Borker, November 2005, Ingenient Technologies
 * Report Bugs/Patches to vishal.borker@gmail.com
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
 *
 */
#include <linux/config.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/smp_lock.h>
#include <linux/timer.h>
#include <linux/usb_ch9.h>
#include <linux/usb_gadget.h>
#include <linux/dma-mapping.h>

#include <asm/arch/gio.h>
#include <asm/byteorder.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/system.h>
#include <asm/unaligned.h>
#include "dm320_reg_def.h"
#include "dm320_usb_config.h"
#include "dm320_usb.h"

#define EP_BULK_IN_NUM  1    // Endpoing on which we send data to the Host controller Tx EP
#define EP_BULK_OUT_NUM 2    // Endpoint on which we get data from the Host controller Rx EP
#define EP_INTR_IN_NUM  3    // INTR Endpoint on which the hos periodically polls for interrupt data.
#define HOST_CONNECT	1
#define HOST_DISCONNECT 0
/*
 * We have a set of debug MACROS's each enabling a different level of debug commands.
 * define all of the below to enable full debug support
 *
 */
static int paddr_status = 0;
#define IO_DEBUG 
#ifdef IO_DEBUG
#define printkio(fmt,args...) \
    printk(fmt , ## args)
#else
#define printkio(fmt,args...) \
    do { } while (0)
#endif          /* DEBUG */

#ifdef CORE_DEBUG
#define core_debug(fmt,args...) \
    printk(fmt , ## args)
#else
#define core_debug(fmt,args...) \
    do { } while (0)
#endif          /* DEBUG */

#ifdef PRINT_INFO
#define dm320_info(fmt,args...) \
    printk(fmt , ## args)
#else
#define dm320_info(fmt,args...) \
    do { } while (0)
#endif          /* DEBUG */


#define DM320_USB_BASE 0x80000000
#define DRIVER_DESC		"DM320 USB Peripheral Controller (test version 1.0) "
#define DRIVER_VERSION		"2005 September 01, Ingenient Technologies "
#define DMA_ADDR_INVALID	(~(dma_addr_t)0)


const char driver_name [] = "dm320";
const char driver_desc [] = DRIVER_DESC;
const char ep0name [] = "ep0";
const char *ep_name [] = {
    ep0name,
    "ep1in-bulk", "ep2out-bulk",
    "ep3in-intr",
};

#define	DEBUG			/* messages on error and most fault paths */
#define	VERBOSE			/* extra debug messages (success too) */

#include "dm320.h"

#define DIR_STRING(bAddress) (((bAddress) & USB_DIR_IN) ? "in" : "out")

/*
 * function prototypes
 */
static int use_dma = 0;
static struct usb_ep_ops dm320_ep_ops;


static void nuke (struct dm320_ep *);
static void usb_reset (struct dm320 *dev);
static void usb_reinit (struct dm320 *dev);
static void ep_reset (struct dm320_ep *ep);
static void dm320_reset_ep(unsigned int index);
static void dm320_clear_intrs();
static void dm320_reconnect_init(struct dm320 *dev);
static int kick_dma (struct dm320_ep *ep, struct dm320_request *req);
static int dm320_fifo_status (struct usb_ep *_ep);

static void    flushEPFIFO( int iEndpoint )  //Vishal: Derived from USB_FlushFifo(i)
{
//	printk("dm320 : %s: %d\n", __FUNCTION__, iEndpoint);
    if( ( iEndpoint < USB_EP0_SELECT ) || ( iEndpoint >= RX_EP_MAX ) )
        return ;
    writeb( iEndpoint, USB_INDEX );
    if( iEndpoint == USB_EP0_SELECT )
    {
        writeb( USB_CSR2_FLFIFO, USB_CSR2 );
        writeb( USB_CSR2_FLFIFO, USB_CSR2 );
    }
    else
    {
        writeb( USB_TXCSR1_FLFIFO, USB_PER_TXCSR1 );
        writeb( USB_RXCSR1_FLFIFO, USB_PER_RXCSR1 );
    }
}

static int ep_name_to_index(char *name)
{
	if(!strcmp(name,ep0name))
		return 0; 
	if(!strcmp(name,"ep1in-bulk"))
		return 1;
	if(!strcmp(name,"ep2out-bulk"))
		return 2;
	if(!strcmp(name,"ep3in-intr"))
		return 3;
}



static int
dm320_enable (struct usb_ep *_ep, const struct usb_endpoint_descriptor *desc)
{
	unsigned int ep_num;
	ep_num = ep_name_to_index(_ep->name);
	return 0;
}


static void
done (struct dm320_ep *ep, struct dm320_request *req, int status)
{
	struct dm320		*dev;
	unsigned		stopped = ep->stopped;

	if (ep->num == 0) {
		if (ep->dev->protocol_stall) {
			ep->stopped = 1;
			set_halt (ep);
		}
		allow_status (ep);
	}
	dev = the_controller;
	if (req->req.status == -EINPROGRESS)
        req->req.status = status;
	else
        	status = req->req.status;
    /* don't modify queue heads during completion callback */
//    spin_unlock (&dev->lock);
    req->req.complete (&ep->ep, &req->req);
    list_del_init (&req->queue);
  //  spin_lock (&dev->lock);
    ep->stopped = stopped;

}

/* returns: 0: still running, 1: completed, negative: errno */
static int write_fifo (struct dm320_ep *ep, struct dm320_request *req)
{
	u8		*buf;
	int ret = -1;
	int count = 0 , max = 0 , total = 0 ;
	int		is_last;
	int		status;
	int     nBytesWritten   = 0;
	while(1)
	{

		if(req){
			buf = req->req.buf + req->req.actual;
			prefetch (buf);
			total = req->req.length - req->req.actual;

		//printkio("%s: <1> request of size: %d, wrote  so far: %d, total left: %d\n", __FUNCTION__,req->req.length,req->req.actual, total);
		mdelay(1);
		}else {
		    //printkio("%s: req NULL. Invalid condition. Cannot occur. \n", __FUNCTION__);
			return 0;
		}
		count = ep->ep.maxpacket;
		if (count > total)  /* min() cannot be used on a bitfield */
        		count = total;
		if(count <= 0)
			goto finish;
		if( ( count = USB_WriteEP(ep_name_to_index(ep->ep.name),buf , total ) ) <= 0 )
        	{
            		printkio("USB_WriteEP( ) error! : %d\n", count );
	        	return ret;
        	}
		req->req.actual += count;
		//printkio("%s: <1> request of size: %d, wrote  so far: %d, total left: %d\n", __FUNCTION__,req->req.length,req->req.actual, total);
		if ((req->req.length == req->req.actual) || count < BULK_TRANS_SIZE) /*Modify to handle control ep's as well */
        	{
finish:
		
			//printkio("%s: wrote . Calling Done.\n", __FUNCTION__);
            		done (ep, req, 0);
            		return 0;
        	}
	}
	return 0;
//	return -1;
}

static int read_fifo (struct dm320_ep *ep, struct dm320_request *req)
{

	u8		*buf;
	unsigned	count, max, total= 0;
	int		is_last;
	int		status;
	int     nBytesWritten   = 0;
	
	while(1)
	{
		//udelay(250);
		//udelay(5);
		if(req){
			buf = req->req.buf+ req->req.actual;
		}
		if( ( count = USB_ReadEP(ep_name_to_index(ep->ep.name),buf , BULK_TRANS_SIZE ) ) <= 0 )
			return -1;
		else
			req->req.actual += count;
        if (req->req.length == req->req.actual || count < BULK_TRANS_SIZE ) /*Modify to handle control ep's as well */
		{
		//	printk("%s: read . Calling Done.\n", __FUNCTION__);
			
		        done (ep, req, 0);
			return 0;
		}
   }
	//return -1;
	return 0;
}

/* returns 0 on success, else negative errno */
static int kick_dma (struct dm320_ep *ep, struct dm320_request *req)
{
	return -1;
}


int rx_pending = 0 ;


int rx_pending = 0 ;

static int
dm320_queue (struct usb_ep *_ep, struct usb_request *_req, int gfp_flags)
{
	struct dm320_request	*req;
	struct dm320_ep	*ep;
	struct dm320		*dev;
	unsigned long		flags;
//	spin_lock_irqsave (&dev->lock, flags);
	int			status = -1;
	int ep_num;
	u8			s;
	
	ep = container_of (_ep, struct dm320_ep, ep);
	dev = ep->dev;
	req = container_of (_req, struct dm320_request, req);
	if (!_req || !_req->complete || !_req->buf)
		return -EINVAL;
	if(!_ep)
		return -EINVAL;
	if (!dev->driver || dev->gadget.speed == USB_SPEED_UNKNOWN)
		return -ESHUTDOWN;
	_req->status = -EINPROGRESS;
	_req->actual = 0;
	
	spin_lock_irqsave (&dev->lock, flags);
	local_irq_disable();
	//printk("\n\n %s: queued the request of length %d to ep: %s\n", __FUNCTION__,_req->length, ep->ep.name);
	/* if list is empty */
	if(_req->length==0 && (ep->is_in || ep_num  == 3) ){
		printk("NULL PACKET SENDING\n");
    		writeb( readb( USB_PER_TXCSR1 ) | USB_TXCSR1_TXPKTRDY, USB_PER_TXCSR1 );
		list_add_tail (&req->queue, &ep->queue);
		goto done;
	}

	if (list_empty (&ep->queue) && !ep->stopped) {
		/* maybe there's no control data, just status ack */
		if (ep->num == 0 && _req->length == 0) { /* No dta to transfer */
			done (ep, req, 0);
			printk("%s: %s status ack\n",__FUNCTION__, ep->ep.name);
			goto done;
		}
		/* try dma first */
		status = kick_dma (ep, req);
		if (status < 0) {
			/* dma failed (most likely in use by another endpoint) 
			 * fallback to pio
			 */
			status = 0;
			ep_num = ep_name_to_index(ep->ep.name);
		
			if((ep->is_in)|| ep_num  == 3)
				status = write_fifo (ep, req);
			else 
			{
				if(rx_pending)
				{
					//printk("%s: rx pending interrupt\n", __FUNCTION__);
					status = read_fifo (ep, req);   /*Vishal: read from FIFO into request struct */		
					rx_pending = 0;
				}
				else
					status = -1;
			}
			//	status = -1;   /*Vishal: read from FIFO into request struct */
		}
	}
	/*Add to queue of pertinent endpoint */
	if (status != 0)
		list_add_tail (&req->queue, &ep->queue);
done:
	local_irq_enable();
	spin_unlock_irqrestore (&dev->lock, flags);
	return 0;
}



/* dequeue ALL requests */
static void nuke (struct dm320_ep *ep)
{
	struct dm320_request	*req;
	/* called with spinlock held */
//	ep->stopped = 1;
	while (!list_empty (&ep->queue)) {
		req = list_entry (ep->queue.next,struct dm320_request,queue);
		done (ep, req, -ESHUTDOWN);
	}

}



/* dequeue JUST ONE request */
static int dm320_dequeue (struct usb_ep *_ep, struct usb_request *_req)
{
	struct dm320_ep *ep;
	struct dm320		*dev;
	dev = the_controller;
    if (!_ep)
        return 0;
    ep = container_of (_ep, struct dm320_ep, ep);
	printk("%s: dm320  API called\n", __FUNCTION__);
    spin_lock (&dev->lock);
	nuke(ep);	
    spin_unlock (&dev->lock);
	return 0;
}

//#define PIPE_STALL 1
static int
dm320_set_halt (struct usb_ep *_ep, int value)
{

	struct dm320_ep	*ep;
	unsigned long		flags;
	int			retval = 0;
#ifdef  PIPE_STALL
	printk("%s: Non functional dm320  API called\n", __FUNCTION__);
	writeb(EP_BULK_IN_NUM, USB_INDEX );
	writeb(readb(USB_PER_TXCSR1)|USB_TXCSR1_SENDST,USB_PER_TXCSR1);
	printk("Stalled BULK-IN Endpoint\n");
	return 0;
#endif
	return -1;
}

static int
dm320_fifo_status (struct usb_ep *_ep)
{
	printk("%s: Non functional dm320  API called\n", __FUNCTION__);
}

static void
dm320_fifo_flush (struct usb_ep *_ep)
{
	//printk("dm320: Flushing H/W FIFO's\n");
	flushEPFIFO( ep_name_to_index(_ep->name) );
}

static void
stop_activity (struct dm320 *dev, struct usb_gadget_driver *driver)
{
	printk("%s: Non functional dm320 USB gadget API\n", __FUNCTION__);
}


static int dm320_disable (struct usb_ep *_ep)
{
	struct dm320_ep	*ep;
	unsigned long		flags;
	ep = container_of (_ep, struct dm320_ep, ep);
	if (!_ep || !ep->desc || _ep->name == ep0name)
		return -EINVAL;
	spin_lock_irqsave (&ep->dev->lock, flags);
	nuke (ep);
	ep_reset (ep);
	VDEBUG (ep->dev, "disabled %s\n", _ep->name);
	spin_unlock_irqrestore (&ep->dev->lock, flags);
	return 0;
}


static struct dm320	*the_controller;
static void usb_reinit (struct dm320 *dev)
{
	int	tmp;
    for (tmp = 0; tmp < NUM_OF_ENDPOINTS + 1; tmp++) {
        struct dm320_ep   *ep = &dev->ep[tmp];

        ep->ep.name = ep_name [tmp];
        ep->dev = dev;
        ep->num = tmp;
        if (tmp > 0 && tmp <=  NUM_OF_ENDPOINTS + 1) {
            ep->fifo_size = EP0_FIFO_SIZE;
        } else
   	        ep->fifo_size = EP0_FIFO_SIZE;
	     ep_reset (ep);
    }
    dev->ep [0].ep.maxpacket = EP0_FIFO_SIZE;     /* Vishal: CHANGE /8 handled this statically. Not good*/
    //Vishal : dev->ep [1].ep.maxpacket = 8 ; /* Bulk  IN  endpoint */
    dev->ep [1].ep.maxpacket = BULK_TRANS_SIZE ; /* Bulk  IN  endpoint */
	dev->ep [1].is_in = 1;
    //Vishal: dev->ep [2].ep.maxpacket = 8;  /* Bulk  OUT endpoint */
    dev->ep [2].ep.maxpacket = BULK_TRANS_SIZE;  /* Bulk  OUT endpoint */
	dev->ep [2].is_in = 0;
    dev->ep [3].ep.maxpacket = INTR_TRANS_SIZE;  /* INTR IN  endpoint */

    dev->gadget.ep0 = &dev->ep [0].ep;
    dev->ep [0].stopped = 0;
	dev->ep[0].pending = dev->ep[1].pending = dev->ep[2].pending = dev->ep[2].pending= 0;
    INIT_LIST_HEAD (&dev->gadget.ep0->ep_list);
}

static void ep0_start (struct dm320 *dev)
{
	dm320_reset_ep(0);
	writeb( USB_EP0, USB_INTRTX1E );		// enable endpoint EP0 RX/TX )
	writeb( USB_RESET | USB_RESUME | USB_SUSPEND | USB_SESSREQ | USB_SOF , USB_INTRUSBE );		// Peripheral mode & 'B' type( slave )	

}

static void ep_reset (struct dm320_ep *ep)
{
	ep->desc = NULL;
	INIT_LIST_HEAD (&ep->queue);
	ep->ep.maxpacket = ~0;
	ep->ep.ops = &dm320_ep_ops;
	dm320_reset_ep(ep);
}


static struct usb_request *
dm320_alloc_request (struct usb_ep *_ep, int gfp_flags)
{
    struct dm320_ep *ep;
    struct dm320_request    *req;
    if (!_ep)
        return NULL;
    ep = container_of (_ep, struct dm320_ep, ep);
    req = kmalloc (sizeof *req, gfp_flags);
    if (!req)
	      return NULL;
    memset (req, 0, sizeof *req);
    req->req.dma = DMA_ADDR_INVALID;
    INIT_LIST_HEAD (&req->queue);
    return &req->req;

}

static void
dm320_free_request (struct usb_ep *_ep, struct usb_request *_req)
{

    struct dm320_ep *ep;
    struct dm320_request    *req;
    ep = container_of (_ep, struct dm320_ep, ep);
    if (!_ep || !_req)
        return;
    req = container_of (_req, struct dm320_request, req);
    kfree (req);
	
}


static void *
dm320_alloc_buffer (struct usb_ep *_ep,unsigned bytes,dma_addr_t *dma,int gfp_flags)
{
    void            *retval;
    struct dm320_ep *ep;

    ep = container_of (_ep, struct dm320_ep, ep);
    if (!_ep)
        return NULL;
    retval = kmalloc (bytes, gfp_flags);
    return retval;
}

static void
dm320_free_buffer(struct usb_ep *_ep,void *buf,dma_addr_t dma,unsigned bytes)
{
    kfree (buf);
}


static struct usb_ep_ops dm320_ep_ops = {
	.enable		= dm320_enable,
	.disable	= dm320_disable,
	
	.alloc_request	= dm320_alloc_request,
	.free_request	= dm320_free_request,

	.alloc_buffer	= dm320_alloc_buffer,
	.free_buffer	= dm320_free_buffer,

	.queue		= dm320_queue,
	.dequeue	= dm320_dequeue,

	.set_halt	= dm320_set_halt,
	.fifo_status	= dm320_fifo_status,
	.fifo_flush	= dm320_fifo_flush,
};

/* when a driver is successfully registered, it will receive
 * control requests including set_configuration(), which enables
 * non-control requests.  then usb traffic follows until a
 * disconnect is reported.  then a host may connect again, or
 * the driver might get unbound.
 */
static int usb_gadget_register_driver (struct usb_gadget_driver *driver)
{
	struct usb_ctrlrequest  r;
	int			retval = -22; 
	unsigned		i;
	if (!driver
			|| (driver->speed != USB_SPEED_FULL)
			|| !driver->bind
			|| !driver->unbind
			|| !driver->setup) {
		return -EINVAL;
	}
	if (!the_controller)
		return -ENODEV;
	if (the_controller->driver)
		return -EBUSY;
	for (i = 0; i < 4; i++)
		the_controller->ep [i].irqs = 0;
	/* hook up the driver ... */
	the_controller->softconnect = 1;
	driver->driver.bus = NULL;	
	the_controller->driver = driver;
	the_controller->gadget.dev.driver = &driver->driver;
	retval = driver->bind (&the_controller->gadget);

	if (retval) {
		DEBUG (the_controller, "bind to driver %s --> %d\n",
				driver->driver.name, retval);
		the_controller->driver = NULL;
		return retval;
	}
	/* ... then enable host detection and ep0; and we're ready
	 * for set_configuration as well as eventual disconnect.
	 */
	ep0_start (the_controller);
	return 0;
}
EXPORT_SYMBOL (usb_gadget_register_driver);



static int usb_gadget_unregister_driver (struct usb_gadget_driver *driver)
{
	struct dm320	*dev = the_controller;
	unsigned long	flags;
	if (!dev)
		return -ENODEV;
	if (!driver || driver != dev->driver)
		return -EINVAL;

	spin_lock_irqsave (&dev->lock, flags);
	stop_activity (dev, driver);
	spin_unlock_irqrestore (&dev->lock, flags);
	//dm320_pullup (&dev->gadget, 0);
	driver->unbind (&dev->gadget);
	dev->driver = NULL;
	printk("unregistered driver '%s'\n", driver->driver.name);
	return 0;
}
EXPORT_SYMBOL (usb_gadget_unregister_driver);

static int dm320_get_frame (struct usb_gadget *_gadget)
{
	struct dm320		*dev;
	unsigned long		flags;
	u16			retval;

	if (!_gadget)
		return -ENODEV;
	dev = container_of (_gadget, struct dm320, gadget);
	spin_lock_irqsave (&dev->lock, flags);
	retval = readb(USB_FRAME2) << 8;  /*read frame reg: 16bit value */
	retval |= readb(USB_FRAME1);
	spin_unlock_irqrestore (&dev->lock, flags);
	return retval;
}

static int dm320_wakeup (struct usb_gadget *_gadget)
{
	struct dm320		*dev;
	u8			tmp;
	unsigned long		flags;
	if (!_gadget)
		return 0;
	dev = container_of (_gadget, struct dm320, gadget);
	spin_lock_irqsave (&dev->lock, flags);
	writeb (USB_POWER_RESUME,USB_POWER);
	spin_unlock_irqrestore (&dev->lock, flags);
	return 0;
}



static int dm320_set_selfpowered (struct usb_gadget *_gadget, int value)
{
	struct dm320		*dev;
	if (!_gadget)
		return -ENODEV;
	dev = container_of (_gadget, struct dm320, gadget);
	dev->is_selfpowered = value;
	return 0;
}
static int dm320_pullup (struct usb_gadget *_gadget, int is_on)
{
	struct dm320		*dev;
	unsigned long		flags;
	if (!_gadget)
		return -ENODEV;
	dev = container_of (_gadget, struct dm320, gadget);
	spin_lock_irqsave (&dev->lock, flags);
	dev->softconnect = (is_on != 0);
	if (is_on)
	{
		/* Vishal: Code to enable detectiong of plug/unplug */
		gio_set_bitset( GIO_DP_PULLUP );		// set D+ pullup
	}
	else
	{
		gio_set_bitclr( GIO_DP_PULLUP );		// clear D+ pullup
		/* Vishal: Code to enable detectiong of plug/unplug */
	}
	spin_unlock_irqrestore (&dev->lock, flags);
	return 0;
}

static const struct usb_gadget_ops dm320_ops = {
	.get_frame		= dm320_get_frame,
	.wakeup			= dm320_wakeup,
	.set_selfpowered	= dm320_set_selfpowered,
};


static struct dm320_ep *
get_ep_by_addr (struct dm320 *dev, u16 wIndex)
{
	struct dm320_ep	*ep;

	if ((wIndex & USB_ENDPOINT_NUMBER_MASK) == 0)
		return &dev->ep [0];
	
	list_for_each_entry (ep, &dev->gadget.ep_list, ep.ep_list) {
		u8	bEndpointAddress;

		if (!ep->desc)
			continue;
		bEndpointAddress = ep->desc->bEndpointAddress;
		if ((wIndex ^ bEndpointAddress) & USB_DIR_IN)
			continue;
		if ((wIndex & 0x0f) == (bEndpointAddress & 0x0f))
			return ep;
	}
	return NULL;
}


static int    dm320_read_intr_regs( void )
{
    unsigned int    nIntrTx     = readb( USB_INTRTX1 ) & 0xFF;
    unsigned int    nIntrRx     = readb( USB_INTRRX1 ) & 0xFF;
    unsigned int    nIntrUSB    = readb( USB_INTRUSB ) & 0xFF;
    return( ( nIntrTx << 12 ) + ( ( nIntrRx >> 1 ) << 8 ) + nIntrUSB );
}

static int     dm320_get_intr_by_priority( int iInterruptFlags )
{
    if( ( iInterruptFlags & USB_CONNECTED       ) != 0 )        return  USB_CONNECTED;
    if( ( iInterruptFlags & USB_DISCONNECTED    ) != 0 )        return  USB_DISCONNECTED;
    if( ( iInterruptFlags & USB_RESET           ) != 0 )        return  USB_RESET;              // reset is first for device
    if( ( iInterruptFlags & USB_RESUME          ) != 0 )        return  USB_RESUME;
    if( ( iInterruptFlags & USB_SESSREQ         ) != 0 )        return  USB_SESSREQ;
    if( ( iInterruptFlags & USB_VBUSERR         ) != 0 )        return  USB_VBUSERR;
    if( ( iInterruptFlags & USB_SOF             ) != 0 )        return  USB_SOF;
    if( ( iInterruptFlags & USB_SUSPEND         ) != 0 )        return  USB_SUSPEND;
    if( ( iInterruptFlags & USB_CONTROL         ) != 0 )        return  USB_CONTROL;
    if( ( iInterruptFlags & USB_RXFIFO          ) != 0 )        return  USB_RXFIFO;
    if( ( iInterruptFlags & USB_TXFIFO          ) != 0 )        return  USB_TXFIFO;

    return USB_NO_INTERRUPT;
}

static uint8_t     dm320_get_peripheral_addr( struct dm320 *dev )
{
    return dev->peripheral_addr;
}

static void    dm320_set_addr_reg( unsigned char ucAddress )
{
	core_debug("dm320: %s:writing new address to the device\n", __FUNCTION__);	
    writeb( ucAddress, USB_FADDR );
}

static void    dm320_set_peripheral_addr( struct dm320 *dev, unsigned char ucNewAddress )  //USB_SetPeripheralAddr
{
    dev->peripheral_addr  = ucNewAddress;
}




static void  dm320_reset_service( )
{
	printk("dm320: %s\n", __FUNCTION__);
	flushEPFIFO(0);
	flushEPFIFO(1);
	flushEPFIFO(2);
	flushEPFIFO(3);
	return;
}



static void dm320_control_service(struct dm320 *dev)
{
	struct usb_ctrlrequest  r;
    struct dm320_ep	*ep;
	u8			num, scratch;
	int			tmp = 0;
	struct dm320_request		*req;
	/* starting a control request? */
	if (dev->gadget.speed == USB_SPEED_UNKNOWN) {
			dev->gadget.speed = USB_SPEED_FULL;
			DEBUG (dev, "%s speed\n",(dev->gadget.speed == USB_SPEED_HIGH)? "high" : "full");
		}
	ep = &dev->ep[0];
	ep->irqs++;
		/* make sure any leftover interrupt state is cleared */
	while (!list_empty (&ep->queue)) {
			req = list_entry (ep->queue.next,
					struct dm320_request, queue);
			done (ep, req, (req->req.actual == req->req.length)? 0 : -EPROTO);
	}

	ep->stopped = 0;
	dev->protocol_stall = 0;
	if( USB_ReadEP( USB_EP0_SELECT, &r, sizeof(struct usb_ctrlrequest ) ) <= 0 )
        return;

	/* watch control traffic at the token level, and force
	 * synchronization before letting the status phase happen.
	 */
	ep->is_in = (r.bRequestType & USB_DIR_IN) != 0;
	if ((r.bRequestType & USB_TYPE_MASK) != USB_TYPE_STANDARD)
	{
		dm320_set_CSR0_reg( USB_CSR0_CLRRXRDY);
		goto delegate;
	}
	
	switch (r.bRequest) {
		case USB_REQ_GET_STATUS: {

			struct dm320_ep	*e;
			u16			status = 0;
			dm320_set_CSR0_reg( USB_CSR0_CLRRXRDY | USB_CSR0_DATAEND );
			if ((r.bRequestType & USB_RECIP_MASK) 
					== USB_RECIP_ENDPOINT) {
				if ((e = get_ep_by_addr (dev, r.wIndex)) == 0
						|| r.wLength > 2)
					goto do_stall;
				set_fifo_bytecount (&dev->ep [0], 0);
				allow_status (ep);
				VDEBUG (dev, "%s stat %02x\n", ep->ep.name,
						status);
				goto next_endpoints;
			} else if ((r.bRequestType & USB_RECIP_MASK)
					== USB_RECIP_DEVICE) {
				if (r.wLength > 2)
					goto do_stall;
				if (dev->is_selfpowered)
					status = (1 << USB_DEVICE_SELF_POWERED);
				set_fifo_bytecount (&dev->ep [0], 0);
				allow_status (ep);
				VDEBUG (dev, "device stat %02x\n", status);
				goto next_endpoints;
			} else if ((r.bRequestType & USB_RECIP_MASK)
					== USB_RECIP_INTERFACE) {
				if (r.wLength > 2)
					goto do_stall;
				set_fifo_bytecount (&dev->ep [0], 0);
				allow_status (ep);
				VDEBUG (dev, "interface status %02x\n", status);
				goto next_endpoints;
			}
		}
		case USB_REQ_CLEAR_FEATURE: {
			
			struct dm320_ep	*e;
			dm320_set_CSR0_reg( USB_CSR0_CLRRXRDY | USB_CSR0_DATAEND );
			if (r.bRequestType != USB_RECIP_ENDPOINT)
				goto delegate;
			if (r.wValue != USB_ENDPOINT_HALT
					|| r.wLength != 0)
				goto do_stall;
			if ((e = get_ep_by_addr (dev, r.wIndex)) == 0)
				goto do_stall;
			clear_halt (e);
			allow_status (ep);
			VDEBUG (dev, "%s clear halt\n", ep->ep.name);	
			goto next_endpoints;
			}
		case USB_REQ_SET_FEATURE: {
			
			struct dm320_ep	*e;
			dm320_set_CSR0_reg( USB_CSR0_CLRRXRDY | USB_CSR0_DATAEND );	
			if (r.bRequestType == USB_RECIP_DEVICE) {
				allow_status (ep);
				VDEBUG (dev, "test mode: %d\n", r.wIndex);
				goto next_endpoints;
			} else if (r.bRequestType != USB_RECIP_ENDPOINT)
				goto delegate;
			if (r.wValue != USB_ENDPOINT_HALT
					|| r.wLength != 0)
				goto do_stall;
			if ((e = get_ep_by_addr (dev, r.wIndex)) == 0)
				goto do_stall;
			set_halt (e);
			allow_status (ep);
			VDEBUG (dev, "%s set halt\n", ep->ep.name);
			goto next_endpoints;
			}
		case USB_REQ_SET_ADDRESS: {
			dm320_set_CSR0_reg( USB_CSR0_CLRRXRDY | USB_CSR0_DATAEND );
			dm320_set_peripheral_addr(dev,r.wValue & 0xff);
			allow_status (ep);
			break;
			}
		case USB_REQ_GET_DESCRIPTOR: {
			dm320_set_CSR0_reg( USB_CSR0_CLRRXRDY );
			goto delegate;
			}

		case USB_REQ_SET_DESCRIPTOR: {
			dm320_set_CSR0_reg( USB_CSR0_CLRRXRDY );
			goto delegate;
			}
		case USB_REQ_GET_CONFIGURATION: {
			dm320_set_CSR0_reg( USB_CSR0_CLRRXRDY | USB_CSR0_DATAEND);
			goto delegate;
			}
		case USB_REQ_SET_CONFIGURATION: {
			dm320_set_CSR0_reg( USB_CSR0_CLRRXRDY | USB_CSR0_DATAEND);
			goto delegate;
			}
		case USB_REQ_GET_INTERFACE: {
			dm320_set_CSR0_reg( USB_CSR0_CLRRXRDY | USB_CSR0_DATAEND);
			goto delegate;
			}
		case USB_REQ_SET_INTERFACE: {
			dm320_set_CSR0_reg( USB_CSR0_CLRRXRDY | USB_CSR0_DATAEND);
			goto delegate;
			}
		default:
			 dm320_set_CSR0_reg( USB_CSR0_CLRRXRDY | USB_CSR0_SENDST );
			 break;
delegate:
			//spin_unlock (&dev->lock);
			if(dev)
			if(&dev->gadget!= NULL && dev->driver!=NULL)
				tmp = dev->driver->setup (&dev->gadget, &r);
			else
				printk("dm320: Gadget device is NULL!!\n");
			//spin_lock (&dev->lock);
		}
		/* stall ep0 on error */
		if (tmp < 0) {
do_stall:
			dev->protocol_stall = 1;
		}
next_endpoints:
	return;
}

static int  dm320_rx_service( struct dm320 *dev)
{
 
    struct dm320_ep *ep;
	int count;
	int status; 
    u8          num, scratch;
    int         tmp = 0;
    struct dm320_request        *req;
	struct usb_ep *_ep;
	u8		*buf;
	int status;
    ep = &dev->ep[EP_BULK_OUT_NUM];
    ep->irqs++;
	if (!list_empty (&ep->queue))
	{
		req = list_entry (ep->queue.next,
			struct dm320_request, queue);
	}
	else
	{
		//printk("%s: Pending interrupt\n", __FUNCTION__);
		rx_pending = 1 ; 
		return 0;
	}
	read_fifo(ep, req);
	return 1;
}

static int  dm320_tx_service(struct dm320 *dev )
{
    struct dm320_ep *ep;
	int count;
	int status; 
    u8          num, scratch;
    int         tmp = 0;
    struct dm320_request        *req;
	struct usb_ep *_ep;
	u8		*buf;

#ifdef PIPE_STALL
	writeb(EP_BULK_IN_NUM, USB_INDEX );
	if( readb(USB_PER_TXCSR1) &  USB_TXCSR1_SENTST)
	{
		writeb(readb(USB_PER_TXCSR1)& ~USB_TXCSR1_SENTST,USB_PER_TXCSR1);
		writeb(readb(USB_PER_TXCSR1)& ~USB_TXCSR1_SENDST,USB_PER_TXCSR1);
	 	printk("Bulk IN peipe stalled, clearing SENTST\n");
	}
#endif
    if (dev->gadget.speed == USB_SPEED_UNKNOWN) {
            dev->gadget.speed = USB_SPEED_FULL;
            DEBUG (dev, "%s speed\n",(dev->gadget.speed == USB_SPEED_HIGH)? "high" : "full");
        }
    ep = &dev->ep[EP_BULK_IN_NUM];
    ep->irqs++;
	if (!list_empty (&ep->queue))
		req = list_entry (ep->queue.next,
			struct dm320_request, queue);
	else
		return 0;
	write_fifo(ep, req);
	return 1;
}

static void dm320_disconnect_service( )
{
	return;
}
static void dm320_suspend_service( )
{
 
	return;
}


void do_pending_transfers()
{
#if 0
	 writeb( iEndpoint, USB_INDEX );
         writeb( readb( USB_PER_RXCSR1 ) & ~(USB_RXCSR1_RXPKTRDY), USB_PER_RXCSR1 );//manually clear RXPKTRDY bit in PER_RXCSR1 register
#endif

}


void do_pending_transfers()
{
#if 0
	 writeb( iEndpoint, USB_INDEX );
         writeb( readb( USB_PER_RXCSR1 ) & ~(USB_RXCSR1_RXPKTRDY), USB_PER_RXCSR1 );//manually clear RXPKTRDY bit in PER_RXCSR1 register
#endif

}

static irqreturn_t  handle_core_irqs(int irq, void *dev_id, struct pt_regs *regs )
{
	int 					iInterruptFlags,i;
	int     				iInterrupt;
	int     				iCount;
	unsigned char   		ucCSR0;
	struct dm320 			*dev = the_controller;
	unsigned long		flags;
	spin_lock_irqsave (&dev->lock, flags);
	
	iInterruptFlags = dm320_read_intr_regs( );
	do
	{
		iInterrupt = dm320_get_intr_by_priority( iInterruptFlags );
		switch( iInterrupt ) 
		{
			case USB_RESET:				
					printk("Event: RESET Interrupt\n" );
					paddr_status  = 0 ;
#if 0
					if( )
					{
						
					}
#endif
				break;
				
			case USB_SESSREQ:
				break;
				
			case USB_VBUSERR:
				printk("INT VBUSERR\n" );
				break;
				
			case USB_CONNECTED:
				printk( "INT CONNECT\n" );
				break;

			case USB_RESUME:
				printk(  "INT RESUME\n" );
				break;

			case USB_CONTROL:
				writeb( USB_EP0_SELECT, USB_INDEX );
				ucCSR0	= readb( USB_PER_CSR0 );			
				dm320_info( "ucCSR0 : 0x%02X\n", ucCSR0 );
				if( ucCSR0 == 0x00 )		// response complete interrupt 
				{
					char	chNewAddr;
					if(paddr_status  == 0 )
					{
						if( ( chNewAddr = dm320_get_peripheral_addr(dev) ) > 0 )
						{
							dm320_set_addr_reg( chNewAddr & 0xFF );
							 paddr_status  = 1;
						}
						break;
					}
				}
				if( ( ucCSR0 & USB_CSR0_RXPKTRDY ) != 0 )
				{					
			    	dm320_info("dm320:%s: USB_CSR0_RXPKTRDY set\n", __FUNCTION__);	
					iCount = readb( USB_COUNT0 );			// number of received bytes
					dm320_control_service(dev);
				}
				else if( ( ucCSR0 & USB_CSR0_SENTST ) != 0 )
				{
					dm320_info( KERN_INFO "INT TXFIFO0 stall.[%04x].\n", ucCSR0 ); 
					writeb( 0, USB_PER_CSR0 );		
				}
				else if( ( ucCSR0 & USB_CSR0_SETEND ) != 0 )
				{
					dm320_info( KERN_INFO "INT TXFIFO0 setend from HOST[%04x].\n", ucCSR0 );
					*(unsigned char *)USB_PER_CSR0 = USB_CSR0_CLRSETEND;
				}
				else if( ( ucCSR0 & USB_CSR0_TXPKTRDY ) != 0 )
				{
					dm320_info( KERN_INFO "INT TXPKTRDY\n" );
					// Nothing to do now...
				}
				else
				{
					// ----- FIXME -----
					// Now ignore these unknown interrupts...
					dm320_set_CSR0_reg( USB_CSR0_CLRRXRDY | USB_CSR0_DATAEND ); // Vishal: added this$#$
					//printk( KERN_INFO "Attention: Unknown USB Core Interrupt encountered!\n" );
				}
				
				break;
            case USB_RXFIFO:
				//	printk("%s: USB_RXFIFO interrupt\n", __FUNCTION__);
                    dm320_rx_service(dev);
                    break;


            case USB_TXFIFO:
                    //if(dm320_tx_service(dev))
		    //printk ("USB_TXFIFO\n");
                    	dm320_tx_service(dev);
                break;


			case USB_SOF:
			      do_pending_transfers();	
//	      printk("%s: SOF \n", __FUNCTION__);
				  break;	

			case USB_DISCONNECTED:
				printk("dm320: DISCONNECT Intr.\n" );
				dm320_disconnect_service( );
				break;

			case USB_SUSPEND:
				printk("dm320: SUSPEND Intr.\n" );
					dm320_suspend_service( );
				break;

			default:
				//printk("UNKNOWN USB core interrupt!\n" );
				break;
		}

		iInterruptFlags = iInterruptFlags & ~iInterrupt;
	
	} 	while( iInterruptFlags != USB_NO_INTERRUPT );
	
	spin_unlock_irqrestore(&dev->lock, flags);
	return IRQ_HANDLED;
}


static int     dm320_connection_status( void )
{
    unsigned short      usGIO;
	mdelay(50);
    usGIO   = inw( IO_GIO_BITSET0 );
    if( ( usGIO & ( 1 << GIO_USB_ATTACH ) ) == 0 )  // check if USB device was connected or disconnected
    {
		if(the_controller->last_event == HOST_DISCONNECT)
			return -1;
		else
		{
			the_controller->last_event = HOST_DISCONNECT;
			return 0;                                           // 0 : DISCONNECTED
		}
    }
	else /* Connect Event */
	{	
		if(the_controller->last_event == HOST_CONNECT)
			return -1;
		else
		{
			the_controller->last_event = HOST_CONNECT;
			return 1;                                           // 0 : DISCONNECTED
		}
	}
	return -1;
}

static irqreturn_t  handle_attach_irqs (int irq, void *_dev, struct pt_regs * r)
{
	u8			tmp, mask;
	struct dm320		*dev = _dev;
	int i, connect;
//	spin_lock (&dev->lock);
	connect=dm320_connection_status();
#if 0
	if(connect == HOST_CONNECT )
		printk("%s: Device Connected\n", __FUNCTION__);
	else if (connect == HOST_DISCONNECT)
		printk("%s: Device DisConnected\n", __FUNCTION__);
#endif
    if(connect < 0 )	
   	{
		printk("%s: Debounce detected\n", __FUNCTION__);
		return 0;
	}	
	
	if (connect == HOST_DISCONNECT) {
			stop_activity(dev, dev->driver);   /* Vishal : Stop activity...haha!!*/
			printk("%s: Disconnect the USB device\n", __FUNCTION__);
			dev->gadget.speed = USB_SPEED_UNKNOWN;
			outw( inw( IO_CLK_LPCTL1 ) | 0x0010, IO_CLK_LPCTL1 ); 
			printk("Disconnect: USB device\n");
//			spin_unlock (&dev->lock);
			if((readb(USB_PER_TXCSR1) & USB_TXCSR1_FIFOEMP) )
			{
				writeb(USB_TXCSR1_FLFIFO , USB_PER_TXCSR1);
		 		mdelay(5);
			}
			return IRQ_HANDLED;
	}
	if(the_controller)
	{
		printk("dm320.c: Controller exists. Initializing\n");
	    usb_reset (the_controller);
		outw(inw(IO_INTC_FISEL0) & 0x7F, IO_INTC_FISEL0);
		outw(inw(IO_INTC_EINT0) |  0x80, IO_INTC_EINT0);
		dm320_reconnect_init(dev);
		//usb_reinit (the_controller);
       //writeb( USB_DEVCTL_SESSREQ, USB_DEVCTL );  // For 'B' device initiate session request protocol
	}
//	spin_unlock (&dev->lock);

	printk("dm320 : Connect event handled\n");
	return IRQ_HANDLED;

}

static void gadget_release (struct device *_dev)
{
	struct dm320	*dev = dev_get_drvdata (_dev);
	printk("%s: released\n", __FUNCTION__);
	kfree (dev);
}


static int dm320_remove (struct dm320 *dev)
{
	/* clean up resources allocated during probe() */
	dev = the_controller;	
    printk("%s: Disconnect the USB device\n", __FUNCTION__);
    dev->gadget.speed = USB_SPEED_UNKNOWN;
    outw( inw( IO_CLK_LPCTL1 ) | 0x0010, IO_CLK_LPCTL1 );
    printk("Disconnect: USB device\n");
	if (dev->attach_irq)
	   	free_irq( IRQ_USB_ATTACH, handle_attach_irqs );
	if (dev->core_irq)
    	free_irq( IRQ_USB_CORE, handle_core_irqs );
	gio_disable_irq( GIO_USB_ATTACH );
    unrequest_gio( GIO_USB_ATTACH );
    unrequest_gio( GIO_DP_PULLUP );
    stop_activity (dev, dev->driver);   /* Vishal : Stop activity...!!*/
	device_unregister (&dev->gadget.dev);
	if (dev->driver) {
		/* should have been done already by driver model core */
		printk("WARNING: remove, driver '%s' is still registered\n",
				dev->driver->driver.name);
		usb_gadget_unregister_driver (dev->driver);
	}
	kfree(the_controller);
	the_controller = NULL;	
	return 0;
}


static void dm320_reset_ep(unsigned int index)  /*  */
{
	writeb(index,USB_INDEX );            
    writeb( USB_CSR0_CLRSETEND | USB_CSR0_CLRRXRDY, USB_PER_CSR0 );
    writeb( USB_CSR2_FLFIFO, USB_CSR2 );            // flush FIFO
    writeb( USB_CSR2_FLFIFO, USB_CSR2 );            // flush FIFO for double buffer
}



static void dm320_setup_ep_regs(struct dm320 *dev)   /* Setup Conrol Endpoint, EP1 and EP2 */
{
	int i;
	int		nBufPos;
	writeb( USB_EP0_SELECT, USB_INDEX );            // select EP0
    writeb( USB_CSR0_CLRSETEND | USB_CSR0_CLRRXRDY, USB_PER_CSR0 );
    writeb( USB_CSR2_FLFIFO, USB_CSR2 );            // flush FIFO
    writeb( USB_CSR2_FLFIFO, USB_CSR2 );            // flush FIFO for double buffer

    writeb( nBufPos & 0xFF,     USB_TXFIFO1 );      // TX ADDL = 0x00
    writeb( nBufPos & 0xFF,     USB_RXFIFO1 );      // RX ADDL = 0x00
    writeb( USB_TXFIFO2_SZ_64,  USB_TXFIFO2 );      // TX fixed 64 bytes, single buffer, ADDH = 0
    writeb( USB_TXFIFO2_SZ_64,  USB_RXFIFO2 );      // RX fixed 64 bytes, single buffer, ADDH = 0
    writeb( EP0_FIFO_SIZE / 8 ,  USB_TXMAXP );       // max TX packet size = 8 * 8 = 64 bytes
    writeb( EP0_FIFO_SIZE / 8 ,  USB_RXMAXP );       // max RX packet size = 8 * 8 = 64 bytes

    nBufPos = EP0_FIFO_SIZE;                        // move to next buffer position
	/*  Setup  bulk-in, bulk-out, iso-in, iso-out, and intr endpoints *
	    These endpoints will be configured using the stEndpointInfo[] array
		if you need to add a new end poing to change the confiuration 
		for an existing endpoint, you will need to change that array
	*/
    for( i = 0; i < NUM_OF_ENDPOINTS ; i++ )
    {
        writeb( stEndpointInfo[i].fAddr & 0x0F, USB_INDEX );
        if( stEndpointInfo[i].nSize )
        {
            char    fLowAddr    = ( nBufPos / 8 ) & 0xFF;
            char    fHighAddr   = ( nBufPos >= 2048 ) ? 1 : 0;
            char    fPacketSize = 0x00;
            char    fIsDualBuf  = ( stEndpointInfo[i].fBufMode == EP_BUF_SINGLE ) ? 0x00 : 0x10;
            switch( stEndpointInfo[i].nSize )
            {
                case 16:        fPacketSize = USB_TXFIFO2_SZ_16;        break;
                case 32:        fPacketSize = USB_TXFIFO2_SZ_32;        break;
                case 64:        fPacketSize = USB_TXFIFO2_SZ_64;        break;
                case 128:       fPacketSize = USB_TXFIFO2_SZ_128;       break;
                case 256:       fPacketSize = USB_TXFIFO2_SZ_256;       break;
                case 512:       fPacketSize = USB_TXFIFO2_SZ_512;       break;
                case 1024:      fPacketSize = USB_TXFIFO2_SZ_1024;      break;
            }
		    // DM320 USB registers  :   OUT direction
            // USB specification    :   IN direction that mean device-to-host
            if( stEndpointInfo[i].fAddr & 0x80 )
            {
                writeb( USB_TXCSR1_CLRDATTOG | USB_TXCSR1_FLFIFO | USB_TXCSR1_UNDERRUN, USB_PER_TXCSR1 );
                writeb( USB_TXCSR1_FLFIFO, USB_PER_TXCSR1 );    // clear FIFO for double buffer
                writeb( USB_TXCSR2_FRDATTOG | USB_TXCSR2_MODE_TX, USB_TXCSR2 );                       // no DMA sending
                writeb( fLowAddr,                               USB_TXFIFO1 );
                writeb( fHighAddr | fPacketSize | fIsDualBuf,   USB_TXFIFO2 );
                writeb( stEndpointInfo[i].nSize / 8,            USB_TXMAXP );
            }
            // DM320 USB registers  :   IN direction
            // USB specification    :   OUT direction that mean host-to-device
            else
            {
                // clear Data toggle, FIFO, OVERRUN of RXCSR1
                writeb( USB_RXCSR1_CLRDATTOG | USB_RXCSR1_FLFIFO, USB_PER_RXCSR1 );                           
                writeb( USB_RXCSR1_FLFIFO, USB_PER_RXCSR1 );    // clear FIFO for double buffer

                writeb( 0x00, USB_PER_RXCSR2);                                 // no DMA receiving
//              writeb( USB_RXCSR2_AUTOCLR, USB_PER_RXCSR2 );                                 // AUTOCLR for DMA receiving
                writeb( fLowAddr,                               USB_RXFIFO1 );
                writeb( fHighAddr | fPacketSize | fIsDualBuf,   USB_RXFIFO2 );
                Vishal: writeb( stEndpointInfo[i].nSize / 8,            USB_RXMAXP );
            }
            nBufPos += stEndpointInfo[i].nSize;
        }
        else
            writeb( 0x00, USB_RXMAXP );
    }
}




static void dm320_init_per_mode()
{
    outw( inw( IO_CLK_MOD2 ) | 0x0060, IO_CLK_MOD2 );           // 		0x0040 : USB clock enable
	                                                            // 		0x0020 : GIO clock enable
	outw( inw( IO_CLK_LPCTL1 ) & 0xFFEF, IO_CLK_LPCTL1 );       // USB Power down mode off
																// 		0x0010 : Set USB power down mode.
	writel( 0x00000000, AHB_USBCTL );                     		// USB controller in peripheral mode. ( 0x00000008 : host mode )
	writeb( USB_DEVCTL_SESSREQ, USB_DEVCTL );     /* ATTENTION : vishal :for bug fix only!!    **ATTENTION** */
	// Setup USB core registers
	writeb( 0x00, USB_FADDR );									// reset peripheral address register
	writeb( 0x00, USB_POWER );									// reset power control register
}

static void dm320_clear_intrs()
{

	outw( ( 1 << IRQ_USB_DMA  ), IO_INTC_IRQ0 );				// clear USB core interrupt flag
	outw( ( 1 << IRQ_USB_CORE ), IO_INTC_IRQ0 );				// clear USB DMA interrupt flag
	readb( USB_INTRTX1 );                    					// clear TX interrupt flags
	readb( USB_INTRRX1 );                       				// clear RX interrupt flags
	readb( USB_INTRUSB );                      					// clear control interrupt flags
	readb( USBDMA_INTR );
}

static void dm320_enable_intrs()
{

	writeb( ( BULK_OUT_EP << 1 ) | USB_EP0, USB_INTRTX1E );		// enable endpoint transmmit interrupt ( including EP0 RX/TX )
	writeb( ( BULK_IN_EP << 1 ), USB_INTRRX1E );   				// enable endpoint receving interrupt
	writeb( USB_RESET | USB_RESUME | USB_SUSPEND | USB_SESSREQ | USB_SOF , USB_INTRUSBE );		// Peripheral mode & 'B' type( slave )	
}


static void dm320_reconnect_init(struct dm320 *dev)
{
    dev->gadget.speed = USB_SPEED_UNKNOWN;
	printk("device speed is UNKNOWN\n");
    paddr_status  = 0 ;
    dev->peripheral_addr = 0;
}




static void 	dm320_usb_hw_init(struct dm320 *dev )
{
    dm320_init_per_mode();
	dm320_clear_intrs();	
	dm320_enable_intrs();
	dm320_setup_ep_regs(dev);
	dev->peripheral_addr = 0;
	dm320_set_addr_reg(0);
	writeb( USB_EP0_SELECT, USB_INDEX );         	// set default endpoint as EP0
}


static void usb_reset (struct dm320 *dev)
{
	printk("dm320  : %s\n", __FUNCTION__);;
	dev->gadget.speed = USB_SPEED_UNKNOWN;
 	dm320_usb_hw_init(dev );
	printk("device speed is UNKNOWN\n");
	INIT_LIST_HEAD (&dev->gadget.ep_list);
    list_add_tail (&dev->ep [1].ep.ep_list, &dev->gadget.ep_list);
    list_add_tail (&dev->ep [2].ep.ep_list, &dev->gadget.ep_list);
    list_add_tail (&dev->ep [3].ep.ep_list, &dev->gadget.ep_list);

}


static int  dm320_init_gio()
{

	outw( inw( IO_CLK_DIV4 ) | ( ( ( 4 ) - 1 ) << 8 ) | ( ( 1 ) - 1 ), IO_CLK_DIV4 );
    if(request_gio( GIO_DP_PULLUP )!=0 )            // D+ pullup control GIO
    {
        printk(" dm320  : GIO for D+ Pullup already taken\n");
        return -EBUSY; 
    }
    gio_set_dir( GIO_DP_PULLUP, 0 );        // output direction
    gio_set_bitset( GIO_DP_PULLUP );        // set D+ pullup
    if(request_gio( GIO_USB_ATTACH )!=0)
    {
        printk(" dm320  : USB Attach Interrupt handler GIO Interrupt already taken\n");
        return -EBUSY;
    }
    gio_enable_irq( GIO_USB_ATTACH, GIO_ANY_EDGE );
    outw(inw(IO_GIO_CHAT0) | (1 << GIO_USB_ATTACH),IO_GIO_CHAT0);
	return 0;
}

static int dm320_register_irqs()
{

    if( request_irq( IRQ_USB_ATTACH, handle_attach_irqs, SA_INTERRUPT, "usb_external", handle_attach_irqs )!= 0 )
    {
        printk("dm320 USB: Attach IRQ could not be registered  \n");
        return -EBUSY;
    }
        // Register USB core control interrupt handler
    if(request_irq( IRQ_USB_CORE, handle_core_irqs, SA_INTERRUPT, "usb_core", handle_core_irqs )!=0 )
    {
        printk("dm320 USB: Core IRQ could not be registered\n");
        return -EBUSY;
    }
	return 0;
}

static int dm320_probe (void)
{
	int			retval;
	if (the_controller!= NULL) {  		/* Check if the driver for the DM320 device mode is already running */
		printk(" dm320: driver already running.\n");
		return -EBUSY;		/* return busy status and exit driver...am i addicted to commenting code? */	
	}
	the_controller = kmalloc (sizeof(struct dm320), SLAB_KERNEL);
	if (the_controller == NULL) {
		printk("dm320: cannot allocate memory for USB function driver!\n");
		retval = -ENOMEM;
		goto done;
	}
	memset (the_controller, 0, sizeof(struct dm320));
	spin_lock_init (&the_controller->lock);
	the_controller->gadget.ops = &dm320_ops;    
	the_controller->gadget.is_dualspeed = 0;  /* The DM320 is a full speed controlelr only!! */
	strcpy (the_controller->gadget.dev.bus_id, "DM320 Gadget");
	the_controller->gadget.dev.release = gadget_release;	
	the_controller->gadget.name = driver_name;
	the_controller->enabled = 1;

	the_controller->chiprev = inw( IO_BUSC_REVR );
	printk("dm320: %s :Board revision : %d.%d\n",__FUNCTION__, the_controller->chiprev >> 4, the_controller->chiprev & 0xF );
	outw( inw( IO_CLK_MOD2 ) | 0x0060, IO_CLK_MOD2 );           // Enable USB clock & GIO clock 
 
	if(dm320_init_gio() != 0 )
		goto done;

   	if(dm320_register_irqs() != 0 )
		goto done;
    the_controller->attach_irq = IRQ_USB_ATTACH;
    the_controller->core_irq = IRQ_USB_CORE;
	usb_reset (the_controller);
	usb_reinit (the_controller);
    if( (dm320_connection_status( ) ) > 0 )
	{
#if 0
		usb_reset (the_controller);
        outw(inw(IO_INTC_FISEL0) & 0x7F, IO_INTC_FISEL0);
        outw(inw(IO_INTC_EINT0) |  0x80, IO_INTC_EINT0);
        dm320_reconnect_init(the_controller);
#endif
	}

       writeb( USB_DEVCTL_SESSREQ, USB_DEVCTL );  // For 'B' device initiate session request protocol
	
	INFO (the_controller, "%s\n", driver_desc);
	INFO (the_controller, "core irq %d, attach irq %d, dma irq %d\n",
			the_controller->core_irq, the_controller->attach_irq, the_controller->dma_irq);
	INFO (the_controller, "version: %s\n", DRIVER_VERSION);
	device_register (&the_controller->gadget.dev);	
	return 0;

done:
	if (the_controller)
		dm320_remove (the_controller);
	
}

static int __init init (void)
{
	return dm320_probe ();
}

static void __exit cleanup (void)
{
 	dm320_remove (the_controller);
    
}


module_init (init);
module_exit (cleanup);

MODULE_DESCRIPTION (DRIVER_DESC);
MODULE_AUTHOR ("Ingenient Technologies");
MODULE_LICENSE ("GPL");
