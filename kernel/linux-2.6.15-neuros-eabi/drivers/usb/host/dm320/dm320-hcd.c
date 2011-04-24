/*
 * DM320 HCD (Host Controller Driver) for USB.
 *
 *
 * Periodic scheduling is based on Roman's OHCI code
 *     Copyright (C) 1999 Roman Weissgaerber
 *
 * Heavily rewrriten to work with khubd to support hotplug. 
 *     Copyright (C) 2006 Neuros Technology LLC. mgao@neuros 10-08-2006
 *
 * Small fixes for compatibility with USB wifi dongle.
 *     Copyright (C) 2007-2008 Neuros Technology LLC. nerochiaro@neuros 27-04-2007
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

#undef    VERBOSE
#undef    PACKET_TRACE


#include <linux/config.h>

#ifdef CONFIG_USB_DEBUG
#    define DEBUG
#else
#    undef DEBUG
#endif

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/ioport.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/smp_lock.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/proc_fs.h>
#include <linux/timer.h>
#include <linux/list.h>
#include <linux/interrupt.h>
#include <linux/usb.h>
#include <linux/usb_dm320.h>
#include <linux/platform_device.h>
#include <linux/kthread.h>

#include <asm/io.h>
#include <asm/irq.h>
#include <asm/system.h>
#include <asm/byteorder.h>
#include <asm/arch/gio.h>

#include "../../core/hcd.h"
#include "dm320.h"
#include "dm320_usb.h"
#include "dm320_reg_def.h"
#include "hc_dm320.h"

MODULE_DESCRIPTION("DM320 HS USB Host Controller Driver");
MODULE_LICENSE("GPL");

#define DRIVER_VERSION    "27 Oct. 2006"

#ifndef DEBUG
#    define    STUB_DEBUG_FILE
#endif

/* this doesn't understand urb->iso_frame_desc[], but if you had a driver
 * that just queued one ISO frame per URB then iso transfers "should" work
 * using the normal urb status fields.
 */
#define    DISABLE_ISO

#define USE_VBUS_GIO
#define GIO_USB_POWER 7

// TODO: it doesn't seem to change anything if i change this to a much lower value. are there actual docs for these registers ? --nerochiaro
#define USB_REG_NAK_LIMIT 255

// TODO: Higher values are better for USB1.1, lower for stuff like the WiFi driver. There might be a setting that works for both, this needs tuning --nerochiaro
// #define USB_REG_RX_INTERVAL 0xf0
// #define USB_REG_TX_INTERVAL 0xf0
#define USB_REG_RX_INTERVAL 0x06 
#define USB_REG_TX_INTERVAL 0x01

static int nBufPosIn = 0, nBufPosOut = 0;
static const char hcd_name[] = "DM320-USB-HCD";
static int usb_core_irq_num = -1;
static spinlock_t lock = SPIN_LOCK_UNLOCKED;
static int in_eps[USB_MAX_DEVICE_EP];
static int out_eps[USB_MAX_DEVICE_EP];
static int stalled[USB_MAX_DEVICE_EP];
struct port_t port;
static void* devdescsave = NULL;
static int ep_index[2][USB_MAX_HARDWARE_EP];
static int transfer_started;

static unsigned int get_ctrl_sts_reg(struct dm320* dm320, int ep_index, unsigned int cs_reg, int pid);
static int dm320_setup_ep(struct dm320* dm320, struct urb* urb, struct dm320h_ep* ep);
int thread_wrapper(void* data);

// Rewrite the usb connection STATE machine here. --- MG
enum USB_CONNECTION_STATE
{
    START__,
    CONNECTED__,
    DISCONNECTED__,
    RESET__,
    GOING2IDLE__,
    IDLE__,
};

static int state = START__;
#define USB_PLUG_DEBOUNCE__ 2
static int plug_debounce;
#define USB_IDLE_DEBOUNCE__ 2
static int idle_debounce;
#define GOING2IDLE_STATE() do{ state = GOING2IDLE__; \
                               idle_debounce = USB_IDLE_DEBOUNCE__; \
                             } while(0)

// Manually pack 2 u16 into u32
#define PK2U32(lo, hi) (((*(u16*)(&(hi)))<<16)|(*(u16*)(&(lo))))

enum plugaction
  {
    USB_PLUG__,
    USB_PULL__,
    USB_NOACTION__,
  };
static enum plugaction action = USB_NOACTION__; /* 0:add  1:remove  2:noaction*/
static enum plugaction proc_action = USB_NOACTION__;

// init control variables.
static void init_control_vars(struct dm320 *dm320)
{
    int i;

    spin_lock(&dm320->lock);

    transfer_started = 0;

    for(i = 0; i < USB_MAX_DEVICE_EP; i++)
    {
        out_eps[i] = 0;    
        in_eps[i]  = 0;
        stalled[i] = 0;    
    }

    dm320->getconfigdescFlag = 0;
    dm320->grecvd = 0;
    dm320->getdevdescFlag = 0;
    dm320->numendpts = 0;

    spin_unlock(&dm320->lock);
}

void enable_interrupt(u16 enable)
{
    unsigned long flags;

    if (usb_core_irq_num < 0)
    {
        printk("%s %d: ERROR usb_core_irq_num not set\n", __FILE__, __LINE__); 
        return;
    }

    if (enable & (1 << usb_core_irq_num))
    {
        spin_lock_irqsave(&lock, flags);
        outw(inw(IO_INTC_EINT0) | (1 << usb_core_irq_num), IO_INTC_EINT0);
        spin_unlock_irqrestore(&lock, flags);
    }
}

u16 disable_interrupt(void)
{
    unsigned long flags;
    u16 enable;

    if (usb_core_irq_num < 0)
    {
        printk("%s %d: ERROR usb_core_irq_num not set\n", __FILE__, __LINE__); 
        return 0;
    }

    spin_lock_irqsave(&lock, flags);
    enable = inw(IO_INTC_EINT0);
    outw(enable &~ (1 << usb_core_irq_num), IO_INTC_EINT0);
    spin_unlock_irqrestore(&lock, flags);

    return(enable & (1 << usb_core_irq_num));
}

#if 0
static void dump_usb_regs(void)
{
    int i;
    volatile u8* reg;
    u8 save_index;
    u16 flags;

    flags = disable_interrupt();

    printk("\n\nUSB REGS\n\n");

    reg = (volatile u8*)USB_BASE;
    do{
        printk("%p: %02x\n", reg, *reg);
    } while(++reg < (volatile u8*)USB_TXMAXP);
    printk("\n");

    save_index = readb(USB_INDEX);
    for(i = 0; i < 4; i++)
    {
        writeb(i, USB_INDEX);
        printk("index: %d\n", readb(USB_INDEX));
        reg = (volatile u8*)USB_TXMAXP;
        do{
            printk("%p: %02x\n", reg, *reg);
        } while(++reg < (volatile u8*)USB_FIFO0);
        printk("\n");
    }
    writeb(save_index, USB_INDEX);
    printk("\n");

    reg = (volatile u8*)USB_FIFO0;
    do{
        printk("%p: %02x\n", reg, *reg);
    } while(++reg <= (volatile u8*)USB_FIFO4);
    printk("\n\n");

    enable_interrupt(flags);
}
#endif

void RregShow(void)
{
    char i = 0;

    for(i = 0; i < 0x30; i++)
    {
        DBG("Reg %x = %x\n", i, readb(USB_BASE+i));
    }
}

int get_ep_index(int epnum, int is_out)
{
    int ret_index;

    if (is_out)
    {
        is_out = 1;
    }

    if (epnum == CTRL_EP)
    {
        // control endpoint is always 0
        ret_index = 0;
        goto out;
    }

    for (ret_index = 1; ret_index < USB_MAX_HARDWARE_EP; ret_index++)
    {
        if (!ep_index[is_out][ret_index] ||
            (ep_index[is_out][ret_index] == epnum))
        {
            ep_index[is_out][ret_index] = epnum;
            goto out;
        }
    }

    printk("%s %d unrecoverable error, usb endpoints exceeded %d %d %d\n", 
        __FUNCTION__, __LINE__, ret_index, USB_MAX_HARDWARE_EP, is_out); 
    ret_index = -1;

out:
    return ret_index;
}

int dm320_read_ep(struct dm320* dm320, int ep_index, void* buffer, int size)
{
    int                     ret = -1;
    int                     count;
    volatile unsigned char* pucFIFO;
    unsigned char*          buf,*tmp;
    u16                     flags;

    if ((ep_index < USB_MIN_EP0) || (ep_index > USB_MAX_HARDWARE_EP))
    {
        return ret;
    }

    flags = disable_interrupt();
    spin_lock(&dm320->lock);

    writeb(ep_index, USB_INDEX);

    if (ep_index == USB_MIN_EP0)
    {
        buf  = (unsigned char*)buffer;
        count = readb(USB_COUNT0);
        if (count > size)
            count = size;
    }
    else
    {
        count = (readb(USB_RXCOUNT2)<< 8) |  readb(USB_RXCOUNT1);
        DBG(" %s: bytes available on ep%d : %d \n",__FUNCTION__,ep_index, count);
        if(count > size)
        {
            count = size;
        }
    }
    ret = count;
    buf = (unsigned char*)buffer;
    pucFIFO = (unsigned char*)USB_FIFO0;
    pucFIFO = pucFIFO + (ep_index * 4);
    tmp = buf;    
    while(count > 0)
    {
        *buf++ = *pucFIFO;
        count--;
    }
    if (ret > 0)
    {
        DBG("%s: %d bytes read, from ep%d\n",__FUNCTION__, ret, ep_index);
    }

    spin_unlock(&dm320->lock);
    enable_interrupt(flags);

    return ret;
}

int dm320_write_ep( struct dm320* dm320, int ep_index, void* buffer, int size)
{
    volatile unsigned char*     pucFIFO;
    unsigned char*              buf;
    int                         ret = -1;
    int                         count;
    u16                         flags;

    if((ep_index < USB_MIN_EP0) || (ep_index > USB_MAX_HARDWARE_EP))
    {
        return ret;
    }

    flags = disable_interrupt();
    spin_lock(&dm320->lock);

    writeb(ep_index, USB_INDEX);

    if (ep_index == USB_MIN_EP0)
       count = EP0_FIFO_SIZE;
    else
       count = BULK_TRANS_SIZE;

    if(size < count)
        count = size;

    ret    = count;
    buf  = (unsigned char *)buffer;
    pucFIFO = (unsigned char *)USB_FIFO0;
    pucFIFO = pucFIFO + (ep_index * sizeof(u32));
    DBG("endpoint: %d,address: %d\n",readb(USB_INDEX), readb(USB_FADDR));
    while(count > 0)
    {
        writeb(*buf++, pucFIFO);
        count--;
    }

    spin_unlock(&dm320->lock);
    enable_interrupt(flags);

    return ret;
}

/*-------------------------------------------------------------------------*/

static void port_power(struct dm320* dm320, int is_on)
{
    struct usb_hcd* hcd = dm320_to_hcd(dm320);
    u16                 flags;

    flags = disable_interrupt();
    spin_lock(&dm320->lock);

    /* hub is inactive unless the port is powered */
    if (is_on) 
    {
        if (dm320->port1 & (1 << USB_PORT_FEAT_POWER))
        {
            goto out;
        }

        dm320->port1 = (1 << USB_PORT_FEAT_POWER);
        hcd->self.controller->power.power_state = PMSG_ON;
    } 
    else 
    {
        dm320->port1 = 0;
        hcd->state = HC_STATE_HALT;
        hcd->self.controller->power.power_state = PMSG_SUSPEND;
    }

    if (dm320->board && dm320->board->port_power) 
    {
        /* switch VBUS, at 500mA unless hub power budget gets set */
        DBG("power %s\n", is_on ? "on" : "off");
        dm320->board->port_power(hcd->self.controller, is_on);
    }

    /* reset as thoroughly as we can */
    if (dm320->board && dm320->board->reset)
    {
        dm320->board->reset(hcd->self.controller);
    }
    else 
    {
        mdelay(20);
    }

out:
    spin_unlock(&dm320->lock);
    enable_interrupt(flags);
}

/*-------------------------------------------------------------------------*/

/* This is a PIO-only HCD.  Queueing appends URBs to the endpoint's queue,
 * and may start I/O.  Endpoint queues are scanned during completion irq
 * handlers (one per packet: ACK, NAK, faults, etc) and urb cancellation.
 *
 * Using an external DMA engine to copy a packet at a time could work,
 * though setup/teardown costs may be too big to make it worthwhile.
 */

/* SETUP starts a new control request.  Devices are not allowed to
 * STALL or NAK these; they must cancel any pending control requests.
 */
static void setup_packet(
    struct dm320*       dm320,
    struct dm320h_ep*   ep,
    struct urb*         urb,
    u8                  bank,
    u8                  control)
{
    int                 epnum = 0;
    int                 is_out = 0;
    int                 ep_index = 0;
    u16                 flags;
    u8                  len;

    flags = disable_interrupt();
    spin_lock(&dm320->lock);
    spin_lock(&urb->lock);

    epnum = usb_pipeendpoint(urb->pipe);
    is_out = usb_pipeout(urb->pipe) ? 1 : 0;
    ep_index = get_ep_index(epnum, is_out);
    len = sizeof(struct usb_ctrlrequest);
    dm320_write_ep(dm320, ep_index, urb->setup_packet, len);

    if((((char*)urb->setup_packet)[1] == GET_DESCRIPTOR) && 
       (((char*)urb->setup_packet)[3] == DEVICE))
    {
        dm320->getdevdescFlag=1;
    }
    if((((char *)urb->setup_packet)[1] == GET_DESCRIPTOR) && 
        (((char*)urb->setup_packet)[3] == CONFIGURATION))
    {
        dm320->grecvd = 0;
        dm320->getconfigdescFlag = 1;
    }
    
    /* set the SETPKT, TXPKTRDY bits of HST_CSR0 */
    writeb((USB_HST_CSR0_SETPKT|USB_HST_CSR0_TXPKTRDY),USB_HST_CSR0);
    ep->length = 0;
    PACKET("SETUP qh%p\n", ep);

    spin_unlock(&urb->lock);
    spin_unlock(&dm320->lock);
    enable_interrupt(flags);
}

/* STATUS finishes control requests, often after IN or OUT data packets */
static void status_packet(
    struct dm320*       dm320,
    struct dm320h_ep*   ep,
    struct urb*         urb,
    u8                  bank,
    u8                  control)
{
    int             do_out;
    int             epnum = 0;
    int             ep_index = 0;
    int             is_out = 0;
    u16             flags;

    flags = disable_interrupt();
    spin_lock(&dm320->lock);
    spin_lock(&urb->lock);

    epnum = usb_pipeendpoint(urb->pipe);
    is_out = usb_pipeout(urb->pipe) ? 1 : 0;
    ep_index = get_ep_index(epnum, is_out);

    writeb(ep_index, USB_INDEX);

    do_out = urb->transfer_buffer_length && usb_pipein(urb->pipe);
    /* always data1; sometimes IN */
    if(ep_index == CTRL_EP)
    {
        if(usb_pipeout(urb->pipe)) 
        { 
            N_DBG("%s:%d:%s\n", __FILE__, __LINE__, __FUNCTION__);
            writeb( USB_HST_CSR0_REQPKT|USB_HST_CSR0_STPKT, USB_HST_CSR0);
        }
        else 
        {
            N_DBG("%s:%d:%s\n", __FILE__, __LINE__, __FUNCTION__);
            writeb( ( USB_HST_CSR0_TXPKTRDY | USB_HST_CSR0_STPKT ), USB_HST_CSR0);
        }
    }

    ep->length = 0;
    N_DBG("STATUS%s/%s qh%p, epnum=%d, is_out=%d\n", ep->nak_count ? "/retry" : "",
            do_out ? "out" : "in", ep, epnum, usb_pipeout(urb->pipe));

    PACKET("STATUS%s/%s qh%p\n", ep->nak_count ? "/retry" : "",
            do_out ? "out" : "in", ep);

    spin_unlock(&urb->lock);
    spin_unlock(&dm320->lock);
    enable_interrupt(flags);
}

/* IN packets can be used with any type of endpoint. here we just
 * start the transfer, data from the peripheral may arrive later.
 * urb->iso_frame_desc is currently ignored here...
 */
static void in_packet(
    struct dm320*       dm320,
    struct dm320h_ep*   ep,
    struct urb*         urb,
    u8                  bank,
    u8                  control)
{
    int             epnum = 0;
    int             ep_index = 0;
    int             is_out = 0;
    u16             flags;
    u8              len;

    flags = disable_interrupt();
    spin_lock(&dm320->lock);
    spin_lock(&urb->lock);

    epnum = usb_pipeendpoint(urb->pipe);
    is_out = usb_pipeout(urb->pipe) ? 1 : 0;
    ep_index = get_ep_index(epnum, is_out);

    writeb(ep_index, USB_INDEX);

    /* avoid losing data on overflow */
    len = ep->maxpacket;

    if(ep_index == CTRL_EP) 
    {
        /* set the REQPKT bit of HST_CSR0 */
        if(len)
        {
            writeb( USB_HST_CSR0_REQPKT, USB_HST_CSR0);
        }
        else
        {
            writeb((USB_HST_CSR0_REQPKT|USB_HST_CSR0_STPKT), USB_HST_CSR0);
        }
    }
    else 
    {
        writeb(USB_HST_RXCSR1_REQPKT, USB_HST_RXCSR1);
    }

    ep->length = min((int)len,
            urb->transfer_buffer_length - urb->actual_length);
    
    N_DBG("IN%s/%d qh%p len%d\n", ep->nak_count ? "/retry" : "",
            !!usb_gettoggle(urb->dev, ep->epnum, 0), ep, len);

    spin_unlock(&urb->lock);
    spin_unlock(&dm320->lock);
    enable_interrupt(flags);
}

/* OUT packets can be used with any type of endpoint.
 * urb->iso_frame_desc is currently ignored here...
 */
static void out_packet(
    struct dm320*       dm320,
    struct dm320h_ep*   ep,
    struct urb*         urb,
    u8                  bank,
    u8                  control)
{
    void*               buf;
    int                 epnum = 0;
    int                 is_out = 0;
    int                 ep_index = 0;
    u16                 flags;
    u8                  len;

    flags = disable_interrupt();
    spin_lock(&dm320->lock);
    spin_lock(&urb->lock);

    epnum = usb_pipeendpoint(urb->pipe);
    is_out = usb_pipeout(urb->pipe) ? 1 : 0;
    ep_index = get_ep_index(epnum, is_out);

    buf = urb->transfer_buffer + urb->actual_length;
    prefetch(buf);

    len = min((int)ep->maxpacket,
            urb->transfer_buffer_length - urb->actual_length);

    dm320_write_ep(dm320, ep_index, buf, len);

    if (ep_index == CTRL_EP) 
    {
        N_DBG("CTRL_EP: %s:%d:%s: len=%d\n", __FILE__, __LINE__, __FUNCTION__,len);
        if (len)
        {
            writeb(USB_HST_CSR0_TXPKTRDY, USB_HST_CSR0);
        }
        else
            writeb(USB_HST_CSR0_TXPKTRDY|USB_HST_CSR0_STPKT, USB_HST_CSR0);
    }
    else 
    {
        writeb(USB_HST_TXCSR1_TXPKTRDY, USB_HST_TXCSR1);
    }

    ep->length = len;
    N_DBG("OUT%s/%d qh%p len%d\n", ep->nak_count ? "/retry" : "",
            !!usb_gettoggle(urb->dev, ep->epnum, 1), ep, len);

    spin_unlock(&urb->lock);
    spin_unlock(&dm320->lock);
    enable_interrupt(flags);
}

/*-------------------------------------------------------------------------*/

/* pick the next endpoint for a transaction, and issue it.
 * frames start with periodic transfers (after whatever is pending
 * from the previous frame), and the rest of the time is async
 * transfers, scheduled round-robin.
 */
static struct dm320h_ep* start(struct dm320* dm320, u8 bank)
{
    struct dm320h_ep*   ep;
    struct urb*         urb;
    u8                  control;
    u16             flags;

    flags = disable_interrupt();
    spin_lock(&dm320->lock);

    /* use endpoint at schedule head */
    if (dm320->next_periodic) 
    {
        printk("PERIODIC SCHEDULING...\n");
        ep = dm320->next_periodic;
        dm320->next_periodic = ep->next;
    } 
    else 
    {
        if (dm320->next_async)
        {
            ep = dm320->next_async;
        }
        else if (!list_empty(&dm320->async))
        {
            ep = container_of(dm320->async.next,
                    struct dm320h_ep, schedule);
        }
        else 
        {
             /* could set up the first fullspeed periodic
             * transfer for the next frame ...
             */
             ep = NULL;
             goto out;
        }

        if (ep->schedule.next == &dm320->async)
        {
            dm320->next_async = NULL;
        }
        else
        {
            dm320->next_async = container_of(ep->schedule.next,
                    struct dm320h_ep, schedule);
        }
    }

    if (unlikely(list_empty(&ep->hep->urb_list))) 
    {
        DBG("empty %p queue?\n", ep);
        ep = NULL;
        goto out;
    }

    urb = container_of(ep->hep->urb_list.next, struct urb, urb_list);
    spin_lock(&urb->lock);

    control = ep->defctrl;

    dm320_setup_ep(dm320, urb, ep);

    ep->toggle = usb_gettoggle(urb->dev, ep->epnum, usb_pipeout(urb->pipe));
    switch (ep->nextpid) 
    {
        case USB_PID_IN:
            in_packet(dm320, ep, urb, bank, control);
            break;

        case USB_PID_OUT:
            out_packet(dm320, ep, urb, bank, control);
            break;

        case USB_PID_SETUP:
            setup_packet(dm320, ep, urb, bank, control);
            break;

        case USB_PID_ACK:        /* for control status */
            status_packet(dm320, ep, urb, bank, control);
            break;

        default:
            DBG("bad ep%p pid %02x\n", ep, ep->nextpid);
            ep = NULL;
    }

    spin_unlock(&urb->lock);

out:
    spin_unlock(&dm320->lock);
    enable_interrupt(flags);
    return ep;
}

#define MIN_JIFFIES    ((msecs_to_jiffies(2) > 1) ? msecs_to_jiffies(2) : 2)
static inline void start_transfer(struct dm320* dm320)
{
    u16             flags;

    flags = disable_interrupt();
    spin_lock(&dm320->lock);

    if (dm320->port1 & (1 << USB_PORT_FEAT_SUSPEND))
    {
        goto out;
    }

    if (dm320->active_a == NULL) 
    {
        dm320->active_a = start(dm320, 0);
        if (dm320->active_a != NULL)
        {
            dm320->jiffies_a = jiffies + MIN_JIFFIES;
        }
    }

out:
    spin_unlock(&dm320->lock);
    enable_interrupt(flags);
}

static void finish_request(
    struct dm320*       dm320,
    struct dm320h_ep*   ep,
    struct urb*         urb,
    struct pt_regs*     regs,
    int                 status
) __releases(dm320->lock) __acquires(dm320->lock)
{
    u16             flags;

    flags = disable_interrupt();
    spin_lock(&dm320->lock);
    spin_lock(&urb->lock);

    N_DBG("%s:%d:%s, urb->status = %d status=%d\n", __FILE__, __LINE__, __FUNCTION__, urb->status,status);

    if (usb_pipecontrol(urb->pipe))
    {
        ep->nextpid = USB_PID_SETUP;
    }

    if (urb->status == -EINPROGRESS)
    {
        urb->status = status;
    }
    urb->hcpriv = NULL;

    spin_unlock(&dm320->lock);
    usb_hcd_giveback_urb(dm320_to_hcd(dm320), urb, regs);
    spin_lock(&dm320->lock);

    /* leave active endpoints in the schedule */
    if (!list_empty(&ep->hep->urb_list))
    {
        goto out;
    }

    /* async deschedule? */
    if (!list_empty(&ep->schedule)) 
    {
        list_del_init(&ep->schedule);
        if (ep == dm320->next_async)
        {
            dm320->next_async = NULL;
        }
        goto out;
    }

out:
    spin_unlock(&urb->lock);
    spin_unlock(&dm320->lock);
    enable_interrupt(flags);
}

static void done(struct dm320* dm320, struct dm320h_ep* ep, unsigned int cs_reg, struct pt_regs* regs)
{
    unsigned int    status;
    struct urb*     urb;
    int             urbstat = -EINPROGRESS;
    int             epnum = 0;
    int             ep_index = 0;
    int             is_out;
    u8              csr_sts=0;

    if (!transfer_started)
    {
        return;
    }

    if (unlikely(!ep))
    {
        return;
    }

    //flags = disable_interrupt();
    urb = container_of(ep->hep->urb_list.next, struct urb, urb_list);

    if (!urb)
    {
        goto bail;
    }

    //spin_lock(&dm320->lock);
    spin_lock(&urb->lock);

    epnum = usb_pipeendpoint(urb->pipe);
    is_out = usb_pipeout(urb->pipe) ? 1 : 0;
    ep_index = get_ep_index(epnum, is_out);

    if (epnum > USB_MAX_DEVICE_EP)
    {
        printk("%s %d number of device endpoints exceeds USB_MAX_DEVICE_EP\n",
            __FUNCTION__, __LINE__); 
    }

    status = get_ctrl_sts_reg(dm320, ep_index, cs_reg,ep->nextpid);
    N_DBG("%s:%d:%s, status = %x\n", __FILE__, __LINE__, __FUNCTION__, status);

    //writeb(ep_index, USB_INDEX);
        
    /* we can safely ignore NAKs */
    if (status & CSR_MASK_NAK) 
    {
        if (urb && (urbstat != -EINPROGRESS || urb->status != -EINPROGRESS)) printk("NAK is not reported properly\n");

        //clear NAK
        N_DBG("%s:%d:%s NAK\n", __FILE__, __LINE__, __FUNCTION__);
        if (!ep->period)
        {
            ep->nak_count++;
        }
        ep->error_count = 0;

        //we returned timeout before after 250 NAKs, but it seems that for he spec NAKs are perfectly 
        //normal, so we keep taking them forever with no problem. --nerochiaro
        //TODO: i'm not sure what it does, but check also what the USB_NAKLMT0 register is for.
        /*
        if((ep->nak_count % 500) == 0)
        {
          printk("\n[dm320 usb host]********** WARN: NAKs so far on endpoint %d => %d *******************\n", ep_index, ep->nak_count);
          //urbstat = -ETIMEDOUT;
          //ep->nak_count=0;
        }
        */
    } 
    /* ACK advances transfer, toggle, and maybe queue */
    else if (status & CSR_MASK_ACK) 
    {
        struct usb_device*  udev = urb->dev;
        int                 len=0;
        unsigned char*      buf;

        /* urb->iso_frame_desc is currently ignored here... */
        N_DBG("%s:%d:%s ACK ep->nextpid=%x\n", __FILE__, __LINE__, __FUNCTION__,ep->nextpid);
        ep->nak_count = ep->error_count = 0;
        switch (ep->nextpid) 
        {
            case USB_PID_OUT:
                urb->actual_length += ep->length;
                usb_settoggle(urb->dev, ep->epnum, usb_pipeout(urb->pipe), !ep->toggle);
                if (urb->actual_length
                        == urb->transfer_buffer_length) 
                {
                    if (usb_pipecontrol(urb->pipe))
                    {
                        ep->nextpid = USB_PID_ACK;
                    }

                    /* some bulk protocols terminate OUT transfers
                     * by a short packet, using ZLPs not padding.
                     */
                    else if (ep->length < ep->maxpacket
                            || !(urb->transfer_flags
                                & URB_ZERO_PACKET))
                    {
                        N_DBG("%s:%d:%s\n", __FILE__, __LINE__, __FUNCTION__);
                            urbstat = 0;
                    }
                }
                break;

            case USB_PID_IN:
                buf = urb->transfer_buffer + urb->actual_length;
                prefetchw(buf);
                if(ep_index == CTRL_EP)
                {
                    len = readb(USB_COUNT0);
                }
                else
                {
                    len = (readb(USB_RXCOUNT2) << 8) |  readb(USB_RXCOUNT1);
                }    
                if (len > ep->length) 
                {
                    len = ep->length;
                    N_DBG("EOVERFLOW !!!!!!!!!!!!!\n");
                    urb->status = 0;
                }
                urb->actual_length += len;
                dm320_read_ep(dm320, ep_index, buf, len);
                usb_settoggle(urb->dev, ep->epnum, usb_pipeout(urb->pipe), !ep->toggle);
                if (urb->actual_length == urb->transfer_buffer_length)
                {
                    urbstat = 0;
                }
                else if (len < ep->maxpacket) 
                {
                    if (urb->transfer_flags & URB_SHORT_NOT_OK)
                    {
                        urbstat = -EREMOTEIO;
                    }
                    else
                    {
                        urbstat = 0;
                    }
                }
                if(usb_pipecontrol(urb->pipe) && urbstat == 0)
                {
                    if(dm320->getdevdescFlag)
                    {
                        if(dm320->devdesc == NULL)
                        {
                            if (devdescsave != NULL)
                            {
                                printk("%s: devdesc possible memory leak\n", __FUNCTION__); 
                            }
                            devdescsave = kmalloc(sizeof(struct usb_device_descriptor), GFP_ATOMIC);
                            dm320->devdesc = devdescsave;
                        }
                        if(!dm320->devdesc)
                        {
                            printk("%s: %d : %s Memory allocation failure\n",__FILE__,__LINE__,__FUNCTION__);
                            goto out;
                        }
                        memcpy(dm320->devdesc,buf,len);

                        dm320->getdevdescFlag = 0;
                    }
                    if(dm320->getconfigdescFlag)
                    {
                        int totbytes;
                        memcpy(&dm320->gdata[dm320->grecvd], buf, len);
                        dm320->grecvd += len;
                        totbytes = (dm320->gdata[3] << 8 | dm320->gdata[2]);    
                        if(dm320->grecvd >= totbytes)
                        {
                            buf = dm320->gdata;
                            dm320->confdesc = (struct usb_config_descriptor*)buf;
                            dm320->intdesc = (struct usb_interface_descriptor*)(buf + sizeof(struct usb_config_descriptor));
                            dm320->numendpts = dm320->intdesc->bNumEndpoints;
                            dm320->getconfigdescFlag = 0;
                            dm320->grecvd = 0;
                        }
                    }
                }
                if (usb_pipecontrol(urb->pipe)
                        && (urbstat == -EREMOTEIO
                            || urbstat == 0)) 
                {
                    /* NOTE if the status stage STALLs (why?),
                     * this reports the wrong urb status.
                     */
                    if (urb->status == -EINPROGRESS)
                    {
                        urb->status = urbstat;
                    }

                    urb = NULL;
                    ep->nextpid = USB_PID_ACK;
                }

                break;

            case USB_PID_SETUP:
                if (urb->transfer_buffer_length == urb->actual_length)
                    ep->nextpid = USB_PID_ACK;
                else if (usb_pipeout(urb->pipe)) 
                {
                    usb_settoggle(udev, 0, 1, 1);
                    ep->nextpid = USB_PID_OUT;
                } 
                else 
                {
                    usb_settoggle(udev, 0, 0, 1);
                    ep->nextpid = USB_PID_IN;
                }
                break;

            case USB_PID_ACK:
                urbstat = 0;
                break;
            }
            N_DBG("%s:%d:%s ep->nextpid=%x\n", __FILE__, __LINE__, __FUNCTION__,ep->nextpid);

            switch(cs_reg)
            {
                case CSR_TXRXEP0:
                    csr_sts = readb(USB_HST_CSR0);
                    if ((csr_sts & USB_HST_CSR0_RXPKTRDY)) 
                    {
                        writeb(readb(USB_HST_CSR0) & ~USB_HST_CSR0_RXPKTRDY, USB_HST_CSR0);
                    }
                    break;

                case CSR_RXEPS:
                    csr_sts = readb(USB_HST_RXCSR1);
                    if ((csr_sts & USB_HST_RXCSR1_RXPKTRDY)) 
                    {
                        writeb(readb(USB_HST_RXCSR1) | USB_HST_RXCSR1_RXPKTRDY | 
                            USB_HST_RXCSR1_FLFIFO, USB_HST_RXCSR1);
                    }
                    break;

                case CSR_TXEPS:
                    break;
        }
    } 
    /* STALL stops all transfers */
    else if (status & CSR_MASK_STALL) 
    {
        PACKET("...STALL_%02x qh%p\n", bank, ep);
        printk("***STALL ep%d ***\n", epnum);
        ep->nak_count = ep->error_count = 0;
        urbstat = -EPIPE;
        udelay(200);                
        stalled[epnum] = 1;

        if (is_out)
        {
            out_eps[epnum] = 0;
        }
        else
        {
            in_eps[epnum] = 0;
        }
    } 
    /* error? retry, until "3 strikes" */
    /* Yes, I know that I should be smacked in the 
       head 17 times with the usb spec, but
       it works on older jump drives. One  
       day a better fix needs to be found. */
    //else if (++ep->error_count >= 20) 
    else if (++ep->error_count >= 200) 
    {
        printk("dm320-hcd.c: error_count overflow------------------\n");
        if (status & CSR_MASK_ERR)
        {
            urbstat = -EILSEQ;
            urb->status= -EILSEQ;
        }
        else if (status & CSR_MASK_OVRRUN) 
        {
            urbstat = -EOVERFLOW;
        }
        else if (status & CSR_MASK_UNDRRUN)
        {
            urbstat = -EOVERFLOW;
        }
        else
        {
            urbstat = -EPROTO;
        }
        ep->error_count = 0;
        N_DBG("%s:%d:%s (++ep->error_count >= 3), urbstat=%d ,status=%d\n", __FILE__, __LINE__, __FUNCTION__,urbstat,status);
        PACKET("...3STRIKES_%02x %02x qh%p stat %d\n",
                cs_reg, status, ep, urbstat);
    }

    if (urb && (urbstat != -EINPROGRESS || urb->status != -EINPROGRESS))
    {
        finish_request(dm320, ep, urb, regs, urbstat);
    }

out:
    spin_unlock(&urb->lock);
    //spin_unlock(&dm320->lock);
bail:
    //enable_interrupt(flags);
    return;
}

unsigned int dm320_read_intr_regs(void)
{
    unsigned int intsts=0;    
    unsigned int intr_tx     = readb(USB_INTRTX1) & 0xFF;
    unsigned int intr_rx     = readb(USB_INTRRX1) & 0xFF;
    unsigned int intr_usb    = readb(USB_INTRUSB) & 0xFF;

    intsts = (intr_tx << 12) + ((intr_rx >> 1) << 8) + intr_usb;
    intsts = intsts & 0x1ffff;
    return intsts;
}

unsigned int dm320_get_intr_by_priority( unsigned int intr_flags )
{
    if( ( intr_flags & USB_RESUME          ) != 0 )        return  USB_RESUME;
    if( ( intr_flags & USB_SESSREQ         ) != 0 )        return  USB_SESSREQ;
    if( ( intr_flags & USB_VBUSERR         ) != 0 )        return  USB_VBUSERR;
    if( ( intr_flags & USB_CONNECTED       ) != 0 )        return  USB_CONNECTED;
    if( ( intr_flags & USB_RESET           ) != 0 )        return  USB_RESET; 
    if( ( intr_flags & USB_CONTROL         ) != 0 )        return  USB_CONTROL;
    if( ( intr_flags & USB_RXFIFO          ) != 0 )        return  USB_RXFIFO;
    if( ( intr_flags & USB_TXFIFO          ) != 0 )        return  USB_TXFIFO;
    if( ( intr_flags & USB_SOF             ) != 0 )        return  USB_SOF;
    if( ( intr_flags & USB_DISCONNECTED    ) != 0 )        return  USB_DISCONNECTED;
    if( ( intr_flags & USB_SUSPEND         ) != 0 )        return  USB_SUSPEND;

    return USB_NO_INTERRUPT;
}

static unsigned int get_ctrl_sts_reg(struct dm320* dm320, int ep_index, unsigned int cs_reg, int pid)
{
    unsigned int status = 0;
    unsigned short csr_sts;
    u16 flags;

    flags = disable_interrupt();
    spin_lock(&dm320->lock);

    writeb(ep_index, USB_INDEX);

    switch(cs_reg)
    {
    case CSR_TXRXEP0:
        csr_sts = readb(USB_HST_CSR0);
        
        if((pid==USB_PID_IN))
        {
            if(csr_sts & USB_HST_CSR0_RXPKTRDY)
            {
                status |= CSR_MASK_ACK;
            }
            else if(csr_sts & USB_HST_CSR0_NAKTO) 
            {
                status |= CSR_MASK_NAK;
                writeb((csr_sts & ~USB_HST_CSR0_NAKTO), USB_HST_CSR0);
            }
            else if(csr_sts & USB_HST_CSR0_RXSTALL) 
            {
                status |= CSR_MASK_STALL;
                printk("stall on ep = %d @%d\n",ep_index,__LINE__);
                writeb((csr_sts & ~USB_HST_CSR0_RXSTALL), USB_HST_CSR0);
            }
            else if(csr_sts & USB_HST_CSR0_ERR) 
            {
                status |= CSR_MASK_ERR;
                writeb((csr_sts & ~USB_HST_CSR0_ERR), USB_HST_CSR0);
            }
        }
        else
        {
            if(!(csr_sts & USB_HST_CSR0_TXPKTRDY)) 
            {
                status |= CSR_MASK_ACK;
            }
            else if(csr_sts & USB_HST_CSR0_NAKTO) 
            {
                status |= CSR_MASK_NAK;
                writeb((csr_sts & ~USB_HST_CSR0_NAKTO), USB_HST_CSR0);
            }
            else if(csr_sts & USB_HST_CSR0_RXSTALL) 
            {
                status |= CSR_MASK_STALL;
                printk("stall on ep = %d @%d\n",ep_index,__LINE__);
                writeb((csr_sts & ~USB_HST_CSR0_RXSTALL), USB_HST_CSR0);
            }
            else if(csr_sts & USB_HST_CSR0_ERR) 
            {
                status |= CSR_MASK_ERR;
                writeb((csr_sts & ~USB_HST_CSR0_ERR), USB_HST_CSR0);
            }
        }
        break;
    
    case CSR_RXEPS:
        csr_sts = readb(USB_HST_RXCSR1);
        DBG("CSR_RXEPS: %s:%d:%s, csr_sts = %x\n", __FILE__, __LINE__, __FUNCTION__, csr_sts);
        if(csr_sts & USB_HST_RXCSR1_RXPKTRDY) 
        {
            status |= CSR_MASK_ACK;
        }
        else if(csr_sts & USB_HST_RXCSR1_DATERR) 
        {
            status |= CSR_MASK_NAK;
            writeb(csr_sts | USB_HST_RXCSR1_RXPKTRDY, USB_HST_RXCSR1);
        }
        else if(csr_sts & USB_HST_RXCSR1_RXSTALL) 
        {
            printk("stall on ep = %d @%d\n",ep_index,__LINE__);
            status |= CSR_MASK_STALL;
            writeb(csr_sts & ~USB_HST_RXCSR1_RXSTALL, USB_HST_RXCSR1);
        }
        else if(csr_sts & USB_HST_RXCSR1_ERR) 
        {
            status |= CSR_MASK_ERR;
            writeb(csr_sts & ~USB_HST_RXCSR1_ERR, USB_HST_RXCSR1);
        }
        break;

    case CSR_TXEPS:
        csr_sts = readb(USB_HST_TXCSR1);
        DBG("CSR_TRXEPS: %s:%d:%s, csr_sts = %x\n", __FILE__, __LINE__, __FUNCTION__, csr_sts);
    
        if(!(csr_sts & USB_HST_TXCSR1_TXPKTRDY)) 
        {
            status |= CSR_MASK_ACK;
        }
        else if(csr_sts & USB_HST_TXCSR1_NAKTO) 
        {
            status |= CSR_MASK_NAK;
            writeb(csr_sts & ~USB_HST_TXCSR1_NAKTO, USB_HST_TXCSR1);
        }
        else if(csr_sts & USB_HST_TXCSR1_RXSTALL) 
        {
            printk("stall on ep = %d @%d\n",ep_index,__LINE__);
            status |= CSR_MASK_STALL;
            writeb(csr_sts & ~USB_HST_TXCSR1_RXSTALL, USB_HST_TXCSR1);
        }        
        else if(csr_sts & USB_HST_TXCSR1_ERR)
        {
            status |=  CSR_MASK_ERR;
            writeb(csr_sts & ~USB_HST_TXCSR1_ERR, USB_HST_TXCSR1);
        }
        break;
    }

    spin_unlock(&dm320->lock);
    enable_interrupt(flags);
    return status;
}

static int dm320_setup_ep(struct dm320* dm320, struct urb* urb, struct dm320h_ep* ep)
{
    int mxpktsize = 0, epnum = 0, type = 0;
    int is_out = 0, dev_addr = 0, speed = 0;
    int ep_index = 0;
    int speeds[5] = {0, 3, 2, 1, 0};
    int mxpktsz = 0, type_reg = 0, fLowAddr = 0, fHighAddr = 0;
    int types[5] = {ISO_EP, CTRL_EP, INTR_EP, BULK_EP};
    int size = mxpktsz;
    u16 flags;

    flags = disable_interrupt();
    spin_lock(&dm320->lock);
    spin_lock(&urb->lock);

    mxpktsize = usb_maxpacket(urb->dev, urb->pipe, usb_pipeout (urb->pipe));
    epnum = usb_pipeendpoint(urb->pipe);
    is_out = usb_pipeout(urb->pipe) ? 1 : 0;
    ep_index = get_ep_index(epnum, is_out);
    type = usb_pipetype(urb->pipe);
    dev_addr = usb_pipedevice(urb->pipe);
    speed = speeds[urb->dev->speed];
    writeb(dev_addr, USB_FADDR);
    writeb(ep_index, USB_INDEX);

    if (epnum > USB_MAX_DEVICE_EP)
    {
        printk("%s %d number of device endpoints exceeds USB_MAX_DEVICE_EP\n",
            __FUNCTION__, __LINE__); 
    }

    switch(mxpktsize)
    {
        case 8:     mxpktsz = 0x00; break;
        case 16:    mxpktsz = 0x20; break;
        case 32:    mxpktsz = 0x40; break;
        case 64:    mxpktsz = 0x60; break;
        case 128:   mxpktsz = 0x80; break;
        case 256:   mxpktsz = 0xa0; break;
        case 512:   mxpktsz = 0xc0; break;
        case 1024:  mxpktsz = 0xe0; break;
    }

    if(ep_index == CTRL_EP && type == PIPE_CONTROL) 
    {
        writeb(mxpktsz, USB_TXFIFO2);   // TX  bytes, single buffer, ADDH = 0
        writeb(mxpktsz, USB_RXFIFO2);   // RX  bytes, single buffer, ADDH = 0
        writeb(size, USB_TXMAXP);       // max TX packet size = size / 8
        writeb(size, USB_RXMAXP);       // max RX packet size = size / 8
        writeb(USB_REG_NAK_LIMIT, USB_NAKLMT0);

        //TODO hub portion to be done
        goto out;
    }

    if(is_out)
    {
        if(out_eps[epnum] == 1)
        {
            goto out;
        }
        else
        {
            out_eps[epnum] = 1;
        }
    }
    else
    {
        if(in_eps[epnum] == 1)
        {
            goto out;
        }
        else
        {
            in_eps[epnum] = 1;
        }
    }

    type = types[usb_pipetype(urb->pipe)];
    type_reg = (((type & 0x3) << 4)) | (epnum & 0xF);
    if(is_out)
    {
        DBG("OUT: mxpktsz=%x, epnum=%d, nBufPosOut=%d, nBufPosIn=%d type_reg=%x, type=%d\n", mxpktsz,epnum, nBufPosOut, nBufPosIn,type_reg, type);

        writeb((mxpktsize / 8), USB_TXMAXP);
        writeb(type_reg, USB_TXTYPE);
        writeb(USB_REG_TX_INTERVAL, USB_TXINTVL);

        writeb(readb(USB_INTRTX1E) | (1 << ep_index), USB_INTRTX1E);
        if(stalled[epnum] == 0)
        {
            nBufPosOut += 64;
        }
        if(2048-nBufPosOut < mxpktsize) 
        {
            nBufPosOut=0;
        }

        if (nBufPosOut >= 2048)  printk("FIXME: dm320-hcd.c buffer out overflow.\n");
 
        nBufPosOut = nBufPosOut % 2048;
        fLowAddr = (nBufPosOut / 8) & 0xFF ;
        fHighAddr = (nBufPosOut >= 2048) ? 1 : 0;
        writeb(fLowAddr ,USB_TXFIFO1);
        writeb(fHighAddr | mxpktsz, USB_TXFIFO2);
        if((readb(USB_HST_TXCSR1) & USB_HST_TXCSR1_FIFOEMP) != 0)
        {
            writeb(readb(USB_HST_TXCSR1) | USB_HST_TXCSR1_CLRDATTOG | USB_HST_TXCSR1_FLFIFO, USB_HST_TXCSR1);
        }
        else
        {
            writeb(USB_HST_TXCSR1_CLRDATTOG, USB_HST_TXCSR1);
        }
    } 
    else 
    {
        DBG("IN: mxpktsz=%x, epnum=%d, nBufPosOut=%d, nBufPosIn=%d type_reg=%x, type=%d\n", mxpktsz,epnum, nBufPosOut, nBufPosIn,type_reg, type);

        writeb((mxpktsize / 8),USB_RXMAXP);
        writeb(type_reg, USB_RXTYPE);
        writeb(USB_REG_RX_INTERVAL, USB_RXINTVL);

        writeb(readb(USB_INTRRX1E) | (1 << ep_index), USB_INTRRX1E);
        if(stalled[epnum]==0)
        {
            nBufPosIn += 64;
        }
        if((2048-nBufPosIn) < mxpktsize) 
        {
            nBufPosIn=0;
        }

        if (nBufPosIn >= 2048)  printk("FIXME: dm320-hcd.c buffer in overflow.\n");

        nBufPosIn = nBufPosIn % 2048;
        fLowAddr = (nBufPosIn/8) & 0xFF ;
        fHighAddr = (nBufPosIn >= 2048) ? 1 : 0;
        writeb( fLowAddr ,USB_RXFIFO1 );
        writeb( fHighAddr | mxpktsz ,   USB_RXFIFO2 );

        if((readb(USB_HST_RXCSR1) & USB_HST_RXCSR1_RXPKTRDY) != 0)
        {
            writeb(USB_HST_RXCSR1_CLRDATTOG | USB_HST_RXCSR1_FLFIFO, USB_HST_RXCSR1);
        }
        else
        {
            writeb(USB_HST_RXCSR1_CLRDATTOG, USB_HST_RXCSR1);
        }
    }
    
    // TODO hub portion to be done
    // TODO check the necessity of the toggle clear

out:
    spin_unlock(&urb->lock);
    spin_unlock(&dm320->lock);
    enable_interrupt(flags);
    return 0;
}

static void dm320_reset_service(void)
{
    u16 flags;

    flags = disable_interrupt();

    writeb(0x00, USB_INDEX);
    writeb(0x00, USB_FADDR);

    writeb(0x00, USB_POWER);
    writeb(0x08, USB_POWER); // reset the device connected.
    mdelay(250);
    writeb(0x00, USB_POWER);  // remove reset

    enable_interrupt(flags);
}

void dm320_sof_service(struct dm320* dm320)
{
    unsigned index;

    spin_lock(&dm320->lock);

    index = dm320->frame++ % (PERIODIC_SIZE - 1);
    dm320->stat_sof++;
    /* be graceful about almost-inevitable periodic schedule
    * overruns:  continue the previous frame's transfers iff
    * this one has nothing scheduled.
    */
    if (dm320->next_periodic) 
    {
        dm320->stat_overrun++;
    }
    if (dm320->periodic[index])
    {
        dm320->next_periodic = dm320->periodic[index];
    }

    spin_unlock(&dm320->lock);
}

static irqreturn_t  dm320h_irq(struct usb_hcd* hcd, struct pt_regs* regs )
{
    int                 intr_flags;
    int                 intr_sts;
    struct dm320*       dm320 = hcd_to_dm320(hcd);
    irqreturn_t         ret = IRQ_NONE;
    u16             flags;

    flags = disable_interrupt();
    spin_lock(&dm320->lock);

    intr_flags = dm320_read_intr_regs();
    do{
        intr_sts = dm320_get_intr_by_priority(intr_flags);
        switch(intr_sts) 
        {
            case USB_RESET:                
                printk("INT RESET\r\n" );
                break;
                
            case USB_SESSREQ:
                printk("%s %s %d please implent USB_SESSREQ\n", 
                    __FILE__, __FUNCTION__, __LINE__);
                break;
                
            case USB_CONNECTED:
              //printk("INT CONNECTED\r\n" );
                action = USB_PLUG__;
                plug_debounce = USB_PLUG_DEBOUNCE__;
                break;

            case USB_RESUME:
                printk("%s %s %d please implent USB_RESUME\n", 
                    __FILE__, __FUNCTION__, __LINE__);
                break;

            case USB_CONTROL:
                N_DBG("INT CONTROL [%x] \n",readb(USB_HST_CSR0));
                done(dm320, dm320->active_a, (unsigned int)CSR_TXRXEP0, regs);
                dm320->active_a = NULL;
                dm320->stat_a++;
                break;

            case USB_RXFIFO:
                N_DBG("INT RXFIFO [%x] \r\n",  readb(USB_HST_RXCSR1));
                done(dm320, dm320->active_a, (unsigned int)CSR_RXEPS, regs);
                dm320->active_a = NULL;
                dm320->stat_a++;
                break;

            case USB_TXFIFO:
                N_DBG("INT TXFIFO [%x] \r\n",  readb(USB_HST_TXCSR1));
                done(dm320, dm320->active_a, (unsigned int)CSR_TXEPS, regs);
                dm320->active_a = NULL;
                dm320->stat_a++;
                break;

            case USB_SOF:
                dm320_sof_service(dm320);
                break;

            case USB_DISCONNECTED:
              //printk("INT DISCONNECTED\r\n" );
                action = USB_PULL__;
                plug_debounce = USB_PLUG_DEBOUNCE__;
                break;

            case USB_SUSPEND:
                printk("%s %s %d please implent USB_SUSPEND\n", 
                    __FILE__, __FUNCTION__, __LINE__);
                break;

            default:
                //printk("intr_sts : %x\n",intr_sts);
                break;
        }
        if (intr_flags) 
        {
            if ((dm320->port1 & (1 << USB_PORT_FEAT_ENABLE)) && transfer_started)
            {
              start_transfer(dm320);
            }
            ret = IRQ_HANDLED;
        }
        intr_flags = intr_flags & ~intr_sts;
    } while(intr_flags != USB_NO_INTERRUPT);

    spin_unlock(&dm320->lock);
    enable_interrupt(flags);
    return ret;
}

/*-------------------------------------------------------------------------*/

static int dm320h_urb_enqueue(
    struct usb_hcd*             hcd,
    struct usb_host_endpoint*   hep,
    struct urb*                 urb,
    gfp_t                       mem_flags)
{
    struct dm320*       dm320;
    struct usb_device*  udev;
    unsigned int        pipe;
    int                 is_out;
    int                 type;
    int                 epnum;
    struct dm320h_ep    *ep = NULL;
    u16                 flags;
    int                 retval = 0;

    flags = disable_interrupt();

    dm320 = hcd_to_dm320(hcd);
    spin_lock(&dm320->lock);
    spin_lock(&urb->lock);

    udev = urb->dev;
    pipe = urb->pipe;
    is_out = !usb_pipein(pipe);
    type = usb_pipetype(pipe);
    epnum = usb_pipeendpoint(pipe);

#ifdef DISABLE_ISO
    if (type == PIPE_ISOCHRONOUS)
    {
        retval = -ENOSPC;
        goto fail;
    }
#endif

    /* avoid all allocations within spinlocks */
    if (!hep->hcpriv)
    {
        ep = kzalloc(sizeof *ep, mem_flags);
    }

    /* don't submit to a dead or disabled port */
    if (!(dm320->port1 & (1 << USB_PORT_FEAT_ENABLE)))
    {
        retval = -ENODEV;
        if (ep) kfree(ep);
        goto fail;
    }

    if (hep->hcpriv) 
    {
        if (ep) kfree(ep);
        ep = hep->hcpriv;
    } 
    else if (!ep) 
    {
        retval = -ENOMEM;
        printk("%s:%d:%s: failed here\n",__FILE__,__LINE__,__FUNCTION__);
        goto fail;
    } 
    else 
    {
        INIT_LIST_HEAD(&ep->schedule);
        ep->udev = usb_get_dev(udev);
        ep->epnum = epnum;
        ep->maxpacket = usb_maxpacket(udev, urb->pipe, is_out);
        usb_settoggle(udev, epnum, is_out, 0);

        if (type == PIPE_CONTROL)
        {
            ep->nextpid = USB_PID_SETUP;
        }
        else if (is_out)
        {
            ep->nextpid = USB_PID_OUT;
        }
        else
        {
            ep->nextpid = USB_PID_IN;
        }

        if (ep->maxpacket > H_MAXPACKET) 
        {
            /* iso packets up to 240 bytes could work... */
            DBG("dev %d ep%d maxpacket %d\n",
                udev->devnum, epnum, ep->maxpacket);
            retval = -EINVAL;
            printk("%s:%d:%s: failed here\n",__FILE__,__LINE__,__FUNCTION__);
            goto fail;
        }

        switch (type) 
        {
            case PIPE_ISOCHRONOUS:
            case PIPE_INTERRUPT:
                if (urb->interval > PERIODIC_SIZE)
                {
                    urb->interval = PERIODIC_SIZE;
                }
                ep->period = urb->interval;
                ep->branch = PERIODIC_SIZE;
                ep->load = usb_calc_bus_time(udev->speed, !is_out,
                    (type == PIPE_ISOCHRONOUS),
                    usb_maxpacket(udev, pipe, is_out))
                        / 1000;
                break;
        }

        ep->hep = hep;
        hep->hcpriv = ep;
    }
    DBG("TYPE=%x\n", type);

    /* maybe put endpoint into schedule */
    switch (type) 
    {
        case PIPE_CONTROL:
        case PIPE_BULK:
          // schedule only if it is not already.
            if (list_empty(&ep->schedule))
                list_add_tail(&ep->schedule, &dm320->async);
            break;

        case PIPE_ISOCHRONOUS:
        case PIPE_INTERRUPT:
            printk("%s %s %d please implent PIPE_INTERRUPT\n", 
            __FILE__, __FUNCTION__, __LINE__);
            break;
    }

    /* in case of unlink-during-submit */
    if (urb->status != -EINPROGRESS) 
    {
        N_DBG("%s:%d:%s\n", __FILE__, __LINE__, __FUNCTION__);
        DBG("1_urb->transfer_buffer_length=%x\n", urb->transfer_buffer_length);
        finish_request(dm320, ep, urb, NULL, 0);
        N_DBG("%s:%d:%s\n", __FILE__, __LINE__, __FUNCTION__);
        retval = 0;
        printk("%s:%d:%s: failed here\n",__FILE__,__LINE__,__FUNCTION__);
        goto fail;
    }
    urb->hcpriv = hep;

    transfer_started = 1;
    start_transfer(dm320);

fail:
    spin_unlock(&urb->lock);
    spin_unlock(&dm320->lock);
    enable_interrupt(flags);
    return retval;
}

static int dm320h_urb_dequeue(struct usb_hcd* hcd, struct urb* urb)
{
    struct dm320*               dm320;
    struct usb_host_endpoint*   hep;
    u16                         flags;
    struct dm320h_ep*           ep;
    int                         retval = 0;

    flags = disable_interrupt();

    dm320 = hcd_to_dm320(hcd);
    spin_lock(&dm320->lock);
    spin_lock(&urb->lock);

    hep = urb->hcpriv;
    if (!hep)
    {
        retval = -EINVAL;
        printk("%s:%d:%s: failed here\n",__FILE__,__LINE__,__FUNCTION__);
        goto done;
    }

    ep = hep->hcpriv;
    if (ep) 
    {
        /* finish right away if this urb can't be active ...
         * note that some drivers wrongly expect delays
         */
        if (ep->hep->urb_list.next != &urb->urb_list) 
        {
            /* not front of queue?  never active */
            /* for active transfers, we expect an IRQ */
        } 
        else if (dm320->active_a == ep) 
        {
            if (time_before_eq(dm320->jiffies_a, jiffies)) 
            {
                dm320->active_a = NULL;
            } 
            else
            {
                urb = NULL;
            }
        } 

        if (urb)
        {
            finish_request(dm320, ep, urb, NULL, 0);
        }
        else
        {
            VDBG("dequeue, urb %p active %s; wait4irq\n", urb,
                (dm320->active_a == ep) ? "A" : "B");
        }
    }
    else
    {
        retval = -EINVAL;
    }

done:
    spin_unlock(&urb->lock);
    spin_unlock(&dm320->lock);
    enable_interrupt(flags);
    return retval;
}

static void dm320h_endpoint_disable(struct usb_hcd* hcd, struct usb_host_endpoint* hep)
{
    struct dm320h_ep*    ep;
    u16                 flags;

    flags = disable_interrupt();

    ep = hep->hcpriv;
    if (!ep)
    {
        goto out;
    }

    /* assume we'd just wait for the irq */
    if (!list_empty(&hep->urb_list))
    {
        enable_interrupt(flags);
        msleep(30);
        flags = disable_interrupt();

        // refetch just in case data changed during interrupt.
        ep = hep->hcpriv;
        if (!ep)
          {
            goto out;
          }
    }

    if (!list_empty(&hep->urb_list))
    {
        WARN("ep %p not empty?\n", ep);
    }

    usb_put_dev(ep->udev);
    kfree(ep);
    hep->hcpriv = NULL;

out:
    enable_interrupt(flags);
}

static int dm320h_get_frame(struct usb_hcd* hcd)
{
    struct dm320 *dm320 = hcd_to_dm320(hcd);

    /* wrong except while periodic transfers are scheduled;
     * never matches the on-the-wire frame;
     * subject to overruns.
     */
    return (dm320->frame);
}


/*-------------------------------------------------------------------------*/

/* the virtual root hub timer IRQ checks for hub status */
static int dm320h_hub_status_data(struct usb_hcd *hcd, char *buf)
{
    /* tell khubd port 1 changed */
    if ((state != IDLE__) || (action != USB_NOACTION__))
    {
        *buf = (1 << 1);
    }
    else 
    {
        *buf = 0;
    }

    return 1;
}

static void dm320h_hub_descriptor (
    struct dm320*                dm320,
    struct usb_hub_descriptor*   desc)
{
    u16        temp = 0;

    spin_lock(&dm320->lock);

    desc->bDescriptorType = 0x29;
    desc->bHubContrCurrent = 0;

    desc->bNbrPorts = 1;
    desc->bDescLength = 9;

    /* per-port power switching (gang of one!), or none */
    desc->bPwrOn2PwrGood = 0;
    if (dm320->board && dm320->board->port_power) 
    {
        desc->bPwrOn2PwrGood = dm320->board->potpg;
        if (!desc->bPwrOn2PwrGood)
        {
            desc->bPwrOn2PwrGood = 10;
        }
        temp = 0x0001;
    } 
    else
    {
        temp = 0x0002;
    }

    /* no overcurrent errors detection/handling */
    temp |= 0x0010;

    desc->wHubCharacteristics = (__force __u16)cpu_to_le16(temp);
    DBG("desc->wHubCharacteristic = %x, desc->bPwrOn2PwrGood=%x\n", desc->wHubCharacteristics, desc->bPwrOn2PwrGood);

    /* two bitmaps:  ports removable, and legacy PortPwrCtrlMask */
    desc->bitmap[0] = 0;
    desc->bitmap[1] = ~0;

    spin_unlock(&dm320->lock);
}

static int dm320h_hub_control(
    struct usb_hcd    *hcd,
    u16        typeReq,
    u16        wValue,
    u16        wIndex,
    char        *buf,
    u16        wLength)
{
    struct dm320*       dm320;
    int                 retval = 0;
    u16                 flags;

    DBG("%s:%d:%s: typeReq=%x, wValue=%x, wIndex=%x\n", __FILE__, __LINE__, __FUNCTION__, typeReq, wValue, wIndex);

    flags = disable_interrupt();

    dm320 = hcd_to_dm320(hcd);
    spin_lock(&dm320->lock);

    hcd->uses_new_polling = 0;

    switch (typeReq) 
    {
        case ClearHubFeature:
        case SetHubFeature:
            switch (wValue) 
            {
                case C_HUB_OVER_CURRENT:
                case C_HUB_LOCAL_POWER:
                    break;

                default:
                    goto error;
            }
            break;

        case ClearPortFeature:
            if (wIndex != 1 || wLength != 0)
                goto error;
            switch (wValue) 
            {
                case USB_PORT_FEAT_ENABLE:
                    port.p_sts.Power = 1;
                    break;

                case USB_PORT_FEAT_SUSPEND:
                    port.p_sts.Suspend = 1;
                    if (!(dm320->port1 & (1 << USB_PORT_FEAT_SUSPEND)))
                        break;

                    /* 20 msec of resume/K signaling, other irqs blocked */
                    DBG("start resume...\n");
                    break;

                case USB_PORT_FEAT_POWER:
                    port.p_sts.Power = 1;
                    port_power(dm320, 0);
                    break;

                case USB_PORT_FEAT_C_ENABLE:
                    port.p_cng.Enable = 1;

                case USB_PORT_FEAT_C_SUSPEND:
                    port.p_cng.Suspend = 1;
                    break;

                case USB_PORT_FEAT_C_CONNECTION:
                    port.p_cng.Connection = 0;
                    break;

                case USB_PORT_FEAT_C_OVER_CURRENT:
                    port.p_cng.OverCurrent = 1;
                    break;

                case USB_PORT_FEAT_C_RESET:
                  //printk("ClearPortFeature: RESET_\n");
                    port.p_cng.Reset = 0;
                    GOING2IDLE_STATE();
                    break;

                default:
                    goto error;
            }
            break;

        case GetHubDescriptor:
            dm320h_hub_descriptor(dm320, (struct usb_hub_descriptor *) buf);
            break;

        case GetHubStatus:
            *(__le32 *) buf = cpu_to_le32(0);
            break;

        case GetPortStatus:
            switch(state)
            {
            case START__:
              //printk("state == START_\n");
              // reinit controls here
              init_control_vars(dm320);
              plug_debounce = USB_PLUG_DEBOUNCE__;
              writeb(readb(USB_DEVCTL)|0x01,USB_DEVCTL); // set SESSREQ bit of DEVCTL
              GOING2IDLE_STATE();
              break;

            case CONNECTED__:
              //printk("state == CONNECTED_\n");
              if (plug_debounce-- > 0) 
                {
                  if (action != USB_PLUG__) GOING2IDLE_STATE();
                  break;
                }
              action = USB_NOACTION__;
              proc_action = USB_PLUG__;
              memset(&port, 0, sizeof(struct port_t));
              memset(&ep_index, 0, sizeof(ep_index));
              GOING2IDLE_STATE();
              port.p_sts.Enable = 1;
              port.p_sts.Connection = 1;
              port.p_cng.Connection = 1;
              break;

            case RESET__:
              //printk("state == RESET_\n");
              port.p_sts.Reset = 0;
              break;

            case GOING2IDLE__:
              if (idle_debounce-- <= 0) state = IDLE__;
              break;

            case IDLE__:
              switch (action)
                {
                case USB_PLUG__: state = CONNECTED__;    break;
                case USB_PULL__: state = DISCONNECTED__; break;
                default:                                 break;
                }
              //printk("state == IDLE_\n");
              break;

            case DISCONNECTED__:
              //printk("state == DISCONNECTED_\n");
              if (plug_debounce-- > 0) 
                {
                  if (action != USB_PULL__) GOING2IDLE_STATE();
                  break;
                }
              action = USB_NOACTION__;
              proc_action = USB_PULL__;

              state = START__;
              port.p_sts.Enable = 0;
              port.p_sts.Connection = 0;
              port.p_cng.Connection = 1; 
              break;

            default:
              printk("UNKNOWN STATE: %s:%d:%s\n", __FILE__, __LINE__, __FUNCTION__);
              break;
            }

            dm320->port1 = PK2U32(port.p_sts, port.p_cng);
            *(__le32 *) buf = cpu_to_le32(dm320->port1);
            DBG("Line:%d, GetPortStatus %x\n", __LINE__, *(__le32 *) buf);
            retval = 0;
            break;

        case SetPortFeature:
            if (wIndex != 1 || wLength != 0)
                goto error;

            switch (wValue)
            {
                case USB_PORT_FEAT_SUSPEND:
                    port.p_sts.Suspend = 1;
                    if (dm320->port1 & (1 << USB_PORT_FEAT_RESET))
                    {
                        goto error;
                    }
                    if (!(dm320->port1 & (1 << USB_PORT_FEAT_ENABLE)))
                    {
                        goto error;
                    }
                    break;

                case USB_PORT_FEAT_POWER:
                    port_power(dm320, 1);
                    port.p_sts.Power = 1;
                    break;

                case USB_PORT_FEAT_RESET:                  
                    dm320_reset_service();
                    port.p_sts.Reset = 1;
                    state = RESET__;
                    break;
                  
                default:
                    DBG("%s:%d:%s ERROR\n", __FILE__, __LINE__, __FUNCTION__);
                    printk("%s:%d:%s ERROR\n", __FILE__, __LINE__, __FUNCTION__);
                    goto error;
            }
            break;

        default:
error:
            /* "protocol stall" on error */
            retval = -EPIPE;
            break;
    }

    spin_unlock(&dm320->lock);
    enable_interrupt(flags);

    return retval;
}

#ifdef    CONFIG_PM
static int dm320h_bus_suspend(struct usb_hcd* hcd)
{
    // SOFs off
    DBG("%s:%d:%s\n", __FILE__, __LINE__, __FUNCTION__);

    return 0;
}

static int dm320h_bus_resume(struct usb_hcd* hcd)
{
    // SOFs on
    DBG("%s:%d:%s\n", __FILE__, __LINE__, __FUNCTION__);

    return 0;
}

#else

#define    dm320h_bus_suspend   NULL
#define    dm320h_bus_resume    NULL

#endif


/*-------------------------------------------------------------------------*/

#ifdef STUB_DEBUG_FILE

static inline void create_debug_file(struct dm320* dm320) {}
static inline void remove_debug_file(struct dm320* dm320) {}

#else

#include <linux/proc_fs.h>
#include <linux/seq_file.h>

static int proc_dm320h_show(struct seq_file* s, void* unused)
{
    struct dm320*       dm320 = s->private;
    struct dm320h_ep*   ep;
    unsigned            i;

    seq_printf(s, "%s\n%s version %s\nportstatus[1] = %08x\n",
        dm320_to_hcd(dm320)->product_desc,
        hcd_name, DRIVER_VERSION,
        dm320->port1);

    seq_printf(s, "insert/remove: %ld\n", dm320->stat_insrmv);
    seq_printf(s, "current session:  done_a %ld done_b %ld "
            "wake %ld sof %ld overrun %ld lost %ld\n\n",
        dm320->stat_a, dm320->stat_b,
        dm320->stat_wake, dm320->stat_sof,
        dm320->stat_overrun, dm320->stat_lost);

    spin_lock_irq(&dm320->lock);

    seq_printf(s, "\n");
    list_for_each_entry (ep, &dm320->async, schedule) 
    {
        struct urb        *urb;

        seq_printf(s, "%sqh%p, ep%d%s, maxpacket %d"
                    " nak %d err %d\n",
            (ep == dm320->active_a) ? "(A) " : "",
            ep, ep->epnum,
            ({ char *s; switch (ep->nextpid) {
            case USB_PID_IN: s = "in"; break;
            case USB_PID_OUT: s = "out"; break;
            case USB_PID_SETUP: s = "setup"; break;
            case USB_PID_ACK: s = "status"; break;
            default: s = "?"; break;
            }; s;}),
            ep->maxpacket,
            ep->nak_count, ep->error_count);
        list_for_each_entry (urb, &ep->hep->urb_list, urb_list) 
        {
            seq_printf(s, "  urb%p, %d/%d\n", urb,
                urb->actual_length,
                urb->transfer_buffer_length);
        }
    }
    if (!list_empty(&dm320->async))
    {
        seq_printf(s, "\n");
    }

    seq_printf(s, "periodic size= %d\n", PERIODIC_SIZE);

    for (i = 0; i < PERIODIC_SIZE; i++) 
    {
        ep = dm320->periodic[i];
        if (!ep)
        {
            continue;
        }
        seq_printf(s, "%2d [%3d]:\n", i, dm320->load[i]);

        /* DUMB: prints shared entries multiple times */
        do {
            seq_printf(s,
                "   %sqh%d/%p (%sdev%d ep%d%s max %d) "
                "err %d\n",
                (ep == dm320->active_a) ? "(A) " : "",
                ep->period, ep,
                (ep->udev->speed == USB_SPEED_FULL)
                    ? "" : "ls ",
                ep->udev->devnum, ep->epnum,
                (ep->epnum == CTRL_EP) ? ""
                    : ((ep->nextpid == USB_PID_IN)
                        ? "in"
                        : "out"),
            ep->maxpacket, ep->error_count);
            ep = ep->next;
        } while (ep);
    }

    spin_unlock_irq(&dm320->lock);
    seq_printf(s, "\n");

    return 0;
}

static int proc_dm320h_open(struct inode* inode, struct file* file)
{
    return single_open(file, proc_dm320h_show, PDE(inode)->data);
}

static struct file_operations proc_ops = {
    .open       = proc_dm320h_open,
    .read       = seq_read,
    .llseek     = seq_lseek,
    .release    = single_release,
};

/* expect just one dm320 per system */
static const char proc_filename[] = "driver/dm320h";

static void create_debug_file(struct dm320* dm320)
{
    struct proc_dir_entry *pde;

    spin_lock(&dm320->lock);

    pde = create_proc_entry(proc_filename, 0, NULL);
    if (pde == NULL)
    {
        return;
    }

    pde->proc_fops = &proc_ops;
    pde->data = dm320;
    dm320->pde = pde;

    spin_unlock(&dm320->lock);
}

static void remove_debug_file(struct dm320* dm320)
{
    spin_lock(&dm320->lock);

    if (dm320->pde)
    {
        remove_proc_entry(proc_filename, NULL);
    }

    spin_unlock(&dm320->lock);
}

#endif

/*-------------------------------------------------------------------------*/
/* ------------------------------------------------------------------------*/
/*  DM340 specific */
/* ------------------------------------------------------------------------*/
static int dm320_initialize(struct dm320 *dm320)
{
    u32 regVal;
    u16              flags;

    flags = disable_interrupt();

    // clear existed interrupt if any.
    readb(USB_INTRUSB);
    writeb(0x00,USB_INTRUSBE);

    // clear session if any.
    writeb(0x00,USB_DEVCTL); 

    // put controller into peripheral mode to initiate a controller reset.
    writel(0x04, AHB_USBCTL); 
    mdelay(32);
    regVal = readl(AHB_USBCTL);
    mdelay(1);
    writel(regVal, AHB_USBCTL);
    mdelay(1);
    regVal &=~ 0x4;
    writel(regVal, AHB_USBCTL);
    mdelay(1);
 
    // 0x0040 : USB clock enable
    outw(inw(IO_CLK_MOD2) | 0x0060, IO_CLK_MOD2);       

    // USB Power down mode off
    outw(inw(IO_CLK_LPCTL1) & 0xFFEF, IO_CLK_LPCTL1);   

    // USB controller in host mode.
    writel(readl(AHB_USBCTL)|0x00000008, AHB_USBCTL);   

    writeb(USB_REG_NAK_LIMIT, USB_NAKLMT0);

    // clear session if any.
    writeb(0x00,USB_DEVCTL);   
    
    enable_interrupt(flags);

    return 0;
}

static int dm320_enable_intrs(struct dm320 *dm320)
{
    unsigned long int_flags;

    spin_lock_irqsave(&lock, int_flags);
    outw(inw(IO_INTC_FISEL0) & 0x7fff, IO_INTC_FISEL0); // USB INT IRQ selected
    writeb(0x00,USB_INTRUSBE);
    writeb((readb(USB_INTRTX1E) | USB_EP0), USB_INTRTX1E);
    writeb(0x3e,USB_INTRUSBE);
    spin_unlock_irqrestore(&lock, int_flags);

    return 0;
}

static void dm320h_stop(struct usb_hcd *hcd)
{
    struct dm320*    dm320;
    u16              flags;

    flags = disable_interrupt();

    dm320 = hcd_to_dm320(hcd);
    spin_lock(&dm320->lock);

    del_timer_sync(&hcd->rh_timer);
    port_power(dm320, 0);

    spin_unlock(&dm320->lock);
    enable_interrupt(flags);
}

static int dm320h_start(struct usb_hcd *hcd)
{
    struct dm320*   dm320;
    u16              flags;

    flags = disable_interrupt();

    dm320 = hcd_to_dm320(hcd);
    spin_lock(&dm320->lock);

    dm320_initialize(dm320);
    dm320_enable_intrs(dm320);

    /* chip has been reset, VBUS power is off */
    hcd->state = HC_STATE_RUNNING;

    if (dm320->board) 
    {
        hcd->can_wakeup = dm320->board->can_wakeup;
        hcd->power_budget = dm320->board->power * 2;
    }

    /* enable power and interupts */
    port_power(dm320, 1);

    spin_unlock(&dm320->lock);
    enable_interrupt(flags);

    return 0;
}


/*-------------------------------------------------------------------------*/

static struct hc_driver dm320h_hc_driver = {
    .description =          hcd_name,
    .hcd_priv_size =        sizeof(struct dm320),

    /*
     * generic hardware linkage
     */
    .irq =                  dm320h_irq,
    .flags =                HCD_USB2 | HCD_MEMORY,

    /* Basic lifecycle operations */
    .start =                dm320h_start,
    .stop =                 dm320h_stop,

    /*
     * managing i/o requests and associated device resources
     */
    .urb_enqueue =          dm320h_urb_enqueue,
    .urb_dequeue =          dm320h_urb_dequeue,
    .endpoint_disable =     dm320h_endpoint_disable,

    /*
     * periodic schedule support
     */
    .get_frame_number =     dm320h_get_frame,

    /*
     * root hub support
     */
    .hub_status_data =      dm320h_hub_status_data,
    .hub_control =          dm320h_hub_control,
    .bus_suspend =          dm320h_bus_suspend,
    .bus_resume =           dm320h_bus_resume,
};

/*-------------------------------------------------------------------------*/

static int __devexit dm320h_remove(struct platform_device *dev)
{
    struct usb_hcd      *hcd;
    struct dm320        *dm320;
    u16              flags;

    flags = disable_interrupt();
    spin_lock(&dm320->lock);

    hcd = platform_get_drvdata(dev);
    dm320 = hcd_to_dm320(hcd);

#ifndef STUB_DEBUG_FILE
    remove_debug_file(dm320);
#endif
    usb_remove_hcd(hcd);

    usb_put_hcd(hcd);
    
    outw(inw(IO_CLK_LPCTL1) | 0x0010, IO_CLK_LPCTL1);   // USB Power down mode off
    outw(inw(IO_CLK_MOD2) & 0xffbf, IO_CLK_MOD2);       //0x0040 : USB clock disable
    writeb(0x00,USB_INTRUSBE);

    enable_interrupt(flags);
    spin_unlock(&dm320->lock);

    return 0;
}

static int __devinit dm320h_probe(struct platform_device *dev)
{
    struct usb_hcd  *hcd;
    struct dm320    *dm320;
    struct resource *addr;
    void __iomem    *addr_reg;
    int             retval = 0;

    /* basic sanity checks first.  board-specific init logic should
     * have initialized these three resources and probably board
     * specific platform_data.  we don't probe for IRQs, and do only
     * minimal sanity checking.
     */

    usb_core_irq_num = platform_get_irq(dev, 0);
    if (dev->num_resources < 2 || usb_core_irq_num < 0)
    {
        retval =  -ENODEV;
        goto out;
    }
    disable_interrupt();        // must have usb_core_irq_num first

    /* refuse to confuse usbcore */
    if (dev->dev.dma_mask) 
    {
        DBG("no we won't dma\n");
        retval = -EINVAL;
        goto out;
    }

    /* the chip may be wired for either kind of addressing */
    addr = platform_get_resource(dev, IORESOURCE_MEM, 0);
    if (!addr) 
    {
        retval = -ENODEV;
        goto out;
    }

    /* allocate and initialize hcd */
    hcd = usb_create_hcd(&dm320h_hc_driver, &dev->dev, dev->dev.bus_id);
    if (!hcd) 
    {
        retval = -ENOMEM;
        goto out;
    }
    hcd->rsrc_start = addr->start;
    dm320 = hcd_to_dm320(hcd);
    memset(dm320, 0, sizeof(struct dm320));

    spin_lock_init(&dm320->lock);
    INIT_LIST_HEAD(&dm320->async);
    dm320->board = dev->dev.platform_data;
    addr_reg = (unsigned int*)0x80000000;
    dm320->addr_reg = addr_reg;
    dm320->data_reg = addr_reg;

    init_control_vars(dm320);

#ifndef STUB_DEBUG_FILE
    create_debug_file(dm320);
#endif

    //interrupt is enabled by usb_add_hcd
    //enable_interrupt(1 << usb_core_irq_num);

    // this enables interrupt.
    usb_add_hcd(hcd, usb_core_irq_num, SA_INTERRUPT);
out:
    return retval;
}

#ifdef    CONFIG_PM

/* for this device there's no useful distinction between the controller
 * and its root hub, except that the root hub only gets direct PM calls
 * when CONFIG_USB_SUSPEND is enabled.
 */

static int dm320h_suspend(struct platform_device *dev, pm_message_t state)
{
    struct usb_hcd  *hcd;
    struct dm320    *dm320;
    int             retval = 0;
    u16              flags;

    flags = disable_interrupt();
    spin_lock(&dm320->lock);

    hcd = platform_get_drvdata(dev);
    dm320 = hcd_to_dm320(hcd);

    if (state.event == PM_EVENT_FREEZE)
    {
        retval = dm320h_bus_suspend(hcd);
    }
    else if (state.event == PM_EVENT_SUSPEND)
    {
        port_power(dm320, 0);
    }
    if (retval == 0)
    {
        dev->dev.power.power_state = state;
    }

    spin_unlock(&dm320->lock);
    enable_interrupt(flags);
    return retval;
}

static int dm320h_resume(struct platform_device *dev)
{
    struct usb_hcd  *hcd;
    struct dm320    *dm320;
    u16              flags;

    flags = disable_interrupt();
    spin_lock(&dm320->lock);

    hcd = platform_get_drvdata(dev);
    dm320 = hcd_to_dm320(hcd);

    /* with no "check to see if VBUS is still powered" board hook,
     * let's assume it'd only be powered to enable remote wakeup.
     */
    if (dev->dev.power.power_state.event == PM_EVENT_SUSPEND ||
        !hcd->can_wakeup) 
    {
        dm320->port1 = 0;
        port_power(dm320, 1);
        return 0;
    }

    dev->dev.power.power_state = PMSG_ON;

    spin_unlock(&dm320->lock);
    enable_interrupt(flags);
    return(dm320h_bus_resume(hcd));
}

#else

#define    dm320h_suspend   NULL
#define    dm320h_resume    NULL

#endif


/* this driver is exported so dm320_cs can depend on it */
struct platform_driver dm320h_driver = {
    .probe =    dm320h_probe,
    .remove =   __devexit_p(dm320h_remove),

    .suspend =  dm320h_suspend,
    .resume =   dm320h_resume,
    .driver =   {
        .name =     (char *) hcd_name,
        .owner =    THIS_MODULE,
    },
};
EXPORT_SYMBOL(dm320h_driver);


static int usbhost_proc_read(char * page, char ** start, off_t offset,
        int count, int * eof, void * data)
{
  int length = 0;

   if ( proc_action == USB_PLUG__ )
    length = sprintf( page, "%s\n", "add" );
  else /* proc_action == USB_PULL__ */
    length = sprintf( page, "%s\n", "remove" );

  *eof = 1;
  return length;
}

static void proc_usbhost_init( void )
{
  create_proc_read_entry( "usbhost", 0, 0, usbhost_proc_read, (void *)NULL );
}

/*-------------------------------------------------------------------------*/

static int __init dm320h_init(void)
{
    if (usb_disabled())
    {
        return -ENODEV;
    }

    proc_usbhost_init();

#ifdef USE_VBUS_GIO
    if(request_gio(GIO_USB_POWER))
        ERR("GIO %x has been used\n", GIO_USB_POWER);
    gio_set_dir(GIO_USB_POWER, 0);   
    gio_set_bitset(GIO_USB_POWER);
#endif

    INFO("driver %s, %s\n", hcd_name, DRIVER_VERSION);
    return platform_driver_register(&dm320h_driver);
}
module_init(dm320h_init);

static void __exit dm320h_cleanup(void)
{
    disable_interrupt();

    remove_proc_entry("usbhost", 0);
#ifdef USE_VBUS_GIO
    unrequest_gio(GIO_USB_POWER);
#endif
    platform_driver_unregister(&dm320h_driver);
    if (devdescsave) kfree(devdescsave);
}
module_exit(dm320h_cleanup);
