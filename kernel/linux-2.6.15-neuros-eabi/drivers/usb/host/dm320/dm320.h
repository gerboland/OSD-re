/*
 * DM320HS register declarations and HCD data structures
 *
 * Copyright (C) 2004 Psion Teklogix
 * Copyright (C) 2004 David Brownell
 * Copyright (C) 2001 Cypress Semiconductor Inc. 
 */

/*
 * DM320HS has transfer registers, and control registers.  In host/master
 * mode one set of registers is used; in peripheral/slave mode, another.
 *  - SL11H only has some "A" transfer registers from 0x00-0x04
 *  - DM320HS also has "B" registers from 0x08-0x0c
 *  - SL811S (or HS in slave mode) has four A+B sets, at 00, 10, 20, 30
 */

#include "dm320_reg_def.h"

#define H_MAXPACKET         120        /* bytes in A or B fifos */

/*-------------------------------------------------------------------------*/

#define    LOG2_PERIODIC_SIZE    5    /* arbitrary; this matches OHCI */
#define    PERIODIC_SIZE        (1 << LOG2_PERIODIC_SIZE)

struct dm320 {
    spinlock_t                  lock;
    void __iomem                *addr_reg;
    void __iomem                *data_reg;
    struct dm320_platform_data  *board;
    struct proc_dir_entry       *pde;

    unsigned long               stat_insrmv;
    unsigned long               stat_wake;
    unsigned long               stat_sof;
    unsigned long               stat_a;
    unsigned long               stat_b;
    unsigned long               stat_lost;
    unsigned long               stat_overrun;

    /* sw model */
    struct timer_list           timer;
    struct dm320h_ep            *next_periodic;
    struct dm320h_ep            *next_async;

    struct dm320h_ep            *active_a;
    unsigned long               jiffies_a;

    u32                         port1;
    u16                         frame;

    /* async schedule: control, bulk */
    struct list_head            async;
    unsigned int                uses_new_polling:1;
    /* periodic schedule: interrupt, iso */
    u16                         load[PERIODIC_SIZE];
    struct dm320h_ep            *periodic[PERIODIC_SIZE];
    unsigned                    periodic_count;
    
    int                         getconfigdescFlag;
    int                         getdevdescFlag;
    int                         grecvd;
    char                        gdata[64];
    struct usb_device_descriptor *devdesc;
    struct usb_config_descriptor *confdesc;
    struct usb_interface_descriptor *intdesc;
    struct usb_endpoint_descriptor *endptdesc[10];
    int                         numendpts;
};

static inline struct dm320 *hcd_to_dm320(struct usb_hcd *hcd)
{
    return (struct dm320 *) (hcd->hcd_priv);
}

static inline struct usb_hcd *dm320_to_hcd(struct dm320 *dm320)
{
    return container_of((void *) dm320, struct usb_hcd, hcd_priv);
}

struct dm320h_ep {
    struct usb_host_endpoint *hep;
    struct usb_device   *udev;

    u8                  defctrl;
    u8                  maxpacket;
    u8                  epnum;
    u8                  nextpid;

    u16                 error_count;
    u16                 nak_count;
    u16                 length; /* of current packet */

    /* periodic schedule */
    u16                 period;
    u16                 branch;
    u16                 load;
    u32                 toggle;
    struct dm320h_ep    *next;

    /* async schedule */
    struct list_head    schedule;
};

/*-------------------------------------------------------------------------*/

/* These register utilities should work for the SL811S register API too
 * NOTE:  caller must hold dm320->lock.
 */

static inline void
dm320_write_buf(struct dm320 *dm320, int addr, const void *buf, size_t count)
{
    const u8    *data;
    void __iomem    *data_reg;

    if (!count)
        return;
    writeb(addr, dm320->addr_reg);

    data = buf;
    data_reg = dm320->data_reg;
    do {
        writeb(*data++, data_reg);
    } while (--count);
}

static inline void
dm320_read_buf(struct dm320 *dm320, int addr, void *buf, size_t count)
{
    u8         *data;
    void __iomem    *data_reg;

    if (!count)
        return;
    writeb(addr, dm320->addr_reg);

    data = buf;
    data_reg = dm320->data_reg;
    do {
        *data++ = readb(data_reg);
    } while (--count);
}

/*-------------------------------------------------------------------------*/

//#define DEBUG    1
#undef DEBUG
#ifdef DEBUG
#define DBG(stuff...)        printk(KERN_DEBUG "dm320: " stuff)
#else
#define DBG(stuff...)        do{}while(0)
#endif

//#define NEW_DEBUG
#ifdef NEW_DEBUG
#define N_DBG(stuff...)           printk(stuff)
#else
#define N_DBG(stuff...)           do{}while(0)
#endif

#ifdef VERBOSE
#    define VDBG        DBG
#else
#    define VDBG(stuff...)    do{}while(0)
#endif

#ifdef PACKET_TRACE
#    define PACKET        VDBG
#else
#    define PACKET(stuff...)    do{}while(0)
#endif

#define ERR(stuff...)       printk(KERN_ERR "dm320: " stuff)
#define WARN(stuff...)      printk(KERN_WARNING "dm320: " stuff)
#define INFO(stuff...)      printk(KERN_INFO "dm320: " stuff)

