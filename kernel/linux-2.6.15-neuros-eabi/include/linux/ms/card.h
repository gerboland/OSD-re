/*
 *  linux/include/linux/ms/card.h
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 *  Card driver specific definitions.
 */
#ifndef LINUX_MS_CARD_H
#define LINUX_MS_CARD_H

#include <linux/ms/ms.h>
#include <linux/ms/mem_stick_reg.h>

struct ms_host;

/*
 * MS device
 */
struct ms_card {
	struct list_head	node;		/* node in hosts devices list */
	struct ms_host		*host;		/* the host this device belongs to */
	struct device		dev;		/* the device */
	//unsigned int		rca;		/* relative card address of device */
	unsigned int		state;		/* (our) card state */

#define MS_STATE_PRESENT	(1<<0)		/* present in sysfs */
#define MS_STATE_DEAD		(1<<1)		/* device no longer in stack */
#define MS_STATE_BAD		(1<<2)		/* unrecognised device */
#define MS_STATE_MSCARD		(1<<3)		/* is an Memory stick pro card */
#define MS_STATE_PROCARD	(1<<4)		/* is an Memory stick pro card */
#define MS_STATE_READONLY	(1<<5)		/* card is read-only */

	struct system_information sys_info;	/* 96-bytes of card information*/
	int cur_blk_no;
	unsigned char *cur_blk_data;
};

#define ms_card_present(c)	((c)->state & MS_STATE_PRESENT)
#define ms_card_dead(c)		((c)->state & MS_STATE_DEAD)
#define ms_card_bad(c)		((c)->state & MS_STATE_BAD)
#define ms_card_ms_pro(c)	((c)->state & MS_STATE_PROCARD)
#define ms_card_ms(c)		((c)->state & MS_STATE_MSCARD)
#define ms_card_readonly(c)	((c)->state & MS_STATE_READONLY)

#define ms_card_set_present(c)	((c)->state |= MS_STATE_PRESENT)
#define ms_card_set_dead(c)	((c)->state |= MS_STATE_DEAD)
#define ms_card_set_bad(c)	((c)->state |= MS_STATE_BAD)
#define ms_card_set_ms_pro(c)	((c)->state |= MS_STATE_PROCARD)
#define ms_card_set_ms(c)	((c)->state |= MS_STATE_MSCARD)
#define ms_card_set_readonly(c) ((c)->state |= MS_STATE_READONLY)

//#define ms_card_name(c)		((c)->cid.prod_name)
#define ms_card_id(c)		((c)->dev.bus_id)

#define ms_list_to_card(l)	container_of(l, struct ms_card, node)
#define ms_get_drvdata(c)	dev_get_drvdata(&(c)->dev)
#define ms_set_drvdata(c,d)	dev_set_drvdata(&(c)->dev, d)


/*
 * MS device driver (e.g., Flash card, I/O card...)
 */
struct ms_driver {
	struct device_driver drv;
	int (*probe)(struct ms_card *);
	void (*remove)(struct ms_card *);
	int (*suspend)(struct ms_card *, pm_message_t);
	int (*resume)(struct ms_card *);
};

extern int ms_register_driver(struct ms_driver *);
extern void ms_unregister_driver(struct ms_driver *);

static inline int ms_card_claim_host(struct ms_card *card)
{
	return __ms_claim_host(card->host, card);
}

#define ms_card_release_host(c)	ms_release_host((c)->host)

#endif
