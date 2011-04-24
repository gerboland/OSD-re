/*
 *  linux/drivers/davinci/i2c_davinci_client.c
 *
 * Copyright (C) 2004 Texas Instruments Inc
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
/* i2c_davinci_client.c */

#include <linux/config.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/vmalloc.h>
#include <linux/slab.h>
#include <linux/proc_fs.h>
#include <linux/ctype.h>
#include <linux/delay.h>
#include <linux/i2c.h>

#include <asm/arch/gios.h>
#include <asm/arch/gio.h>

//#define DAVINCI_I2C_TEST

/* The EEPROM Serial Bus */
struct davinci_bus_ops {
	unsigned char     version;
	int               (*init)(void);
	void              (*cleanup)(void);
	int               (*read)(u8 size, u8 *val);
	int               (*write)(u8 size, u8 *val);
};

struct davinci_i2c_param {
	/* I2C parameters */
	struct            i2c_client client;
	struct            i2c_driver driver;
};


static struct davinci_i2c_param davinci_i2c_dev;

struct i2c_client *davinci_i2c_adapter = &davinci_i2c_dev.client;

struct i2c_client *davinci_i2c_get_client( void )
{
    return &davinci_i2c_dev.client;
}
EXPORT_SYMBOL( davinci_i2c_get_client );

static int
davinci_i2c_read_reg(u8 size, u8 *val)
{
	int               err;
	struct            i2c_client *client = &davinci_i2c_dev.client;

	struct i2c_msg msg[1];
	//    unsigned char data[65] = { 0x00 , 0x00 };

	if (!client->adapter)
	return -ENODEV;

	/*msg->addr = client->addr;
	msg->flags = 0;
	msg->len = 2;
	msg->buf = val;

	err = i2c_transfer(client->adapter, msg, 1);

	if (err >= 0) {*/
	msg->len = size;
	msg->flags = I2C_M_RD;
	err = i2c_transfer(client->adapter, msg, 1);
	/*}*/
	if (err >= 0) {
	//        *val = *data;
	return 0;
	}

	return err;
}

static int
davinci_i2c_write_reg(u8 size, u8 * val)
{
	int            	err;
	struct         	i2c_client *client = &davinci_i2c_dev.client;

	struct 		i2c_msg msg[1];
#ifdef CONFIG_DAVINCI_BLK_DEV_CF
	char   		cmd[4] = { 4, 6, 0x00, 0x09 };
#endif
        u8              my_value;
        int             gio;

	if (!client->adapter)
	return -ENODEV;

#ifdef CONFIG_DAVINCI_BLK_DEV_CF

	msg->addr = 0x23;
	msg->flags = 0;
	msg->len = 4;
	msg->buf = cmd;

	err = i2c_transfer(client->adapter, msg, 1);

	if (err < 0)
	return err;
#endif

        /* DM420 - Enable TVP5160 */
        /* two-step process - first de-assert PWRDN, then */
        /* de-assert TVP5160 reset */
	msg->addr = 0x3B;
	msg->flags = 0;
	msg->len = 1;
        my_value = 0x3f;
	msg->buf = &my_value;

	err = i2c_transfer(client->adapter, msg, 1);
	if (err <= 0) {
            printk( "%s: Write to address 0x%x failed.\n",
                    __FUNCTION__, msg->addr );
        }

        udelay( 50 );

	msg->addr = 0x3B;
	msg->flags = 0;
	msg->len = 1;
        my_value = 0x7f;
	msg->buf = &my_value;

	err = i2c_transfer(client->adapter, msg, 1);
	if (err <= 0) {
            printk( "%s: Write to address 0x%x failed.\n",
                    __FUNCTION__, msg->addr );
        }


        /* DM420 - Enable on-board McBSP buffers */
        my_value = 0xF1; 
	msg->addr = 0x3D;
	msg->flags = 0;
	msg->len = 1;
	msg->buf = &my_value;

	err = i2c_transfer(client->adapter, msg, 1);

	if (err <= 0) {
            printk( "%s: Write to address 0x%x failed.\n",
                    __FUNCTION__, msg->addr );
        }

        udelay( 100 );

#if 1
        /* DM420 - Setup PLL1705 (audio clocking) */
        gio = GIO_EXP_PLL1705_FS2;
        if ( 0 == request_gio( gio ) ) {
            gio_set_dir( gio, bit_low );
            gio_set_bitclr( gio );
        } else {
            printk( "%s: Request GIO %d failed.\n",
                    __FUNCTION__, gio );
        }

        gio = GIO_EXP_PLL1705_CSEL;
        if ( 0 == request_gio( gio ) ) {
            gio_set_dir( gio, bit_low );
            gio_set_bitset( gio );
        } else {
            printk( "%s: Request GIO %d failed.\n",
                    __FUNCTION__, gio );
        }

        gio = GIO_EXP_PLL1705_SR;
        if ( 0 == request_gio( gio ) ) {
            gio_set_dir( gio, bit_low );
            gio_set_bitclr( gio );
        } else {
            printk( "%s: Request GIO %d failed.\n",
                    __FUNCTION__, gio );
        }
#else

        my_value = 0x04; 
	msg->addr = 0x3E;
	msg->flags = 0;
	msg->len = 1;
	msg->buf = &my_value;
	err = i2c_transfer(client->adapter, msg, 1);

	if (err <= 0) {
            printk( "%s: Write to address 0x%x failed.\n",
                    __FUNCTION__, msg->addr );
        }
#endif


	return err;
}

static int
davinci_i2c_attach_client(struct i2c_adapter *adap, int addr)
{
	struct davinci_i2c_param *davinci_i2c_if = &davinci_i2c_dev;
	struct i2c_client *client = &davinci_i2c_if->client;
	int err;

	if (client->adapter)
		return -EBUSY;  /* our client is already attached */

	client->addr = addr;
	client->flags = I2C_CLIENT_ALLOW_USE;
	client->driver = &davinci_i2c_if->driver;
	client->adapter = adap;

	err = i2c_attach_client(client);
	if (err) {
		client->adapter = NULL;
		return err;
	}

	return 0;
}

static int
davinci_i2c_detach_client(struct i2c_client *client)
{
	int err;

	if (!client->adapter)
	  return -ENODEV; /* our client isn't attached */

	err = i2c_detach_client(client);
	client->adapter = NULL;
	return err;
}

static int
davinci_i2c_probe_adapter(struct i2c_adapter *adap)
{
/*	    return davinci_i2c_attach_client(adap, 0x50);*/
	return davinci_i2c_attach_client(adap, 0x3B);

}

static int
davinci_i2c_init(void)
{
	int err;
	struct i2c_driver *driver = &davinci_i2c_dev.driver;

	driver->owner = THIS_MODULE;
	strlcpy(driver->name, "Davinci I2C driver", sizeof(driver->name));
	driver->id = I2C_DRIVERID_EXP0;
	driver->flags = I2C_DF_NOTIFY;
	driver->attach_adapter = davinci_i2c_probe_adapter;
	driver->detach_client = davinci_i2c_detach_client;

	err = i2c_add_driver(driver);
	if (err) {
	  printk(KERN_ERR "Failed to register Davinci I2C client.\n");
	  return err;
	}
	return 0;
}

static void davinci_i2c_cleanup (void)
{
	i2c_detach_client(&davinci_i2c_dev.client);
	davinci_i2c_dev.client.adapter = NULL;

	return;
}


struct davinci_bus_ops davinci_i2c_fops = {
	.version = 0x01,
	.init    = davinci_i2c_init,
	.cleanup = davinci_i2c_cleanup,
	.read    = davinci_i2c_read_reg,
	.write   = davinci_i2c_write_reg,
};

static int __init
davinci_i2c_perienable(void)
{
	char			value = 0;

	/* enable the ATA access through I2C */
	davinci_i2c_fops.init ();

        value = 0x1f; /* for DM420 */

	davinci_i2c_fops.write(1,&value);

	return 0;
}

module_init(davinci_i2c_perienable);




