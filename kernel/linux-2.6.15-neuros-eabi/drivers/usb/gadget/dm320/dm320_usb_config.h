#ifndef 	_LINUX_DD_DM320_USB_CONFIG_H_
#define 	_LINUX_DD_DM320_USB_CONFIG_H_

// These GIO defines are for Ingenient's borad
#define		GIO_USB_ATTACH			6						// USB attach/detach detection => GIO 6
#define		IRQ_USB_ATTACH			IRQ_GIO6				// External Interrupt #6 ID 
#define		GIO_DP_PULLUP			GIO_USB_ENABLE			// Ingenient's USB D+ pullup register control => GIO 17

#define		EP0_FIFO_SIZE			64						// don't change this value ( from DM320 TRM )

#define 	TX_EP_MAX				5						// number of max. endpoints
#define		RX_EP_MAX				TX_EP_MAX				// number of max. endpoints


			
//#define		BULK_TRANS_SIZE			64						// from USB specifiation and DM320 TRM
#define		BULK_TRANS_SIZE			64						// from USB specifiation and DM320 TRM
#define		INTR_TRANS_SIZE			64						// from USB specifiation and DM320 TRM
#define		BULK_BUF_SIZE		  (1024*1024)				// bulk transmission buffer size each for TX and RX
															
// Vendor ID & Product ID to find proprietry USB device.  Check your own vendor ID & product ID
#define		USER_VENDOR_ID			0xD320					// vendor  ID is 0xD320 ( means DM320 )
#define		USER_PRODUCT_ID			0x3211					// product ID is 0x3211 ( means DM320 & 1.1 )

// Endpoint number for TX and RX
#define		BULK_OUT_EP				0x02					// output endpoint for DM320 ( IN w.r.t USB specification )
#define		BULK_IN_EP				0x01					// input endpoint for DM320 ( OUT w.r.t USB specification )
#define		INTR_IN_EP				0x03

// Buffer ID
#define		ID_BULK_IN				0						// TX buffer ID ( IN w.r.t USB specification )
#define		ID_BULK_OUT				1						// RX buffer ID ( OUT w.r.t USB specification )


#endif 















