
/** 
  
struct MGC_EpFifoDescriptor {
    u8 bType; 	0 for autoconfig, CNTR, ISOC, BULK, INTR 
    u8 bDir; 	0 for autoconfig, INOUT, IN, OUT 
    int wSize;		0 for autoconfig, or the size 
};

#define MUSB_EPD_AUTOCONFIG	0

#define MUSB_EPD_T_CNTRL	1
#define MUSB_EPD_T_ISOC		2
#define MUSB_EPD_T_BULK		3
#define MUSB_EPD_T_INTR		4

#define MUSB_EPD_D_INOUT	0
#define MUSB_EPD_D_TX		1
#define MUSB_EPD_D_RX		2
*/
struct MUSB_EpFifoDescriptor MUSB_aEpFifoDescriptors[MUSB_C_NUM_EPS] = {

	{},			/* EP0 use the default */
	{MUSB_EPD_T_BULK, MUSB_EPD_D_TX, 512},
	{MUSB_EPD_T_BULK, MUSB_EPD_D_RX, 512}

};
