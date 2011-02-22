
#ifndef  __ITNAV_H


/* IT320 has two nav switches. User application can use poll system call
 * to wait on or read events generated by navigation buttons. 
 * An event is generated when any of the primary nav buttons is pressed, and
 * the returned evevt data consists of the button info of the primary 
 * nav switch and state of 2nd nav switch at the time of the button being 
 * pressed.
 */ 

typedef int itn_event_t;

#define NAV_PKEY_MASK	 		0xFF
#define NAV_PKEY_NONE  			0
#define NAV_PKEY_RIGHT_PRD 		1
#define NAV_PKEY_RIGHT_REL 		2
#define NAV_PKEY_RIGHT_RPT 		3
#define NAV_PKEY_UP_PRD   		4
#define NAV_PKEY_UP_REL   		5
#define NAV_PKEY_UP_RPT   		6
#define NAV_PKEY_LEFT_PRD 		7
#define NAV_PKEY_LEFT_REL  		8
#define NAV_PKEY_LEFT_RPT  		9
#define NAV_PKEY_DOWN_PRD  	       10
#define NAV_PKEY_DOWN_REL  	       11
#define NAV_PKEY_DOWN_RPT  	       12
#define NAV_PKEY_SEL_PRD 	       13
#define NAV_PKEY_SEL_REL   	       14
#define NAV_PKEY_SEL_RPT   	       15

#define NAV_SKEY_RIGHT_PRD 	       16
#define NAV_SKEY_RIGHT_REL 	       17
#define NAV_SKEY_RIGHT_RPT 	       18
#define NAV_SKEY_UP_PRD   	       19
#define NAV_SKEY_UP_REL   	       20
#define NAV_SKEY_UP_RPT   	       21
#define NAV_SKEY_LEFT_PRD  	       22
#define NAV_SKEY_LEFT_REL  	       23
#define NAV_SKEY_LEFT_RPT  	       24
#define NAV_SKEY_DOWN_PRD  	       25
#define NAV_SKEY_DOWN_REL  	       26
#define NAV_SKEY_DOWN_RPT  	       27
#define NAV_SKEY_SEL_PRD  	       28
#define NAV_SKEY_SEL_REL  	       29
#define NAV_SKEY_SEL_RPT  	       30

#define NAV_PKEY_RIGHT 	 	  	0
#define NAV_PKEY_UP   			1
#define NAV_PKEY_LEFT 			2
#define NAV_PKEY_DOWN  			3
#define NAV_PKEY_SEL 			4

#define NAV_SKEY_RIGHT 			5
#define NAV_SKEY_UP   			6
#define NAV_SKEY_LEFT  			7
#define NAV_SKEY_DOWN  			8
#define NAV_SKEY_SEL  	       	        9

struct repeat_rate
{
        int key;
        int delay;
        int auto_repeat_rate;
};

struct set_event_mask
{
        int mask;
};

static itn_event_t key_evt_map[] = {
                NAV_PKEY_RIGHT_PRD,
                NAV_PKEY_RIGHT_REL,
                NAV_PKEY_RIGHT_RPT,
                NAV_PKEY_UP_PRD,
                NAV_PKEY_UP_REL,
                NAV_PKEY_UP_RPT,
                NAV_PKEY_LEFT_PRD,
                NAV_PKEY_LEFT_REL,
                NAV_PKEY_LEFT_RPT,
                NAV_PKEY_DOWN_PRD,
                NAV_PKEY_DOWN_REL,
                NAV_PKEY_DOWN_RPT,
                NAV_PKEY_SEL_PRD,
                NAV_PKEY_SEL_REL,
                NAV_PKEY_SEL_RPT,
                NAV_SKEY_RIGHT_PRD,
                NAV_SKEY_RIGHT_REL,
                NAV_SKEY_RIGHT_RPT,
                NAV_SKEY_UP_PRD,
                NAV_SKEY_UP_REL,
                NAV_SKEY_UP_RPT,
                NAV_SKEY_LEFT_PRD,
                NAV_SKEY_LEFT_REL,
                NAV_SKEY_LEFT_RPT,
                NAV_SKEY_DOWN_PRD,
                NAV_SKEY_DOWN_REL,
                NAV_SKEY_DOWN_RPT,
                NAV_SKEY_SEL_PRD,
                NAV_SKEY_SEL_REL,
                NAV_SKEY_SEL_RPT
};


#define NAV_IOCTL_MAGIC  0xEE
//#define SET_EVENT_MASK  	_IOWR(NAV_IOCTL_MAGIC,1,int)
#define SET_EVENT_MASK  	NAV_IOCTL_MAGIC+1
#define SET_EVENT_UMASK  	NAV_IOCTL_MAGIC+2
#define GET_EVENT_MASK  	NAV_IOCTL_MAGIC+3
#define SET_REPEAT_RATE 	NAV_IOCTL_MAGIC+4
#define GET_REPEAT_RATE 	NAV_IOCTL_MAGIC+5
#define GET_KEYPAD_STATUS 	NAV_IOCTL_MAGIC+6
#define GET_KEY_EVENT 		NAV_IOCTL_MAGIC+7

#define NAV_AKEY_MASK  0xFF00
#define NAV_AKEY_STATE(key) (1<<(8+(key) - 1))

#ifndef __KERNEL__
#define is_right_key_pressed(evt) ((evt)&NAV_PKEY_MASK == NAV_PKEY_RIGHT)
#define is_left_key_pressed(evt) ((evt)&NAV_PKEY_MASK == NAV_PKEY_LEFT)
#define is_up_key_pressed(evt) ((evt)&NAV_PKEY_MASK == NAV_PKEY_UP)
#define is_down_key_pressed(evt) ((evt)&NAV_PKEY_MASK == NAV_PKEY_DOWN)
#define is_sel_key_pressed(evt) ((evt)&NAV_PKEY_MASK == NAV_PKEY_SEL)
#endif

#endif /* __ITNAV_H */
