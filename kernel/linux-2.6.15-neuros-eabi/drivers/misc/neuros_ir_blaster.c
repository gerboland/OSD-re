/*
 *  Copyright(C) 2006-2007 Neuros Technology International LLC. 
 *               <www.neurostechnology.com>
 *
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, version 2 of the License.
 *
 *  This program is distributed in the hope that, in addition to its 
 *  original purpose to support Neuros hardware, it will be useful 
 *  otherwise, but WITHOUT ANY WARRANTY; without even the implied 
 *  warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  
 *  See the GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 *
 ****************************************************************************
 *
 * IR BLASTER driver.
 *
 * REVISION:
 * 3) Split from irrtc driver ----------------------------- 2006-12-20 nerochiaro
 * 2) proc interupt support ------------------------------- 2006-06-20 EY
 * 1) Initial creation. ----------------------------------- 2005-06-02 MG 
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/poll.h>
#include <asm/io.h>
#include <asm/arch/io_registers.h>
#include <asm/arch/irqs.h>

#include <linux/neuros_ir.h>
#include <linux/neuros_ir_blaster.h>

#define MOD_DESC "Neuros IR Blaster Driver (c) 2006"

#if 0
    #define dbg(fmt, arg...) \
        printk(KERN_INFO "%s:%d> " fmt, __func__, __LINE__ , ## arg)
#else
    #define dbg(fmt, arg...)
#endif

#define LEARNING_THROUGH_MSP430 0
#define WAIT_BLASTER_TIME 10
#define DELAY_FOR_IR 200

#ifdef BLASTER_THROUGH_ARM
#define LEANRING_COMPLETE_DELAY (HZ)
#define POLL_RELEASE_DELAY (HZ/10)
#define KEY_WAVE_PRESENT 1
#define WAIT_LEARN_COMPLETE 2
#define WAIT_RELEASE_REMOTE 4
#define GIO34 0x0004
#define BLS_TIMER_PRESCALE 26
#define BLASTER 0
#define CAPTURE 1
#define INT_TIMER1_TIME_WASTE 3
#define INT_CAPTURE_TIME_WASTE 2
#define MAX_COUNTER_BIT 16
#define MAX_COUNTER ((1<<MAX_COUNTER_BIT)-1)
#define WAIT_HARDWARE_RESET 50 //50ms
static int learning_status = 0;
static int bls_wave_count;
static int bls_status=BLS_COMPLETE;
static int timer_int_counter=0;
static int wave_len=0;
static int int_type;
#else
#define LEANRING_COMPLETE_DELAY (HZ/10)
static uint8_t real_key = 0;
#endif
static struct timer_list learning_key_timer;
static struct blaster_data_type *BlsKey;
static struct blaster_data_pack *bls_data_pack;

struct irrtc_device {
    int    key;
};
static struct irrtc_device device;
static wait_queue_head_t wait;

extern void set_osd_key(int);
extern int get_osd_key(void);
extern void lock_data_protect(void);
extern void unlock_data_protect(void);
extern void set_factory_test(int);
extern void report_key(int key);

static void set_timer1_div(struct blaster_data_pack *blsdat)
{
	unsigned int div;
	int count;
	static int sbit;
	if(wave_len)
	{	
		div=wave_len;
		if(div>MAX_COUNTER)
			div=MAX_COUNTER;
		wave_len-=div;
	}
	else
	{
		count=blsdat->bitstimes&BITS_COUNT_MASK;
		if (count==bls_wave_count)
			sbit=0;
		if (keybit_get(blsdat->mbits, (count-bls_wave_count)))
		{
			if (keybit_get(blsdat->dbits, (count-bls_wave_count)))
			{
				div=blsdat->specbits[sbit];
				sbit++;
			} else
			{
				div=blsdat->bit2;
			}
		} else
		{
			if (keybit_get(blsdat->dbits, (count-bls_wave_count)))
			{
				div=blsdat->bit1;
			} else
			{
				div=blsdat->bit0;
			}
		}
	}
    outw(BLS_TIMER_PRESCALE, IO_TIMER1_TMPRSCL);
	//div-=div>>5;   // The ideal situation is that set the timer prescale to let the timer cycle is 1us 
								// but the nearest setting make the cycle 1.032us so do this adjustment with 
								// divider value
	div-=INT_TIMER1_TIME_WASTE;
	if(div>MAX_COUNTER)
	{
		wave_len=div-MAX_COUNTER;
		div=MAX_COUNTER;
	}
	div--;
	outw(div, IO_TIMER1_TMDIV);
}

static void blaster_key(struct blaster_data_pack* blsdat)
{ 
#ifdef BLASTER_THROUGH_ARM
	uint16_t bitset2;
	enable_irq(IRQ_TIMER1);
	wave_len=0;
	bls_status=BLS_START;
	bls_wave_count=blsdat->bitstimes&BITS_COUNT_MASK;
	if(bls_wave_count>BLASTER_MAX_CHANGE  || bls_wave_count<=0)
	{
		bls_status=BLS_ERROR;
		if(bls_data_pack)
		{
			kfree(bls_data_pack);
			bls_data_pack=NULL;
		}
		return;
	}
	outw( inw( IO_GIO_DIR2 ) & (~(GIO34)), IO_GIO_DIR2 );  //gio 34 direction output
	/*check if the io port status correct if not correct set it's logic to reverse of start level and hold for a momemt*/
	bitset2=inw( IO_GIO_BITSET2 );
	if (((bitset2 & GIO34)!=0)&&((bls_data_pack->bitstimes&BITS_COUNT_MASK)!=0))
	{
		outw(GIO34, IO_GIO_BITCLR2);
		msleep(WAIT_HARDWARE_RESET);
	}
	else if (((bitset2 & GIO34)==0)&&((bls_data_pack->bitstimes&BITS_COUNT_MASK)==0))
	{
		outw( GIO34, IO_GIO_BITSET2 ); 
		msleep(WAIT_HARDWARE_RESET);
	}
	if(blsdat->bitstimes & FIRST_LEVEL_BIT_MASK)
		outw( GIO34, IO_GIO_BITSET2 ); 
	else
		outw( GIO34, IO_GIO_BITCLR2 ); 
	set_timer1_div(blsdat);
	wmb();
    outw(2,IO_TIMER1_TMMD);

#else
	int i;
	uint8_t *a = (uint8_t*)blsdat;

	a = (uint8_t*)(&blsdat->bitstimes);
	dbg("data = %X\n", *(a));
	i2c_write(regBLASTER_DATA, *(a));
	dbg("data = %X\n", *(a+1));
	i2c_write(regBLASTER_DATA+1, *(a+1));
	a = (uint8_t*)(&blsdat->bit0);
	dbg("data = %X\n", *(a));
	i2c_write(regBLASTER_BIT0, *(a));
	dbg("data = %X\n", *(a+1));
	i2c_write(regBLASTER_BIT0+1, *(a+1));
	a = (uint8_t*)(&blsdat->bit1);
	dbg("data = %X\n", *(a));
	i2c_write(regBLASTER_BIT1, *(a));
	dbg("data = %X\n", *(a+1));
	i2c_write(regBLASTER_BIT1+1, *(a+1));
	a = (uint8_t*)(&blsdat->bit2);
	dbg("data = %X\n", *(a));
	i2c_write(regBLASTER_BIT2, *(a));
	dbg("data = %X\n", *(a+1));
	i2c_write(regBLASTER_BIT2+1, *(a+1));
	for (i=0;i<BLASTER_MAX_CHANGE/8;i++)
	{
		dbg("data = %X\n", blsdat->mbits[i]);
		i2c_write(regBLASTER_MBITS+i, blsdat->mbits[i]);
		dbg("data = %X\n", blsdat->mbits[i]);
		i2c_write(regBLASTER_DBITS+i, blsdat->dbits[i]);
	}
	a = (uint8_t*)(blsdat->specbits);
	for (i=0;i<BLASTER_MAX_SBITS*2;i++)
	{
		dbg("data = %X\n", *(a+i));
		i2c_write(regFLASH_DATA+i, *(a+i)); 
	}
	i2c_write(regCMND, cmdBLASTER_SEND); 
	msleep(100);
	i=0;
	while (i2c_read(regBLASTER_DATA)!=(*(uint8_t*)blsdat) && i<WAIT_BLASTER_TIME)
	{
		msleep(50);
		i++;
	}
#endif
}

void timer_handle(unsigned long data)
{
#ifdef BLASTER_THROUGH_ARM
	int osd_key;
	dbg("bitstimes=%d\n", BlsKey->bitstimes);
	if (learning_status & WAIT_LEARN_COMPLETE)
	{
		lock_data_protect();
		osd_key=get_osd_key();
		unlock_data_protect();
		if (!osd_key )
		{
			disable_irq(IRQ_TIMER1);
			report_key(LEARNING_COMPLETE_KEY);
			//report_key(UP_KEY);
		}
		learning_status |= WAIT_RELEASE_REMOTE;
		learning_status &= ~WAIT_LEARN_COMPLETE;
		learning_key_timer.expires = jiffies + POLL_RELEASE_DELAY;
		learning_key_timer.function = timer_handle;
		add_timer(&learning_key_timer);
	}
	else if (learning_status & WAIT_RELEASE_REMOTE)
	{
		if (learning_status & KEY_WAVE_PRESENT)
		{
			learning_status &= ~KEY_WAVE_PRESENT;
			learning_key_timer.expires = jiffies + POLL_RELEASE_DELAY;
			learning_key_timer.function = timer_handle;
			add_timer(&learning_key_timer);
		}
		else
		{
			outw( inw( IO_GIO_IRQPORT) & (~GIO3), IO_GIO_IRQPORT);	// gio 3 external IRQ disable
			lock_data_protect();
			osd_key=get_osd_key();
			unlock_data_protect();
			if (!osd_key )
			{
				report_key(RELEASE_REMOTE_KEY);
				//report_key(UP_KEY);
			}
		}
	}
#else
	int *a = (int*)data;
	int atimes;
	int osd_key;
	dbg("bitstimes=%d\n", BlsKey->bitstimes);
	if ( *a & 1)
	{
		(*a) &= ~1;
		learning_key_timer.expires = jiffies + LEANRING_COMPLETE_DELAY;
		learning_key_timer.data = (unsigned long)a;
		learning_key_timer.function = timer_handle;
		add_timer(&learning_key_timer);
	}
	else if ((BlsKey->bitstimes & MAX_REPEAT_TIMES) || real_key)
	{
		if (real_key)
		{
			atimes = BlsKey->bitstimes & MAX_REPEAT_TIMES;
			if ((++atimes) > MAX_REPEAT_TIMES)
				atimes=MAX_REPEAT_TIMES;
			BlsKey->bitstimes &= (~MAX_REPEAT_TIMES);
			BlsKey->bitstimes |= atimes;
			real_key=0;
		}
		(*a) &= ~2;
		mdelay(DELAY_FOR_IR);
		outw( inw( IO_GIO_IRQPORT) & (~GIO3), IO_GIO_IRQPORT);	// gio 3 external IRQ disable
		lock_data_protect();
		osd_key=get_osd_key();
		unlock_data_protect();
		if (!osd_key )
		{
			report_key(LEARNING_COMPLETE_KEY);
			//report_key(UP_KEY);
		}
	}
	else
	{
		(*a) &= ~2;
	}
#endif
}

static int capture_key(struct blaster_data_type* blsdat)
{
#if LEARNING_THROUGH_MSP430
    int i;
    blsdat->bitstimes = i2c_read(regBITSTIMES);
    blsdat->interval = i2c_read(regINTERVAL);
    blsdat->start1 = i2c_read(regSTART1);
    blsdat->start2 = i2c_read(regSTART2);
    for (i = 0; i < BLASTER_MAX_BITS; i++)
        blsdat->bits[i] = i2c_read(regBITS + i); 
#else
#ifdef BLASTER_THROUGH_ARM
    static int times=0;
	static int old_counter;
	static int old_int_counter;
	int counter;
	int td;

	if (!(learning_status & WAIT_RELEASE_REMOTE))
	{
		counter=inw(IO_TIMER1_TMCNT);
		if (old_int_counter==timer_int_counter)
			td=counter - old_counter;
		else
			td=((timer_int_counter - old_int_counter)<<MAX_COUNTER_BIT) + counter - old_counter \
			- INT_CAPTURE_TIME_WASTE;
		old_counter=counter;
		old_int_counter=timer_int_counter;
		if (!(learning_status & WAIT_LEARN_COMPLETE))
		{
			times = 0;
			learning_key_timer.function = timer_handle;
			mod_timer(&learning_key_timer, jiffies + LEANRING_COMPLETE_DELAY);
			learning_status |= WAIT_LEARN_COMPLETE;
		}

		if (0 == times++)
		{
			blsdat->bitstimes |= (((inw(IO_GIO_BITSET0) | GIO3) << 12)&FIRST_LEVEL_BIT_MASK);
			return 0;
		}
		blsdat->bits[times-2]=td;
		blsdat->bitstimes++;
		if (times == BLASTER_MAX_CHANGE+1)
		{
			report_key(LEARNING_COMPLETE_KEY);
			learning_key_timer.function = timer_handle;
			mod_timer(&learning_key_timer, jiffies + POLL_RELEASE_DELAY);
			learning_status |= WAIT_RELEASE_REMOTE;
			learning_status &= ~WAIT_LEARN_COMPLETE;
		}
	}
	else
		learning_status	|= KEY_WAVE_PRESENT;
#else
    /*static int start1=0;
    static int start2=0;*/
    static int oldusec=0;
    static int oldsec=0;
    static int times=0;
    static int oatimes;
    static int atimes;
    static struct timeval tm;
    static struct timeval stm;
    static int td;
    static int timer_added = 0;

    timer_added |= 1;
    if(!(timer_added&2))
    {
        learning_key_timer.data = (unsigned long) &timer_added;
        learning_key_timer.function = timer_handle;
        mod_timer(&learning_key_timer, jiffies + LEANRING_COMPLETE_DELAY);
        timer_added |= 2;
    } 

    do_gettimeofday(&tm);
    if (tm.tv_sec != oldsec)
        td = (tm.tv_sec-oldsec) * 1000000 + tm.tv_usec - oldusec;
    else
        td = tm.tv_usec-oldusec;

    oldsec = tm.tv_sec;
    oldusec = tm.tv_usec;
    if (/*(tm.tv_sec-stm.tv_sec)*1000000+(tm.tv_usec-stm.tv_usec)*/td > 100000) //500ms
    {
        times = 0;
        atimes = 1;
        memset(blsdat, 0, sizeof(struct blaster_data_type));
    }
    if (0 == times++)
    {
        stm = tm;
        oldusec = tm.tv_usec;
        blsdat->bitstimes |= ((inw(IO_GIO_BITSET0) | GIO3) << 12);
        oatimes = BLASTER_MAX_BITS;
    }
    else
    {
        if (times > (BLASTER_MAX_CHANGE - 2))
            return 0; 
        if (td > 10000)//10ms
        {
            if (real_key)
            {
                real_key = 0;
                oatimes = times;
                if(td<0xffff)
                    blsdat->interval = td;
                else
                    blsdat->interval = 0xffff;
                if (atimes > MAX_REPEAT_TIMES)
                    atimes = MAX_REPEAT_TIMES;
                if(atimes==1)
				{
					blsdat->bitstimes &= 0x8000;
                    blsdat->bitstimes |= ((((times - 1) & (BITS_COUNT_MASK>>BITS_COUNT_START)) << BITS_COUNT_START) + (atimes & MAX_REPEAT_TIMES));
				}
                else
                {
                    blsdat->bitstimes &= (~MAX_REPEAT_TIMES);
                    blsdat->bitstimes |= (atimes & MAX_REPEAT_TIMES);
                }
                atimes++;
            }
            times = 1;
            stm = tm;
            return 0;
        }

        if (times == 2)
        {
            blsdat->bitstimes |= END_FLAG_MASK;
        }  
        else if (times == 5)
        {
            blsdat->bitstimes &= ~END_FLAG_MASK;
        }
        if (times < 10)
            blsdat->end[times-2] = td;

        else if (times == 10)
        {
            real_key = 1;
            blsdat->start1 = blsdat->end[0];
            blsdat->start2 = blsdat->end[1];
            blsdat->bits[0] = blsdat->end[2];
            blsdat->bits[1] = blsdat->end[3];
            blsdat->bits[2] = blsdat->end[4];
            blsdat->bits[3] = blsdat->end[5];
            blsdat->bits[4] = blsdat->end[6];
            blsdat->bits[5] = blsdat->end[7];
            blsdat->bits[6] = td;
            memset(blsdat->end, 0, 8);
        }
        else if (times > 10)
            blsdat->bits[times-4] = td;
    }
#endif  //BLASTER_THROUGH_ARM
#endif //LEARNING_THROUGH_MSP430

    return(0);
}

static irqreturn_t handle_bls_timer1_irqs(int irq, void * dev_id, struct pt_regs * regs)
{
	uint16_t bitset2;
	if(int_type==CAPTURE)
	{
		timer_int_counter++;
		return IRQ_HANDLED;
	}
	if(bls_wave_count==0)
		return IRQ_HANDLED;
	if (!wave_len)
	{
		bitset2=inw( IO_GIO_BITSET2 );
		if (bitset2 & GIO34)
			outw(GIO34, IO_GIO_BITCLR2);
		else
			outw( GIO34, IO_GIO_BITSET2 ); 
		bls_wave_count--;
	}
	if(bls_wave_count==0)
	{
		disable_irq(IRQ_TIMER1);
		outw( inw( IO_GIO_DIR2 ) | GIO34, IO_GIO_DIR2 );  //gio 34 direction input
		outw(0,IO_TIMER1_TMMD);
		bls_status=BLS_COMPLETE;
		if(bls_data_pack)
		{
			kfree(bls_data_pack);
			bls_data_pack=NULL;
		}
		return IRQ_HANDLED;
	}
    else
	{
		set_timer1_div(bls_data_pack);
	}
	return IRQ_HANDLED;
}

static irqreturn_t handle_capture_irqs(int irq, void * dev_id, struct pt_regs * regs)
{
#if  !LEARNING_THROUGH_MSP430
        capture_key(BlsKey);
#endif

    return IRQ_HANDLED;
}

//--------------------------------------------------- DEVICE --------------------------------------------------------------

static int irrtc_open(struct inode * inode, struct file * file)
{
#ifdef ONLYOPENONE
    if (opened)
        return -EBUSY;
    opened = 1;
#endif

    return 0;
}

static int irrtc_release(struct inode * inode, struct file * file)
{
#ifdef ONLYOPENONE
    if (!opened)
        return -ERESTARTSYS;

    opened = 0;
#endif

    return 0;
}

static int irrtc_ioctl(struct inode * inode, struct file * file,
					   unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	int arg_size;
	if (_IOC_TYPE(cmd) != NEUROS_IR_BLASTER_IOC_MAGIC)
	{
		ret = -EINVAL;
		goto bail;
	}

	arg_size = _IOC_SIZE(cmd);
	if (_IOC_DIR(cmd) & _IOC_READ)
		ret = !access_ok(VERIFY_WRITE, (void *)arg, arg_size);
	else if (_IOC_DIR(cmd) & _IOC_WRITE)
		ret = !access_ok(VERIFY_READ, (void *)arg, arg_size);
	if (ret) goto bail;
	switch (cmd)
	{
	case RRB_BLASTER_KEY:
		{
			if (bls_status!=BLS_START)
			{
				int_type=BLASTER;
				bls_data_pack=kmalloc(sizeof(struct blaster_data_pack), GFP_KERNEL);
				if (bls_data_pack==NULL)
				{
					ret = -EINVAL;
					break;
				}
				copy_from_user(bls_data_pack, (void*)arg, sizeof(struct blaster_data_pack));
				blaster_key(bls_data_pack);
			}
		}
		break;
	case RRB_CAPTURE_KEY:
		{
			//outw( inw( IO_GIO_IRQPORT) & ~0x0001, IO_GIO_IRQPORT);  // gio 0 external IRQ(ir) disable when learning
			set_osd_key(0);
			int_type=CAPTURE;
			timer_int_counter=0;
			outw(BLS_TIMER_PRESCALE, IO_TIMER1_TMPRSCL);
			outw(MAX_COUNTER, IO_TIMER1_TMDIV);
			wmb();
			outw(2,IO_TIMER1_TMMD);
			enable_irq(IRQ_TIMER1);
			BlsKey=kmalloc(sizeof(struct blaster_data_type), GFP_KERNEL);
			if(BlsKey==NULL)
			{
				ret = -EINVAL;
				break;
			}
			memset(BlsKey, 0, sizeof(struct blaster_data_type));
			learning_status=0;
			outw( inw( IO_GIO_IRQPORT) | GIO3, IO_GIO_IRQPORT);	 // gio 3 external IRQ enable
		}
		break;
	case RRB_READ_LEARNING_DATA:
		{
			if(BlsKey)
			{
				ret |= copy_to_user((void*)arg, BlsKey, sizeof(struct blaster_data_type));
				kfree(BlsKey);
				BlsKey=NULL;
			}
		}
		break;
	case RRB_FACTORY_TEST:
		{
			set_factory_test(1);
		}
		break;
	case RRB_GET_BLASTER_STATUS:
		{
			ret |= copy_to_user((void*)arg, &bls_status, sizeof(bls_status));
		}
		break;

	default:
		ret = -EINVAL;
		break;
	}

	bail:
	return ret;
}

static struct file_operations irrtc_fops = {
    .open    = irrtc_open,
    .release = irrtc_release,
    .ioctl   = irrtc_ioctl,
};

//-------------------------------------------------- INIT / EXIT ----------------------------------------------------------

static const char * pname = "NEUROS_IRBLAST(KM):";

static int blaster_init( void )
{
    outw( inw( IO_GIO_DIR2 ) & (~(0x0002)), IO_GIO_DIR2 );  //gio 33 direction output
    outw( inw( IO_GIO_FSEL3) | 0x0003, IO_GIO_FSEL3 );  //gio 33 fsel  clkout1b

    outw( (inw( IO_CLK_OSEL) & 0xFF0F)| 0x0000, IO_CLK_OSEL );  //clk arm clock
    //outw( inw( IO_CLK_OSEL) & (~(0x0000)), IO_CLK_OSEL );  //clk arm clock

    outw( 0x0530, IO_CLK_O1DIV);  //clk div 2*(x+1)  38K:0x0A47 40K:0x09C3 
#ifdef  BLASTER_THROUGH_ARM
    outw( inw( IO_GIO_DIR2 ) | GIO34, IO_GIO_DIR2 );  //gio 34 direction input
    outw( inw( IO_GIO_FSEL3) & (~(0x000C)), IO_GIO_FSEL3 );  //gio 34 fsel  clr 2:3 normal gio
    request_irq(IRQ_TIMER1, handle_bls_timer1_irqs,SA_INTERRUPT|SA_TIMER , "ir_blaster_timer1", &device); //TIMER__INTERRUPT
    disable_irq(IRQ_TIMER1);
#endif

    outw( inw( IO_GIO_DIR0 ) | GIO3, IO_GIO_DIR0 );  //gio 3 direction input
    //outw( inw( IO_GIO_INV0 ) | GIO3, IO_GIO_INV0 );  //gio 3 inv
    outw( inw( IO_GIO_IRQPORT) & (~GIO3), IO_GIO_IRQPORT); //gio 3 external IRQ disable
    outw( inw( IO_GIO_IRQEDGE) | GIO3, IO_GIO_IRQEDGE);  // gio 3 any edge

    request_irq(IRQ_GIO3, handle_capture_irqs,SA_INTERRUPT , "ir_capture", &device); //SA_SHIRQSA_INTERRUPT
    return 0;
}

//~ static void irqs_irrtc_init( void )
//~ {
	//~ outw( inw( IO_GIO_DIR0 ) | 0x0001, IO_GIO_DIR0 );  //gio 0 direction input
	//~ outw( inw( IO_GIO_INV0 ) | 0x0001, IO_GIO_INV0 );  //gio 0 inv
	//~ outw( inw( IO_GIO_IRQPORT) | 0x0001, IO_GIO_IRQPORT);  // gio 0 external IRQ enable
	//~ request_irq(IRQ_GIO0, handle_irrtc_irqs, 0, "irrtc", &device); 
//~ }

//~ static void irqs_irrtc_exit( void )
//~ {
    //~ free_irq(IRQ_GIO0, &device);
//~ }

static int __init irrtc_init(void)
{
    int status = 0;
    init_timer(&learning_key_timer);
    init_waitqueue_head (&wait);

    printk(KERN_INFO "\t" MOD_DESC "\n");

    status = register_chrdev(NEUROS_IR_BLASTER_MAJOR, "ir_blaster", &irrtc_fops);
    if (status != 0)
    {
        if (status == -EINVAL) printk(KERN_ERR "%s Couldn't register device: invalid major number %d.\n", pname, NEUROS_IR_BLASTER_MAJOR);
        else if (status == -EBUSY) printk(KERN_ERR "%s Couldn't register device: major number %d already busy.\n", pname, NEUROS_IR_BLASTER_MAJOR);
        else printk(KERN_ERR "%s Couldn't register device: error %d.\n", pname, status);
        status = -1;
        goto out;
    }

    //~ irqs_irrtc_init();
    blaster_init();

out:
    return status;
}

static void __exit irrtc_exit(void)
{
    //~ irqs_irrtc_exit();

    unregister_chrdev(NEUROS_IR_BLASTER_MAJOR, "ir_blaster");
}

MODULE_AUTHOR("Neuros");
MODULE_DESCRIPTION(MOD_DESC);
MODULE_LICENSE("Neuros Technology LLC");

module_init(irrtc_init);
module_exit(irrtc_exit);


