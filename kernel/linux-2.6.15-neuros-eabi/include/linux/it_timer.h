#ifndef __IT_TIMER_H
#define __IT_TIMER_H

/* Clock = 27000000MHz
 * 27000000t = 1s
 * 27000t = 1ms
 * 27t = 1us
 */

#include <linux/config.h>

#define TIMER_STOP           0x0000
#define TIMER_ONE_SHOT       0x0001
#define TIMER_FREE_RUN       0x0002
#define TIMER_CCD_SHUTTER    0x0100
#define TIMER_CCD_STROBE     0x0200
#define TIMER_POLARITY       0x0400
#define TIMER_TRIGGER_SELECT 0x0800
#define TIMER_TRIGGER_READY  0x1000

#define TIMER_MAX_MSPRESCALAR 1000

static inline unsigned short itimer_ms2divisor(unsigned long ms)
{
	unsigned int div =
		ms * ((CONFIG_SYS_CLK_FREQ / 1000) / TIMER_MAX_MSPRESCALAR);

	if (div > 0xFFFF) return -EOVERFLOW;

	return (unsigned short) div;
}//itimer_ms2divisor

static inline unsigned short itimer_us2prescalar(unsigned long us,
						 unsigned char res)
{
	unsigned short prescalar = (CONFIG_SYS_CLK_FREQ / 1000000) * res;

	if (prescalar > 0x0400) return -EOVERFLOW;

	return prescalar;
}//itimer_us2prescalar

static inline unsigned short itimer_us2divisor(unsigned long us,
					       unsigned char res)
{
	unsigned int div = (us / res);

	if (div > 0xFFFF) return -EOVERFLOW;

	return (unsigned short) div;
}//itimer_us2divisor

#ifdef __KERNEL__
typedef unsigned char itimer_t;

int request_itimer(itimer_t timer);
void unrequest_itimer(itimer_t timer);

int itimer_get_mode(itimer_t timer);

/** Set the Prescalar register for a hardware Timer.
 * The prescalar setting will automatically decrement
 * the prescale value by 1.
 *
 * @param timer The timer to set the prescalar for.
 * This timer cannot be timer 0 as this is used by
 * the Linux system.
 * @param value The new value to set the timer to.
 * This timer cannot be greater than 1024.
 */
void itimer_set_prescalar(itimer_t timer, u16 value);

/** Set the Divisor register for a hardware Timer.
 * The divisor setting will automatically decrement
 * the divisor value by 1.
 *
 * @param timer The timer to set the divisor for.
 * This timer cannot be timer 0 as this is used by
 * the Linux system.
 * @param value The new value to set the timer to.
 * This timer cannot be greater than 65535.
 */
void itimer_set_divisor(itimer_t timer, u16 value);

void itimer_set_mode(itimer_t timer, u16 value);

void itimer_trigger(itimer_t timer);
#endif /* __KERNEL__ */

#endif
