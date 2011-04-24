/*
 * linux/include/asm-armnommu/arch-dm270/latency.h
 * Timing support for preempt-stats patch
 *
 *   Copyright (C) 2003 Cadenux, LLC (http://www.cadenux.com)
 *   author:  Gregory Nutt <greg.nutt@cadenux.com>
 *
 * Derived from linux/include/asm-arm/arch-omap15/preem_latency.h
 * developed by MonteVista.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 * THIS  SOFTWARE  IS PROVIDED   ``AS  IS'' AND   ANY  EXPRESS OR IMPLIED
 * WARRANTIES,   INCLUDING, BUT NOT  LIMITED  TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN
 * NO  EVENT  SHALL   THE AUTHOR  BE    LIABLE FOR ANY   DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED   TO, PROCUREMENT OF  SUBSTITUTE GOODS  OR SERVICES; LOSS OF
 * USE, DATA,  OR PROFITS; OR  BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN  CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * You should have received a copy of the  GNU General Public License along
 * with this program; if not, write  to the Free Software Foundation, Inc.,
 * 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#ifndef __ASM_ARCH_LATENCY_H
#define __ASM_ARCH_LATENCY_H

#include <asm/hardware.h>
#include <asm/arch/timex.h> /* For CLOCK_DIVISOR */

extern unsigned long itdm320_gettimeoffset(void);

/* CLOCK_DIVISOR represents the number of clock ticks for to
 * achieve an interrupt frequency of HZ (see asm/param.h).
 * Here, we explicitly assume HZ=100 or that the period
 * is 10 msec.
 */

#define NSEC_PER_TICK           ((10*1000000) / CLOCK_DIVISOR)
#define TICKS_PER_USEC          (CLOCK_DIVISOR / (10*1000))

#define CLOCK_DIVISOR_K         (CLOCK_DIVISOR / 1000)

/* Clock access macros */

#define readclock_init()
#define readclock(x)	        (x=itdm320_gettimeoffset())

/* This inline function converts the raw timer value into
 * a microsecond timer.
 */

static inline unsigned long clock_to_usecs(unsigned long clk)
{
#if defined (CONFIG_ARCH_NTOSD_DM320)
   return(clk);
#else 

  /* Dividing the clk in an MS and LS part eliminates
   * the possibility of overflow.
   */

  register unsigned long ls;
  register unsigned long ms;

#if (1000*CLOCK_DIVISOR_K == CLOCK_DIVISOR)
  /* The following optimized calcuation works and is accurate
   * because I know that CLOCK_DIVISOR is an even multiple of
   * 1000.
   *
   * The following converts the LS part of the clock to
   * units of 5*usec with only fractional usec losses.
   * Overflow cannot happen.
   */

  ls = clk & 0x0003ffff; /* LSB = tick, range: 0-262143 */
  ls = (20 * ls) / CLOCK_DIVISOR_K;

  /* The following converts the LS part of the clock to
   * units of 2*usec with only fractional usec losses.
   * Overflow is okay here.
   */

  ms = clk >> 18;  /* LSB=262144*tick, range: 0-16383 */
  ms = ms * (20*262144 / CLOCK_DIVISOR_K);
#else
  /* The general case...
   *
   * The following converts the LS part of the clock to
   * units of 2*usec with only fractional usec losses.
   * Overflow cannot happen.
   */

  ls = clk & 0x0003ffff; /* LSB = tick, range: 0-262143 */
  ls = ((20*1000) * ls) / CLOCK_DIVISOR;

  /* The following converts the LS part of the clock to
   * units of 2*usec with only fractional usec losses.
   * Overflow is okay here.
   */

  ms = clk >> 18;  /* LSB=262144*tick, range: 0-16383 */
  ms = ms * (20*1000*262144 / CLOCK_DIVISOR);
#endif

  /* Return the result in units of 1 usec with rounding. */

  return (ls + ms + 1) >> 1;
#endif // CONFIG_ARCH_ITDM320_20
}

#define INTERRUPTS_ENABLED(x)   (!(x & PSR_I_BIT))

#endif /* __ASM_ARCH_LATENCY_H */
