/*	$OpenBSD: clock.c,v 1.18 2007/03/20 20:59:54 kettenis Exp $	*/
/*	$NetBSD: clock.c,v 1.1 1996/09/30 16:34:40 ws Exp $	*/

/*
 * Copyright (C) 1995, 1996 Wolfgang Solfrank.
 * Copyright (C) 1995, 1996 TooLs GmbH.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. All advertising materials mentioning features or use of this software
 *    must display the following acknowledgement:
 *	This product includes software developed by TooLs GmbH.
 * 4. The name of TooLs GmbH may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY TOOLS GMBH ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL TOOLS GMBH BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <sys/param.h>
#include <sys/kernel.h>
#include <sys/systm.h>
#include <sys/evcount.h>
#include <sys/timetc.h>

#include <machine/autoconf.h>
#include <machine/pio.h>
#include <machine/intr.h>
#include <machine/vmparam.h>
#include <machine/powerpc.h>
#include <dev/ofw/openfirm.h>

void decr_intr(struct clockframe *frame);
u_int tb_get_timecount(struct timecounter *);

/*
 * Initially we assume a processor with a bus frequency of 12.5 MHz.
 */
static u_int32_t ticks_per_sec = 3125000;
static u_int32_t ns_per_tick = 320;
static int32_t ticks_per_intr;
static volatile u_int64_t lasttb;

static struct timecounter tb_timecounter = {
	tb_get_timecount, NULL, 0x7fffffff, 0, "tb", 0, NULL
};

time_read_t  *time_read;
time_write_t *time_write;

/* event tracking variables, when the next events of each time should occur */
u_int64_t nexttimerevent, prevtb, nextstatevent;

/* vars for stats */
int statint;
u_int32_t statvar;
u_int32_t statmin;

static struct evcount clk_count;
static struct evcount stat_count;
static int clk_irq = PPC_CLK_IRQ;
static int stat_irq = PPC_STAT_IRQ;

/*
 * Set up the system's time, given a `reasonable' time value.
 */
void
inittodr(time_t base)
{
	int badbase = 0, waszero = base == 0;
	char *bad = NULL;
	struct timeval tv;
	struct timespec ts;

        if (base < 5 * SECYR) {
                /*
                 * If base is 0, assume filesystem time is just unknown
                 * instead of preposterous. Don't bark.
                 */
                if (base != 0)
                        printf("WARNING: preposterous time in file system\n");
                /* not going to use it anyway, if the chip is readable */
                base = 21*SECYR + 186*SECDAY + SECDAY/2;
                badbase = 1;
        }

	if (time_read != NULL) {
		u_int32_t cursec;
		(*time_read)(&cursec);
		tv.tv_sec = cursec;
		tv.tv_usec = 0;
	} else {
		/* force failure */
		tv.tv_sec = tv.tv_usec = 0;
	}

	if (tv.tv_sec == 0) {
		/*
		 * Believe the time in the file system for lack of
		 * anything better, resetting the clock.
		 */
		bad = "WARNING: unable to get date/time";
		tv.tv_sec = base;
		tv.tv_usec = 0;
		if (!badbase)
			resettodr();
	} else {
		int deltat;

		tv.tv_sec += tz.tz_minuteswest * 60;
		if (tz.tz_dsttime)
			tv.tv_sec -= 3600;

		deltat = tv.tv_sec - base;

		if (deltat < 0)
			deltat = -deltat;
		if (!(waszero || deltat < 2 * SECDAY)) {
			printf("WARNING: clock %s %d days",
			    tv.tv_sec < base ? "lost" : "gained", deltat / SECDAY);
			bad = "";

			if (tv.tv_sec < base && deltat > 1000 * SECDAY) {
				printf(", using FS time");
				tv.tv_sec = base;
			}
		}
	}

	ts.tv_sec = tv.tv_sec;
	ts.tv_nsec = tv.tv_usec * 1000;
	tc_setclock(&ts);

	if (bad) {
		printf("%s", bad);
		printf(" -- CHECK AND RESET THE DATE!\n");
	}
}

/*
 * Similar to the above
 */
void
resettodr(void)
{
	struct timeval tv;

	if (time_second == 0)
		return;

	microtime(&tv);

	if (time_write != NULL) {
		tv.tv_sec -= tz.tz_minuteswest * 60;
		if (tz.tz_dsttime) {
			tv.tv_sec += 3600;
		}
		(*time_write)(tv.tv_sec);
	}
}

volatile int statspending;

void
decr_intr(struct clockframe *frame)
{
	u_int64_t tb;
	u_int64_t nextevent;
	int nstats;
	int s;

	/*
	 * Check whether we are initialized.
	 */
	if (!ticks_per_intr)
		return;


	/*
	 * Based on the actual time delay since the last decrementer reload,
	 * we arrange for earlier interrupt next time.
	 */

	tb = ppc_mftb();
	while (nexttimerevent <= tb)
		nexttimerevent += ticks_per_intr;

	prevtb = nexttimerevent - ticks_per_intr;

	for (nstats = 0; nextstatevent <= tb; nstats++) {
		int r;
		do {
			r = random() & (statvar -1);
		} while (r == 0); /* random == 0 not allowed */
		nextstatevent += statmin + r;
	}

	/* only count timer ticks for CLK_IRQ */
	stat_count.ec_count += nstats;

	if (nexttimerevent < nextstatevent)
		nextevent = nexttimerevent;
	else
		nextevent = nextstatevent;

	/*
	 * Need to work about the near constant skew this introduces???
	 * reloading tb here could cause a missed tick.
	 */
	ppc_mtdec(nextevent - tb);

	if (curcpu()->ci_cpl & SPL_CLOCK) {
		statspending += nstats;
	} else {
		nstats += statspending;
		statspending = 0;

		s = splclock();

		/*
		 * Reenable interrupts
		 */
		ppc_intr_enable(1);

		/*
		 * Do standard timer interrupt stuff.
		 * Do softclock stuff only on the last iteration.
		 */
		frame->pri = s | SINT_CLOCK;
		while (lasttb < prevtb - ticks_per_intr) {
			/* sync lasttb with hardclock */
			lasttb += ticks_per_intr;
			clk_count.ec_count++;
			hardclock(frame);
		}

		frame->pri = s;
		while (lasttb < prevtb) {
			/* sync lasttb with hardclock */
			lasttb += ticks_per_intr;
			clk_count.ec_count++;
			hardclock(frame);
		}

		while (nstats-- > 0)
			statclock(frame);

		splx(s);
		(void) ppc_intr_disable();

		/* if a tick has occurred while dealing with these,
		 * dont service it now, delay until the next tick.
		 */
	}
}

void
cpu_initclocks()
{
	int intrstate;
	int r;
	int minint;
	u_int64_t nextevent;

	intrstate = ppc_intr_disable();

	stathz = 100;
	profhz = 1000; /* must be a multiple of stathz */

	/* init secondary clock to stathz */
	statint = ticks_per_sec / stathz;
	statvar = 0x40000000; /* really big power of two */
	/* find largest 2^n which is nearly smaller than statint/2  */
	minint = statint / 2 + 100;
	while (statvar > minint)
		statvar >>= 1;

	statmin = statint - (statvar >> 1);


	lasttb = ppc_mftb();
	nexttimerevent = lasttb + ticks_per_intr;
	do {
		r = random() & (statvar -1);
	} while (r == 0); /* random == 0 not allowed */
	nextstatevent = lasttb + statmin + r;

	if (nexttimerevent < nextstatevent)
		nextevent = nexttimerevent;
	else
		nextevent = nextstatevent;

	evcount_attach(&clk_count, "clock", (void *)&clk_irq, &evcount_intr);
	evcount_attach(&stat_count, "stat", (void *)&stat_irq, &evcount_intr);

	tb_timecounter.tc_frequency = ticks_per_sec;
	tc_init(&tb_timecounter);

	ppc_mtdec(nextevent-lasttb);
	ppc_intr_enable(intrstate);
}

void
calc_delayconst(void)
{
	int qhandle, phandle;
	char name[32];
	int s;

	/*
	 * Get this info during autoconf?				XXX
	 */
	for (qhandle = OF_peer(0); qhandle; qhandle = phandle) {
		if (OF_getprop(qhandle, "device_type", name, sizeof name) >= 0
		    && !strcmp(name, "cpu")
		    && OF_getprop(qhandle, "timebase-frequency",
		    &ticks_per_sec, sizeof ticks_per_sec) >= 0) {
			/*
			 * Should check for correct CPU here?		XXX
			 */
			s = ppc_intr_disable();
			ns_per_tick = 1000000000 / ticks_per_sec;
			ticks_per_intr = ticks_per_sec / hz;
			ppc_intr_enable(s);
			break;
		}
		if ((phandle = OF_child(qhandle)))
			continue;
		while (qhandle) {
			if ((phandle = OF_peer(qhandle)))
				break;
			qhandle = OF_parent(qhandle);
		}
	}

	if (!phandle)
		panic("no cpu node");
}

/*
 * Wait for about n microseconds (us) (at least!).
 */
void
delay(unsigned n)
{
	u_int64_t tb;
	u_int32_t tbh, tbl, scratch;

	tb = ppc_mftb();
	tb += (n * 1000 + ns_per_tick - 1) / ns_per_tick;
	tbh = tb >> 32;
	tbl = (u_int32_t)tb;
	asm ("1: mftbu %0; cmplw %0,%1; blt 1b; bgt 2f;"
	     " mftb %0; cmplw %0,%2; blt 1b; 2:"
	     :: "r"(scratch), "r"(tbh), "r"(tbl));
}

/*
 * Nothing to do.
 */
void
setstatclockrate(int newhz)
{
	int minint;
	int intrstate;

	intrstate = ppc_intr_disable();

	statint = ticks_per_sec / newhz;
	statvar = 0x40000000; /* really big power of two */
	/* find largest 2^n which is nearly smaller than statint/2 */
	minint = statint / 2 + 100;
	while (statvar > minint)
		statvar >>= 1;

	statmin = statint - (statvar >> 1);
	ppc_intr_enable(intrstate);

	/*
	 * XXX this allows the next stat timer to occur then it switches
	 * to the new frequency. Rather than switching instantly.
	 */
}

u_int
tb_get_timecount(struct timecounter *tc)
{
	return ppc_mftbl();
}
