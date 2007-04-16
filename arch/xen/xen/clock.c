/*	$NetBSD: clock.c,v 1.14 2005/08/11 20:32:56 cube Exp $	*/

/*
 *
 * Copyright (c) 2004 Christian Limpach.
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
 *      This product includes software developed by Christian Limpach.
 * 4. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/time.h>
#include <sys/kernel.h>
#include <sys/device.h>
#include <sys/sysctl.h>

#include <machine/xen.h>
#include <machine/hypervisor.h>
#include <machine/evtchn.h>
#include <machine/cpufunc.h>
#include <machine/cpu_counter.h>

#include <dev/clock_subr.h>

int xen_timer_handler(void *, struct intrframe *);

/* These are periodically updated in shared_info, and then copied here. */
static volatile uint64_t shadow_tsc_stamp; /* TSC at last update of time vals. */
static volatile uint64_t shadow_system_time; /* Time, in nanosecs, since boot. */
static volatile struct timeval shadow_tv; /* Time since 00:00:00 UTC, Jan 1, 1970 */
static volatile unsigned long shadow_time_version; /* XXXSMP */
static volatile uint32_t shadow_freq_mul;
static volatile int8_t shadow_freq_shift;

static int timeset;

static uint64_t processed_system_time;
static struct timeval cc_time;
static int64_t cc_ms_delta;
static int64_t cc_denom;

#ifdef DOM0OPS
/* If we're dom0, send our time to Xen every minute or so. */
int xen_timepush_ticks = 0;
static struct timeout xen_timepush_co;
#endif

/*
 * XXX
 * Don't know yet what to do with those, how to get rid of them.
 */
uint64_t cpuspeed;
u_quad_t pentium_base_tsc = 0;

#define NS_PER_TICK (1000000000ULL/hz)

/*
 * Reads a consistent set of time-base values from Xen, into a shadow data
 * area.  Must be called at splclock.
 */
static void
get_time_values_from_xen(void)
{
	volatile struct vcpu_time_info *t =
	    &HYPERVISOR_shared_info->vcpu_info[0].time;
	uint32_t tversion;

	do {
		shadow_time_version = t->version;
		x86_lfence();
		shadow_tsc_stamp = t->tsc_timestamp;
		shadow_system_time = t->system_time;
		shadow_freq_mul = t->tsc_to_system_mul;
		shadow_freq_shift = t->tsc_shift;
		x86_lfence();
	} while ((t->version & 1) || (shadow_time_version != t->version));
	do {
		tversion = HYPERVISOR_shared_info->wc_version;
		x86_lfence();
		shadow_tv.tv_sec = HYPERVISOR_shared_info->wc_sec;
		shadow_tv.tv_usec = HYPERVISOR_shared_info->wc_nsec;
		x86_lfence();
	} while ((HYPERVISOR_shared_info->wc_version & 1) ||
	    (tversion != HYPERVISOR_shared_info->wc_version));
	shadow_tv.tv_usec = shadow_tv.tv_usec / 1000;
}

/*
 * Are the values we have up to date?
 */
static inline int
time_values_up_to_date(void)
{
	int rv;

	x86_lfence();
	rv = shadow_time_version ==
		HYPERVISOR_shared_info->vcpu_info[0].time.version; /* XXXSMP */

	x86_lfence();

	return rv;
}

/*
 * Xen 3 helpfully provides the CPU clock speed in the form of a multiplier
 * and shift that can be used to convert a cycle count into nanoseconds
 * without using an actual (slow) divide insn.
 */
static inline uint64_t
scale_delta(uint64_t delta, uint32_t mul_frac, int8_t shift)
{
	if (shift < 0)
		delta >>= -shift;
	else
		delta <<= shift;

	/*
	 * Here, we multiply a 64-bit and a 32-bit value, and take the top
	 * 64 bits of that 96-bit product.  This is broken up into two
	 * 32*32=>64-bit multiplies and a 64-bit add.  The casts are needed
	 * to hint to GCC that both multiplicands really are 32-bit; the
	 * generated code is still fairly bad, but not insanely so.
	 */
	return ((uint64_t)(uint32_t)(delta >> 32) * mul_frac)
	    + ((((uint64_t)(uint32_t)(delta & 0xFFFFFFFF)) * mul_frac) >> 32);
}


static uint64_t
get_tsc_offset_ns(void)
{
	uint64_t tsc_delta, offset;

	tsc_delta = cpu_counter() - shadow_tsc_stamp;
	offset = scale_delta(tsc_delta, shadow_freq_mul,
	    shadow_freq_shift);
#ifdef XEN_CLOCK_DEBUG
	if (offset > 10 * NS_PER_TICK)
		printf("get_tsc_offset_ns: tsc_delta=%llu offset=%llu\n",
			tsc_delta, offset);
#endif
	return offset;
}

static uint64_t
get_system_time(void)
{
	uint64_t stime;

	for (;;) {
		stime = shadow_system_time + get_tsc_offset_ns();

		/* if the timestamp went stale before we used it, refresh */
		if (time_values_up_to_date())
			break;
		get_time_values_from_xen();
	}
	return stime;
}

void
inittodr(time_t base)
{
	int s;
	uint64_t t;

	if (base && base < 15 * SECYR) {
		printf("WARNING: preposterous time in file system\n");
		base = 17*SECYR + 186*SECDAY + SECDAY/2;
	}

	s = splclock();
	get_time_values_from_xen();
	splx(s);

	t = shadow_tv.tv_sec * 1000000ULL +
	    shadow_tv.tv_usec + processed_system_time / 1000;
	time.tv_usec = t % 1000000ULL;
	time.tv_sec = t / 1000000ULL;
#ifdef XEN_CLOCK_DEBUG
	printf("readclock: %ld (%ld)\n", time.tv_sec, base);
#endif

	if (base != 0 && base < time.tv_sec - 5*SECYR)
		printf("WARNING: file system time much less than clock time\n");
	else if (base > time.tv_sec + 5*SECYR) {
		printf("WARNING: clock time much less than file system time\n");
		printf("WARNING: using file system time\n");
		goto fstime;
	}

	timeset = 1;
	return;

fstime:
	timeset = 1;
	time.tv_sec = base;
	printf("WARNING: CHECK AND RESET THE DATE!\n");
}

static void
resettodr_i(void)
{
#ifdef DOM0OPS
	dom0_op_t op;
	int s;
#endif
#ifdef DEBUG_CLOCK
	struct timeval sent_delta;
#endif

	/*
	 * We might have been called by boot() due to a crash early
	 * on.  Don't reset the clock chip in this case.
	 */
	if (!timeset)
		return;

#ifdef DEBUG_CLOCK
	do {
		char pm;

		if (timercmp(&time, &shadow_tv, >)) {
			timersub(&time, &shadow_tv, &sent_delta);
			pm = '+';
		} else {
			timersub(&shadow_tv, &time, &sent_delta);
			pm = '-';
		}
		printf("resettodr: stepping Xen clock by %c%ld.%06ld\n",
			pm, sent_delta.tv_sec, sent_delta.tv_usec);
	} while (0);
#endif
#ifdef DOM0OPS
	if (xen_start_info.flags & SIF_PRIVILEGED) {
		s = splclock();

		op.cmd = DOM0_SETTIME;
		op.u.settime.secs	 = time.tv_sec;
		op.u.settime.nsecs	 = time.tv_usec * 1000;
		op.u.settime.system_time = processed_system_time;
		HYPERVISOR_dom0_op(&op);

		splx(s);
	}
#endif
}

/*
 * When the clock is administratively set, in addition to resetting
 * Xen's clock if possible, we should also allow xen_microtime to
 * to set backwards without complaint.
 */
void
resettodr(void)
{
	resettodr_i();

	timerclear(&cc_time);
	/* XXX: On SMP iterate over all cpu's and clear the time
	 */
}

void
startrtclock(void)
{

}

/*
 * Wait approximately `n' microseconds.
 */
void
xen_delay(int n)
{
	if (n < 500000) {
		/*
		 * shadow_system_time is updated at best every hz tick,
		 * it's not precise enouth for short delays. Use the CPU
		 * counter instead. We assume it's working at this point.
		 */
		u_int64_t cc, cc2, when;

		cc = cpu_counter();
		when = cc + (u_int64_t)n * cpuspeed / 1LL;
		if (when < cc) {
			/* wait for counter to wrap */
			cc2 = cpu_counter();
			while (cc2 > cc)
				cc2 = cpu_counter();
		}
		cc2 = cpu_counter();
		while (cc2 < when)
			cc2 = cpu_counter();

		return;
	} else {
		uint64_t now, when;

		/* for large delays, we can use shadow_system_time and
		 * setup a timer so that the CPU can eventually be used by
		 * other domains. Note that we can't use set_timer/block
		 * because interrupts may be disabled.
		 */
		get_time_values_from_xen();
		now = shadow_system_time + get_tsc_offset_ns();
		when = now + n * 1000;
		while (now < when) {
			HYPERVISOR_yield();
			__insn_barrier();
			get_time_values_from_xen();
			now = shadow_system_time + get_tsc_offset_ns();
			__insn_barrier();
		}
	}
}

void
xen_microtime(struct timeval *tv)
{
	int64_t cycles;
	int s;

	s = splclock();
	*tv = time;
	cycles = cpu_counter() - shadow_tsc_stamp;
	KDASSERT(cycles > 0);
	cycles += cc_denom * cpuspeed / 1000LL;
#ifdef XEN_CLOCK_DEBUG
	if (cycles <= 0) {
		printf("xen_microtime: CPU counter has decreased by %lli"
			" since last hardclock(9)\n", -cycles);
	}
#endif
	tv->tv_usec += cycles * cc_ms_delta * hz / (cpuspeed * 1000000);
	KDASSERT(tv->tv_usec < 2000000);
	while (tv->tv_usec >= 1000000) {
		tv->tv_usec -= 1000000;
		tv->tv_sec++;
	}
	/* Avoid small backsteps, e.g. at the beginning of a negative adjustment. */
	if (timerisset(&cc_time) &&
	    timercmp(tv, &cc_time, <)) {
		struct timeval backstep;

		timersub(&cc_time, tv, &backstep);
		if (backstep.tv_sec == 0) {	/* if it was < 1sec */
			*tv = cc_time;
#ifdef XEN_CLOCK_DEBUG
			printf("xen_microtime: clamping at %ld.%06ld (-%ldus)\n",
				tv->tv_sec, tv->tv_usec, backstep.tv_usec);
		} else {
			printf("xen_microtime: allowing large backstep "
				"%lds to %ld.%06ld\n",
				backstep.tv_sec, tv->tv_sec, tv->tv_usec);
#endif
		}
	}

	cc_time = *tv;
	splx(s);
}


#ifdef DOM0OPS
/* ARGSUSED */
static void
xen_timepush(void *arg)
{
	struct timeout *co = arg;

	resettodr_i();
	if (xen_timepush_ticks > 0) {
		timeout_add(co, xen_timepush_ticks);
	}
}

#ifdef __NetBSD__
/* ARGSUSED */
static int
sysctl_xen_timepush(const int *name, u_int namelen, void *oldp,
		size_t *oldlenp, const void *newp, size_t newlen)
{
	int error = 0, new_ticks;
#ifdef __NetBSD__
	struct sysctl_node node;
#endif

	new_ticks = xen_timepush_ticks;
#ifdef __NetBSD__
	node = *rnode;
	node.sysctl_data = &new_ticks;
	error = sysctl_lookup(SYSCTLFN_CALL(&node));
#endif
	if (error || newp == NULL)
		return error;

	if (new_ticks < 0)
		return EINVAL;
	if (new_ticks != xen_timepush_ticks) {
		xen_timepush_ticks = new_ticks;
		if (new_ticks > 0)
			timeout_add(&xen_timepush_co, new_ticks);
		else
			timeout_del(&xen_timepush_co);
	}

	return 0;
}
#endif /* __NetBSD__ */
#endif /* DOM0OPS */

void
calibrate_cyclecounter(uint64_t freq)
{
	cpuspeed = (freq + 500000) / 1000000;
}

void
xen_initclocks(void)
{
	int evtch;

	
	evtch = bind_virq_to_evtch(VIRQ_TIMER);
	printf("Xen clock: using event channel %d\n", evtch);

	get_time_values_from_xen();
	processed_system_time = shadow_system_time;
	cc_time = time;
	cc_ms_delta = 1;
	cc_denom = cpuspeed * 1;

	event_set_handler(evtch, (int (*)(void *))xen_timer_handler,
	    NULL, IPL_CLOCK, "clock");
	hypervisor_enable_event(evtch);

#ifdef DOM0OPS
	xen_timepush_ticks = 53 * hz + 3; /* avoid exact # of min/sec */
	if (xen_start_info.flags & SIF_PRIVILEGED) {
#ifdef __NetBSD__
		sysctl_createv(NULL, 0, NULL, NULL, CTLFLAG_READWRITE,
			CTLTYPE_INT, "xen_timepush_ticks", SYSCTL_DESCR("How often"
			" to update the hypervisor's time-of-day; 0 to disable"),
			sysctl_xen_timepush, 0, &xen_timepush_ticks, 0,
			CTL_MACHDEP, CTL_CREATE, CTL_EOL);
#endif
		timeout_set(&xen_timepush_co, xen_timepush, &xen_timepush_co);
	}
#endif
}

int
xen_timer_handler(void *arg, struct intrframe *regs)
{
	int64_t delta;
	int ticks_done;
	struct timeval oldtime, elapsed;

	get_time_values_from_xen();

	ticks_done = 0;
	delta = (int64_t)(get_system_time() - processed_system_time);
	while (delta >= (int64_t)NS_PER_TICK) {
		/* Have hardclock do its thing. */
		oldtime = time;
		hardclock((struct clockframe *)regs);

		/* Use that tick length for the coming tick's microtimes. */
		timersub(&time, &oldtime, &elapsed);
		KDASSERT(elapsed.tv_sec == 0);
		cc_ms_delta = elapsed.tv_usec;

		delta -= NS_PER_TICK;
		processed_system_time += NS_PER_TICK;
		ticks_done++;
	}
	/* Re-arm the timer here, if needed; Xen's auto-ticking while runable
	 * is useful only for HZ==100, and even then may be out of phase with
	 * the processed_system_time steps.
	 */
	if (ticks_done != 0)
		HYPERVISOR_set_timer_op(processed_system_time + NS_PER_TICK);

	/* Right now, delta holds the number of ns elapsed from when the last
	 * hardclock(9) allegedly was to when this domain was actually
	 * rescheduled.
	 */
	cc_denom = delta;
	return 0;
}

void
setstatclockrate(int arg)
{
}

void
idle_block(void)
{
	int s, r;
	/*
	 * We set the timer to when we expect the next timer
	 * interrupt.  We could set the timer to later if we could
	 * easily find out when we will have more work (callouts) to
	 * process from hardclock.
	 */
	s = splclock();
	r = HYPERVISOR_set_timer_op(processed_system_time + NS_PER_TICK);
	splx(s);
	if (r == 0)
		HYPERVISOR_block();
}
