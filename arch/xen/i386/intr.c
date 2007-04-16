/*	$NetBSD: intr.c,v 1.6 2005/04/16 22:49:38 bouyer Exp $	*/
/*	NetBSD: intr.c,v 1.20 2004/10/23 21:27:35 yamt Exp	*/

/*
 * Copyright 2002 (c) Wasabi Systems, Inc.
 * All rights reserved.
 *
 * Written by Frank van der Linden for Wasabi Systems, Inc.
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
 *      This product includes software developed for the NetBSD Project by
 *      Wasabi Systems, Inc.
 * 4. The name of Wasabi Systems, Inc. may not be used to endorse
 *    or promote products derived from this software without specific prior
 *    written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY WASABI SYSTEMS, INC. ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL WASABI SYSTEMS, INC
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*-
 * Copyright (c) 1991 The Regents of the University of California.
 * All rights reserved.
 *
 * This code is derived from software contributed to Berkeley by
 * William Jolitz.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the University nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 *     @(#)isa.c       7.2 (Berkeley) 5/13/91
 */
/*-
 * Copyright (c) 1993, 1994 Charles Hannum.
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
 *     This product includes software developed by the University of
 *     California, Berkeley and its contributors.
 * 4. Neither the name of the University nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 *     @(#)isa.c       7.2 (Berkeley) 5/13/91
 */

#include <sys/cdefs.h>

#include <sys/cdefs.h>
#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/syslog.h>
#include <sys/device.h>
#include <sys/malloc.h>
#include <sys/proc.h>
#include <sys/errno.h>

#include <machine/atomic.h>
#include <machine/i8259.h>
#include <machine/cpu.h>
#include <machine/pio.h>

/*
 * Recalculate the interrupt from scratch for an event source.
 */
void
intr_calculatemasks(struct evtsource *evts)
{
	struct intrhand *ih;

	evts->ev_maxlevel = IPL_NONE;
	evts->ev_imask = 0;
	for (ih = evts->ev_handlers; ih != NULL; ih = ih->ih_evt_next) {
		if (ih->ih_level > evts->ev_maxlevel)
			evts->ev_maxlevel = ih->ih_level;
		evts->ev_imask |= (1 << ih->ih_level);
	}
}

/*
 * Fake interrupt handler structures for the benefit of symmetry with
 * other interrupt sources.
 */
struct intrhand fake_softclock_intrhand;
struct intrhand fake_softnet_intrhand;
struct intrhand fake_softserial_intrhand;
struct intrhand fake_timer_intrhand;
struct intrhand fake_ipi_intrhand;
#if defined(DOM0OPS)
struct intrhand fake_softxenevt_intrhand;

extern void Xsoftxenevt(void);
#endif

/*
 * Event counters for the software interrupts.
 */
#ifdef __HAVE_GENERIC_SOFT_INTERRUPTS
struct evcount softclock_evtcnt;
struct evcount softnet_evtcnt;
struct evcount softtty_evtcnt;
#if defined(DOM0OPS)
struct evcount softxenevt_evtcnt;
#endif
#endif /* __HAVE_GENERIC_SOFT_INTERRUPTS */

int imask[NIPL];	/* Bitmask telling what interrupts are blocked. */
int iunmask[NIPL];	/* Bitmask telling what interrupts are accepted. */

/*
 * Initialize all handlers that aren't dynamically allocated, and exist
 * for each CPU. Also init iunmask[].
 */
void
cpu_intr_init(struct cpu_info *ci)
{
	struct iplsource *ipl;
	int i;

	iunmask[0] = 0xfffffffe;
	imask[0] = 0;
	for (i = 1; i < NIPL; i++) {
		iunmask[i] = iunmask[i - 1] & ~(1 << i);
		imask[i] = ~iunmask[i];
	}

	MALLOC(ipl, struct iplsource *, sizeof (struct iplsource), M_DEVBUF,
	    M_WAITOK);
	if (ipl == NULL)
		panic("can't allocate fixed interrupt source");
	bzero(ipl, sizeof(struct iplsource));
	ipl->ipl_recurse = Xsoftclock;
	ipl->ipl_resume = Xsoftclock;
	fake_softclock_intrhand.ih_level = IPL_SOFTCLOCK;
	ipl->ipl_handlers = &fake_softclock_intrhand;
	ci->ci_isources[SIR_CLOCK] = ipl;
#ifdef __HAVE_GENERIC_SOFT_INTERRUPTS
	evcount_attach(&softclock_evtcnt, "softclock", (void *)
	    &softclock_evtcnt.ec_count, &evcount_intr);
#else
	evcount_attach(&fake_softclock_intrhand.ih_count, "softclock", (void *)
	    &fake_softclock_intrhand.ih_irq, &evcount_intr);
#endif

	MALLOC(ipl, struct iplsource *, sizeof (struct iplsource), M_DEVBUF,
	    M_WAITOK);
	if (ipl == NULL)
		panic("can't allocate fixed interrupt source");
	bzero(ipl, sizeof(struct iplsource));
	ipl->ipl_recurse = Xsoftnet;
	ipl->ipl_resume = Xsoftnet;
	fake_softnet_intrhand.ih_level = IPL_SOFTNET;
	ipl->ipl_handlers = &fake_softnet_intrhand;
	ci->ci_isources[SIR_NET] = ipl;
#ifdef __HAVE_GENERIC_SOFT_INTERRUPTS
	evcount_attach(&softnet_evtcnt, "softnet", (void *)
	    &softnet_evtcnt.ec_count, &evcount_intr);
#else
	evcount_attach(&fake_softnet_intrhand.ih_count, "softnet", (void *)
	    &fake_softnet_intrhand.ih_irq, &evcount_intr);
#endif

	MALLOC(ipl, struct iplsource *, sizeof (struct iplsource), M_DEVBUF,
	    M_WAITOK);
	if (ipl == NULL)
		panic("can't allocate fixed interrupt source");
	bzero(ipl, sizeof(struct iplsource));
	ipl->ipl_recurse = Xsofttty;
	ipl->ipl_resume = Xsofttty;
	fake_softserial_intrhand.ih_level = IPL_SOFTTTY;
	ipl->ipl_handlers = &fake_softserial_intrhand;
	ci->ci_isources[SIR_TTY] = ipl;

#ifdef __HAVE_GENERIC_SOFT_INTERRUPTS
	evcount_attach(&softtty_evtcnt, "softtty",
	    (void *) &softtty_evtcnt.ec_count, &evcount_intr);
#else
	evcount_attach(&fake_softserial_intrhand.ih_count, "softserial",
	    (void *) &fake_softserial_intrhand.ih_irq, &evcount_intr);
#endif

#if defined(DOM0OPS)
	MALLOC(ipl, struct iplsource *, sizeof (struct iplsource), M_DEVBUF,
	    M_WAITOK);
	if (ipl == NULL)
		panic("can't allocate fixed interrupt source");
	bzero(ipl, sizeof(struct iplsource));
	ipl->ipl_recurse = Xsoftxenevt;
	ipl->ipl_resume = Xsoftxenevt;
	fake_softxenevt_intrhand.ih_level = IPL_SOFTXENEVT;
	ipl->ipl_handlers = &fake_softxenevt_intrhand;
	ci->ci_isources[SIR_XENEVT] = ipl;
#ifdef __HAVE_GENERIC_SOFT_INTERRUPTS
	evcount_attach(&softxenevt_evtcnt, "softxenevt",
	    (void *)&softxenevt_evtcnt.ec_count, &evcount_intr);
#else
	evcount_attach(&fake_softxenevt_intrhand.ih_count, "xenevt",
	    (void *)&fake_softxenevt_intrhand.ih_irq, &evcount_intr);
#endif
#endif /* defined(DOM0OPS) */
}

#ifdef INTRDEBUG
void
intr_printconfig(void)
{
	struct iplsource *ipl;
	struct intrhand	*ih;
	int		 i;
	struct cpu_info *ci;
	CPU_INFO_ITERATOR cii;

	CPU_INFO_FOREACH(cii, ci) {
		printf("cpu%d: interrupt masks:\n", ci->ci_apicid);
		for (i = 0; i < NIPL; i++)
			printf("IPL %d mask %lx unmask %lx\n", i,
				(u_long)imask[i], (u_long)iunmask[i]);
		simple_lock(&ci->ci_slock);
		for (i = 0; i < NIPL; i++) {
			ipl = ci->ci_isources[i];
			if (ipl == NULL)
				continue;
			printf("ci%u source %d mask %lx unmask %lx\n",
				ci->ci_apicid, i, (u_long)imask[i], (u_long)iunmask[i]);

			for (ih = ipl->ipl_handlers; ih != NULL;
			     ih = ih->ih_ipl_next)
				printf("\thandler %p fun %p level %d evt_next %p ipl_next %p\n",
				    ih, ih->ih_fun, ih->ih_level,
				    ih->ih_evt_next, ih->ih_ipl_next);
		}
		simple_unlock(&ci->ci_slock);
	}
}
#endif	/* INTRDEBUG */
