/*	$NetBSD: hypervisor_machdep.c,v 1.12 2005/04/30 08:37:16 yamt Exp $	*/

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

/******************************************************************************
 * hypervisor.c
 *
 * Communication to/from hypervisor.
 *
 * Copyright (c) 2002-2004, K A Fraser
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */


#include <sys/cdefs.h>

#include <sys/cdefs.h>
#include <sys/param.h>
#include <sys/systm.h>

#include <machine/xen.h>
#include <machine/hypervisor.h>
#include <machine/evtchn.h>
#include <machine/atomic.h>

#include <machine/cpufunc.h>

/* #define PORT_DEBUG 5 */
/* #define EARLY_DEBUG_EVENT */

#ifdef EARLY_DEBUG_EVENT
extern int debug_port;
extern int xen_debug_handler(void *);
#endif

int stipending(void);
int
stipending(void)
{
	uint32_t l1;
	unsigned long l2;
	unsigned int l1i, l2i, port;
	volatile shared_info_t *s = HYPERVISOR_shared_info;
	struct cpu_info *ci;
	int ret;

	ret = 0;
	ci = curcpu();

#if 0
	if (HYPERVISOR_shared_info->events)
		printf("stipending events %08lx mask %08lx ilevel %d\n",
		    HYPERVISOR_shared_info->events,
		    HYPERVISOR_shared_info->events_mask, ci->ci_ilevel);
#endif

#ifdef EARLY_DEBUG_EVENT
	if (xen_atomic_test_bit(&s->evtchn_pending[0], debug_port)) {
		xen_debug_handler(NULL);
		xen_atomic_clear_bit(&s->evtchn_pending[0], debug_port);
	}
#endif

	/*
	 * we're only called after STIC, so we know that we'll have to
	 * STI at the end
	 */
	while (s->vcpu_data[0].evtchn_upcall_pending) {
		cli();
		s->vcpu_data[0].evtchn_upcall_pending = 0;
		/* NB. No need for a barrier here -- XCHG is a barrier
		 * on x86. */
		l1 = xen_atomic_xchg(&s->evtchn_pending_sel, 0);
		while ((l1i = ffs(l1)) != 0) {
			l1i--;
			l1 &= ~(1 << l1i);

			l2 = s->evtchn_pending[l1i] & ~s->evtchn_mask[l1i];
			/*
			 * mask and clear event. More efficient than calling
			 * hypervisor_mask/clear_event for each event.
			 */
			xen_atomic_setbits_l(&s->evtchn_mask[l1i], l2);
			xen_atomic_clearbits_l(&s->evtchn_pending[l1i], l2);
			while ((l2i = ffs(l2)) != 0) {
				l2i--;
				l2 &= ~(1 << l2i);

				port = (l1i << 5) + l2i;
				if (evtsource[port]) {
					hypervisor_set_ipending(
					    evtsource[port]->ev_imask,
					    l1i, l2i);
					evtsource[port]->ev_evcnt.ec_count++;
					if (ret == 0 && cpl <
					    evtsource[port]->ev_maxlevel)
						ret = 1;
				}
#ifdef DOM0OPS
				else
					xenevt_event(port);
#endif
			}
		}
		sti();
	}

#if 0
	if (ci->ci_ipending & 0x1)
		printf("stipending events %08lx mask %08lx ilevel %d ipending %08x\n",
		    HYPERVISOR_shared_info->events,
		    HYPERVISOR_shared_info->events_mask, ci->ci_ilevel,
		    ci->ci_ipending);
#endif

	return (ret);
}

void
do_hypervisor_callback(struct intrframe *regs)
{
	uint32_t l1;
	unsigned long l2;
	unsigned int l1i, l2i, port;
	volatile shared_info_t *s = HYPERVISOR_shared_info;
	int level;

	level = cpl;

#ifdef EARLY_DEBUG_EVENT
	if (xen_atomic_test_bit(&s->evtchn_pending[0], debug_port)) {
		xen_debug_handler(NULL);
		xen_atomic_clear_bit(&s->evtchn_pending[0], debug_port);
	}
#endif

	while (s->vcpu_data[0].evtchn_upcall_pending) {
		s->vcpu_data[0].evtchn_upcall_pending = 0;
		/* NB. No need for a barrier here -- XCHG is a barrier
		 * on x86. */
		l1 = xen_atomic_xchg(&s->evtchn_pending_sel, 0);
		while ((l1i = ffs(l1)) != 0) {
			l1i--;
			l1 &= ~(1 << l1i);

			l2 = s->evtchn_pending[l1i] & ~s->evtchn_mask[l1i];
			/*
			 * mask and clear the pending events.
			 * Doing it here for all event that will be processed
			 * avoids a race with stipending (which can be called
			 * though evtchn_do_event->splx) that could cause an event to
			 * be both processed and marked pending.
			 */
			xen_atomic_setbits_l(&s->evtchn_mask[l1i], l2);
			xen_atomic_clearbits_l(&s->evtchn_pending[l1i], l2);

			while ((l2i = ffs(l2)) != 0) {
				l2i--;
				l2 &= ~(1 << l2i);

				port = (l1i << 5) + l2i;
#ifdef PORT_DEBUG
				if (port == PORT_DEBUG)
					printf("do_hypervisor_callback event %d\n", port);
#endif
				if (evtsource[port])
					evtchn_do_event(port, regs);
#ifdef DOM0OPS
				else
					xenevt_event(port);
#endif
			}
		}
	}

#ifdef DIAGNOSTIC
	if (level != cpl)
		printf("hypervisor done %08x level %d/%d ipending %08x\n",
		    HYPERVISOR_shared_info->evtchn_pending_sel, level,
		    cpl, ipending);
#endif
}

void
hypervisor_unmask_event(unsigned int ev)
{
	volatile shared_info_t *s = HYPERVISOR_shared_info;
#ifdef PORT_DEBUG
	if (ev == PORT_DEBUG)
		printf("hypervisor_unmask_event %d\n", ev);
#endif

	xen_atomic_clear_bit(&s->evtchn_mask[0], ev);
	/*
	 * The following is basically the equivalent of
	 * 'hw_resend_irq'. Just like a real IO-APIC we 'lose the
	 * interrupt edge' if the channel is masked.
	 */
	if (xen_atomic_test_bit(&s->evtchn_pending[0], ev) &&
	    !xen_atomic_test_and_set_bit(&s->evtchn_pending_sel, ev>>5)) {
		xen_atomic_set_bit(&s->vcpu_data[0].evtchn_upcall_pending, 0);
		if (!s->vcpu_data[0].evtchn_upcall_mask)
			hypervisor_force_callback();
	}
}

void
hypervisor_mask_event(unsigned int ev)
{
	volatile shared_info_t *s = HYPERVISOR_shared_info;
#ifdef PORT_DEBUG
	if (ev == PORT_DEBUG)
		printf("hypervisor_mask_event %d\n", ev);
#endif

	xen_atomic_set_bit(&s->evtchn_mask[0], ev);
}

void
hypervisor_clear_event(unsigned int ev)
{
	volatile shared_info_t *s = HYPERVISOR_shared_info;
#ifdef PORT_DEBUG
	if (ev == PORT_DEBUG)
		printf("hypervisor_clear_event %d\n", ev);
#endif

	xen_atomic_clear_bit(&s->evtchn_pending[0], ev);
}

void
hypervisor_enable_ipl(unsigned int ipl)
{
	u_int32_t l1, l2;
	int l1i, l2i;
	struct cpu_info *ci = curcpu();

	/*
	 * enable all events for ipl. As we only set an event in ipl_evt_mask
	 * for its lowest IPL, and pending IPLs are processed high to low,
	 * we know that all callback for this event have been processed.
	 */

	l1 = ci->ci_isources[ipl]->ipl_evt_mask1;
	ci->ci_isources[ipl]->ipl_evt_mask1 = 0;
	while ((l1i = ffs(l1)) != 0) {
		l1i--;
		l1 &= ~(1 << l1i);
		l2 = ci->ci_isources[ipl]->ipl_evt_mask2[l1i];
		ci->ci_isources[ipl]->ipl_evt_mask2[l1i] = 0;
		while ((l2i = ffs(l2)) != 0) {
			int evtch;

			l2i--;
			l2 &= ~(1 << l2i);

			evtch = (l1i << 5) + l2i;
			hypervisor_enable_event(evtch);
		}
	}
}

void
hypervisor_set_ipending(u_int32_t iplmask, int l1, int l2)
{
	int ipl;
	struct cpu_info *ci = curcpu();

	/* set pending bit for the appropriate IPLs */
	ipending |= iplmask;

	/*
	 * And set event pending bit for the lowest IPL. As IPL are handled
	 * from high to low, this ensure that all callbacks will have been
	 * called when we ack the event
	 */
	ipl = ffs(iplmask);
	KASSERT(ipl > 0);
	ipl--;
	ci->ci_isources[ipl]->ipl_evt_mask1 |= 1 << l1;
	ci->ci_isources[ipl]->ipl_evt_mask2[l1] |= 1 << l2;
}
