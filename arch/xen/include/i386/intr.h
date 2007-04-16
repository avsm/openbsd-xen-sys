/*	$OpenBSD: intr.h,v 1.13 2005/12/01 13:47:44 hshoexer Exp $	*/
/*	$NetBSD: intr.h,v 1.5 1996/05/13 06:11:28 mycroft Exp $	*/

/*
 * Copyright (c) 1996 Charles M. Hannum.  All rights reserved.
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
 *	This product includes software developed by Charles M. Hannum.
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

#ifndef _I386_INTR_H_
#define _I386_INTR_H_

//#warning "intr.h"

#include <machine/intrdefs.h>

#ifndef _LOCORE
#include <machine/cpu.h>
#include <machine/pic.h>

#include <machine/hypervisor.h>


/*
 * Interrupt handler chains.  isa_intr_establish() inserts a handler into
 * the list.  The handler is called with its (single) argument.
 */
#include <sys/evcount.h>

struct intrhand {
	int		(*ih_fun)(void *);
	void		*ih_arg;
	struct intrhand	*ih_ipl_next;
	struct intrhand	*ih_evt_next;
	int		ih_level;
#ifndef __HAVE_GENERIC_SOFT_INTERRUPTS
	int		ih_irq;
	struct evcount	ih_count;
#endif
};



/*
 * Struct describing an event channel.
 */

struct evtsource {
	int ev_maxlevel;		/* max. IPL for this source */
	u_int32_t ev_imask;		/* interrupt mask */
	struct intrhand *ev_handlers;	/* handler chain */
	struct evcount ev_evcnt;	/* interrupt counter */
	char ev_evname[32];		/* event counter name */
};

/*
 * Structure describing an interrupt level. struct cpu_info has an array of
 * IPL_MAX of theses. The index in the array is equal to the stub number of
 * the stubcode as present in vector.s
 */

struct intrstub {
	void *ist_recurse;
	void *ist_resume;
};

struct iplsource {
	struct intrhand *ipl_handlers;   /* handler chain */
	void *ipl_recurse;               /* entry for spllower */
	void *ipl_resume;                /* entry for doreti */
	u_int32_t ipl_evt_mask1;	/* pending events for this IPL */
	u_int32_t ipl_evt_mask2[NR_EVENT_CHANNELS];
};

extern struct intrstub xenev_stubs[];	/* definded in vector.s */

extern volatile u_int32_t cpl;		/* Current interrupt priority level. */

extern volatile u_int32_t ipending;	/* Interrupts pending. */
extern int imask[];	/* Bitmasks telling what interrupts are blocked. */
extern int iunmask[];	/* Bitmasks telling what interrupts are accepted. */

#define IMASK(level) imask[level]
#define IUNMASK(level) iunmask[level]

extern void Xspllower(int nlevel);

extern int splraise(int);
extern int spllower(int);
extern void splx(int);
#ifndef __HAVE_GENERIC_SOFT_INTERRUPTS
extern void softintr(int, int);
#endif

/*
 * compiler barrier: prevent reordering of instructions.
 * XXX something similar will move to <sys/cdefs.h>
 * or thereabouts.
 * This prevents the compiler from reordering code around
 * this "instruction", acting as a sequence point for code generation.
 */

#define	__splbarrier() __asm __volatile("":::"memory")

/* SPL asserts */
#ifdef DIAGNOSTIC
/*
 * Although this function is implemented in MI code, it must be in this MD
 * header because we don't want this header to include MI includes.
 */
void splassert_fail(int, int, const char *);
extern int splassert_ctl;
void splassert_check(int, const char *);
#define splassert(__wantipl) do {			\
	if (__predict_false(splassert_ctl > 0)) {	\
		splassert_check(__wantipl, __func__);	\
	}						\
} while (0)
#else
#define splassert(wantipl) do { /* nada */ } while (0)
#endif

/*
 * Hardware interrupt masks
 */
#define	splbio()	splraise(IPL_BIO)
#define	splnet()	splraise(IPL_NET)
#define	spltty()	splraise(IPL_TTY)
#define	splaudio()	splraise(IPL_AUDIO)
#define	splclock()	splraise(IPL_CLOCK)
#define	splstatclock()	splhigh()
#define splipi()	splraise(IPL_IPI)

/*
 * Software interrupt masks
 */
#define	splsoftclock()		splraise(IPL_SOFTCLOCK)
#define	splsoftnet()		splraise(IPL_SOFTNET)
#define	splsofttty()		splraise(IPL_SOFTTTY)
#define splsoftxenevt()		splraise(IPL_SOFTXENEVT)

/*
 * Miscellaneous
 */
#define	splvm()		splraise(IPL_VM)
#define	splhigh()	splraise(IPL_HIGH)
//#warning "splhigh"
#define	splsched()	splraise(IPL_SCHED)
#define spllock() 	splhigh()
#define	spl0()		spllower(IPL_NONE)

#define	setsoftast()	(astpending = 1)
#ifndef __HAVE_GENERIC_SOFT_INTERRUPTS
#define	setsoftclock()	softintr(1 << SIR_CLOCK, IPL_SOFTCLOCK)
#define	setsoftnet()	softintr(1 << SIR_NET, IPL_SOFTNET)
#define	setsofttty()	softintr(1 << SIR_TTY, IPL_SOFTTTY)
#endif

#define I386_IPI_HALT		0x00000001
#define I386_IPI_MICROSET	0x00000002
#define I386_IPI_FLUSH_FPU	0x00000004
#define I386_IPI_SYNCH_FPU	0x00000008
#define I386_IPI_TLB		0x00000010
#define I386_IPI_MTRR		0x00000020
#define I386_IPI_GDT		0x00000040
#define I386_IPI_DDB		0x00000080	/* synchronize while in ddb */

#define I386_NIPI	8

#ifdef MULTIPROCESSOR
int i386_send_ipi(struct cpu_info *, int);
void i386_broadcast_ipi(int);
void i386_multicast_ipi(int, int);
void i386_ipi_handler(void);
void i386_intlock(int);
void i386_intunlock(int);
void i386_softintlock(void);
void i386_softintunlock(void);

extern void (*ipifunc[I386_NIPI])(struct cpu_info *);
#endif

void intr_calculatemasks(struct evtsource *);
void cpu_intr_init(struct cpu_info *);
#ifdef INTRDEBUG
void intr_printconfig(void);
#endif


#ifdef __HAVE_GENERIC_SOFT_INTERRUPTS
/*
 * Software interrupt registration
 *
 * We hand-code this to ensure that it's atomic.
 *
 * XXX always scheduled on the current CPU.
 */
static inline void
softintr(int sir)
{
	__asm volatile("lock ; orl %1, %0" :
	    "=m"(ipending) : "ir" (1 << sir));
}


/*
 * XXX
 */
#define setsoftnet()	softintr(SIR_NET)
#endif


/*
 * Stub declarations.
 */

extern void Xsoftclock(void);
extern void Xsoftnet(void);
extern void Xsofttty(void);
extern void Xsoftxenevt(void);

#endif /* !_LOCORE */


#ifdef __HAVE_GENERIC_SOFT_INTERRUPTS

/*
 * Generic software interrupt support.
 */
#define X86_SOFTINTR_SOFTCLOCK		0
#define X86_SOFTINTR_SOFTNET		1
#define X86_SOFTINTR_SOFTTTY		2
#define X86_NSOFTINTR			3


#ifndef _LOCORE
#include <sys/queue.h>

struct x86_soft_intrhand {
	TAILQ_ENTRY(x86_soft_intrhand)
		sih_q;
	struct x86_soft_intr *sih_intrhead;
	void	(*sih_fn)(void *);
	void	*sih_arg;
	int	sih_pending;
};

struct x86_soft_intr {
	TAILQ_HEAD(, x86_soft_intrhand)
		softintr_q;
	int softintr_ssir;
	struct simplelock softintr_slock;
};

#define x86_softintr_lock(si, s)				\
do {								\
	(s) = splhigh();					\
	simple_lock(&si->softintr_slock);			\
} while (/*CONSTCOND*/ 0)

#define x86_softintr_unlock(si, s)				\
do {								\
	simple_unlock(&si->softintr_slock);			\
	splx((s));						\
} while (/*CONSTCOND*/ 0)

void *softintr_establish(int, void(*)(void *), void *);
void softintr_disestablish(void *);
void softintr_init(void);
void softintr_dispatch(int);

#define softintr_schedule(arg)						\
do {									\
	struct x86_soft_intrhand *__sih = (arg);			\
	struct x86_soft_intr *__si = __sih->sih_intrhead;		\
	int __s;							\
									\
	x86_softintr_lock(__si, __s);					\
	if (__sih->sih_pending == 0) {					\
		TAILQ_INSERT_TAIL(&__si->softintr_q, __sih, sih_q);	\
		__sih->sih_pending = 1;					\
		softintr(__si->softintr_ssir);				\
	}								\
	x86_softintr_unlock(__si, __s);					\
} while (/*CONSTCOND*/ 0)
#endif /* _LOCORE */

#endif /* __HAVE_GENERIC_SOFT_INTERRUPTS */

#endif /* !_I386_INTR_H_ */
