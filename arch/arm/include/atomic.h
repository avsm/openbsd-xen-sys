/*	$OpenBSD: atomic.h,v 1.4 2007/02/19 17:18:42 deraadt Exp $	*/

/* Public Domain */

#ifndef __ARM_ATOMIC_H__
#define __ARM_ATOMIC_H__

#if defined(_KERNEL)

/*
 * on pre-v6 arm processors, it is necessary to disable interrupts if
 * in the kernel and atomic updates are necessary without full mutexes
 */

static __inline void
atomic_setbits_int(__volatile unsigned int *uip, unsigned int v)
{
	int oldirqstate;
	oldirqstate = disable_interrupts(I32_bit|F32_bit);
	*uip |= v;
	restore_interrupts(oldirqstate);
}

static __inline void
atomic_clearbits_int(__volatile unsigned int *uip, unsigned int v)
{
	int oldirqstate;
	oldirqstate = disable_interrupts(I32_bit|F32_bit);
	*uip &= ~v;
	restore_interrupts(oldirqstate);
}

#endif /* defined(_KERNEL) */
#endif /* __ARM_ATOMIC_H__ */
