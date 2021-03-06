/*	$OpenBSD: atomic.h,v 1.1 2007/02/06 17:13:33 art Exp $	*/

/* Public Domain */

#ifndef __SPARC_ATOMIC_H__
#define __SPARC_ATOMIC_H__

#if defined(_KERNEL)

static __inline void
atomic_setbits_int(__volatile unsigned int *uip, unsigned int v)
{
	*uip |= v;
}

static __inline void
atomic_clearbits_int(__volatile unsigned int *uip, unsigned int v)
{
	*uip &= ~v;
}

#endif /* defined(_KERNEL) */
#endif /* __SPARC_ATOMIC_H__ */
