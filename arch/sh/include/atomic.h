/*	$OpenBSD$	*/

/* Public Domain */

#ifndef __SH_ATOMIC_H__
#define __SH_ATOMIC_H__

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

#endif
