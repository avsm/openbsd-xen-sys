/*	$OpenBSD$	*/

/* Public Domain */

#ifndef __MIPS64_ATOMIC_H__
#define __MIPS64_ATOMIC_H__

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
