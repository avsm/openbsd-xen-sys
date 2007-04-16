/*	$OpenBSD: spinlock.h,v 1.1 1999/01/08 08:25:34 d Exp $	*/

#ifdef I686_CPU
#include <machine/i386/spinlock.h>
#endif

#ifdef amd64
#include <machine/amd64/spinlock.h>
#endif
