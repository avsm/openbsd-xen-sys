/*	$OpenBSD: spinlock.h,v 1.1 1999/01/08 08:25:35 d Exp $	*/

#ifndef _M68K_SPINLOCK_H_
#define _M68K_SPINLOCK_H_

#define _SPINLOCK_UNLOCKED	(0)
#define _SPINLOCK_LOCKED	(1)
typedef int _spinlock_lock_t;

#endif
