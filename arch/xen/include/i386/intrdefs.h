/*	$OpenBSD: intrdefs.h,v 1.8 2005/11/30 17:50:20 hshoexer Exp $	*/
/*	$NetBSD: intrdefs.h,v 1.2 2003/05/04 22:01:56 fvdl Exp $	*/

#ifndef _i386_INTRDEFS_H
#define _i386_INTRDEFS_H

/*
 * XXX Fix comments
 *
 * Intel APICs (advanced programmable interrupt controllers) have
 * bytesized priority registers where the upper nibble is the actual
 * interrupt priority level (a.k.a. IPL).  Interrupt vectors are
 * closely tied to these levels as interrupts whose vectors' upper
 * nibble is lower than or equal to the current level are blocked.
 * Not all 256 possible vectors are available for interrupts in
 * APIC systems, only
 *
 * For systems where instead the older ICU (interrupt controlling
 * unit, a.k.a. PIC or 82C59) is used, the IPL is not directly useful,
 * since the interrupt blocking is handled via interrupt masks instead
 * of levels.  However the IPL is easily used as an offset into arrays
 * of masks.
 */

/*
 * Interrupt priority levels.
 *
 * XXX We are somewhat sloppy about what we mean by IPLs, sometimes
 * XXX we refer to the eight-bit value suitable for storing into APICs'
 * XXX priority registers, other times about the four-bit entity found
 * XXX in the former values' upper nibble, which can be used as offsets
 * XXX in various arrays of our implementation.  We are hoping that
 * XXX the context will provide enough information to not make this
 * XXX sloppy naming a real problem.
 *
 * There are tty, network and disk drivers that use free() at interrupt
 * time, so imp > (tty | net | bio).
 *
 * Since run queues may be manipulated by both the statclock and tty,
 * network, and disk drivers, clock > imp.
 *
 * IPL_HIGH must block everything that can manipulate a run queue.
 *
 * XXX Ultimately we may need serial drivers to run at the absolute highest
 * XXX priority to avoid overruns, then we must make serial > high.
 *
 * The level numbers are picked to fit into APIC vector priorities.
 */
#define	IPL_NONE	0x0		/* nothing */
#define	IPL_SOFTCLOCK	0x1		/* timeouts */
#define	IPL_SOFTNET	0x2		/* protocol stacks */
#define IPL_SOFTXENEVT	0x3		/* /dev/xenevt */
#define	IPL_BIO		0x4		/* block I/O */
#define	IPL_NET		0x5		/* network */
#define	IPL_SOFTTTY	0x6		/* delayed terminal handling */
#define IPL_CTRL	0x7		/* control events */
#define	IPL_TTY		0x8		/* terminal */
#define	IPL_VM		0x9		/* memory allocation */
#define	IPL_AUDIO	0xa		/* audio */
#define	IPL_CLOCK	0xb		/* clock */
#define	IPL_STATCLOCK	IPL_CLOCK	/* statclock */
#define	IPL_SCHED	IPL_CLOCK
#define	IPL_HIGH	0xc		/* everything */
#define	IPL_IPI		0xd		/* interprocessor interrupt */
#define IPL_DEBUG	0xe		/* debug events */
#define IPL_DIE		0xf		/* die events */
#define NIPL		16

/* Interrupt sharing types. */
#define	IST_NONE	0	/* none */
#define	IST_PULSE	1	/* pulsed */
#define	IST_EDGE	2	/* edge-triggered */
#define	IST_LEVEL	3	/* level-triggered */

/* Soft interrupt masks. */
#define	SIR_CLOCK	IPL_SOFTCLOCK
#define	SIR_NET		IPL_SOFTNET
#define SIR_XENEVT	IPL_SOFTXENEVT
#define	SIR_TTY		IPL_SOFTTTY

#define IREENT_MAGIC	0x18041969

#endif /* _I386_INTRDEFS_H */
