/*	$OpenBSD: i82489var.h,v 1.2 2004/06/13 21:49:16 niklas Exp $	*/
/*	$NetBSD: i82489var.h,v 1.1.2.2 2000/02/21 18:46:14 sommerfeld Exp $	*/

/*-
 * Copyright (c) 1998 The NetBSD Foundation, Inc.
 * All rights reserved.
 *
 * This code is derived from software contributed to The NetBSD Foundation
 * by Frank van der Linden.
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
 *	This product includes software developed by the NetBSD
 *	Foundation, Inc. and its contributors.
 * 4. Neither the name of The NetBSD Foundation nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE NETBSD FOUNDATION, INC. AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE FOUNDATION OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _I386_I82489VAR_H_
#define _I386_I82489VAR_H_

static __inline__ u_int32_t i82489_readreg(int);
static __inline__ void i82489_writereg(int, u_int32_t);

#ifdef _KERNEL
extern volatile u_int32_t local_apic[];
#endif

static __inline__ u_int32_t
i82489_readreg(reg)
	int reg;
{
	return *((volatile u_int32_t *)(((volatile u_int8_t *)local_apic)
	    + reg));
}

static __inline__ void
i82489_writereg(reg, val)
	int reg;
	u_int32_t val;
{
	*((volatile u_int32_t *)(((volatile u_int8_t *)local_apic) + reg)) =
	    val;
	/*
	 * intel xeon errata p53:
	 *   write to a lapic register sometimes may appear to have not occured
	 * workaround:
	 *   follow write with a read [from id register]
	 */
	val = *((volatile u_int32_t *)(((volatile u_int8_t *)local_apic) +
	    LAPIC_ID));
}

/*
 * "spurious interrupt vector"; vector used by interrupt which was
 * aborted because the CPU masked it after it happened but before it
 * was delivered.. "Oh, sorry, i caught you at a bad time".
 * Low-order 4 bits must be all ones.
 */
extern void i386_spurious(void);
extern void Xintrspurious(void);
#define LAPIC_SPURIOUS_VECTOR		0xef

/*
 * Vector used for inter-processor interrupts.
 */
extern void Xintripi(void);
#define LAPIC_IPI_VECTOR		IPL_IPI

/*
 * Vector used for local apic timer interrupts.
 */

extern void Xintrltimer(void);
#define LAPIC_TIMER_VECTOR		IPL_CLOCK

/*
 * Vectors to be used for self-soft-interrupts.
 */

#define LAPIC_SOFTCLOCK_VECTOR		IPL_SOFTCLOCK
#define LAPIC_SOFTNET_VECTOR		IPL_SOFTNET
#define LAPIC_SOFTTTY_VECTOR		IPL_SOFTTTY

extern void Xintrsoftclock(void);
extern void Xintrsoftnet(void);
extern void Xintrsofttty(void);

extern void (*apichandler[])(void);

struct cpu_info;

extern void lapic_boot_init(paddr_t);
extern void lapic_initclocks(void);
extern void lapic_set_lvt(void);
extern void lapic_set_softvectors(void);
extern void lapic_enable(void);
extern void lapic_calibrate_timer(struct cpu_info *);

#endif
