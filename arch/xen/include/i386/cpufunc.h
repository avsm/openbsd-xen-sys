/*	$OpenBSD: cpufunc.h,v 1.6 2005/11/24 16:11:38 hshoexer Exp $	*/
/*	$NetBSD: cpufunc.h,v 1.8 1994/10/27 04:15:59 cgd Exp $	*/

/*
 * Copyright (c) 1993 Charles Hannum.
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
 *      This product includes software developed by Charles Hannum.
 * 4. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission
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

#ifndef _I386_CPUFUNC_H_
#define	_I386_CPUFUNC_H_

#ifdef _KERNEL

/*
 * Functions to provide access to i386-specific instructions.
 */

#include <sys/cdefs.h>
#include <sys/types.h>

#include <machine/segments.h>
#include <machine/specialreg.h>
#include <machine/xen.h>
#include <machine/hypervisor.h>

#define __insn_barrier()        __asm __volatile("":::"memory")

static __inline void x86_lfence(void);
static __inline void x86_sfence(void);
static __inline void lidt(struct region_descriptor *region);
static __inline u_int rcr2(void);
static __inline void lcr4(u_int);
static __inline u_int rcr4(void);
static __inline void disable_intr(void);
static __inline void enable_intr(void);
static __inline u_int read_eflags(void);
static __inline void write_eflags(u_int);
static __inline void wrmsr(u_int, u_int64_t);
static __inline u_int64_t rdmsr(u_int);
static __inline void breakpoint(void);

static __inline void
x86_lfence(void)
{
	/*
	 * XXX it's better to use real lfence insn if available.
	 */
	__asm __volatile("lock; addl $0, 0(%%esp)" : : : "memory");
}

static __inline void
x86_sfence(void)
{
	/*
	 * nothing to do at the CPU level, just put a barrier for compiler
	 */
	__insn_barrier();
}

static __inline void
lidt(struct region_descriptor *region)
{
	__asm __volatile("lidt %0" : : "m" (*region));
}

static __inline u_int
rcr2(void)
{
	return 0;
}

static __inline void
lcr4(u_int val)
{
	__asm __volatile("movl %0,%%cr4" : : "r" (val));
}

static __inline u_int
rcr4(void)
{
	u_int val;
	__asm __volatile("movl %%cr4,%0" : "=r" (val));
	return val;
}

/* XXXX ought to be in psl.h with spl() functions */

static __inline void
disable_intr(void)
{
	__cli();
}

static __inline void
enable_intr(void)
{
	__sti();
}

static __inline u_int
read_eflags(void)
{
	u_int ef;

	__asm __volatile("pushfl; popl %0" : "=r" (ef));
	return (ef);
}

static __inline void
write_eflags(u_int ef)
{
	__asm __volatile("pushl %0; popfl" : : "r" (ef));
}

static __inline void
wrmsr(u_int msr, u_int64_t newval)
{
	__asm __volatile("wrmsr" : : "A" (newval), "c" (msr));
}

static __inline u_int64_t
rdmsr(u_int msr)
{
	u_int64_t rv;

	__asm __volatile("rdmsr" : "=A" (rv) : "c" (msr));
	return (rv);
}

static __inline u_int64_t
xrdtsc(void)
{
	u_int64_t rv;

	__asm __volatile("rdtsc" : "=A" (rv));
	return (rv);
}

static __inline u_int64_t
rdpmc(u_int pmc)
{
	u_int64_t rv;

	__asm __volatile("rdpmc" : "=A" (rv) : "c" (pmc));
	return (rv);
}

/* Break into DDB/KGDB. */
static __inline void
breakpoint(void)
{
	__asm __volatile("int $3");
}

#define read_psl() (HYPERVISOR_shared_info->vcpu_data[0].evtchn_upcall_mask)
#define write_psl(x) do {						\
    __insn_barrier();						\
    HYPERVISOR_shared_info->vcpu_data[0].evtchn_upcall_mask = (x) ;	\
    x86_lfence();							\
    if ((x) == 0 && HYPERVISOR_shared_info->vcpu_data[0].evtchn_upcall_pending) \
    hypervisor_force_callback();					\
} while (0)


#endif /* _KERNEL */
#endif /* !_I386_CPUFUNC_H_ */
