/*	$NetBSD: vector.S,v 1.11 2005/05/31 00:45:05 chs Exp $	*/
/*	NetBSD: 1.13 2004/03/11 11:39:26 yamt Exp 	*/

/*
 * Copyright 2002 (c) Wasabi Systems, Inc.
 * All rights reserved.
 *
 * Written by Frank van der Linden for Wasabi Systems, Inc.
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
 *      This product includes software developed for the NetBSD Project by
 *      Wasabi Systems, Inc.
 * 4. The name of Wasabi Systems, Inc. may not be used to endorse
 *    or promote products derived from this software without specific prior
 *    written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY WASABI SYSTEMS, INC. ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL WASABI SYSTEMS, INC
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*-
 * Copyright (c) 1998 The NetBSD Foundation, Inc.
 * All rights reserved.
 *
 * This code is derived from software contributed to The NetBSD Foundation
 * by Charles M. Hannum.
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
 *        This product includes software developed by the NetBSD
 *        Foundation, Inc. and its contributors.
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

#include <machine/asm.h>
#include <machine/frameasm.h>
#include <machine/segments.h>
#include <machine/trap.h>
#include <machine/intr.h>
#include <machine/psl.h>
#include <machine/xen.h>

#include <net/netisr.h>

#include "npx.h"
#include "assym.h"

/*
 * Macros for interrupt entry, call to handler, and exit.
 *
 * XXX
 * The interrupt frame is set up to look like a trap frame.  This may be a
 * waste.  The only handler which needs a frame is the clock handler, and it
 * only needs a few bits.  Xdoreti() needs a trap frame for handling ASTs, but
 * it could easily convert the frame on demand.
 *
 * The direct costs of setting up a trap frame are two pushl's (error code and
 * trap number), an addl to get rid of these, and pushing and popping the
 * callee-saved registers %esi, %edi, %ebx, and %ebp twice.
 *
 * If the interrupt frame is made more flexible,  INTR can push %eax first and
 * decide the ipending case with less overhead, e.g., by avoiding loading the
 * segment registers.
 *
 */

#define MY_COUNT _C_LABEL(uvmexp)

#ifdef MULTIPROCESSOR
#define LOCK_KERNEL(ipl)	pushl ipl; call _C_LABEL(i386_intlock); addl $4,%esp
#define UNLOCK_KERNEL(ipl)	pushl ipl; call _C_LABEL(i386_intunlock); addl $4,%esp
#else
#define LOCK_KERNEL(ipl)
#define UNLOCK_KERNEL(ipl)
#endif

#define voidop(num)

#ifdef __HAVE_GENERIC_SOFT_INTERRUPTS
#define INTRCNT
#else
#define INTRCNT								\
	movl	IH_ARG(%ebx),%eax					;\
	orl	%eax,%eax		/* should it be counted? */	;\
	jz	3f			/* no, skip it */		;\
	addl	$1,IH_COUNT(%ebx)	/* count the intrs */		;\
	adcl	$0,IH_COUNT+4(%ebx)					;\
3:									;
#endif

#define	XENINTRSTUB(name, num, early_ack, late_ack, mask, unmask, level_mask) \
IDTVEC(recurse_/**/name/**/num)						;\
	pushfl								;\
	pushl	%cs							;\
	pushl	%esi							;\
	pushl	$0			/* dummy error code */		;\
	pushl	$T_ASTFLT		/* trap # for doing ASTs */	;\
	INTRENTRY							;\
	incl	MY_COUNT+V_INTR		/* statistical info */		;\
IDTVEC(resume_/**/name/**/num/**/)					 \
	movl	$IREENT_MAGIC,TF_ERR(%esp)				;\
	movl	%ebx,%esi						;\
	movl	CPUVAR(ISOURCES) + (num) * 4, %ebp			;\
1:									\
	pushl	%esi							;\
	movl	$num,CPL						;\
	STI(%eax)			/* safe to take intrs now */	;\
	incl	CPUVAR(IDEPTH)						;\
	movl	IS_HANDLERS(%ebp),%ebx					;\
	LOCK_KERNEL(IF_PPL(%esp))					;\
6:									\
	pushl	%esp							;\
	pushl	IH_ARG(%ebx)		/* get handler arg */		;\
	call	*IH_FUN(%ebx)		/* call it */			;\
	addl	$8,%esp			/* toss the arg */		;\
	INTRCNT								\
	movl	IH_IPL_NEXT(%ebx),%ebx	/* next handler in chain */	;\
	testl	%ebx,%ebx						;\
	jnz	6b							;\
5:									\
	UNLOCK_KERNEL(IF_PPL(%esp))					;\
	CLI(%eax)							;\
	unmask(num)			/* unmask it in hardware */	;\
	late_ack(num)							;\
	STI(%eax)							;\
	jmp	_C_LABEL(Xdoreti)	/* lower spl and do ASTs */	;\

#if 0
#ifdef DOM0OPS
#define hypervisor_asm_unmask(num)			\
	movl	irq_to_evtchn + (num) * 4,%ecx		;\
	movl	HYPERVISOR_shared_info,%eax		;\
	lock						;\
	btrl	%ecx,EVENTS_MASK(%eax)			;\
	movl	%ebx, %ecx				;\
	movl	$physdev_op_notify, %ebx		;\
	movl	$__HYPERVISOR_physdev_op, %eax		;\
	int	$0x82					;\
	movl	%ecx, %ebx
#else
#define hypervisor_asm_unmask(num)			\
	movl	irq_to_evtchn + (num) * 4,%ecx		;\
	movl	HYPERVISOR_shared_info,%eax		;\
	lock						;\
	btrl	%ecx,EVENTS_MASK(%eax)
#endif	/* DOM0OPS */
#else	/* if 0 */
# Just unmasking the event isn't enouth, we also need to
# reassert the event pending bit if needed. For now just call
# the C function doing it, maybe rewrite in inline assembly ?
#define hypervisor_asm_unmask(num)			\
	pushl $num					;\
	call _C_LABEL(hypervisor_enable_ipl)		;\
	addl	$4,%esp
#endif	/* if 0 */

#if defined(DEBUG) && defined(notdef)
#define STRAY_INITIALIZE \
        xorl    %esi,%esi
#define STRAY_INTEGRATE \
        orl     %eax,%esi
#define STRAY_TEST \
        testl   %esi,%esi                                               ;\
        jz      _C_LABEL(Xstray_/**/name/**/num)
#else /* !DEBUG */
#define STRAY_INITIALIZE
#define STRAY_INTEGRATE
#define STRAY_TEST
#endif /* DEBUG */

#ifdef DDB
#define MAKE_FRAME \
        leal    -8(%esp),%ebp
#else /* !DDB */
#define MAKE_FRAME
#endif /* DDB */

XENINTRSTUB(xenev,0,voidop,voidop,voidop,hypervisor_asm_unmask,voidop)
XENINTRSTUB(xenev,1,voidop,voidop,voidop,hypervisor_asm_unmask,voidop)
XENINTRSTUB(xenev,2,voidop,voidop,voidop,hypervisor_asm_unmask,voidop)
XENINTRSTUB(xenev,3,voidop,voidop,voidop,hypervisor_asm_unmask,voidop)
XENINTRSTUB(xenev,4,voidop,voidop,voidop,hypervisor_asm_unmask,voidop)
XENINTRSTUB(xenev,5,voidop,voidop,voidop,hypervisor_asm_unmask,voidop)
XENINTRSTUB(xenev,6,voidop,voidop,voidop,hypervisor_asm_unmask,voidop)
XENINTRSTUB(xenev,7,voidop,voidop,voidop,hypervisor_asm_unmask,voidop)
XENINTRSTUB(xenev,8,voidop,voidop,voidop,hypervisor_asm_unmask,voidop)
XENINTRSTUB(xenev,9,voidop,voidop,voidop,hypervisor_asm_unmask,voidop)
XENINTRSTUB(xenev,10,voidop,voidop,voidop,hypervisor_asm_unmask,voidop)
XENINTRSTUB(xenev,11,voidop,voidop,voidop,hypervisor_asm_unmask,voidop)
XENINTRSTUB(xenev,12,voidop,voidop,voidop,hypervisor_asm_unmask,voidop)
XENINTRSTUB(xenev,13,voidop,voidop,voidop,hypervisor_asm_unmask,voidop)
XENINTRSTUB(xenev,14,voidop,voidop,voidop,hypervisor_asm_unmask,voidop)
XENINTRSTUB(xenev,15,voidop,voidop,voidop,hypervisor_asm_unmask,voidop)
XENINTRSTUB(xenev,16,voidop,voidop,voidop,hypervisor_asm_unmask,voidop)
XENINTRSTUB(xenev,17,voidop,voidop,voidop,hypervisor_asm_unmask,voidop)
XENINTRSTUB(xenev,18,voidop,voidop,voidop,hypervisor_asm_unmask,voidop)
XENINTRSTUB(xenev,19,voidop,voidop,voidop,hypervisor_asm_unmask,voidop)
XENINTRSTUB(xenev,20,voidop,voidop,voidop,hypervisor_asm_unmask,voidop)
XENINTRSTUB(xenev,21,voidop,voidop,voidop,hypervisor_asm_unmask,voidop)
XENINTRSTUB(xenev,22,voidop,voidop,voidop,hypervisor_asm_unmask,voidop)
XENINTRSTUB(xenev,23,voidop,voidop,voidop,hypervisor_asm_unmask,voidop)
XENINTRSTUB(xenev,24,voidop,voidop,voidop,hypervisor_asm_unmask,voidop)
XENINTRSTUB(xenev,25,voidop,voidop,voidop,hypervisor_asm_unmask,voidop)
XENINTRSTUB(xenev,26,voidop,voidop,voidop,hypervisor_asm_unmask,voidop)
XENINTRSTUB(xenev,27,voidop,voidop,voidop,hypervisor_asm_unmask,voidop)
XENINTRSTUB(xenev,28,voidop,voidop,voidop,hypervisor_asm_unmask,voidop)
XENINTRSTUB(xenev,29,voidop,voidop,voidop,hypervisor_asm_unmask,voidop)
XENINTRSTUB(xenev,30,voidop,voidop,voidop,hypervisor_asm_unmask,voidop)
XENINTRSTUB(xenev,31,voidop,voidop,voidop,hypervisor_asm_unmask,voidop)

.globl _C_LABEL(xenev_stubs)
_C_LABEL(xenev_stubs):
	.long	_C_LABEL(Xrecurse_xenev0), _C_LABEL(Xresume_xenev0)
	.long	_C_LABEL(Xrecurse_xenev1) ,_C_LABEL(Xresume_xenev1)
	.long	_C_LABEL(Xrecurse_xenev2) ,_C_LABEL(Xresume_xenev2)
	.long	_C_LABEL(Xrecurse_xenev3) ,_C_LABEL(Xresume_xenev3)
	.long	_C_LABEL(Xrecurse_xenev4) ,_C_LABEL(Xresume_xenev4)
	.long	_C_LABEL(Xrecurse_xenev5) ,_C_LABEL(Xresume_xenev5)
	.long	_C_LABEL(Xrecurse_xenev6) ,_C_LABEL(Xresume_xenev6)
	.long	_C_LABEL(Xrecurse_xenev7) ,_C_LABEL(Xresume_xenev7)
	.long	_C_LABEL(Xrecurse_xenev8) ,_C_LABEL(Xresume_xenev8)
	.long	_C_LABEL(Xrecurse_xenev9) ,_C_LABEL(Xresume_xenev9)
	.long	_C_LABEL(Xrecurse_xenev10), _C_LABEL(Xresume_xenev10)
	.long	_C_LABEL(Xrecurse_xenev11), _C_LABEL(Xresume_xenev11)
	.long	_C_LABEL(Xrecurse_xenev12), _C_LABEL(Xresume_xenev12)
	.long	_C_LABEL(Xrecurse_xenev13), _C_LABEL(Xresume_xenev13)
	.long	_C_LABEL(Xrecurse_xenev14), _C_LABEL(Xresume_xenev14)
	.long	_C_LABEL(Xrecurse_xenev15), _C_LABEL(Xresume_xenev15)
	.long	_C_LABEL(Xrecurse_xenev16), _C_LABEL(Xresume_xenev16)
	.long	_C_LABEL(Xrecurse_xenev17), _C_LABEL(Xresume_xenev17)
	.long	_C_LABEL(Xrecurse_xenev18), _C_LABEL(Xresume_xenev18)
	.long	_C_LABEL(Xrecurse_xenev19), _C_LABEL(Xresume_xenev19)
	.long	_C_LABEL(Xrecurse_xenev20), _C_LABEL(Xresume_xenev20)
	.long	_C_LABEL(Xrecurse_xenev21), _C_LABEL(Xresume_xenev21)
	.long	_C_LABEL(Xrecurse_xenev22), _C_LABEL(Xresume_xenev22)
	.long	_C_LABEL(Xrecurse_xenev23), _C_LABEL(Xresume_xenev23)
	.long	_C_LABEL(Xrecurse_xenev24), _C_LABEL(Xresume_xenev24)
	.long	_C_LABEL(Xrecurse_xenev25), _C_LABEL(Xresume_xenev25)
	.long	_C_LABEL(Xrecurse_xenev26), _C_LABEL(Xresume_xenev26)
	.long	_C_LABEL(Xrecurse_xenev27), _C_LABEL(Xresume_xenev27)
	.long	_C_LABEL(Xrecurse_xenev28), _C_LABEL(Xresume_xenev28)
	.long	_C_LABEL(Xrecurse_xenev29), _C_LABEL(Xresume_xenev29)
	.long	_C_LABEL(Xrecurse_xenev30), _C_LABEL(Xresume_xenev30)
	.long	_C_LABEL(Xrecurse_xenev31), _C_LABEL(Xresume_xenev31)

IDTVEC(exceptions)
	.long	_C_LABEL(Xtrap00), _C_LABEL(Xtrap01)
	.long	_C_LABEL(Xtrap02), _C_LABEL(Xtrap03)
	.long	_C_LABEL(Xtrap04), _C_LABEL(Xtrap05)
	.long	_C_LABEL(Xtrap06), _C_LABEL(Xtrap07)
	.long	_C_LABEL(Xtrap08), _C_LABEL(Xtrap09)
	.long	_C_LABEL(Xtrap0a), _C_LABEL(Xtrap0b)
	.long	_C_LABEL(Xtrap0c), _C_LABEL(Xtrap0d)
	.long	_C_LABEL(Xtrap0e), _C_LABEL(Xtrap0f)
	.long	_C_LABEL(Xtrap10), _C_LABEL(Xtrap11)
	.long	_C_LABEL(Xtrap12), _C_LABEL(Xtrap13)
	.long	_C_LABEL(Xtrap14), _C_LABEL(Xtrap15)
	.long	_C_LABEL(Xtrap16), _C_LABEL(Xtrap17)
	.long	_C_LABEL(Xtrap18), _C_LABEL(Xtrap19)
	.long	_C_LABEL(Xtrap1a), _C_LABEL(Xtrap1b)
	.long	_C_LABEL(Xtrap1c), _C_LABEL(Xtrap1d)
	.long	_C_LABEL(Xtrap1e), _C_LABEL(Xtrap1f)
