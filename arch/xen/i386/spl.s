/*	$OpenBSD: spl.s,v 1.10 2005/12/02 16:58:43 hshoexer Exp $	*/
/*	$NetBSD: icu.s,v 1.45 1996/01/07 03:59:34 mycroft Exp $	*/

/*-
 * Copyright (c) 1993, 1994, 1995 Charles M. Hannum.  All rights reserved.
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
 *	This product includes software developed by Charles M. Hannum.
 * 4. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
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

#include <net/netisr.h>

	.data
	.globl	_C_LABEL(imen),_C_LABEL(ipending),_C_LABEL(netisr)
_C_LABEL(imen):
	.long	0xffff		# interrupt mask enable (all off)
_C_LABEL(ipending):
	.long	0		# interupts pending
_C_LABEL(netisr):
	.long	0		# scheduling bits for network

	.text
/*
 * Process pending interrupts.
 *
 * Important registers:
 *   ebx - cpl
 *   esi - address to resume loop at
 *   edi - scratch for Xsoftnet
 *   ecx - scratch
 */
IDTVEC(spllower)
#if defined(DDB) || defined(GPROF)
	pushl	%ebp
	movl	%esp,%ebp
#endif
	pushl	%ebx
	pushl	%esi
	pushl	%edi
	pushl	%ecx
#if defined(DDB) || defined(GPROF)
	movl	8(%ebp),%ebx
#else
	movl	20(%esp),%ebx
#endif
	movl	$1f,%esi		# address to resume loop at
1:	movl	%ebx,%eax		# get cpl
	movl	_C_LABEL(iunmask)(,%eax,4),%eax
	CLI(%ecx)
	andl	_C_LABEL(ipending),%eax		# any non-masked bits left?
	jz	2f
	bsrl	%eax,%eax
	btrl	%eax,_C_LABEL(ipending)
	movl	CPUVAR(ISOURCES)(,%eax,4),%eax
	jmp	*IS_RECURSE(%eax)
2:	movl	%ebx,CPL
	STIC(%ecx)
	jz	3f
	call	_C_LABEL(stipending)
	testl	%eax,%eax
	jnz	1b
3:	popl	%ecx
	popl	%edi
	popl	%esi
	popl	%ebx
#if defined(DDB) || defined(GPROF)
	leave
#endif
	ret

/*
 * Handle return from interrupt after device handler finishes.
 *
 * Important registers:
 *   ebx - cpl to restore
 *   esi - address to resume loop at
 *   edi - scratch for Xsoftnet
 */
IDTVEC(doreti)
	popl	%ebx			# get previous priority
	decl	CPUVAR(IDEPTH)
	movl	$1f,%esi		# address to resume loop at
1:	movl	%ebx,%eax
	movl	_C_LABEL(iunmask)(,%eax,4),%eax
	CLI(%ecx)
	andl	_C_LABEL(ipending),%eax
	jz	2f
	bsrl	%eax,%eax		# slow, but not worth optimizing
	btrl    %eax,_C_LABEL(ipending)
	movl	CPUVAR(ISOURCES)(,%eax, 4),%eax
	jmp	*IS_RESUME(%eax)
2:	/* Check for ASTs on exit to user mode. */
	movl	%ebx,CPL
3:
	testb   $CHK_UPL,TF_CS(%esp)
	jnz	doreti_checkast
#ifdef VM86
	testl	$PSL_VM,TF_EFLAGS(%esp)
	jz	4f
#else
	jmp	4f
#endif
	.globl doreti_checkast
doreti_checkast:
	CHECK_ASTPENDING(%ecx)
	jz	4f
	CLEAR_ASTPENDING(%ecx)
	STI(%ecx)
	movl	$T_ASTFLT,TF_TRAPNO(%esp)	/* XXX undo later. */
	/* Pushed T_ASTFLT into tf_trapno on entry. */
	pushl	%esp
	call	_C_LABEL(trap)
	addl	$4,%esp
	CLI(%ecx)
	jmp	3b
4:	STIC(%ecx)
	jz	5f
	call	_C_LABEL(stipending)
	testl	%eax,%eax
	jnz	1b
5:	INTRFASTEXIT


/*
 * Soft interrupt handlers
 */

#include "pccom.h"

#define EVCNTLO(x)	_C_LABEL(x) + EV_EVCNTLO
#define EVCNTHI(x)	_C_LABEL(x) + EV_EVCNTHI

IDTVEC(softtty)
	movl	$IPL_SOFTTTY,CPL
	STI(%eax)
	incl	CPUVAR(IDEPTH)
#ifdef MULTIPROCESSOR
	call	_C_LABEL(i386_softintlock)
#endif
#ifdef __HAVE_GENERIC_SOFT_INTERRUPTS
	addl	$1,EVCNTLO(softtty_evtcnt)
	adcl	$0,EVCNTHI(softtty_evtcnt)
	pushl	$X86_SOFTINTR_SOFTTTY
	call	_C_LABEL(softintr_dispatch)
	addl	$4,%esp
#else
#if NPCCOM > 0
	call	_C_LABEL(comsoft)
#endif
#endif /* __HAVE_GENERIC_SOFT_INTERRUPTS */
#ifdef MULTIPROCESSOR	
	call	_C_LABEL(i386_softintunlock)
#endif
	movl	%ebx,CPL
	decl	CPUVAR(IDEPTH)
	jmp	*%esi

	/* XXX Do the legacy netisrs here for now. */
#define DONETISR(s, c) \
	.globl  _C_LABEL(c)	;\
	testl	$(1 << s),%edi	;\
	jz	1f		;\
	call	_C_LABEL(c)	;\
1:

IDTVEC(softnet)
	movl	$IPL_SOFTNET,CPL
	STI(%eax)
	incl	CPUVAR(IDEPTH)
#ifdef MULTIPROCESSOR
	call	_C_LABEL(i386_softintlock)
#endif
#ifdef __HAVE_GENERIC_SOFT_INTERRUPTS
	addl	$1,EVCNTLO(softnet_evtcnt)
	adcl	$0,EVCNTHI(softnet_evtcnt)
#endif /* __HAVE_GENERIC_SOFT_INTERRUPTS */
	xorl	%edi,%edi
	xchgl	_C_LABEL(netisr),%edi

#include <net/netisr_dispatch.h>

#ifdef __HAVE_GENERIC_SOFT_INTERRUPTS
	pushl	$X86_SOFTINTR_SOFTNET
	call	_C_LABEL(softintr_dispatch)
	addl	$4,%esp
#endif /* __HAVE_GENERIC_SOFT_INTERRUPTS */

#ifdef MULTIPROCESSOR	
	call	_C_LABEL(i386_softintunlock)
#endif
 	movl	%ebx,CPL
	decl	CPUVAR(IDEPTH)
	jmp	*%esi
#undef DONETISR

IDTVEC(softclock)
	movl	$IPL_SOFTCLOCK,CPL
	STI(%eax)
	incl	CPUVAR(IDEPTH)
#ifdef MULTIPROCESSOR
	call	_C_LABEL(i386_softintlock)
#endif
#ifdef __HAVE_GENERIC_SOFT_INTERRUPTS
	addl	$1,EVCNTLO(softclock_evtcnt)
	adcl	$0,EVCNTHI(softclock_evtcnt)

	pushl	$X86_SOFTINTR_SOFTCLOCK
	call	_C_LABEL(softintr_dispatch)
	addl	$4,%esp
#else
	call	_C_LABEL(softclock)
#endif /* __HAVE_GENERIC_SOFT_INTERRUPTS */
#ifdef MULTIPROCESSOR	
	call	_C_LABEL(i386_softintunlock)
#endif
	movl	%ebx,CPL
	decl	CPUVAR(IDEPTH)
	jmp	*%esi

#if defined(DOM0OPS)
IDTVEC(softxenevt)
	movl	$IPL_SOFTXENEVT, CPL
	STI(%eax)
	incl	CPUVAR(IDEPTH)
#ifdef MULTIPROCESSOR
	call	_C_LABEL(i386_softintlock)
#endif
#if __HAVE_GENERIC_SOFT_INTERRUPTS
	addl    $1,EVCNTLO(softxenevt_evtcnt)
	adcl    $0,EVCNTHI(softxenevt_evtcnt)
#if 0
	call	_C_LABEL(xenevt_notify)
#endif
#endif /* __HAVE_GENERIC_SOFT_INTERRUPTS */
#ifdef MULTIPROCESSOR
	call	_C_LABEL(x86_softintunlock)
#endif
	decl	CPUVAR(IDEPTH)
	jmp	*%esi
#endif /* defined(DOM0OPS) */
