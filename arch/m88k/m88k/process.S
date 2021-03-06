/*	$OpenBSD: process.S,v 1.14 2006/11/18 22:49:43 miod Exp $	*/
/*
 * Copyright (c) 1996 Nivas Madhur
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
 *      This product includes software developed by Nivas Madhur.
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
 *
 */

#include "assym.h"
#include <machine/asm.h>
#include <machine/psl.h>
#include <machine/intr.h>

#ifdef DIAGNOSTIC
	data
	align	4
ASLOCAL(swchanpanic)
	string	"switch wchan %x\0"
	align	4
ASLOCAL(swsrunpanic)
	string	"switch SRUN %x\0"

	text
	align	8
ASLOCAL(Lswchanpanic)
	or.u	r2, r0, hi16(_ASM_LABEL(swchanpanic))
	or	r2, r2, lo16(_ASM_LABEL(swchanpanic))
	bsr.n	_C_LABEL(panic)
	 or	r3, r0, r9

ASLOCAL(Lswsrunpanic)
	or.u	r2, r0, hi16(_ASM_LABEL(swsrunpanic))
	or	r2, r2, lo16(_ASM_LABEL(swsrunpanic))
	bsr.n	_C_LABEL(panic)
	 or	r3, r0, r9
#endif

/*
 * void switch_exit(struct proc *p)
 *
 * Do the final work to exit from a process. After switching to the
 * idle stack and pcb, invoke exit2() on behalf of the exiting process,
 * then continue into cpu_switch() to select another process to run.
 */

ENTRY(switch_exit)
	/*
	 * Disable interrupts since we are about to change the kernel
	 * stack.
	 */
	ldcr	r3, PSR
	set	r3, r3, 1<PSR_INTERRUPT_DISABLE_BIT>
	stcr	r3, PSR
	FLUSH_PIPELINE

	/*
	 * Change pcb to idle u. area, i.e., set r31 to top of stack
	 * and set curpcb to point to the cpu's idle stack.
	 * r2 contains proc *p.
	 */
	ldcr	r10, CPU
	ld	r30, r10, CI_IDLE_PCB
	addu	r31, r30, USIZE				/* now on idle stack */
	st	r30, r10, CI_CURPCB			/* curpcb = idle_pcb */

	/* Schedule the vmspace and stack to be freed. */
	bsr.n	_C_LABEL(exit2)
	 st	r0, r10, CI_CURPROC			/* curproc = NULL */

	/*
	 * exit2() has acquired the scheduler lock for us. Jump into
	 * cpu_switch(), after the context save since we do not need
	 * to save anything.
	 */
	br	_ASM_LABEL(cpu_switch_search)

/*
 * void cpu_switch(struct proc *p)
 *
 * Find a runnable process and switch to it. On entry, the scheduler lock is
 * held; it has to be released before returning to the process.
 *
 * Note that this code ignores its proc parameter and assumes it has the
 * same value as curproc. This may change in mi_switch() in the future,
 * be careful.
 */
ENTRY(cpu_switch)
	/*
	 * Disable interrupts, we do not want to be disturbed while
	 * saving context.
	 */
	ldcr	r2, PSR
	set	r2, r2, 1<PSR_INTERRUPT_DISABLE_BIT>
	stcr	r2, PSR
	FLUSH_PIPELINE

	/*
	 * Save state of previous process in its pcb, and pmap_deactivate()
	 * the process.
	 */
	ldcr	r2,  CPU
	ld	r2,  r2,  CI_CURPCB
	st	r1,  r2,  PCB_PC		/* save return address */
	bsr	_ASM_LABEL(__savectx)
	/* note that we don't need to recover r1 at this point */

	ldcr	r11, CPU
	ld	r2,  r11, CI_CURPROC

	/*
	 * Note that we can still use curpcb as our stack after
	 * pmap_deactivate() has been called, as it does not affect the u
	 * area mappings.
	 */
	bsr.n	_C_LABEL(pmap_deactivate)
	 st	r0,  r11, CI_CURPROC		/* curproc = NULL */

#ifdef MULTIPROCESSOR
	/*
	 * We need to switch to the processor's idle stack now (in case the
	 * process we are using the stack of gets scheduled on another
	 * processor).
	 */
	ldcr	r10, CPU
	ld	r30, r10, CI_IDLE_PCB
	addu	r31, r30, USIZE				/* now on idle stack */
	st	r30, r10, CI_CURPCB			/* curpcb = idle_pcb */
#endif

ASLOCAL(cpu_switch_search)
	/*
	 * This is the start of the idle loop. Find the highest-priority
	 * queue that isn't empty, then take the first proc from that queue.
	 */
	or.u	r7, r0, hi16(_C_LABEL(whichqs))
	ld	r7, r7, lo16(_C_LABEL(whichqs))
	bcnd	ne0, r7, _ASM_LABEL(cpu_switch_found)

#if defined(MULTIPROCESSOR) || defined(LOCKDEBUG)
	bsr	_C_LABEL(sched_unlock_idle)
#endif

#ifdef MULTIPROCESSOR
ASGLOBAL(cpu_switch_idle)
#else
ASLOCAL(cpu_switch_idle)
#endif
	/*
	 * There were no runnable processes. Enable all interrupts and
	 * busy-wait for this to change.
	 * Note that, besides doing setipl(IPL_NONE), this will actually enable
	 * interrupts in the psr. Bootstrap of secondary processors
	 * relies upon this.
	 */
	ldcr	r2, PSR
	clr	r2, r2, 1<PSR_INTERRUPT_DISABLE_BIT>
	stcr	r2, PSR
	FLUSH_PIPELINE

	bsr.n	_C_LABEL(setipl)
	 or	r2, r0, IPL_NONE

	or.u	r7, r0, hi16(_C_LABEL(whichqs))
	ld	r7, r7, lo16(_C_LABEL(whichqs))
	bcnd	eq0, r7, _ASM_LABEL(cpu_switch_idle)
	/* XXX run fancy things here, such as page zeroing... */

#if defined(MULTIPROCESSOR) || defined(LOCKDEBUG)
	bsr	_C_LABEL(sched_lock_idle)
#endif

ASLOCAL(cpu_switch_found)
	/*
	 * Disable interrupts.
	 */
	ldcr	r2, PSR
	set	r2, r2, 1<PSR_INTERRUPT_DISABLE_BIT>
	stcr	r2, PSR
	FLUSH_PIPELINE

	/*
	 * An interrupt could have occured between the last whichqs check
	 * and the call to setipl(). Check again that whichqs is nonzero.
	 */
	or.u	r7, r0, hi16(_C_LABEL(whichqs))	/* reload whichqs */
	ld	r7, r7, lo16(_C_LABEL(whichqs))
	bcnd	eq0, r7, _ASM_LABEL(cpu_switch_search)

	/* XXX use ff1, like powerpc... needs *runqueue() adjustments */
	xor	r6, r6, r6		/* set r6 to 0 */
1:	bb1	0,  r7, 2f		/* if rightmost bit set, done */
	extu	r7, r7, 0<1>		/* else, right shift whichqs, */
	br.n	1b			/* increment r6, and repeat */
	 addu	r6, r6, 1
2:
	or.u	r7, r0, hi16(_C_LABEL(qs))
	or	r7, r7, lo16(_C_LABEL(qs))

	/*
	 * Need to make
	 *	p->p_forw->p_back = p->p_back and
	 *	p->p_back->p_forw = p->p_forw where
	 *	p is q->p_forw.
	 * Remember that q->p_forw == p and p->p_back == q.
	 */

	lda.d	r8,  r7[r6]	/* r8 = &qs[ff1(whichqs)] */
	ld	r9,  r8, P_FORW	/* r8 is q, r9 is p */

	ld	r12, r9, P_FORW	/* r12 = p->p_forw */
	st	r8, r12, P_BACK	/* p->p_forw->p_back = q (p->p_back) */
	st	r12, r8, P_FORW	/* q->p_forw = p->p_forw */
	lda.d	r8,  r7[r6]	/* reload r8 with qs[ff1(whichqs)] */
	ld	r12, r8, P_FORW	/* q->p_forw */
	cmp	r12, r12, r8	/* q == q->p_forw; anyone left on queue? */
	bb1	ne,  r12, 3f	/* yes, skip clearing bit in whichqs  */

	or	r12, r0, 1
	mak	r12, r12, r6
	or.u	r7,  r0, hi16(_C_LABEL(whichqs))
	ld	r8,  r7, lo16(_C_LABEL(whichqs))
	and.c	r8,  r8, r12	/* whichqs &= ~the bit */
	st	r8,  r7, lo16(_C_LABEL(whichqs))
3:
#ifdef DIAGNOSTIC
	ld	r2, r9, P_WCHAN
	bcnd	ne0, r2, _ASM_LABEL(Lswchanpanic)
	ld.b	r2, r9, P_STAT
	cmp	r2, r2, SRUN
	bb1	ne, r2, _ASM_LABEL(Lswsrunpanic)
#endif

	ldcr	r11, CPU
	st	r0,  r11, CI_WANT_RESCHED		/* clear want_resched */

	st	r9,  r11, CI_CURPROC			/* curproc = p */
	or	r2,  r0,  SONPROC
	st.b	r2,  r9,  P_STAT

	ld	r3,  r9,  P_ADDR
	st	r0,  r9,  P_BACK			/* p->p_back = 0 */
	st	r3,  r11, CI_CURPCB			/* curpcb = p->p_addr */

	/* pmap_activate() the process' pmap */
	bsr.n	_C_LABEL(pmap_activate)
	 or	r2, r0, r9

	ldcr	r10,  CPU
	ld	r10,  r10, CI_CURPCB

	/* restore from the current context */
	ld	r2,  r10, PCB_FCR62
	ld	r3,  r10, PCB_FCR63
	fstcr	r2,  fcr62
	fstcr	r3,  fcr63
	ld	r15, r10, PCB_R15
	ld	r16, r10, PCB_R16
	ld	r17, r10, PCB_R17
	ld	r18, r10, PCB_R18
	ld	r19, r10, PCB_R19
	ld	r20, r10, PCB_R20
	ld	r21, r10, PCB_R21
	ld	r22, r10, PCB_R22
	ld	r23, r10, PCB_R23
	ld	r24, r10, PCB_R24
	ld	r25, r10, PCB_R25
	ld	r26, r10, PCB_R26
	ld	r27, r10, PCB_R27
	ld	r28, r10, PCB_R28
	ld	r29, r10, PCB_R29
	ld	r30, r10, PCB_R30	/* restore frame pointer & stack */
	ld	r31, r10, PCB_SP
#if defined(MULTIPROCESSOR) || defined(LOCKDEBUG)
	bsr.n	_C_LABEL(sched_unlock_idle)
	 or	r14, r10, r0
	ld	r1,  r14, PCB_PC
	ld	r14, r14, PCB_R14
#else
	ld	r1,  r10, PCB_PC
	ld	r14, r10, PCB_R14
#endif

	/*
	 * Enable interrupts again.
	 */
	ldcr	r2, PSR
	clr	r2, r2, 1<PSR_INTERRUPT_DISABLE_BIT>
	stcr	r2, PSR
	FLUSH_PIPELINE

	jmp	r1

/*
 * savectx(pcb)
 * Update pcb, saving current processor state.
 */
ENTRY(savectx)
	/*
	 * Save preserved general register set.
	 */
	st	r1,  r2,  PCB_PC		/* save return address */
ASLOCAL(__savectx)
	st	r14, r2,  PCB_R14
	st	r15, r2,  PCB_R15
	st	r16, r2,  PCB_R16
	st	r17, r2,  PCB_R17
	st	r18, r2,  PCB_R18
	st	r19, r2,  PCB_R19
	st	r20, r2,  PCB_R20
	st	r21, r2,  PCB_R21
	st	r22, r2,  PCB_R22
	st	r23, r2,  PCB_R23
	st	r24, r2,  PCB_R24
	st	r25, r2,  PCB_R25
	st	r26, r2,  PCB_R26
	st	r27, r2,  PCB_R27
	st	r28, r2,  PCB_R28
	st	r29, r2,  PCB_R29
	st	r30, r2,  PCB_R30	/* save frame pointer & stack pointer */
	st	r31, r2,  PCB_SP

	/*
	 * Save FP state.
	 */
	fldcr	r4,  fcr62
	fldcr	r5,  fcr63
	st	r4,  r2, PCB_FCR62
	jmp.n	r1
	 st	r5,  r2, PCB_FCR63
