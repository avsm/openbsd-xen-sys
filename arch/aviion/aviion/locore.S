/*	$OpenBSD: locore.S,v 1.2 2006/05/20 11:57:02 miod Exp $	*/
/*
 * Copyright (c) 2005, Miodrag Vallat.
 * Copyright (c) 1998 Steve Murphree, Jr.
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
/*
 * Mach Operating System
 * Copyright (c) 1993-1991 Carnegie Mellon University
 * Copyright (c) 1991 OMRON Corporation
 * All Rights Reserved.
 *
 * Permission to use, copy, modify and distribute this software and its
 * documentation is hereby granted, provided that both the copyright
 * notice and this permission notice appear in all copies of the
 * software, derivative works or modified versions, and any portions
 * thereof, and that both notices appear in supporting documentation.
 *
 * CARNEGIE MELLON AND OMRON ALLOW FREE USE OF THIS SOFTWARE IN ITS "AS IS"
 * CONDITION.  CARNEGIE MELLON AND OMRON DISCLAIM ANY LIABILITY OF ANY KIND
 * FOR ANY DAMAGES WHATSOEVER RESULTING FROM THE USE OF THIS SOFTWARE.
 *
 * Carnegie Mellon requests users of this software to return to
 *
 *  Software Distribution Coordinator  or  Software.Distribution@CS.CMU.EDU
 *  School of Computer Science
 *  Carnegie Mellon University
 *  Pittsburgh PA 15213-3890
 *
 * any improvements or extensions that they make and grant Carnegie the
 * rights to redistribute these changes.
 */

#include "assym.h"
#include "ksyms.h"

#include <machine/asm.h>
#include <machine/m88100.h>
#include <machine/param.h>
#include <machine/psl.h>
#include <machine/trap.h>
#include <machine/vmparam.h>

#define	SYM_MAGIC	0x6274ef2e

	text

GLOBAL(kernelstart)
GLOBAL(kernel_text)
ASGLOBAL(start)
	/*
	 * A few identical jump instructions to make sure the pipeline is
	 * in a good state. Probably overkill, but it's cheap.
	 */
	br	_ASM_LABEL(main_start)
	br	_ASM_LABEL(main_start)
	br	_ASM_LABEL(main_start)
	br	_ASM_LABEL(main_start)

	/*
	 * Startup code for main processor.
	 */
ASLOCAL(main_start)
	/*
	 * Save the arguments passed by the PROM
	 * 	r2 boot string
	 *	r3 boot device
	 *	r4 boot unit number
	 *	r5 boot partition number
	 *	r6 magic number if not loaded by the PROM
	 *	r7 end of loaded binary if not loaded by the PROM
	 */
	or.u	r13, r0,  hi16(_C_LABEL(prom_bootargs))
	st	r2,  r13, lo16(_C_LABEL(prom_bootargs))
	or.u	r13, r0,  hi16(_C_LABEL(bootdev))
	st	r3,  r13, lo16(_C_LABEL(bootdev))
	or.u	r13, r0,  hi16(_C_LABEL(bootunit))
	st	r4,  r13, lo16(_C_LABEL(bootunit))
	or.u	r13, r0,  hi16(_C_LABEL(bootpart))
	st	r5,  r13, lo16(_C_LABEL(bootpart))
#if defined(DDB) || NKSYMS > 0
	or.u	r12, r0,  hi16(SYM_MAGIC)
	or	r12, r12, lo16(SYM_MAGIC)
	cmp	r2,  r6,  r12
	bcnd	ne0, r2,  1f
	or.u	r13, r0,  hi16(_C_LABEL(esym))
	st	r7,  r13, lo16(_C_LABEL(esym))
1:
#endif

	/* set cputyp */
	ldcr	r1,  PID
	extu	r2,  r1,  8<8>
	bcnd.n	eq0, r2,  1f
	 or.u	r13, r0,  hi16(_C_LABEL(cputyp))
	or.u	r8,  r0,  hi16(CPU_88110)
	br.n	2f
	 or	r8,  r8,  lo16(CPU_88110)
1:
	or.u	r8,  r0,  hi16(CPU_88100)
	or	r8,  r8,  lo16(CPU_88100)
2:
	st	r8,  r13, lo16(_C_LABEL(cputyp))

	/*
	 * CPU Initialization
	 *
	 * I use r11 and r22 here because they're easy to not
	 * get mixed up -- r10, for example, looks too similar
	 * to r0 when not being careful....
	 *
	 * Ensure that the PSR is as we like:
	 *	supervisor mode
	 *	big-endian byte ordering
	 *	concurrent operation allowed
	 *	carry bit clear (I don't think we really care about this)
	 *	FPU enabled
	 *	misaligned access raises an exception
	 *	interrupts disabled
	 *	shadow registers frozen
	 *
	 * The manual says not to disable interrupts and freeze shadowing
	 * at the same time because interrupts are not actually disabled
	 * until after the next instruction. Well, if an interrupt
	 * occurs now, we're in deep trouble anyway, so I'm going to do
	 * the two together.
	 *
	 * Upon a reset (or poweron, I guess), the PSR indicates:
	 *   supervisor mode
	 *   interrupts, shadowing, FPU, misaligned exception: all disabled
	 *
	 * We'll just construct our own turning on what we want.
	 *
	 *	jfriedl@omron.co.jp
	 */

	cmp	r2, r8, CPU_88110	/* r8 contains cputyp */
	bb1	eq, r2, 1f	/* if it's a 'mc88110, skip SSBR */
	stcr	r0, SSBR	/* clear this for later */
1:
	stcr	r0, SR1		/* clear the CPU flags */

	set	r11, r0,  1<PSR_SUPERVISOR_MODE_BIT>
	set	r11, r11, 1<PSR_INTERRUPT_DISABLE_BIT>
	set	r11, r11, 1<PSR_GRAPHICS_DISABLE_BIT>
	/*
	 * XXX On 88110 processors, force serial instruction execution for now.
	 * Situation where OoO would break will be hopefully taken care of in
	 * the near future -- miod
	 */
#if 0
	clr	r11, r11, 1<PSR_SERIAL_MODE_BIT>
#else
	set	r11, r11, 1<PSR_SERIAL_MODE_BIT>
#endif
	set	r11, r11, 1<PSR_SERIALIZE_BIT>
	stcr	r11, PSR
	FLUSH_PIPELINE

	/* save PROM vbr */
	ldcr	r12, VBR
	or.u	r13, r0,  hi16(_C_LABEL(prom_vbr))
	st	r12,  r13, lo16(_C_LABEL(prom_vbr))
	stcr	r0,  VBR

#ifdef MULTIPROCESSOR
	/*
	 * Have curcpu() point at the dummy cpuinfo structure,
	 * so that cpu_number() does not dereference random memory.
	 * This is necessary for early spl usage, despite the fact that
	 * interrupts are disabled...
	 */
	or.u	r11, r0,  hi16(_ASM_LABEL(dummy_cpu))
	or	r11, r11, lo16(_ASM_LABEL(dummy_cpu))
	stcr	r11, CPU

	/*
	 * SCM PROM idles all secondary MPUs upon startup, so at this point
	 * we do not have to compete with them.
	 */
#endif	/* MULTIPROCESSOR */

	/* Switch to interrupt stack */
	or.u	r31, r0,  hi16(_ASM_LABEL(intstack_end))
	or	r31, r31, lo16(_ASM_LABEL(intstack_end))

#ifdef M88110
#ifdef M88100
	cmp	r2, r8, CPU_88110 /* r8 contains cputyp */
	bb1	ne, r2, 1f	/* if it's a 'mc88110, use different vectors */
#endif
	or.u	r3, r0, hi16(_C_LABEL(m88110_vector_list))
	br.n	2f
	 or	r3, r3, lo16(_C_LABEL(m88110_vector_list))
1:
#endif /* M88110 */
#ifdef M88100
	or.u	r3, r0, hi16(_C_LABEL(vector_list))
	or	r3, r3, lo16(_C_LABEL(vector_list))
#endif /* M88100 */
2:
	bsr.n	_C_LABEL(vector_init)
	 ldcr	r2, VBR

	/*
	 * aviion_bootstrap(), among other things, clears proc0's u area.
	 * We are still using the interrupt stack here, thus we are not
	 * affected...
	 */
	bsr	_C_LABEL(aviion_bootstrap)

	/*
	 * ...and we can switch to the u area stack now.
	 */
	ldcr	r10, CPU
	ld	r31, r10, CI_CURPCB

	/* call main() - no arguments although main() still defines one */
	bsr.n	_C_LABEL(main)
	 addu	r31, r31, USIZE

	or.u	r2, r0, hi16(_ASM_LABEL(main_panic))
	bsr.n	_C_LABEL(panic)
	 or	r2, r2, lo16(_ASM_LABEL(main_panic))

	data
	.align	4
ASLOCAL(main_panic)
	string	"main() returned\0"
	text
	.align	8

#ifdef MULTIPROCESSOR

	/*
	 * Startup code for secondary processors.
	 * Some of these initializations are very close to main_start; refer
	 * to the comments there for details.
	 */
GLOBAL(secondary_start)
	or.u	r31, r0,  hi16(_ASM_LABEL(slavestack_end))
	or	r31, r31, lo16(_ASM_LABEL(slavestack_end))

	or.u	r13, r0,  hi16(_C_LABEL(cputyp))
	ld	r8,  r13, lo16(_C_LABEL(cputyp))

	cmp	r2, r8, CPU_88110
	bb1	eq, r2, 1f
	stcr	r0, SSBR
1:
	stcr	r0, SR1

	set	r11, r0,  1<PSR_SUPERVISOR_MODE_BIT>
	set	r11, r11, 1<PSR_INTERRUPT_DISABLE_BIT>
	set	r11, r11, 1<PSR_GRAPHICS_DISABLE_BIT>
	/*
	 * XXX On 88110 processors, force serial instruction execution for now.
	 * Situation where OoO would break will be hopefully taken care of in
	 * the near future -- miod
	 */
#if 0
	clr	r11, r11, 1<PSR_SERIAL_MODE_BIT>
#else
	set	r11, r11, 1<PSR_SERIAL_MODE_BIT>
#endif
	set	r11, r11, 1<PSR_SERIALIZE_BIT>
	stcr	r11, PSR
	FLUSH_PIPELINE
	stcr	r0,  VBR	/* set Vector Base Register to 0, ALWAYS! */
	FLUSH_PIPELINE

	/*
	 * Have curcpu() point at the dummy cpuinfo structure,
	 * so that cpu_number() does not dereference random memory.
	 * This is necessary for early spl usage, despite the fact that
	 * interrupts are disabled...
	 */
	or.u	r11, r0,  hi16(_ASM_LABEL(dummy_cpu))
	or	r11, r11, lo16(_ASM_LABEL(dummy_cpu))
	stcr	r11, CPU

	/*
	 * Since there may be more than one secondary MPU, compete with them
	 * to initialize safely.
	 */
	or.u	r11, r0,  hi16(_C_LABEL(cpu_mutex))
	or	r11, r11, lo16(_C_LABEL(cpu_mutex))
1:
	or	r22, r0,  1
	xmem	r22, r11, r0		/* If r22 gets 0, we have the lock... */
	bcnd	eq0, r22, 4f		/* ...but if not, we must wait */
2:
	/* just watch the lock until it looks clear */
	ld	r22, r11, r0
	bcnd	eq0, r22, 1b
	/* since we can be here with caches off, add a few nops to
	   keep the bus from getting overloaded */
	or	r2,  r0,  lo16(1000)
3:
	subu	r2,  r2,  1
	bcnd	ne0, r2,  3b
	br	1b
4:
	
	/*
	 * While holding the cpu_mutex, the secondary cpu can use the slavestack
	 * to call secondary_pre_main() to determine its cpu number.
	 * After that, however, it should allocate its own stack and switch
	 * to it.
	 */

	bsr	_C_LABEL(secondary_pre_main)	/* set cpu number */

	ldcr	r2, CPU
	ld	r3, r2, CI_IDLE_PCB
	bsr.n	_C_LABEL(secondary_main)
	 addu	r31, r3, USIZE			/* switch to idle stack */

	/*
	 * At this point, the CPU has been correctly initialized and has
	 * identified itself on the console.
	 * All it needs now is to jump to the idle loop and wait for work to
	 * be offered.
	 */
	br	_ASM_LABEL(cpu_switch_idle)

#endif	/* MULTIPROCESSOR */

/* 
 * void delay(int us)
 *
 * The processor loops (busy waits) for the given number of microseconds:
 * Thus, delay(1000000) will delay for one second.
 * (originally from Mach 2.5) 
 */
GLOBAL(delay)
	or.u	r3, r0, hi16(_cpuspeed)
	ld	r3, r3, lo16(_cpuspeed)
	mul	r4, r2, r3
	subu	r4, r4, 4	/* overhead of these instructions */

	/* now loop for the given number of cycles */
1: 
	bcnd.n	gt0, r4, 1b
	 subu	r4, r4, 2	/* two cycles per iteration */

	jmp	r1

/*****************************************************************************/

	data
	.align	PAGE_SIZE
GLOBAL(kernel_sdt)		/* SDT (segment descriptor table */
	space	0x2000		/* 8K - 4K phys, 4K virt*/

	.align	PAGE_SIZE
ASGLOBAL(intstack)
	space	USIZE
ASGLOBAL(intstack_end)

#ifdef MULTIPROCESSOR
	space	PAGE_SIZE	/* 4K, small, interim stack */
ASLOCAL(slavestack_end)
#endif

/*
 * Main processor's idle pcb and stack.
 * Should be page aligned.
 */
	.align	PAGE_SIZE
GLOBAL(idle_u)
	space	USIZE

/*
 * Process 0's u.
 * Should be page aligned.
 */
	.align	PAGE_SIZE
ASLOCAL(u0)
	space	USIZE
GLOBAL(proc0paddr)
	word	_ASM_LABEL(u0)	/*  KVA of proc0 uarea */

#ifdef MULTIPROCESSOR
/* Dummy cpuinfo structure, for cpu_number() to work early. */
ASLOCAL(dummy_cpu)
	word	1	/* ci_alive */
	word	0	/* ci_curproc */
	word	0	/* ci_curpcb */
	word	0	/* ci_cpuid */
#endif	/* MULTIPROCESSOR */

#if defined(DDB) || NKSYMS > 0
GLOBAL(esym)
	word 	0
#endif /* DDB || NKSYMS > 0 */
