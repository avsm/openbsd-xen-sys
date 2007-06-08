/*	$OpenBSD: locore.s,v 1.17 2005/12/02 16:58:43 hshoexer Exp $	*/
/*	$NetBSD: locore.s,v 1.145 1996/05/03 19:41:19 christos Exp $	*/

/*-
 * Copyright (c) 1993, 1994, 1995 Charles M. Hannum.  All rights reserved.
 * Copyright (c) 1990 The Regents of the University of California.
 * All rights reserved.
 *
 * This code is derived from software contributed to Berkeley by
 * William Jolitz.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the University nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 *	@(#)locore.s	7.3 (Berkeley) 5/13/91
 */

#include "npx.h"
#include "assym.h"
#include "lapic.h"
#include "ioapic.h"
#include "ksyms.h"

#include <machine/xen.h>

#include <sys/errno.h>
#include <sys/syscall.h>
#ifdef COMPAT_SVR4
#include <compat/svr4/svr4_syscall.h>
#endif
#ifdef COMPAT_LINUX
#include <compat/linux/linux_syscall.h>
#endif
#ifdef COMPAT_FREEBSD
#include <compat/freebsd/freebsd_syscall.h>
#endif

#include <machine/cputypes.h>
#include <machine/param.h>
#include <machine/pte.h>
#include <machine/segments.h>
#include <machine/specialreg.h>
#include <machine/trap.h>
#include <machine/frameasm.h>

#include <dev/isa/isareg.h>

#if NLAPIC > 0 
#include <machine/i82489reg.h>
#endif

/*
 * override user-land alignment before including asm.h
 */

#define	ALIGN_DATA	.align  4
#define	ALIGN_TEXT	.align  4,0x90	/* 4-byte boundaries, NOP-filled */
#define	SUPERALIGN_TEXT	.align  16,0x90	/* 16-byte boundaries better for 486 */
#define _ALIGN_TEXT	ALIGN_TEXT
#include <machine/asm.h>

/* NB: NOP now preserves registers so NOPs can be inserted anywhere */
/* XXX: NOP and FASTER_NOP are misleadingly named */
#ifdef DUMMY_NOPS	/* this will break some older machines */
#define	FASTER_NOP
#define	NOP
#else
#define	FASTER_NOP	pushl %eax ; inb $0x84,%al ; popl %eax
#define	NOP	pushl %eax ; inb $0x84,%al ; inb $0x84,%al ; popl %eax
#endif

#define CPL _C_LABEL(cpl)

#if defined(MULTIPROCESSOR)
#include <machine/i82489reg.h>

#define	GET_CPUINFO(reg)				\
	movl	_C_LABEL(lapic_id),reg	;		\
	shrl	$LAPIC_ID_SHIFT,reg	; 		\
	movl	_C_LABEL(cpu_info)(,reg,4),reg
#else
#define	GET_CPUINFO(reg)				\
	leal	_C_LABEL(cpu_info_primary),reg
#endif

#define	GET_CURPROC(reg, treg)				\
	GET_CPUINFO(treg)			;	\
	movl	CPU_INFO_CURPROC(treg),reg

#define	PUSH_CURPROC(treg)				\
	GET_CPUINFO(treg)			;	\
	pushl	CPU_INFO_CURPROC(treg)
       
#define	CLEAR_CURPROC(treg)				\
	GET_CPUINFO(treg)			;	\
	movl	$0,CPU_INFO_CURPROC(treg)
       
#define	SET_CURPROC(proc,cpu)				\
	GET_CPUINFO(cpu)			;	\
	movl	proc,CPU_INFO_CURPROC(cpu)	;	\
	movl	cpu,P_CPU(proc)
       
#define	GET_CURPCB(reg)					\
	GET_CPUINFO(reg)			;	\
	movl	CPU_INFO_CURPCB(reg),reg

#define	SET_CURPCB(reg,treg)				\
	GET_CPUINFO(treg)			;	\
	movl	reg,CPU_INFO_CURPCB(treg)

#define	CLEAR_RESCHED(treg)				\
	GET_CPUINFO(treg)			;	\
	xorl	%eax,%eax			;	\
	movl	%eax,CPU_INFO_RESCHED(treg)

#define	CHECK_ASTPENDING(treg)				\
	GET_CPUINFO(treg)			;	\
	cmpl	$0,CPU_INFO_ASTPENDING(treg)
               
#define	CLEAR_ASTPENDING(cireg)				\
	movl	$0,CPU_INFO_ASTPENDING(cireg)

/* We can't use push and pop in INTRENTRY and INTRFASTEXIT.
 * The reason is hypervisor_callback.
 */
/*
 * These are used on interrupt or trap entry or exit.
 */
#define INTRENTRY \
	cld;	\
	subl	$TF_PUSHSIZE,%esp;		\
	movl	%gs,TF_GS(%esp);		\
	movl	%fs,TF_FS(%esp);		\
	movl	%eax,TF_EAX(%esp);		\
	movl	%es,TF_ES(%esp);		\
	movl	%ds,TF_DS(%esp);		\
	movl	$GSEL(GDATA_SEL, SEL_KPL),%eax;	\
	movl	%edi,TF_EDI(%esp);		\
	movl	%esi,TF_ESI(%esp);		\
	movl	%eax,%ds;			\
	movl	%ebp,TF_EBP(%esp);		\
	movl	%eax,%es;			\
	movl	%ebx,TF_EBX(%esp);		\
	movl	%eax,%gs;			\
	movl	%edx,TF_EDX(%esp);		\
	movl	$GSEL(GCPU_SEL, SEL_KPL),%eax;	\
	movl	%ecx,TF_ECX(%esp);		\
	movl	%eax,%fs
		
#define INTRFASTEXIT	\
	movl	TF_GS(%esp),%gs;		\
	movl	TF_FS(%esp),%fs;		\
	movl	TF_ES(%esp),%es;		\
	movl	TF_DS(%esp),%ds;		\
	movl	TF_EDI(%esp),%edi;		\
	movl	TF_ESI(%esp),%esi;		\
	movl	TF_EBP(%esp),%ebp;		\
	movl	TF_EBX(%esp),%ebx;		\
	movl	TF_EDX(%esp),%edx;		\
	movl	TF_ECX(%esp),%ecx;		\
	movl	TF_EAX(%esp),%eax;		\
	addl	$(TF_PUSHSIZE+0x8),%esp;	\
	iret


/*
 * PTmap is recursive pagemap at top of virtual address space.
 * Within PTmap, the page directory can be found (third indirection).
 */
	.globl	_C_LABEL(PTmap), _C_LABEL(PTD), _C_LABEL(PTDpde)
	.set	_C_LABEL(PTmap), (PDSLOT_PTE << PDSHIFT)
	.set	_C_LABEL(PTD), (_C_LABEL(PTmap) + PDSLOT_PTE * NBPG)
	.set	_C_LABEL(PTDpde), (_C_LABEL(PTD) + PDSLOT_PTE * 4)	# XXX 4 == sizeof pde

/*
 * APTmap, APTD is the alternate recursive pagemap.
 * It's used when modifying another process's page tables.
 */
	.globl	_C_LABEL(APTmap), _C_LABEL(APTD), _C_LABEL(APTDpde)
	.set	_C_LABEL(APTmap), (APTDPTDI << PDSHIFT)
	.set	_C_LABEL(APTD), (_C_LABEL(APTmap) + APTDPTDI * NBPG)
	# XXX 4 == sizeof pde
	.set	_C_LABEL(APTDpde), (_C_LABEL(PTD) + APTDPTDI * 4)

/*
 * Xen guest identifier and loader selection
 */
.section __xen_guest
        .ascii  "GUEST_OS=openbsd,GUEST_VER=4.1,XEN_VER=xen-3.0"
	.ascii	",VIRT_BASE=0xd0100000"	/* KERNTEXTOFF */
	.ascii	",ELF_PADDR_OFFSET=0xd0100000"	/* KERNTEXTOFF */
        .ascii	",HYPERCALL_PAGE=0x00000001"
                /* (__PHYSICAL_START+HYPERCALL_PAGE_OFFSET)>>PAGE_SIZE) */
	.ascii	",PAE=no"
        .ascii  ",LOADER=generic"
#if (NKSYMS || defined(DDB) || defined(LKM)) && !defined(SYMTAB_SPACE)
        .ascii  ",BSD_SYMTAB=yes"
#endif
        .byte   0

/*
 * Initialization
 */
	.data

	.globl	_C_LABEL(cpu), _C_LABEL(cpu_id), _C_LABEL(cpu_vendor)
	.globl	_C_LABEL(cpu_brandstr)
	.globl	_C_LABEL(cpuid_level)
	.globl	_C_LABEL(cpu_feature), _C_LABEL(cpu_ecxfeature)
	.globl	_C_LABEL(cpu_cache_eax), _C_LABEL(cpu_cache_ebx)
	.globl	_C_LABEL(cpu_cache_ecx), _C_LABEL(cpu_cache_edx)
	.globl	_C_LABEL(cold), _C_LABEL(cnvmem), _C_LABEL(extmem)
	.globl	_C_LABEL(esym)
	.globl	_C_LABEL(boothowto), _C_LABEL(bootdev), _C_LABEL(atdevbase)
	.globl	_C_LABEL(proc0paddr), _C_LABEL(PTDpaddr)
	.globl	_C_LABEL(gdt)
	.globl	_C_LABEL(bootapiver), _C_LABEL(bootargc), _C_LABEL(bootargv)
#ifndef MULTIPROCESSOR
	.globl	_C_LABEL(curpcb)
#endif
	.globl	_C_LABEL(cpl)

#if NLAPIC > 0
#ifdef __ELF__
	.align NBPG
#else
	.align 12
#endif
	.globl _C_LABEL(local_apic), _C_LABEL(lapic_id)
_C_LABEL(local_apic):
	.space	LAPIC_ID
_C_LABEL(lapic_id):
	.long	0x00000000
	.space	LAPIC_TPRI-(LAPIC_ID+4)
_C_LABEL(cpl):
	.space	LAPIC_PPRI-LAPIC_TPRI
_C_LABEL(lapic_ppr):
	.space	LAPIC_ISR-LAPIC_PPRI
_C_LABEL(lapic_isr):
	.space	NBPG-LAPIC_ISR
#else
_C_LABEL(cpl):
	.long	0
#endif

_C_LABEL(cpu):		.long	0	# are we 386, 386sx, 486, 586 or 686
_C_LABEL(cpu_id):	.long	0	# saved from 'cpuid' instruction
_C_LABEL(cpu_feature):	.long	0	# feature flags from 'cpuid' instruction
_C_LABEL(cpu_ecxfeature):.long	0	# extended feature flags from 'cpuid'
_C_LABEL(cpuid_level):	.long	-1	# max. lvl accepted by 'cpuid' insn
_C_LABEL(cpu_cache_eax):.long	0
_C_LABEL(cpu_cache_ebx):.long	0
_C_LABEL(cpu_cache_ecx):.long	0
_C_LABEL(cpu_cache_edx):.long	0
_C_LABEL(cpu_vendor): .space 16	# vendor string returned by 'cpuid' instruction
_C_LABEL(cpu_brandstr):	.space 48 # brand string returned by 'cpuid'
_C_LABEL(cold):		.long	1	# cold till we are not
_C_LABEL(esym):		.long	0	# ptr to end of syms
_C_LABEL(cnvmem):	.long	0	# conventional memory size
_C_LABEL(extmem):	.long	0	# extended memory size
_C_LABEL(boothowto):	.long	0	# boot flags
_C_LABEL(atdevbase):	.long	0	# location of start of iomem in virtual
_C_LABEL(bootapiver):	.long	0	# /boot API version
_C_LABEL(bootargc):	.long	0	# /boot argc
_C_LABEL(bootargv):	.long	0	# /boot argv
_C_LABEL(bootdev):	.long	0	# device we booted from
_C_LABEL(proc0paddr):	.long	0
_C_LABEL(PTDpaddr):	.long	0	# paddr of PTD, for libkvm

	.space 512
tmpstk:
	.long tmpstk, __KERNEL_DS


#define	RELOC(x)	((x))

	.text
	.globl	start
	.globl	_C_LABEL(kernel_text)
	_C_LABEL(kernel_text) = KERNTEXTOFF
start:
	cld

	movl	%esp,%ebx		# save start of available space

        lss     tmpstk,%esp             # bootstrap stack end location

        /* Clear BSS first so that there are no surprises... */
        xorl    %eax,%eax
        movl    $RELOC(__bss_start),%edi
        movl    $RELOC(_end),%ecx
        subl    %edi,%ecx
        rep stosb

        movl    %ebx,RELOC(avail_start) # now init avail_start in bbs

        /* Copy the necessary stuff from start_info structure. */
        /* We need to copy shared_info early, so that sti/cli work */
        movl    $RELOC(start_info_union),%edi
        movl    $128,%ecx
        rep movsl

        /* (howto, [bootdev], bootinfo, basemem, extmem). */
        xorl    %eax,%eax
        movl    %eax,RELOC(boothowto)
#ifdef COMPAT_OLDBOOT
        movl    %eax,RELOC(bootdev)
#endif  
        movl    $0x20000,%eax
        movl    %eax,RELOC(boothowto)

        /* First, reset the PSL. */
        pushl   $PSL_MBO
        popfl

        /* Clear segment registers; always null in proc0. */
        xorl    %eax,%eax
        movw    %ax,%fs
        movw    %ax,%gs

	/* Find out CPU type.  We are at least an i586. */
        xorl    %eax,%eax
        cpuid
	movl	%eax,RELOC(_C_LABEL(cpuid_level))
	movl	%ebx,RELOC(_C_LABEL(cpu_vendor))	# store vendor string
	movl	%edx,RELOC(_C_LABEL(cpu_vendor))+4
	movl	%ecx,RELOC(_C_LABEL(cpu_vendor))+8
	movl	$0,  RELOC(_C_LABEL(cpu_vendor))+12

	movl	$1,%eax
	cpuid
	movl	%eax,RELOC(_C_LABEL(cpu_id))	# store cpu_id and features
	/*
	 * XXX
	 */
	andl	$~(CPUID_PGE|CPUID_PSE|CPUID_MTRR|CPUID_FXSR|CPUID_SEP),%edx
	movl	%edx,RELOC(_C_LABEL(cpu_feature))
	movl	%ecx,RELOC(_C_LABEL(cpu_ecxfeature))

	movl	RELOC(_C_LABEL(cpuid_level)),%eax
	cmp	$2,%eax
	jl	1f

	movl	$2,%eax
	cpuid
/*
	cmp	$1,%al
	jne	1f
*/

	movl	%eax,RELOC(_C_LABEL(cpu_cache_eax))
	movl	%ebx,RELOC(_C_LABEL(cpu_cache_ebx))
	movl	%ecx,RELOC(_C_LABEL(cpu_cache_ecx))
	movl	%edx,RELOC(_C_LABEL(cpu_cache_edx))

1:
	/* Check if brand identification string is supported */
	movl	$0x80000000,%eax
	cpuid
	cmpl	$0x80000000,%eax
	jbe	2f
	movl	$0x80000002,%eax
	cpuid
	movl	%eax,RELOC(_C_LABEL(cpu_brandstr))
	movl	%ebx,RELOC(_C_LABEL(cpu_brandstr))+4
	movl	%ecx,RELOC(_C_LABEL(cpu_brandstr))+8
	movl	%edx,RELOC(_C_LABEL(cpu_brandstr))+12
	movl	$0x80000003,%eax
	cpuid
	movl	%eax,RELOC(_C_LABEL(cpu_brandstr))+16
	movl	%ebx,RELOC(_C_LABEL(cpu_brandstr))+20
	movl	%ecx,RELOC(_C_LABEL(cpu_brandstr))+24
	movl	%edx,RELOC(_C_LABEL(cpu_brandstr))+28
	movl	$0x80000004,%eax
	cpuid
	movl	%eax,RELOC(_C_LABEL(cpu_brandstr))+32
	movl	%ebx,RELOC(_C_LABEL(cpu_brandstr))+36
	movl	%ecx,RELOC(_C_LABEL(cpu_brandstr))+40
	andl	$0x00ffffff,%edx	/* Shouldn't be necessary */
	movl	%edx,RELOC(_C_LABEL(cpu_brandstr))+44
2:
	xorl	%eax,%eax
/*
 * Virtual address space of kernel:
 *
 * text | data | bss | [syms] | page dir | proc0 kstack | Sysmap
 *			      0          1       2      3
 */
#define	PROC0PDIR	((0)		* NBPG)
#define	PROC0STACK	((1)		* NBPG)
#define	SYSMAP		((1+UPAGES)	* NBPG)
#define	TABLESIZE	((1+UPAGES) * NBPG) /* + _C_LABEL(nkpde) * NBPG */

	/* Find end of kernel image. */
        movl    RELOC(avail_start),%edi
        /* Calculate where to start the bootstrap tables. */
        movl    %edi,%esi

        /* 
         * Calculate the size of the kernel page table directory, and
         * how many entries it will have.
         */
        movl    RELOC(nkpde),%ecx               # get nkpde
        cmpl    $NKPTP_MIN,%ecx                 # larger than min?
        jge     1f
        movl    $NKPTP_MIN,%ecx                 # set at min
        jmp     2f
1:      cmpl    $NKPTP_MAX,%ecx                 # larger than max?
        jle     2f
        movl    $NKPTP_MAX,%ecx
2:

	/* Clear memory for bootstrap tables. */
        shll    $PGSHIFT,%ecx
        addl    $TABLESIZE,%ecx
        addl    %esi,%ecx                       # end of tables
        movl    %ecx,RELOC(gdt)
        addl    $PAGE_SIZE,%ecx
        movl    %ecx,RELOC(avail_start)
        subl    %edi,%ecx                       # size of tables
        shrl    $2,%ecx
        xorl    %eax,%eax
        cld
        rep
        stosl

/*
 * fillkpt
 *	eax = pte (page frame | control | status)
 *	ebx = page table address
 *	ecx = number of pages to map
 */
#define	fillkpt		\
1:	movl	%eax,(%ebx)	; \
	addl	$NBPG,%eax	; /* increment physical address */ \
	addl	$4,%ebx		; /* next pte */ \
	loop	1b		;

/*
 * Build initial page tables.
 */
	/* Calculate end of text segment, rounded to a page. */
	leal	(RELOC(_C_LABEL(etext))+PGOFSET),%edx
	andl	$~PGOFSET,%edx

	/* Skip over the first 1MB. */
	movl	$RELOC(KERNTEXTOFF),%eax
	movl	%eax,%ecx
	subl	$KERNBASE,%ecx
	shrl	$PGSHIFT,%ecx
	leal	(SYSMAP)(%esi,%ecx,4),%ebx

	/* Map the kernel text read-only. */
	movl	%edx,%ecx
	subl	%eax,%ecx
	shrl	$PGSHIFT,%ecx
	orl	$(PG_V|PG_KR),%eax
	fillkpt

	/* Map the data, BSS, and bootstrap tables read-write. */
	movl	RELOC(_C_LABEL(avail_start)),%ecx
	subl	%edx,%ecx				# subtract end of text
	shrl	$PGSHIFT,%ecx
	leal	(PG_V|PG_KW)(%edx),%eax
	fillkpt

	/* map the shared_info page read/write */
	movl	RELOC(_C_LABEL(start_info_union)+START_INFO_SHARED_INFO),%eax
	orl	$(PG_V|PG_RW), %eax
	movl	%eax, (%ebx)
	addl	$4,%ebx

	/* map the console page if not domain 0 */
	movl	RELOC(start_info_union)+START_INFO_FLAGS,%eax
	testl	$SIF_INITDOMAIN, %eax
	jnz	1f
	movl	RELOC(start_info_union)+START_INFO_CONSOLE_MFN,%eax
	shll	$PGSHIFT,%eax
	orl	$(PG_V|PG_RW), %eax
	movl	%eax, (%ebx)
	addl	$4,%ebx
	movl	RELOC(start_info_union)+START_INFO_STORE_MFN,%eax
	shll	$PGSHIFT,%eax
	orl	$(PG_V|PG_RW), %eax
	movl	%eax, (%ebx)
	addl	$4,%ebx
1:

/*
 * Construct a page table directory.
*/
	/* Map kernel PDEs. */
	movl	RELOC(_C_LABEL(nkpde)),%ecx		# count of pde s,
	leal	(PROC0PDIR+PDSLOT_KERN*4)(%esi),%ebx	# where temp maps!
	leal	(SYSMAP+PG_V|PG_KW)(%esi),%eax		# pte for KPT in proc 0
	fillkpt

	/* Install a PDE recursively mapping page directory as a page table! */
	leal	(PROC0PDIR+PG_V/*|PG_KW*/)(%esi),%eax	# pte for ptd XXX
	movl	%eax,(PROC0PDIR+PDSLOT_PTE*4)(%esi)	# recursive PD slot

	/* Save phys. addr of PTD, for libkvm. */
	movl	%esi,RELOC(_C_LABEL(PTDpaddr))

	call	xpmap_init

	/* cr0 is 0x8005003b */

	/* Relocate atdevbase. */
	movl	_C_LABEL(avail_start),%edx
	movl    %edx,_C_LABEL(HYPERVISOR_shared_info)
	addl    $PAGE_SIZE,%edx

	/* set console and xenstore pages virtual address if not domain0 */
	movl	RELOC(start_info_union)+START_INFO_FLAGS,%eax
	testl	$SIF_INITDOMAIN, %eax
	jnz	1f
	movl	%edx,_C_LABEL(xencons_interface)
	addl	$PAGE_SIZE,%edx			# xencons_interface
	movl	%edx,_C_LABEL(xenstore_interface)
	addl	$PAGE_SIZE,%edx			# xenstore_interface
1:
	movl    %edx,_C_LABEL(atdevbase)

	/* Set up bootstrap stack. */
	leal	(PROC0STACK)(%esi),%eax
	movl	%eax,_C_LABEL(proc0paddr)
	leal	(USPACE-FRAMESIZE)(%eax),%esp
	subl	$KERNBASE,%esi
	movl	%esi,PCB_CR3(%eax)	# pcb->pcb_cr3
	xorl	%ebp,%ebp		# mark end of frames

	movl    _C_LABEL(atdevbase),%eax
	pushl	%eax
	call	_C_LABEL(init386)	# wire 386 chip for unix operation
	addl	$4,%esp

	call	_C_LABEL(main)


/* space for the hypercall call page */
#define HYPERCALL_PAGE_OFFSET 0x1000
.org HYPERCALL_PAGE_OFFSET
ENTRY(hypercall_page)
.skip 0x1000

/*
 * void proc_trampoline(void);
 * This is a trampoline function pushed onto the stack of a newly created
 * process in order to do some additional setup.  The trampoline is entered by
 * cpu_switch()ing to the process, so we abuse the callee-saved registers used
 * by cpu_switch() to store the information about the stub to call.
 * NOTE: This function does not have a normal calling sequence!
 */
/* LINTSTUB: Func: void proc_trampoline(void) */
NENTRY(proc_trampoline)
#ifdef MULTIPROCESSOR
	call	_C_LABEL(proc_trampoline_mp)
#endif
	movl	$IPL_NONE,CPL
	pushl	%ebx
	call	*%esi
	addl	$4,%esp
	INTRFASTEXIT
	/* NOTREACHED */

/*****************************************************************************/

/*
 * Signal trampoline; copied to top of user stack.
 */
NENTRY(sigcode)
	movl	SIGF_FPSTATE(%esp),%esi	# FPU state area if need saving
	testl	%esi,%esi
	jz	1f
	fnsave	(%esi)
1:	call	*SIGF_HANDLER(%esp)
	testl	%esi,%esi
	jz	2f
	frstor	(%esi)
	jmp	2f

	.globl  _C_LABEL(sigcode_xmm)
_C_LABEL(sigcode_xmm):
	movl	SIGF_FPSTATE(%esp),%esi	# FPU state area if need saving
	testl	%esi,%esi
	jz	1f
	fxsave	(%esi)
	fninit
1:	call	*SIGF_HANDLER(%esp)
	testl	%esi,%esi
	jz	2f
	fxrstor	(%esi)

2:	leal	SIGF_SC(%esp),%eax	# scp (the call may have clobbered the
					# copy at SIGF_SCP(%esp))
	pushl	%eax
	pushl	%eax			# junk to fake return address
	movl	$SYS_sigreturn,%eax
	int	$0x80			# enter kernel with args on stack
	movl	$SYS_exit,%eax
	int	$0x80			# exit if sigreturn fails
	.globl	_C_LABEL(esigcode)
_C_LABEL(esigcode):

/*****************************************************************************/

#ifdef COMPAT_SVR4
NENTRY(svr4_sigcode)
	call	*SVR4_SIGF_HANDLER(%esp)
	leal	SVR4_SIGF_UC(%esp),%eax	# ucp (the call may have clobbered the
					# copy at SIGF_UCP(%esp))
	pushl	%eax
	pushl	$1			# setcontext(p) == syscontext(1, p)
	pushl	%eax			# junk to fake return address
	movl	$SVR4_SYS_context,%eax
	int	$0x80			# enter kernel with args on stack
	movl	$SVR4_SYS_exit,%eax
	int	$0x80			# exit if sigreturn fails
	.globl	_C_LABEL(svr4_esigcode)
_C_LABEL(svr4_esigcode):
#endif

/*****************************************************************************/

#ifdef COMPAT_LINUX
/*
 * Signal trampoline; copied to top of user stack.
 */
NENTRY(linux_sigcode)
	call	*LINUX_SIGF_HANDLER(%esp)
	leal	LINUX_SIGF_SC(%esp),%ebx # scp (the call may have clobbered the
					# copy at SIGF_SCP(%esp))
	pushl	%eax			# junk to fake return address
	movl	$LINUX_SYS_sigreturn,%eax
	int	$0x80			# enter kernel with args on stack
	movl	$LINUX_SYS_exit,%eax
	int	$0x80			# exit if sigreturn fails
	.globl	_C_LABEL(linux_esigcode)
_C_LABEL(linux_esigcode):
#endif

/*****************************************************************************/

#ifdef COMPAT_FREEBSD
/*
 * Signal trampoline; copied to top of user stack.
 */
NENTRY(freebsd_sigcode)
	call	*FREEBSD_SIGF_HANDLER(%esp)
	leal	FREEBSD_SIGF_SC(%esp),%eax # scp (the call may have clobbered
					# the copy at SIGF_SCP(%esp))
	pushl	%eax
	pushl	%eax			# junk to fake return address
	movl	$FREEBSD_SYS_sigreturn,%eax
	int	$0x80			# enter kernel with args on stack
	movl	$FREEBSD_SYS_exit,%eax
	int	$0x80			# exit if sigreturn fails
	.globl	_C_LABEL(freebsd_esigcode)
_C_LABEL(freebsd_esigcode):
#endif

/*****************************************************************************/

/*
 * The following primitives are used to fill and copy regions of memory.
 */

/*
 * void lgdt_finish(void);
 * Finish load a new GDT pointer (do any necessary cleanup). 
 */
/* LINTSTUB: Func: void lgdt_finish(void) */
NENTRY(lgdt_finish)
        movl    $GSEL(GDATA_SEL, SEL_KPL),%eax
        movw    %ax,%ds
        movw    %ax,%es
        movw    %ax,%gs
        movw    %ax,%ss
        movl    $GSEL(GCPU_SEL, SEL_KPL),%eax
        movw    %ax,%fs
        /* Reload code selector by doing intersegment return. */
        popl    %eax
        pushl   $GSEL(GCODE_SEL, SEL_KPL)
        pushl   %eax
        lret

/*****************************************************************************/

/*
 * fillw(short pattern, caddr_t addr, size_t len);
 * Write len copies of pattern at addr.
 */
ENTRY(fillw)
	pushl	%edi
	movl	8(%esp),%eax
	movl	12(%esp),%edi
	movw	%ax,%cx
	rorl	$16,%eax
	movw	%cx,%ax
	cld
	movl	16(%esp),%ecx
	shrl	%ecx			# do longwords
	rep
	stosl
	movl	16(%esp),%ecx
	andl	$1,%ecx			# do remainder
	rep
	stosw
	popl	%edi
	ret


/* Frame pointer reserve on stack. */
#ifdef DDB
#define FPADD 4
#else
#define FPADD 0
#endif

/*
 * kcopy(caddr_t from, caddr_t to, size_t len);
 * Copy len bytes, abort on fault.
 */
ENTRY(kcopy)
#ifdef DDB
	pushl	%ebp
	movl	%esp,%ebp
#endif
	pushl	%esi
	pushl	%edi
	GET_CURPCB(%eax)		# load curpcb into eax and set on-fault
	pushl	PCB_ONFAULT(%eax)
	movl	$_C_LABEL(copy_fault), PCB_ONFAULT(%eax)

	movl	16+FPADD(%esp),%esi
	movl	20+FPADD(%esp),%edi
	movl	24+FPADD(%esp),%ecx
	movl	%edi,%eax
	subl	%esi,%eax
	cmpl	%ecx,%eax		# overlapping?
	jb	1f
	cld				# nope, copy forward
	shrl	$2,%ecx			# copy by 32-bit words
	rep
	movsl
	movl	24+FPADD(%esp),%ecx
	andl	$3,%ecx			# any bytes left?
	rep
	movsb

	GET_CURPCB(%edx)		# XXX save curpcb?
	popl	PCB_ONFAULT(%edx)
	popl	%edi
	popl	%esi
	xorl	%eax,%eax
#ifdef DDB
	leave
#endif
	ret

	ALIGN_TEXT
1:	addl	%ecx,%edi		# copy backward
	addl	%ecx,%esi
	std
	andl	$3,%ecx			# any fractional bytes?
	decl	%edi
	decl	%esi
	rep
	movsb
	movl	24+FPADD(%esp),%ecx	# copy remainder by 32-bit words
	shrl	$2,%ecx
	subl	$3,%esi
	subl	$3,%edi
	rep
	movsl
	cld

	GET_CURPCB(%edx)
	popl	PCB_ONFAULT(%edx)
	popl	%edi
	popl	%esi
	xorl	%eax,%eax
#ifdef DDB
	leave
#endif
	ret
	
/*
 * bcopy(caddr_t from, caddr_t to, size_t len);
 * Copy len bytes.
 */
ALTENTRY(ovbcopy)
ENTRY(bcopy)
	pushl	%esi
	pushl	%edi
	movl	12(%esp),%esi
	movl	16(%esp),%edi
	movl	20(%esp),%ecx
	movl	%edi,%eax
	subl	%esi,%eax
	cmpl	%ecx,%eax		# overlapping?
	jb	1f
	cld				# nope, copy forward
	shrl	$2,%ecx			# copy by 32-bit words
	rep
	movsl
	movl	20(%esp),%ecx
	andl	$3,%ecx			# any bytes left?
	rep
	movsb
	popl	%edi
	popl	%esi
	ret

	ALIGN_TEXT
1:	addl	%ecx,%edi		# copy backward
	addl	%ecx,%esi
	std
	andl	$3,%ecx			# any fractional bytes?
	decl	%edi
	decl	%esi
	rep
	movsb
	movl	20(%esp),%ecx		# copy remainder by 32-bit words
	shrl	$2,%ecx
	subl	$3,%esi
	subl	$3,%edi
	rep
	movsl
	popl	%edi
	popl	%esi
	cld
	ret

/*
 * Emulate memcpy() by swapping the first two arguments and calling bcopy()
 */
ENTRY(memcpy)
	movl	4(%esp),%ecx
	xchg	8(%esp),%ecx
	movl	%ecx,4(%esp)
	jmp	_C_LABEL(bcopy)

/*****************************************************************************/

/*
 * The following primitives are used to copy data in and out of the user's
 * address space.
 */

/*
 * copyout(caddr_t from, caddr_t to, size_t len);
 * Copy len bytes into the user's address space.
 */
ENTRY(copyout)
#ifdef DDB
	pushl	%ebp
	movl	%esp,%ebp
#endif
	pushl	%esi
	pushl	%edi
	pushl	$0	
	
	movl	16+FPADD(%esp),%esi
	movl	20+FPADD(%esp),%edi
	movl	24+FPADD(%esp),%eax

	/*
	 * We check that the end of the destination buffer is not past the end
	 * of the user's address space.  If it's not, then we only need to
	 * check that each page is writable.  The 486 will do this for us; the
	 * 386 will not.  (We assume that pages in user space that are not
	 * writable by the user are not writable by the kernel either.)
	 */
	movl	%edi,%edx
	addl	%eax,%edx
	jc	_C_LABEL(copy_fault)
	cmpl	$VM_MAXUSER_ADDRESS,%edx
	ja	_C_LABEL(copy_fault)

#if defined(I386_CPU)
#if defined(I486_CPU) || defined(I586_CPU) || defined(I686_CPU)
	cmpl	$CPUCLASS_386,_C_LABEL(cpu_class)
	jne	3f
#endif /* I486_CPU || I586_CPU || I686_CPU */

	testl	%eax,%eax		# anything to do?
	jz	3f

	/*
	 * We have to check each PTE for (write) permission, since the CPU
	 * doesn't do it for us.
	 */

	/* Compute number of pages. */
	movl	%edi,%ecx
	andl	$PGOFSET,%ecx
	addl	%eax,%ecx
	decl	%ecx
	shrl	$PGSHIFT,%ecx

	/* Compute PTE offset for start address. */
	shrl	$PGSHIFT,%edi

	GET_CURPCB(%edx)
	movl	$2f, PCB_ONFAULT(%edx)

1:	/* Check PTE for each page. */
	testb	$PG_RW,_C_LABEL(PTmap)(,%edi,4)
	jz	2f

4:	incl	%edi
	decl	%ecx
	jns	1b

	movl	20+FPADD(%esp),%edi
	movl	24+FPADD(%esp),%eax
	jmp	3f

2:	/* Simulate a trap. */
	pushl	%ecx
	movl	%edi,%eax
	shll	$PGSHIFT,%eax
	pushl	%eax
	call	_C_LABEL(trapwrite)	# trapwrite(addr)
	addl	$4,%esp			# pop argument
	popl	%ecx
	testl	%eax,%eax		# if not ok, return EFAULT
	jz	4b
	jmp	_C_LABEL(copy_fault)
#endif /* I386_CPU */

3:	GET_CURPCB(%edx)
	movl	$_C_LABEL(copy_fault),PCB_ONFAULT(%edx)

	/* bcopy(%esi, %edi, %eax); */
	cld
	movl	%eax,%ecx
	shrl	$2,%ecx
	rep
	movsl
	movl	%eax,%ecx
	andl	$3,%ecx
	rep
	movsb

	popl	PCB_ONFAULT(%edx)
	popl	%edi
	popl	%esi
	xorl	%eax,%eax
#ifdef DDB
	leave
#endif
	ret

/*
 * copyin(caddr_t from, caddr_t to, size_t len);
 * Copy len bytes from the user's address space.
 */
ENTRY(copyin)
#ifdef DDB
	pushl	%ebp
	movl	%esp,%ebp
#endif
	pushl	%esi
	pushl	%edi
	GET_CURPCB(%eax)
	pushl	$0
	movl	$_C_LABEL(copy_fault),PCB_ONFAULT(%eax)
	
	movl	16+FPADD(%esp),%esi
	movl	20+FPADD(%esp),%edi
	movl	24+FPADD(%esp),%eax

	/*
	 * We check that the end of the destination buffer is not past the end
	 * of the user's address space.  If it's not, then we only need to
	 * check that each page is readable, and the CPU will do that for us.
	 */
	movl	%esi,%edx
	addl	%eax,%edx
	jc	_C_LABEL(copy_fault)
	cmpl	$VM_MAXUSER_ADDRESS,%edx
	ja	_C_LABEL(copy_fault)

3:	/* bcopy(%esi, %edi, %eax); */
	cld
	movl	%eax,%ecx
	shrl	$2,%ecx
	rep
	movsl
	movb	%al,%cl
	andb	$3,%cl
	rep
	movsb

	GET_CURPCB(%edx)
	popl	PCB_ONFAULT(%edx)
	popl	%edi
	popl	%esi
	xorl	%eax,%eax
#ifdef DDB
	leave
#endif
	ret

ENTRY(copy_fault)
	GET_CURPCB(%edx)
	popl	PCB_ONFAULT(%edx)
	popl	%edi
	popl	%esi
	movl	$EFAULT,%eax
#ifdef DDB
	leave
#endif
	ret

/*
 * copyoutstr(caddr_t from, caddr_t to, size_t maxlen, size_t *lencopied);
 * Copy a NUL-terminated string, at most maxlen characters long, into the
 * user's address space.  Return the number of characters copied (including the
 * NUL) in *lencopied.  If the string is too long, return ENAMETOOLONG; else
 * return 0 or EFAULT.
 */
ENTRY(copyoutstr)
#ifdef DDB
	pushl	%ebp
	movl	%esp,%ebp
#endif
	pushl	%esi
	pushl	%edi

	movl	12+FPADD(%esp),%esi		# esi = from
	movl	16+FPADD(%esp),%edi		# edi = to
	movl	20+FPADD(%esp),%edx		# edx = maxlen

#if defined(I386_CPU)
#if defined(I486_CPU) || defined(I586_CPU) || defined(I686_CPU)
	cmpl	$CPUCLASS_386,_C_LABEL(cpu_class)
	jne	5f
#endif /* I486_CPU || I586_CPU || I686_CPU */

	/* Compute number of bytes in first page. */
	movl	%edi,%eax
	andl	$PGOFSET,%eax
	movl	$NBPG,%ecx
	subl	%eax,%ecx		# ecx = NBPG - (src % NBPG)

	GET_CURPCB(%ecx)
	movl	$6f, PCB_ONFAULT(%ecx)

1:	/*
	 * Once per page, check that we are still within the bounds of user
	 * space, and check for a write fault.
	 */
	cmpl	$VM_MAXUSER_ADDRESS,%edi
	jae	_C_LABEL(copystr_fault)

	/* Compute PTE offset for start address. */
	movl	%edi,%eax
	shrl	$PGSHIFT,%eax		# calculate pte address

	testb	$PG_RW,_C_LABEL(PTmap)(,%eax,4)
	jnz	2f

6:	/* Simulate a trap. */
	pushl	%edx
	pushl	%edi
	call	_C_LABEL(trapwrite)	# trapwrite(addr)
	addl	$4,%esp			# clear argument from stack
	popl	%edx
	testl	%eax,%eax
	jnz	_C_LABEL(copystr_fault)

2:	/* Copy up to end of this page. */
	subl	%ecx,%edx		# predecrement total count
	jnc	3f
	addl	%edx,%ecx		# ecx += (edx - ecx) = edx
	xorl	%edx,%edx

3:	decl	%ecx
	js	4f
	lodsb
	stosb
	testb	%al,%al
	jnz	3b

	/* Success -- 0 byte reached. */
	addl	%ecx,%edx		# add back residual for this page
	xorl	%eax,%eax
	jmp	copystr_return

4:	/* Go to next page, if any. */
	movl	$NBPG,%ecx
	testl	%edx,%edx
	jnz	1b

	/* edx is zero -- return ENAMETOOLONG. */
	movl	$ENAMETOOLONG,%eax
	jmp	copystr_return
#endif /* I386_CPU */

#if defined(I486_CPU) || defined(I586_CPU) || defined(I686_CPU)
5:	GET_CURPCB(%eax)
	movl	$_C_LABEL(copystr_fault),PCB_ONFAULT(%eax)
	/*
	 * Get min(%edx, VM_MAXUSER_ADDRESS-%edi).
	 */
	movl	$VM_MAXUSER_ADDRESS,%eax
	subl	%edi,%eax
	cmpl	%edx,%eax
	jae	1f
	movl	%eax,%edx
	movl	%eax,20+FPADD(%esp)

1:	incl	%edx
	cld

1:	decl	%edx
	jz	2f
	cmpl	$VM_MAXUSER_ADDRESS,%edi
	jae	_C_LABEL(copystr_fault)
	lodsb
	stosb
	testb	%al,%al
	jnz	1b

	/* Success -- 0 byte reached. */
	decl	%edx
	xorl	%eax,%eax
	jmp	copystr_return

2:	/* edx is zero -- return EFAULT or ENAMETOOLONG. */
	cmpl	$VM_MAXUSER_ADDRESS,%edi
	jae	_C_LABEL(copystr_fault)
	movl	$ENAMETOOLONG,%eax
	jmp	copystr_return
#endif /* I486_CPU || I586_CPU || I686_CPU */

/*
 * copyinstr(caddr_t from, caddr_t to, size_t maxlen, size_t *lencopied);
 * Copy a NUL-terminated string, at most maxlen characters long, from the
 * user's address space.  Return the number of characters copied (including the
 * NUL) in *lencopied.  If the string is too long, return ENAMETOOLONG; else
 * return 0 or EFAULT.
 */
ENTRY(copyinstr)
#ifdef DDB
	pushl	%ebp
	movl	%esp,%ebp
#endif
	pushl	%esi
	pushl	%edi
	GET_CURPCB(%ecx)
	movl	$_C_LABEL(copystr_fault),PCB_ONFAULT(%ecx)

	movl	12+FPADD(%esp),%esi		# %esi = from
	movl	16+FPADD(%esp),%edi		# %edi = to
	movl	20+FPADD(%esp),%edx		# %edx = maxlen

	/*
	 * Get min(%edx, VM_MAXUSER_ADDRESS-%esi).
	 */
	movl	$VM_MAXUSER_ADDRESS,%eax
	subl	%esi,%eax
	cmpl	%edx,%eax
	jae	1f
	movl	%eax,%edx
	movl	%eax,20+FPADD(%esp)

1:	incl	%edx
	cld

1:	decl	%edx
	jz	2f
	cmpl    $VM_MAXUSER_ADDRESS,%esi
	jae     _C_LABEL(copystr_fault)
	lodsb
	stosb
	testb	%al,%al
	jnz	1b

	/* Success -- 0 byte reached. */
	decl	%edx
	xorl	%eax,%eax
	jmp	copystr_return

2:	/* edx is zero -- return EFAULT or ENAMETOOLONG. */
	cmpl	$VM_MAXUSER_ADDRESS,%esi
	jae	_C_LABEL(copystr_fault)
	movl	$ENAMETOOLONG,%eax
	jmp	copystr_return

ENTRY(copystr_fault)
	movl	$EFAULT,%eax

copystr_return:
	/* Set *lencopied and return %eax. */
	GET_CURPCB(%ecx)
	movl	$0,PCB_ONFAULT(%ecx)
	movl	20+FPADD(%esp),%ecx
	subl	%edx,%ecx
	movl	24+FPADD(%esp),%edx
	testl	%edx,%edx
	jz	8f
	movl	%ecx,(%edx)

8:	popl	%edi
	popl	%esi
#ifdef DDB
	leave
#endif
	ret

/*
 * copystr(caddr_t from, caddr_t to, size_t maxlen, size_t *lencopied);
 * Copy a NUL-terminated string, at most maxlen characters long.  Return the
 * number of characters copied (including the NUL) in *lencopied.  If the
 * string is too long, return ENAMETOOLONG; else return 0.
 */
ENTRY(copystr)
#ifdef DDB
	pushl	%ebp
	movl	%esp,%ebp
#endif
	pushl	%esi
	pushl	%edi

	movl	12+FPADD(%esp),%esi		# esi = from
	movl	16+FPADD(%esp),%edi		# edi = to
	movl	20+FPADD(%esp),%edx		# edx = maxlen
	incl	%edx
	cld

1:	decl	%edx
	jz	4f
	lodsb
	stosb
	testb	%al,%al
	jnz	1b

	/* Success -- 0 byte reached. */
	decl	%edx
	xorl	%eax,%eax
	jmp	6f

4:	/* edx is zero -- return ENAMETOOLONG. */
	movl	$ENAMETOOLONG,%eax

6:	/* Set *lencopied and return %eax. */
	movl	20+FPADD(%esp),%ecx
	subl	%edx,%ecx
	movl	24+FPADD(%esp),%edx
	testl	%edx,%edx
	jz	7f
	movl	%ecx,(%edx)

7:	popl	%edi
	popl	%esi
#ifdef DDB
	leave
#endif
	ret

/*****************************************************************************/

/*
 * The following is i386-specific nonsense.
 */

ENTRY(setjmp)
	movl	4(%esp),%eax
	movl	%ebx,(%eax)		# save ebx
	movl	%esp,4(%eax)		# save esp
	movl	%ebp,8(%eax)		# save ebp
	movl	%esi,12(%eax)		# save esi
	movl	%edi,16(%eax)		# save edi
	movl	(%esp),%edx		# get rta
	movl	%edx,20(%eax)		# save eip
	xorl	%eax,%eax		# return (0);
	ret

ENTRY(longjmp)
	movl	4(%esp),%eax
	movl	(%eax),%ebx		# restore ebx
	movl	4(%eax),%esp		# restore esp
	movl	8(%eax),%ebp		# restore ebp
	movl	12(%eax),%esi		# restore esi
	movl	16(%eax),%edi		# restore edi
	movl	20(%eax),%edx		# get rta
	movl	%edx,(%esp)		# put in return frame
	xorl	%eax,%eax		# return (1);
	incl	%eax
	ret

/*****************************************************************************/

/*
 * The following primitives manipulate the run queues.
 * whichqs tells which of the 32 queues qs have processes in them.
 * Setrq puts processes into queues, Remrq removes them from queues.
 * The running process is on no queue, other processes are on a queue
 * related to p->p_pri, divided by 4 actually to shrink the 0-127 range
 * of priorities into the 32 available queues.
 */
	.globl	_C_LABEL(whichqs),_C_LABEL(qs),_C_LABEL(uvmexp),_C_LABEL(panic)
/*
 * setrunqueue(struct proc *p);
 * Insert a process on the appropriate queue.  Should be called at splclock().
 */
NENTRY(setrunqueue)
	movl	4(%esp),%eax
#ifdef DIAGNOSTIC
	cmpl	$0,P_BACK(%eax)	# should not be on q already
	jne	1f
	cmpl	$0,P_WCHAN(%eax)
	jne	1f
	cmpb	$SRUN,P_STAT(%eax)
	jne	1f
#endif /* DIAGNOSTIC */
	movzbl	P_PRIORITY(%eax),%edx
	shrl	$2,%edx
	btsl	%edx,_C_LABEL(whichqs)		# set q full bit
	leal	_C_LABEL(qs)(,%edx,8),%edx	# locate q hdr
	movl	P_BACK(%edx),%ecx
	movl	%edx,P_FORW(%eax)	# link process on tail of q
	movl	%eax,P_BACK(%edx)
	movl	%eax,P_FORW(%ecx)
	movl	%ecx,P_BACK(%eax)
	ret
#ifdef DIAGNOSTIC
1:	pushl	$2f
	call	_C_LABEL(panic)
	/* NOTREACHED */
2:	.asciz	"setrunqueue"
#endif /* DIAGNOSTIC */

/*
 * remrunqueue(struct proc *p);
 * Remove a process from its queue.  Should be called at splclock().
 */
NENTRY(remrunqueue)
	movl	4(%esp),%ecx
	movzbl	P_PRIORITY(%ecx),%eax
#ifdef DIAGNOSTIC
	shrl	$2,%eax
	btl	%eax,_C_LABEL(whichqs)
	jnc	1f
#endif /* DIAGNOSTIC */
	movl	P_BACK(%ecx),%edx	# unlink process
	movl	$0,P_BACK(%ecx)		# zap reverse link to indicate off list
	movl	P_FORW(%ecx),%ecx
	movl	%ecx,P_FORW(%edx)
	movl	%edx,P_BACK(%ecx)
	cmpl	%ecx,%edx		# q still has something?
	jne	2f
#ifndef DIAGNOSTIC
	shrl	$2,%eax
#endif
	btrl	%eax,_C_LABEL(whichqs)		# no; clear bit
2:	ret
#ifdef DIAGNOSTIC
1:	pushl	$3f
	call	_C_LABEL(panic)
	/* NOTREACHED */
3:	.asciz	"remrunqueue"
#endif /* DIAGNOSTIC */

/*
 * When no processes are on the runq, cpu_switch() branches to here to wait for
 * something to come ready.
 */
ENTRY(idle)
	/* Skip context saving if we have none. */
	testl %esi,%esi
	jz	1f

	/*
	 * idling:	save old context.
	 *
	 * Registers:
	 *   %eax, %ebx, %ecx - scratch
	 *   %esi - old proc, then old pcb
	 *   %edi - idle pcb
	 *   %edx - idle TSS selector
	 */

	pushl	%esi
	call	_C_LABEL(pmap_deactivate)	# pmap_deactivate(oldproc)
	addl	$4,%esp

	movl	P_ADDR(%esi),%esi

	/* Save stack pointers. */
	movl	%esp,PCB_ESP(%esi)
	movl	%ebp,PCB_EBP(%esi)

	/* Find idle PCB for this CPU */
#ifndef MULTIPROCESSOR
	movl	$_C_LABEL(proc0),%ebx
	movl	P_ADDR(%ebx),%edi
	movl	P_MD_TSS_SEL(%ebx),%edx
#else
	GET_CPUINFO(%ebx)
	movl	CPU_INFO_IDLE_PCB(%ebx),%edi
	movl	CPU_INFO_IDLE_TSS_SEL(%ebx),%edx
#endif

	/* Restore the idle context (avoid interrupts) */
	CLI(%ecx)

	/* Restore stack pointers. */
	movl	PCB_ESP(%edi),%esp
	movl	PCB_EBP(%edi),%ebp

	/* Switch address space. */
	pushl	%edi
	call	_C_LABEL(i386_switch_context)
	addl	$4,%esp

	/* Record new pcb. */
	SET_CURPCB(%edi,%ecx)

	xorl	%esi,%esi
	STI(%eax)

1:
#if defined(MULTIPROCESSOR) || defined(LOCKDEBUG)	
	call	_C_LABEL(sched_unlock_idle)
#endif
	/* Interrupts are okay again */
	pushl	$IPL_NONE		# spl0()
	call	_C_LABEL(Xspllower)	# process pending interrupts
	addl	$4,%esp
	jmp	idle_start
ENTRY(idle_zero)
	STIC(%eax)
	jz	4f
	call	_C_LABEL(stipending)
	testl	%eax, %eax
	jz	4f
	pushl	$IPL_NONE
	call	_C_LABEL(Xspllower)
	addl	$4,%esp
	jmp	idle_start
4:
	call	_C_LABEL(uvm_pageidlezero)
	CLI(%eax)
	cmpl	$0,_C_LABEL(whichqs)
	jnz	idle_exit
ENTRY(idle_loop)
	/* Try to zero some pages. */
	movl	_C_LABEL(uvm)+UVM_PAGE_IDLE_ZERO,%ecx
	testl	%ecx,%ecx
	jnz	idle_zero
	call	_C_LABEL(idle_block)
	cmpl	$0,_C_LABEL(whichqs)
	jnz	idle_exit
	STIC(%eax)
	jz	1f
	call	_C_LABEL(stipending)
	testl	%eax,%eax
	jz	1f
	pushl	$IPL_NONE
	call	_C_LABEL(Xspllower)
	addl	$4,%esp
	jmp	idle_start
1:
	movl	$SCHEDOP_yield,%ebx
	call	(hypercall_page + (HYPERVISOR_sched_op * 32))
ENTRY(idle_start)
	CLI(%eax)
	cmpl	$0,_C_LABEL(whichqs)
	jz	_C_LABEL(idle_loop)

ENTRY(idle_exit)
	movl	$IPL_HIGH,CPL		# splhigh
	STI(%eax)
#if defined(MULTIPROCESSOR) || defined(LOCKDEBUG)	
	call	_C_LABEL(sched_lock_idle)
#endif
	jmp	switch_search
		
#ifdef DIAGNOSTIC
NENTRY(switch_error)
	pushl	$1f
	call	_C_LABEL(panic)
	/* NOTREACHED */
1:	.asciz	"cpu_switch"
#endif /* DIAGNOSTIC */

/*
 * cpu_switch(void);
 * Find a runnable process and switch to it.  Wait if necessary.  If the new
 * process is the same as the old one, we short-circuit the context save and
 * restore.
 */
ENTRY(cpu_switch)
	pushl	%ebx
	pushl	%esi
	pushl	%edi
	pushl	CPL

	GET_CURPROC(%esi,%ecx)

	/*
	 * Clear curproc so that we don't accumulate system time while idle.
	 * This also insures that schedcpu() will move the old process to
	 * the correct queue if it happens to get called from the spllower()
	 * below and changes the priority.  (See corresponding comment in
	 * userret()).
	 */
	CLEAR_CURPROC(%ecx)

switch_search:
	/*
	 * First phase: find new process.
	 *
	 * Registers:
	 *   %eax - queue head, scratch, then zero
	 *   %ebx - queue number
	 *   %ecx - cached value of whichqs
	 *   %edx - next process in queue
	 *   %esi - old process
	 *   %edi - new process
	 */

	/* Wait for new process. */
	movl	_C_LABEL(whichqs),%ecx
	bsfl	%ecx,%ebx		# find a full q
	jz	_C_LABEL(idle)		# if none, idle
	leal	_C_LABEL(qs)(,%ebx,8),%eax	# select q
	movl	P_FORW(%eax),%edi	# unlink from front of process q
#ifdef	DIAGNOSTIC
	cmpl	%edi,%eax		# linked to self (i.e. nothing queued)?
	je	_C_LABEL(switch_error)	# not possible
#endif /* DIAGNOSTIC */
	movl	P_FORW(%edi),%edx
	movl	%edx,P_FORW(%eax)
	movl	%eax,P_BACK(%edx)

	cmpl	%edx,%eax		# q empty?
	jne	3f

	btrl	%ebx,%ecx		# yes, clear to indicate empty
	movl	%ecx,_C_LABEL(whichqs)	# update q status

3:	/* We just did it. */
	CLEAR_RESCHED(%ecx)

#ifdef	DIAGNOSTIC
	cmpl	%eax,P_WCHAN(%edi)	# Waiting for something?
	jne	_C_LABEL(switch_error)	# Yes; shouldn't be queued.
	cmpb	$SRUN,P_STAT(%edi)	# In run state?
	jne	_C_LABEL(switch_error)	# No; shouldn't be queued.
#endif /* DIAGNOSTIC */

	/* Isolate process.  XXX Is this necessary? */
	movl	%eax,P_BACK(%edi)

	/* Record new process. */
	movb	$SONPROC,P_STAT(%edi)	# p->p_stat = SONPROC
	SET_CURPROC(%edi,%ecx)

	/* Skip context switch if same process. */
	xorl	%ebx,%ebx
	cmpl	%edi,%esi
	je	switch_return

	/* If old process exited, don't bother. */
	testl	%esi,%esi
	jz	switch_exited

	/*
	 * Second phase: save old context.
	 *
	 * Registers:
	 *   %eax, %ecx - scratch
	 *   %esi - old process, then old pcb
	 *   %edi - new process
	 */

	pushl	%esi
	call	_C_LABEL(pmap_deactivate)
	addl	$4,%esp

	movl	P_ADDR(%esi),%esi

	/* Save stack pointers. */
	movl	%esp,PCB_ESP(%esi)
	movl	%ebp,PCB_EBP(%esi)

switch_exited:
	/*
	 * Third phase: restore saved context.
	 *
	 * Registers:
	 *   %eax, %ecx, %edx - scratch
	 *   %esi - new pcb
	 *   %edi - new process
	 */

	/* No interrupts while loading new state. */
	CLI(%eax)
	movl	P_ADDR(%edi),%esi

	/* Restore stack pointers. */
	movl	PCB_ESP(%esi),%esp
	movl	PCB_EBP(%esi),%ebp

#if 0
	/* Don't bother with the rest if switching to a system process. */
	testl	$P_SYSTEM,P_FLAG(%edi)
	jnz	switch_restored
#endif

	/*
	 * Activate the address space.  We're curproc, so %cr3 will
	 * be reloaded, but we're not yet curpcb, so the LDT won't
	 * be reloaded, although the PCB copy of the selector will
	 * be refreshed from the pmap.
	 */
	pushl	%edi
	call	_C_LABEL(pmap_activate)
	addl	$4,%esp
	
	pushl	%esi
	call	_C_LABEL(i386_switch_context)
	addl	$4,%esp

	/* Record new pcb. */
	SET_CURPCB(%esi, %ecx)

	/* Interrupts are okay again. */
	STI(%eax)

switch_return:
#if 0
	pushl	%edi
	GET_CPUINFO(%ebx)
	leal	CPU_INFO_NAME(%ebx),%ebx
	pushl	%ebx
	pushl	$1f
	call	_C_LABEL(printf)
	addl	$0xc,%esp
#endif
#if defined(MULTIPROCESSOR) || defined(LOCKDEBUG)     
	call    _C_LABEL(sched_unlock_idle)
#endif
	/*
	 * Restore old cpl from stack.  Note that this is always an increase,
	 * due to the spl0() on entry.
	 */
	popl	CPL

	movl	%edi,%eax		# return (p);
	popl	%edi
	popl	%esi
	popl	%ebx
	ret
1:	.asciz	"%s: scheduled %x\n"
/*
 * switch_exit(struct proc *p);
 * Switch to the appropriate idle context (proc0's if uniprocessor; the cpu's if
 * multiprocessor) and deallocate the address space and kernel stack for p.
 * Then jump into cpu_switch(), as if we were in the idle proc all along.
 */
#ifndef MULTIPROCESSOR
	.globl	_C_LABEL(proc0)
#endif
ENTRY(switch_exit)
	movl	4(%esp),%edi		# old process
#ifndef MULTIPROCESSOR
	movl	$_C_LABEL(proc0),%ebx
	movl	P_ADDR(%ebx),%esi
	movl	P_MD_TSS_SEL(%ebx),%edx
#else
	GET_CPUINFO(%ebx)
	movl	CPU_INFO_IDLE_PCB(%ebx),%esi
	movl	CPU_INFO_IDLE_TSS_SEL(%ebx),%edx
#endif

	/* In case we fault... */
	CLEAR_CURPROC(%ecx)

	/* Restore the idle context. */
	CLI(%ecx)

	/* Restore stack pointers. */
	movl	PCB_ESP(%esi),%esp
	movl	PCB_EBP(%esi),%ebp

	pushl	%esi
	call	_C_LABEL(i386_switch_context)
	addl	$4,%esp

	/* Record new pcb. */
	SET_CURPCB(%esi, %ecx)

	/* Interrupts are okay again. */
	STI(%eax)

	/*
	 * Schedule the dead process's vmspace and stack to be freed.
	 */
	pushl	%edi			/* exit2(p) */
	call	_C_LABEL(exit2)
	addl	$4,%esp

	/* Jump into cpu_switch() with the right state. */
	xorl	%esi,%esi
	CLEAR_CURPROC(%ecx)
	jmp	switch_search

/*
 * savectx(struct pcb *pcb);
 * Update pcb, saving current processor state.
 */
ENTRY(savectx)
	movl	4(%esp),%edx		# edx = p->p_addr

	/* Save stack pointers. */
	movl	%esp,PCB_ESP(%edx)
	movl	%ebp,PCB_EBP(%edx)

	ret

/*****************************************************************************/

/*
 * Trap and fault vector routines
 *
 * On exit from the kernel to user mode, we always need to check for ASTs.  In
 * addition, we need to do this atomically; otherwise an interrupt may occur
 * which causes an AST, but it won't get processed until the next kernel entry
 * (possibly the next clock tick).  Thus, we disable interrupt before checking,
 * and only enable them again on the final `iret' or before calling the AST
 * handler.
 *
 * XXX - debugger traps are now interrupt gates so at least bdb doesn't lose
 * control.  The sti's give the standard losing behaviour for ddb and kgdb.
 */
#define	IDTVEC(name)	ALIGN_TEXT; .globl X/**/name; X/**/name:

#define	TRAP(a)		pushl $(a) ; jmp _C_LABEL(alltraps)
#define	ZTRAP(a)	pushl $0 ; TRAP(a)
/* XXX */
#define	BPTTRAP(a)	testb $(PSL_I>>8),13(%esp) ; jz 1f ; STI(%eax) ; 1: ; \
			TRAP(a)

	.text
IDTVEC(trap00)
IDTVEC(div)
	ZTRAP(T_DIVIDE)
IDTVEC(trap01)
IDTVEC(dbg)
	subl	$4,%esp
	pushl	%eax
	movl	%dr6,%eax
	movl	%eax,4(%esp)
	andb	$~0xf,%al
	movl	%eax,%dr6
	popl	%eax
	BPTTRAP(T_TRCTRAP)
IDTVEC(trap02)
IDTVEC(nmi)
	ZTRAP(T_NMI)
IDTVEC(trap03)
IDTVEC(bpt)
	pushl	$0
	BPTTRAP(T_BPTFLT)
IDTVEC(trap04)
IDTVEC(ofl)
	ZTRAP(T_OFLOW)
IDTVEC(trap05)
IDTVEC(bnd)
	ZTRAP(T_BOUND)
IDTVEC(trap06)
IDTVEC(ill)
	ZTRAP(T_PRIVINFLT)
IDTVEC(trap07)
IDTVEC(dna)
#if NNPX > 0
	pushl	$0			# dummy error code
	pushl	$T_DNA
	INTRENTRY
#ifdef MULTIPROCESSOR
	GET_CPUINFO(%eax)
	pushl	%eax
#else
	pushl	$_C_LABEL(cpu_info_primary)
#endif
	call	*_C_LABEL(npxdna_func)
	addl	$4,%esp
	testl	%eax,%eax
	jz	calltrap
	INTRFASTEXIT
#else
	ZTRAP(T_DNA)
#endif
IDTVEC(trap08)			/* XXX */
IDTVEC(dble)
	TRAP(T_DOUBLEFLT)
IDTVEC(trap09)
IDTVEC(fpusegm)
	ZTRAP(T_FPOPFLT)
IDTVEC(trap0a)
IDTVEC(tss)
	TRAP(T_TSSFLT)
IDTVEC(trap0b)
IDTVEC(missing)
	TRAP(T_SEGNPFLT)
IDTVEC(trap0c)
IDTVEC(stk)
	TRAP(T_STKFLT)
IDTVEC(trap0d)
IDTVEC(prot)
	TRAP(T_PROTFLT)
IDTVEC(trap0e)
IDTVEC(page)
	TRAP(T_PAGEFLT)
IDTVEC(trap0f)
IDTVEC(rsvd)
	ZTRAP(T_RESERVED)
IDTVEC(mchk)
	ZTRAP(T_MACHK)
IDTVEC(simd)
	ZTRAP(T_XFTRAP)
IDTVEC(intrspurious)
	/*
	 * The Pentium Pro local APIC may erroneously call this vector for a
	 * default IR7.  Just ignore it.
	 *
	 * (The local APIC does this when CPL is raised while it's on the 
	 * way to delivering an interrupt.. presumably enough has been set 
	 * up that it's inconvenient to abort delivery completely..)
	 */
	iret
IDTVEC(trap10)
IDTVEC(fpu)
#if NNPX > 0
	/*
	 * Handle like an interrupt so that we can call npxintr to clear the
	 * error.  It would be better to handle npx interrupts as traps but
	 * this is difficult for nested interrupts.
	 */
	pushl	$0			# dummy error code
	pushl	$T_ASTFLT
	INTRENTRY
	pushl	CPL			# if_ppl in intrframe
	pushl	%esp			# push address of intrframe
	incl	_C_LABEL(uvmexp)+V_TRAP
	call	_C_LABEL(npxintr)
	addl	$8,%esp			# pop address and if_ppl
	INTRFASTEXIT
#else
	ZTRAP(T_ARITHTRAP)
#endif
IDTVEC(trap11)
IDTVEC(align)
	ZTRAP(T_ALIGNFLT)
IDTVEC(trap12)
IDTVEC(trap13)
IDTVEC(trap14)
IDTVEC(trap15)
IDTVEC(trap16)
IDTVEC(trap17)
IDTVEC(trap18)
IDTVEC(trap19)
IDTVEC(trap1a)
IDTVEC(trap1b)
IDTVEC(trap1c)
IDTVEC(trap1d)
IDTVEC(trap1e)
IDTVEC(trap1f)
	/* 18 - 31 reserved for future exp */

/*
 * If an error is detected during trap, syscall, or interrupt exit, trap() will
 * change %eip to point to one of these labels.  We clean up the stack, if
 * necessary, and resume as if we were handling a general protection fault.
 * This will cause the process to get a SIGBUS.
 */
NENTRY(resume_iret)
	ZTRAP(T_PROTFLT)
NENTRY(resume_pop_ds)
	movl	%es,TF_ES(%esp)
	movl	$GSEL(GDATA_SEL, SEL_KPL),%eax
	movw	%ax,%es
NENTRY(resume_pop_es)
	movl	%gs,TF_GS(%esp)
	movl	$GSEL(GDATA_SEL, SEL_KPL),%eax
	movw	%ax,%gs
NENTRY(resume_pop_gs)
	movl	%fs,TF_FS(%esp)
	movl	$GSEL(GDATA_SEL, SEL_KPL),%eax
	movw	%ax,%fs
NENTRY(resume_pop_fs)
	movl	$T_PROTFLT,TF_TRAPNO(%esp)
	jmp	calltrap

NENTRY(alltraps)
	INTRENTRY
calltrap:
#ifdef DIAGNOSTIC
	movl	CPL,%ebx
#endif /* DIAGNOSTIC */
	pushl	%esp
	call	_C_LABEL(trap)
	addl	$4,%esp
2:	/* Check for ASTs on exit to user mode. */
	CLI(%ecx)
	CHECK_ASTPENDING(%ecx)
	je	1f
	testb	$CHK_UPL,TF_CS(%esp)
#ifdef VM86
	jnz	5f
	testl	$PSL_VM,TF_EFLAGS(%esp)
#endif
	jz	1f
5:	CLEAR_ASTPENDING(%ecx)
	STI(%ecx)
	movl	$T_ASTFLT,TF_TRAPNO(%esp)
	pushl	%esp
	call	_C_LABEL(trap)
	addl	$4,%esp
	jmp	2b
1:	STIC(%ecx)
	jz	6f
	call	_C_LABEL(stipending)
#ifndef DIAGNOSTIC
6:	INTRFASTEXIT
#else 
6:	cmpl	CPL,%ebx
	jne	3f
	INTRFASTEXIT
3:	STI(%ecx)
	pushl	$4f
	call	_C_LABEL(printf)
	addl	$4,%esp
#if defined(DDB) && 0
	int	$3
#endif /* DDB */
	movl	%ebx,CPL
	jmp	2b
4:	.asciz	"WARNING: SPL NOT LOWERED ON TRAP EXIT\n"
#endif /* DIAGNOSTIC */

/*
 * Old call gate entry for syscall
 */
IDTVEC(osyscall)
	pushl	$1f
	call	_C_LABEL(panic)
1:	.asciz	"osyscall"
	/* Set eflags in trap frame. */
	pushfl
	popl	8(%esp)
	/* Turn off trace flag and nested task. */
	pushfl
	andb	$~((PSL_T|PSL_NT)>>8),1(%esp)
	popfl
	pushl	$7		# size of instruction for restart
	jmp	syscall1
IDTVEC(osyscall_end)

/*
 * Trap gate entry for syscall
 */
IDTVEC(syscall)
	pushl	$2		# size of instruction for restart
syscall1:
	pushl	$T_ASTFLT	# trap # for doing ASTs
	INTRENTRY
	pushl	%esp
	call	_C_LABEL(syscall)
	addl	$4,%esp
2:	/* Check for ASTs on exit to user mode. */
	CLI(%ecx)
	CHECK_ASTPENDING(%ecx)
	je	1f
	/* Always returning to user mode here. */
	CLEAR_ASTPENDING(%ecx)
	STI(%ecx)
	/* Pushed T_ASTFLT into tf_trapno on entry. */
	pushl	%esp
	call	_C_LABEL(trap)
	addl	$4,%esp
	jmp	2b
1:	STI(%ecx)
	INTRFASTEXIT

/* Hypervisor and failsafe callback */
# A note on the "critical region" in our callback handler.
# We want to avoid stacking callback handlers due to events occurring
# during handling of the last event. To do this, we keep events disabled
# until weve done all processing. HOWEVER, we must enable events before
# popping the stack frame (cant be done atomically) and so it would still
# be possible to get enough handler activations to overflow the stack.
# Although unlikely, bugs of that kind are hard to track down, so we'd
# like to avoid the possibility.
# So, on entry to the handler we detect whether we interrupted an
# existing activation in its critical region -- if so, we pop the current
# activation and restart the handler using the previous one.
NENTRY(hypervisor_callback)
	pushl	$0			# dummy error code
	pushl	$T_ASTFLT
	INTRENTRY
        movl	TF_EIP(%esp),%eax
        cmpl	$scrit,%eax
        jb	11f
        cmpl	$ecrit,%eax
        jb	critical_region_fixup
11:     pushl	CPL
        push	%esp
        call	do_hypervisor_callback
        add	$8,%esp
        xorl	%eax,%eax
        movb	TF_CS(%esp),%cl
        test	$CHK_UPL,%cl		# slow return to ring 2 or 3
        je	safesti
        movl	CPL,%ebx
        jmp	doreti_checkast
safesti:
        movl	HYPERVISOR_shared_info,%esi
        XEN_UNBLOCK_EVENTS(%esi)	# reenable event callbacks
scrit:  /**** START OF CRITICAL REGION ****/
        XEN_TEST_PENDING(%esi)
        jnz	14f			# process more events if necessary...
        INTRFASTEXIT
critiret:
14:     XEN_BLOCK_EVENTS(%esi)
        jmp  11b
ecrit:  /**** END OF CRITICAL REGION ****/
# [How we do the fixup]. We want to merge the current stack frame with the
# just-interrupted frame. How we do this depends on where in the critical
# region the interrupted handler was executing, and so how many saved
# registers are in each frame. We do this quickly using the lookup table
# 'critical_fixup_table'. For each byte offset in the critical region, it
# provides the number of bytes which have already been popped from the
# interrupted stack frame.
critical_region_fixup:
        cmpl	$(critiret-1),%eax	    # eip points to iret?
	jne	1f
	movl	$(TF_PUSHSIZE+0x8),%eax
	jmp	2f
1:	xorl	%eax,%eax
2:
				# %eax contains num bytes popped
        mov  %esp,%esi
        add  %eax,%esi        # %esi points at end of src region
        mov  %esp,%edi
	add  $(TF_PUSHSIZE+0x8+0xC),%edi # %edi points at end of dst region
        mov  %eax,%ecx
        shr  $2,%ecx          # convert words to bytes
        je   16f              # skip loop if nothing to copy
15:     subl $4,%esi          # pre-decrementing copy loop
        subl $4,%edi
        movl (%esi),%eax
        movl %eax,(%edi)
        loop 15b
16:     movl %edi,%esp        # final %edi is top of merged stack
        jmp  11b


# Hypervisor uses this for application faults while it executes.
NENTRY(failsafe_callback)
	pop	%ds
	pop	%es
	pop	%fs
	pop	%gs
	call	_C_LABEL(xen_failsafe_handler)
	iret

#include <xen/i386/vector.s>
#include <xen/i386/spl.s>

/*
 * bzero (void *b, size_t len)
 *	write len zero bytes to the string b.
 */

ENTRY(bzero)
	pushl	%edi
	movl	8(%esp),%edi
	movl	12(%esp),%edx

	cld				/* set fill direction forward */
	xorl	%eax,%eax		/* set fill data to 0 */

	/*
	 * if the string is too short, it's really not worth the overhead
	 * of aligning to word boundries, etc.  So we jump to a plain
	 * unaligned set.
	 */
	cmpl	$16,%edx
	jb	7f

	movl	%edi,%ecx		/* compute misalignment */
	negl	%ecx
	andl	$3,%ecx
	subl	%ecx,%edx
	rep				/* zero until word aligned */
	stosb

#if defined(I486_CPU)
#if defined(I386_CPU) || defined(I586_CPU) || defined(I686_CPU)
	cmpl	$CPUCLASS_486,_C_LABEL(cpu_class)
	jne	8f
#endif

	movl	%edx,%ecx
	shrl	$6,%ecx
	jz	8f
	andl	$63,%edx
1:	movl	%eax,(%edi)
	movl	%eax,4(%edi)
	movl	%eax,8(%edi)
	movl	%eax,12(%edi)
	movl	%eax,16(%edi)
	movl	%eax,20(%edi)
	movl	%eax,24(%edi)
	movl	%eax,28(%edi)
	movl	%eax,32(%edi)
	movl	%eax,36(%edi)
	movl	%eax,40(%edi)
	movl	%eax,44(%edi)
	movl	%eax,48(%edi)
	movl	%eax,52(%edi)
	movl	%eax,56(%edi)
	movl	%eax,60(%edi)
	addl	$64,%edi
	decl	%ecx
	jnz	1b
#endif

8:	movl	%edx,%ecx		/* zero by words */
	shrl	$2,%ecx
	andl	$3,%edx
	rep
	stosl

7:	movl	%edx,%ecx		/* zero remainder bytes */
	rep
	stosb

	popl	%edi
	ret

#if defined(I686_CPU) && !defined(SMALL_KERNEL)
ENTRY(sse2_pagezero)
	pushl	%ebx
	movl	8(%esp),%ecx
	movl	%ecx,%eax
	addl	$4096,%eax
	xor	%ebx,%ebx
1:
	movnti	%ebx,(%ecx)
	addl	$4,%ecx
	cmpl	%ecx,%eax
	jne	1b
	sfence
	popl	%ebx
	ret

ENTRY(i686_pagezero)
	pushl	%edi
	pushl	%ebx

	movl	12(%esp), %edi
	movl	$1024, %ecx
	cld

	ALIGN_TEXT
1:
	xorl	%eax, %eax
	repe
	scasl
	jnz	2f

	popl	%ebx
	popl	%edi
	ret

	ALIGN_TEXT

2:
	incl	%ecx
	subl	$4, %edi

	movl	%ecx, %edx
	cmpl	$16, %ecx

	jge	3f

	movl	%edi, %ebx
	andl	$0x3f, %ebx
	shrl	%ebx
	shrl	%ebx
	movl	$16, %ecx
	subl	%ebx, %ecx

3:
	subl	%ecx, %edx
	rep
	stosl

	movl	%edx, %ecx
	testl	%edx, %edx
	jnz	1b

	popl	%ebx
	popl	%edi
	ret
#endif

#if NLAPIC > 0 
#include <xen/i386/apicvec.s>
#endif

#include <xen/i386/mutex.S>
