/*	$NetBSD: frameasm.h,v 1.2 2005/03/09 22:39:20 bouyer Exp $	*/
/*	NetBSD: frameasm.h,v 1.4 2004/02/20 17:35:01 yamt Exp 	*/

#ifndef _I386_FRAMEASM_H_
#define _I386_FRAMEASM_H_

/* XXX assym.h */
#define TRAP_INSTR	int $0x82
#define __HYPERVISOR_stack_switch          4
#define __HYPERVISOR_fpu_taskswitch	   7
#define __HYPERVISOR_physdev_op		   19

#define	DO_DEFERRED_SWITCH(reg) \
	cmpl	$0, CPUVAR(WANT_PMAPLOAD)		; \
	jz	1f					; \
	call	_C_LABEL(pmap_load)			; \
	1:

#define	CHECK_DEFERRED_SWITCH(reg) \
	cmpl	$0, CPUVAR(WANT_PMAPLOAD)

#define XEN_BLOCK_EVENTS(reg)	movb $1,EVTCHN_UPCALL_MASK(reg)
#define XEN_UNBLOCK_EVENTS(reg)	movb $0,EVTCHN_UPCALL_MASK(reg)
#define XEN_TEST_PENDING(reg)	testb $0xFF,EVTCHN_UPCALL_PENDING(reg)

#define CLI(reg)	movl	_C_LABEL(HYPERVISOR_shared_info),reg ;	\
			XEN_BLOCK_EVENTS(reg)
#define STI(reg)	movl	_C_LABEL(HYPERVISOR_shared_info),reg ;	\
			XEN_UNBLOCK_EVENTS(reg)
#define STIC(reg)	movl	_C_LABEL(HYPERVISOR_shared_info),reg ;	\
			XEN_UNBLOCK_EVENTS(reg)  ; \
			testb $0xff,EVTCHN_UPCALL_PENDING(reg)

#endif /* _I386_FRAMEASM_H_ */
