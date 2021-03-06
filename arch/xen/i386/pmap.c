/*	$OpenBSD: pmap.c,v 1.100 2007/04/04 17:44:45 art Exp $	*/
/*	$NetBSD: pmap.c,v 1.91 2000/06/02 17:46:37 thorpej Exp $	*/

/*
 *
 * Copyright (c) 1997 Charles D. Cranor and Washington University.
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
 *      This product includes software developed by Charles D. Cranor and
 *      Washington University.
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

/*
 * pmap.c: i386 pmap module rewrite
 * Chuck Cranor <chuck@ccrc.wustl.edu>
 * 11-Aug-97
 *
 * history of this pmap module: in addition to my own input, i used
 *    the following references for this rewrite of the i386 pmap:
 *
 * [1] the NetBSD i386 pmap.   this pmap appears to be based on the
 *     BSD hp300 pmap done by Mike Hibler at University of Utah.
 *     it was then ported to the i386 by William Jolitz of UUNET
 *     Technologies, Inc.   Then Charles M. Hannum of the NetBSD
 *     project fixed some bugs and provided some speed ups.
 *
 * [2] the FreeBSD i386 pmap.   this pmap seems to be the
 *     Hibler/Jolitz pmap, as modified for FreeBSD by John S. Dyson
 *     and David Greenman.
 *
 * [3] the Mach pmap.   this pmap, from CMU, seems to have migrated
 *     between several processors.   the VAX version was done by
 *     Avadis Tevanian, Jr., and Michael Wayne Young.    the i386
 *     version was done by Lance Berc, Mike Kupfer, Bob Baron,
 *     David Golub, and Richard Draves.    the alpha version was
 *     done by Alessandro Forin (CMU/Mach) and Chris Demetriou
 *     (NetBSD/alpha).
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/proc.h>
#include <sys/malloc.h>
#include <sys/pool.h>
#include <sys/user.h>
#include <sys/kernel.h>
#include <sys/mutex.h>

#include <uvm/uvm.h>

#include <machine/atomic.h>
#include <machine/cpu.h>
#include <machine/specialreg.h>
#include <machine/gdt.h>

#include <dev/isa/isareg.h>
#ifdef __NetBSD__
#include <machine/isa_machdep.h>
#endif
#ifdef __OpenBSD__
#include <sys/msgbuf.h>
#include <stand/boot/bootarg.h>
#endif

#include <machine/xen.h>
#include <machine/hypervisor.h>
#include <machine/xenpmap.h>

#if 0
#define XENDEBUG
#endif

#ifdef XENDEBUG
#define XENPRINTF(x) printf x
#define XENPRINTK(x) printk x
#else
#define XENPRINTF(x)
#define XENPRINTK(x)
#endif


#if defined(DEBUG) || defined(XENDEBUG)
void pmap_dump(struct pmap *, vaddr_t, vaddr_t);
#endif

/*
 * general info:
 *
 *  - for an explanation of how the i386 MMU hardware works see
 *    the comments in <machine/pte.h>.
 *
 *  - for an explanation of the general memory structure used by
 *    this pmap (including the recursive mapping), see the comments
 *    in <machine/pmap.h>.
 *
 * this file contains the code for the "pmap module."   the module's
 * job is to manage the hardware's virtual to physical address mappings.
 * note that there are two levels of mapping in the VM system:
 *
 *  [1] the upper layer of the VM system uses vm_map's and vm_map_entry's
 *      to map ranges of virtual address space to objects/files.  for
 *      example, the vm_map may say: "map VA 0x1000 to 0x22000 read-only
 *      to the file /bin/ls starting at offset zero."   note that
 *      the upper layer mapping is not concerned with how individual
 *      vm_pages are mapped.
 *
 *  [2] the lower layer of the VM system (the pmap) maintains the mappings
 *      from virtual addresses.   it is concerned with which vm_page is
 *      mapped where.   for example, when you run /bin/ls and start
 *      at page 0x1000 the fault routine may lookup the correct page
 *      of the /bin/ls file and then ask the pmap layer to establish
 *      a mapping for it.
 *
 * note that information in the lower layer of the VM system can be
 * thrown away since it can easily be reconstructed from the info
 * in the upper layer.
 *
 * data structures we use include:
 *
 *  - struct pmap: describes the address space of one thread
 *  - struct pv_entry: describes one <PMAP,VA> mapping of a PA
 *  - struct pv_head: there is one pv_head per managed page of
 *	physical memory.   the pv_head points to a list of pv_entry
 *	structures which describe all the <PMAP,VA> pairs that this
 *      page is mapped in.    this is critical for page based operations
 *      such as pmap_page_protect() [change protection on _all_ mappings
 *      of a page]
 *  - pv_page/pv_page_info: pv_entry's are allocated out of pv_page's.
 *      if we run out of pv_entry's we allocate a new pv_page and free
 *      its pv_entrys.
 */

/*
 * memory allocation
 *
 *  - there are three data structures that we must dynamically allocate:
 *
 * [A] new process' page directory page (PDP)
 *	- plan 1: done at pmap_create() we use
 *	  uvm_km_alloc(kernel_map, PAGE_SIZE)  [fka kmem_alloc] to do this
 *	  allocation.
 *
 * if we are low in free physical memory then we sleep in
 * uvm_km_alloc -- in this case this is ok since we are creating
 * a new pmap and should not be holding any locks.
 *
 * if the kernel is totally out of virtual space
 * (i.e. uvm_km_alloc returns NULL), then we panic.
 *
 * XXX: the fork code currently has no way to return an "out of
 * memory, try again" error code since uvm_fork [fka vm_fork]
 * is a void function.
 *
 * [B] new page tables pages (PTP)
 * 	call uvm_pagealloc()
 * 		=> success: zero page, add to pm_pdir
 * 		=> failure: we are out of free vm_pages, let pmap_enter()
 *		   tell UVM about it.
 *
 * note: for kernel PTPs, we start with NKPTP of them.   as we map
 * kernel memory (at uvm_map time) we check to see if we've grown
 * the kernel pmap.   if so, we call the optional function
 * pmap_growkernel() to grow the kernel PTPs in advance.
 *
 * [C] pv_entry structures
 *	- plan 1: try to allocate one off the free list
 *		=> success: done!
 *		=> failure: no more free pv_entrys on the list
 *	- plan 2: try to allocate a new pv_page to add a chunk of
 *	pv_entrys to the free list
 *		[a] obtain a free, unmapped, VA in kmem_map.  either
 *		we have one saved from a previous call, or we allocate
 *		one now using a "vm_map_lock_try" in uvm_map
 *		=> success: we have an unmapped VA, continue to [b]
 *		=> failure: unable to lock kmem_map or out of VA in it.
 *			move on to plan 3.
 *		[b] allocate a page for the VA
 *		=> success: map it in, free the pv_entry's, DONE!
 *		=> failure: no free vm_pages, etc.
 *			save VA for later call to [a], go to plan 3.
 *	If we fail, we simply let pmap_enter() tell UVM about it.
 */

/*
 * locking
 *
 * we have the following locks that we must contend with:
 *
 * "normal" locks:
 *
 *  - pmap_main_lock
 *    this lock is used to prevent deadlock and/or provide mutex
 *    access to the pmap system.   most operations lock the pmap
 *    structure first, then they lock the pv_lists (if needed).
 *    however, some operations such as pmap_page_protect lock
 *    the pv_lists and then lock pmaps.   in order to prevent a
 *    cycle, we require a mutex lock when locking the pv_lists
 *    first.   thus, the "pmap = >pv_list" lockers must gain a
 *    read-lock on pmap_main_lock before locking the pmap.   and
 *    the "pv_list => pmap" lockers must gain a write-lock on
 *    pmap_main_lock before locking.    since only one thread
 *    can write-lock a lock at a time, this provides mutex.
 *
 * "simple" locks:
 *
 * - pmap lock (per pmap, part of uvm_object)
 *   this lock protects the fields in the pmap structure including
 *   the non-kernel PDEs in the PDP, and the PTEs.  it also locks
 *   in the alternate PTE space (since that is determined by the
 *   entry in the PDP).
 *
 * - pvh_lock (per pv_head)
 *   this lock protects the pv_entry list which is chained off the
 *   pv_head structure for a specific managed PA.   it is locked
 *   when traversing the list (e.g. adding/removing mappings,
 *   syncing R/M bits, etc.)
 *
 * - pvalloc_lock
 *   this lock protects the data structures which are used to manage
 *   the free list of pv_entry structures.
 *
 * - pmaps_lock
 *   this lock protects the list of active pmaps (headed by "pmaps").
 *   we lock it when adding or removing pmaps from this list.
 *
 */

/*
 * locking data structures
 */

struct simplelock pvalloc_lock;
struct simplelock pmaps_lock;

#if defined(MULTIPROCESSOR) && 0

struct lock pmap_main_lock;

#define PMAP_MAP_TO_HEAD_LOCK() \
     spinlockmgr(&pmap_main_lock, LK_SHARED, (void *) 0)
#define PMAP_MAP_TO_HEAD_UNLOCK() \
     spinlockmgr(&pmap_main_lock, LK_RELEASE, (void *) 0)

#define PMAP_HEAD_TO_MAP_LOCK() \
     spinlockmgr(&pmap_main_lock, LK_EXCLUSIVE, (void *) 0)
#define PMAP_HEAD_TO_MAP_UNLOCK() \
     spinlockmgr(&pmap_main_lock, LK_RELEASE, (void *) 0)

#else

#define PMAP_MAP_TO_HEAD_LOCK()		/* null */
#define PMAP_MAP_TO_HEAD_UNLOCK()	/* null */

#define PMAP_HEAD_TO_MAP_LOCK()		/* null */
#define PMAP_HEAD_TO_MAP_UNLOCK()	/* null */

#endif

/*
 * TLB Shootdown:
 *
 * When a mapping is changed in a pmap, the TLB entry corresponding to
 * the virtual address must be invalidated on all processors.  In order
 * to accomplish this on systems with multiple processors, messages are
 * sent from the processor which performs the mapping change to all
 * processors on which the pmap is active.  For other processors, the
 * ASN generation numbers for that processor is invalidated, so that
 * the next time the pmap is activated on that processor, a new ASN
 * will be allocated (which implicitly invalidates all TLB entries).
 *
 * Shootdown job queue entries are allocated using a simple special-
 * purpose allocator for speed.
 */
struct pmap_tlb_shootdown_job {
	TAILQ_ENTRY(pmap_tlb_shootdown_job) pj_list;
	vaddr_t pj_va;			/* virtual address */
	pmap_t pj_pmap;			/* the pmap which maps the address */
	pt_entry_t pj_pte;		/* the PTE bits */
	struct pmap_tlb_shootdown_job *pj_nextfree;
};

struct pmap_tlb_shootdown_q {
	TAILQ_HEAD(, pmap_tlb_shootdown_job) pq_head;
	int pq_pte;			/* aggregate PTE bits */
	int pq_count;			/* number of pending requests */
	struct mutex pq_mutex;		/* mutex on queue */
	int pq_flushg;			/* pending flush global */
	int pq_flushu;			/* pending flush user */
} pmap_tlb_shootdown_q[I386_MAXPROCS];

#define	PMAP_TLB_MAXJOBS	16

void	pmap_tlb_shootdown_q_drain(struct pmap_tlb_shootdown_q *);
struct pmap_tlb_shootdown_job *pmap_tlb_shootdown_job_get(
    struct pmap_tlb_shootdown_q *);
void	pmap_tlb_shootdown_job_put(struct pmap_tlb_shootdown_q *,
    struct pmap_tlb_shootdown_job *);

struct mutex pmap_tlb_shootdown_job_mutex;
struct pmap_tlb_shootdown_job *pj_page, *pj_free;

/*
 * global data structures
 */

struct pmap kernel_pmap_store;	/* the kernel's pmap (proc0) */

/*
 * nkpde is the number of kernel PTPs allocated for the kernel at
 * boot time (NKPTP is a compile time override).   this number can
 * grow dynamically as needed (but once allocated, we never free
 * kernel PTPs).
 */

int nkpde = NKPTP;
#ifdef NKPDE
#error "obsolete NKPDE: use NKPTP"
#endif

/*
 * pmap_pg_g: if our processor supports PG_G in the PTE then we
 * set pmap_pg_g to PG_G (otherwise it is zero).
 */

int pmap_pg_g = 0;

/*
 * i386 physical memory comes in a big contig chunk with a small
 * hole toward the front of it...  the following 4 paddr_t's
 * (shared with machdep.c) describe the physical address space
 * of this machine.
 */
paddr_t avail_start;	/* PA of first available physical page */
paddr_t avail_end;	/* PA of last available physical page */

paddr_t pmap_pa_start;  /* PA of first physical page for this domain */
paddr_t pmap_pa_end;    /* PA of last physical page for this domain */

/* MA of last physical page of the machine */
paddr_t pmap_mem_end = HYPERVISOR_VIRT_START;	/* updated for domain-0 */

/*
 * other data structures
 */

static pt_entry_t protection_codes[8];     /* maps MI prot to i386 prot code */
static boolean_t pmap_initialized = FALSE; /* pmap_init done yet? */

/*
 * the following two vaddr_t's are used during system startup
 * to keep track of how much of the kernel's VM space we have used.
 * once the system is started, the management of the remaining kernel
 * VM space is turned over to the kernel_map vm_map.
 */

static vaddr_t virtual_avail;	/* VA of first free KVA */
static vaddr_t virtual_end;	/* VA of last free KVA */


/*
 * pv_page management structures: locked by pvalloc_lock
 */

TAILQ_HEAD(pv_pagelist, pv_page);
static struct pv_pagelist pv_freepages;	/* list of pv_pages with free entrys */
static struct pv_pagelist pv_unusedpgs; /* list of unused pv_pages */
static int pv_nfpvents;			/* # of free pv entries */
static struct pv_page *pv_initpage;	/* bootstrap page from kernel_map */
static vaddr_t pv_cachedva;		/* cached VA for later use */

#define PVE_LOWAT (PVE_PER_PVPAGE / 2)	/* free pv_entry low water mark */
#define PVE_HIWAT (PVE_LOWAT + (PVE_PER_PVPAGE * 2))
					/* high water mark */

/*
 * linked list of all non-kernel pmaps
 */

static struct pmap_head pmaps;
static struct pmap *pmaps_hand = NULL;	/* used by pmap_steal_ptp */

/*
 * pool that pmap structures are allocated from
 */

struct pool pmap_pmap_pool;

/*
 * MULTIPROCESSOR: special VA's/ PTE's are actually allocated inside a
 * I386_MAXPROCS*NPTECL array of PTE's, to avoid cache line thrashing
 * due to false sharing.
 */

#ifdef MULTIPROCESSOR
#define PTESLEW(pte, id) ((pte)+(id)*NPTECL)
#define VASLEW(va,id) ((va)+(id)*NPTECL*NBPG)
#else
#define PTESLEW(pte, id) (pte)
#define VASLEW(va,id) (va)
#endif

/*
 * special VAs and the PTEs that map them
 */

static pt_entry_t *csrc_pte, *cdst_pte, *zero_pte, *ptp_pte;
static caddr_t csrcp, cdstp, zerop, ptpp;
caddr_t vmmap; /* XXX: used by mem.c... it should really uvm_map_reserve it */

extern vaddr_t ldt_vaddr;
extern paddr_t ldt_paddr;

#if defined(I586_CPU)
/* stuff to fix the pentium f00f bug */
extern vaddr_t pentium_idt_vaddr;
#endif


/*
 * local prototypes
 */

struct pv_entry	*pmap_add_pvpage(struct pv_page *, boolean_t);
struct vm_page	*pmap_alloc_ptp(struct pmap *, int, boolean_t);
struct pv_entry	*pmap_alloc_pv(struct pmap *, int); /* see codes below */
#define ALLOCPV_NEED	0	/* need PV now */
#define ALLOCPV_TRY	1	/* just try to allocate, don't steal */
#define ALLOCPV_NONEED	2	/* don't need PV, just growing cache */
struct pv_entry	*pmap_alloc_pvpage(struct pmap *, int);
void		 pmap_enter_pv(struct pv_head *,
					    struct pv_entry *, struct pmap *,
					    vaddr_t, struct vm_page *);
void		 pmap_free_pv(struct pmap *, struct pv_entry *);
void		 pmap_free_pvs(struct pmap *, struct pv_entry *);
void		 pmap_free_pv_doit(struct pv_entry *);
void		 pmap_free_pvpage(void);
struct vm_page	*pmap_get_ptp(struct pmap *, int, boolean_t);
boolean_t	 pmap_is_curpmap(struct pmap *);
boolean_t	 pmap_is_active(struct pmap *, int);
pt_entry_t	*pmap_map_ptes(struct pmap *);
struct pv_entry	*pmap_remove_pv(struct pv_head *, struct pmap *,
					     vaddr_t);
void		 pmap_do_remove(struct pmap *, vaddr_t, vaddr_t, int);
boolean_t	 pmap_remove_pte(struct pmap *, struct vm_page *, pt_entry_t *,
				vaddr_t, int32_t *, int);
void		 pmap_remove_ptes(struct pmap *, struct vm_page *, vaddr_t,
				vaddr_t, vaddr_t, int32_t *, int);

#define PMAP_REMOVE_ALL		0	/* remove all mappings */
#define PMAP_REMOVE_SKIPWIRED	1	/* skip wired mappings */


struct vm_page	*pmap_steal_ptp(struct uvm_object *, vaddr_t);
vaddr_t		 pmap_tmpmap_pa(paddr_t);
pt_entry_t	*pmap_tmpmap_pvepte(struct pv_entry *);
void		 pmap_tmpunmap_pa(void);
void		 pmap_tmpunmap_pvepte(struct pv_entry *);
void		 pmap_apte_flush(struct pmap *);
boolean_t	 pmap_try_steal_pv(struct pv_head *,
						struct pv_entry *,
						struct pv_entry *);
void		 pmap_unmap_ptes(struct pmap *);
void		 pmap_exec_account(struct pmap *, vaddr_t, pt_entry_t,
		     pt_entry_t);

pt_entry_t	 pte_mtop(pt_entry_t);
pt_entry_t	 pte_get_ma(pt_entry_t *);
pt_entry_t	 pte_get(pt_entry_t *);
pt_entry_t	 pte_atomic_update_ma(pt_entry_t *, pt_entry_t *, pt_entry_t);
pt_entry_t	 pte_atomic_update(pt_entry_t *, pt_entry_t *, pt_entry_t);

void			pmap_pinit(pmap_t);
void			pmap_release(pmap_t);

void			pmap_zero_phys(paddr_t);

/*
 * p m a p   i n l i n e   h e l p e r   f u n c t i o n s
 */

/*
 * pmap_is_curpmap: is this pmap the one currently loaded [in %cr3]?
 *		of course the kernel is always loaded
 */

boolean_t
pmap_is_curpmap(struct pmap *pmap)
{
	return ((pmap == pmap_kernel()) ||
		(pmap == curpcb->pcb_pmap));
}

/*
 * pmap_is_active: is this pmap loaded into the specified processor's %cr3?
 */

boolean_t
pmap_is_active(struct pmap *pmap, int cpu_id)
{

	return (pmap == pmap_kernel() ||
	    (pmap->pm_cpus & (1U << cpu_id)) != 0);
}

/*
 * pmap_tmpmap_pa: map a page in for tmp usage
 */

vaddr_t
pmap_tmpmap_pa(paddr_t pa)
{
#ifdef MULTIPROCESSOR
	int id = cpu_number();
#endif
	pt_entry_t *ptpte = PTESLEW(ptp_pte, id);
	pt_entry_t *maptp;
	caddr_t ptpva = VASLEW(ptpp, id);
#if defined(DIAGNOSTIC)
	if (*ptpte)
		panic("pmap_tmpmap_pa: ptp_pte in use?");
#endif
	maptp = (pt_entry_t *)vtomach((vaddr_t)ptpte);
	PTE_SET(ptpte, maptp, PG_V /* | PG_RW */ | pa); /* always a new mapping */
	return((vaddr_t)ptpva);
}

/*
 * pmap_tmpunmap_pa: unmap a tmp use page (undoes pmap_tmpmap_pa)
 */

void
pmap_tmpunmap_pa(void)
{
#ifdef MULTIPROCESSOR
	int id = cpu_number();
#endif
	pt_entry_t *ptpte = PTESLEW(ptp_pte, id);
	pt_entry_t *maptp;
	caddr_t ptpva = VASLEW(ptpp, id);
#if defined(DIAGNOSTIC)
	if (!pmap_valid_entry(*ptpte))
		panic("pmap_tmpunmap_pa: our pte invalid?");
#endif
	maptp = (pt_entry_t *)vtomach((vaddr_t)ptpte);
	PTE_CLEAR(ptpte, maptp);	/* zap! */
	pmap_update_pg((vaddr_t)ptpva);
#ifdef MULTIPROCESSOR
	/*
	 * No need for tlb shootdown here, since ptp_pte is per-CPU.
	 */
#endif
}

/*
 * pmap_tmpmap_pvepte: get a quick mapping of a PTE for a pv_entry
 *
 * => do NOT use this on kernel mappings [why?  because pv_ptp may be NULL]
 */

pt_entry_t *
pmap_tmpmap_pvepte(struct pv_entry *pve)
{
#ifdef DIAGNOSTIC
	if (pve->pv_pmap == pmap_kernel())
		panic("pmap_tmpmap_pvepte: attempt to map kernel");
#endif

	/* is it current pmap?  use direct mapping... */
	if (pmap_is_curpmap(pve->pv_pmap))
		return(vtopte(pve->pv_va));

	return(((pt_entry_t *)pmap_tmpmap_pa(VM_PAGE_TO_PHYS(pve->pv_ptp)))
	       + ptei((unsigned)pve->pv_va));
}

/*
 * pmap_tmpunmap_pvepte: release a mapping obtained with pmap_tmpmap_pvepte
 */

void
pmap_tmpunmap_pvepte(struct pv_entry *pve)
{
	/* was it current pmap?   if so, return */
	if (pmap_is_curpmap(pve->pv_pmap))
		return;

	pmap_tmpunmap_pa();
}

void
pmap_apte_flush(struct pmap *pmap)
{
#if defined(MULTIPROCESSOR)
	struct pmap_tlb_shootdown_q *pq;
	struct cpu_info *ci, *self = curcpu();
	CPU_INFO_ITERATOR cii;
	int s;
#endif

	tlbflush();		/* flush TLB on current processor */
#if defined(MULTIPROCESSOR)
	/*
	 * Flush the APTE mapping from all other CPUs that
	 * are using the pmap we are using (who's APTE space
	 * is the one we've just modified).
	 *
	 * XXXthorpej -- find a way to defer the IPI.
	 */
	CPU_INFO_FOREACH(cii, ci) {
		if (ci == self)
			continue;
		if (pmap_is_active(pmap, ci->ci_cpuid)) {
			pq = &pmap_tlb_shootdown_q[ci->ci_cpuid];
			s = splipi();
			mtx_enter(&pq->pq_mutex);
			pq->pq_flushu++;
			mtx_leave(&pq->pq_mutex);
			splx(s);
			i386_send_ipi(ci, I386_IPI_TLB);
		}
	}
#endif
}

/*
 * pmap_map_ptes: map a pmap's PTEs into KVM and lock them in
 *
 * => we lock enough pmaps to keep things locked in
 * => must be undone with pmap_unmap_ptes before returning
 */

pt_entry_t *
pmap_map_ptes(struct pmap *pmap)
{
	pd_entry_t opde, *mapdp;
	struct pmap *ourpmap;

	/* the kernel's pmap is always accessible */
	if (pmap == pmap_kernel()) {
		return(PTE_BASE);
	}

	/* if curpmap then we are always mapped */
	if (pmap_is_curpmap(pmap)) {
		simple_lock(&pmap->pm_obj.vmobjlock);
		return(PTE_BASE);
	}

	ourpmap = curpcb->pcb_pmap;

	/* need to lock both curpmap and pmap: use ordered locking */
	if ((unsigned) pmap < (unsigned) ourpmap) {
		simple_lock(&pmap->pm_obj.vmobjlock);
		simple_lock(&ourpmap->pm_obj.vmobjlock);
	} else {
		simple_lock(&ourpmap->pm_obj.vmobjlock);
		simple_lock(&pmap->pm_obj.vmobjlock);
	}

	/* need to load a new alternate pt space into curpmap? */
	opde = PDE_GET(APDP_PDE);
	if (!pmap_valid_entry(opde) || (opde & PG_FRAME) != pmap->pm_pdirpa) {
		XENPRINTF(("pmap_map_ptes: %p %p/%p set %p/%p\n",
		    pmap,
		    (void *)vtophys((vaddr_t)APDP_PDE),
		    (void *)xpmap_ptom(vtophys((vaddr_t)APDP_PDE)),
		    (void *)pmap->pm_pdirpa,
		    (void *)xpmap_ptom(pmap->pm_pdirpa)));
		mapdp = (pt_entry_t *)vtomach((vaddr_t)APDP_PDE);
		PDE_SET(APDP_PDE, mapdp, pmap->pm_pdirpa | PG_V);
		if (pmap_valid_entry(opde))
			pmap_apte_flush(ourpmap);
		XENPRINTF(("pmap_map_ptes: APDP_PDE set done\n"));
	}
	return(APTE_BASE);
}

/*
 * pmap_unmap_ptes: unlock the PTE mapping of "pmap"
 */

void
pmap_unmap_ptes(struct pmap *pmap)
{
	if (pmap == pmap_kernel()) {
		return;
	}
	if (pmap_is_curpmap(pmap)) {
		simple_unlock(&pmap->pm_obj.vmobjlock);
	} else {
#if defined(MULTIPROCESSOR)
		pd_entry_t *mapdp;

		mapdp = (pt_entry_t *)vtomach((vaddr_t)APDP_PDE);
		PDE_CLEAR(APDP_PDE, mapdp);
		pmap_apte_flush(curpcb->pcb_pmap);
#endif
		XENPRINTF(("pmap_unmap_ptes: clear %p/%p set %p/%p\n",
		    (void *)vtophys((vaddr_t)APDP_PDE),
		    (void *)xpmap_ptom(vtophys((vaddr_t)APDP_PDE)),
		    (void *)pmap->pm_pdirpa,
		    (void *)xpmap_ptom(pmap->pm_pdirpa)));
		simple_unlock(&pmap->pm_obj.vmobjlock);
		simple_unlock(&curpcb->pcb_pmap->pm_obj.vmobjlock);
	}
}

void
pmap_exec_account(struct pmap *pm, vaddr_t va,
    pt_entry_t opte, pt_entry_t npte)
{
	if (curproc == NULL || curproc->p_vmspace == NULL ||
	    pm != vm_map_pmap(&curproc->p_vmspace->vm_map))
		return;

	if ((opte ^ npte) & PG_X) {
#ifdef MULTIPROCESSOR
		int32_t cpumask = 0;

		pmap_tlb_shootdown(pm, va, opte, &cpumask);
		pmap_tlb_shootnow(cpumask);
#else
		/* Don't bother deferring in the single CPU case. */
		pmap_update_pg(va);
#endif
	}

	/*
	 * Executability was removed on the last executable change.
	 * Reset the code segment to something conservative and
	 * let the trap handler deal with setting the right limit.
	 * We can't do that because of locking constraints on the vm map.
	 *
	 * XXX - floating cs - set this _really_ low.
	 */
	if ((opte & PG_X) && (npte & PG_X) == 0 && va == pm->pm_hiexec) {
		struct trapframe *tf = curproc->p_md.md_regs;
		struct pcb *pcb = &curproc->p_addr->u_pcb;

		pcb->pcb_cs = tf->tf_cs = GSEL(GUCODE_SEL, SEL_UPL);
		pm->pm_hiexec = I386_MAX_EXE_ADDR;
	}
}

pt_entry_t
pte_mtop(pt_entry_t pte)
{
	pt_entry_t ppte;

	KDASSERT(pmap_valid_entry(pte));
	ppte = xpmap_mtop(pte);
	if ((ppte & PG_FRAME) == XPMAP_OFFSET) {
		XENPRINTK(("pte_mtop: null page %08x -> %08x\n", ppte, pte));
		ppte = pte;
	}

	return ppte;
}

pt_entry_t
pte_get_ma(pt_entry_t *pte)
{
	return *pte;
}

pt_entry_t
pte_get(pt_entry_t *pte)
{
	if (pmap_valid_entry(*pte))
		return pte_mtop(*pte);
	return *pte;
}

pt_entry_t
pte_atomic_update_ma(pt_entry_t *pte, pt_entry_t *mapte, pt_entry_t npte)
{
	pt_entry_t opte;
	int s = splvm();

	XENPRINTK(("pte_atomic_update_ma pte %p mapte %p npte %08x\n", pte,
	    mapte, npte));

	opte = PTE_GET_MA(pte);
	if (opte > pmap_mem_end) {
		/* must remove opte unchecked */
		if (npte > pmap_mem_end)
			/* must set npte unchecked */
			xpq_queue_unchecked_pte_update(mapte, npte);
		else {
			/* must set npte checked */
			xpq_queue_unchecked_pte_update(mapte, 0);
			xpq_queue_pte_update(mapte, npte);
		}
	} else {
		/* must remove opte checked */
		if (npte > pmap_mem_end) {
			/* must set npte unchecked */
			xpq_queue_pte_update(mapte, 0);
			xpq_queue_unchecked_pte_update(mapte, npte);
		} else
			/* must set npte checked */
			xpq_queue_pte_update(mapte, npte);
	}
	xpq_flush_queue();
	splx(s);

	return opte;
}

static inline int
pte_atomic_update_ma_domid(pt_entry_t *pte, pt_entry_t npte, pt_entry_t *opte,
    int domid)
{
	pt_entry_t *maptp = (pt_entry_t *)vtomach((vaddr_t)pte);
	int error;

	if (domid == DOMID_SELF) {
		*opte = pte_atomic_update_ma(pte, maptp, npte);
		error = 0;
	} else {
		/* XXX */
		*opte = PTE_GET_MA(pte);
		error = xpq_update_foreign(maptp, npte, domid);
	}

	return error;
}

pt_entry_t
pte_atomic_update(pt_entry_t *pte, pt_entry_t *mapte, pt_entry_t npte)
{
	pt_entry_t opte;

	opte = pte_atomic_update_ma(pte, mapte, npte);

	return pte_mtop(opte);
}

/*
 * Fixup the code segment to cover all potential executable mappings.
 * returns 0 if no changes to the code segment were made.
 */
int
pmap_exec_fixup(struct vm_map *map, struct trapframe *tf, struct pcb *pcb)
{
	struct vm_map_entry *ent;
	struct pmap *pm = vm_map_pmap(map);
	vaddr_t va = 0;

	vm_map_lock_read(map);
	for (ent = (&map->header)->next; ent != &map->header; ent = ent->next) {
		/*
		 * This entry has greater va than the entries before.
		 * We need to make it point to the last page, not past it.
		 */
		if (ent->protection & VM_PROT_EXECUTE)
			va = trunc_page(ent->end) - PAGE_SIZE;
	}
	vm_map_unlock_read(map);

	if (va == pm->pm_hiexec && tf->tf_cs == GSEL(GUCODE1_SEL, SEL_UPL))
		return (0);

	pm->pm_hiexec = va;

	if (pm->pm_hiexec > (vaddr_t)I386_MAX_EXE_ADDR) {
		pcb->pcb_cs = tf->tf_cs = GSEL(GUCODE1_SEL, SEL_UPL);
	} else {
		pcb->pcb_cs = tf->tf_cs = GSEL(GUCODE_SEL, SEL_UPL);
		return (0);
	}

	return (1);
}

/*
 * p m a p   k e n t e r   f u n c t i o n s
 *
 * functions to quickly enter/remove pages from the kernel address
 * space.   pmap_kremove is exported to MI kernel.  we make use of
 * the recursive PTE mappings.
 */

/*
 * pmap_kenter_pa: enter a kernel mapping without R/M (pv_entry) tracking
 *
 * => no need to lock anything, assume va is already allocated
 * => should be faster than normal pmap enter function
 */

void
pmap_kenter_pa(vaddr_t va, paddr_t pa, vm_prot_t prot)
{
	paddr_t ma;

	if (__predict_false(pa < pmap_pa_start || pmap_pa_end <= pa)) {
		ma = pa; /* hack */
	} else {
		ma = xpmap_ptom(pa);
	}

	pmap_kenter_ma(va, ma, prot);
}

/*
 * pmap_kenter_ma: enter a kernel mapping without R/M (pv_entry) tracking
 *
 * => no need to lock anything, assume va is already allocated
 * => should be faster than normal pmap enter function
 */

void
pmap_kenter_ma(vaddr_t va, paddr_t ma, vm_prot_t prot)
{
	pt_entry_t *maptp, *pte, opte, npte;

	KASSERT(va >= VM_MIN_KERNEL_ADDRESS);
	pte = vtopte(va);
	npte = ma | ((prot & VM_PROT_WRITE) ? PG_RW : PG_RO) | PG_V | pmap_pg_g;

	maptp = (pt_entry_t *)vtomach((vaddr_t)pte);
	opte = pte_atomic_update_ma(pte, maptp, npte);	/* Zap! */
	XENPRINTK(("pmap_kenter_ma(%p, %p) %p, was %08lx now %08lx\n",
	    (void *)va, (void *)ma, pte, opte, npte));

	if ((opte & (PG_V | PG_U)) == (PG_V | PG_U)) {
		int32_t cpumask = 0;

		pmap_tlb_shootdown(pmap_kernel(), va, opte, &cpumask);
		pmap_tlb_shootnow(cpumask);
	}
}

/*
 * pmap_kremove: remove a kernel mapping(s) without R/M (pv_entry) tracking
 *
 * => no need to lock anything
 * => caller must dispose of any vm_page mapped in the va range
 * => note: not an inline function
 * => we assume the va is page aligned and the len is a multiple of PAGE_SIZE
 */

void
pmap_kremove(vaddr_t va, vsize_t len)
{
	pt_entry_t *pte, opte, *maptp;
	int32_t cpumask = 0;

	XENPRINTF(("pmap_kremove va %p, len %08lx\n", (void *)va, len));
	len >>= PAGE_SHIFT;
	for ( /* null */ ; len ; len--, va += PAGE_SIZE) {
		if (va < VM_MIN_KERNEL_ADDRESS)
			pte = vtopte(va);
		else
			pte = kvtopte(va);
		maptp = (pt_entry_t *)vtomach((vaddr_t)pte);
		opte = pte_atomic_update_ma(pte, maptp, 0);
		XENPRINTK(("pmap_kremove pte %p, was %08x\n", pte, opte));
#ifdef DIAGNOSTIC
		if (opte & PG_PVLIST)
			panic("pmap_kremove: PG_PVLIST mapping for 0x%lx", va);
#endif
		if ((opte & (PG_V | PG_U)) == (PG_V | PG_U))
			pmap_tlb_shootdown(pmap_kernel(), va, opte, &cpumask);
	}
	pmap_tlb_shootnow(cpumask);
}

/*
 * p m a p   i n i t   f u n c t i o n s
 *
 * pmap_bootstrap and pmap_init are called during system startup
 * to init the pmap module.   pmap_bootstrap() does a low level
 * init just to get things rolling.   pmap_init() finishes the job.
 */

/*
 * pmap_bootstrap: get the system in a state where it can run with VM
 *	properly enabled (called before main()).   the VM system is
 *      fully init'd later...
 *
 * => on i386, locore.s has already enabled the MMU by allocating
 *	a PDP for the kernel, and nkpde PTP's for the kernel.
 * => kva_start is the first free virtual address in kernel space
 */

void
pmap_bootstrap(vaddr_t kva_start)
{
	struct pmap *kpm;
	vaddr_t kva;
	pt_entry_t *pte, *maptp;
	int i;

	/*
	 * set the page size (default value is 4K which is ok)
	 */

	uvm_setpagesize();

	/*
	 * a quick sanity check
	 */

	if (PAGE_SIZE != NBPG)
		panic("pmap_bootstrap: PAGE_SIZE != NBPG");

	/*
	 * use the very last page of physical memory for the message buffer
	 */

	avail_end -= i386_round_page(MSGBUFSIZE);
	/*
	 * The arguments passed in from /boot needs space too.
	 */
	avail_end -= i386_round_page(bootargc);

	/*
	 * set up our local static global vars that keep track of the
	 * usage of KVM before kernel_map is set up
	 */

	virtual_avail = kva_start;		/* first free KVA */
	virtual_end = VM_MAX_KERNEL_ADDRESS;	/* last KVA */

	if (xen_start_info.flags & SIF_PRIVILEGED)
		pmap_mem_end = find_pmap_mem_end(kva_start);

	XENPRINTK(("pmap_bootstrap:  kva_start 0x%08x virtual_avail 0x%08x "
	    "virtual_end 0x%08x pmap_mem_end 0x%08x\n", kva_start,
	    virtual_avail, virtual_end, pmap_mem_end));

	/*
	 * set up protection_codes: we need to be able to convert from
	 * a MI protection code (some combo of VM_PROT...) to something
	 * we can jam into a i386 PTE.
	 */

	protection_codes[UVM_PROT_NONE] = 0;  			/* --- */
	protection_codes[UVM_PROT_EXEC] = PG_X;			/* --x */
	protection_codes[UVM_PROT_READ] = PG_RO;		/* -r- */
	protection_codes[UVM_PROT_RX] = PG_X;			/* -rx */
	protection_codes[UVM_PROT_WRITE] = PG_RW;		/* w-- */
	protection_codes[UVM_PROT_WX] = PG_RW|PG_X;		/* w-x */
	protection_codes[UVM_PROT_RW] = PG_RW;			/* wr- */
	protection_codes[UVM_PROT_RWX] = PG_RW|PG_X;		/* wrx */

	/*
	 * now we init the kernel's pmap
	 *
	 * the kernel pmap's pm_obj is not used for much.   however, in
	 * user pmaps the pm_obj contains the list of active PTPs.
	 * the pm_obj currently does not have a pager.   it might be possible
	 * to add a pager that would allow a process to read-only mmap its
	 * own page tables (fast user level vtophys?).   this may or may not
	 * be useful.
	 */

	kpm = pmap_kernel();
	simple_lock_init(&kpm->pm_obj.vmobjlock);
	kpm->pm_obj.pgops = NULL;
	TAILQ_INIT(&kpm->pm_obj.memq);
	kpm->pm_obj.uo_npages = 0;
	kpm->pm_obj.uo_refs = 1;
	bzero(&kpm->pm_list, sizeof(kpm->pm_list));  /* pm_list not used */
	kpm->pm_pdir = (pd_entry_t *)(proc0.p_addr->u_pcb.pcb_cr3 + KERNBASE);
	XENPRINTK(("pmap_bootstrap: pm_pdipa %p PTDpaddr %p\n",
	    (void *)proc0.p_addr->u_pcb.pcb_cr3, (void *)PTDpaddr));
	kpm->pm_pdirpa = (u_int32_t) proc0.p_addr->u_pcb.pcb_cr3;
	kpm->pm_stats.wired_count = kpm->pm_stats.resident_count =
		i386_btop(kva_start - VM_MIN_KERNEL_ADDRESS);

	/*
	 * the above is just a rough estimate and not critical to the proper
	 * operation of the system.
	 */

	/*
	 * enable global TLB entries if they are supported
	 */

	if (cpu_feature & CPUID_PGE) {
		lcr4(rcr4() | CR4_PGE);	/* enable hardware (via %cr4) */
		pmap_pg_g = PG_G;		/* enable software */

		/* add PG_G attribute to already mapped kernel pages */
		for (kva = VM_MIN_KERNEL_ADDRESS ; kva < virtual_avail ;
		     kva += PAGE_SIZE)
			if (pmap_valid_entry(PTE_BASE[i386_btop(kva)])) {
				maptp = (pt_entry_t *)vtomach((vaddr_t)
				    &PTE_BASE[i386_btop(kva)]);
				PTE_SETBITS(&PTE_BASE[i386_btop(kva)], maptp,
				    PG_G);
			}
	}

	/*
	 * now we allocate the "special" VAs which are used for tmp mappings
	 * by the pmap (and other modules).    we allocate the VAs by advancing
	 * virtual_avail (note that there are no pages mapped at these VAs).
	 * we find the PTE that maps the allocated VA via the linear PTE
	 * mapping.
	 */

	pte = PTE_BASE + i386_btop(virtual_avail);

#ifdef MULTIPROCESSOR
	/*
	 * Waste some VA space to avoid false sharing of cache lines
	 * for page table pages: Give each possible CPU a cache line
	 * of PTE's (8) to play with, though we only need 4.  We could
	 * recycle some of this waste by putting the idle stacks here
	 * as well; we could waste less space if we knew the largest
	 * CPU ID beforehand.
	 */
	csrcp = (caddr_t) virtual_avail;  csrc_pte = pte;

	cdstp = (caddr_t) virtual_avail+PAGE_SIZE;  cdst_pte = pte+1;

	zerop = (caddr_t) virtual_avail+PAGE_SIZE*2;  zero_pte = pte+2;

	ptpp = (caddr_t) virtual_avail+PAGE_SIZE*3;  ptp_pte = pte+3;

	virtual_avail += PAGE_SIZE * I386_MAXPROCS * NPTECL;
	pte += I386_MAXPROCS * NPTECL;
#else
	csrcp = (caddr_t) virtual_avail;  csrc_pte = pte;	/* allocate */
	virtual_avail += PAGE_SIZE; pte++;			/* advance */

	cdstp = (caddr_t) virtual_avail;  cdst_pte = pte;
	virtual_avail += PAGE_SIZE; pte++;

	zerop = (caddr_t) virtual_avail;  zero_pte = pte;
	virtual_avail += PAGE_SIZE; pte++;

	ptpp = (caddr_t) virtual_avail;  ptp_pte = pte;
	virtual_avail += PAGE_SIZE; pte++;
#endif

	XENPRINTK(("pmap_bootstrap csrcp %p cdstp %p zerop %p ptpp %p\n",
	    csrc_pte, cdst_pte, zero_pte, ptp_pte));

	/*
	 * Nothing after this point actually needs pte;
	 */
	pte = (void *)0xdeadbeef;

	/* XXX: vmmap used by mem.c... should be uvm_map_reserve */
	vmmap = (char *)virtual_avail;			/* don't need pte */
	virtual_avail += PAGE_SIZE;

	msgbufp = (struct msgbuf *)virtual_avail;	/* don't need pte */
	virtual_avail += round_page(MSGBUFSIZE);

	bootargp = (bootarg_t *)virtual_avail;		/* don't need pte */
	virtual_avail += round_page(bootargc);

	ldt_vaddr = virtual_avail;			/* don't need pte */
	virtual_avail += PAGE_SIZE;
	ldt_paddr = avail_start;			/* steal a page */
	avail_start += PAGE_SIZE;

	/*
	 * now we reserve some VM for mapping pages when doing a crash dump
	 */

	virtual_avail = reserve_dumppages(virtual_avail);

	/*
	 * init the static-global locks and global lists.
	 */

#if defined(MULTIPROCESSOR) && 0
	spinlockinit(&pmap_main_lock, "pmaplk", 0);
#endif
	simple_lock_init(&pvalloc_lock);
	simple_lock_init(&pmaps_lock);
	LIST_INIT(&pmaps);
	TAILQ_INIT(&pv_freepages);
	TAILQ_INIT(&pv_unusedpgs);

	/*
	 * initialize the pmap pool.
	 */

	pool_init(&pmap_pmap_pool, sizeof(struct pmap), 0, 0, 0, "pmappl",
	    &pool_allocator_nointr);

	/*
	 * Initialize the TLB shootdown queues.
	 */

	mtx_init(&pmap_tlb_shootdown_job_mutex, IPL_NONE);

	for (i = 0; i < I386_MAXPROCS; i++) {
		TAILQ_INIT(&pmap_tlb_shootdown_q[i].pq_head);
		mtx_init(&pmap_tlb_shootdown_q[i].pq_mutex, IPL_IPI);
	}

	/*
	 * ensure the TLB is sync'd with reality by flushing it...
	 */

	tlbflush();

	XENPRINTK(("pmap_bootstrap:  kva_start 0x%08x virtual_avail 0x%08x "
	    "virtual_end 0x%08x\n", kva_start, virtual_avail, virtual_end));
}

/*
 * pmap_init: called from uvm_init, our job is to get the pmap
 * system ready to manage mappings... this mainly means initing
 * the pv_entry stuff.
 */

void
pmap_init(void)
{
	int npages, lcv, i;
	vaddr_t addr;
	vsize_t s;

	/*
	 * compute the number of pages we have and then allocate RAM
	 * for each pages' pv_head and saved attributes.
	 */

	npages = 0;
	for (lcv = 0 ; lcv < vm_nphysseg ; lcv++)
		npages += (vm_physmem[lcv].end - vm_physmem[lcv].start);
	s = (vsize_t) (sizeof(struct pv_head) * npages +
		       sizeof(char) * npages);
	s = round_page(s); /* round up */
	addr = (vaddr_t) uvm_km_zalloc(kernel_map, s);
	if (addr == 0)
		panic("pmap_init: unable to allocate pv_heads");

	/*
	 * init all pv_head's and attrs in one bzero
	 */

	/* allocate pv_head stuff first */
	for (lcv = 0 ; lcv < vm_nphysseg ; lcv++) {
		vm_physmem[lcv].pmseg.pvhead = (struct pv_head *) addr;
		addr = (vaddr_t)(vm_physmem[lcv].pmseg.pvhead +
				 (vm_physmem[lcv].end - vm_physmem[lcv].start));
		for (i = 0;
		     i < (vm_physmem[lcv].end - vm_physmem[lcv].start); i++) {
			simple_lock_init(
			    &vm_physmem[lcv].pmseg.pvhead[i].pvh_lock);
		}
	}

	/* now allocate attrs */
	for (lcv = 0 ; lcv < vm_nphysseg ; lcv++) {
		vm_physmem[lcv].pmseg.attrs = (char *) addr;
		addr = (vaddr_t)(vm_physmem[lcv].pmseg.attrs +
				 (vm_physmem[lcv].end - vm_physmem[lcv].start));
	}

	/*
	 * now we need to free enough pv_entry structures to allow us to get
	 * the kmem_map allocated and inited (done after this function is
	 * finished).  to do this we allocate one bootstrap page out of
	 * kernel_map and use it to provide an initial pool of pv_entry
	 * structures.   we never free this page.
	 */

	pv_initpage = (struct pv_page *) uvm_km_alloc(kernel_map, PAGE_SIZE);
	if (pv_initpage == NULL)
		panic("pmap_init: pv_initpage");
	pv_cachedva = 0;   /* a VA we have allocated but not used yet */
	pv_nfpvents = 0;
	(void) pmap_add_pvpage(pv_initpage, FALSE);

	pj_page = (void *)uvm_km_alloc(kernel_map, PAGE_SIZE);
	if (pj_page == NULL)
		panic("pmap_init: pj_page");

	for (i = 0; i < PAGE_SIZE / sizeof *pj_page - 1; i++)
		pj_page[i].pj_nextfree = &pj_page[i + 1];
	pj_page[i].pj_nextfree = NULL;
	pj_free = &pj_page[0];

	/*
	 * done: pmap module is up (and ready for business)
	 */

	pmap_initialized = TRUE;
}

/*
 * p v _ e n t r y   f u n c t i o n s
 */

/*
 * pv_entry allocation functions:
 *   the main pv_entry allocation functions are:
 *     pmap_alloc_pv: allocate a pv_entry structure
 *     pmap_free_pv: free one pv_entry
 *     pmap_free_pvs: free a list of pv_entrys
 *
 * the rest are helper functions
 */

/*
 * pmap_alloc_pv: inline function to allocate a pv_entry structure
 * => we lock pvalloc_lock
 * => if we fail, we call out to pmap_alloc_pvpage
 * => 3 modes:
 *    ALLOCPV_NEED   = we really need a pv_entry, even if we have to steal it
 *    ALLOCPV_TRY    = we want a pv_entry, but not enough to steal
 *    ALLOCPV_NONEED = we are trying to grow our free list, don't really need
 *			one now
 *
 * "try" is for optional functions like pmap_copy().
 */

struct pv_entry *
pmap_alloc_pv(struct pmap *pmap, int mode)
{
	struct pv_page *pvpage;
	struct pv_entry *pv;

	simple_lock(&pvalloc_lock);

	pvpage = TAILQ_FIRST(&pv_freepages);
	if (pvpage != NULL) {
		pvpage->pvinfo.pvpi_nfree--;
		if (pvpage->pvinfo.pvpi_nfree == 0) {
			/* nothing left in this one? */
			TAILQ_REMOVE(&pv_freepages, pvpage, pvinfo.pvpi_list);
		}
		pv = pvpage->pvinfo.pvpi_pvfree;
#ifdef DIAGNOSTIC
		if (pv == NULL)
			panic("pmap_alloc_pv: pvpi_nfree off");
#endif
		pvpage->pvinfo.pvpi_pvfree = pv->pv_next;
		pv_nfpvents--;  /* took one from pool */
	} else {
		pv = NULL;		/* need more of them */
	}

	/*
	 * if below low water mark or we didn't get a pv_entry we try and
	 * create more pv_entrys ...
	 */

	if (pv_nfpvents < PVE_LOWAT || pv == NULL) {
		if (pv == NULL)
			pv = pmap_alloc_pvpage(pmap, (mode == ALLOCPV_TRY) ?
					       mode : ALLOCPV_NEED);
		else
			(void) pmap_alloc_pvpage(pmap, ALLOCPV_NONEED);
	}

	simple_unlock(&pvalloc_lock);
	return(pv);
}

/*
 * pmap_alloc_pvpage: maybe allocate a new pvpage
 *
 * if need_entry is false: try and allocate a new pv_page
 * if need_entry is true: try and allocate a new pv_page and return a
 *	new pv_entry from it.   if we are unable to allocate a pv_page
 *	we make a last ditch effort to steal a pv_page from some other
 *	mapping.    if that fails, we panic...
 *
 * => we assume that the caller holds pvalloc_lock
 */

struct pv_entry *
pmap_alloc_pvpage(struct pmap *pmap, int mode)
{
	struct vm_page *pg;
	struct pv_page *pvpage;
	int lcv, idx, npg, s;
	struct pv_entry *pv, *cpv, *prevpv;

	/*
	 * if we need_entry and we've got unused pv_pages, allocate from there
	 */

	pvpage = TAILQ_FIRST(&pv_unusedpgs);
	if (mode != ALLOCPV_NONEED && pvpage != NULL) {

		/* move it to pv_freepages list */
		TAILQ_REMOVE(&pv_unusedpgs, pvpage, pvinfo.pvpi_list);
		TAILQ_INSERT_HEAD(&pv_freepages, pvpage, pvinfo.pvpi_list);

		/* allocate a pv_entry */
		pvpage->pvinfo.pvpi_nfree--;	/* can't go to zero */
		pv = pvpage->pvinfo.pvpi_pvfree;
#ifdef DIAGNOSTIC
		if (pv == NULL)
			panic("pmap_alloc_pvpage: pvpi_nfree off");
#endif
		pvpage->pvinfo.pvpi_pvfree = pv->pv_next;

		pv_nfpvents--;  /* took one from pool */
		return(pv);
	}

	/*
	 *  see if we've got a cached unmapped VA that we can map a page in.
	 * if not, try to allocate one.
	 */

	s = splvm();   /* must protect kmem_map with splvm! */
	if (pv_cachedva == 0) {
		pv_cachedva = uvm_km_kmemalloc(kmem_map, NULL,
		    NBPG, UVM_KMF_TRYLOCK|UVM_KMF_VALLOC);
	}
	splx(s);
	if (pv_cachedva == 0)
		goto steal_one;

	pg = uvm_pagealloc(NULL, 0, NULL, UVM_PGA_USERESERVE);
	if (pg == NULL)
		goto steal_one;
	
	atomic_clearbits_int(&pg->pg_flags, PG_BUSY);

	/*
	 * add a mapping for our new pv_page and free its entrys (save one!)
	 *
	 * NOTE: If we are allocating a PV page for the kernel pmap, the
	 * pmap is already locked!  (...but entering the mapping is safe...)
	 */

	pmap_kenter_pa(pv_cachedva, VM_PAGE_TO_PHYS(pg),
	    VM_PROT_READ|VM_PROT_WRITE);
	pmap_update(pmap_kernel());
	pvpage = (struct pv_page *) pv_cachedva;
	pv_cachedva = 0;
	return(pmap_add_pvpage(pvpage, mode != ALLOCPV_NONEED));

steal_one:
	/*
	 * if we don't really need a pv_entry right now, we can just return.
	 */

	if (mode != ALLOCPV_NEED)
		return(NULL);

	/*
	 * last ditch effort!   we couldn't allocate a free page to make
	 * more pv_entrys so we try and steal one from someone else.
	 */

	pv = NULL;
	for (lcv = 0 ; pv == NULL && lcv < vm_nphysseg ; lcv++) {
		npg = vm_physmem[lcv].end - vm_physmem[lcv].start;
		for (idx = 0 ; idx < npg ; idx++) {
			struct pv_head *pvhead = vm_physmem[lcv].pmseg.pvhead;

			if (pvhead->pvh_list == NULL)
				continue;	/* spot check */
			if (!simple_lock_try(&pvhead->pvh_lock))
				continue;
			cpv = prevpv = pvhead->pvh_list;
			while (cpv) {
				if (pmap_try_steal_pv(pvhead, cpv, prevpv))
					break;
				prevpv = cpv;
				cpv = cpv->pv_next;
			}
			simple_unlock(&pvhead->pvh_lock);
			/* got one?  break out of the loop! */
			if (cpv) {
				pv = cpv;
				break;
			}
		}
	}

	return(pv);
}

/*
 * pmap_try_steal_pv: try and steal a pv_entry from a pmap
 *
 * => return true if we did it!
 */

boolean_t
pmap_try_steal_pv(struct pv_head *pvh,
		struct pv_entry *cpv, struct pv_entry *prevpv)
{
	pt_entry_t *ptep, opte;
	pt_entry_t *maptp;
#ifdef MULTIPROCESSOR
	int32_t cpumask = 0;
#endif

	/*
	 * we never steal kernel mappings or mappings from pmaps we can't lock
	 */

	if (cpv->pv_pmap == pmap_kernel() ||
	    !simple_lock_try(&cpv->pv_pmap->pm_obj.vmobjlock))
		return(FALSE);

	/*
	 * yes, we can try and steal it.   first we need to remove the
	 * mapping from the pmap.
	 */

	ptep = pmap_tmpmap_pvepte(cpv);
	if (*ptep & PG_W) {
		ptep = NULL;	/* wired page, avoid stealing this one */
	} else {
		/* zap! */
		maptp = (pt_entry_t *)vtomach(cpv->pv_va);
		PTE_ATOMIC_CLEAR(ptep, maptp, opte);
#ifdef MULTIPROCESSOR
		pmap_tlb_shootdown(cpv->pv_pmap, cpv->pv_va, opte, &cpumask);
		pmap_tlb_shootnow(cpumask);
#else
		/* Don't bother deferring in the single CPU case. */
		if (pmap_is_curpmap(cpv->pv_pmap))
			pmap_update_pg(cpv->pv_va);
#endif
		pmap_tmpunmap_pvepte(cpv);
	}
	if (ptep == NULL) {
		simple_unlock(&cpv->pv_pmap->pm_obj.vmobjlock);
		return(FALSE);	/* wired page, abort! */
	}
	cpv->pv_pmap->pm_stats.resident_count--;
	if (cpv->pv_ptp && cpv->pv_ptp->wire_count)
		/* drop PTP's wired count */
		cpv->pv_ptp->wire_count--;

	/*
	 * XXX: if wire_count goes to one the PTP could be freed, however,
	 * we'd have to lock the page queues (etc.) to do that and it could
	 * cause deadlock headaches.   besides, the pmap we just stole from
	 * may want the mapping back anyway, so leave the PTP around.
	 */

	/*
	 * now we need to remove the entry from the pvlist
	 */

	if (cpv == pvh->pvh_list)
		pvh->pvh_list = cpv->pv_next;
	else
		prevpv->pv_next = cpv->pv_next;
	return(TRUE);
}

/*
 * pmap_add_pvpage: add a pv_page's pv_entrys to the free list
 *
 * => caller must hold pvalloc_lock
 * => if need_entry is true, we allocate and return one pv_entry
 */

struct pv_entry *
pmap_add_pvpage(struct pv_page *pvp, boolean_t need_entry)
{
	int tofree, lcv;

	/* do we need to return one? */
	tofree = (need_entry) ? PVE_PER_PVPAGE - 1 : PVE_PER_PVPAGE;

	pvp->pvinfo.pvpi_pvfree = NULL;
	pvp->pvinfo.pvpi_nfree = tofree;
	for (lcv = 0 ; lcv < tofree ; lcv++) {
		pvp->pvents[lcv].pv_next = pvp->pvinfo.pvpi_pvfree;
		pvp->pvinfo.pvpi_pvfree = &pvp->pvents[lcv];
	}
	if (need_entry)
		TAILQ_INSERT_TAIL(&pv_freepages, pvp, pvinfo.pvpi_list);
	else
		TAILQ_INSERT_TAIL(&pv_unusedpgs, pvp, pvinfo.pvpi_list);
	pv_nfpvents += tofree;
	return((need_entry) ? &pvp->pvents[lcv] : NULL);
}

/*
 * pmap_free_pv_doit: actually free a pv_entry
 *
 * => do not call this directly!  instead use either
 *    1. pmap_free_pv ==> free a single pv_entry
 *    2. pmap_free_pvs => free a list of pv_entrys
 * => we must be holding pvalloc_lock
 */

void
pmap_free_pv_doit(struct pv_entry *pv)
{
	struct pv_page *pvp;

	pvp = (struct pv_page *) i386_trunc_page(pv);
	pv_nfpvents++;
	pvp->pvinfo.pvpi_nfree++;

	/* nfree == 1 => fully allocated page just became partly allocated */
	if (pvp->pvinfo.pvpi_nfree == 1) {
		TAILQ_INSERT_HEAD(&pv_freepages, pvp, pvinfo.pvpi_list);
	}

	/* free it */
	pv->pv_next = pvp->pvinfo.pvpi_pvfree;
	pvp->pvinfo.pvpi_pvfree = pv;

	/*
	 * are all pv_page's pv_entry's free?  move it to unused queue.
	 */

	if (pvp->pvinfo.pvpi_nfree == PVE_PER_PVPAGE) {
		TAILQ_REMOVE(&pv_freepages, pvp, pvinfo.pvpi_list);
		TAILQ_INSERT_HEAD(&pv_unusedpgs, pvp, pvinfo.pvpi_list);
	}
}

/*
 * pmap_free_pv: free a single pv_entry
 *
 * => we gain the pvalloc_lock
 */

void
pmap_free_pv(struct pmap *pmap, struct pv_entry *pv)
{
	simple_lock(&pvalloc_lock);
	pmap_free_pv_doit(pv);

	/*
	 * Can't free the PV page if the PV entries were associated with
	 * the kernel pmap; the pmap is already locked.
	 */
	if (pv_nfpvents > PVE_HIWAT && TAILQ_FIRST(&pv_unusedpgs) != NULL &&
	    pmap != pmap_kernel())
		pmap_free_pvpage();

	simple_unlock(&pvalloc_lock);
}

/*
 * pmap_free_pvs: free a list of pv_entrys
 *
 * => we gain the pvalloc_lock
 */

void
pmap_free_pvs(struct pmap *pmap, struct pv_entry *pvs)
{
	struct pv_entry *nextpv;

	simple_lock(&pvalloc_lock);

	for ( /* null */ ; pvs != NULL ; pvs = nextpv) {
		nextpv = pvs->pv_next;
		pmap_free_pv_doit(pvs);
	}

	/*
	 * Can't free the PV page if the PV entries were associated with
	 * the kernel pmap; the pmap is already locked.
	 */
	if (pv_nfpvents > PVE_HIWAT && TAILQ_FIRST(&pv_unusedpgs) != NULL &&
	    pmap != pmap_kernel())
		pmap_free_pvpage();

	simple_unlock(&pvalloc_lock);
}


/*
 * pmap_free_pvpage: try and free an unused pv_page structure
 *
 * => assume caller is holding the pvalloc_lock and that
 *	there is a page on the pv_unusedpgs list
 * => if we can't get a lock on the kmem_map we try again later
 */

void
pmap_free_pvpage(void)
{
	int s;
	struct vm_map *map;
	struct vm_map_entry *dead_entries;
	struct pv_page *pvp;

	s = splvm(); /* protect kmem_map */
	pvp = TAILQ_FIRST(&pv_unusedpgs);

	/*
	 * note: watch out for pv_initpage which is allocated out of
	 * kernel_map rather than kmem_map.
	 */

	if (pvp == pv_initpage)
		map = kernel_map;
	else
		map = kmem_map;
	if (vm_map_lock_try(map)) {

		/* remove pvp from pv_unusedpgs */
		TAILQ_REMOVE(&pv_unusedpgs, pvp, pvinfo.pvpi_list);

		/* unmap the page */
		dead_entries = NULL;
		uvm_unmap_remove(map, (vaddr_t)pvp, ((vaddr_t)pvp) + PAGE_SIZE,
		    &dead_entries, NULL);
		vm_map_unlock(map);

		if (dead_entries != NULL)
			uvm_unmap_detach(dead_entries, 0);

		pv_nfpvents -= PVE_PER_PVPAGE;  /* update free count */
	}

	if (pvp == pv_initpage)
		/* no more initpage, we've freed it */
		pv_initpage = NULL;

	splx(s);
}

/*
 * main pv_entry manipulation functions:
 *   pmap_enter_pv: enter a mapping onto a pv_head list
 *   pmap_remove_pv: remove a mapping from a pv_head list
 *
 * NOTE: pmap_enter_pv expects to lock the pvh itself
 *       pmap_remove_pv expects te caller to lock the pvh before calling
 */

/*
 * pmap_enter_pv: enter a mapping onto a pv_head lst
 *
 * => caller should hold the proper lock on pmap_main_lock
 * => caller should have pmap locked
 * => we will gain the lock on the pv_head and allocate the new pv_entry
 * => caller should adjust ptp's wire_count before calling
 */

void
pmap_enter_pv(struct pv_head *pvh, struct pv_entry *pve,
		struct pmap *pmap, vaddr_t va, struct vm_page *ptp)
{
	pve->pv_pmap = pmap;
	pve->pv_va = va;
	pve->pv_ptp = ptp;			/* NULL for kernel pmap */
	simple_lock(&pvh->pvh_lock);		/* lock pv_head */
	pve->pv_next = pvh->pvh_list;		/* add to ... */
	pvh->pvh_list = pve;			/* ... locked list */
	simple_unlock(&pvh->pvh_lock);		/* unlock, done! */
}

/*
 * pmap_remove_pv: try to remove a mapping from a pv_list
 *
 * => caller should hold proper lock on pmap_main_lock
 * => pmap should be locked
 * => caller should hold lock on pv_head [so that attrs can be adjusted]
 * => caller should adjust ptp's wire_count and free PTP if needed
 * => we return the removed pve
 */

struct pv_entry *
pmap_remove_pv(struct pv_head *pvh, struct pmap *pmap, vaddr_t va)
{
	struct pv_entry *pve, **prevptr;

	prevptr = &pvh->pvh_list;		/* previous pv_entry pointer */
	pve = *prevptr;
	while (pve) {
		if (pve->pv_pmap == pmap && pve->pv_va == va) {	/* match? */
			*prevptr = pve->pv_next;		/* remove it! */
			break;
		}
		prevptr = &pve->pv_next;		/* previous pointer */
		pve = pve->pv_next;			/* advance */
	}
	return(pve);				/* return removed pve */
}

/*
 * p t p   f u n c t i o n s
 */

/*
 * pmap_alloc_ptp: allocate a PTP for a PMAP
 *
 * => pmap should already be locked by caller
 * => we use the ptp's wire_count to count the number of active mappings
 *	in the PTP (we start it at one to prevent any chance this PTP
 *	will ever leak onto the active/inactive queues)
 * => we should not be holding any pv_head locks (in case we are forced
 *	to call pmap_steal_ptp())
 * => we may need to lock pv_head's if we have to steal a PTP
 * => just_try: true if we want a PTP, but not enough to steal one
 * 	from another pmap (e.g. during optional functions like pmap_copy)
 */

struct vm_page *
pmap_alloc_ptp(struct pmap *pmap, int pde_index, boolean_t just_try)
{
	struct vm_page *ptp;
	pd_entry_t *mapdp;

	ptp = uvm_pagealloc(&pmap->pm_obj, ptp_i2o(pde_index), NULL,
			    UVM_PGA_USERESERVE|UVM_PGA_ZERO);
	if (ptp == NULL) {
		if (just_try)
			return(NULL);
		ptp = pmap_steal_ptp(&pmap->pm_obj, ptp_i2o(pde_index));
		if (ptp == NULL) {
			return (NULL);
		}
		/* stole one; zero it. */
		pmap_zero_page(ptp);
	}

	/* got one! */
	atomic_clearbits_int(&ptp->pg_flags, PG_BUSY);
	ptp->wire_count = 1;	/* no mappings yet */
	mapdp = (pt_entry_t *)vtomach((vaddr_t)&pmap->pm_pdir[pde_index]);
	PDE_SET(&pmap->pm_pdir[pde_index], mapdp,
	    (pd_entry_t)(VM_PAGE_TO_PHYS(ptp) | PG_u | PG_RW | PG_V));
	pmap->pm_stats.resident_count++;	/* count PTP as resident */
	pmap->pm_ptphint = ptp;
	return(ptp);
}

/*
 * pmap_steal_ptp: steal a PTP from any pmap that we can access
 *
 * => obj is locked by caller.
 * => we can throw away mappings at this level (except in the kernel's pmap)
 * => stolen PTP is placed in <obj,offset> pmap
 * => we lock pv_head's
 * => hopefully, this function will be seldom used [much better to have
 *	enough free pages around for us to allocate off the free page list]
 */

struct vm_page *
pmap_steal_ptp(struct uvm_object *obj, vaddr_t offset)
{
	struct vm_page *ptp = NULL;
	struct pmap *firstpmap;
	struct uvm_object *curobj;
	pt_entry_t *ptes;
	pt_entry_t *maptp;
	int idx, lcv;
	boolean_t caller_locked, we_locked;
	int32_t cpumask = 0;

	simple_lock(&pmaps_lock);
	if (pmaps_hand == NULL)
		pmaps_hand = LIST_FIRST(&pmaps);
	firstpmap = pmaps_hand;

	do { /* while we haven't looped back around to firstpmap */

		curobj = &pmaps_hand->pm_obj;
		we_locked = FALSE;
		caller_locked = (curobj == obj);
		if (!caller_locked) {
			we_locked = simple_lock_try(&curobj->vmobjlock);
		}
		if (caller_locked || we_locked) {
			TAILQ_FOREACH(ptp, &curobj->memq, listq) {

				/*
				 * might have found a PTP we can steal
				 * (unless it has wired pages).
				 */

				idx = ptp_o2i(ptp->offset);
#ifdef DIAGNOSTIC
				if (VM_PAGE_TO_PHYS(ptp) !=
				    (PDE_GET(&pmaps_hand->pm_pdir[idx])
				    & PG_FRAME))
					panic("pmap_steal_ptp: PTP mismatch!");
#endif

				ptes = (pt_entry_t *)
					pmap_tmpmap_pa(VM_PAGE_TO_PHYS(ptp));
				for (lcv = 0 ; lcv < PTES_PER_PTP ; lcv++)
					if ((ptes[lcv] & (PG_V|PG_W)) ==
					    (PG_V|PG_W))
						break;
				if (lcv == PTES_PER_PTP)
					pmap_remove_ptes(pmaps_hand, ptp,
					    (vaddr_t)ptes, ptp_i2v(idx),
					    ptp_i2v(idx+1), &cpumask,
					    PMAP_REMOVE_ALL);
				pmap_tmpunmap_pa();

				if (lcv != PTES_PER_PTP)
					/* wired, try next PTP */
					continue;

				/*
				 * got it!!!
				 */
				/* zap the old PTE */
				maptp = (pt_entry_t *)vtomach(
					(vaddr_t)&pmaps_hand->pm_pdir[idx]);
				PTE_CLEAR(&pmaps_hand->pm_pdir[idx],
					maptp);
				pmaps_hand->pm_stats.resident_count--;
#ifdef MULTIPROCESSOR
				pmap_apte_flush(pmaps_hand);
#else
				if (pmap_is_curpmap(pmaps_hand))
					pmap_apte_flush(pmaps_hand);
				else if (pmap_valid_entry(PDE_GET(APDP_PDE)) &&
				    (PDE_GET(APDP_PDE) & PG_FRAME) ==
				    pmaps_hand->pm_pdirpa)
					pmap_update_pg(((vaddr_t)APTE_BASE) +
						       ptp->offset);
#endif

				/* put it in our pmap! */
				uvm_pagerealloc(ptp, obj, offset);
				break;	/* break out of "for" loop */
			}
			if (we_locked) {
				simple_unlock(&curobj->vmobjlock);
			}
		}

		/* advance the pmaps_hand */
		pmaps_hand = LIST_NEXT(pmaps_hand, pm_list);
		if (pmaps_hand == NULL) {
			pmaps_hand = LIST_FIRST(&pmaps);
		}

	} while (ptp == NULL && pmaps_hand != firstpmap);

	simple_unlock(&pmaps_lock);
	pmap_tlb_shootnow(cpumask);
	return(ptp);
}

/*
 * pmap_get_ptp: get a PTP (if there isn't one, allocate a new one)
 *
 * => pmap should NOT be pmap_kernel()
 * => pmap should be locked
 */

struct vm_page *
pmap_get_ptp(struct pmap *pmap, int pde_index, boolean_t just_try)
{
	struct vm_page *ptp;

	if (pmap_valid_entry(pmap->pm_pdir[pde_index])) {

		/* valid... check hint (saves us a PA->PG lookup) */
		if (pmap->pm_ptphint &&
		    (PDE_GET(&pmap->pm_pdir[pde_index]) & PG_FRAME) ==
		    VM_PAGE_TO_PHYS(pmap->pm_ptphint))
			return(pmap->pm_ptphint);

		ptp = uvm_pagelookup(&pmap->pm_obj, ptp_i2o(pde_index));
#ifdef DIAGNOSTIC
		if (ptp == NULL)
			panic("pmap_get_ptp: unmanaged user PTP");
#endif
		pmap->pm_ptphint = ptp;
		return(ptp);
	}

	/* allocate a new PTP (updates ptphint) */
	return(pmap_alloc_ptp(pmap, pde_index, just_try));
}

/*
 * p m a p  l i f e c y c l e   f u n c t i o n s
 */

/*
 * pmap_create: create a pmap
 *
 * => note: old pmap interface took a "size" args which allowed for
 *	the creation of "software only" pmaps (not in bsd).
 */

struct pmap *
pmap_create(void)
{
	struct pmap *pmap;

	pmap = pool_get(&pmap_pmap_pool, PR_WAITOK);
	pmap_pinit(pmap);
	return(pmap);
}

/*
 * pmap_pinit: given a zero'd pmap structure, init it.
 */

void
pmap_pinit(struct pmap *pmap)
{
	int s;

	/* init uvm_object */
	simple_lock_init(&pmap->pm_obj.vmobjlock);
	pmap->pm_obj.pgops = NULL;	/* currently not a mappable object */
	TAILQ_INIT(&pmap->pm_obj.memq);
	pmap->pm_obj.uo_npages = 0;
	pmap->pm_obj.uo_refs = 1;
	pmap->pm_stats.wired_count = 0;
	pmap->pm_stats.resident_count = 1;	/* count the PDP allocd below */
	pmap->pm_ptphint = NULL;
	pmap->pm_hiexec = 0;
	pmap->pm_flags = 0;

	/* allocate PDP */
	pmap->pm_pdir = (pd_entry_t *) uvm_km_alloc(kernel_map, NBPG);
	if (pmap->pm_pdir == NULL)
		panic("pmap_pinit: kernel_map out of virtual space!");
	(void) pmap_extract(pmap_kernel(), (vaddr_t)pmap->pm_pdir,
			    (paddr_t *)&pmap->pm_pdirpa);

	/* init PDP */
	/* zero init area */
	bzero(pmap->pm_pdir, PDSLOT_PTE * sizeof(pd_entry_t));
	/* put in recursive PDE to map the PTEs */
	pmap->pm_pdir[PDSLOT_PTE] = xpmap_ptom(pmap->pm_pdirpa | PG_V /*| PG_KW*/);

	/* init the LDT */
	pmap->pm_ldt = NULL;
	pmap->pm_ldt_len = 0;
	pmap->pm_ldt_sel = GSEL(GLDT_SEL, SEL_KPL);

	/*
	 * we need to lock pmaps_lock to prevent nkpde from changing on
	 * us.   note that there is no need to splvm to protect us from
	 * malloc since malloc allocates out of a submap and we should have
	 * already allocated kernel PTPs to cover the range...
	 */
	simple_lock(&pmaps_lock);
	/* put in kernel VM PDEs */
	bcopy(&PDP_BASE[PDSLOT_KERN], &pmap->pm_pdir[PDSLOT_KERN],
	       nkpde * sizeof(pd_entry_t));
	/* zero the rest */
	bzero(&pmap->pm_pdir[PDSLOT_KERN + nkpde],
	       NBPG - ((PDSLOT_KERN + nkpde) * sizeof(pd_entry_t)));

	XENPRINTF(("pmap_init: pdir %08x pdirpa %08x\n",
		(vaddr_t)pmap->pm_pdir, pmap->pm_pdirpa));

	pmap_kenter_pa((vaddr_t)pmap->pm_pdir, pmap->pm_pdirpa, VM_PROT_READ);
	pmap_update(pmap_kernel());

	/* pin page type */
	s = splvm();
	xpq_queue_pin_table(xpmap_ptom(pmap->pm_pdirpa), XPQ_PIN_L2_TABLE);
	xpq_flush_queue();
	splx(s);

	XENPRINTF(("pmap_pinit: %p set pm_pdirpa %p/%p slotval %p\n", pmap,
	    (void *)pmap->pm_pdirpa,
	    (void *)xpmap_ptom(pmap->pm_pdirpa),
	    (void *)pmap->pm_pdir[PDSLOT_PTE]));

	LIST_INSERT_HEAD(&pmaps, pmap, pm_list);
	simple_unlock(&pmaps_lock);
}

/*
 * pmap_destroy: drop reference count on pmap.   free pmap if
 *	reference count goes to zero.
 */

void
pmap_destroy(struct pmap *pmap)
{
	int refs;

	/*
	 * drop reference count
	 */

	simple_lock(&pmap->pm_obj.vmobjlock);
	refs = --pmap->pm_obj.uo_refs;
	simple_unlock(&pmap->pm_obj.vmobjlock);
	if (refs > 0) {
		return;
	}

	/*
	 * reference count is zero, free pmap resources and then free pmap.
	 */

	XENPRINTF(("pmap_destroy: %p pm_pdirpa %p/%p\n", pmap,
	    (void *)pmap->pm_pdirpa,
	    (void *)xpmap_ptom(pmap->pm_pdirpa)));

	pmap_release(pmap);
	pool_put(&pmap_pmap_pool, pmap);
}

/*
 * pmap_release: release all resources held by a pmap
 *
 * => if pmap is still referenced it should be locked
 * => XXX: we currently don't expect any busy PTPs because we don't
 *    allow anything to map them (except for the kernel's private
 *    recursive mapping) or make them busy.
 */

void
pmap_release(struct pmap *pmap)
{
	int s;

	/*
	 * remove it from global list of pmaps
	 */

	simple_lock(&pmaps_lock);
	if (pmap == pmaps_hand)
		pmaps_hand = LIST_NEXT(pmaps_hand, pm_list);
	LIST_REMOVE(pmap, pm_list);
	simple_unlock(&pmaps_lock);

	/*
	 * destroyed pmap shouldn't have remaining PTPs
	 */

	KASSERT(pmap->pm_obj.uo_npages == 0);
	KASSERT(TAILQ_EMPTY(&pmap->pm_obj.memq));	

	XENPRINTF(("pmap_release: pdir %08x pdirpa %08x\n",
		(vaddr_t)pmap->pm_pdir, pmap->pm_pdirpa));

	/* unpin page type */
	s = splvm();
	xpq_queue_unpin_table(xpmap_ptom(pmap->pm_pdirpa));
	xpq_flush_queue();
	splx(s);

	/*
	 * MULTIPROCESSOR -- no need to flush out of other processors'
	 * APTE space because we do that in pmap_unmap_ptes().
	 */
	pmap_kremove((vaddr_t)pmap->pm_pdir, 1);
	pmap_update(pmap_kernel());

#ifdef USER_LDT
	if (pmap->pm_flags & PMF_USER_LDT) {
		/*
		 * no need to switch the LDT; this address space is gone,
		 * nothing is using it.
		 *
		 * No need to lock the pmap for ldt_free (or anything else),
		 * we're the last one to use it.
		 */
		ldt_free(pmap);
		uvm_km_free(kernel_map, (vaddr_t)pmap->pm_ldt,
			    pmap->pm_ldt_len * sizeof(union descriptor));
	}
#endif
}

/*
 *	Add a reference to the specified pmap.
 */

void
pmap_reference(struct pmap *pmap)
{
	simple_lock(&pmap->pm_obj.vmobjlock);
	pmap->pm_obj.uo_refs++;
	simple_unlock(&pmap->pm_obj.vmobjlock);
}

#if defined(PMAP_FORK)
/*
 * pmap_fork: perform any necessary data structure manipulation when
 * a VM space is forked.
 */

void
pmap_fork(struct pmap *pmap1, struct pmap *pmap2)
{
	simple_lock(&pmap1->pm_obj.vmobjlock);
	simple_lock(&pmap2->pm_obj.vmobjlock);

#ifdef USER_LDT
	/* Copy the LDT, if necessary. */
	if (pmap1->pm_flags & PMF_USER_LDT) {
		union descriptor *new_ldt;
		size_t len;

		len = pmap1->pm_ldt_len * sizeof(union descriptor);
		new_ldt = (union descriptor *)uvm_km_alloc(kernel_map, len);
		bcopy(pmap1->pm_ldt, new_ldt, len);
		pmap2->pm_ldt = new_ldt;
		pmap2->pm_ldt_len = pmap1->pm_ldt_len;
		pmap2->pm_flags |= PMF_USER_LDT;
		ldt_alloc(pmap2, new_ldt, len);
	}
#endif /* USER_LDT */

	simple_unlock(&pmap2->pm_obj.vmobjlock);
	simple_unlock(&pmap1->pm_obj.vmobjlock);
}
#endif /* PMAP_FORK */

#ifdef USER_LDT
/*
 * pmap_ldt_cleanup: if the pmap has a local LDT, deallocate it, and
 * restore the default.
 */

void
pmap_ldt_cleanup(struct proc *p)
{
	struct pcb *pcb = &p->p_addr->u_pcb;
	pmap_t pmap = p->p_vmspace->vm_map.pmap;
	union descriptor *old_ldt = NULL;
	size_t len = 0;

	simple_lock(&pmap->pm_obj.vmobjlock);

	if (pmap->pm_flags & PMF_USER_LDT) {
		ldt_free(pmap);
		pmap->pm_ldt_sel = GSEL(GLDT_SEL, SEL_KPL);
		pcb->pcb_ldt_sel = pmap->pm_ldt_sel;
		if (pcb == curpcb)
			lldt(pcb->pcb_ldt_sel);
		old_ldt = pmap->pm_ldt;
		len = pmap->pm_ldt_len * sizeof(union descriptor);
		pmap->pm_ldt = NULL;
		pmap->pm_ldt_len = 0;
		pmap->pm_flags &= ~PMF_USER_LDT;
	}

	simple_unlock(&pmap->pm_obj.vmobjlock);

	if (old_ldt != NULL)
		uvm_km_free(kernel_map, (vaddr_t)old_ldt, len);
}
#endif /* USER_LDT */

/*
 * pmap_activate: activate a process' pmap (fill in %cr3 and LDT info)
 *
 * => called from cpu_switch()
 * => if proc is the curproc, then load it into the MMU
 */

void
pmap_activate(struct proc *p)
{
	struct pcb *pcb = &p->p_addr->u_pcb;
	struct pmap *pmap = vm_map_pmap(&p->p_vmspace->vm_map);
	pd_entry_t *mapdp;

	/*
	 * Clear ADPD slot before loading %cr3 since Xen only allows linear
	 * pagetable mappings in the current pagetable.
	 */
	mapdp = (pt_entry_t *)vtomach((vaddr_t)APDP_PDE);
	PDE_CLEAR(APDP_PDE, mapdp);

	pcb->pcb_pmap = pmap;
	pcb->pcb_ldt_sel = pmap->pm_ldt_sel;
	pcb->pcb_cr3 = pmap->pm_pdirpa;
	if (p == curproc) {
		lcr3(pcb->pcb_cr3);
		lldt(pcb->pcb_ldt_sel);

		/*
		 * mark the pmap in use by this processor.
		 */
		i386_atomic_setbits_l(&pmap->pm_cpus, (1U << cpu_number()));
	}
}

/*
 * pmap_deactivate: deactivate a process' pmap
 */

void
pmap_deactivate(struct proc *p)
{
	struct pmap *pmap = p->p_vmspace->vm_map.pmap;

	/*
	 * mark the pmap no longer in use by this processor.
	 */
	i386_atomic_clearbits_l(&pmap->pm_cpus, (1U << cpu_number()));

	XENPRINTF(("pmap_deactivate %p ebp %p esp %p\n",
			p, (void *)p->p_addr->u_pcb.pcb_ebp,
			(void *)p->p_addr->u_pcb.pcb_esp));
}

/*
 * end of lifecycle functions
 */

/*
 * some misc. functions
 */

/*
 * pmap_extract: extract a PA for the given VA
 */

boolean_t
pmap_extract(struct pmap *pmap, vaddr_t va, paddr_t *pap)
{
	pt_entry_t *ptes, pte;

	if (PDE_GET(&pmap->pm_pdir[pdei(va)])) {
		ptes = pmap_map_ptes(pmap);
		pte = PTE_GET(&ptes[i386_btop(va)]);
		pmap_unmap_ptes(pmap);
		if ((pte & PG_V) != 0) {
			if (pap != NULL)
				*pap = (pte & PG_FRAME) | (va & ~PG_FRAME);
			return (TRUE);
		}
	}
	return (FALSE);
}

/*
 * pmap_extract_ma: like pmap_extract, but returns machine address
 */


boolean_t
pmap_extract_ma(struct pmap *pmap, vaddr_t va, paddr_t *pap)
{
	pt_entry_t *ptes, pte;

	if (PDE_GET(&pmap->pm_pdir[pdei(va)])) {
		ptes = pmap_map_ptes(pmap);
		pte = PTE_GET_MA(&ptes[i386_btop(va)]);
		pmap_unmap_ptes(pmap);
		if ((pte & PG_V) != 0) {
			if (pap != NULL)
				*pap = (pte & PG_FRAME) | (va & ~PG_FRAME);
			return (TRUE);
		}
	}
	return (FALSE);
}

/*
 * vtophys: virtual address to physical address.  For use by
 * machine-dependent code only.
 */

paddr_t
vtophys(vaddr_t va)
{
	paddr_t pa;

	if (pmap_extract(pmap_kernel(), va, &pa) == TRUE)
		return (pa);
	return (0);
}

/*
 * pmap_virtual_space: used during bootup [pmap_steal_memory] to
 *	determine the bounds of the kernel virtual addess space.
 */

void
pmap_virtual_space(vaddr_t *startp, vaddr_t *endp)
{
	*startp = virtual_avail;
	*endp = virtual_end;
}

/*
 * pmap_zero_page: zero a page
 */
void (*pagezero)(void *, size_t) = bzero;

void
pmap_zero_page(struct vm_page *pg)
{
	pmap_zero_phys(VM_PAGE_TO_PHYS(pg));	/* XXX ? */
}

/*
 * pmap_zero_phys: same as pmap_zero_page, but for use before vm_pages are
 * initialized.
 */
void
pmap_zero_phys(paddr_t pa)
{
#ifdef MULTIPROCESSOR
	int id = cpu_number();
#endif
	pt_entry_t *zpte = PTESLEW(zero_pte, id);
	pt_entry_t *maptp;
	caddr_t zerova = VASLEW(zerop, id);

#ifdef DIAGNOSTIC
	if (*zpte)
		panic("pmap_zero_phys: lock botch");
#endif

	maptp = (pt_entry_t *)vtomach((vaddr_t)zpte);
	PTE_SET(zpte, maptp,
		(pa & PG_FRAME) | PG_V | PG_RW | PG_M | PG_U);	/* map in */
	pmap_update_pg((vaddr_t)zerova);	/* flush TLB */
	pagezero(zerova, PAGE_SIZE);		/* zero */
	PTE_CLEAR(zpte, maptp);			/* zap! */
}

/*
 * pmap_zero_page_uncached: the same, except uncached.
 */

boolean_t
pmap_zero_page_uncached(paddr_t pa)
{
#ifdef MULTIPROCESSOR
	int id = cpu_number();
#endif
	pt_entry_t *zpte = PTESLEW(zero_pte, id);
	pt_entry_t *maptp;
	caddr_t zerova = VASLEW(zerop, id);

#ifdef DIAGNOSTIC
	if (*zpte)
		panic("pmap_zero_page_uncached: lock botch");
#endif

	maptp = (pt_entry_t *)vtomach((vaddr_t)zpte);
	PTE_SET(zpte, maptp, (pa & PG_FRAME) | PG_V | PG_RW |
	    ((cpu_class != CPUCLASS_386) ? PG_N : 0));	/* map in */
	pmap_update_pg((vaddr_t)zerova);		/* flush TLB */
	pagezero(zerova, PAGE_SIZE);			/* zero */
	PTE_CLEAR(zpte, maptp);				/* zap! */

	return (TRUE);
}

/*
 * pmap_copy_page: copy a page
 */

void
pmap_copy_page(struct vm_page *srcpg, struct vm_page *dstpg)
{
	paddr_t srcpa = VM_PAGE_TO_PHYS(srcpg);
	paddr_t dstpa = VM_PAGE_TO_PHYS(dstpg);
#ifdef MULTIPROCESSOR
	int id = cpu_number();
#endif
	pt_entry_t *spte = PTESLEW(csrc_pte,id), *maspte;
	pt_entry_t *dpte = PTESLEW(cdst_pte,id), *madpte;
	caddr_t csrcva = VASLEW(csrcp, id);
	caddr_t cdstva = VASLEW(cdstp, id);

#ifdef DIAGNOSTIC
	if (PTE_GET(spte) || PTE_GET(dpte))
		panic("pmap_copy_page: lock botch");
#endif

	maspte = (pt_entry_t *)vtomach((vaddr_t)spte);
	madpte = (pt_entry_t *)vtomach((vaddr_t)dpte);
	PTE_SET(spte, maspte, (srcpa & PG_FRAME) | PG_V | PG_RW | PG_U);
	PTE_SET(dpte, madpte, (dstpa & PG_FRAME) | PG_V | PG_RW | PG_M | PG_U);
	pmap_update_2pg((vaddr_t)csrcva, (vaddr_t)cdstva);
	bcopy(csrcva, cdstva, PAGE_SIZE);
	PTE_CLEAR(spte, maspte);		/* zap! */
	PTE_CLEAR(dpte, madpte);		/* zap! */
	pmap_update_2pg((vaddr_t)csrcva, (vaddr_t)cdstva);	/* XXX */
#ifdef MULTIPROCESSOR
	/* Using per-cpu VA; no shootdown required here. */
#endif
}

/*
 * p m a p   r e m o v e   f u n c t i o n s
 *
 * functions that remove mappings
 */

/*
 * pmap_remove_ptes: remove PTEs from a PTP
 *
 * => must have proper locking on pmap_master_lock
 * => caller must hold pmap's lock
 * => PTP must be mapped into KVA
 * => PTP should be null if pmap == pmap_kernel()
 */

void
pmap_remove_ptes(struct pmap *pmap, struct vm_page *ptp,
		vaddr_t ptpva, vaddr_t startva, vaddr_t endva,
		int32_t *cpumaskp, int flags)
{
	struct pv_entry *pv_tofree = NULL;	/* list of pv_entrys to free */
	struct pv_entry *pve;
	pt_entry_t *pte = (pt_entry_t *) ptpva;
	pt_entry_t opte;
	pt_entry_t *maptp;
	int bank, off;

	/*
	 * note that ptpva points to the PTE that maps startva.   this may
	 * or may not be the first PTE in the PTP.
	 *
	 * we loop through the PTP while there are still PTEs to look at
	 * and the wire_count is greater than 1 (because we use the wire_count
	 * to keep track of the number of real PTEs in the PTP).
	 */

	for (/*null*/; startva < endva && (ptp == NULL || ptp->wire_count > 1)
			     ; pte++, startva += NBPG) {
		if (!pmap_valid_entry(*pte))
			continue;			/* VA not mapped */
		if ((flags & PMAP_REMOVE_SKIPWIRED) && (*pte & PG_W)) {
			continue;
		}


		/* atomically save the old PTE and zap! it */
		maptp = (pt_entry_t *)vtomach((vaddr_t)pte);
		opte = pte_atomic_update(pte, maptp, 0);
		pmap_exec_account(pmap, startva, opte, 0);

		if (opte & PG_W)
			pmap->pm_stats.wired_count--;
		pmap->pm_stats.resident_count--;

		if (opte & PG_U)
			pmap_tlb_shootdown(pmap, startva, opte, cpumaskp);

		if (ptp) {
			ptp->wire_count--;		/* dropping a PTE */
			/* Make sure that the PDE is flushed */
			if ((ptp->wire_count <= 1) && !(opte & PG_U))
				pmap_tlb_shootdown(pmap, startva, opte,
				    cpumaskp);
		}

		/*
		 * if we are not on a pv_head list we are done.
		 */

		if ((opte & PG_PVLIST) == 0) {
#if defined(DIAGNOSTIC) && !defined(DOM0OPS)
			if (vm_physseg_find(i386_btop(opte & PG_FRAME), &off)
			    != -1)
				panic("pmap_remove_ptes: managed page without "
				      "PG_PVLIST for 0x%lx", startva);
#endif
			continue;
		}

		/* XXX */
		bank = vm_physseg_find(i386_btop(opte & PG_FRAME), &off);
#ifdef DIAGNOSTIC
		if (bank == -1)
			panic("pmap_remove_ptes: unmanaged page marked "
			      "PG_PVLIST, va = 0x%lx, pa = 0x%lx",
			      startva, (u_long)(opte & PG_FRAME));
#endif

		/* sync R/M bits */
		simple_lock(&vm_physmem[bank].pmseg.pvhead[off].pvh_lock);
		vm_physmem[bank].pmseg.attrs[off] |= (opte & (PG_U|PG_M));
		pve = pmap_remove_pv(&vm_physmem[bank].pmseg.pvhead[off], pmap,
				     startva);
		simple_unlock(&vm_physmem[bank].pmseg.pvhead[off].pvh_lock);

		if (pve) {
			pve->pv_next = pv_tofree;
			pv_tofree = pve;
		}

		/* end of "for" loop: time for next pte */
	}
	if (pv_tofree)
		pmap_free_pvs(pmap, pv_tofree);
}


/*
 * pmap_remove_pte: remove a single PTE from a PTP
 *
 * => must have proper locking on pmap_master_lock
 * => caller must hold pmap's lock
 * => PTP must be mapped into KVA
 * => PTP should be null if pmap == pmap_kernel()
 * => returns true if we removed a mapping
 */

boolean_t
pmap_remove_pte(struct pmap *pmap, struct vm_page *ptp, pt_entry_t *pte,
		vaddr_t va, int32_t *cpumaskp, int flags)
{
	pt_entry_t opte;
	pt_entry_t *maptp;
	int bank, off;
	struct pv_entry *pve;

	if (!pmap_valid_entry(PTE_GET(pte)))
		return(FALSE);		/* VA not mapped */
	if ((flags & PMAP_REMOVE_SKIPWIRED) && (PTE_GET(pte) & PG_W)) {
		return(FALSE);
	}

	/* Save and zap! the old PTE atomically. */
	maptp = (pt_entry_t *)vtomach((vaddr_t)pte);
	opte = pte_atomic_update(pte, maptp, 0);

	XENPRINTF(("pmap_remove_pte: %p, was %08x\n", pte, opte));
	pmap_exec_account(pmap, va, opte, 0);

	if (opte & PG_W)
		pmap->pm_stats.wired_count--;
	pmap->pm_stats.resident_count--;

	if (opte & PG_U)
		pmap_tlb_shootdown(pmap, va, opte, cpumaskp);

	if (ptp) {
		ptp->wire_count--;		/* dropping a PTE */
		/* Make sure that the PDE is flushed */
		if ((ptp->wire_count <= 1) && !(opte & PG_U))
			pmap_tlb_shootdown(pmap, va, opte, cpumaskp);

	}

	/*
	 * if we are not on a pv_head list we are done.
	 */

	if ((opte & PG_PVLIST) == 0) {
#if defined(DIAGNOSTIC) && !defined(DOM0OPS)
		if (vm_physseg_find(i386_btop(opte & PG_FRAME), &off) != -1)
			panic("pmap_remove_pte: managed page without "
			      "PG_PVLIST for 0x%lx", va);
#endif
		return(TRUE);
	}

	bank = vm_physseg_find(i386_btop(opte & PG_FRAME), &off);
#ifdef DIAGNOSTIC
	if (bank == -1)
		panic("pmap_remove_pte: unmanaged page marked "
		    "PG_PVLIST, va = 0x%lx, pa = 0x%lx", va,
		    (u_long)(opte & PG_FRAME));
#endif

	/* sync R/M bits */
	simple_lock(&vm_physmem[bank].pmseg.pvhead[off].pvh_lock);
	vm_physmem[bank].pmseg.attrs[off] |= (opte & (PG_U|PG_M));
	pve = pmap_remove_pv(&vm_physmem[bank].pmseg.pvhead[off], pmap, va);
	simple_unlock(&vm_physmem[bank].pmseg.pvhead[off].pvh_lock);

	if (pve)
		pmap_free_pv(pmap, pve);
	return(TRUE);
}

/*
 * pmap_remove: top level mapping removal function
 *
 * => caller should not be holding any pmap locks
 */

void
pmap_remove(struct pmap *pmap, vaddr_t sva, vaddr_t eva)
{
	pmap_do_remove(pmap, sva, eva, PMAP_REMOVE_ALL);
}

void
pmap_do_remove(struct pmap *pmap, vaddr_t sva, vaddr_t eva, int flags)
{
	pt_entry_t *ptes, opte;
	pt_entry_t *maptp;
	boolean_t result;
	paddr_t ptppa;
	vaddr_t blkendva;
	struct vm_page *ptp;
	int32_t cpumask = 0;
	TAILQ_HEAD(, vm_page) empty_ptps;

	/*
	 * we lock in the pmap => pv_head direction
	 */

	TAILQ_INIT(&empty_ptps);

	PMAP_MAP_TO_HEAD_LOCK();
	ptes = pmap_map_ptes(pmap);	/* locks pmap */

	/*
	 * removing one page?  take shortcut function.
	 */

	if (sva + PAGE_SIZE == eva) {

		if (pmap_valid_entry(pmap->pm_pdir[pdei(sva)])) {

			/* PA of the PTP */
			ptppa = PDE_GET(&pmap->pm_pdir[pdei(sva)]) & PG_FRAME;

			/* get PTP if non-kernel mapping */
			if (pmap == pmap_kernel()) {
				/* we never free kernel PTPs */
				ptp = NULL;
			} else {
				if (pmap->pm_ptphint &&
				    VM_PAGE_TO_PHYS(pmap->pm_ptphint) ==
				    ptppa) {
					ptp = pmap->pm_ptphint;
				} else {
					ptp = PHYS_TO_VM_PAGE(ptppa);
#ifdef DIAGNOSTIC
					if (ptp == NULL)
						panic("pmap_remove: unmanaged "
						      "PTP detected");
#endif
				}
			}

			/* do it! */
			result = pmap_remove_pte(pmap, ptp,
			    &ptes[i386_btop(sva)], sva, &cpumask, flags);

			/*
			 * if mapping removed and the PTP is no longer
			 * being used, free it!
			 */

			if (result && ptp && ptp->wire_count <= 1) {
				pt_entry_t *pte;
				/* zap! */
				pte = &pmap->pm_pdir[pdei(sva)];
				maptp = (pt_entry_t *)vtomach((vaddr_t)pte);
				PTE_ATOMIC_CLEAR(pte, maptp, opte);
#ifdef MULTIPROCESSOR
				/*
				 * XXXthorpej Redundant shootdown can happen
				 * here if we're using APTE space.
				 */
#endif
				pmap_tlb_shootdown(curpcb->pcb_pmap,
				    ((vaddr_t)ptes) + ptp->offset, opte,
				    &cpumask);
#ifdef MULTIPROCESSOR
				/*
				 * Always shoot down the pmap's self-mapping
				 * of the PTP.
				 * XXXthorpej Redundant shootdown can happen
				 * here if pmap == curpcb->pcb_pmap (not APTE
				 * space).
				 */
				pmap_tlb_shootdown(pmap,
				    ((vaddr_t)PTE_BASE) + ptp->offset, opte,
				    &cpumask);
#endif
				pmap->pm_stats.resident_count--;
				if (pmap->pm_ptphint == ptp)
					pmap->pm_ptphint =
					    TAILQ_FIRST(&pmap->pm_obj.memq);
				ptp->wire_count = 0;
				ptp->pg_flags |= PG_ZERO;
				/* Postpone free to shootdown */
				uvm_pagerealloc(ptp, NULL, 0);
				TAILQ_INSERT_TAIL(&empty_ptps, ptp, listq);
			}
		}
		pmap_tlb_shootnow(cpumask);
		pmap_unmap_ptes(pmap);		/* unlock pmap */
		PMAP_MAP_TO_HEAD_UNLOCK();
		/* Now we can free unused ptps */
		TAILQ_FOREACH(ptp, &empty_ptps, listq)
			uvm_pagefree(ptp);
		return;
	}

	cpumask = 0;

	for (/* null */ ; sva < eva ; sva = blkendva) {

		/* determine range of block */
		blkendva = i386_round_pdr(sva+1);
		if (blkendva > eva)
			blkendva = eva;

		/*
		 * XXXCDC: our PTE mappings should never be removed
		 * with pmap_remove!  if we allow this (and why would
		 * we?) then we end up freeing the pmap's page
		 * directory page (PDP) before we are finished using
		 * it when we hit in in the recursive mapping.  this
		 * is BAD.
		 *
		 * long term solution is to move the PTEs out of user
		 * address space.  and into kernel address space (up
		 * with APTE).  then we can set VM_MAXUSER_ADDRESS to
		 * be VM_MAX_ADDRESS.
		 */

		if (pdei(sva) == PDSLOT_PTE)
			/* XXXCDC: ugly hack to avoid freeing PDP here */
			continue;

		if (!pmap_valid_entry(pmap->pm_pdir[pdei(sva)]))
			/* valid block? */
			continue;

		/* PA of the PTP */
		ptppa = (PDE_GET(&pmap->pm_pdir[pdei(sva)]) & PG_FRAME);

		/* get PTP if non-kernel mapping */
		if (pmap == pmap_kernel()) {
			/* we never free kernel PTPs */
			ptp = NULL;
		} else {
			if (pmap->pm_ptphint &&
			    VM_PAGE_TO_PHYS(pmap->pm_ptphint) == ptppa) {
				ptp = pmap->pm_ptphint;
			} else {
				ptp = PHYS_TO_VM_PAGE(ptppa);
#ifdef DIAGNOSTIC
				if (ptp == NULL)
					panic("pmap_remove: unmanaged PTP "
					      "detected");
#endif
			}
		}
		pmap_remove_ptes(pmap, ptp, (vaddr_t)&ptes[i386_btop(sva)],
		    sva, blkendva, &cpumask, flags);

		/* if PTP is no longer being used, free it! */
		if (ptp && ptp->wire_count <= 1) {
			/* zap! */
			maptp = (pt_entry_t *)vtomach(
			    (vaddr_t)&pmap->pm_pdir[pdei(sva)]);
			PTE_ATOMIC_CLEAR(&pmap->pm_pdir[pdei(sva)],
			    maptp, opte);
#if defined(MULTIPROCESSOR)
			/*
			 * XXXthorpej Redundant shootdown can happen here
			 * if we're using APTE space.
			 */
#endif
			pmap_tlb_shootdown(curpcb->pcb_pmap,
			    ((vaddr_t)ptes) + ptp->offset, opte, &cpumask);
#if defined(MULTIPROCESSOR)
			/*
			 * Always shoot down the pmap's self-mapping
			 * of the PTP.
			 * XXXthorpej Redundant shootdown can happen here
			 * if pmap == curpcb->pcb_pmap (not APTE space).
			 */
			pmap_tlb_shootdown(pmap,
			    ((vaddr_t)PTE_BASE) + ptp->offset, opte, &cpumask);
#endif
			pmap->pm_stats.resident_count--;
			if (pmap->pm_ptphint == ptp)	/* update hint? */
				pmap->pm_ptphint =
				    TAILQ_FIRST(&pmap->pm_obj.memq);
			ptp->wire_count = 0;
			ptp->pg_flags |= PG_ZERO;
			/* Postpone free to shootdown */
			uvm_pagerealloc(ptp, NULL, 0);
			TAILQ_INSERT_TAIL(&empty_ptps, ptp, listq);
		}
	}

	pmap_tlb_shootnow(cpumask);
	pmap_unmap_ptes(pmap);
	PMAP_MAP_TO_HEAD_UNLOCK();
	/* Now we can free unused ptps */
	TAILQ_FOREACH(ptp, &empty_ptps, listq)
		uvm_pagefree(ptp);
}

/*
 * pmap_page_remove: remove a managed vm_page from all pmaps that map it
 *
 * => we set pv_head => pmap locking
 * => R/M bits are sync'd back to attrs
 */

void
pmap_page_remove(struct vm_page *pg)
{
	int bank, off;
	struct pv_head *pvh;
	struct pv_entry *pve;
	pt_entry_t *ptes, opte;
	pt_entry_t *maptp;
	int32_t cpumask = 0;
	TAILQ_HEAD(, vm_page) empty_ptps;
	struct vm_page *ptp;

	/* XXX: vm_page should either contain pv_head or have a pointer to it */
	bank = vm_physseg_find(atop(VM_PAGE_TO_PHYS(pg)), &off);
	if (bank == -1) {
		printf("pmap_page_remove: unmanaged page?\n");
		return;
	}

	pvh = &vm_physmem[bank].pmseg.pvhead[off];
	if (pvh->pvh_list == NULL) {
		return;
	}


	TAILQ_INIT(&empty_ptps);

	/* set pv_head => pmap locking */
	PMAP_HEAD_TO_MAP_LOCK();

	/* XXX: needed if we hold head->map lock? */
	simple_lock(&pvh->pvh_lock);

	for (pve = pvh->pvh_list ; pve != NULL ; pve = pve->pv_next) {
		KASSERT(pve->pv_pmap != NULL);
		ptes = pmap_map_ptes(pve->pv_pmap);		/* locks pmap */

#ifdef DIAGNOSTIC
		if (pve->pv_va >= uvm.pager_sva && pve->pv_va < uvm.pager_eva) {
			printf("pmap_page_remove: found pager VA on pv_list\n");
		}
		if (pve->pv_ptp &&
		    (PDE_GET(&pve->pv_pmap->pm_pdir[pdei(pve->pv_va)]) &
		    PG_FRAME) != VM_PAGE_TO_PHYS(pve->pv_ptp)) {
			printf("pmap_page_remove: pg=%p: va=%lx, pv_ptp=%p\n",
			    pg, pve->pv_va, pve->pv_ptp);
			printf("pmap_page_remove: PTP's phys addr: "
			    "actual=%x, recorded=%lx\n",
			    (PDE_GET(&pve->pv_pmap->pm_pdir[pdei(pve->pv_va)])
			    & PG_FRAME), VM_PAGE_TO_PHYS(pve->pv_ptp));
			panic("pmap_page_remove: mapped managed page has "
			    "invalid pv_ptp field");
		}
#endif

		/* Save and zap! old PTE atomically. */
		maptp = (pt_entry_t *)vtomach(
		    (vaddr_t)&ptes[i386_btop(pve->pv_va)]);
		opte = pte_atomic_update(&ptes[i386_btop(pve->pv_va)],
		    maptp, 0);

		if (opte & PG_W)
			pve->pv_pmap->pm_stats.wired_count--;
		pve->pv_pmap->pm_stats.resident_count--;

		/* Shootdown only if referenced */
		if (opte & PG_U)
			pmap_tlb_shootdown(pve->pv_pmap, pve->pv_va, opte,
			    &cpumask);

		/* sync R/M bits */
		vm_physmem[bank].pmseg.attrs[off] |= (opte & (PG_U|PG_M));

		/* update the PTP reference count.  free if last reference. */
		if (pve->pv_ptp) {
			pve->pv_ptp->wire_count--;
			if (pve->pv_ptp->wire_count <= 1) {
				/*
				 * Do we have to shootdown the page just to
				 * get the pte out of the TLB ?
				 */
				if(!(opte & PG_U))
					pmap_tlb_shootdown(pve->pv_pmap,
					    pve->pv_va, opte, &cpumask);

				/* zap! */
				maptp = (pt_entry_t *)vtomach((vaddr_t)
				    &pve->pv_pmap->pm_pdir[pdei(pve->pv_va)]);
				PTE_ATOMIC_CLEAR(&pve->pv_pmap->pm_pdir
				    [pdei(pve->pv_va)], maptp, opte);
				pmap_tlb_shootdown(curpcb->pcb_pmap,
				    ((vaddr_t)ptes) + pve->pv_ptp->offset,
				    opte, &cpumask);
#if defined(MULTIPROCESSOR)
				/*
				 * Always shoot down the other pmap's
				 * self-mapping of the PTP.
				 */
				pmap_tlb_shootdown(pve->pv_pmap,
				    ((vaddr_t)PTE_BASE) + pve->pv_ptp->offset,
				    opte, &cpumask);
#endif
				pve->pv_pmap->pm_stats.resident_count--;
				/* update hint? */
				if (pve->pv_pmap->pm_ptphint == pve->pv_ptp)
					pve->pv_pmap->pm_ptphint =
					    TAILQ_FIRST(&pve->pv_pmap->pm_obj.memq);
				pve->pv_ptp->wire_count = 0;
				pve->pv_ptp->pg_flags |= PG_ZERO;
				/* Free only after the shootdown */
				uvm_pagerealloc(pve->pv_ptp, NULL, 0);
				TAILQ_INSERT_TAIL(&empty_ptps, pve->pv_ptp,
				    listq);
			}
		}
		pmap_unmap_ptes(pve->pv_pmap);		/* unlocks pmap */
	}
	pmap_free_pvs(NULL, pvh->pvh_list);
	pvh->pvh_list = NULL;
	simple_unlock(&pvh->pvh_lock);
	PMAP_HEAD_TO_MAP_UNLOCK();
	pmap_tlb_shootnow(cpumask);

	/* Now we can free unused ptps */
	TAILQ_FOREACH(ptp, &empty_ptps, listq)
		uvm_pagefree(ptp);
}

/*
 * p m a p   a t t r i b u t e  f u n c t i o n s
 * functions that test/change managed page's attributes
 * since a page can be mapped multiple times we must check each PTE that
 * maps it by going down the pv lists.
 */

/*
 * pmap_test_attrs: test a page's attributes
 *
 * => we set pv_head => pmap locking
 */

boolean_t
pmap_test_attrs(struct vm_page *pg, int testbits)
{
	int bank, off;
	char *myattrs;
	struct pv_head *pvh;
	struct pv_entry *pve;
	pt_entry_t *ptes, pte;

	/* XXX: vm_page should either contain pv_head or have a pointer to it */
	bank = vm_physseg_find(atop(VM_PAGE_TO_PHYS(pg)), &off);
	if (bank == -1) {
		printf("pmap_test_attrs: unmanaged page?\n");
		return(FALSE);
	}

	/*
	 * before locking: see if attributes are already set and if so,
	 * return!
	 */

	myattrs = &vm_physmem[bank].pmseg.attrs[off];
	if (*myattrs & testbits)
		return(TRUE);

	/* test to see if there is a list before bothering to lock */
	pvh = &vm_physmem[bank].pmseg.pvhead[off];
	if (pvh->pvh_list == NULL) {
		return(FALSE);
	}

	/* nope, gonna have to do it the hard way */
	PMAP_HEAD_TO_MAP_LOCK();
	/* XXX: needed if we hold head->map lock? */
	simple_lock(&pvh->pvh_lock);

	for (pve = pvh->pvh_list; pve != NULL && (*myattrs & testbits) == 0;
	     pve = pve->pv_next) {
		ptes = pmap_map_ptes(pve->pv_pmap);
		pte = PTE_GET(&ptes[i386_btop(pve->pv_va)]);
		pmap_unmap_ptes(pve->pv_pmap);
		*myattrs |= pte;
	}

	/*
	 * note that we will exit the for loop with a non-null pve if
	 * we have found the bits we are testing for.
	 */

	simple_unlock(&pvh->pvh_lock);
	PMAP_HEAD_TO_MAP_UNLOCK();
	return((*myattrs & testbits) != 0);
}

/*
 * pmap_clear_attrs: clear the specified attribute for a page.
 *
 * => we set pv_head => pmap locking
 * => we return TRUE if we cleared one of the bits we were asked to
 */

boolean_t
pmap_clear_attrs(struct vm_page *pg, int clearbits)
{
	uint32_t result;
	int bank, off;
	struct pv_head *pvh;
	struct pv_entry *pve;
	pt_entry_t *ptes, opte;
	pt_entry_t *maptp;
	char *myattrs;
	int32_t cpumask = 0;

	/* XXX: vm_page should either contain pv_head or have a pointer to it */
	bank = vm_physseg_find(atop(VM_PAGE_TO_PHYS(pg)), &off);
	if (bank == -1)
		panic("pmap_clear_attrs: unmanaged page?\n");

	PMAP_HEAD_TO_MAP_LOCK();
	pvh = &vm_physmem[bank].pmseg.pvhead[off];
	/* XXX: needed if we hold head->map lock? */
	simple_lock(&pvh->pvh_lock);

	myattrs = &vm_physmem[bank].pmseg.attrs[off];
	result = *myattrs & clearbits;
	*myattrs &= ~clearbits;

	for (pve = pvh->pvh_list; pve != NULL; pve = pve->pv_next) {
		KASSERT(pve->pv_va != NULL);
#ifdef DIAGNOSTIC
		if (!pmap_valid_entry(pve->pv_pmap->pm_pdir[pdei(pve->pv_va)]))
			panic("pmap_clear_attrs: mapping without PTP "
			      "detected");
#endif
		ptes = pmap_map_ptes(pve->pv_pmap);	/* locks pmap */
		opte = PTE_GET(&ptes[i386_btop(pve->pv_va)]);
		if (opte & clearbits) {
			/* We need to do something */
			if (clearbits == PG_RW) {
				result |= PG_RW;

				/*
				 * On write protect we might not need to flush
				 * the TLB
				 */

				/* First zap the RW bit */
				maptp = (pt_entry_t *)vtomach(
					(vaddr_t)&ptes[i386_btop(pve->pv_va)]);
				PTE_ATOMIC_CLEARBITS(
					&ptes[i386_btop(pve->pv_va)],
					maptp, PG_RW);
				opte = PTE_GET(&ptes[i386_btop(pve->pv_va)]);

				/*
				 * Then test if it is not cached as RW
				 * in the TLB
				 */
				if (!(opte & PG_M))
					goto no_tlb_shootdown;
			}

			/*
			 * Since we need a shootdown we might as well
			 * always clear PG_U AND PG_M.
			 */

			/* zap! */
			maptp = (pt_entry_t *)vtomach(
				(vaddr_t)&ptes[i386_btop(pve->pv_va)]);
			PTE_ATOMIC_SET(&ptes[i386_btop(pve->pv_va)], maptp,
				(opte & ~(PG_U | PG_M)), opte);

			result |= (opte & ~clearbits);
			*myattrs |= (opte & ~(clearbits));

			pmap_tlb_shootdown(pve->pv_pmap, pve->pv_va, opte,
					&cpumask);
		}
no_tlb_shootdown:
		pmap_unmap_ptes(pve->pv_pmap);		/* unlocks pmap */
	}

	simple_unlock(&pvh->pvh_lock);
	PMAP_HEAD_TO_MAP_UNLOCK();
	pmap_tlb_shootnow(cpumask);

	return (result != 0);
}

/*
 * p m a p   p r o t e c t i o n   f u n c t i o n s
 */

/*
 * pmap_page_protect: change the protection of all recorded mappings
 *	of a managed page
 *
 * => NOTE: this is an inline function in pmap.h
 */

/* see pmap.h */

/*
 * pmap_protect: set the protection in of the pages in a pmap
 *
 * => NOTE: this is an inline function in pmap.h
 */

/* see pmap.h */

/*
 * pmap_write_protect: write-protect pages in a pmap
 */

void
pmap_write_protect(struct pmap *pmap, vaddr_t sva, vaddr_t eva, vm_prot_t prot)
{
	pt_entry_t *ptes, *spte, *epte, npte;
	pt_entry_t *maptp, opte;
	vaddr_t blockend;
	u_int32_t md_prot;
	int32_t cpumask = 0;

	ptes = pmap_map_ptes(pmap);		/* locks pmap */

	/* should be ok, but just in case ... */
	sva &= PG_FRAME;
	eva &= PG_FRAME;

	for (/* null */ ; sva < eva ; sva = blockend) {

		blockend = (sva & PD_MASK) + NBPD;
		if (blockend > eva)
			blockend = eva;

		/*
		 * XXXCDC: our PTE mappings should never be write-protected!
		 *
		 * long term solution is to move the PTEs out of user
		 * address space.  and into kernel address space (up
		 * with APTE).  then we can set VM_MAXUSER_ADDRESS to
		 * be VM_MAX_ADDRESS.
		 */

		/* XXXCDC: ugly hack to avoid freeing PDP here */
		if (pdei(sva) == PDSLOT_PTE)
			continue;

		/* empty block? */
		if (!pmap_valid_entry(pmap->pm_pdir[pdei(sva)]))
			continue;
#ifdef DIAGNOSTIC
		if (sva >= VM_MAXUSER_ADDRESS &&
		    sva < VM_MAX_ADDRESS)
			panic("pmap_write_protect: PTE space");
#endif

		md_prot = protection_codes[prot];
		if (sva < VM_MAXUSER_ADDRESS)
			md_prot |= PG_u;
		else if (sva < VM_MAX_ADDRESS)
			/* XXX: write-prot our PTES? never! */
			md_prot |= (PG_u | PG_RW);

		spte = &ptes[i386_btop(sva)];
		epte = &ptes[i386_btop(blockend)];

		for (/*null */; spte < epte ; spte++, sva += PAGE_SIZE) {

			if (!pmap_valid_entry(PTE_GET(spte))) /* no mapping? */
				continue;

			npte = (PTE_GET(spte) & ~PG_PROT) | md_prot;

			if (npte != PTE_GET(spte)) {
				pmap_exec_account(pmap, sva, PTE_GET(spte),
				    npte);
				maptp = (pt_entry_t *)vtomach((vaddr_t)spte);
				PTE_ATOMIC_SET(spte, maptp, npte, opte);
				pmap_tlb_shootdown(pmap, sva, PTE_GET(spte),
				    &cpumask);
			}
		}
	}

	pmap_tlb_shootnow(cpumask);
	pmap_unmap_ptes(pmap);		/* unlocks pmap */
}

/*
 * end of protection functions
 */

/*
 * pmap_unwire: clear the wired bit in the PTE
 *
 * => mapping should already be in map
 */

void
pmap_unwire(struct pmap *pmap, vaddr_t va)
{
	pt_entry_t *ptes;
	pt_entry_t *maptp;

	if (pmap_valid_entry(pmap->pm_pdir[pdei(va)])) {
		ptes = pmap_map_ptes(pmap);		/* locks pmap */

#ifdef DIAGNOSTIC
		if (!pmap_valid_entry(ptes[i386_btop(va)]))
			panic("pmap_unwire: invalid (unmapped) va 0x%lx", va);
#endif
		if ((ptes[i386_btop(va)] & PG_W) != 0) {
			maptp = (pt_entry_t *)vtomach((vaddr_t)
			    &ptes[i386_btop(va)]);
			PTE_ATOMIC_CLEARBITS(&ptes[i386_btop(va)], maptp, PG_W);
			pmap->pm_stats.wired_count--;
		}
#ifdef DIAGNOSTIC
		else {
			printf("pmap_unwire: wiring for pmap %p va 0x%lx "
			       "didn't change!\n", pmap, va);
		}
#endif
		pmap_unmap_ptes(pmap);		/* unlocks map */
	}
#ifdef DIAGNOSTIC
	else {
		panic("pmap_unwire: invalid PDE");
	}
#endif
}

/*
 * pmap_collect: free resources held by a pmap
 *
 * => optional function.
 * => called when a process is swapped out to free memory.
 */

void
pmap_collect(struct pmap *pmap)
{
	/*
	 * free all of the pt pages by removing the physical mappings
	 * for its entire address space.
	 */

	pmap_do_remove(pmap, VM_MIN_ADDRESS, VM_MAX_ADDRESS,
			PMAP_REMOVE_SKIPWIRED);
}

/*
 * pmap_copy: copy mappings from one pmap to another
 *
 * => optional function
 * void pmap_copy(dst_pmap, src_pmap, dst_addr, len, src_addr)
 */

/*
 * defined as macro in pmap.h
 */


/*
 * pmap_enter: enter a mapping into a pmap
 *
 * => must be done "now" ... no lazy-evaluation
 * => we set pmap => pv_head locking
 */

int
pmap_enter(struct pmap *pmap, vaddr_t va, paddr_t pa, vm_prot_t prot, int flags)
{
	paddr_t ma;

	if (__predict_false(pa < pmap_pa_start || pmap_pa_end <= pa)) {
		ma = pa; /* XXX hack */
	} else {
		ma = xpmap_ptom(pa);
	}

	return pmap_enter_ma(pmap, va, ma, pa, prot, flags, DOMID_SELF);
}

/*
 * pmap_enter_ma: enter a mapping into a pmap
 *
 * => must be done "now" ... no lazy-evaluation
 * => we set pmap => pv_head locking
 */

int
pmap_enter_ma(struct pmap *pmap, vaddr_t va, paddr_t ma, paddr_t pa,
	vm_prot_t prot, int flags, int domid)
{
	pt_entry_t *ptes, opte, npte;
	pt_entry_t *maptp;
	struct vm_page *ptp;
	struct pv_head *pvh;
	struct pv_entry *pve;
	int bank, off, error;
	boolean_t wired = (flags & PMAP_WIRED) != 0;

	/* Never change this to XENPRINTF, otherwise domU won't boot
	 * with XENDEBUG */
	XENPRINTK(("pmap_enter_ma(%p, %p %p %p, %08x, %08x)\n",
		pmap, (void *)va, (void *)ma, (void *)pa, prot, flags));
	KASSERT(domid == DOMID_SELF || pa == 0);

#ifdef DIAGNOSTIC
	/* sanity check: totally out of range? */
	if (va >= VM_MAX_KERNEL_ADDRESS)
		panic("pmap_enter_ma: too big");

	if (va == (vaddr_t) PDP_BASE || va == (vaddr_t) APDP_BASE)
		panic("pmap_enter_ma: trying to map over PDP/APDP!");

	/* sanity check: kernel PTPs should already have been pre-allocated */
	if (va >= VM_MIN_KERNEL_ADDRESS &&
	    !pmap_valid_entry(pmap->pm_pdir[pdei(va)]))
		panic("pmap_enter_ma: missing kernel PTP!");
#endif

	/* get lock */
	PMAP_MAP_TO_HEAD_LOCK();

	/*
	 * map in ptes and get a pointer to our PTP (unless we are the kernel)
	 */

	ptes = pmap_map_ptes(pmap);		/* locks pmap */
	if (pmap == pmap_kernel()) {
		ptp = NULL;
	} else {
		ptp = pmap_get_ptp(pmap, pdei(va), FALSE);
		if (ptp == NULL) {
			if (flags & PMAP_CANFAIL) {
				error = ENOMEM;
				goto out;
			}
			panic("pmap_enter_ma: get ptp failed");
		}
	}
	opte = pte_get_ma(&ptes[i386_btop(va)]);	/* old PTE */
	XENPRINTF(("pmap_enter_ma: npte %p opte %p ptes %p idx %03x\n",
		      (void *)npte, (void *)opte, ptes, i386_btop(va)));
	XENPRINTK(("pmap_enter_ma: ma %08lx pa %08lx va %08lx opte %08x npte %08x "
	    "wired %d count %ld\n", ma, pa, va, opte, npte, wired,
	    pmap->pm_stats.wired_count));

	/*
	 * is there currently a valid mapping at our VA?
	 */

	if (pmap_valid_entry(opte)) {

		/*
		 * first, update pm_stats.  resident count will not
		 * change since we are replacing/changing a valid
		 * mapping.  wired count might change...
		 */
		if (wired && (opte & PG_W) == 0)
			pmap->pm_stats.wired_count++;
		else if (!wired && (opte & PG_W) != 0)
			pmap->pm_stats.wired_count--;

		/*
		 * is the currently mapped MA the same as the one we
		 * want to map?
		 */

		if ((opte & PG_FRAME) == ma) {
			XENPRINTF(("pmap_enter_ma: pmap update opte == ma\n"));

			/* if this is on the PVLIST, sync R/M bit */
			if (opte & PG_PVLIST) {
				KASSERT(domid == DOMID_SELF);
				bank = vm_physseg_find(atop(pa), &off);
#ifdef DIAGNOSTIC
				if (bank == -1)
					panic("pmap_enter_ma: same pa PG_PVLIST"
					      " mapping with unmanaged page "
					      "pa = 0x%lx (0x%lx)", pa,
					      atop(pa));
#endif
				pvh = &vm_physmem[bank].pmseg.pvhead[off];
				simple_lock(&pvh->pvh_lock);
				vm_physmem[bank].pmseg.attrs[off] |= opte;
				simple_unlock(&pvh->pvh_lock);
			} else {
				pvh = NULL;	/* ensure !PG_PVLIST */
			}
			goto enter_now;
		}

		/*
		 * changing PAs: we must remove the old one first
		 */

		/*
		 * if current mapping is on a pvlist,
		 * remove it (sync R/M bits)
		 */

		if (opte & PG_PVLIST) {
			paddr_t opa;
			KASSERT(domid == DOMID_SELF);
			opa = xpmap_mtop(opte & PG_FRAME);
			bank = vm_physseg_find(atop(opa), &off);
#ifdef DIAGNOSTIC
			if (bank == -1)
				panic("pmap_enter_ma: PG_PVLIST mapping with "
				      "unmanaged page "
				      "pa = 0x%lx (0x%lx)", pa, atop(pa));
#endif
			pvh = &vm_physmem[bank].pmseg.pvhead[off];
			simple_lock(&pvh->pvh_lock);
			pve = pmap_remove_pv(pvh, pmap, va);
			vm_physmem[bank].pmseg.attrs[off] |= opte;
			simple_unlock(&pvh->pvh_lock);
		} else {
			pve = NULL;
		}
	} else {	/* opte not valid */
		pve = NULL;
		pmap->pm_stats.resident_count++;
		if (wired)
			pmap->pm_stats.wired_count++;
		if (ptp)
			ptp->wire_count++;      /* count # of valid entrys */
	}

	/*
	 * at this point pm_stats has been updated.   pve is either NULL
	 * or points to a now-free pv_entry structure (the latter case is
	 * if we called pmap_remove_pv above).
	 *
	 * if this entry is to be on a pvlist, enter it now.
	 */

	if (domid == DOMID_SELF) {
		bank = vm_physseg_find(atop(pa), &off);
	} else {
		bank = -1;
	}
	XENPRINTK(("pg %p from %p, init %d\n", PHYS_TO_VM_PAGE(pa), (void *) pa,
		pmap_initialized));
	if (pmap_initialized && bank != -1) {
		pvh = &vm_physmem[bank].pmseg.pvhead[off];
		if (pve == NULL) {
			/* We can not steal a pve - allocate one */
			pve = pmap_alloc_pv(pmap, ALLOCPV_NEED);
			if (pve == NULL) {
				if (flags & PMAP_CANFAIL) {
					error = ENOMEM;
					goto out;
				}
				panic("pmap_enter: no pv entries available");
			}
		}
		/* lock pvh when adding */
		pmap_enter_pv(pvh, pve, pmap, va, ptp);
	} else {

		/* new mapping is not PG_PVLIST.   free pve if we've got one */
		pvh = NULL;		/* ensure !PG_PVLIST */
		if (pve)
			pmap_free_pv(pmap, pve);
	}

enter_now:
	/*
	 * at this point pvh is !NULL if we want the PG_PVLIST bit set
	 */

	npte = protection_codes[prot] | PG_V | ma;

	XENPRINTK(("pmap_enter_ma: npte %p opte %p ptes %p idx %03x\n",
	    (void *)npte, (void *)opte, ptes, i386_btop(va)));

	pmap_exec_account(pmap, va, opte, npte);
	if (pvh)
		npte |= PG_PVLIST;
	if (wired)
		npte |= PG_W;
	if (va < VM_MAXUSER_ADDRESS)
		npte |= PG_u;
	else if (va < VM_MAX_ADDRESS)
		npte |= (PG_u | PG_RW);	/* XXXCDC: no longer needed? */
	if (pmap == pmap_kernel())
		npte |= pmap_pg_g;
	if (flags & VM_PROT_ALL) {
		npte |= PG_U;
		if (flags & VM_PROT_WRITE)
			npte |= PG_M;
	}

	/* zap! */
	maptp = (pt_entry_t *)vtomach((vaddr_t)&ptes[i386_btop(va)]);
	error = pte_atomic_update_ma_domid(&ptes[i386_btop(va)], npte,
	   &opte, domid);
	if (error) {
		goto out;
	}

	if ((opte & ~(PG_M|PG_U)) != npte) {
#ifdef MULTIPROCESSOR
		int32_t cpumask = 0;

		pmap_tlb_shootdown(pmap, va, opte, &cpumask);
		pmap_tlb_shootnow(cpumask);
#else
		/* Don't bother deferring in the single CPU case. */
		if (pmap_is_curpmap(pmap))
			pmap_update_pg(va);
#endif
	}

	error = 0;

out:
	pmap_unmap_ptes(pmap);
	PMAP_MAP_TO_HEAD_UNLOCK();

	XENPRINTK(("pmap_enter_ma: out: %d\n", error));
	return error;
}

/*
 * pmap_growkernel: increase usage of KVM space
 *
 * => we allocate new PTPs for the kernel and install them in all
 *	the pmaps on the system.
 */

vaddr_t
pmap_growkernel(vaddr_t maxkvaddr)
{
	struct pmap *kpm = pmap_kernel(), *pm;
	pd_entry_t *mapdp;
	pt_entry_t *maptp;
	int needed_kpde;   /* needed number of kernel PTPs */
	int s;
	paddr_t ptaddr;

	needed_kpde = (int)(maxkvaddr - VM_MIN_KERNEL_ADDRESS + (NBPD-1))
		/ NBPD;
	XENPRINTF(("pmap_growkernel %p: %d -> %d\n", (void *)maxkvaddr, nkpde,
	    needed_kpde));
	if (needed_kpde <= nkpde)
		goto out;		/* we are OK */

	/*
	 * whoops!   we need to add kernel PTPs
	 */

	s = splhigh();	/* to be safe */
	simple_lock(&kpm->pm_obj.vmobjlock);

	for (/*null*/ ; nkpde < needed_kpde ; nkpde++) {
		mapdp = (pt_entry_t *)vtomach((vaddr_t)
		    &kpm->pm_pdir[PDSLOT_KERN + nkpde]);

		if (uvm.page_init_done == FALSE) {

			/*
			 * we're growing the kernel pmap early (from
			 * uvm_pageboot_alloc()).  this case must be
			 * handled a little differently.
			 */

			if (uvm_page_physget(&ptaddr) == FALSE)
				panic("pmap_growkernel: out of memory");
			pmap_zero_phys(ptaddr);

			XENPRINTF(("xxxx maybe not PG_RW\n"));	/* XXX ? */
			PDE_SET(&kpm->pm_pdir[PDSLOT_KERN + nkpde], mapdp,
			    ptaddr | PG_RW | PG_V);

			/* count PTP as resident */
			kpm->pm_stats.resident_count++;
			continue;
		}

		/*
		 * THIS *MUST* BE CODED SO AS TO WORK IN THE
		 * pmap_initialized == FALSE CASE!  WE MAY BE
		 * INVOKED WHILE pmap_init() IS RUNNING!
		 */

		if (pmap_alloc_ptp(kpm, PDSLOT_KERN + nkpde, FALSE) == NULL) {
			panic("pmap_growkernel: alloc ptp failed");
		}

		/* PG_u not for kernel */
		PDE_CLEARBITS(&kpm->pm_pdir[PDSLOT_KERN + nkpde], mapdp, PG_u);

		/* distribute new kernel PTP to all active pmaps */
		simple_lock(&pmaps_lock);
		LIST_FOREACH(pm, &pmaps, pm_list) {
			maptp = (pt_entry_t *)vtomach(
			    (vaddr_t)&pm->pm_pdir[PDSLOT_KERN + nkpde]);
			PDE_COPY(&pm->pm_pdir[PDSLOT_KERN + nkpde], maptp,
			    &kpm->pm_pdir[PDSLOT_KERN + nkpde]);
		}
		simple_unlock(&pmaps_lock);
	}

	simple_unlock(&kpm->pm_obj.vmobjlock);
	splx(s);

out:
	XENPRINTF(("pmap_growkernel return %d %p\n", nkpde,
	    (void *)(VM_MIN_KERNEL_ADDRESS + (nkpde * NBPD))));
	return (VM_MIN_KERNEL_ADDRESS + (nkpde * NBPD));
}

#if defined(DEBUG) || defined(XENDEBUG)

/*
 * pmap_dump: dump all the mappings from a pmap
 *
 * => caller should not be holding any pmap locks
 */

void
pmap_dump(struct pmap *pmap, vaddr_t sva, vaddr_t eva)
{
	pt_entry_t *ptes, *pte;
	vaddr_t blkendva;

	/*
	 * if end is out of range truncate.
	 * if (end == start) update to max.
	 */

	if (eva > VM_MAXUSER_ADDRESS || eva <= sva)
		eva = VM_MAXUSER_ADDRESS;

	/*
	 * we lock in the pmap => pv_head direction
	 */

	PMAP_MAP_TO_HEAD_LOCK();
	ptes = pmap_map_ptes(pmap);	/* locks pmap */

	/*
	 * dumping a range of pages: we dump in PTP sized blocks (4MB)
	 */

	for (/* null */ ; sva < eva ; sva = blkendva) {

		/* determine range of block */
		blkendva = i386_round_pdr(sva+1);
		if (blkendva > eva)
			blkendva = eva;

		/* valid block? */
		if (!pmap_valid_entry(pmap->pm_pdir[pdei(sva)]))
			continue;

		pte = &ptes[i386_btop(sva)];
		for (/* null */; sva < blkendva ; sva += NBPG, pte++) {
			if (!pmap_valid_entry(*pte))
				continue;
			printf("va %#lx -> pa %#x (pte=%#x)\n",
			       sva, *pte, *pte & PG_FRAME);
		}
	}
	pmap_unmap_ptes(pmap);
	PMAP_MAP_TO_HEAD_UNLOCK();
}
#endif


/******************** TLB shootdown code ********************/

void
pmap_tlb_shootnow(int32_t cpumask)
{
#ifdef MULTIPROCESSOR
	struct cpu_info *ci, *self;
	CPU_INFO_ITERATOR cii;
	int s;
#ifdef DIAGNOSTIC
	int count = 0;
#endif
#endif

	if (cpumask == 0)
		return;

#ifdef MULTIPROCESSOR
	self = curcpu();
	s = splipi();
	self->ci_tlb_ipi_mask = cpumask;
#endif

	pmap_do_tlb_shootdown(0);	/* do *our* work. */

#ifdef MULTIPROCESSOR
	splx(s);

	if (cold)
		return;

	/*
	 * Send the TLB IPI to other CPUs pending shootdowns.
	 */
	CPU_INFO_FOREACH(cii, ci) {
		if (ci == self)
			continue;
		if (cpumask & (1U << ci->ci_cpuid))
			if (i386_send_ipi(ci, I386_IPI_TLB) != 0)
				i386_atomic_clearbits_l(&self->ci_tlb_ipi_mask,
				    (1U << ci->ci_cpuid));
	}

	while (self->ci_tlb_ipi_mask != 0)
#ifdef DIAGNOSTIC
		if (count++ > 100000000)
			panic("TLB IPI rendezvous failed (mask %x)",
			    self->ci_tlb_ipi_mask);
#else
		/* XXX insert pause instruction */
		;
#endif
#endif
}

/*
 * pmap_tlb_shootdown:
 *
 *	Cause the TLB entry for pmap/va to be shot down.
 */
void
pmap_tlb_shootdown(pmap_t pmap, vaddr_t va, pt_entry_t pte, int32_t *cpumaskp)
{
	struct cpu_info *ci, *self;
	struct pmap_tlb_shootdown_q *pq;
	struct pmap_tlb_shootdown_job *pj;
	CPU_INFO_ITERATOR cii;
	int s;

	if (pmap_initialized == FALSE) {
		pmap_update_pg(va);
		return;
	}

	self = curcpu();

	s = splipi();

	XENPRINTF(("pmap_tlb_shootdown: va %lx\n", va));

	CPU_INFO_FOREACH(cii, ci) {
		/* Note: we queue shootdown events for ourselves here! */
		if (pmap_is_active(pmap, ci->ci_cpuid) == 0)
			continue;
		if (ci != self && !(ci->ci_flags & CPUF_RUNNING))
			continue;
		pq = &pmap_tlb_shootdown_q[ci->ci_cpuid];
		mtx_enter(&pq->pq_mutex);

		/*
		 * If there's a global flush already queued, or a
		 * non-global flush, and this pte doesn't have the G
		 * bit set, don't bother.
		 */
		if (pq->pq_flushg > 0 ||
		    (pq->pq_flushu > 0 && (pte & pmap_pg_g) == 0)) {
			mtx_leave(&pq->pq_mutex);
			continue;
		}

#ifdef I386_CPU
		/*
		 * i386 CPUs can't invalidate a single VA, only
		 * flush the entire TLB, so don't bother allocating
		 * jobs for them -- just queue a `flushu'.
		 *
		 * XXX note that this can be executed for non-i386
		 * when called early (before identifycpu() has set
		 * cpu_class)
		 */
		if (cpu_class == CPUCLASS_386) {
			pq->pq_flushu++;
			*cpumaskp |= 1U << ci->ci_cpuid;
			mtx_leave(&pq->pq_mutex);
			continue;
		}
#endif

		pj = pmap_tlb_shootdown_job_get(pq);
		pq->pq_pte |= pte;
		if (pj == NULL) {
			/*
			 * Couldn't allocate a job entry.
			 * Kill it now for this cpu, unless the failure
			 * was due to too many pending flushes; otherwise,
			 * tell other cpus to kill everything..
			 */
			if (ci == self && pq->pq_count < PMAP_TLB_MAXJOBS) {
				pmap_update_pg(va);
				mtx_leave(&pq->pq_mutex);
				continue;
			} else {
				if (pq->pq_pte & pmap_pg_g)
					pq->pq_flushg++;
				else
					pq->pq_flushu++;
				/*
				 * Since we've nailed the whole thing,
				 * drain the job entries pending for that
				 * processor.
				 */
				pmap_tlb_shootdown_q_drain(pq);
				*cpumaskp |= 1U << ci->ci_cpuid;
			}
		} else {
			pj->pj_pmap = pmap;
			pj->pj_va = va;
			pj->pj_pte = pte;
			TAILQ_INSERT_TAIL(&pq->pq_head, pj, pj_list);
			*cpumaskp |= 1U << ci->ci_cpuid;
		}
		mtx_leave(&pq->pq_mutex);
	}
	splx(s);
}

/*
 * pmap_do_tlb_shootdown:
 *
 *	Process pending TLB shootdown operations for this processor.
 */
void
pmap_do_tlb_shootdown(struct cpu_info *self)
{
	u_long cpu_id = cpu_number();
	struct pmap_tlb_shootdown_q *pq = &pmap_tlb_shootdown_q[cpu_id];
	struct pmap_tlb_shootdown_job *pj;
#ifdef MULTIPROCESSOR
	struct cpu_info *ci;
	CPU_INFO_ITERATOR cii;
#endif

	mtx_enter(&pq->pq_mutex);

	if (pq->pq_flushg) {
		tlbflushg();
		pq->pq_flushg = 0;
		pq->pq_flushu = 0;
		pmap_tlb_shootdown_q_drain(pq);
	} else {
		/*
		 * TLB flushes for PTEs with PG_G set may be in the queue
		 * after a flushu, they need to be dealt with.
		 */
		if (pq->pq_flushu) {
			tlbflush();
		}
		while ((pj = TAILQ_FIRST(&pq->pq_head)) != NULL) {
			TAILQ_REMOVE(&pq->pq_head, pj, pj_list);

			if ((!pq->pq_flushu && pmap_is_curpmap(pj->pj_pmap)) ||
			    (pj->pj_pte & pmap_pg_g))
				pmap_update_pg(pj->pj_va);

			pmap_tlb_shootdown_job_put(pq, pj);
		}

		pq->pq_flushu = pq->pq_pte = 0;
	}

#ifdef MULTIPROCESSOR
	CPU_INFO_FOREACH(cii, ci)
		i386_atomic_clearbits_l(&ci->ci_tlb_ipi_mask,
		    (1U << cpu_id));
#endif
	mtx_leave(&pq->pq_mutex);
}

/*
 * pmap_tlb_shootdown_q_drain:
 *
 *	Drain a processor's TLB shootdown queue.  We do not perform
 *	the shootdown operations.  This is merely a convenience
 *	function.
 *
 *	Note: We expect the queue to be locked.
 */
void
pmap_tlb_shootdown_q_drain(struct pmap_tlb_shootdown_q *pq)
{
	struct pmap_tlb_shootdown_job *pj;

	while ((pj = TAILQ_FIRST(&pq->pq_head)) != NULL) {
		TAILQ_REMOVE(&pq->pq_head, pj, pj_list);
		pmap_tlb_shootdown_job_put(pq, pj);
	}
	pq->pq_pte = 0;
}

/*
 * pmap_tlb_shootdown_job_get:
 *
 *	Get a TLB shootdown job queue entry.  This places a limit on
 *	the number of outstanding jobs a processor may have.
 *
 *	Note: We expect the queue to be locked.
 */
struct pmap_tlb_shootdown_job *
pmap_tlb_shootdown_job_get(struct pmap_tlb_shootdown_q *pq)
{
	struct pmap_tlb_shootdown_job *pj;

	if (pq->pq_count >= PMAP_TLB_MAXJOBS)
		return (NULL);

	mtx_enter(&pmap_tlb_shootdown_job_mutex);
	if (pj_free == NULL) {
		mtx_leave(&pmap_tlb_shootdown_job_mutex);
		return NULL;
	}
	pj = pj_free;
	pj_free = pj_free->pj_nextfree;
	mtx_leave(&pmap_tlb_shootdown_job_mutex);

	pq->pq_count++;
	return (pj);
}

/*
 * pmap_tlb_shootdown_job_put:
 *
 *	Put a TLB shootdown job queue entry onto the free list.
 *
 *	Note: We expect the queue to be locked.
 */
void
pmap_tlb_shootdown_job_put(struct pmap_tlb_shootdown_q *pq,
			struct pmap_tlb_shootdown_job *pj)
{
#ifdef DIAGNOSTIC
	if (pq->pq_count == 0)
		panic("pmap_tlb_shootdown_job_put: queue length inconsistency");
#endif
	mtx_enter(&pmap_tlb_shootdown_job_mutex);
	pj->pj_nextfree = pj_free;
	pj_free = pj;
	mtx_leave(&pmap_tlb_shootdown_job_mutex);

	pq->pq_count--;
}
