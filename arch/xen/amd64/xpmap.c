/*
 * Copyright (c) 2006 Mathieu Ropert <mro@adviseo.fr>
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#include <sys/types.h>
#include <sys/param.h>

#include <uvm/uvm.h>

#include <machine/pmap.h>
#include <machine/pte.h>
#include <machine/xen-public/xen.h>
#include <machine/hypervisor.h>
#include <machine/hypercall.h>
#include <machine/xen.h>
#include <machine/cpufunc.h>

static void xen_bt_set_readonly (vaddr_t);
static void xen_bootstrap_tables (vaddr_t, vaddr_t, int, int);

static void xpq_dump (void);

/* How many PDEs ? */
#define TABLE_L2_ENTRIES (NKL2_KIMG_ENTRIES + 1)

/*
 * Function to get rid of Xen bootstrap tables
 */

vaddr_t xen_arch_pmap_bootstrap(vaddr_t first_avail)
{
	int count;
	vaddr_t bootstrap_tables;

	/* XXX: using old xen_start_info seems broken, use Xen PGD instead */
	first_avail = xen_start_info.pt_base;

	/* Space after Xen boostrap tables should be free */
	bootstrap_tables = xen_start_info.pt_base +
		(xen_start_info.nr_pt_frames * PAGE_SIZE);

	/* Calculate how many tables we need */
	count = TABLE_L2_ENTRIES;

	/* 
	 * Xen space we'll reclaim may not be enough for our new page tables,
	 * move bootstrap tables if necessary
	 */

	if (bootstrap_tables < first_avail + ((count + 3) * PAGE_SIZE))
		bootstrap_tables = first_avail + ((count + 3) * PAGE_SIZE);

	/* Create temporary tables */
	xen_bootstrap_tables(xen_start_info.pt_base, bootstrap_tables,
		xen_start_info.nr_pt_frames, count);

	/* Create final tables */
	xen_bootstrap_tables(bootstrap_tables, first_avail, count + 3, count);

	return (first_avail + ((count + 3) * PAGE_SIZE));
}


/*
 * Build a new table and switch to it
 * old_count is # of old tables (including PGD, PDTPE and PDE)
 * new_count is # of new tables (PTE only)
 * we assume areas don't overlap
 */


static void xen_bootstrap_tables (vaddr_t old_pgd, vaddr_t new_pgd,
	int old_count, int new_count)
{
	pd_entry_t *pdtpe, *pde, *pte;
	pd_entry_t *cur_pgd, *bt_pgd;
	paddr_t addr, page;
	vaddr_t avail;
	int i;

	/* 
	 * Create bootstrap page tables
	 * What we need:
	 * - a PGD (level 4)
	 * - a PDTPE (level 3)
	 * - a PDE (level2)
	 * - some PTEs (level 1)
	 */
	
	cur_pgd = (pd_entry_t *) old_pgd;
	bt_pgd = (pd_entry_t *) new_pgd;
	memset (bt_pgd, 0, PAGE_SIZE);
	avail = new_pgd + PAGE_SIZE;

	/* Install level 3 */
	pdtpe = (pd_entry_t *) avail;
	memset (pdtpe, 0, PAGE_SIZE);
	avail += PAGE_SIZE;

	addr = ((paddr_t) pdtpe) - KERNBASE;
	bt_pgd[pl4_pi(KERNTEXTOFF)] = phys_to_mach(addr) | PG_u | PG_RW | PG_V;

	/* Level 2 */
	pde = (pd_entry_t *) avail;
	memset(pde, 0, PAGE_SIZE);
	avail += PAGE_SIZE;

	addr = ((paddr_t) pde) - KERNBASE;
	pdtpe[pl3_pi(KERNTEXTOFF)] = phys_to_mach(addr) | PG_u | PG_RW | PG_V;

	/* Level 1 */
	page = KERNTEXTOFF;
	for (i = 0; i < new_count; i ++) {
		paddr_t cur_page = page;

		pte = (pd_entry_t *) avail;
		avail += PAGE_SIZE;

		memset(pte, 0, PAGE_SIZE);
		while (pl2_pi(page) == pl2_pi (cur_page)) {
			pte[pl1_pi(page)] = phys_to_mach(page - KERNBASE)
					| PG_u | PG_V;
			/* Not a PTP (old or new)? then RW */
			if (!((page >= old_pgd
				&& page < old_pgd + (old_count * PAGE_SIZE))
				|| (page >= new_pgd && page < new_pgd
					+ ((new_count + 3) * PAGE_SIZE))))
				pte[pl1_pi(page)] |= PG_RW;
			page += PAGE_SIZE;
		}

		addr = ((paddr_t) pte) - KERNBASE;
		pde[pl2_pi(cur_page)] = phys_to_mach(addr)
					| PG_u | PG_RW | PG_V;
		/* Mark readonly */
		xen_bt_set_readonly((vaddr_t) pte);
	}

	/* Install recursive page tables mapping */
	bt_pgd[PDIR_SLOT_PTE] = phys_to_mach(new_pgd - KERNBASE) | PG_u | PG_V;

	/* Mark tables RO */
	xen_bt_set_readonly((vaddr_t) pde);
	xen_bt_set_readonly((vaddr_t) pdtpe);
	xen_bt_set_readonly(new_pgd);
	/* Pin the PGD */
	xen_pgd_pin(new_pgd - KERNBASE);
	/* Switch to new tables */
	xen_set_pgd(new_pgd - KERNBASE);

	/* Now we can safely reclaim space taken by old tables */
	
	/* Unpin old PGD */
	xen_pgd_unpin(old_pgd - KERNBASE);
	/* Mark old tables RW */
	page = old_pgd;
	addr = (paddr_t) pde[pl2_pi(page)] & PG_FRAME;
	addr = mach_to_phys(addr);
	pte = (pd_entry_t *) (addr + KERNBASE);
	pte += pl1_pi(page);
	while (page < old_pgd + (old_count * PAGE_SIZE)) {
		addr = ((paddr_t) pte) - KERNBASE;
		addr = phys_to_mach(addr) + (addr & PAGE_MASK);
		xpq_queue_pt_update((pt_entry_t *) addr, *pte | PG_RW);
		page += PAGE_SIZE;
		/* 
		 * Our ptes are contiguous
		 * so it's safe to just "++" here
		 */
		pte++;
	}
	xpq_flush_queue();
}


void xen_pgd_pin(paddr_t page)
{
        struct mmuext_op op;
        op.cmd = MMUEXT_PIN_L4_TABLE;
        op.arg1.mfn = pfn_to_mfn(page >> PAGE_SHIFT);
        HYPERVISOR_mmuext_op(&op, 1, NULL, DOMID_SELF);
}

void xen_pgd_unpin(paddr_t page)
{
        struct mmuext_op op;
        op.cmd = MMUEXT_UNPIN_TABLE;
        op.arg1.mfn = pfn_to_mfn(page >> PAGE_SHIFT);
        HYPERVISOR_mmuext_op(&op, 1, NULL, DOMID_SELF);
}

void xen_set_pgd(paddr_t page)
{
        struct mmuext_op op;
        op.cmd = MMUEXT_NEW_BASEPTR;
        op.arg1.mfn = pfn_to_mfn(page >> PAGE_SHIFT);
        if (HYPERVISOR_mmuext_op(&op, 1, NULL, DOMID_SELF) < 0)
		panic("xen_set_pgd: failed to install new page directory %x",
			page);
}

void xen_set_user_pgd(paddr_t page)
{
	struct mmuext_op op;
	op.cmd = MMUEXT_NEW_USER_BASEPTR;
	op.arg1.mfn = pfn_to_mfn(page >> PAGE_SHIFT);
        if (HYPERVISOR_mmuext_op(&op, 1, NULL, DOMID_SELF) < 0)
		panic("xen_set_user_pgd: failed to install new user page"
			" directory %x", page);
}

/*
 * Bootstrap helper functions
 */

/*
 * Mark a page readonly
 * XXX: assuming vaddr = paddr + KERNBASE
 */

static void xen_bt_set_readonly (vaddr_t page)
{
	pt_entry_t entry;

	entry = phys_to_mach(page - KERNBASE);
	entry |= PG_u | PG_V;

	HYPERVISOR_update_va_mapping (page, entry, UVMF_INVLPG);
}


/*
 * Xen pmap helper functions
 */

/*
 * MMU update ops queues, ported from NetBSD/xen (i386)
 */


#ifndef XPQUEUE_SIZE
#define XPQUEUE_SIZE 2048
#endif

static mmu_update_t xpq_queue[XPQUEUE_SIZE];
static int xpq_idx = 0;

void
xpq_flush_queue()
{
        int ok;

        if (xpq_idx != 0 &&
            HYPERVISOR_mmu_update(xpq_queue, xpq_idx, &ok, DOMID_SELF) < 0) {
		xpq_dump();
                panic("xpq_flush_queue(): HYPERVISOR_mmu_update failed\n");
	}

        xpq_idx = 0;
}

static inline void
xpq_increment_idx(void)
{
        xpq_idx++;
        if (xpq_idx == XPQUEUE_SIZE)
                xpq_flush_queue();
}

void
xpq_queue_pt_update(pt_entry_t *ptr, pt_entry_t val)
{
        xpq_queue[xpq_idx].ptr = (paddr_t)ptr | MMU_NORMAL_PT_UPDATE;
        xpq_queue[xpq_idx].val = val;
        xpq_increment_idx();
}

void
xpq_queue_tlb_flush()
{
        struct mmuext_op op;
        xpq_flush_queue();

        op.cmd = MMUEXT_TLB_FLUSH_LOCAL;
        if (HYPERVISOR_mmuext_op(&op, 1, NULL, DOMID_SELF) < 0)
                panic("xpq_queue_tlb_flush(): local TLB flush failed");
}

void
xpq_flush_cache()
{
        struct mmuext_op op;
        int s = splvm();
        xpq_flush_queue();

        op.cmd = MMUEXT_FLUSH_CACHE;
        if (HYPERVISOR_mmuext_op(&op, 1, NULL, DOMID_SELF) < 0)
                panic("xpq_flush_cache(): cache flush failed");
        splx(s);
}

void
xpq_queue_invlpg(vaddr_t va)
{
        struct mmuext_op op;
        xpq_flush_queue();

        op.cmd = MMUEXT_INVLPG_LOCAL;
        op.arg1.linear_addr = (va & PG_FRAME);
        if (HYPERVISOR_mmuext_op(&op, 1, NULL, DOMID_SELF) < 0) {
		printf("xpq_queue_invlpg(): unable to invalidate va %p\n", va);
                panic("xpq_queue_invlpg(): tlb invalidation failed");
	}
}

static void
xpq_dump(void)
{
	int i;

	printf("<<< xpq queue dump >>>\n");

	for (i = 0; i < xpq_idx; i++)
		printf("%d: ptr %p, val %p (pa %p ma %p va %p)\n", i, 
			xpq_queue[i].ptr, xpq_queue[i].val,
			mach_to_phys(xpq_queue[i].val & PG_FRAME),
			xpq_queue[i].val & PG_FRAME,
			PMAP_DIRECT_MAP(mach_to_phys(
				xpq_queue[i].val & PG_FRAME)));

	printf("<<< end of xpq queue dump >>>\n");
}



