/* $NetBSD: xbd.c,v 1.24 2005/10/18 00:14:43 yamt Exp $ */

/*
 *
 * Copyright (c) 2004 Christian Limpach.
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
 *      This product includes software developed by Christian Limpach.
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

#include <sys/types.h>
#include <sys/param.h>
#include <sys/systm.h>
#include <uvm/uvm.h>
#include <machine/uvm_km_kmemalloc.h>

/*
 * XXX move uvm_km_kmemalloc1() to src/sys/uvm/uvm_km.c and replace
 * uvm_km_kmemalloc() with
 *  #define uvm_km_kmemalloc(map, obj, size, flags)	\
 *	uvm_km_kmemalloc1(map, obj, 0, UVM_UNKNOWN_OFFSET, flags)
 *
 * uvm_km_kmemalloc1() is needed to meet the alignment requirements
 *   for the ring buffer. It only differs in the two align and prefer
 *   arguments which are just passed through to uvm_map().
 */

vaddr_t
uvm_km_kmemalloc1(struct vm_map *map, struct uvm_object *obj,
		vsize_t size, vsize_t align, voff_t prefer, int flags)
{
	vaddr_t kva, loopva;
	vaddr_t offset;
	size_t loopsize;
	struct vm_page *pg;
	UVMHIST_FUNC("uvm_km_kmemalloc1"); UVMHIST_CALLED(maphist);

	UVMHIST_LOG(maphist,"  (map=0x%x, obj=0x%x, size=0x%x, flags=%d)",
		map, obj, size, flags);
	KASSERT(vm_map_pmap(map) == pmap_kernel());

	/*
	 * setup for call
	 */

	size = round_page(size);
	kva = vm_map_min(map);	/* hint */

	/*
	 * allocate some virtual space
	 */
	if (__predict_false(uvm_map(map, &kva, size, obj, prefer, align,
		UVM_MAPFLAG(UVM_PROT_RW, UVM_PROT_RW, UVM_INH_NONE,
			UVM_ADV_RANDOM, (flags & UVM_KMF_TRYLOCK)))
			!= 0)) {
		UVMHIST_LOG(maphist, "<- done (no VM)", 0,0,0,0);
		return (0);
	}

	/*
	 * if all we wanted was VA, return now
	 */

	if (flags & UVM_KMF_VALLOC) {
		UVMHIST_LOG(maphist, "<- done valloc (kva=0x%x)", kva,0,0,0);
		return (kva);
	}

	/*
	 * recover object offset from virtual address
	 */

	if (obj != NULL)
		offset = kva - vm_map_min(kernel_map);
	else
		offset = 0;

	UVMHIST_LOG(maphist, "  kva=0x%x, offset=%x", kva, offset,0,0);

	/*
	 * now allocate and map in the memory... note that we are the only ones
	 * whom should ever get a handle on this area of VM.
	 */

	loopva = kva;
	loopsize = size;
	while (loopsize) {
		if (obj != NULL) {
			simple_lock(&obj->vmobjlock);
		}
		pg = uvm_pagealloc(obj, offset, NULL, 0);
		if (pg) {
			atomic_clearbits_int(&pg->pg_flags, PG_BUSY);
			UVM_PAGE_OWN(pg, NULL);
		}
		if (obj != NULL) {
			simple_unlock(&obj->vmobjlock);
		}

		/*
		 * out of memory?
		 */

		if (__predict_false(pg == NULL)) {
			if ((flags & UVM_KMF_NOWAIT) ||
			    ((flags & UVM_KMF_CANFAIL) &&
			    uvmexp.swpgonly == uvmexp.swpages)) {
				/* free everything! */
				uvm_unmap(map, kva, kva + size);
				return (0);
			} else {
				uvm_wait("km_getwait2");	/* sleep here */
				continue;
			}
		}

		/* map it in: note that we call pmap_enter with the map and
		 * object unlocked in case we are kmem_map/kmem_object
		 * (because if pmap_enter wants to allocate out of kmem_object
		 * it will need to lock it itself!)
		 */

		if (obj == NULL) {
			pmap_kenter_pa(loopva, VM_PAGE_TO_PHYS(pg),
				UVM_PROT_RW);
		} else {
			pmap_enter(map->pmap, loopva, VM_PAGE_TO_PHYS(pg),
				UVM_PROT_RW,
				PMAP_WIRED | VM_PROT_READ | VM_PROT_WRITE);
		}
		loopva += PAGE_SIZE;
		offset += PAGE_SIZE;
		loopsize -= PAGE_SIZE;
	}
	pmap_update(pmap_kernel());

	UVMHIST_LOG(maphist, "<- done (kva=0x%x)", kva, 0,0,0);
	return (kva);
}
