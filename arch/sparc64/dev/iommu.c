/*	$OpenBSD: iommu.c,v 1.17 2002/05/13 19:43:48 jason Exp $	*/
/*	$NetBSD: iommu.c,v 1.47 2002/02/08 20:03:45 eeh Exp $	*/

/*
 * Copyright (c) 2001, 2002 Eduardo Horvath
 * Copyright (c) 1999, 2000 Matthew R. Green
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
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

/*
 * UltraSPARC IOMMU support; used by both the sbus and pci code.
 */
#include <sys/param.h>
#include <sys/extent.h>
#include <sys/malloc.h>
#include <sys/systm.h>
#include <sys/device.h>

#include <uvm/uvm_extern.h>

#include <machine/bus.h>
#include <sparc64/sparc64/cache.h>
#include <sparc64/dev/iommureg.h>
#include <sparc64/dev/iommuvar.h>

#include <machine/autoconf.h>
#include <machine/cpu.h>

#ifdef DDB
#include <machine/db_machdep.h>
#include <ddb/db_sym.h>
#include <ddb/db_extern.h>
#endif

#ifdef DEBUG
#define IDB_BUSDMA	0x1
#define IDB_IOMMU	0x2
#define IDB_INFO	0x4
#define	IDB_SYNC	0x8
int iommudebug = 0x0;
#define DPRINTF(l, s)   do { if (iommudebug & l) printf s; } while (0)
#else
#define DPRINTF(l, s)
#endif

int iommu_dvmamap_sync_seg(bus_dma_tag_t, struct iommu_state *,
    bus_dma_segment_t *, bus_addr_t, bus_size_t, int);

#define iommu_strbuf_flush(i,v) do {				\
	if ((i)->is_sb[0])					\
		bus_space_write_8((i)->is_bustag,		\
			(bus_space_handle_t)(u_long)		\
			&(i)->is_sb[0]->strbuf_pgflush,		\
			0, (v));				\
	if ((i)->is_sb[1])					\
		bus_space_write_8((i)->is_bustag,		\
			(bus_space_handle_t)(u_long)		\
			&(i)->is_sb[1]->strbuf_pgflush,		\
			0, (v));				\
	} while (0)

static	int iommu_strbuf_flush_done(struct iommu_state *);
int64_t iommu_tsb_entry(struct iommu_state *, vaddr_t);
static	int iommu_tv_comp(struct timeval *, struct timeval *);

/*
 * initialise the UltraSPARC IOMMU (SBUS or PCI):
 *	- allocate and setup the iotsb.
 *	- enable the IOMMU
 *	- initialise the streaming buffers (if they exist)
 *	- create a private DVMA map.
 */
void
iommu_init(name, is, tsbsize, iovabase)
	char *name;
	struct iommu_state *is;
	int tsbsize;
	u_int32_t iovabase;
{
	psize_t size;
	vaddr_t va;
	paddr_t pa;
	struct vm_page *m;
	struct pglist mlist;

	/*
	 * Setup the iommu.
	 *
	 * The sun4u iommu is part of the SBUS or PCI controller so we will
	 * deal with it here..
	 *
	 * For sysio and psycho/psycho+ the IOMMU address space always ends at
	 * 0xffffe000, but the starting address depends on the size of the
	 * map.  The map size is 1024 * 2 ^ is->is_tsbsize entries, where each
	 * entry is 8 bytes.  The start of the map can be calculated by
	 * (0xffffe000 << (8 + is->is_tsbsize)).
	 *
	 * But sabre and hummingbird use a different scheme that seems to
	 * be hard-wired, so we read the start and size from the PROM and
	 * just use those values.
	 */
	is->is_cr = (tsbsize << 16) | IOMMUCR_EN;
	is->is_tsbsize = tsbsize;
	if (iovabase == -1) {
		is->is_dvmabase = IOTSB_VSTART(is->is_tsbsize);
		is->is_dvmaend = IOTSB_VEND;
	} else {
		is->is_dvmabase = iovabase;
		is->is_dvmaend = iovabase + IOTSB_VSIZE(tsbsize);
	}

	/*
	 * Allocate memory for I/O pagetables.  They need to be physically
	 * contiguous.
	 */

	size = NBPG<<(is->is_tsbsize);
	TAILQ_INIT(&mlist);
	if (uvm_pglistalloc((psize_t)size, (paddr_t)0, (paddr_t)-1,
		(paddr_t)NBPG, (paddr_t)0, &mlist, 1, 0) != 0)
		panic("iommu_init: no memory");

	va = uvm_km_valloc(kernel_map, size);
	if (va == 0)
		panic("iommu_init: no memory");
	is->is_tsb = (int64_t *)va;

	m = TAILQ_FIRST(&mlist);
	is->is_ptsb = VM_PAGE_TO_PHYS(m);

	/* Map the pages */
	for (; m != NULL; m = TAILQ_NEXT(m,pageq)) {
		pa = VM_PAGE_TO_PHYS(m);
		pmap_enter(pmap_kernel(), va, pa | PMAP_NVC,
			VM_PROT_READ|VM_PROT_WRITE,
			VM_PROT_READ|VM_PROT_WRITE|PMAP_WIRED);
		va += NBPG;
	}
	pmap_update(pmap_kernel());
	bzero(is->is_tsb, size);

#ifdef DEBUG
	if (iommudebug & IDB_INFO)
	{
		/* Probe the iommu */
		struct iommureg *regs = is->is_iommu;

		printf("iommu regs at: cr=%lx tsb=%lx flush=%lx\n",
		    (u_long)&regs->iommu_cr,
		    (u_long)&regs->iommu_tsb,
		    (u_long)&regs->iommu_flush);
		printf("iommu cr=%llx tsb=%llx\n", (unsigned long long)regs->iommu_cr, (unsigned long long)regs->iommu_tsb);
		printf("TSB base %p phys %llx\n", (void *)is->is_tsb, (unsigned long long)is->is_ptsb);
		delay(1000000); /* 1 s */
	}
#endif

	/*
	 * Initialize streaming buffer, if it is there.
	 */
	if (is->is_sb[0] || is->is_sb[1])
		(void)pmap_extract(pmap_kernel(), (vaddr_t)&is->is_flush[0],
		    &is->is_flushpa);

	/*
	 * now actually start up the IOMMU
	 */
	iommu_reset(is);

	/*
	 * Now all the hardware's working we need to allocate a dvma map.
	 */
	printf("IOTDB: %llx to %llx\n", 
	    (unsigned long long)is->is_ptsb,
	    (unsigned long long)(is->is_ptsb + size));
	is->is_dvmamap = extent_create(name,
	    is->is_dvmabase, is->is_dvmaend - NBPG,
	    M_DEVBUF, 0, 0, EX_NOWAIT);
}

/*
 * Streaming buffers don't exist on the UltraSPARC IIi; we should have
 * detected that already and disabled them.  If not, we will notice that
 * they aren't there when the STRBUF_EN bit does not remain.
 */
void
iommu_reset(is)
	struct iommu_state *is;
{
	struct iommu_strbuf *sb;
	int i;

	/* Need to do 64-bit stores */
	bus_space_write_8(is->is_bustag, 
			  (bus_space_handle_t)(u_long)&is->is_iommu->iommu_tsb, 
			  0, is->is_ptsb);
	/* Enable IOMMU in diagnostic mode */
	bus_space_write_8(is->is_bustag, 
			  (bus_space_handle_t)(u_long)&is->is_iommu->iommu_cr, 0, 
			  is->is_cr|IOMMUCR_DE);

	for (i=0; i<2; i++) {
		if ((sb = is->is_sb[i]) != NULL) {
			/* Enable diagnostics mode? */
			bus_space_write_8(is->is_bustag,
			    (bus_space_handle_t)(u_long)&sb->strbuf_ctl,
			    0, STRBUF_EN);

			/* No streaming buffers? Disable them */
			if (bus_space_read_8(is->is_bustag,
			    (bus_space_handle_t)(u_long)&sb->strbuf_ctl,
				0) == 0)
				is->is_sb[i] = 0;
		}
	}
}

/*
 * Here are the iommu control routines. 
 */
void
iommu_enter(is, va, pa, flags)
	struct iommu_state *is;
	vaddr_t va;
	int64_t pa;
	int flags;
{
	int64_t tte;

#ifdef DIAGNOSTIC
	if (va < is->is_dvmabase || va > is->is_dvmaend)
		panic("iommu_enter: va %#lx not in DVMA space", va);
#endif

	tte = MAKEIOTTE(pa, !(flags&BUS_DMA_NOWRITE), !(flags&BUS_DMA_NOCACHE), 
			(flags&BUS_DMA_STREAMING));
tte |= (flags & 0xff000LL)<<(4*8);/* DEBUG */

	/* Is the streamcache flush really needed? */
	if (is->is_sb[0] || is->is_sb[1]) {
		iommu_strbuf_flush(is, va);
		iommu_strbuf_flush_done(is);
	}
	DPRINTF(IDB_IOMMU, ("Clearing TSB slot %d for va %p\n", 
		       (int)IOTSBSLOT(va,is->is_tsbsize), (void *)(u_long)va));
	is->is_tsb[IOTSBSLOT(va,is->is_tsbsize)] = tte;
	bus_space_write_8(is->is_bustag, (bus_space_handle_t)(u_long)
			  &is->is_iommu->iommu_flush, 0, va);
	DPRINTF(IDB_IOMMU, ("iommu_enter: va %lx pa %lx TSB[%lx]@%p=%lx\n",
		       va, (long)pa, (u_long)IOTSBSLOT(va,is->is_tsbsize), 
		       (void *)(u_long)&is->is_tsb[IOTSBSLOT(va,is->is_tsbsize)],
		       (u_long)tte));
}


/*
 * Find the value of a DVMA address (debug routine).
 */
paddr_t
iommu_extract(is, dva)
	struct iommu_state *is;
	vaddr_t dva;
{
	int64_t tte = 0;
	
	if (dva >= is->is_dvmabase && dva < is->is_dvmaend)
		tte = is->is_tsb[IOTSBSLOT(dva,is->is_tsbsize)];

	if ((tte&IOTTE_V) == 0)
		return ((paddr_t)-1L);
	return (tte&IOTTE_PAMASK);
}

/*
 * Fetch a tsb entry with some sanity checking.
 */
int64_t
iommu_tsb_entry(is, dva)
	struct iommu_state *is;
	vaddr_t dva;
{
	int64_t tte;

	if (dva < is->is_dvmabase && dva >= is->is_dvmaend)
		panic("invalid dva: %llx", (long long)dva);

	tte = is->is_tsb[IOTSBSLOT(dva,is->is_tsbsize)];

	if ((tte & IOTTE_V) == 0)
		panic("iommu_tsb_entry: invalid entry %llx\n", (long long)dva);

	return (tte);
}

/*
 * iommu_remove: removes mappings created by iommu_enter
 *
 * Only demap from IOMMU if flag is set.
 *
 * XXX: this function needs better internal error checking.
 */
void
iommu_remove(is, va, len)
	struct iommu_state *is;
	vaddr_t va;
	size_t len;
{
#ifdef DIAGNOSTIC
	if (va < is->is_dvmabase || va > is->is_dvmaend)
		panic("iommu_remove: va 0x%lx not in DVMA space", (u_long)va);
	if ((long)(va + len) < (long)va)
		panic("iommu_remove: va 0x%lx + len 0x%lx wraps", 
		      (long) va, (long) len);
	if (len & ~0xfffffff) 
		panic("iommu_remove: rediculous len 0x%lx", (u_long)len);
#endif

	va = trunc_page(va);
	DPRINTF(IDB_IOMMU, ("iommu_remove: va %lx TSB[%lx]@%p\n",
	    va, (u_long)IOTSBSLOT(va,is->is_tsbsize), 
	    &is->is_tsb[IOTSBSLOT(va,is->is_tsbsize)]));
	while (len > 0) {
		DPRINTF(IDB_IOMMU, ("iommu_remove: clearing TSB slot %d for va %p size %lx\n", 
		    (int)IOTSBSLOT(va,is->is_tsbsize), (void *)(u_long)va, (u_long)len));
		if (is->is_sb[0] || is->is_sb[0]) {
			DPRINTF(IDB_IOMMU, ("iommu_remove: flushing va %p TSB[%lx]@%p=%lx, %lu bytes left\n", 	       
			       (void *)(u_long)va, (long)IOTSBSLOT(va,is->is_tsbsize), 
			       (void *)(u_long)&is->is_tsb[IOTSBSLOT(va,is->is_tsbsize)],
			       (long)(is->is_tsb[IOTSBSLOT(va,is->is_tsbsize)]), 
			       (u_long)len));
			iommu_strbuf_flush(is, va);
			if (len <= NBPG)
				iommu_strbuf_flush_done(is);
			DPRINTF(IDB_IOMMU, ("iommu_remove: flushed va %p TSB[%lx]@%p=%lx, %lu bytes left\n", 	       
			       (void *)(u_long)va, (long)IOTSBSLOT(va,is->is_tsbsize), 
			       (void *)(u_long)&is->is_tsb[IOTSBSLOT(va,is->is_tsbsize)],
			       (long)(is->is_tsb[IOTSBSLOT(va,is->is_tsbsize)]), 
			       (u_long)len));
		}

		if (len <= NBPG)
			len = 0;
		else
			len -= NBPG;

		/* XXX Zero-ing the entry would not require RMW */
		is->is_tsb[IOTSBSLOT(va,is->is_tsbsize)] &= ~IOTTE_V;
		bus_space_write_8(is->is_bustag, (bus_space_handle_t)(u_long)
				  &is->is_iommu->iommu_flush, 0, va);
		va += NBPG;
	}
}

static int
iommu_tv_comp(t1, t2)
	struct timeval *t1, *t2;
{
	if (t1->tv_sec < t2->tv_sec)
		return (-1);
	if (t1->tv_sec > t2->tv_sec)
		return (1);
	/* t1->tv_sec == t2->tv_sec */
	if (t1->tv_usec < t2->tv_usec)
		return (-1);
	if (t1->tv_usec > t2->tv_usec)
		return (1);
	return (0);
}

static int 
iommu_strbuf_flush_done(is)
	struct iommu_state *is;
{
	struct timeval cur, flushtimeout;

	if (!is->is_sb[0] && !is->is_sb[1])
		return (0);

	/*
	 * Streaming buffer flushes:
	 * 
	 *   1 Tell strbuf to flush by storing va to strbuf_pgflush.  If
	 *     we're not on a cache line boundary (64-bits):
	 *   2 Store 0 in flag
	 *   3 Store pointer to flag in flushsync
	 *   4 wait till flushsync becomes 0x1
	 *
	 * If it takes more than .5 sec, something
	 * went wrong.
	 */
	is->is_flush[0] = (is->is_sb[0] == NULL) ? 1 : 0;
	is->is_flush[1] = (is->is_sb[1] == NULL) ? 1 : 0;
	membar_memissue();

	if (is->is_sb[0]) {
		bus_space_write_8(is->is_bustag, (bus_space_handle_t)(u_long)
			&is->is_sb[0]->strbuf_flushsync, 0, is->is_flushpa);
		bus_space_barrier(is->is_bustag, (bus_space_handle_t)(u_long)
		    &is->is_sb[0]->strbuf_flushsync, 0, sizeof(u_int64_t),
		    BUS_SPACE_BARRIER_WRITE);
	}

	if (is->is_sb[1]) {
		bus_space_write_8(is->is_bustag, (bus_space_handle_t)(u_long)
			&is->is_sb[1]->strbuf_flushsync, 0, is->is_flushpa + 8);
		bus_space_barrier(is->is_bustag, (bus_space_handle_t)(u_long)
		    &is->is_sb[1]->strbuf_flushsync, 0, sizeof(u_int64_t),
		    BUS_SPACE_BARRIER_WRITE);
	}

	microtime(&cur);
	flushtimeout.tv_usec = cur.tv_usec + 500000; /* 1/2 sec */
	if (flushtimeout.tv_usec >= 1000000) {
		flushtimeout.tv_usec -= 1000000;
		flushtimeout.tv_sec = cur.tv_sec + 1;
	} else
		flushtimeout.tv_sec = cur.tv_sec;
	
	DPRINTF(IDB_IOMMU, ("iommu_strbuf_flush_done: flush = %lx at va = %lx pa = %lx now=%lx:%lx until = %lx:%lx\n", 
		       (long)is->is_flush, (long)&is->is_flush, 
		       (long)is->is_flushpa, cur.tv_sec, cur.tv_usec, 
		       flushtimeout.tv_sec, flushtimeout.tv_usec));

	/* Bypass non-coherent D$ */
	while (((ldxa(is->is_flushpa, ASI_PHYS_CACHED) == 0) ||
	        (ldxa(is->is_flushpa + 8, ASI_PHYS_CACHED) == 0)) &&
	       (iommu_tv_comp(&cur, &flushtimeout) <= 0)) {
		microtime(&cur);
	}

#ifdef DIAGNOSTIC
	if (((is->is_sb[0] != NULL) && (ldxa(is->is_flushpa, ASI_PHYS_CACHED) == 0)) ||
	    ((is->is_sb[1] != NULL) && (ldxa(is->is_flushpa + 8, ASI_PHYS_CACHED) == 0))) {
		printf("iommu_strbuf_flush_done: flush timeout %p,%p at %p\n",
			(void *)(u_long)is->is_flush[0],
			(void *)(u_long)is->is_flush[1],
			(void *)(u_long)is->is_flushpa); /* panic? */
#ifdef DDB
		Debugger();
#endif
	}
#endif
	DPRINTF(IDB_IOMMU, ("iommu_strbuf_flush_done: flushed\n"));
	return (is->is_flush[0] && is->is_flush[1]);
}

/*
 * IOMMU DVMA operations, common to SBUS and PCI.
 */
int
iommu_dvmamap_load(t, is, map, buf, buflen, p, flags)
	bus_dma_tag_t t;
	struct iommu_state *is;
	bus_dmamap_t map;
	void *buf;
	bus_size_t buflen;
	struct proc *p;
	int flags;
{
	int s;
	int err;
	bus_size_t sgsize;
	paddr_t curaddr;
	u_long dvmaddr, sgstart, sgend;
	bus_size_t align, boundary;
	vaddr_t vaddr = (vaddr_t)buf;
	int seg;
	pmap_t pmap;

	if (map->dm_nsegs) {
		/* Already in use?? */
#ifdef DIAGNOSTIC
		printf("iommu_dvmamap_load: map still in use\n");
#endif
		bus_dmamap_unload(t, map);
	}
	/*
	 * Make sure that on error condition we return "no valid mappings".
	 */
	map->dm_nsegs = 0;

	if (buflen > map->_dm_size) {
		DPRINTF(IDB_BUSDMA,
		    ("iommu_dvmamap_load(): error %d > %d -- "
		     "map size exceeded!\n", (int)buflen, (int)map->_dm_size));
		return (EINVAL);
	}

	sgsize = round_page(buflen + ((int)vaddr & PGOFSET));

	/*
	 * A boundary presented to bus_dmamem_alloc() takes precedence
	 * over boundary in the map.
	 */
	if ((boundary = (map->dm_segs[0]._ds_boundary)) == 0)
		boundary = map->_dm_boundary;
	align = max(map->dm_segs[0]._ds_align, NBPG);

	if (flags & BUS_DMA_24BIT) {
		sgstart = max(is->is_dvmamap->ex_start, 0xff000000);
		sgend = min(is->is_dvmamap->ex_end, 0xffffffff);
	} else {
		sgstart = is->is_dvmamap->ex_start;
		sgend = is->is_dvmamap->ex_end;
	}
	s = splhigh();
	/* 
	 * If our segment size is larger than the boundary we need to 
	 * split the transfer up int little pieces ourselves.
	 */
	err = extent_alloc_subregion(is->is_dvmamap, sgstart, sgend,
	    sgsize, align, 0, (sgsize > boundary) ? 0 : boundary, 
	    EX_NOWAIT|EX_BOUNDZERO, (u_long *)&dvmaddr);
	splx(s);

#ifdef DEBUG
	if (err || (dvmaddr == (bus_addr_t)-1))	
	{ 
		printf("iommu_dvmamap_load(): extent_alloc(%d, %x) failed!\n",
		    (int)sgsize, flags);
#ifdef DDB
		Debugger();
#endif
	}		
#endif	
	if (err != 0)
		return (err);

	if (dvmaddr == (bus_addr_t)-1)
		return (ENOMEM);

	/* Set the active DVMA map */
	map->_dm_dvmastart = dvmaddr;
	map->_dm_dvmasize = sgsize;

	/*
	 * Now split the DVMA range into segments, not crossing
	 * the boundary.
	 */
	seg = 0;
	sgstart = dvmaddr + (vaddr & PGOFSET);
	sgend = sgstart + buflen - 1;
	map->dm_segs[seg].ds_addr = sgstart;
	DPRINTF(IDB_INFO, ("iommu_dvmamap_load: boundary %lx boundary-1 %lx "
		"~(boundary-1) %lx\n", boundary, (boundary-1), ~(boundary-1)));
	while ((sgstart & ~(boundary - 1)) != (sgend & ~(boundary - 1))) {
		/* Oops.  We crossed a boundary.  Split the xfer. */
		DPRINTF(IDB_INFO, ("iommu_dvmamap_load: "
			"seg %d start %lx size %lx\n", seg,
			(long)map->dm_segs[seg].ds_addr, 
			map->dm_segs[seg].ds_len));
		map->dm_segs[seg].ds_len =
		    boundary - (sgstart & (boundary - 1));
		if (++seg > map->_dm_segcnt) {
			/* Too many segments.  Fail the operation. */
			DPRINTF(IDB_INFO, ("iommu_dvmamap_load: "
				"too many segments %d\n", seg));
			s = splhigh();
			/* How can this fail?  And if it does what can we do? */
			err = extent_free(is->is_dvmamap,
				dvmaddr, sgsize, EX_NOWAIT);
			map->_dm_dvmastart = 0;
			map->_dm_dvmasize = 0;
			splx(s);
			return (E2BIG);
		}
		sgstart = roundup(sgstart, boundary);
		map->dm_segs[seg].ds_addr = sgstart;
	}
	map->dm_segs[seg].ds_len = sgend - sgstart + 1;
	DPRINTF(IDB_INFO, ("iommu_dvmamap_load: "
		"seg %d start %lx size %lx\n", seg,
		(long)map->dm_segs[seg].ds_addr, map->dm_segs[seg].ds_len));
	map->dm_nsegs = seg+1;
	map->dm_mapsize = buflen;

	if (p != NULL)
		pmap = p->p_vmspace->vm_map.pmap;
	else
		pmap = pmap_kernel();

	for (; buflen > 0; ) {
		/*
		 * Get the physical address for this page.
		 */
		if (pmap_extract(pmap, (vaddr_t)vaddr, &curaddr) == FALSE) {
			bus_dmamap_unload(t, map);
			return (-1);
		}

		/*
		 * Compute the segment size, and adjust counts.
		 */
		sgsize = NBPG - ((u_long)vaddr & PGOFSET);
		if (buflen < sgsize)
			sgsize = buflen;

		DPRINTF(IDB_BUSDMA,
		    ("iommu_dvmamap_load: map %p loading va %p "
			    "dva %lx at pa %lx\n",
			    map, (void *)vaddr, (long)dvmaddr,
			    (long)(curaddr&~(NBPG-1))));
		iommu_enter(is, trunc_page(dvmaddr), trunc_page(curaddr),
		    flags|0x4000);
			
		dvmaddr += PAGE_SIZE;
		vaddr += sgsize;
		buflen -= sgsize;
	}
#ifdef DIAGNOSTIC
	for (seg = 0; seg < map->dm_nsegs; seg++) {
		if (map->dm_segs[seg].ds_addr < is->is_dvmabase ||
		    map->dm_segs[seg].ds_addr > is->is_dvmaend) {
			printf("seg %d dvmaddr %lx out of range %x - %x\n",
			    seg, (long)map->dm_segs[seg].ds_addr,
			    is->is_dvmabase, is->is_dvmaend);
#ifdef DDB
			Debugger();
#endif
		}
	}
#endif
	return (0);
}


void
iommu_dvmamap_unload(t, is, map)
	bus_dma_tag_t t;
	struct iommu_state *is;
	bus_dmamap_t map;
{
	int error, s;
	bus_size_t sgsize;

	/* Flush the iommu */
#ifdef DEBUG
	if (!map->_dm_dvmastart) {
		printf("iommu_dvmamap_unload: No dvmastart is zero\n");
#ifdef DDB
		Debugger();
#endif
	}
#endif
	iommu_remove(is, map->_dm_dvmastart, map->_dm_dvmasize);

	/* Flush the caches */
	bus_dmamap_unload(t->_parent, map);

	/* Mark the mappings as invalid. */
	map->dm_mapsize = 0;
	map->dm_nsegs = 0;
	
	s = splhigh();
	error = extent_free(is->is_dvmamap, map->_dm_dvmastart, 
		map->_dm_dvmasize, EX_NOWAIT);
	map->_dm_dvmastart = 0;
	map->_dm_dvmasize = 0;
	splx(s);
	if (error != 0)
		printf("warning: %qd of DVMA space lost\n", (long long)sgsize);

	/* Clear the map */
}


int
iommu_dvmamap_load_raw(t, is, map, segs, nsegs, flags, size)
	bus_dma_tag_t t;
	struct iommu_state *is;
	bus_dmamap_t map;
	bus_dma_segment_t *segs;
	int nsegs;
	int flags;
	bus_size_t size;
{
	struct vm_page *m;
	int i, j, s;
	int left;
	int err;
	bus_size_t sgsize;
	paddr_t pa;
	bus_size_t boundary, align;
	u_long dvmaddr, sgstart, sgend;
	struct pglist *mlist;
	int pagesz = PAGE_SIZE;
	int npg = 0; /* DEBUG */

	if (map->dm_nsegs) {
		/* Already in use?? */
#ifdef DIAGNOSTIC
		printf("iommu_dvmamap_load_raw: map still in use\n");
#endif
		bus_dmamap_unload(t, map);
	}

	/*
	 * A boundary presented to bus_dmamem_alloc() takes precedence
	 * over boundary in the map.
	 */
	if ((boundary = segs[0]._ds_boundary) == 0)
		boundary = map->_dm_boundary;

	align = max(segs[0]._ds_align, pagesz);

	/*
	 * Make sure that on error condition we return "no valid mappings".
	 */
	map->dm_nsegs = 0;
	/* Count up the total number of pages we need */
	pa = segs[0].ds_addr;
	sgsize = 0;
	left = size;
	for (i=0; left && i<nsegs; i++) {
		if (round_page(pa) != round_page(segs[i].ds_addr))
			sgsize = round_page(sgsize);
		sgsize += min(left, segs[i].ds_len);
		left -= segs[i].ds_len;
		pa = segs[i].ds_addr + segs[i].ds_len;
	}
	sgsize = round_page(sgsize);

	if (flags & BUS_DMA_24BIT) {
		sgstart = max(is->is_dvmamap->ex_start, 0xff000000);
		sgend = min(is->is_dvmamap->ex_end, 0xffffffff);
	} else {
		sgstart = is->is_dvmamap->ex_start;
		sgend = is->is_dvmamap->ex_end;
	}
	s = splhigh();
	/* 
	 * If our segment size is larger than the boundary we need to 
	 * split the transfer up into little pieces ourselves.
	 */
	err = extent_alloc_subregion(is->is_dvmamap, sgstart, sgend,
	    sgsize, align, 0, (sgsize > boundary) ? 0 : boundary, 
	    EX_NOWAIT|EX_BOUNDZERO, (u_long *)&dvmaddr);
	splx(s);

	if (err != 0)
		return (err);

#ifdef DEBUG
	if (dvmaddr == (bus_addr_t)-1)	
	{ 
		printf("iommu_dvmamap_load_raw(): extent_alloc(%d, %x) failed!\n",
		    (int)sgsize, flags);
#ifdef DDB
		Debugger();
#else
		panic("");
#endif
	}		
#endif	
	if (dvmaddr == (bus_addr_t)-1)
		return (ENOMEM);

	/* Set the active DVMA map */
	map->_dm_dvmastart = dvmaddr;
	map->_dm_dvmasize = sgsize;

	if ((mlist = segs[0]._ds_mlist) == NULL) {
		u_long prev_va = NULL;
		paddr_t prev_pa = 0;
		int end = 0, offset;

		/*
		 * This segs is made up of individual physical
		 * segments, probably by _bus_dmamap_load_uio() or
		 * _bus_dmamap_load_mbuf().  Ignore the mlist and
		 * load each one individually.
		 */
		map->dm_mapsize = size;

		j = 0;
		for (i = 0; i < nsegs; i++) {
			pa = segs[i].ds_addr;
			offset = (pa & PGOFSET);
			pa = trunc_page(pa);
			dvmaddr = trunc_page(dvmaddr);
			left = min(size, segs[i].ds_len);

			DPRINTF(IDB_INFO, ("iommu_dvamap_load_raw: converting "
			    "physseg %d start %lx size %lx\n", i,
			    (long)segs[i].ds_addr, segs[i].ds_len));

			if ((pa == prev_pa) &&
			    ((offset != 0) || (end != offset))) {
				/* We can re-use this mapping */
#ifdef DEBUG
if (iommudebug & 0x10) printf("reusing dva %lx prev %lx pa %lx prev %lx\n",
    dvmaddr, prev_va, pa, prev_pa);
#endif
				dvmaddr = prev_va; 
			}
			sgstart = dvmaddr + offset;
			sgend = sgstart + left - 1;

			/* Are the segments virtually adjacent? */
			if ((j > 0) && (end == offset) &&
			    ((offset = 0) || (pa == prev_pa))) {
				/* Just append to the previous segment. */
#ifdef DEBUG
if (iommudebug & 0x10) {
printf("appending offset %x pa %lx, prev %lx dva %lx prev %lx\n",
    offset, pa, prev_pa, dvmaddr, prev_va);
}
#endif

				map->dm_segs[--j].ds_len += left; 
				DPRINTF(IDB_INFO, ("iommu_dvmamap_load_raw: "
				    "appending seg %d start %lx size %lx\n", j,
				    (long)map->dm_segs[j].ds_addr,
				    map->dm_segs[j].ds_len));
			} else {
				map->dm_segs[j].ds_addr = sgstart;
				map->dm_segs[j].ds_len = left;
				DPRINTF(IDB_INFO, ("iommu_dvmamap_load_raw: "
				    "seg %d start %lx size %lx\n", j,
				    (long)map->dm_segs[j].ds_addr,
				    map->dm_segs[j].ds_len));
			}
			end = (offset + left) & PGOFSET;

			/* Check for boundary issues */
			while ((sgstart & ~(boundary - 1)) !=
				(sgend & ~(boundary - 1))) {
				/* Need a new segment. */
				map->dm_segs[j].ds_len =
					sgstart & (boundary - 1);
				DPRINTF(IDB_INFO, ("iommu_dvmamap_load_raw: "
					"seg %d start %lx size %lx\n", j,
					(long)map->dm_segs[j].ds_addr, 
					map->dm_segs[j].ds_len));
				if (++j > map->_dm_segcnt) {
					iommu_dvmamap_unload(t, is, map);
					return (E2BIG);
				}
				sgstart = roundup(sgstart, boundary);
				map->dm_segs[j].ds_addr = sgstart;
				map->dm_segs[j].ds_len = sgend - sgstart + 1;
			}

			if (sgsize == 0)
				panic("iommu_dmamap_load_raw: size botch");

			/* Now map a series of pages. */
			while (dvmaddr < sgend) {
				DPRINTF(IDB_BUSDMA,
				    ("iommu_dvamap_load_raw: map %p "
				    "loading va %lx at pa %lx\n",
				    map, (long)dvmaddr,
				    (long)(pa)));
				/* Enter if if we haven't before. */
				if (prev_va != dvmaddr)
#ifdef DEBUG
{ if (iommudebug & 0x10) printf("seg %d:5d entering dvma %lx, prev %lx pa %lx\n", i, j, dvmaddr, prev_va, pa);
#endif
					 iommu_enter(is, prev_va = dvmaddr,
					     prev_pa = pa, flags|(++npg<<12)); 
#ifdef DEBUG
} else if (iommudebug & 0x10) printf("seg %d:%d skipping dvma %lx, prev %lx\n", i, j, dvmaddr, prev_va);
#endif

				dvmaddr += pagesz;
				pa += pagesz;
			}

			size -= left;
			++j;
		}

		map->dm_nsegs = j;
#ifdef DIAGNOSTIC
		{
			int seg;
			for (seg = 0; seg < map->dm_nsegs; seg++) {
				if (map->dm_segs[seg].ds_addr < is->is_dvmabase ||
				    map->dm_segs[seg].ds_addr > is->is_dvmaend) {
					printf("seg %d dvmaddr %lx out of range %x - %x\n",
					    seg, (long)map->dm_segs[seg].ds_addr,
					    is->is_dvmabase, is->is_dvmaend);
#ifdef DDB
					Debugger();
#endif
				}
			}
		}
#endif
		return (0);
	}
	/*
	 * This was allocated with bus_dmamem_alloc.
	 * The pages are on an `mlist'.
	 */
	map->dm_mapsize = size;
	i = 0;
	sgstart = dvmaddr;
	sgend = sgstart + size - 1;
	map->dm_segs[i].ds_addr = sgstart;
	while ((sgstart & ~(boundary - 1)) != (sgend & ~(boundary - 1))) {
		/* Oops.  We crossed a boundary.  Split the xfer. */
		map->dm_segs[i].ds_len = sgstart & (boundary - 1);
		DPRINTF(IDB_INFO, ("iommu_dvmamap_load_raw: "
			"seg %d start %lx size %lx\n", i,
			(long)map->dm_segs[i].ds_addr,
			map->dm_segs[i].ds_len));
		if (++i > map->_dm_segcnt) {
			/* Too many segments.  Fail the operation. */
			s = splhigh();
			/* How can this fail?  And if it does what can we do? */
			err = extent_free(is->is_dvmamap,
				dvmaddr, sgsize, EX_NOWAIT);
			map->_dm_dvmastart = 0;
			map->_dm_dvmasize = 0;
			splx(s);
			return (E2BIG);
		}
		sgstart = roundup(sgstart, boundary);
		map->dm_segs[i].ds_addr = sgstart;
	}
	DPRINTF(IDB_INFO, ("iommu_dvmamap_load_raw: "
			"seg %d start %lx size %lx\n", i,
			(long)map->dm_segs[i].ds_addr, map->dm_segs[i].ds_len));
	map->dm_segs[i].ds_len = sgend - sgstart + 1;

	for (m = TAILQ_FIRST(mlist); m != NULL; m = TAILQ_NEXT(m,pageq)) {
		if (sgsize == 0)
			panic("iommu_dmamap_load_raw: size botch");
		pa = VM_PAGE_TO_PHYS(m);

		DPRINTF(IDB_BUSDMA,
		    ("iommu_dvmamap_load_raw: map %p loading va %lx at pa %lx\n",
		    map, (long)dvmaddr, (long)(pa)));
		iommu_enter(is, dvmaddr, pa, flags|0x8000);
			
		dvmaddr += pagesz;
		sgsize -= pagesz;
	}
	map->dm_mapsize = size;
	map->dm_nsegs = i+1;
#ifdef DIAGNOSTIC
	{
		int seg;
		for (seg = 0; seg < map->dm_nsegs; seg++) {
			if (map->dm_segs[seg].ds_addr < is->is_dvmabase ||
			    map->dm_segs[seg].ds_addr > is->is_dvmaend) {
				printf("seg %d dvmaddr %lx out of range %x - %x\n",
				    seg, (long)map->dm_segs[seg].ds_addr,
				    is->is_dvmabase, is->is_dvmaend);
#ifdef DDB
				Debugger();
#endif
			}
		}
       }
#endif
	return (0);
}

void
iommu_dvmamap_sync(t, is, map, offset, len, ops)
	bus_dma_tag_t t;
	struct iommu_state *is;
	bus_dmamap_t map;
	bus_addr_t offset;
	bus_size_t len;
	int ops;
{
	bus_size_t count;
	int i, needsflush = 0;

	for (i = 0; i < map->dm_nsegs; i++) {
		if (offset < map->dm_segs[i].ds_len)
			break;
		offset -= map->dm_segs[i].ds_len;
	}

	if (i == map->dm_nsegs)
		panic("iommu_dvmamap_sync: too short %lu", offset);

	for (; len > 0 && i < map->dm_nsegs; i++) {
		count = min(map->dm_segs[i].ds_len - offset, len);
		needsflush += iommu_dvmamap_sync_seg(t, is, &map->dm_segs[i],
		    offset, count, ops);
		len -= count;
	}

	if (i == map->dm_nsegs && len > 0)
		panic("iommu_dvmamap_sync: leftover %lu", len);

	if (needsflush)
		iommu_strbuf_flush_done(is);
}

/*
 * Flush an individual dma segment, returns non-zero if the streaming buffers
 * need flushing afterwards.
 */
int
iommu_dvmamap_sync_seg(t, is, seg, offset, len, ops)
	bus_dma_tag_t t;
	struct iommu_state *is;
	bus_dma_segment_t *seg;
	bus_addr_t offset;
	bus_size_t len;
	int ops;
{
	int needsflush = 0;
	vaddr_t va = seg->ds_addr + offset;

	if (len == 0)
		goto out;

	len += offset & PGOFSET;

	if (ops & BUS_DMASYNC_PREREAD) {
		DPRINTF(IDB_SYNC,
		    ("iommu_dvmamap_sync_seg: syncing va %p len %lu "
		     "BUS_DMASYNC_PREREAD\n", (void *)(u_long)va, (u_long)len));

		/* Nothing to do */;
	}

	if (ops & BUS_DMASYNC_POSTREAD) {
		DPRINTF(IDB_SYNC,
		    ("iommu_dvmamap_sync_seg: syncing va %p len %lu "
		     "BUS_DMASYNC_POSTREAD\n", (void *)(u_long)va, (u_long)len));
		/* if we have a streaming buffer, flush it here first */
		if (is->is_sb[0] || is->is_sb[1])
			while (len > 0) {
				DPRINTF(IDB_BUSDMA,
				    ("iommu_dvmamap_sync_seg: flushing va %p, %lu "
				     "bytes left\n", (void *)(u_long)va, (u_long)len));
				if (iommu_tsb_entry(is, va) & IOTTE_STREAM) {
					iommu_strbuf_flush(is, va);
					needsflush = 1;
				}
				if (len <= NBPG)
					len = 0;
				else
					len -= NBPG;
				va += NBPG;
			}
	}
	if (ops & BUS_DMASYNC_PREWRITE) {
		DPRINTF(IDB_SYNC,
		    ("iommu_dvmamap_sync_seg: syncing va %p len %lu "
		     "BUS_DMASYNC_PREWRITE\n", (void *)(u_long)va, (u_long)len));
		/* if we have a streaming buffer, flush it here first */
		if (is->is_sb[0] || is->is_sb[1])
			while (len > 0) {
				DPRINTF(IDB_BUSDMA,
				    ("iommu_dvmamap_sync_seg: flushing va %p, %lu "
				     "bytes left\n", (void *)(u_long)va, (u_long)len));
				if (iommu_tsb_entry(is, va) & IOTTE_STREAM) {
					iommu_strbuf_flush(is, va);
					needsflush = 1;
				}
				if (len <= NBPG)
					len = 0;
				else
					len -= NBPG;
				va += NBPG;
			}
	}
	if (ops & BUS_DMASYNC_POSTWRITE) {
		DPRINTF(IDB_SYNC,
		    ("iommu_dvmamap_sync_seg: syncing va %p len %lu "
		     "BUS_DMASYNC_POSTWRITE\n", (void *)(u_long)va, (u_long)len));
		/* Nothing to do */;
	}

out:
	return (needsflush);
}

int
iommu_dvmamem_alloc(t, is, size, alignment, boundary, segs, nsegs, rsegs, flags)
	bus_dma_tag_t t;
	struct iommu_state *is;
	bus_size_t size, alignment, boundary;
	bus_dma_segment_t *segs;
	int nsegs;
	int *rsegs;
	int flags;
{

	DPRINTF(IDB_BUSDMA, ("iommu_dvmamem_alloc: sz %llx align %llx bound %llx "
	   "segp %p flags %d\n", (unsigned long long)size,
	   (unsigned long long)alignment, (unsigned long long)boundary,
	   segs, flags));
	return (bus_dmamem_alloc(t->_parent, size, alignment, boundary,
	    segs, nsegs, rsegs, flags|BUS_DMA_DVMA));
}

void
iommu_dvmamem_free(t, is, segs, nsegs)
	bus_dma_tag_t t;
	struct iommu_state *is;
	bus_dma_segment_t *segs;
	int nsegs;
{

	DPRINTF(IDB_BUSDMA, ("iommu_dvmamem_free: segp %p nsegs %d\n",
	    segs, nsegs));
	bus_dmamem_free(t->_parent, segs, nsegs);
}

/*
 * Map the DVMA mappings into the kernel pmap.
 * Check the flags to see whether we're streaming or coherent.
 */
int
iommu_dvmamem_map(t, is, segs, nsegs, size, kvap, flags)
	bus_dma_tag_t t;
	struct iommu_state *is;
	bus_dma_segment_t *segs;
	int nsegs;
	size_t size;
	caddr_t *kvap;
	int flags;
{
	struct vm_page *m;
	vaddr_t va;
	bus_addr_t addr;
	struct pglist *mlist;
	int cbit;

	DPRINTF(IDB_BUSDMA, ("iommu_dvmamem_map: segp %p nsegs %d size %lx\n",
	    segs, nsegs, size));

	/*
	 * Allocate some space in the kernel map, and then map these pages
	 * into this space.
	 */
	size = round_page(size);
	va = uvm_km_valloc(kernel_map, size);
	if (va == 0)
		return (ENOMEM);

	*kvap = (caddr_t)va;

	/* 
	 * digest flags:
	 */
	cbit = 0;
	if (flags & BUS_DMA_COHERENT)	/* Disable vcache */
		cbit |= PMAP_NVC;
	if (flags & BUS_DMA_NOCACHE)	/* sideffects */
		cbit |= PMAP_NC;

	/*
	 * Now take this and map it into the CPU.
	 */
	mlist = segs[0]._ds_mlist;
	for (m = mlist->tqh_first; m != NULL; m = m->pageq.tqe_next) {
#ifdef DIAGNOSTIC
		if (size == 0)
			panic("iommu_dvmamem_map: size botch");
#endif
		addr = VM_PAGE_TO_PHYS(m);
		DPRINTF(IDB_BUSDMA, ("iommu_dvmamem_map: "
		    "mapping va %lx at %llx\n", va, (unsigned long long)addr | cbit));
		pmap_enter(pmap_kernel(), va, addr | cbit,
		    VM_PROT_READ | VM_PROT_WRITE,
		    VM_PROT_READ | VM_PROT_WRITE | PMAP_WIRED);
		va += PAGE_SIZE;
		size -= PAGE_SIZE;
	}
	pmap_update(pmap_kernel());

	return (0);
}

/*
 * Unmap DVMA mappings from kernel
 */
void
iommu_dvmamem_unmap(t, is, kva, size)
	bus_dma_tag_t t;
	struct iommu_state *is;
	caddr_t kva;
	size_t size;
{
	
	DPRINTF(IDB_BUSDMA, ("iommu_dvmamem_unmap: kvm %p size %lx\n",
	    kva, size));
	    
#ifdef DIAGNOSTIC
	if ((u_long)kva & PGOFSET)
		panic("iommu_dvmamem_unmap");
#endif
	
	size = round_page(size);
	pmap_remove(pmap_kernel(), (vaddr_t)kva, size);
	pmap_update(pmap_kernel());
	uvm_km_free(kernel_map, (vaddr_t)kva, size);
}
