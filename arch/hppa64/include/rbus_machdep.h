/*	$OpenBSD: rbus_machdep.h,v 1.1 2005/04/01 10:40:48 mickey Exp $	*/

/*
 * Copyright (c) 2005 Michael Shalayeff
 * All rights reserved.
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF MIND, USE, DATA OR PROFITS, WHETHER IN
 * AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT
 * OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */


#if !defined _ARCH_HPPA64_HPPA64_RBUS_MACHDEP_H_
#define _ARCH_HPPA64_HPPA64_RBUS_MACHDEP_H_

static __inline int
md_space_map(bus_space_tag_t t, bus_addr_t bpa, bus_size_t  size, int flags, bus_space_handle_t *bshp)
{
	if (bshp)
		*(bshp) = bpa;

	return (0);
}

#define md_space_unmap(t,bsh,s,addrp)	do { *(addrp) = (bsh); } while (0)

struct pci_attach_args;

#define rbus_pccbb_parent_mem(d, p) (*(p)->pa_pc->pc_alloc_parent)((d), (p), 0)
#define rbus_pccbb_parent_io(d, p)  (*(p)->pa_pc->pc_alloc_parent)((d), (p), 1)

#endif /* _ARCH_HPPA64_HPPA64_RBUS_MACHDEP_H_ */
