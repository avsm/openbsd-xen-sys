/*	$OpenBSD: pmap.h,v 1.3 2001/09/02 19:40:24 miod Exp $	*/

#include <powerpc/pmap.h>

paddr_t vtophys __P((vaddr_t));
