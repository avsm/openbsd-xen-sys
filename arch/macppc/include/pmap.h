/*	$OpenBSD: pmap.h,v 1.1 2001/09/01 15:49:06 drahn Exp $	*/

#include <powerpc/pmap.h>

paddr_t vtophys __P((vaddr_t));
