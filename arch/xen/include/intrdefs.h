/*	$OpenBSD: intrdefs.h,v 1.8 2005/11/30 17:50:20 hshoexer Exp $	*/
/*	$NetBSD: intrdefs.h,v 1.2 2003/05/04 22:01:56 fvdl Exp $	*/

#ifdef I686_CPU
#include <machine/i386/intrdefs.h>
#endif

#ifdef amd64
#include <machine/amd64/intrdefs.h>
#endif
