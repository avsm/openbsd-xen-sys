/*	$NetBSD: frameasm.h,v 1.2 2005/03/09 22:39:20 bouyer Exp $	*/
/*	NetBSD: frameasm.h,v 1.4 2004/02/20 17:35:01 yamt Exp 	*/

#ifdef I686_CPU
#include <machine/i386/frameasm.h>
#endif

#ifdef amd64
#include <machine/amd64/frameasm.h>
#endif
