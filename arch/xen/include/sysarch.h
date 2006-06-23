/*	$OpenBSD: sysarch.h,v 1.1.1.1 2005/09/02 16:10:28 hshoexer Exp $	*/
/*	$NetBSD: sysarch.h,v 1.8 1996/01/08 13:51:44 mycroft Exp $	*/

#ifdef I686_CPU
#include <machine/i386/sysarch.h>
#endif

#ifdef amd64
#include <machine/amd64/sysarch.h>
#endif
