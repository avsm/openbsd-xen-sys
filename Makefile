#	$OpenBSD: Makefile,v 1.9 2001/06/25 02:11:59 deraadt Exp $
#	$NetBSD: Makefile,v 1.5 1995/09/15 21:05:21 pk Exp $

SUBDIR=	arch/alpha arch/amiga arch/hp300 arch/i386 \
	arch/mac68k arch/mvme68k arch/mvme88k arch/powerpc \
	arch/sparc arch/sun3 arch/vax

.include <bsd.subdir.mk>
