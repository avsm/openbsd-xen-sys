#	$OpenBSD: Makefile,v 1.1.1.1 2005/09/02 16:10:27 hshoexer Exp $
#	$NetBSD: Makefile,v 1.6 1996/02/01 22:31:28 mycroft Exp $
#	@(#)Makefile	7.3 (Berkeley) 6/9/91

# Makefile for xen tags file and boot blocks

S=${.CURDIR}/../..
.include "$S/kern/Make.tags.inc"

NOPROG=	noprog
NOMAN=	noman

SUBDIR=	stand

TXEN=	$S/arch/xen/tags
SXEN=
AXEN=
GXEN=

# Directories in which to place xen tags links
DXEN=

tags:
	-ctags -wdtf ${TXEN} ${COMM} ${SXEN} ${GXEN}
	egrep "^ENTRY(.*)|^ALTENTRY(.*)" ${AXEN} | \
	    sed "s;\([^:]*\):\([^(]*\)(\([^, )]*\)\(.*\);\3	\1	/^\2(\3\4$$/;" \
		>> ${TXEN}
	sort -o ${TXEN} ${TXEN}

links:
	-for i in ${DXEN}; do \
	    (cd $$i && rm -f tags; ln -s ${TXEN} tags); done

obj:	_SUBDIRUSE

.include <bsd.prog.mk>
