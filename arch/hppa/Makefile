#	$OpenBSD: Makefile,v 1.5 2005/05/06 03:17:29 marco Exp $

# Makefile for hppa tags file and boot blocks

S=${.CURDIR}/../..
.include "$S/kern/Make.tags.inc"

NOPROG=	noprog
NOMAN=	noman

SUBDIR=	stand

THPPA=	$S/arch/hppa/tags
SHPPA=	$S/arch/hppa/hppa/*.[ch] $S/arch/hppa/include/*.h \
	$S/arch/hppa/dev/*.[ch] $S/arch/hppa/gsc/*.[ch] \
	$S/arch/hppa/spmath/*.[ch]
AHPPA=	$S/arch/hppa/hppa/*.S $S/arch/hppa/spmath/*.S \
	$S/lib/libkern/arch/hppa/*.[sS]

# Directories in which to place hppa tags links
DHPPA=	conf dev gsc hppa spmath include

tags:
	-ctags -wdtf ${THPPA} ${COMM} ${SHPPA}
	egrep "^ENTRY(.*)|^ALTENTRY(.*)" ${AHPPA} | \
	    sed "s;\([^:]*\):\([^(]*\)(\([^, )]*\)\(.*\);\3	\1	/^\2(\3\4$$/;" \
		>> ${THPPA}
	sort -o ${THPPA} ${THPPA}

links:
	-for i in ${DHPPA}; do \
	    (cd $$i && rm -f tags; ln -s ${THPPA} tags); done

obj: _SUBDIRUSE

.include <bsd.prog.mk>
