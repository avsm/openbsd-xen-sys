#	$OpenBSD: Makefile,v 1.2 2005/05/06 02:03:15 marco Exp $

S=${.CURDIR}/../..
.include "$S/kern/Make.tags.inc"

NOPROG=	noprog
NOMAN=	noman

.if ${MACHINE} == "macppc"
SUBDIR= stand
.endif

TMACPPC=	$S/arch/macppc/tags
SMACPPC=	$S/arch/macppc/macppc/*.[ch] \
		$S/arch/macppc/include/*.h   \
		$S/arch/macppc/pci/*.[ch]
AMACPPC=	$S/arch/macppc/macppc/*.[sS]

# Directories in which to place macppc tags links
DMACPPC=	conf macppc include pci

tags:
	-ctags -wdtf ${TMACPPC} ${COMM} ${SMACPPC}
	egrep "^ENTRY(.*)|^ALTENTRY(.*)" ${AMACPPC} | \
	    sed "s;\([^:]*\):\([^(]*\)(\([^, )]*\)\(.*\);\3	\1	/^\2(\3\4$$/;" \
		>> ${TMACPPC}
	sort -o ${TMACPPC} ${TMACPPC}

links:
	-for i in ${DMACPPC}; do \
	    cd $$i && rm -f tags; ln -s ${TMACPPC} tags; done


obj:	_SUBDIRUSE

.include <bsd.prog.mk>
