# $OpenBSD: Makefile,v 1.4 2006/08/17 08:32:30 damien Exp $

NOPROG=
NOMAN=
CC=${HOSTCC}

# PCI capable systems only
.if (${MACHINE_ARCH} == "powerpc") || (${MACHINE} == "alpha") || \
    (${MACHINE} == "amd64") || (${MACHINE} == "armish") || \
    (${MACHINE} == "cats") || (${MACHINE} == "hppa") || \
    (${MACHINE} == "hppa64") || (${MACHINE} == "i386") || \
    (${MACHINE} == "sgi") || (${MACHINE} == "sparc64")

FIRM=	ral-rt2561 ral-rt2561s ral-rt2661

PROG=	build

CLEANFILES+= ${FIRM} ${PROG}

all: ${FIRM}

${FIRM}: build
	${.OBJDIR}/build

realinstall:

afterinstall:
	${INSTALL} -c -o ${BINOWN} -g ${BINGRP} -m 644 \
	    ${FIRM} ${DESTDIR}/etc/firmware
	${INSTALL} -c -o ${BINOWN} -g ${BINGRP} -m 644 \
	    ${.CURDIR}/ral-license ${DESTDIR}/etc/firmware

.endif

.include <bsd.prog.mk>
