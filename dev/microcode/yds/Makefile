# $OpenBSD: Makefile,v 1.2 2005/06/16 08:23:49 markus Exp $

NOPROG=
NOMAN=
CC=${HOSTCC}

# PCI capable systems only
.if (${MACHINE} == "i386") || (${MACHINE} == "amd64") || \
    (${MACHINE} == "alpha") || (${MACHINE} == "sparc64") || \
    (${MACHINE_ARCH} == "powerpc") || (${MACHINE} == "cats") || \
    (${MACHINE} == "hppa") || (${MACHINE} == "hppa64") || \
    (${MACHINE} == "sgi")

FIRM= 	yds

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
	    ${.CURDIR}/yds-license ${DESTDIR}/etc/firmware
.endif

.include <bsd.prog.mk>
