#	from: @(#)Makefile	8.1 (Berkeley) 6/10/93
#	$OpenBSD: Makefile,v 1.10 2005/09/11 13:36:00 deraadt Exp $

S=	${.CURDIR}/../../../..
DEFS=-DSTAGE2_RELOC=${STAGE2_RELOC}
INCPATH=-I${.CURDIR} -I${.CURDIR}/../libsa -I${.CURDIR}/../libbug \
	-I${.CURDIR}/../../include -I${S} -I${S}/lib/libsa
CFLAGS+=${INCPATH} ${DEFS} ${COPTS}
CLEANFILES+=bootxx

.include "${S}/arch/mvme88k/stand/bugcrt/Makefile.inc"
.include "${S}/arch/mvme88k/stand/libbug/Makefile.inc"
.include "${S}/arch/mvme88k/stand/libsa/Makefile.inc"
.include "${S}/arch/mvme88k/stand/libz/Makefile.inc"

SRCS=   bootxx.c conf.c version.c

LIBS=	${LIBSA} ${LIBBUG} ${LIBZ}

OBJS= ${SRCS:N*.h:R:S/$/.o/g}

BOOTS=	bootxx
ALL=	${BOOTS}

all: ${ALL}

bootxx: ${OBJS} ${STAGE1} ${LIBS}
	${LD} -S -N -T ${STAGE1_RELOC} ${STAGE1} -o $@ \
	    ${OBJS} ${LIBS} `cc -print-libgcc-file-name`

install:
	install -c -m 555 -g bin -o bin ${BOOTS} ${DESTDIR}${MDEC_DIR}

.include <bsd.prog.mk>
