/* $NetBSD: xbdvar.h,v 1.8 2005/12/11 12:19:48 christos Exp $ */

/*
 *
 * Copyright (c) 2004 Christian Limpach.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. All advertising materials mentioning features or use of this software
 *    must display the following acknowledgement:
 *      This product includes software developed by Christian Limpach.
 * 4. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


#ifndef _XEN_XBCVAR_H_
#define _XEN_XBCVAR_H_

#define XBD_INITED	0x00010000 /* unit has been initialized */

struct xbc_softc {
	struct device		sc_dev;		/* base device glue */
	struct scsi_link	sc_link;	/* scsi link */
	struct scsi_xfer	*sc_xs;

	struct simplelock	sc_slock;	/* our lock */
	int			sc_shutdown;	/* about to be removed */
	uint32_t		sc_flags;
#if NRND > 0
	rndsource_element_t	sc_rnd_source;
#endif
};

struct xbc_attach_args {
	const char 		*xa_device;
#if 0
	vdisk_t			*xa_xd;
#endif
	struct sysctlnode	*xa_diskcookies;
};

int xbc_scan(struct device *, struct xbc_attach_args *, cfprint_t);

#endif /* _XEN_XBCVAR_H_ */
