/*	$OpenBSD$	*/
/*	$NetBSD: mainbus.c,v 1.1 2006/09/01 21:26:18 uwe Exp $	*/

/*-
 * Copyright (c) 2002 The NetBSD Foundation, Inc.
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
 *        This product includes software developed by the NetBSD
 *        Foundation, Inc. and its contributors.
 * 4. Neither the name of The NetBSD Foundation nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE NETBSD FOUNDATION, INC. AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE FOUNDATION OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "obio.h"
#include "pci.h"

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/device.h>

#include <machine/autoconf.h>
#include <machine/bus.h>

#include <landisk/dev/obiovar.h>

static int mainbus_match(struct device *, struct cfdata *, void *);
static void mainbus_attach(struct device *, struct device *, void *);

CFATTACH_DECL(mainbus, sizeof(struct device),
    mainbus_match, mainbus_attach, NULL, NULL);

static int mainbus_print(void *, const char *);

/* There can be only one. */
int mainbus_found = 0;

static int
mainbus_match(struct device *parent, struct cfdata *cf, void *aux)
{

	if (mainbus_found)
		return (0);

	return (1);
}

static void
mainbus_attach(struct device *parent, struct device *self, void *aux)
{
	union {
		struct mainbus_attach_args mba_mba;
		struct confargs mba_ca;
		struct obiobus_attach_args mba_oba;
	} mba;

	mainbus_found = 1;

	printf("\n");

	/* CPU */
	memset(&mba, 0, sizeof(mba));
	mba.mba_ca.ca_name = "cpu";
	mba.mba_ca.ca_node = 0;
	config_found(self, &mba, mainbus_print);

#if NPCI > 0
	/* SH PCIC */
	memset(&mba, 0, sizeof(mba));
	mba.mba_mba.ma_name = "shpcic";
	config_found(self, &mba, mainbus_print);
#endif

	/* SH bus */
	memset(&mba, 0, sizeof(mba));
	mba.mba_mba.ma_name = "shb";
	config_found(self, &mba, mainbus_print);

#if NOBIO > 0
	/* on-board I/O */
	memset(&mba, 0, sizeof(mba));
	mba.mba_oba.oba_busname = "obio";
	mba.mba_oba.oba_iot = &obio_bus_io;
	mba.mba_oba.oba_memt = &obio_bus_mem;
	config_found(self, &mba, mainbus_print);
#endif
}

static int
mainbus_print(void *aux, const char *pnp)
{

	return (pnp ? QUIET : UNCONF);
}
