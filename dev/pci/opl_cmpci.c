/*	$OpenBSD: opl_cmpci.c,v 1.2 2006/07/27 05:55:03 miod Exp $	*/

/*
 * Copyright (c) 1998 The NetBSD Foundation, Inc.
 * All rights reserved.
 *
 * This code is derived from software contributed to The NetBSD Foundation
 * by Lennart Augustsson (augustss@netbsd.org).
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

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/errno.h>
#include <sys/device.h>
#include <sys/malloc.h>
#include <sys/proc.h>
#include <sys/conf.h>
#include <sys/selinfo.h>
#include <sys/audioio.h>
#include <sys/midiio.h>

#include <machine/bus.h>

#include <dev/audio_if.h>
#include <dev/midi_if.h>
#include <dev/ic/oplreg.h>
#include <dev/ic/oplvar.h>

#include <dev/pci/pcireg.h>
#include <dev/pci/pcivar.h>

#include <dev/pci/cmpcireg.h>
#include <dev/pci/cmpcivar.h>

int	opl_cmpci_match(struct device *, void *, void *);
void	opl_cmpci_attach(struct device *, struct device *, void *);

struct cfdriver opl_cmpci_cd = {
	NULL, "opl_cmpci", DV_DULL
};

struct cfattach opl_cmpci_ca = {
	sizeof (struct opl_softc), opl_cmpci_match, opl_cmpci_attach
};

int
opl_cmpci_match(struct device *parent, void *match, void *aux)
{
	struct audio_attach_args *aa = (struct audio_attach_args *)aux;
	struct cmpci_softc *ssc = (struct cmpci_softc *)parent;
	struct opl_attach_arg oaa;

	if (aa->type != AUDIODEV_TYPE_OPL)
		return (0);
	memset(&oaa, 0, sizeof oaa);
	oaa.iot = ssc->sc_iot;
	oaa.ioh = ssc->sc_ioh;
	oaa.offs = CMPCI_REG_FM_BASE;
	return (opl_find(&oaa));
}

void
opl_cmpci_attach(struct device *parent, struct device *self, void *aux)
{
	struct cmpci_softc *ssc = (struct cmpci_softc *)parent;
	struct opl_softc *sc = (struct opl_softc *)self;

	sc->ioh = ssc->sc_ioh;
	sc->iot = ssc->sc_iot;
	sc->offs = CMPCI_REG_FM_BASE;
	strlcpy(sc->syn.name, "CMPCI ", sizeof sc->syn.name);

	opl_attach(sc);
}
