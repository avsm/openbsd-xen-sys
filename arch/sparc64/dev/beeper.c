/*	$OpenBSD: beeper.c,v 1.4 2002/03/14 01:26:44 millert Exp $	*/

/*
 * Copyright (c) 2001 Jason L. Wright (jason@thought.net)
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
 *	This product includes software developed by Jason L. Wright
 * 4. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Effort sponsored in part by the Defense Advanced Research Projects
 * Agency (DARPA) and Air Force Research Laboratory, Air Force
 * Materiel Command, USAF, under agreement number F30602-01-2-0537.
 *
 */

/*
 * Driver for beeper device on SUNW,Ultra-1-Engine.
 */

#include <sys/types.h>
#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/device.h>
#include <sys/conf.h>
#include <sys/timeout.h>

#include <machine/bus.h>
#include <machine/autoconf.h>
#include <machine/openfirm.h>

#include <sparc64/dev/ebusreg.h>
#include <sparc64/dev/ebusvar.h>

#include "pckbd.h"
#if NPCKBD > 0
#include <dev/ic/pckbcvar.h>
#include <dev/pckbc/pckbdvar.h>
#endif

struct beeper_softc {
	struct device		sc_dev;
	bus_space_tag_t		sc_iot;
	bus_space_handle_t	sc_ioh;
	struct timeout		sc_to;
	int 			sc_belltimeout, sc_bellactive;
};

#define	BEEP_REG	0

int	beeper_match(struct device *, void *, void *);
void	beeper_attach(struct device *, struct device *, void *);

struct cfattach beeper_ca = {
	sizeof(struct beeper_softc), beeper_match, beeper_attach
};

struct cfdriver beeper_cd = {
	NULL, "beeper", DV_DULL
};

#if NPCKBD > 0
void beeper_stop(void *);
void beeper_bell(void *, u_int, u_int, u_int, int);
#endif

int
beeper_match(parent, match, aux)
	struct device *parent;
	void *match;
	void *aux;
{
	struct ebus_attach_args *ea = aux;

	if (strcmp(ea->ea_name, "beeper") == 0)
		return (1);
	return (0);
}

void
beeper_attach(parent, self, aux)
	struct device *parent, *self;
	void *aux;
{
	struct beeper_softc *sc = (void *)self;
	struct ebus_attach_args *ea = aux;

	sc->sc_iot = ea->ea_bustag;

	/* Use prom address if available, otherwise map it. */
	if (ea->ea_nvaddrs)
		sc->sc_ioh = (bus_space_handle_t)ea->ea_vaddrs[0];
	else if (ebus_bus_map(sc->sc_iot, 0,
			      EBUS_PADDR_FROM_REG(&ea->ea_regs[0]),
			      ea->ea_regs[0].size,
			      BUS_SPACE_MAP_LINEAR,
			      0, &sc->sc_ioh) != 0) {
		printf(": can't map register space\n");
                return;
	}

#if NPCKBD > 0
	timeout_set(&sc->sc_to, beeper_stop, sc);
	pckbd_hookup_bell(beeper_bell, sc);
#endif
	printf("\n");
}

#if NPCKBD > 0
void
beeper_stop(vsc)
	void *vsc;
{
	struct beeper_softc *sc = vsc;
	int s;

	s = spltty();
	bus_space_write_4(sc->sc_iot, sc->sc_ioh, BEEP_REG, 0);
	sc->sc_bellactive = 0;
	sc->sc_belltimeout = 0;
	splx(s);
}

void
beeper_bell(vsc, pitch, period, volume, poll)
	void *vsc;
	u_int pitch, period, volume;
	int poll;
{
	struct beeper_softc *sc = vsc;
	int s, ticks;

	ticks = (period * hz) / 1000;
	if (ticks <= 0)
		ticks = 1;

	s = spltty();
	if (sc->sc_bellactive) {
		if (sc->sc_belltimeout == 0)
			timeout_del(&sc->sc_to);
	}
	if (pitch == 0 || period == 0) {
		beeper_stop(sc);
		splx(s);
		return;
	}
	if (!sc->sc_bellactive) {
		sc->sc_bellactive = 1;
		sc->sc_belltimeout = 1;
		bus_space_write_4(sc->sc_iot, sc->sc_ioh, BEEP_REG, 1);
		timeout_add(&sc->sc_to, ticks);
	}
	splx(s);
}
#endif /* NPCKBD > 0 */
