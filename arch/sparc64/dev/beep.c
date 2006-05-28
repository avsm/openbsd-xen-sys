/*	$OpenBSD: beep.c,v 1.10 2005/12/20 16:50:33 martin Exp $	*/

/*
 * Copyright (c) 2006 Jason L. Wright (jason@thought.net)
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
 */

/*
 * Driver for beeper device on BBC machines (Blade 1k, 2k, etc)
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

#define	BEEP_CTRL		0
#define	BEEP_CNT_0		2
#define	BEEP_CNT_1		3
#define	BEEP_CNT_2		4
#define	BEEP_CNT_3		5

#define	BEEP_CTRL_ON		0x01
#define	BEEP_CTRL_OFF		0x00

struct beep_softc {
	struct device		sc_dev;
	bus_space_tag_t		sc_iot;
	bus_space_handle_t	sc_ioh;
};

int	beep_match(struct device *, void *, void *);
void	beep_attach(struct device *, struct device *, void *);

struct cfattach beep_ca = {
	sizeof(struct beep_softc), beep_match, beep_attach
};

struct cfdriver beep_cd = {
	NULL, "beep", DV_DULL
};

int
beep_match(struct device *parent, void *match, void *aux)
{
	struct ebus_attach_args *ea = aux;

	if (strcmp(ea->ea_name, "beep") == 0)
		return (1);
	return (0);
}

void
beep_attach(parent, self, aux)
	struct device *parent, *self;
	void *aux;
{
	struct beep_softc *sc = (void *)self;
	struct ebus_attach_args *ea = aux;

	sc->sc_iot = ea->ea_memtag;

	/* Use prom address if available, otherwise map it. */
	if (ea->ea_nvaddrs) {
		if (bus_space_map(sc->sc_iot, ea->ea_vaddrs[0], 0,
		    BUS_SPACE_MAP_PROMADDRESS, &sc->sc_ioh)) {
			printf(": can't map PROM register space\n");
			return;
		}
	} else if (ebus_bus_map(sc->sc_iot, 0,
	    EBUS_PADDR_FROM_REG(&ea->ea_regs[0]), ea->ea_regs[0].size, 0, 0,
	    &sc->sc_ioh) != 0) {
		printf(": can't map register space\n");
                return;
	}

	printf("\n");
}
