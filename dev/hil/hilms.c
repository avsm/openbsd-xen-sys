/*	$OpenBSD$	*/
/*
 * Copyright (c) 2003, Miodrag Vallat.
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
 *
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/device.h>
#include <sys/ioctl.h>

#include <machine/autoconf.h>
#include <machine/bus.h>
#include <machine/cpu.h>

#include <dev/hil/hilreg.h>
#include <dev/hil/hilvar.h>
#include <dev/hil/hildevs.h>

#include <dev/wscons/wsconsio.h>
#include <dev/wscons/wsmousevar.h>

struct hilms_softc {
	struct device	sc_dev;

	int		sc_features;
	int		sc_axes;
	int		sc_enabled;
	int		sc_buttonstate;

	struct device	*sc_wsmousedev;
};

int	hilmsprobe(struct device *, void *, void *);
void	hilmsattach(struct device *, struct device *, void *);

struct cfdriver hilms_cd = {
	NULL, "hilms", DV_DULL
};

struct cfattach hilms_ca = {
	sizeof(struct hilms_softc), hilmsprobe, hilmsattach
};

int	hilms_enable(void *);
int	hilms_ioctl(void *, u_long, caddr_t, int, struct proc *);
void	hilms_disable(void *);

const struct wsmouse_accessops hilms_accessops = {
	hilms_enable,
	hilms_ioctl,
	hilms_disable,
};

void	hilms_callback(void *, u_int, u_int8_t *);

int
hilmsprobe(struct device *parent, void *match, void *aux)
{
	struct hil_attach_args *ha = aux;

	if (ha->ha_type != HIL_DEVICE_MOUSE)
		return (0);

	/*
	 * Reject anything that has only buttons - they are handled as
	 * keyboards, really.
	 */
	if (ha->ha_infolen > 1 && (ha->ha_info[1] & HIL_AXMASK) == 0)
		return (0);

	return (1);
}

void
hilmsattach(struct device *parent, struct device *self, void *aux)
{
	struct hilms_softc *sc = (void *)self;
	struct hil_attach_args *ha = aux;
	struct wsmousedev_attach_args a;
	int iob, buttons, rx, ry;

	/*
	 * Interpret the identification bytes, if any
	 */
	buttons = rx = ry = 0;
	if (ha->ha_infolen > 1) {
		sc->sc_features = ha->ha_info[1];
		sc->sc_axes = sc->sc_features & HIL_AXMASK;

		if (sc->sc_features & HIL_IOB) {
			/* skip resolution bytes */
			iob = 4;
			if (sc->sc_features & HIL_ABSOLUTE) {
				/* skip ranges */
				rx = ha->ha_info[4] | (ha->ha_info[5] << 8);
				if (sc->sc_axes > 1)
					ry = ha->ha_info[6] |
					    (ha->ha_info[7] << 8);
				iob += 2 * sc->sc_axes;
			}

			if (iob >= ha->ha_infolen) {
				sc->sc_features &= ~(HIL_IOB | HILIOB_PIO);
			} else {
				iob = ha->ha_info[iob];
				buttons = iob & HILIOB_BMASK;
				sc->sc_features |= (iob & HILIOB_PIO);
			}
		}
	}

	printf(", %d axes", sc->sc_axes);
	if (buttons == 1)
		printf(", 1 button");
	else if (buttons > 1)
		printf(", %d buttons", buttons);
	if (sc->sc_features & HILIOB_PIO)
		printf(", pressure sensor");
	if (sc->sc_features & HIL_ABSOLUTE) {
		printf ("\n%s: %d", sc->sc_dev.dv_xname, rx);
		if (ry != 0)
			printf("x%d", ry);
		else
			printf(" linear");
		printf(" fixed area");
	}

	hil_callback_register((struct hil_softc *)parent, ha->ha_code,
	    hilms_callback, sc);

	printf("\n");

	sc->sc_enabled = 0;

	a.accessops = &hilms_accessops;
	a.accesscookie = sc;

	sc->sc_wsmousedev = config_found(self, &a, wsmousedevprint);
}

int
hilms_enable(void *v)
{
	struct hilms_softc *sc = v;

	if (sc->sc_enabled)
		return EBUSY;

	sc->sc_enabled = 1;
	sc->sc_buttonstate = 0;

	return (0);
}

void
hilms_disable(void *v)
{
	struct hilms_softc *sc = v;

	sc->sc_enabled = 0;
}

int
hilms_ioctl(void *v, u_long cmd, caddr_t data, int flag, struct proc *p)
{
#if 0
	struct hilms_softc *sc = v;
#endif

	switch (cmd) {
	case WSMOUSEIO_GTYPE:
		*(int *)data = WSMOUSE_TYPE_HIL;
		return 0;
	}

	return -1;
}

void
hilms_callback(void *v, u_int buflen, u_int8_t *buf)
{
	struct hilms_softc *sc = v;
	int type, flags;
	int dx, dy, dz, button;
#ifdef DIAGNOSTIC
	int minlen;
#endif

	/*
	 * Ignore packet if we don't need it
	 */
	if (sc->sc_enabled == 0)
		return;

	type = *buf++;

#ifdef DIAGNOSTIC
	/*
	 * Check that the packet contains all the expected data,
	 * ignore it if too short.
	 */
	minlen = 1;
	if (type & HIL_MOUSEMOTION) {
		minlen += sc->sc_axes <<
		    (sc->sc_features & HIL_16_BITS) ? 1 : 0;
	}
	if (type & HIL_MOUSEBUTTON)
		minlen++;

	if (minlen > buflen)
		return;
#endif

	/*
	 * The packet can contain both a mouse motion and a button event.
	 * In this case, the motion data comes first.
	 */

	if (type & HIL_MOUSEMOTION) {
		flags = sc->sc_features & HIL_ABSOLUTE ?
		    WSMOUSE_INPUT_ABSOLUTE_X | WSMOUSE_INPUT_ABSOLUTE_Y |
		    WSMOUSE_INPUT_ABSOLUTE_Z : WSMOUSE_INPUT_DELTA;
		dx = *buf++;
		if (sc->sc_features & HIL_16_BITS)
			dx |= (*buf++) << 8;
		if (sc->sc_axes > 1) {
			dy = *buf++;
			if (sc->sc_features & HIL_16_BITS)
				dy |= (*buf++) << 8;
			if (sc->sc_axes > 2) {
				dz = *buf++;
				if (sc->sc_features & HIL_16_BITS)
					dz |= (*buf++) << 8;
			} else
				dz = 0;
		} else
			dy = dz = 0;
	} else
		dx = dy = dz = flags = 0;

	if (type & HIL_MOUSEBUTTON) {
		button = *buf;
		/*
		 * The pressure sensor is very primitive and only has
		 * a boolean behaviour, as an extra mouse button, which is
		 * down if there is pressure or the pen is near the tablet,
		 * and up if there is no pressure or the pen is far from the
		 * tablet - at least for Tablet id 0x94, P/N 46088B
		 *
		 * The corresponding codes are 0x8f and 0x8e. Convert them
		 * to a pseudo fourth button - even if the tablet never
		 * has three buttons.
		 */
		button = (button - 0x80) >> 1;
		if (button > 4)
			button = 4;

		if (*buf >> 1) {
			/* Button released, or no pressure */
			sc->sc_buttonstate &= ~(1 << button);
		} else {
			/* Button pressed, or pressure */
			sc->sc_buttonstate |= (1 << button);
		}
		/* buf++; */
	}
	
	if (sc->sc_wsmousedev != NULL)
		wsmouse_input(sc->sc_wsmousedev,
		    sc->sc_buttonstate, dx, dy, dz, flags);
}
