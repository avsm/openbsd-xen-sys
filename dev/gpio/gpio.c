/*	$OpenBSD: gpio.c,v 1.6 2006/01/14 12:33:49 grange Exp $	*/

/*
 * Copyright (c) 2004, 2006 Alexander Yurchenko <grange@openbsd.org>
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

/*
 * General Purpose Input/Output framework.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/conf.h>
#include <sys/device.h>
#include <sys/fcntl.h>
#include <sys/ioctl.h>
#include <sys/gpio.h>
#include <sys/vnode.h>

#include <dev/gpio/gpiovar.h>

struct gpio_softc {
	struct device sc_dev;

	gpio_chipset_tag_t sc_gc;	/* our GPIO controller */
	gpio_pin_t *sc_pins;		/* pins array */
	int sc_npins;			/* total number of pins */

	int sc_opened;
};

int	gpio_match(struct device *, void *, void *);
void	gpio_attach(struct device *, struct device *, void *);
int	gpio_detach(struct device *, int);
int	gpio_search(struct device *, void *, void *);
int	gpio_print(void *, const char *);

struct cfattach gpio_ca = {
	sizeof (struct gpio_softc),
	gpio_match,
	gpio_attach,
	gpio_detach
};

struct cfdriver gpio_cd = {
	NULL, "gpio", DV_DULL
};

int
gpio_match(struct device *parent, void *match, void *aux)
{
	struct cfdata *cf = match;
	struct gpiobus_attach_args *gba = aux;

	if (strcmp(gba->gba_name, cf->cf_driver->cd_name) != 0)
		return (0);

	return (1);
}

void
gpio_attach(struct device *parent, struct device *self, void *aux)
{
	struct gpio_softc *sc = (struct gpio_softc *)self;
	struct gpiobus_attach_args *gba = aux;

	sc->sc_gc = gba->gba_gc;
	sc->sc_pins = gba->gba_pins;
	sc->sc_npins = gba->gba_npins;

	printf(": %d pins\n", sc->sc_npins);

	/*
	 * Attach all devices that can be connected to the GPIO pins
	 * described in the kernel configuration file.
	 */
	config_search(gpio_search, self, sc);
}

int
gpio_detach(struct device *self, int flags)
{
	int maj, mn;

	/* Locate the major number */
	for (maj = 0; maj < nchrdev; maj++)
		if (cdevsw[maj].d_open == gpioopen)
			break;

	/* Nuke the vnodes for any open instances (calls close) */
	mn = self->dv_unit;
	vdevgone(maj, mn, mn, VCHR);

	return (0);
}

int
gpio_search(struct device *parent, void *arg, void *aux)
{
	struct cfdata *cf = arg;
	struct gpio_attach_args ga;

	ga.ga_gpio = aux;
	ga.ga_offset = cf->cf_loc[0];
	ga.ga_mask = cf->cf_loc[1];

	if (cf->cf_attach->ca_match(parent, cf, &ga) > 0)
		config_attach(parent, cf, &ga, gpio_print);

	return (0);
}

int
gpio_print(void *aux, const char *pnp)
{
	struct gpio_attach_args *ga = aux;
	int i;

	printf(" pins");
	for (i = 0; i < 32; i++)
		if (ga->ga_mask & (1 << i))
			printf(" %d", ga->ga_offset + i);

	return (UNCONF);
}

int
gpiobus_print(void *aux, const char *pnp)
{
	struct gpiobus_attach_args *gba = aux;

	if (pnp != NULL)
		printf("%s at %s", gba->gba_name, pnp);

	return (UNCONF);
}

int
gpio_pin_map(void *gpio, int offset, u_int32_t mask, struct gpio_pinmap *map)
{
	struct gpio_softc *sc = gpio;
	int npins, pin, i;

	npins = gpio_npins(mask);
	if (npins > sc->sc_npins)
		return (1);

	for (npins = 0, i = 0; i < 32; i++)
		if (mask & (1 << i)) {
			pin = offset + i;
			if (pin < 0 || pin >= sc->sc_npins)
				return (1);
			if (sc->sc_pins[pin].pin_mapped)
				return (1);
			sc->sc_pins[pin].pin_mapped = 1;
			map->pm_map[npins++] = pin;
		}
	map->pm_size = npins;

	return (0);
}

void
gpio_pin_unmap(void *gpio, struct gpio_pinmap *map)
{
	struct gpio_softc *sc = gpio;
	int pin, i;

	for (i = 0; i < map->pm_size; i++) {
		pin = map->pm_map[i];
		sc->sc_pins[pin].pin_mapped = 0;
	}
}

int
gpio_pin_read(void *gpio, struct gpio_pinmap *map, int pin)
{
	struct gpio_softc *sc = gpio;

	return (gpiobus_pin_read(sc->sc_gc, map->pm_map[pin]));
}

void
gpio_pin_write(void *gpio, struct gpio_pinmap *map, int pin, int value)
{
	struct gpio_softc *sc = gpio;

	return (gpiobus_pin_write(sc->sc_gc, map->pm_map[pin], value));
}

void
gpio_pin_ctl(void *gpio, struct gpio_pinmap *map, int pin, int flags)
{
	struct gpio_softc *sc = gpio;

	return (gpiobus_pin_ctl(sc->sc_gc, map->pm_map[pin], flags));
}

int
gpio_pin_caps(void *gpio, struct gpio_pinmap *map, int pin)
{
	struct gpio_softc *sc = gpio;

	return (sc->sc_pins[map->pm_map[pin]].pin_caps);
}

int
gpio_npins(u_int32_t mask)
{
	int npins, i;

	for (npins = 0, i = 0; i < 32; i++)
		if (mask & (1 << i))
			npins++;

	return (npins);
}

int
gpioopen(dev_t dev, int flag, int mode, struct proc *p)
{
	struct gpio_softc *sc;

	sc = (struct gpio_softc *)device_lookup(&gpio_cd, minor(dev));
	if (sc == NULL)
		return (ENXIO);

	if (sc->sc_opened)
		return (EBUSY);
	sc->sc_opened = 1;

	return (0);
}

int
gpioclose(dev_t dev, int flag, int mode, struct proc *p)
{
	struct gpio_softc *sc;

	sc = (struct gpio_softc *)device_lookup(&gpio_cd, minor(dev));
	sc->sc_opened = 0;

	return (0);
}

int
gpioioctl(dev_t dev, u_long cmd, caddr_t data, int flag, struct proc *p)
{
	struct gpio_softc *sc;
	gpio_chipset_tag_t gc;
	struct gpio_info *info;
	struct gpio_pin_op *op;
	struct gpio_pin_ctl *ctl;
	int pin, value, flags;

	sc = (struct gpio_softc *)device_lookup(&gpio_cd, minor(dev));
	gc = sc->sc_gc;

	switch (cmd) {
	case GPIOINFO:
		info = (struct gpio_info *)data;

		info->gpio_npins = sc->sc_npins;
		break;
	case GPIOPINREAD:
		op = (struct gpio_pin_op *)data;

		pin = op->gp_pin;
		if (pin < 0 || pin >= sc->sc_npins)
			return (EINVAL);

		/* return read value */
		op->gp_value = gpiobus_pin_read(gc, pin);
		break;
	case GPIOPINWRITE:
		if ((flag & FWRITE) == 0)
			return (EBADF);

		op = (struct gpio_pin_op *)data;

		pin = op->gp_pin;
		if (pin < 0 || pin >= sc->sc_npins)
			return (EINVAL);
		if (sc->sc_pins[pin].pin_mapped)
			return (EBUSY);

		value = op->gp_value;
		if (value != GPIO_PIN_LOW && value != GPIO_PIN_HIGH)
			return (EINVAL);

		gpiobus_pin_write(gc, pin, value);
		/* return old value */
		op->gp_value = sc->sc_pins[pin].pin_state;
		/* update current value */
		sc->sc_pins[pin].pin_state = value;
		break;
	case GPIOPINTOGGLE:
		if ((flag & FWRITE) == 0)
			return (EBADF);

		op = (struct gpio_pin_op *)data;

		pin = op->gp_pin;
		if (pin < 0 || pin >= sc->sc_npins)
			return (EINVAL);
		if (sc->sc_pins[pin].pin_mapped)
			return (EBUSY);

		value = (sc->sc_pins[pin].pin_state == GPIO_PIN_LOW ?
		    GPIO_PIN_HIGH : GPIO_PIN_LOW);
		gpiobus_pin_write(gc, pin, value);
		/* return old value */
		op->gp_value = sc->sc_pins[pin].pin_state;
		/* update current value */
		sc->sc_pins[pin].pin_state = value;
		break;
	case GPIOPINCTL:
		if ((flag & FWRITE) == 0)
			return (EBADF);

		ctl = (struct gpio_pin_ctl *)data;

		pin = ctl->gp_pin;
		if (pin < 0 || pin >= sc->sc_npins)
			return (EINVAL);
		if (sc->sc_pins[pin].pin_mapped)
			return (EBUSY);

		flags = ctl->gp_flags;
		/* check that the controller supports all requested flags */
		if ((flags & sc->sc_pins[pin].pin_caps) != flags)
			return (ENODEV);

		ctl->gp_caps = sc->sc_pins[pin].pin_caps;
		/* return old value */
		ctl->gp_flags = sc->sc_pins[pin].pin_flags;
		if (flags > 0) {
			gpiobus_pin_ctl(gc, pin, flags);
			/* update current value */
			sc->sc_pins[pin].pin_flags = flags;
		}
		break;
	default:
		return (ENOTTY);
	}

	return (0);
}
