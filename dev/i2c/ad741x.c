/*	$OpenBSD: ad741x.c,v 1.1 2005/11/15 16:24:49 deraadt Exp $	*/

/*
 * Copyright (c) 2005 Theo de Raadt
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

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/device.h>
#include <sys/sensors.h>

#include <dev/i2c/i2cvar.h>

/* AD741x registers */
#define AD741X_TEMP	0x00
#define AD741X_CONFIG	0x01
#define AD741X_THYST	0x02
#define AD741X_TOTI	0x03
#define AD741X_ADC	0x04
#define AD741X_CONFIG2	0x05

#define AD741X_CONFMASK	0xe0

/* Sensors */
#define ADC_TEMP		0
#define ADC_ADC0		1
#define ADC_ADC1		2
#define ADC_ADC2		3	
#define ADC_ADC3		4
#define ADC_NUM_SENSORS		5

struct adc_softc {
	struct device sc_dev;
	i2c_tag_t sc_tag;
	i2c_addr_t sc_addr;

	struct sensor sc_sensor[ADC_NUM_SENSORS];
};

int	adc_match(struct device *, void *, void *);
void	adc_attach(struct device *, struct device *, void *);
int	adc_check(struct i2c_attach_args *, u_int8_t *, u_int8_t *);
void	adc_refresh(void *);

struct cfattach adc_ca = {
	sizeof(struct adc_softc), adc_match, adc_attach
};

struct cfdriver adc_cd = {
	NULL, "adc", DV_DULL
};

int
adc_match(struct device *parent, void *match, void *aux)
{
	struct i2c_attach_args *ia = aux;

	if (ia->ia_compat) {
		if (strcmp(ia->ia_compat, "ad7417") == 0)
			return (1);
		return (0);
	}
	return (1);
}

void
adc_attach(struct device *parent, struct device *self, void *aux)
{
	struct adc_softc *sc = (struct adc_softc *)self;
	struct i2c_attach_args *ia = aux;
	u_int8_t cmd, data;
	int i;

	sc->sc_tag = ia->ia_tag;
	sc->sc_addr = ia->ia_addr;

	cmd = AD741X_CONFIG;
	data = 0;
	if (iic_exec(sc->sc_tag, I2C_OP_WRITE_WITH_STOP,
	    sc->sc_addr, &cmd, sizeof cmd, &data, sizeof data, 0)) {
		printf(", config reset failed\n");
		return;
	}

	cmd = AD741X_CONFIG2;
	data = 0;
	if (iic_exec(sc->sc_tag, I2C_OP_WRITE_WITH_STOP,
	    sc->sc_addr, &cmd, sizeof cmd, &data, sizeof data, 0)) {
		printf(", config2 reset failed\n");
		return;
	}

	/* Initialize sensor data. */
	for (i = 0; i < ADC_NUM_SENSORS; i++)
		strlcpy(sc->sc_sensor[i].device, sc->sc_dev.dv_xname,
		    sizeof(sc->sc_sensor[i].device));

	sc->sc_sensor[ADC_TEMP].type = SENSOR_TEMP;
	strlcpy(sc->sc_sensor[ADC_TEMP].desc, "Internal",
	    sizeof(sc->sc_sensor[ADC_TEMP].desc));

	sc->sc_sensor[ADC_ADC0].type = SENSOR_INTEGER;
	strlcpy(sc->sc_sensor[ADC_ADC0].desc, "adc0",
	    sizeof(sc->sc_sensor[ADC_ADC0].desc));
	sc->sc_sensor[ADC_ADC1].type = SENSOR_INTEGER;
	strlcpy(sc->sc_sensor[ADC_ADC1].desc, "adc1",
	    sizeof(sc->sc_sensor[ADC_ADC1].desc));
	sc->sc_sensor[ADC_ADC2].type = SENSOR_INTEGER;
	strlcpy(sc->sc_sensor[ADC_ADC2].desc, "adc2",
	    sizeof(sc->sc_sensor[ADC_ADC2].desc));
	sc->sc_sensor[ADC_ADC3].type = SENSOR_INTEGER;
	strlcpy(sc->sc_sensor[ADC_ADC3].desc, "adc3",
	    sizeof(sc->sc_sensor[ADC_ADC3].desc));

	if (sensor_task_register(sc, adc_refresh, 5)) {
		printf(", unable to register update task\n");
		return;
	}

	for (i = 0; i < ADC_NUM_SENSORS; i++)
		SENSOR_ADD(&sc->sc_sensor[i]);

	printf("\n");
}

void	adc_readport(struct adc_softc *, u_int8_t, u_int8_t, int);

void
adc_refresh(void *arg)
{
	struct adc_softc *sc = arg;
	u_int8_t cmd, data[2], reg;
	int i;

	iic_acquire_bus(sc->sc_tag, 0);

	cmd = AD741X_CONFIG;
	if (iic_exec(sc->sc_tag, I2C_OP_READ_WITH_STOP,
	    sc->sc_addr, &cmd, sizeof cmd, &reg, sizeof reg, 0))
		goto fail;
	reg = (reg & AD741X_CONFMASK) | (0 << 5);
	if (iic_exec(sc->sc_tag, I2C_OP_WRITE_WITH_STOP,
	    sc->sc_addr, &cmd, sizeof cmd, &reg, sizeof reg, 0))
		goto fail;
	cmd = AD741X_TEMP;
	if (iic_exec(sc->sc_tag, I2C_OP_READ_WITH_STOP,
	    sc->sc_addr, &cmd, sizeof cmd, &data, sizeof data, 0))
		goto fail;
	sc->sc_sensor[ADC_TEMP].value = 273150000 +
	    ((data[0] << 8 | data[1]) >> 6) * 250000;

	for (i = 0; i < 4; i++) {
		reg = (reg & AD741X_CONFMASK) | (i << 5);
		if (iic_exec(sc->sc_tag, I2C_OP_WRITE_WITH_STOP,
		    sc->sc_addr, &cmd, sizeof cmd, &reg, sizeof reg, 0))
			goto fail;
		cmd = AD741X_ADC;
		if (iic_exec(sc->sc_tag, I2C_OP_READ_WITH_STOP,
		    sc->sc_addr, &cmd, sizeof cmd, &data, sizeof data, 0))
			goto fail;
		sc->sc_sensor[ADC_ADC0 + i].value =
		    (data[0] << 8 | data[1]) >> 6;
	}

fail:
	iic_release_bus(sc->sc_tag, 0);
}
