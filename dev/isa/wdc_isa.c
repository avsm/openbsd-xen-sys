/*      $OpenBSD: wdc_isa.c,v 1.10 2003/10/17 08:14:09 grange Exp $     */
/*	$NetBSD: wdc_isa.c,v 1.15 1999/05/19 14:41:25 bouyer Exp $ */

/*-
 * Copyright (c) 1998 The NetBSD Foundation, Inc.
 * All rights reserved.
 *
 * This code is derived from software contributed to The NetBSD Foundation
 * by Charles M. Hannum and by Onno van der Linden.
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

#include <sys/types.h>
#include <sys/param.h>
#include <sys/systm.h>
#include <sys/device.h>
#include <sys/malloc.h>

#include <machine/bus.h>
#include <machine/intr.h>

#include <dev/isa/isavar.h>
#include <dev/isa/isadmavar.h>

#include <dev/ata/atavar.h>
#include <dev/ic/wdcvar.h>

#ifdef __OpenBSD__
#include "isadma.h"
#else
#define NISADMA 1
#endif

#define	WDC_ISA_REG_NPORTS	8
#define	WDC_ISA_AUXREG_OFFSET	0x206
#define	WDC_ISA_AUXREG_NPORTS	1 /* XXX "fdc" owns ports 0x3f7/0x377 */

/* options passed via the 'flags' config keyword */
#define WDC_OPTIONS_32	0x01 /* try to use 32bit data I/O */

struct wdc_isa_softc {
	struct	wdc_softc sc_wdcdev;
	struct	channel_softc *wdc_chanptr;
	struct	channel_softc wdc_channel;
#ifdef __OpenBSD__
	struct  device *sc_isa;
#endif
	isa_chipset_tag_t sc_ic;
	void	*sc_ih;
	int	sc_drq;
};

#ifndef __OpenBSD__
int	wdc_isa_probe(struct device *, struct cfdata *, void *);
#else
int	wdc_isa_probe(struct device *, void *, void *);
#endif
void	wdc_isa_attach(struct device *, struct device *, void *);

struct cfattach wdc_isa_ca = {
	sizeof(struct wdc_isa_softc), wdc_isa_probe, wdc_isa_attach
};

#if NISADMA > 0
static void	wdc_isa_dma_setup(struct wdc_isa_softc *);
static int	wdc_isa_dma_init(void *, int, int, void *, size_t, int);
static void 	wdc_isa_dma_start(void *, int, int);
static int	wdc_isa_dma_finish(void *, int, int, int);
#endif	/* NISADMA > 0 */

int
wdc_isa_probe(parent, match, aux)
	struct device *parent;
#ifndef __OpenBSD__
	struct cfdata *match;
#else
	void *match;
#endif
	void *aux;
{
	struct channel_softc ch;
	struct isa_attach_args *ia = aux;
	struct cfdata *cf = match;
	int result = 0;

	bzero(&ch, sizeof ch);
	ch.cmd_iot = ia->ia_iot;
	if (bus_space_map(ch.cmd_iot, ia->ia_iobase, WDC_ISA_REG_NPORTS, 0,
	    &ch.cmd_ioh))
		goto out;

	ch.ctl_iot = ia->ia_iot;
	if (bus_space_map(ch.ctl_iot, ia->ia_iobase + WDC_ISA_AUXREG_OFFSET,
	    WDC_ISA_AUXREG_NPORTS, 0, &ch.ctl_ioh))
		goto outunmap;

	if (cf->cf_flags & WDC_OPTION_PROBE_VERBOSE)
		ch.ch_flags |= WDCF_VERBOSE_PROBE;

	result = wdcprobe(&ch);
	if (result) {
		ia->ia_iosize = WDC_ISA_REG_NPORTS;
		ia->ia_msize = 0;
	}

	bus_space_unmap(ch.ctl_iot, ch.ctl_ioh, WDC_ISA_AUXREG_NPORTS);
outunmap:
	bus_space_unmap(ch.cmd_iot, ch.cmd_ioh, WDC_ISA_REG_NPORTS);
out:
	return (result);
}

void
wdc_isa_attach(parent, self, aux)
	struct device *parent, *self;
	void *aux;
{
	struct wdc_isa_softc *sc = (void *)self;
	struct isa_attach_args *ia = aux;

	printf("\n");

	sc->wdc_channel.cmd_iot = ia->ia_iot;
	sc->wdc_channel.ctl_iot = ia->ia_iot;
	sc->sc_ic = ia->ia_ic;
	sc->sc_isa = parent;
	if (bus_space_map(sc->wdc_channel.cmd_iot, ia->ia_iobase,
	    WDC_ISA_REG_NPORTS, 0, &sc->wdc_channel.cmd_ioh) ||
	    bus_space_map(sc->wdc_channel.ctl_iot,
	      ia->ia_iobase + WDC_ISA_AUXREG_OFFSET, WDC_ISA_AUXREG_NPORTS,
	      0, &sc->wdc_channel.ctl_ioh)) {
		printf("%s: couldn't map registers\n",
		    sc->sc_wdcdev.sc_dev.dv_xname);
	}
	sc->wdc_channel.data32iot = sc->wdc_channel.cmd_iot;
	sc->wdc_channel.data32ioh = sc->wdc_channel.cmd_ioh;

#ifdef __OpenBSD__
	sc->sc_ih = isa_intr_establish(ia->ia_ic, ia->ia_irq, IST_EDGE,
	    IPL_BIO, wdcintr, &sc->wdc_channel, sc->sc_wdcdev.sc_dev.dv_xname);
#else
	sc->sc_ih = isa_intr_establish(ia->ia_ic, ia->ia_irq, IST_EDGE,
	    IPL_BIO, wdcintr, &sc->wdc_channel);
#endif
	if (ia->ia_drq != DRQUNK) {
#if NISADMA > 0
		sc->sc_drq = ia->ia_drq;

		sc->sc_wdcdev.cap |= WDC_CAPABILITY_DMA;
		sc->sc_wdcdev.dma_arg = sc;
		sc->sc_wdcdev.dma_init = wdc_isa_dma_init;
		sc->sc_wdcdev.dma_start = wdc_isa_dma_start;
		sc->sc_wdcdev.dma_finish = wdc_isa_dma_finish;
		wdc_isa_dma_setup(sc);
#else	/* NISADMA > 0 */
		printf("%s: ignoring drq, isa dma not supported",
		    sc->sc_wdcdev.sc_dev.dv_xname);
#endif	/* NISADMA > 0 */
	}
	sc->sc_wdcdev.cap |= WDC_CAPABILITY_DATA16 | WDC_CAPABILITY_PREATA;
	if (sc->sc_wdcdev.sc_dev.dv_cfdata->cf_flags & WDC_OPTIONS_32)
		sc->sc_wdcdev.cap |= WDC_CAPABILITY_DATA32;
	sc->sc_wdcdev.PIO_cap = 0;
	sc->wdc_chanptr = &sc->wdc_channel;
	sc->sc_wdcdev.channels = &sc->wdc_chanptr;
	sc->sc_wdcdev.nchannels = 1;
	sc->wdc_channel.channel = 0;
	sc->wdc_channel.wdc = &sc->sc_wdcdev;
	sc->wdc_channel.ch_queue = malloc(sizeof(struct channel_queue),
	    M_DEVBUF, M_NOWAIT);
	if (sc->wdc_channel.ch_queue == NULL) {
		printf("%s: can't allocate memory for command queue",
		    sc->sc_wdcdev.sc_dev.dv_xname);
		return;
	}
	wdcattach(&sc->wdc_channel);
	wdc_print_current_modes(&sc->wdc_channel);
}

#if NISADMA > 0
static void
wdc_isa_dma_setup(sc)
	struct wdc_isa_softc *sc;
{
#ifndef __OpenBSD__
	if (isa_dmamap_create(sc->sc_ic, sc->sc_drq,
	    MAXPHYS, BUS_DMA_NOWAIT|BUS_DMA_ALLOCNOW)) {
#else
	if (isa_dmamap_create(sc->sc_isa, sc->sc_drq,
	    MAXPHYS, BUS_DMA_NOWAIT|BUS_DMA_ALLOCNOW)) {			      
#endif
		printf("%s: can't create map for drq %d\n",
		    sc->sc_wdcdev.sc_dev.dv_xname, sc->sc_drq);
		sc->sc_wdcdev.cap &= ~WDC_CAPABILITY_DMA;
	}
}

static int
wdc_isa_dma_init(v, channel, drive, databuf, datalen, read)
	void *v;
	int channel, drive;
	void *databuf;
	size_t datalen;
	int read;
{
	struct wdc_isa_softc *sc = v;

#ifndef __OpenBSD__
	isa_dmastart(sc->sc_ic, sc->sc_drq, databuf, datalen, NULL,
	    (read ? DMAMODE_READ : DMAMODE_WRITE) | DMAMODE_DEMAND,
	    BUS_DMA_NOWAIT);
#else
	isa_dmastart(sc->sc_isa, sc->sc_drq, databuf, datalen, NULL,
	    (read ? DMAMODE_READ : DMAMODE_WRITE),
	    BUS_DMA_NOWAIT);
#endif
	return 0;
}

static void
wdc_isa_dma_start(v, channel, drive)
	void *v;
	int channel, drive;
{
	/* nothing to do */
}

static int
wdc_isa_dma_finish(v, channel, drive, force)
	void *v;
	int channel, drive;
	int force;
{
	struct wdc_isa_softc *sc = v;

#ifndef __OpenBSD__
	isa_dmadone(sc->sc_ic, sc->sc_drq);
#else
	isa_dmadone(sc->sc_isa, sc->sc_drq);
#endif
	return 0;
}
#endif	/* NISADMA > 0 */
