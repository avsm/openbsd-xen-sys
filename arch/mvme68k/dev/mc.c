/*	$OpenBSD: mc.c,v 1.16 2004/07/30 22:29:45 miod Exp $ */

/*
 * Copyright (c) 1995 Theo de Raadt
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

/*
 * VME162/VME172 MCchip
 */
#include <sys/param.h>
#include <sys/conf.h>
#include <sys/ioctl.h>
#include <sys/proc.h>
#include <sys/user.h>
#include <sys/tty.h>
#include <sys/uio.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/syslog.h>
#include <sys/fcntl.h>
#include <sys/device.h>
#include <machine/cpu.h>
#include <machine/autoconf.h>
#include <dev/cons.h>

#include <mvme68k/dev/mcreg.h>

struct mcsoftc {
	struct device	sc_dev;
	vaddr_t		sc_vaddr;
	paddr_t		sc_paddr;
	struct mcreg	*sc_mc;
	struct intrhand	sc_nmiih;
};

void mcattach(struct device *, struct device *, void *);
int  mcmatch(struct device *, void *, void *);
int  mcabort(void *);
int  mc_print(void *, const char *);
int  mc_scan(struct device *, void *, void *);

struct cfattach mc_ca = {
	sizeof(struct mcsoftc), mcmatch, mcattach
};

struct cfdriver mc_cd = {
	NULL, "mc", DV_DULL
};

struct mcreg *sys_mc = NULL;

int
mcmatch(parent, vcf, args)
	struct device *parent;
	void *vcf, *args;
{
	struct confargs *ca = args;
	struct mcreg *mc = (struct mcreg *)(IIOV(ca->ca_paddr) + MC_MCCHIP_OFF);

	if ((cputyp != CPU_172 && cputyp != CPU_162) ||
	    badvaddr((vaddr_t)mc, 1) || mc->mc_chipid != MC_CHIPID)
		return (0);
	return (1);
}

int
mc_print(args, bus)
	void *args;
	const char *bus;
{
	struct confargs *ca = args;

	if (ca->ca_offset != -1)
		printf(" offset 0x%x", ca->ca_offset);
	if (ca->ca_ipl > 0)
		printf(" ipl %d", ca->ca_ipl);
	return (UNCONF);
}

int
mc_scan(parent, child, args)
	struct device *parent;
	void *child, *args;
{
	struct cfdata *cf = child;
	struct mcsoftc *sc = (struct mcsoftc *)parent;
	struct confargs oca;

	bzero(&oca, sizeof oca);
	oca.ca_offset = cf->cf_loc[0];
	oca.ca_ipl = cf->cf_loc[1];
	if (oca.ca_offset != -1 && ISIIOVA(sc->sc_vaddr + oca.ca_offset)) {
		oca.ca_vaddr = sc->sc_vaddr + oca.ca_offset;
		oca.ca_paddr = sc->sc_paddr + oca.ca_offset;
	} else {
		oca.ca_vaddr = (vaddr_t)-1;
		oca.ca_paddr = (paddr_t)-1;
	}
	oca.ca_bustype = BUS_MC;
	oca.ca_name = cf->cf_driver->cd_name;
	if ((*cf->cf_attach->ca_match)(parent, cf, &oca) == 0)
		return (0);
	config_attach(parent, cf, &oca, mc_print);
	return (1);
}

void
mcattach(parent, self, args)
	struct device *parent, *self;
	void *args;
{
	struct confargs *ca = args;
	struct mcsoftc *sc = (struct mcsoftc *)self;

	if (sys_mc)
		panic("mc already attached!");

	/*
	 * since we know ourself to land in intiobase land,
	 * we must adjust our address
	 */
	sc->sc_paddr = ca->ca_paddr;
	sc->sc_vaddr = IIOV(sc->sc_paddr);
	sc->sc_mc = (struct mcreg *)(sc->sc_vaddr + MC_MCCHIP_OFF);
	sys_mc = sc->sc_mc;

	printf(": rev %d\n", sc->sc_mc->mc_chiprev);

	sc->sc_nmiih.ih_fn = mcabort;
	sc->sc_nmiih.ih_ipl = 7;
	sc->sc_nmiih.ih_wantframe = 1;
	mcintr_establish(MCV_ABORT, &sc->sc_nmiih, self->dv_xname);

	sc->sc_mc->mc_abortirq = 7 | MC_IRQ_IEN | MC_IRQ_ICLR;
	sc->sc_mc->mc_vecbase = MC_VECBASE;

	sc->sc_mc->mc_genctl |= MC_GENCTL_IEN;		/* global irq enable */

	config_search(mc_scan, self, args);
}

/*
 * MC interrupts land in a MC_NVEC sized hole starting at MC_VECBASE
 */
int
mcintr_establish(vec, ih, name)
	int vec;
	struct intrhand *ih;
	const char *name;
{
#ifdef DIAGNOSTIC
	if (vec < 0 || vec >= MC_NVEC)
		panic("mcintr_establish: illegal vector for %s: 0x%x",
		    name, vec);
#endif

	return intr_establish(MC_VECBASE + vec, ih, name);
}

int
mcabort(frame)
	void *frame;
{
	/* wait for it to debounce */
	while (sys_mc->mc_abortirq & MC_ABORT_ABS)
		;

	sys_mc->mc_abortirq = sys_mc->mc_abortirq | MC_IRQ_ICLR;

	nmihand(frame);
	return (1);
}

#include "flash.h"

#if NFLASH > 0
void
mc_enableflashwrite(on)
	int on;
{
	struct mcsoftc *sc = (struct mcsoftc *) mc_cd.cd_devs[0];
	volatile u_char *ena, x;
	/* 
	 * Check MC chip revision, as the way to enable flash writes
	 * has been changed from a memory location in BBRAM to a 
	 * bit in the Flash Control Reg.  XXX - smurph
	 */
	if (sc->sc_mc->mc_chiprev == 0x01) {
		if (on) 
			sc->sc_mc->mc_flashctl |= MC_FLASHCTL_WRITE;
		else
			sc->sc_mc->mc_flashctl &= ~MC_FLASHCTL_WRITE;
	} else {
		ena = (u_char *)sc->sc_vaddr +
			 (on ? MC_ENAFLASHWRITE_OFFSET : MC_DISFLASHWRITE_OFFSET);
		x = *ena;
	}
}
/*
 * Function to check if we booted from flash or prom.
 * If we booted from PROM, flash mem is available.
 */
int 
mc_hasflash(void)
{
	struct mcsoftc *sc = (struct mcsoftc *) mc_cd.cd_devs[0];
   if (sc->sc_mc->mc_input & MC_INPUT_PROM)
		return 1;
	else 
		return 0;
}
#endif
