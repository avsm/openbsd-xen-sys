/*	$OpenBSD: ciphy.c,v 1.8 2005/08/02 03:46:07 pvalchev Exp $	*/
/*	$FreeBSD: ciphy.c,v 1.1 2004/09/10 20:57:45 wpaul Exp $	*/
/*
 * Copyright (c) 2004
 *	Bill Paul <wpaul@windriver.com>.  All rights reserved.
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
 *	This product includes software developed by Bill Paul.
 * 4. Neither the name of the author nor the names of any co-contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY Bill Paul AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL Bill Paul OR THE VOICES IN HIS HEAD
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Driver for the Cicada CS8201 10/100/1000 copper PHY.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/device.h>
#include <sys/socket.h>
#include <sys/errno.h>

#include <net/if.h>
#include <net/if_media.h>

#ifdef INET
#include <netinet/in.h>
#include <netinet/if_ether.h>
#endif

#include <dev/pci/pcivar.h>

#include <dev/mii/mii.h>
#include <dev/mii/miivar.h>
#include <dev/mii/miidevs.h>

#include <dev/mii/ciphyreg.h>

#include <machine/bus.h>

int	ciphymatch(struct device *, void *, void *);
void	ciphyattach(struct device *, struct device *, void *);

struct cfattach ciphy_ca = {
	sizeof(struct mii_softc),
	ciphymatch,
	ciphyattach,
	mii_phy_detach,
	mii_phy_activate
};

struct cfdriver ciphy_cd = {
	NULL, "ciphy", DV_DULL
};

int	ciphy_service(struct mii_softc *, struct mii_data *, int);
void	ciphy_status(struct mii_softc *);
void	ciphy_reset(struct mii_softc *);
void	ciphy_fixup(struct mii_softc *);

const struct mii_phy_funcs ciphy_funcs = {
	ciphy_service, ciphy_status, ciphy_reset,
};

static const struct mii_phydesc ciphys[] = {
	{ MII_OUI_CICADA,		MII_MODEL_CICADA_CS8201,
	  MII_STR_CICADA_CS8201 },
	{ MII_OUI_CICADA,		MII_MODEL_CICADA_CS8201A,
	  MII_STR_CICADA_CS8201A },
	{ MII_OUI_CICADA,		MII_MODEL_CICADA_CS8201B,
	  MII_STR_CICADA_CS8201B },
	{ MII_OUI_xxCICADA,		MII_MODEL_xxCICADA_CS8201B,
	  MII_STR_xxCICADA_CS8201B },

	{ 0,			0,
	  NULL },
};

int
ciphymatch(struct device *parent, void *match, void *aux)
{
	struct mii_attach_args *ma = aux;

	if (mii_phy_match(ma, ciphys) != NULL)
		return (10);

	return (0);
}

void
ciphyattach(struct device *parent, struct device *self, void *aux)
{
	struct mii_softc *sc = (struct mii_softc *)self;
	struct mii_attach_args *ma = aux;
	struct mii_data *mii = ma->mii_data;
	const struct mii_phydesc *mpd;

	mpd = mii_phy_match(ma, ciphys);
	printf(": %s, rev. %d\n", mpd->mpd_name, MII_REV(ma->mii_id2));

	sc->mii_inst = mii->mii_instance;
	sc->mii_phy = ma->mii_phyno;
	sc->mii_funcs = &ciphy_funcs;
	sc->mii_pdata = mii;
	sc->mii_flags = ma->mii_flags;
	sc->mii_anegticks = MII_ANEGTICKS;

	sc->mii_flags |= MIIF_NOISOLATE;

	PHY_RESET(sc);

	sc->mii_capabilities =
	    PHY_READ(sc, MII_BMSR) & ma->mii_capmask;
	if (sc->mii_capabilities & BMSR_EXTSTAT)
		sc->mii_extcapabilities = PHY_READ(sc, MII_EXTSR);
	if ((sc->mii_capabilities & BMSR_MEDIAMASK) ||
	    (sc->mii_capabilities & EXTSR_MEDIAMASK))
		mii_phy_add_media(sc);
}

int
ciphy_service(struct mii_softc *sc, struct mii_data *mii, int cmd)
{
	struct ifmedia_entry *ife = mii->mii_media.ifm_cur;
	int reg, speed, gig;

	switch (cmd) {
	case MII_POLLSTAT:
		/*
		 * If we're not polling our PHY instance, just return.
		 */
		if (IFM_INST(ife->ifm_media) != sc->mii_inst)
			return (0);
		break;

	case MII_MEDIACHG:
		/*
		 * If the media indicates a different PHY instance,
		 * isolate ourselves.
		 */
		if (IFM_INST(ife->ifm_media) != sc->mii_inst) {
			reg = PHY_READ(sc, MII_BMCR);
			PHY_WRITE(sc, MII_BMCR, reg | BMCR_ISO);
			return (0);
		}

		/*
		 * If the interface is not up, don't do anything.
		 */
		if ((mii->mii_ifp->if_flags & IFF_UP) == 0)
			break;

		ciphy_fixup(sc);	/* XXX hardware bug work-around */

		switch (IFM_SUBTYPE(ife->ifm_media)) {
		case IFM_AUTO:
#ifdef foo
			/*
			 * If we're already in auto mode, just return.
			 */
			if (PHY_READ(sc, CIPHY_MII_BMCR) & CIPHY_BMCR_AUTOEN)
				return (0);
#endif
			if (mii_phy_auto(sc, 0) == EJUSTRETURN)
				return (0);
			break;
		case IFM_1000_T:
			speed = CIPHY_S1000;
			goto setit;
		case IFM_100_TX:
			speed = CIPHY_S100;
			goto setit;
		case IFM_10_T:
			speed = CIPHY_S10;
setit:
			if ((ife->ifm_media & IFM_GMASK) == IFM_FDX) {
				speed |= CIPHY_BMCR_FDX;
				gig = CIPHY_1000CTL_AFD;
			} else {
				gig = CIPHY_1000CTL_AHD;
			}

			PHY_WRITE(sc, CIPHY_MII_1000CTL, 0);
			PHY_WRITE(sc, CIPHY_MII_BMCR, speed);
			PHY_WRITE(sc, CIPHY_MII_ANAR, CIPHY_SEL_TYPE);

			if (IFM_SUBTYPE(ife->ifm_media) != IFM_1000_T) 
				break;

			PHY_WRITE(sc, CIPHY_MII_1000CTL, gig);
			PHY_WRITE(sc, CIPHY_MII_BMCR,
			    speed|CIPHY_BMCR_AUTOEN|CIPHY_BMCR_STARTNEG);

			/*
			 * When setting the link manually, one side must
			 * be the master and the other the slave. However
			 * ifmedia doesn't give us a good way to specify
			 * this, so we fake it by using one of the LINK
			 * flags. If LINK0 is set, we program the PHY to
			 * be a master, otherwise it's a slave.
			 */
			if ((mii->mii_ifp->if_flags & IFF_LINK0)) {
				PHY_WRITE(sc, CIPHY_MII_1000CTL,
				    gig|CIPHY_1000CTL_MSE|CIPHY_1000CTL_MSC);
			} else {
				PHY_WRITE(sc, CIPHY_MII_1000CTL,
				    gig|CIPHY_1000CTL_MSE);
			}
			break;
		case IFM_NONE:
			PHY_WRITE(sc, MII_BMCR, BMCR_ISO|BMCR_PDOWN);
			break;
		case IFM_100_T4:
		default:
			return (EINVAL);
		}
		break;

	case MII_TICK:
		/*
		 * If we're not currently selected, just return.
		 */
		if (IFM_INST(ife->ifm_media) != sc->mii_inst)
			return (0);

		if (mii_phy_tick(sc) == EJUSTRETURN)
			return (0);
		break;
	}

	/* Update the media status. */
	ciphy_status(sc);

	/*
	 * Callback if something changed. Note that we need to poke
	 * apply fixups for certain PHY revs.
	 */
	if (sc->mii_media_active != mii->mii_media_active || 
	    sc->mii_media_status != mii->mii_media_status ||
	    cmd == MII_MEDIACHG) {
		ciphy_fixup(sc);
	}
	mii_phy_update(sc, cmd);
	return (0);
}

void
ciphy_status(struct mii_softc *sc)
{
	struct mii_data *mii = sc->mii_pdata;
	int bmsr, bmcr;

	mii->mii_media_status = IFM_AVALID;
	mii->mii_media_active = IFM_ETHER;

	bmsr = PHY_READ(sc, MII_BMSR) | PHY_READ(sc, MII_BMSR);

	if (bmsr & BMSR_LINK)
		mii->mii_media_status |= IFM_ACTIVE;

	bmcr = PHY_READ(sc, CIPHY_MII_BMCR);

	if (bmcr & CIPHY_BMCR_LOOP)
		mii->mii_media_active |= IFM_LOOP;

	if (bmcr & CIPHY_BMCR_AUTOEN) {
		if ((bmsr & CIPHY_BMSR_ACOMP) == 0) {
			/* Erg, still trying, I guess... */
			mii->mii_media_active |= IFM_NONE;
			return;
		}
	}

	bmsr = PHY_READ(sc, CIPHY_MII_AUXCSR);
	switch (bmsr & CIPHY_AUXCSR_SPEED) {
	case CIPHY_SPEED10:
		mii->mii_media_active |= IFM_10_T;
		break;
	case CIPHY_SPEED100:
		mii->mii_media_active |= IFM_100_TX;
		break;
	case CIPHY_SPEED1000:
		mii->mii_media_active |= IFM_1000_T;
		break;
	default:
		printf("%s: unknown PHY speed %x\n",
		    sc->mii_dev.dv_xname, bmsr & CIPHY_AUXCSR_SPEED);
		break;
	}

	if (bmsr & CIPHY_AUXCSR_FDX)
		mii->mii_media_active |= IFM_FDX;
}

void
ciphy_reset(struct mii_softc *sc)
{
	mii_phy_reset(sc);
	DELAY(1000);
}

#define PHY_SETBIT(x, y, z) \
	PHY_WRITE(x, y, (PHY_READ(x, y) | (z)))
#define PHY_CLRBIT(x, y, z) \
	PHY_WRITE(x, y, (PHY_READ(x, y) & ~(z)))

void
ciphy_fixup(struct mii_softc *sc)
{
	uint16_t		model;
	uint16_t		status, speed;

	model = MII_MODEL(PHY_READ(sc, CIPHY_MII_PHYIDR2));
	status = PHY_READ(sc, CIPHY_MII_AUXCSR);
	speed = status & CIPHY_AUXCSR_SPEED;

	switch (model) {
	case MII_MODEL_CICADA_CS8201:

		/* Turn off "aux mode" (whatever that means) */
		PHY_SETBIT(sc, CIPHY_MII_AUXCSR, CIPHY_AUXCSR_MDPPS);

		/*
		 * Work around speed polling bug in VT3119/VT3216
		 * when using MII in full duplex mode.
		 */
		if ((speed == CIPHY_SPEED10 || speed == CIPHY_SPEED100) &&
		    (status & CIPHY_AUXCSR_FDX)) {
			PHY_SETBIT(sc, CIPHY_MII_10BTCSR, CIPHY_10BTCSR_ECHO);
		} else {
			PHY_CLRBIT(sc, CIPHY_MII_10BTCSR, CIPHY_10BTCSR_ECHO);
		}

		/* Enable link/activity LED blink. */
		PHY_SETBIT(sc, CIPHY_MII_LED, CIPHY_LED_LINKACTBLINK);

		break;

	case MII_MODEL_CICADA_CS8201A:
	case MII_MODEL_CICADA_CS8201B:

		/*
		 * Work around speed polling bug in VT3119/VT3216
		 * when using MII in full duplex mode.
		 */
		if ((speed == CIPHY_SPEED10 || speed == CIPHY_SPEED100) &&
		    (status & CIPHY_AUXCSR_FDX)) {
			PHY_SETBIT(sc, CIPHY_MII_10BTCSR, CIPHY_10BTCSR_ECHO);
		} else {
			PHY_CLRBIT(sc, CIPHY_MII_10BTCSR, CIPHY_10BTCSR_ECHO);
		}

		break;
	default:
		printf("%s: unknown CICADA PHY model %x\n",
		    sc->mii_dev.dv_xname, model);
		break;
	}
}
