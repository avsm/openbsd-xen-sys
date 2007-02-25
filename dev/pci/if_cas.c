/*	$OpenBSD$	*/

/*
 *
 * Copyright (C) 2001 Eduardo Horvath.
 * All rights reserved.
 *
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
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR  ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR  BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 */

/*
 * Driver for Sun Cassini ethernet controllers.
 */

#include "bpfilter.h"

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/timeout.h>
#include <sys/mbuf.h>
#include <sys/syslog.h>
#include <sys/malloc.h>
#include <sys/kernel.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/errno.h>
#include <sys/device.h>

#include <machine/endian.h>

#include <net/if.h>
#include <net/if_dl.h>
#include <net/if_media.h>

#ifdef INET
#include <netinet/in.h>
#include <netinet/if_ether.h>
#endif

#if NBPFILTER > 0
#include <net/bpf.h>
#endif

#include <machine/bus.h>
#include <machine/intr.h>

#include <dev/mii/mii.h>
#include <dev/mii/miivar.h>
#include <dev/mii/mii_bitbang.h>

#include <dev/pci/if_casreg.h>
#include <dev/pci/if_casvar.h>

#include <dev/pci/pcivar.h>
#include <dev/pci/pcireg.h>
#include <dev/pci/pcidevs.h>

#ifdef __sparc64__
#include <dev/ofw/openfirm.h>
#endif

#define TRIES	10000

struct cfdriver cas_cd = {
	NULL, "cas", DV_IFNET
};

int	cas_match(struct device *, void *, void *);
void	cas_attach(struct device *, struct device *, void *);
int	cas_pci_enaddr(struct cas_softc *, struct pci_attach_args *);

struct cfattach cas_ca = {
	sizeof(struct cas_softc), cas_match, cas_attach
};

void		cas_start(struct ifnet *);
void		cas_stop(struct ifnet *, int);
int		cas_ioctl(struct ifnet *, u_long, caddr_t);
void		cas_tick(void *);
void		cas_watchdog(struct ifnet *);
void		cas_shutdown(void *);
int		cas_init(struct ifnet *);
void		cas_init_regs(struct cas_softc *);
int		cas_ringsize(int);
int		cas_meminit(struct cas_softc *);
void		cas_mifinit(struct cas_softc *);
int		cas_bitwait(struct cas_softc *, bus_space_handle_t, int,
		    u_int32_t, u_int32_t);
void		cas_reset(struct cas_softc *);
int		cas_reset_rx(struct cas_softc *);
int		cas_reset_tx(struct cas_softc *);
int		cas_disable_rx(struct cas_softc *);
int		cas_disable_tx(struct cas_softc *);
void		cas_rxdrain(struct cas_softc *);
int		cas_add_rxbuf(struct cas_softc *, int idx);
void		cas_setladrf(struct cas_softc *);
int		cas_encap(struct cas_softc *, struct mbuf *, u_int32_t *);

/* MII methods & callbacks */
int		cas_mii_readreg(struct device *, int, int);
void		cas_mii_writereg(struct device *, int, int, int);
void		cas_mii_statchg(struct device *);
int		cas_pcs_readreg(struct device *, int, int);
void		cas_pcs_writereg(struct device *, int, int, int);

int		cas_mediachange(struct ifnet *);
void		cas_mediastatus(struct ifnet *, struct ifmediareq *);

struct mbuf	*cas_get(struct cas_softc *, int, int);
int		cas_eint(struct cas_softc *, u_int);
int		cas_rint(struct cas_softc *);
int		cas_tint(struct cas_softc *, u_int32_t);
int		cas_pint(struct cas_softc *);

#ifdef CAS_DEBUG
#define	DPRINTF(sc, x)	if ((sc)->sc_arpcom.ac_if.if_flags & IFF_DEBUG) \
				printf x
#else
#define	DPRINTF(sc, x)	/* nothing */
#endif

const struct pci_matchid cas_pci_devices[] = {
	{ PCI_VENDOR_SUN, PCI_PRODUCT_SUN_CASSINI }
};

int
cas_match(struct device *parent, void *cf, void *aux)
{
	return (pci_matchbyid((struct pci_attach_args *)aux, cas_pci_devices,
	    sizeof(cas_pci_devices)/sizeof(cas_pci_devices[0])));
}

#define	PROMHDR_PTR_DATA	0x18
#define	PROMDATA_PTR_VPD	0x08
#define	PROMDATA_DATA2		0x0a

static const u_int8_t cas_promhdr[] = { 0x55, 0xaa };
static const u_int8_t cas_promdat[] = {
	'P', 'C', 'I', 'R',
	PCI_VENDOR_SUN & 0xff, PCI_VENDOR_SUN >> 8,
	PCI_PRODUCT_SUN_CASSINI & 0xff, PCI_PRODUCT_SUN_CASSINI >> 8
};

static const u_int8_t cas_promdat2[] = {
	0x18, 0x00,			/* structure length */
	0x00,				/* structure revision */
	0x00,				/* interface revision */
	PCI_SUBCLASS_NETWORK_ETHERNET,	/* subclass code */
	PCI_CLASS_NETWORK		/* class code */
};

int
cas_pci_enaddr(struct cas_softc *sc, struct pci_attach_args *pa)
{
	struct pci_vpd *vpd;
	bus_space_handle_t romh;
	bus_space_tag_t romt;
	bus_size_t romsize;
	u_int8_t buf[32];
	pcireg_t address, mask;
	int dataoff, vpdoff;
	int rv = -1;

	address = pci_conf_read(pa->pa_pc, pa->pa_tag, PCI_ROM_REG);
	pci_conf_write(pa->pa_pc, pa->pa_tag, PCI_ROM_REG, 0xfffffffe);
	mask = pci_conf_read(pa->pa_pc, pa->pa_tag, PCI_ROM_REG);
	address |= PCI_ROM_ENABLE;
	pci_conf_write(pa->pa_pc, pa->pa_tag, PCI_ROM_REG, address);

	romt = pa->pa_memt;
	romsize = PCI_ROM_SIZE(mask);
	if (bus_space_map(romt, PCI_ROM_ADDR(address), romsize, 0, &romh)) {
		romsize = 0;
		goto fail;
	}

	bus_space_read_region_1(romt, romh, 0, buf, sizeof(buf));
	if (bcmp(buf, cas_promhdr, sizeof(cas_promhdr)))
		goto fail;

	dataoff = buf[PROMHDR_PTR_DATA] | (buf[PROMHDR_PTR_DATA + 1] << 8);
	if (dataoff < 0x1c)
		goto fail;

	bus_space_read_region_1(romt, romh, dataoff, buf, sizeof(buf));
	if (bcmp(buf, cas_promdat, sizeof(cas_promdat)) ||
	    bcmp(buf + PROMDATA_DATA2, cas_promdat2, sizeof(cas_promdat2)))
		goto fail;

	vpdoff = buf[PROMDATA_PTR_VPD] | (buf[PROMDATA_PTR_VPD + 1] << 8);
	if (vpdoff < 0x1c)
		goto fail;

	bus_space_read_region_1(romt, romh, vpdoff, buf, sizeof(buf));

	/*
	 * The VPD is not in PCI 2.2 standard format.  The length in
	 * the resource header is big endian.
	 */
	vpd = (struct pci_vpd *)(buf + 3);
	if (!PCI_VPDRES_ISLARGE(buf[0]) ||
	    PCI_VPDRES_LARGE_NAME(buf[0]) != PCI_VPDRES_TYPE_VPD)
		goto fail;
	if (vpd->vpd_key0 != 'N' || vpd->vpd_key1 != 'A')
		goto fail;

	bcopy(buf + 6, sc->sc_arpcom.ac_enaddr, ETHER_ADDR_LEN);
	rv = 0;

 fail:
	if (romsize != 0)
		bus_space_unmap(romt, romh, romsize);

	address = pci_conf_read(pa->pa_pc, pa->pa_tag, PCI_ROM_REG);
	address &= ~PCI_ROM_ENABLE;
	pci_conf_write(pa->pa_pc, pa->pa_tag, PCI_ROM_REG, address);

	return (rv);
}

void
cas_attach(struct device *parent, struct device *self, void *aux)
{
	struct pci_attach_args *pa = aux;
	struct cas_softc *sc = (void *)self;
	pci_intr_handle_t ih;
#ifdef __sparc64__
	/* XXX the following declarations should be elsewhere */
	extern void myetheraddr(u_char *);
#endif
	const char *intrstr = NULL;
	bus_size_t size;
	int gotenaddr = 0;

	sc->sc_dmatag = pa->pa_dmat;

#define PCI_CAS_BASEADDR	0x10
	if (pci_mapreg_map(pa, PCI_CAS_BASEADDR, PCI_MAPREG_TYPE_MEM, 0,
	    &sc->sc_memt, &sc->sc_memh, NULL, &size, 0) != 0) {
		printf(": could not map registers\n");
		return;
	}

	if (cas_pci_enaddr(sc, pa) == 0)
		gotenaddr = 1;

#ifdef __sparc64__
	if (!gotenaddr) {
		if (OF_getprop(PCITAG_NODE(pa->pa_tag), "local-mac-address",
		    sc->sc_arpcom.ac_enaddr, ETHER_ADDR_LEN) <= 0)
			myetheraddr(sc->sc_arpcom.ac_enaddr);
		gotenaddr = 1;
	}
#endif
#ifdef __powerpc__
	if (!gotenaddr) {
		pci_ether_hw_addr(pa->pa_pc, sc->sc_arpcom.ac_enaddr);
		gotenaddr = 1;
	}
#endif

	sc->sc_burst = 16;	/* XXX */

	if (pci_intr_map(pa, &ih) != 0) {
		printf(": couldn't map interrupt\n");
		bus_space_unmap(sc->sc_memt, sc->sc_memh, size);
		return;
	}
	intrstr = pci_intr_string(pa->pa_pc, ih);
	sc->sc_ih = pci_intr_establish(pa->pa_pc,
	    ih, IPL_NET, cas_intr, sc, self->dv_xname);
	if (sc->sc_ih == NULL) {
		printf(": couldn't establish interrupt");
		if (intrstr != NULL)
			printf(" at %s", intrstr);
		printf("\n");
		bus_space_unmap(sc->sc_memt, sc->sc_memh, size);
		return;
	}

	printf(": %s", intrstr);

	/*
	 * call the main configure
	 */
	cas_config(sc);
}

/*
 * cas_config:
 *
 *	Attach a Cassini interface to the system.
 */
void
cas_config(struct cas_softc *sc)
{
	struct ifnet *ifp = &sc->sc_arpcom.ac_if;
	struct mii_data *mii = &sc->sc_mii;
	struct mii_softc *child;
	int i, error;

	/* Make sure the chip is stopped. */
	ifp->if_softc = sc;
	cas_reset(sc);

	/*
	 * Allocate the control data structures, and create and load the
	 * DMA map for it.
	 */
	if ((error = bus_dmamem_alloc(sc->sc_dmatag,
	    sizeof(struct cas_control_data), PAGE_SIZE, 0, &sc->sc_cdseg,
	    1, &sc->sc_cdnseg, 0)) != 0) {
		printf("\n%s: unable to allocate control data, error = %d\n",
		    sc->sc_dev.dv_xname, error);
		goto fail_0;
	}

	/* XXX should map this in with correct endianness */
	if ((error = bus_dmamem_map(sc->sc_dmatag, &sc->sc_cdseg, sc->sc_cdnseg,
	    sizeof(struct cas_control_data), (caddr_t *)&sc->sc_control_data,
	    BUS_DMA_COHERENT)) != 0) {
		printf("\n%s: unable to map control data, error = %d\n",
		    sc->sc_dev.dv_xname, error);
		goto fail_1;
	}

	if ((error = bus_dmamap_create(sc->sc_dmatag,
	    sizeof(struct cas_control_data), 1,
	    sizeof(struct cas_control_data), 0, 0, &sc->sc_cddmamap)) != 0) {
		printf("\n%s: unable to create control data DMA map, "
		    "error = %d\n", sc->sc_dev.dv_xname, error);
		goto fail_2;
	}

	if ((error = bus_dmamap_load(sc->sc_dmatag, sc->sc_cddmamap,
	    sc->sc_control_data, sizeof(struct cas_control_data), NULL,
	    0)) != 0) {
		printf("\n%s: unable to load control data DMA map, error = %d\n",
		    sc->sc_dev.dv_xname, error);
		goto fail_3;
	}

	/*
	 * Create the receive buffer DMA maps.
	 */
	for (i = 0; i < CAS_NRXDESC; i++) {
		if ((error = bus_dmamap_create(sc->sc_dmatag, MCLBYTES, 1,
		    MCLBYTES, 0, 0, &sc->sc_rxsoft[i].rxs_dmamap)) != 0) {
			printf("\n%s: unable to create rx DMA map %d, "
			    "error = %d\n", sc->sc_dev.dv_xname, i, error);
			goto fail_5;
		}
		sc->sc_rxsoft[i].rxs_mbuf = NULL;
	}
	/*
	 * Create the transmit buffer DMA maps.
	 */
	for (i = 0; i < CAS_NTXDESC; i++) {
		if ((error = bus_dmamap_create(sc->sc_dmatag, MCLBYTES,
		    CAS_NTXSEGS, MCLBYTES, 0, BUS_DMA_NOWAIT,
		    &sc->sc_txd[i].sd_map)) != 0) {
			printf("\n%s: unable to create tx DMA map %d, "
			    "error = %d\n", sc->sc_dev.dv_xname, i, error);
			goto fail_6;
		}
		sc->sc_txd[i].sd_mbuf = NULL;
	}

	/*
	 * From this point forward, the attachment cannot fail.  A failure
	 * before this point releases all resources that may have been
	 * allocated.
	 */

	/* Announce ourselves. */
	printf(", address %s\n", ether_sprintf(sc->sc_arpcom.ac_enaddr));

	/* Get RX FIFO size */
	sc->sc_rxfifosize = 64 *
	    bus_space_read_4(sc->sc_memt, sc->sc_memh, CAS_RX_FIFO_SIZE);

	/* Initialize ifnet structure. */
	strlcpy(ifp->if_xname, sc->sc_dev.dv_xname, sizeof ifp->if_xname);
	ifp->if_softc = sc;
	ifp->if_flags =
	    IFF_BROADCAST | IFF_SIMPLEX | IFF_NOTRAILERS | IFF_MULTICAST;
	ifp->if_start = cas_start;
	ifp->if_ioctl = cas_ioctl;
	ifp->if_watchdog = cas_watchdog;
	IFQ_SET_MAXLEN(&ifp->if_snd, CAS_NTXDESC - 1);
	IFQ_SET_READY(&ifp->if_snd);

	ifp->if_capabilities = IFCAP_VLAN_MTU;

	/* Initialize ifmedia structures and MII info */
	mii->mii_ifp = ifp;
	mii->mii_readreg = cas_mii_readreg;
	mii->mii_writereg = cas_mii_writereg;
	mii->mii_statchg = cas_mii_statchg;

	ifmedia_init(&mii->mii_media, 0, cas_mediachange, cas_mediastatus);

	bus_space_write_4(sc->sc_memt, sc->sc_memh, CAS_MII_DATAPATH_MODE, 0);

	cas_mifinit(sc);

	if (sc->sc_mif_config & CAS_MIF_CONFIG_MDI1) {
		sc->sc_mif_config |= CAS_MIF_CONFIG_PHY_SEL;
		bus_space_write_4(sc->sc_memt, sc->sc_memh,
	            CAS_MIF_CONFIG, sc->sc_mif_config);
	}

	mii_attach(&sc->sc_dev, mii, 0xffffffff, MII_PHY_ANY,
	    MII_OFFSET_ANY, 0);

	child = LIST_FIRST(&mii->mii_phys);
	if (child == NULL &&
	    sc->sc_mif_config & (CAS_MIF_CONFIG_MDI0|CAS_MIF_CONFIG_MDI1)) {
		/* 
		 * Try the external PCS SERDES if we didn't find any
		 * MII devices.
		 */
		bus_space_write_4(sc->sc_memt, sc->sc_memh,
		    CAS_MII_DATAPATH_MODE, CAS_MII_DATAPATH_SERDES);

		bus_space_write_4(sc->sc_memt, sc->sc_memh,
		    CAS_MII_SLINK_CONTROL,
		    CAS_MII_SLINK_LOOPBACK|CAS_MII_SLINK_EN_SYNC_D);

		bus_space_write_4(sc->sc_memt, sc->sc_memh,
		     CAS_MII_CONFIG, CAS_MII_CONFIG_ENABLE);

		mii->mii_readreg = cas_pcs_readreg;
		mii->mii_writereg = cas_pcs_writereg;

		mii_attach(&sc->sc_dev, mii, 0xffffffff, MII_PHY_ANY,
		    MII_OFFSET_ANY, MIIF_NOISOLATE);
	}

	child = LIST_FIRST(&mii->mii_phys);
	if (child == NULL) {
		/* No PHY attached */
		ifmedia_add(&sc->sc_media, IFM_ETHER|IFM_MANUAL, 0, NULL);
		ifmedia_set(&sc->sc_media, IFM_ETHER|IFM_MANUAL);
	} else {
		/*
		 * Walk along the list of attached MII devices and
		 * establish an `MII instance' to `phy number'
		 * mapping. We'll use this mapping in media change
		 * requests to determine which phy to use to program
		 * the MIF configuration register.
		 */
		for (; child != NULL; child = LIST_NEXT(child, mii_list)) {
			/*
			 * Note: we support just two PHYs: the built-in
			 * internal device and an external on the MII
			 * connector.
			 */
			if (child->mii_phy > 1 || child->mii_inst > 1) {
				printf("%s: cannot accommodate MII device %s"
				       " at phy %d, instance %d\n",
				       sc->sc_dev.dv_xname,
				       child->mii_dev.dv_xname,
				       child->mii_phy, child->mii_inst);
				continue;
			}

			sc->sc_phys[child->mii_inst] = child->mii_phy;
		}

#if 0
		/*
		 * Now select and activate the PHY we will use.
		 *
		 * The order of preference is External (MDI1),
		 * Internal (MDI0), Serial Link (no MII).
		 */
		if (sc->sc_phys[1]) {
#ifdef CAS_DEBUG
			printf("using external phy\n");
#endif
			sc->sc_mif_config |= CAS_MIF_CONFIG_PHY_SEL;
		} else {
#ifdef CAS_DEBUG
			printf("using internal phy\n");
#endif
			sc->sc_mif_config &= ~CAS_MIF_CONFIG_PHY_SEL;
		}
		bus_space_write_4(sc->sc_memt, sc->sc_memh, CAS_MIF_CONFIG, 
			sc->sc_mif_config);
#endif

		/*
		 * XXX - we can really do the following ONLY if the
		 * phy indeed has the auto negotiation capability!!
		 */
		ifmedia_set(&sc->sc_media, IFM_ETHER|IFM_AUTO);
	}

	/* Attach the interface. */
	if_attach(ifp);
	ether_ifattach(ifp);

	sc->sc_sh = shutdownhook_establish(cas_shutdown, sc);
	if (sc->sc_sh == NULL)
		panic("cas_config: can't establish shutdownhook");

	timeout_set(&sc->sc_tick_ch, cas_tick, sc);
	return;

	/*
	 * Free any resources we've allocated during the failed attach
	 * attempt.  Do this in reverse order and fall through.
	 */
 fail_6:
	for (i = 0; i < CAS_NTXDESC; i++) {
		if (sc->sc_txd[i].sd_map != NULL)
			bus_dmamap_destroy(sc->sc_dmatag,
			    sc->sc_txd[i].sd_map);
	}
 fail_5:
	for (i = 0; i < CAS_NRXDESC; i++) {
		if (sc->sc_rxsoft[i].rxs_dmamap != NULL)
			bus_dmamap_destroy(sc->sc_dmatag,
			    sc->sc_rxsoft[i].rxs_dmamap);
	}
	bus_dmamap_unload(sc->sc_dmatag, sc->sc_cddmamap);
 fail_3:
	bus_dmamap_destroy(sc->sc_dmatag, sc->sc_cddmamap);
 fail_2:
	bus_dmamem_unmap(sc->sc_dmatag, (caddr_t)sc->sc_control_data,
	    sizeof(struct cas_control_data));
 fail_1:
	bus_dmamem_free(sc->sc_dmatag, &sc->sc_cdseg, sc->sc_cdnseg);
 fail_0:
	return;
}


void
cas_tick(void *arg)
{
	struct cas_softc *sc = arg;
	struct ifnet *ifp = &sc->sc_arpcom.ac_if;
	bus_space_tag_t t = sc->sc_memt;
	bus_space_handle_t mac = sc->sc_memh;
	int s;

	/* unload collisions counters */
	ifp->if_collisions +=
	    bus_space_read_4(t, mac, CAS_MAC_NORM_COLL_CNT) +
	    bus_space_read_4(t, mac, CAS_MAC_FIRST_COLL_CNT) +
	    bus_space_read_4(t, mac, CAS_MAC_EXCESS_COLL_CNT) +
	    bus_space_read_4(t, mac, CAS_MAC_LATE_COLL_CNT);

	/* clear the hardware counters */
	bus_space_write_4(t, mac, CAS_MAC_NORM_COLL_CNT, 0);
	bus_space_write_4(t, mac, CAS_MAC_FIRST_COLL_CNT, 0);
	bus_space_write_4(t, mac, CAS_MAC_EXCESS_COLL_CNT, 0);
	bus_space_write_4(t, mac, CAS_MAC_LATE_COLL_CNT, 0);

	s = splnet();
	mii_tick(&sc->sc_mii);
	splx(s);

	timeout_add(&sc->sc_tick_ch, hz);
}

int
cas_bitwait(struct cas_softc *sc, bus_space_handle_t h, int r,
    u_int32_t clr, u_int32_t set)
{
	int i;
	u_int32_t reg;

	for (i = TRIES; i--; DELAY(100)) {
		reg = bus_space_read_4(sc->sc_memt, h, r);
		if ((reg & clr) == 0 && (reg & set) == set)
			return (1);
	}

	return (0);
}

void
cas_reset(struct cas_softc *sc)
{
	bus_space_tag_t t = sc->sc_memt;
	bus_space_handle_t h = sc->sc_memh;
	int s;

	s = splnet();
	DPRINTF(sc, ("%s: cas_reset\n", sc->sc_dev.dv_xname));
	cas_reset_rx(sc);
	cas_reset_tx(sc);

	/* Do a full reset */
	bus_space_write_4(t, h, CAS_RESET, CAS_RESET_RX|CAS_RESET_TX);
	if (!cas_bitwait(sc, h, CAS_RESET, CAS_RESET_RX | CAS_RESET_TX, 0))
		printf("%s: cannot reset device\n", sc->sc_dev.dv_xname);
	splx(s);
}


/*
 * cas_rxdrain:
 *
 *	Drain the receive queue.
 */
void
cas_rxdrain(struct cas_softc *sc)
{
	struct cas_rxsoft *rxs;
	int i;

	for (i = 0; i < CAS_NRXDESC; i++) {
		rxs = &sc->sc_rxsoft[i];
		if (rxs->rxs_mbuf != NULL) {
			bus_dmamap_sync(sc->sc_dmatag, rxs->rxs_dmamap, 0,
			    rxs->rxs_dmamap->dm_mapsize, BUS_DMASYNC_POSTREAD);
			bus_dmamap_unload(sc->sc_dmatag, rxs->rxs_dmamap);
			m_freem(rxs->rxs_mbuf);
			rxs->rxs_mbuf = NULL;
		}
	}
}

/*
 * Reset the whole thing.
 */
void
cas_stop(struct ifnet *ifp, int disable)
{
	struct cas_softc *sc = (struct cas_softc *)ifp->if_softc;
	struct cas_sxd *sd;
	u_int32_t i;

	DPRINTF(sc, ("%s: cas_stop\n", sc->sc_dev.dv_xname));

	timeout_del(&sc->sc_tick_ch);

	/*
	 * Mark the interface down and cancel the watchdog timer.
	 */
	ifp->if_flags &= ~(IFF_RUNNING | IFF_OACTIVE);
	ifp->if_timer = 0;

	mii_down(&sc->sc_mii);

	cas_reset_rx(sc);
	cas_reset_tx(sc);

	/*
	 * Release any queued transmit buffers.
	 */
	for (i = 0; i < CAS_NTXDESC; i++) {
		sd = &sc->sc_txd[i];
		if (sd->sd_mbuf != NULL) {
			bus_dmamap_sync(sc->sc_dmatag, sd->sd_map, 0,
			    sd->sd_map->dm_mapsize, BUS_DMASYNC_POSTWRITE);
			bus_dmamap_unload(sc->sc_dmatag, sd->sd_map);
			m_freem(sd->sd_mbuf);
			sd->sd_mbuf = NULL;
		}
	}
	sc->sc_tx_cnt = sc->sc_tx_prod = sc->sc_tx_cons = 0;

	if (disable)
		cas_rxdrain(sc);
}


/*
 * Reset the receiver
 */
int
cas_reset_rx(struct cas_softc *sc)
{
	bus_space_tag_t t = sc->sc_memt;
	bus_space_handle_t h = sc->sc_memh;

	/*
	 * Resetting while DMA is in progress can cause a bus hang, so we
	 * disable DMA first.
	 */
	cas_disable_rx(sc);
	bus_space_write_4(t, h, CAS_RX_CONFIG, 0);
	/* Wait till it finishes */
	if (!cas_bitwait(sc, h, CAS_RX_CONFIG, 1, 0))
		printf("%s: cannot disable rx dma\n", sc->sc_dev.dv_xname);
	/* Wait 5ms extra. */
	delay(5000);

	/* Finally, reset the ERX */
	bus_space_write_4(t, h, CAS_RESET, CAS_RESET_RX);
	/* Wait till it finishes */
	if (!cas_bitwait(sc, h, CAS_RESET, CAS_RESET_RX, 0)) {
		printf("%s: cannot reset receiver\n", sc->sc_dev.dv_xname);
		return (1);
	}
	return (0);
}


/*
 * Reset the transmitter
 */
int
cas_reset_tx(struct cas_softc *sc)
{
	bus_space_tag_t t = sc->sc_memt;
	bus_space_handle_t h = sc->sc_memh;

	/*
	 * Resetting while DMA is in progress can cause a bus hang, so we
	 * disable DMA first.
	 */
	cas_disable_tx(sc);
	bus_space_write_4(t, h, CAS_TX_CONFIG, 0);
	/* Wait till it finishes */
	if (!cas_bitwait(sc, h, CAS_TX_CONFIG, 1, 0))
		printf("%s: cannot disable tx dma\n", sc->sc_dev.dv_xname);
	/* Wait 5ms extra. */
	delay(5000);

	/* Finally, reset the ETX */
	bus_space_write_4(t, h, CAS_RESET, CAS_RESET_TX);
	/* Wait till it finishes */
	if (!cas_bitwait(sc, h, CAS_RESET, CAS_RESET_TX, 0)) {
		printf("%s: cannot reset transmitter\n",
			sc->sc_dev.dv_xname);
		return (1);
	}
	return (0);
}

/*
 * disable receiver.
 */
int
cas_disable_rx(struct cas_softc *sc)
{
	bus_space_tag_t t = sc->sc_memt;
	bus_space_handle_t h = sc->sc_memh;
	u_int32_t cfg;

	/* Flip the enable bit */
	cfg = bus_space_read_4(t, h, CAS_MAC_RX_CONFIG);
	cfg &= ~CAS_MAC_RX_ENABLE;
	bus_space_write_4(t, h, CAS_MAC_RX_CONFIG, cfg);

	/* Wait for it to finish */
	return (cas_bitwait(sc, h, CAS_MAC_RX_CONFIG, CAS_MAC_RX_ENABLE, 0));
}

/*
 * disable transmitter.
 */
int
cas_disable_tx(struct cas_softc *sc)
{
	bus_space_tag_t t = sc->sc_memt;
	bus_space_handle_t h = sc->sc_memh;
	u_int32_t cfg;

	/* Flip the enable bit */
	cfg = bus_space_read_4(t, h, CAS_MAC_TX_CONFIG);
	cfg &= ~CAS_MAC_TX_ENABLE;
	bus_space_write_4(t, h, CAS_MAC_TX_CONFIG, cfg);

	/* Wait for it to finish */
	return (cas_bitwait(sc, h, CAS_MAC_TX_CONFIG, CAS_MAC_TX_ENABLE, 0));
}

/*
 * Initialize interface.
 */
int
cas_meminit(struct cas_softc *sc)
{
	struct cas_rxsoft *rxs;
	int i, error;

	/*
	 * Initialize the transmit descriptor ring.
	 */
	for (i = 0; i < CAS_NTXDESC; i++) {
		sc->sc_txdescs[i].gd_flags = 0;
		sc->sc_txdescs[i].gd_addr = 0;
	}
	CAS_CDTXSYNC(sc, 0, CAS_NTXDESC,
	    BUS_DMASYNC_PREREAD|BUS_DMASYNC_PREWRITE);

	/*
	 * Initialize the receive descriptor and receive job
	 * descriptor rings.
	 */
	for (i = 0; i < CAS_NRXDESC; i++) {
		rxs = &sc->sc_rxsoft[i];
		if (rxs->rxs_mbuf == NULL) {
			if ((error = cas_add_rxbuf(sc, i)) != 0) {
				printf("%s: unable to allocate or map rx "
				    "buffer %d, error = %d\n",
				    sc->sc_dev.dv_xname, i, error);
				/*
				 * XXX Should attempt to run with fewer receive
				 * XXX buffers instead of just failing.
				 */
				cas_rxdrain(sc);
				return (1);
			}
		} else
			CAS_INIT_RXDESC(sc, i);
	}
	sc->sc_rxptr = 0;

	return (0);
}

int
cas_ringsize(int sz)
{
	switch (sz) {
	case 32:
		return CAS_RING_SZ_32;
	case 64:
		return CAS_RING_SZ_64;
	case 128:
		return CAS_RING_SZ_128;
	case 256:
		return CAS_RING_SZ_256;
	case 512:
		return CAS_RING_SZ_512;
	case 1024:
		return CAS_RING_SZ_1024;
	case 2048:
		return CAS_RING_SZ_2048;
	case 4096:
		return CAS_RING_SZ_4096;
	case 8192:
		return CAS_RING_SZ_8192;
	default:
		printf("cas: invalid Receive Descriptor ring size %d\n", sz);
		return CAS_RING_SZ_32;
	}
}

/*
 * Initialization of interface; set up initialization block
 * and transmit/receive descriptor rings.
 */
int
cas_init(struct ifnet *ifp)
{

	struct cas_softc *sc = (struct cas_softc *)ifp->if_softc;
	bus_space_tag_t t = sc->sc_memt;
	bus_space_handle_t h = sc->sc_memh;
	int s;
	u_int max_frame_size;
	u_int32_t v;

	s = splnet();

	DPRINTF(sc, ("%s: cas_init: calling stop\n", sc->sc_dev.dv_xname));
	/*
	 * Initialization sequence. The numbered steps below correspond
	 * to the sequence outlined in section 6.3.5.1 in the Ethernet
	 * Channel Engine manual (part of the PCIO manual).
	 * See also the STP2002-STQ document from Sun Microsystems.
	 */

	/* step 1 & 2. Reset the Ethernet Channel */
	cas_stop(ifp, 0);
	cas_reset(sc);
	DPRINTF(sc, ("%s: cas_init: restarting\n", sc->sc_dev.dv_xname));

	/* Re-initialize the MIF */
	cas_mifinit(sc);

	/* step 3. Setup data structures in host memory */
	cas_meminit(sc);

	/* step 4. TX MAC registers & counters */
	cas_init_regs(sc);
	max_frame_size = ETHER_MAX_LEN + ETHER_VLAN_ENCAP_LEN;
	v = (max_frame_size) | (0x2000 << 16) /* Burst size */;
	bus_space_write_4(t, h, CAS_MAC_MAC_MAX_FRAME, v);

	/* step 5. RX MAC registers & counters */
	cas_setladrf(sc);

	/* step 6 & 7. Program Descriptor Ring Base Addresses */
	bus_space_write_4(t, h, CAS_TX_RING_PTR_HI, 
	    (((uint64_t)CAS_CDTXADDR(sc,0)) >> 32));
	bus_space_write_4(t, h, CAS_TX_RING_PTR_LO, CAS_CDTXADDR(sc, 0));

	bus_space_write_4(t, h, CAS_RX_RING_PTR_HI, 
	    (((uint64_t)CAS_CDRXADDR(sc,0)) >> 32));
	bus_space_write_4(t, h, CAS_RX_RING_PTR_LO, CAS_CDRXADDR(sc, 0));

	/* step 8. Global Configuration & Interrupt Mask */
	bus_space_write_4(t, h, CAS_INTMASK,
		      ~(CAS_INTR_TX_INTME|
			CAS_INTR_TX_EMPTY|
			CAS_INTR_RX_DONE|CAS_INTR_RX_NOBUF|
			CAS_INTR_RX_TAG_ERR|CAS_INTR_PCS|
			CAS_INTR_MAC_CONTROL|CAS_INTR_MIF|
			CAS_INTR_BERR));
	bus_space_write_4(t, h, CAS_MAC_RX_MASK,
	    CAS_MAC_RX_DONE|CAS_MAC_RX_FRAME_CNT);
	bus_space_write_4(t, h, CAS_MAC_TX_MASK, 0 /*CAS_MAC_TX_XMIT_DONE*/);
	bus_space_write_4(t, h, CAS_MAC_CONTROL_MASK, 0); /* XXXX */

	/* step 9. ETX Configuration: use mostly default values */

	/* Enable DMA */
	v = cas_ringsize(CAS_NTXDESC /*XXX*/) << 13;
	bus_space_write_4(t, h, CAS_TX_CONFIG,
	    v|CAS_TX_CONFIG_TXDMA_EN|(1<<24)|(1<<29));
	bus_space_write_4(t, h, CAS_TX_KICK, 0);

	/* step 10. ERX Configuration */

	/* Encode Receive Descriptor ring size: four possible values */
	v = cas_ringsize(CAS_NRXDESC /*XXX*/) << 1;

	/* Enable DMA */
	bus_space_write_4(t, h, CAS_RX_CONFIG, 
		v|(CAS_THRSH_1024<<CAS_RX_CONFIG_FIFO_THRS_SHIFT)|
		(2<<CAS_RX_CONFIG_FBOFF_SHFT)|CAS_RX_CONFIG_RXDMA_EN|
		(0<<CAS_RX_CONFIG_CXM_START_SHFT));
	/*
	 * The following value is for an OFF Threshold of about 3/4 full
	 * and an ON Threshold of 1/4 full.
	 */
	bus_space_write_4(t, h, CAS_RX_PAUSE_THRESH,
	    (3 * sc->sc_rxfifosize / 256) |
	    (   (sc->sc_rxfifosize / 256) << 12));
	bus_space_write_4(t, h, CAS_RX_BLANKING, (6<<12)|6);

	/* step 11. Configure Media */
	mii_mediachg(&sc->sc_mii);

	/* step 12. RX_MAC Configuration Register */
	v = bus_space_read_4(t, h, CAS_MAC_RX_CONFIG);
	v |= CAS_MAC_RX_ENABLE | CAS_MAC_RX_STRIP_CRC;
	bus_space_write_4(t, h, CAS_MAC_RX_CONFIG, v);

	/* step 14. Issue Transmit Pending command */

	/* step 15.  Give the receiver a swift kick */
	bus_space_write_4(t, h, CAS_RX_KICK, CAS_NRXDESC-4);

	/* Start the one second timer. */
	timeout_add(&sc->sc_tick_ch, hz);

	ifp->if_flags |= IFF_RUNNING;
	ifp->if_flags &= ~IFF_OACTIVE;
	ifp->if_timer = 0;
	splx(s);

	return (0);
}

void
cas_init_regs(struct cas_softc *sc)
{
	bus_space_tag_t t = sc->sc_memt;
	bus_space_handle_t h = sc->sc_memh;
	u_int32_t v, r;

	/* These regs are not cleared on reset */
	sc->sc_inited = 0;
	if (!sc->sc_inited) {

		/* Wooo.  Magic values. */
		bus_space_write_4(t, h, CAS_MAC_IPG0, 0);
		bus_space_write_4(t, h, CAS_MAC_IPG1, 8);
		bus_space_write_4(t, h, CAS_MAC_IPG2, 4);

		bus_space_write_4(t, h, CAS_MAC_MAC_MIN_FRAME, ETHER_MIN_LEN);
		/* Max frame and max burst size */
		v = ETHER_MAX_LEN | (0x2000 << 16) /* Burst size */;
		bus_space_write_4(t, h, CAS_MAC_MAC_MAX_FRAME, v);

		bus_space_write_4(t, h, CAS_MAC_PREAMBLE_LEN, 0x7);
		bus_space_write_4(t, h, CAS_MAC_JAM_SIZE, 0x4);
		bus_space_write_4(t, h, CAS_MAC_ATTEMPT_LIMIT, 0x10);
		/* Dunno.... */
		bus_space_write_4(t, h, CAS_MAC_CONTROL_TYPE, 0x8088);
		bus_space_write_4(t, h, CAS_MAC_RANDOM_SEED,
		    ((sc->sc_arpcom.ac_enaddr[5]<<8)|sc->sc_arpcom.ac_enaddr[4])&0x3ff);

		/* Secondary MAC addresses set to 0:0:0:0:0:0 */
		for (r = CAS_MAC_ADDR3; r < CAS_MAC_ADDR42; r += 4)
		  	bus_space_write_4(t, h, r, 0);

		/* MAC control addr set to 0:1:c2:0:1:80 */
		bus_space_write_4(t, h, CAS_MAC_ADDR42, 0x0001);
		bus_space_write_4(t, h, CAS_MAC_ADDR43, 0xc200);
		bus_space_write_4(t, h, CAS_MAC_ADDR44, 0x0180);

		/* MAC filter addr set to 0:0:0:0:0:0 */
		bus_space_write_4(t, h, CAS_MAC_ADDR_FILTER0, 0);
		bus_space_write_4(t, h, CAS_MAC_ADDR_FILTER1, 0);
		bus_space_write_4(t, h, CAS_MAC_ADDR_FILTER2, 0);

		bus_space_write_4(t, h, CAS_MAC_ADR_FLT_MASK1_2, 0);
		bus_space_write_4(t, h, CAS_MAC_ADR_FLT_MASK0, 0);

		/* Hash table initialized to 0 */
		for (r = CAS_MAC_HASH0; r <= CAS_MAC_HASH15; r += 4)
			bus_space_write_4(t, h, r, 0);

		sc->sc_inited = 1;
	}

	/* Counters need to be zeroed */
	bus_space_write_4(t, h, CAS_MAC_NORM_COLL_CNT, 0);
	bus_space_write_4(t, h, CAS_MAC_FIRST_COLL_CNT, 0);
	bus_space_write_4(t, h, CAS_MAC_EXCESS_COLL_CNT, 0);
	bus_space_write_4(t, h, CAS_MAC_LATE_COLL_CNT, 0);
	bus_space_write_4(t, h, CAS_MAC_DEFER_TMR_CNT, 0);
	bus_space_write_4(t, h, CAS_MAC_PEAK_ATTEMPTS, 0);
	bus_space_write_4(t, h, CAS_MAC_RX_FRAME_COUNT, 0);
	bus_space_write_4(t, h, CAS_MAC_RX_LEN_ERR_CNT, 0);
	bus_space_write_4(t, h, CAS_MAC_RX_ALIGN_ERR, 0);
	bus_space_write_4(t, h, CAS_MAC_RX_CRC_ERR_CNT, 0);
	bus_space_write_4(t, h, CAS_MAC_RX_CODE_VIOL, 0);

	/* Un-pause stuff */
	bus_space_write_4(t, h, CAS_MAC_SEND_PAUSE_CMD, 0);

	/*
	 * Set the station address.
	 */
	bus_space_write_4(t, h, CAS_MAC_ADDR0, 
		(sc->sc_arpcom.ac_enaddr[4]<<8) | sc->sc_arpcom.ac_enaddr[5]);
	bus_space_write_4(t, h, CAS_MAC_ADDR1, 
		(sc->sc_arpcom.ac_enaddr[2]<<8) | sc->sc_arpcom.ac_enaddr[3]);
	bus_space_write_4(t, h, CAS_MAC_ADDR2, 
		(sc->sc_arpcom.ac_enaddr[0]<<8) | sc->sc_arpcom.ac_enaddr[1]);
}

/*
 * Receive interrupt.
 */
int
cas_rint(struct cas_softc *sc)
{
	struct ifnet *ifp = &sc->sc_arpcom.ac_if;
	bus_space_tag_t t = sc->sc_memt;
	bus_space_handle_t h = sc->sc_memh;
	struct ether_header *eh;
	struct cas_rxsoft *rxs;
	struct mbuf *m;
	u_int64_t rxstat;
	int i, len;

	for (i = sc->sc_rxptr;; i = CAS_NEXTRX(i)) {
		rxs = &sc->sc_rxsoft[i];

		CAS_CDRXSYNC(sc, i,
		    BUS_DMASYNC_POSTREAD|BUS_DMASYNC_POSTWRITE);

		rxstat = CAS_DMA_READ(sc, sc->sc_rxdescs[i].gd_flags);

		if (rxstat & CAS_RD_OWN) {
			/*
			 * We have processed all of the receive buffers.
			 */
			break;
		}

		if (rxstat & CAS_RD_BAD_CRC) {
#ifdef CAS_DEBUG
			printf("%s: receive error: CRC error\n",
				sc->sc_dev.dv_xname);
#endif
			CAS_INIT_RXDESC(sc, i);
			continue;
		}

		bus_dmamap_sync(sc->sc_dmatag, rxs->rxs_dmamap, 0,
		    rxs->rxs_dmamap->dm_mapsize, BUS_DMASYNC_POSTREAD);
#ifdef CAS_DEBUG
		if (ifp->if_flags & IFF_DEBUG) {
			printf("    rxsoft %p descriptor %d: ", rxs, i);
			printf("gd_flags: 0x%016llx\t", (long long)
				CAS_DMA_READ(sc, sc->sc_rxdescs[i].gd_flags));
			printf("gd_addr: 0x%016llx\n", (long long)
				CAS_DMA_READ(sc, sc->sc_rxdescs[i].gd_addr));
		}
#endif

		/* No errors; receive the packet. */
		len = CAS_RD_BUFLEN(rxstat);

		/*
		 * Allocate a new mbuf cluster.  If that fails, we are
		 * out of memory, and must drop the packet and recycle
		 * the buffer that's already attached to this descriptor.
		 */
		m = rxs->rxs_mbuf;
		if (cas_add_rxbuf(sc, i) != 0) {
			ifp->if_ierrors++;
			CAS_INIT_RXDESC(sc, i);
			bus_dmamap_sync(sc->sc_dmatag, rxs->rxs_dmamap, 0,
			    rxs->rxs_dmamap->dm_mapsize, BUS_DMASYNC_PREREAD);
			continue;
		}
		m->m_data += 2; /* We're already off by two */

		ifp->if_ipackets++;
		eh = mtod(m, struct ether_header *);
		m->m_pkthdr.rcvif = ifp;
		m->m_pkthdr.len = m->m_len = len;

#if NBPFILTER > 0
		/*
		 * Pass this up to any BPF listeners, but only
		 * pass it up the stack if its for us.
		 */
		if (ifp->if_bpf)
			bpf_mtap(ifp->if_bpf, m, BPF_DIRECTION_IN);
#endif /* NPBFILTER > 0 */

		/* Pass it on. */
		ether_input_mbuf(ifp, m);
	}

	/* Update the receive pointer. */
	sc->sc_rxptr = i;
	bus_space_write_4(t, h, CAS_RX_KICK, i);

	DPRINTF(sc, ("cas_rint: done sc->rxptr %d, complete %d\n",
		sc->sc_rxptr, bus_space_read_4(t, h, CAS_RX_COMPLETION)));

	return (1);
}


/*
 * cas_add_rxbuf:
 *
 *	Add a receive buffer to the indicated descriptor.
 */
int
cas_add_rxbuf(struct cas_softc *sc, int idx)
{
	struct cas_rxsoft *rxs = &sc->sc_rxsoft[idx];
	struct mbuf *m;
	int error;

	MGETHDR(m, M_DONTWAIT, MT_DATA);
	if (m == NULL)
		return (ENOBUFS);

	MCLGET(m, M_DONTWAIT);
	if ((m->m_flags & M_EXT) == 0) {
		m_freem(m);
		return (ENOBUFS);
	}

#ifdef CAS_DEBUG
/* bzero the packet to check dma */
	memset(m->m_ext.ext_buf, 0, m->m_ext.ext_size);
#endif

	if (rxs->rxs_mbuf != NULL)
		bus_dmamap_unload(sc->sc_dmatag, rxs->rxs_dmamap);

	rxs->rxs_mbuf = m;

	error = bus_dmamap_load(sc->sc_dmatag, rxs->rxs_dmamap,
	    m->m_ext.ext_buf, m->m_ext.ext_size, NULL,
	    BUS_DMA_READ|BUS_DMA_NOWAIT);
	if (error) {
		printf("%s: can't load rx DMA map %d, error = %d\n",
		    sc->sc_dev.dv_xname, idx, error);
		panic("cas_add_rxbuf");	/* XXX */
	}

	bus_dmamap_sync(sc->sc_dmatag, rxs->rxs_dmamap, 0,
	    rxs->rxs_dmamap->dm_mapsize, BUS_DMASYNC_PREREAD);

	CAS_INIT_RXDESC(sc, idx);

	return (0);
}


int
cas_eint(struct cas_softc *sc, u_int status)
{
	if ((status & CAS_INTR_MIF) != 0) {
#ifdef CAS_DEBUG
		printf("%s: link status changed\n", sc->sc_dev.dv_xname);
#endif
		return (1);
	}

	printf("%s: status=%b\n", sc->sc_dev.dv_xname, status, CAS_INTR_BITS);
	return (1);
}

int
cas_pint(struct cas_softc *sc)
{
	bus_space_tag_t t = sc->sc_memt;
	bus_space_handle_t seb = sc->sc_memh;
	u_int32_t status;

	status = bus_space_read_4(t, seb, CAS_MII_INTERRUP_STATUS);
	status |= bus_space_read_4(t, seb, CAS_MII_INTERRUP_STATUS);
#ifdef CAS_DEBUG
	if (status)
		printf("%s: link status changed\n", sc->sc_dev.dv_xname);
#endif
	return (1);
}

int
cas_intr(void *v)
{
	struct cas_softc *sc = (struct cas_softc *)v;
	struct ifnet *ifp = &sc->sc_arpcom.ac_if;
	bus_space_tag_t t = sc->sc_memt;
	bus_space_handle_t seb = sc->sc_memh;
	u_int32_t status;
	int r = 0;

	status = bus_space_read_4(t, seb, CAS_STATUS);
	DPRINTF(sc, ("%s: cas_intr: cplt %xstatus %b\n",
		sc->sc_dev.dv_xname, (status>>19), status, CAS_INTR_BITS));

	if ((status & CAS_INTR_PCS) != 0)
		r |= cas_pint(sc);

	if ((status & (CAS_INTR_RX_TAG_ERR | CAS_INTR_BERR)) != 0)
		r |= cas_eint(sc, status);

	if ((status & (CAS_INTR_TX_EMPTY | CAS_INTR_TX_INTME)) != 0)
		r |= cas_tint(sc, status);

	if ((status & (CAS_INTR_RX_DONE | CAS_INTR_RX_NOBUF)) != 0)
		r |= cas_rint(sc);

	/* We should eventually do more than just print out error stats. */
	if (status & CAS_INTR_TX_MAC) {
		int txstat = bus_space_read_4(t, seb, CAS_MAC_TX_STATUS);
#ifdef CAS_DEBUG
		if (txstat & ~CAS_MAC_TX_XMIT_DONE)
			printf("%s: MAC tx fault, status %x\n",
			    sc->sc_dev.dv_xname, txstat);
#endif
		if (txstat & (CAS_MAC_TX_UNDERRUN | CAS_MAC_TX_PKT_TOO_LONG))
			cas_init(ifp);
	}
	if (status & CAS_INTR_RX_MAC) {
		int rxstat = bus_space_read_4(t, seb, CAS_MAC_RX_STATUS);
#ifdef CAS_DEBUG
 		if (rxstat & ~CAS_MAC_RX_DONE)
 			printf("%s: MAC rx fault, status %x\n",
 			    sc->sc_dev.dv_xname, rxstat);
#endif
		/*
		 * On some chip revisions CAS_MAC_RX_OVERFLOW happen often
		 * due to a silicon bug so handle them silently.
		 */
		if (rxstat & CAS_MAC_RX_OVERFLOW) {
			ifp->if_ierrors++;
			cas_init(ifp);
		}
#ifdef CAS_DEBUG
		else if (rxstat & ~(CAS_MAC_RX_DONE | CAS_MAC_RX_FRAME_CNT))
			printf("%s: MAC rx fault, status %x\n",
			    sc->sc_dev.dv_xname, rxstat);
#endif
	}
	return (r);
}


void
cas_watchdog(struct ifnet *ifp)
{
	struct cas_softc *sc = ifp->if_softc;

	DPRINTF(sc, ("cas_watchdog: CAS_RX_CONFIG %x CAS_MAC_RX_STATUS %x "
		"CAS_MAC_RX_CONFIG %x\n",
		bus_space_read_4(sc->sc_memt, sc->sc_memh, CAS_RX_CONFIG),
		bus_space_read_4(sc->sc_memt, sc->sc_memh, CAS_MAC_RX_STATUS),
		bus_space_read_4(sc->sc_memt, sc->sc_memh, CAS_MAC_RX_CONFIG)));

	log(LOG_ERR, "%s: device timeout\n", sc->sc_dev.dv_xname);
	++ifp->if_oerrors;

	/* Try to get more packets going. */
	cas_init(ifp);
}

/*
 * Initialize the MII Management Interface
 */
void
cas_mifinit(struct cas_softc *sc)
{
	bus_space_tag_t t = sc->sc_memt;
	bus_space_handle_t mif = sc->sc_memh;

	/* Configure the MIF in frame mode */
	sc->sc_mif_config = bus_space_read_4(t, mif, CAS_MIF_CONFIG);
	sc->sc_mif_config &= ~CAS_MIF_CONFIG_BB_ENA;
	bus_space_write_4(t, mif, CAS_MIF_CONFIG, sc->sc_mif_config);
}

/*
 * MII interface
 *
 * The Cassini MII interface supports at least three different operating modes:
 *
 * Bitbang mode is implemented using data, clock and output enable registers.
 *
 * Frame mode is implemented by loading a complete frame into the frame
 * register and polling the valid bit for completion.
 *
 * Polling mode uses the frame register but completion is indicated by
 * an interrupt.
 *
 */
int
cas_mii_readreg(struct device *self, int phy, int reg)
{
	struct cas_softc *sc = (void *)self;
	bus_space_tag_t t = sc->sc_memt;
	bus_space_handle_t mif = sc->sc_memh;
	int n;
	u_int32_t v;

#if 0
#ifdef CAS_DEBUG
	if (sc->sc_debug)
		printf("cas_mii_readreg: phy %d reg %d\n", phy, reg);
#endif
#endif

	/* Construct the frame command */
	v = (reg << CAS_MIF_REG_SHIFT)	| (phy << CAS_MIF_PHY_SHIFT) |
		CAS_MIF_FRAME_READ;

	bus_space_write_4(t, mif, CAS_MIF_FRAME, v);
	for (n = 0; n < 100; n++) {
		DELAY(1);
		v = bus_space_read_4(t, mif, CAS_MIF_FRAME);
		if (v & CAS_MIF_FRAME_TA0)
			return (v & CAS_MIF_FRAME_DATA);
	}

	printf("%s: mii_read timeout\n", sc->sc_dev.dv_xname);
	return (0);
}

void
cas_mii_writereg(struct device *self, int phy, int reg, int val)
{
	struct cas_softc *sc = (void *)self;
	bus_space_tag_t t = sc->sc_memt;
	bus_space_handle_t mif = sc->sc_memh;
	int n;
	u_int32_t v;

#if 0
#ifdef CAS_DEBUG
	if (sc->sc_debug)
		printf("cas_mii_writereg: phy %d reg %d val %x\n",
			phy, reg, val);
#endif
#endif

#if 0
	/* Select the desired PHY in the MIF configuration register */
	v = bus_space_read_4(t, mif, CAS_MIF_CONFIG);
	/* Clear PHY select bit */
	v &= ~CAS_MIF_CONFIG_PHY_SEL;
	if (phy == CAS_PHYAD_EXTERNAL)
		/* Set PHY select bit to get at external device */
		v |= CAS_MIF_CONFIG_PHY_SEL;
	bus_space_write_4(t, mif, CAS_MIF_CONFIG, v);
#endif
	/* Construct the frame command */
	v = CAS_MIF_FRAME_WRITE			|
	    (phy << CAS_MIF_PHY_SHIFT)		|
	    (reg << CAS_MIF_REG_SHIFT)		|
	    (val & CAS_MIF_FRAME_DATA);

	bus_space_write_4(t, mif, CAS_MIF_FRAME, v);
	for (n = 0; n < 100; n++) {
		DELAY(1);
		v = bus_space_read_4(t, mif, CAS_MIF_FRAME);
		if (v & CAS_MIF_FRAME_TA0)
			return;
	}

	printf("%s: mii_write timeout\n", sc->sc_dev.dv_xname);
}

void
cas_mii_statchg(struct device *dev)
{
	struct cas_softc *sc = (void *)dev;
#ifdef CAS_DEBUG
	int instance = IFM_INST(sc->sc_mii.mii_media.ifm_cur->ifm_media);
#endif
	bus_space_tag_t t = sc->sc_memt;
	bus_space_handle_t mac = sc->sc_memh;
	u_int32_t v;

#ifdef CAS_DEBUG
	if (sc->sc_debug)
		printf("cas_mii_statchg: status change: phy = %d\n",
		    sc->sc_phys[instance]);
#endif

	/* Set tx full duplex options */
	bus_space_write_4(t, mac, CAS_MAC_TX_CONFIG, 0);
	delay(10000); /* reg must be cleared and delay before changing. */
	v = CAS_MAC_TX_ENA_IPG0|CAS_MAC_TX_NGU|CAS_MAC_TX_NGU_LIMIT|
		CAS_MAC_TX_ENABLE;
	if ((IFM_OPTIONS(sc->sc_mii.mii_media_active) & IFM_FDX) != 0) {
		v |= CAS_MAC_TX_IGN_CARRIER|CAS_MAC_TX_IGN_COLLIS;
	}
	bus_space_write_4(t, mac, CAS_MAC_TX_CONFIG, v);

	/* XIF Configuration */
	v = CAS_MAC_XIF_TX_MII_ENA;
	v |= CAS_MAC_XIF_LINK_LED;

#if 0
	/* If an external transceiver is connected, enable its MII drivers */
	sc->sc_mif_config = bus_space_read_4(t, mac, CAS_MIF_CONFIG);
#endif

	/* MII needs echo disable if half duplex. */
	if ((IFM_OPTIONS(sc->sc_mii.mii_media_active) & IFM_FDX) != 0)
		/* turn on full duplex LED */
		v |= CAS_MAC_XIF_FDPLX_LED;
	else
		/* half duplex -- disable echo */
		v |= CAS_MAC_XIF_ECHO_DISABL;

	switch (IFM_SUBTYPE(sc->sc_mii.mii_media_active)) {
	case IFM_1000_T:  /* Gigabit using GMII interface */
	case IFM_1000_SX:
		v |= CAS_MAC_XIF_GMII_MODE;
		break;
	default:
		v &= ~CAS_MAC_XIF_GMII_MODE;
	}
	bus_space_write_4(t, mac, CAS_MAC_XIF_CONFIG, v);
}

int
cas_pcs_readreg(struct device *self, int phy, int reg)
{
	struct cas_softc *sc = (void *)self;
	bus_space_tag_t t = sc->sc_memt;
	bus_space_handle_t pcs = sc->sc_memh;

#ifdef CAS_DEBUG
	if (sc->sc_debug)
		printf("cas_pcs_readreg: phy %d reg %d\n", phy, reg);
#endif

	if (phy != CAS_PHYAD_EXTERNAL)
		return (0);

	switch (reg) {
	case MII_BMCR:
		reg = CAS_MII_CONTROL;
		break;
	case MII_BMSR:
		reg = CAS_MII_STATUS;
		break;
	case MII_ANAR:
		reg = CAS_MII_ANAR;
		break;
	case MII_ANLPAR:
		reg = CAS_MII_ANLPAR;
		break;
	case MII_EXTSR:
		return (EXTSR_1000XFDX|EXTSR_1000XHDX);
	default:
		return (0);
	}

	return bus_space_read_4(t, pcs, reg);
}

void
cas_pcs_writereg(struct device *self, int phy, int reg, int val)
{
	struct cas_softc *sc = (void *)self;
	bus_space_tag_t t = sc->sc_memt;
	bus_space_handle_t pcs = sc->sc_memh;

#ifdef CAS_DEBUG
	if (sc->sc_debug)
		printf("cas_pcs_writereg: phy %d reg %d val %x\n",
			phy, reg, val);
#endif

	if (phy != CAS_PHYAD_EXTERNAL)
		return;

	switch (reg) {
	case MII_BMCR:
		reg = CAS_MII_CONTROL;
		break;
	case MII_BMSR:
		reg = CAS_MII_STATUS;
		break;
	case MII_ANAR:
		reg = CAS_MII_ANAR;
		break;
	case MII_ANLPAR:
		reg = CAS_MII_ANLPAR;
		break;
	default:
		return;
	}

	bus_space_write_4(t, pcs, reg, val);

	if (reg == CAS_MII_ANAR) {
		bus_space_write_4(t, pcs, CAS_MII_SLINK_CONTROL,
		    CAS_MII_SLINK_LOOPBACK|CAS_MII_SLINK_EN_SYNC_D);
		bus_space_write_4(t, pcs, CAS_MII_CONFIG,
		    CAS_MII_CONFIG_ENABLE);
	}
}

int
cas_mediachange(struct ifnet *ifp)
{
	struct cas_softc *sc = ifp->if_softc;
	struct mii_data *mii = &sc->sc_mii;

	if (mii->mii_instance) {
		struct mii_softc *miisc;
		LIST_FOREACH(miisc, &mii->mii_phys, mii_list)
			mii_phy_reset(miisc);
	}

	return (mii_mediachg(&sc->sc_mii));
}

void
cas_mediastatus(struct ifnet *ifp, struct ifmediareq *ifmr)
{
	struct cas_softc *sc = ifp->if_softc;

	mii_pollstat(&sc->sc_mii);
	ifmr->ifm_active = sc->sc_mii.mii_media_active;
	ifmr->ifm_status = sc->sc_mii.mii_media_status;
}

/*
 * Process an ioctl request.
 */
int
cas_ioctl(struct ifnet *ifp, u_long cmd, caddr_t data)
{
	struct cas_softc *sc = ifp->if_softc;
	struct ifaddr *ifa = (struct ifaddr *)data;
	struct ifreq *ifr = (struct ifreq *)data;
	int s, error = 0;

	s = splnet();

	if ((error = ether_ioctl(ifp, &sc->sc_arpcom, cmd, data)) > 0) {
		splx(s);
		return (error);
	}

	switch (cmd) {

	case SIOCSIFADDR:
		ifp->if_flags |= IFF_UP;
		if ((ifp->if_flags & IFF_RUNNING) == 0)
			cas_init(ifp);
#ifdef INET
		if (ifa->ifa_addr->sa_family == AF_INET)
			arp_ifinit(&sc->sc_arpcom, ifa);
#endif
		break;

	case SIOCSIFFLAGS:
		if (ifp->if_flags & IFF_UP) {
			if ((ifp->if_flags & IFF_RUNNING) &&
			    ((ifp->if_flags ^ sc->sc_if_flags) &
			     (IFF_ALLMULTI | IFF_PROMISC)) != 0)
				cas_setladrf(sc);
			else {
				if ((ifp->if_flags & IFF_RUNNING) == 0)
					cas_init(ifp);
			}
		} else {
			if (ifp->if_flags & IFF_RUNNING)
				cas_stop(ifp, 1);
		}
		sc->sc_if_flags = ifp->if_flags;

#ifdef CAS_DEBUG
		sc->sc_debug = (ifp->if_flags & IFF_DEBUG) != 0 ? 1 : 0;
#endif
		break;

	case SIOCSIFMTU:
		if (ifr->ifr_mtu > ETHERMTU || ifr->ifr_mtu < ETHERMIN) {
			error = EINVAL;
		} else if (ifp->if_mtu != ifr->ifr_mtu) {
			ifp->if_mtu = ifr->ifr_mtu;
		}
		break;

	case SIOCADDMULTI:
	case SIOCDELMULTI:
		error = (cmd == SIOCADDMULTI) ?
		    ether_addmulti(ifr, &sc->sc_arpcom) :
		    ether_delmulti(ifr, &sc->sc_arpcom);

		if (error == ENETRESET) {
			/*
			 * Multicast list has changed; set the hardware filter
			 * accordingly.
			 */
			if (ifp->if_flags & IFF_RUNNING)
				cas_setladrf(sc);
			error = 0;
		}
		break;

	case SIOCGIFMEDIA:
	case SIOCSIFMEDIA:
		error = ifmedia_ioctl(ifp, ifr, &sc->sc_media, cmd);
		break;

	default:
		error = EINVAL;
		break;
	}

	splx(s);
	return (error);
}


void
cas_shutdown(void *arg)
{
	struct cas_softc *sc = (struct cas_softc *)arg;
	struct ifnet *ifp = &sc->sc_arpcom.ac_if;

	cas_stop(ifp, 1);
}

/*
 * Set up the logical address filter.
 */
void
cas_setladrf(struct cas_softc *sc)
{
	struct ifnet *ifp = &sc->sc_arpcom.ac_if;
	struct ether_multi *enm;
	struct ether_multistep step;
	struct arpcom *ac = &sc->sc_arpcom;
	bus_space_tag_t t = sc->sc_memt;
	bus_space_handle_t h = sc->sc_memh;
	u_int32_t crc, hash[16], v;
	int i;

	/* Get current RX configuration */
	v = bus_space_read_4(t, h, CAS_MAC_RX_CONFIG);


	/*
	 * Turn off promiscuous mode, promiscuous group mode (all multicast),
	 * and hash filter.  Depending on the case, the right bit will be
	 * enabled.
	 */
	v &= ~(CAS_MAC_RX_PROMISCUOUS|CAS_MAC_RX_HASH_FILTER|
	    CAS_MAC_RX_PROMISC_GRP);

	if ((ifp->if_flags & IFF_PROMISC) != 0) {
		/* Turn on promiscuous mode */
		v |= CAS_MAC_RX_PROMISCUOUS;
		ifp->if_flags |= IFF_ALLMULTI;
		goto chipit;
	}

	/*
	 * Set up multicast address filter by passing all multicast addresses
	 * through a crc generator, and then using the high order 8 bits as an
	 * index into the 256 bit logical address filter.  The high order 4
	 * bits selects the word, while the other 4 bits select the bit within
	 * the word (where bit 0 is the MSB).
	 */

	/* Clear hash table */
	for (i = 0; i < 16; i++)
		hash[i] = 0;


	ETHER_FIRST_MULTI(step, ac, enm);
	while (enm != NULL) {
		if (bcmp(enm->enm_addrlo, enm->enm_addrhi, ETHER_ADDR_LEN)) {
			/*
			 * We must listen to a range of multicast addresses.
			 * For now, just accept all multicasts, rather than
			 * trying to set only those filter bits needed to match
			 * the range.  (At this time, the only use of address
			 * ranges is for IP multicast routing, for which the
			 * range is big enough to require all bits set.)
			 * XXX use the addr filter for this
			 */
			ifp->if_flags |= IFF_ALLMULTI;
			v |= CAS_MAC_RX_PROMISC_GRP;
			goto chipit;
		}

		crc = ether_crc32_le(enm->enm_addrlo, ETHER_ADDR_LEN);

		/* Just want the 8 most significant bits. */
		crc >>= 24;

		/* Set the corresponding bit in the filter. */
		hash[crc >> 4] |= 1 << (15 - (crc & 15));

		ETHER_NEXT_MULTI(step, enm);
	}

	v |= CAS_MAC_RX_HASH_FILTER;
	ifp->if_flags &= ~IFF_ALLMULTI;

	/* Now load the hash table into the chip (if we are using it) */
	for (i = 0; i < 16; i++) {
		bus_space_write_4(t, h,
		    CAS_MAC_HASH0 + i * (CAS_MAC_HASH1-CAS_MAC_HASH0),
		    hash[i]);
	}

chipit:
	bus_space_write_4(t, h, CAS_MAC_RX_CONFIG, v);
}

int
cas_encap(struct cas_softc *sc, struct mbuf *mhead, u_int32_t *bixp)
{
	u_int64_t flags;
	u_int32_t cur, frag, i;
	bus_dmamap_t map;

	cur = frag = *bixp;
	map = sc->sc_txd[cur].sd_map;

	if (bus_dmamap_load_mbuf(sc->sc_dmatag, map, mhead,
	    BUS_DMA_NOWAIT) != 0) {
		return (ENOBUFS);
	}

	if ((sc->sc_tx_cnt + map->dm_nsegs) > (CAS_NTXDESC - 2)) {
		bus_dmamap_unload(sc->sc_dmatag, map);
		return (ENOBUFS);
	}

	bus_dmamap_sync(sc->sc_dmatag, map, 0, map->dm_mapsize,
	    BUS_DMASYNC_PREWRITE);

	for (i = 0; i < map->dm_nsegs; i++) {
		sc->sc_txdescs[frag].gd_addr =
		    CAS_DMA_WRITE(sc, map->dm_segs[i].ds_addr);
		flags = (map->dm_segs[i].ds_len & CAS_TD_BUFSIZE) |
		    (i == 0 ? CAS_TD_START_OF_PACKET : 0) |
		    ((i == (map->dm_nsegs - 1)) ? CAS_TD_END_OF_PACKET : 0);
		sc->sc_txdescs[frag].gd_flags = CAS_DMA_WRITE(sc, flags);
		bus_dmamap_sync(sc->sc_dmatag, sc->sc_cddmamap,
		    CAS_CDTXOFF(frag), sizeof(struct cas_desc),
		    BUS_DMASYNC_PREWRITE);
		cur = frag;
		if (++frag == CAS_NTXDESC)
			frag = 0;
	}

	sc->sc_tx_cnt += map->dm_nsegs;
	sc->sc_txd[*bixp].sd_map = sc->sc_txd[cur].sd_map;
	sc->sc_txd[cur].sd_map = map;
	sc->sc_txd[cur].sd_mbuf = mhead;

	bus_space_write_4(sc->sc_memt, sc->sc_memh, CAS_TX_KICK, frag);

	*bixp = frag;

	/* sync descriptors */

	return (0);
}

/*
 * Transmit interrupt.
 */
int
cas_tint(struct cas_softc *sc, u_int32_t status)
{
	struct ifnet *ifp = &sc->sc_arpcom.ac_if;
	struct cas_sxd *sd;
	u_int32_t cons, hwcons;

	hwcons = status >> 19;
	cons = sc->sc_tx_cons;
	while (cons != hwcons) {
		sd = &sc->sc_txd[cons];
		if (sd->sd_mbuf != NULL) {
			bus_dmamap_sync(sc->sc_dmatag, sd->sd_map, 0,
			    sd->sd_map->dm_mapsize, BUS_DMASYNC_POSTWRITE);
			bus_dmamap_unload(sc->sc_dmatag, sd->sd_map);
			m_freem(sd->sd_mbuf);
			sd->sd_mbuf = NULL;
		}
		sc->sc_tx_cnt--;
		ifp->if_opackets++;
		if (++cons == CAS_NTXDESC)
			cons = 0;
	}
	sc->sc_tx_cons = cons;

	cas_start(ifp);

	if (sc->sc_tx_cnt == 0)
		ifp->if_timer = 0;

	return (1);
}

void
cas_start(struct ifnet *ifp)
{
	struct cas_softc *sc = ifp->if_softc;
	struct mbuf *m;
	u_int32_t bix;

	if ((ifp->if_flags & (IFF_RUNNING | IFF_OACTIVE)) != IFF_RUNNING)
		return;

	bix = sc->sc_tx_prod;
	while (sc->sc_txd[bix].sd_mbuf == NULL) {
		IFQ_POLL(&ifp->if_snd, m);
		if (m == NULL)
			break;

#if NBPFILTER > 0
		/*
		 * If BPF is listening on this interface, let it see the
		 * packet before we commit it to the wire.
		 */
		if (ifp->if_bpf)
			bpf_mtap(ifp->if_bpf, m, BPF_DIRECTION_OUT);
#endif

		/*
		 * Encapsulate this packet and start it going...
		 * or fail...
		 */
		if (cas_encap(sc, m, &bix)) {
			ifp->if_timer = 2;
			break;
		}

		IFQ_DEQUEUE(&ifp->if_snd, m);
		ifp->if_timer = 5;
	}

	sc->sc_tx_prod = bix;
}