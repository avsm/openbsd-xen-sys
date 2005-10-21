/*	$OpenBSD: if_hme_pci.c,v 1.8 2003/02/14 21:05:12 henric Exp $	*/
/*	$NetBSD: if_hme_pci.c,v 1.3 2000/12/28 22:59:13 sommerfeld Exp $	*/

/*
 * Copyright (c) 2000 Matthew R. Green
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
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

/*
 * PCI front-end device driver for the HME ethernet device.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/syslog.h>
#include <sys/device.h>
#include <sys/malloc.h>
#include <sys/socket.h>

#include <net/if.h>
#include <net/if_dl.h>
#include <net/if_media.h>

#ifdef INET
#include <netinet/in.h>
#include <netinet/in_systm.h>
#include <netinet/in_var.h>
#include <netinet/ip.h>
#include <netinet/if_ether.h>
#endif

#include <dev/mii/mii.h>
#include <dev/mii/miivar.h>

#ifdef __sparc64__
#include <machine/autoconf.h>
#include <dev/ofw/openfirm.h>
#endif
#include <machine/cpu.h>

#include <dev/pci/pcivar.h>
#include <dev/pci/pcireg.h>
#include <dev/pci/pcidevs.h>

#include <dev/ic/hmevar.h>

struct hme_pci_softc {
	struct	hme_softc	hsc_hme;	/* HME device */
	bus_space_tag_t		hsc_memt;
	bus_space_handle_t	hsc_memh;
	void			*hsc_ih;
};

int	hmematch_pci(struct device *, void *, void *);
void	hmeattach_pci(struct device *, struct device *, void *);
int	hme_pci_enaddr(struct hme_softc *, struct pci_attach_args *);

struct cfattach hme_pci_ca = {
	sizeof(struct hme_pci_softc), hmematch_pci, hmeattach_pci
};

int
hmematch_pci(parent, vcf, aux)
	struct device *parent;
	void *vcf;
	void *aux;
{
	struct pci_attach_args *pa = aux;

	if (PCI_VENDOR(pa->pa_id) == PCI_VENDOR_SUN && 
	    PCI_PRODUCT(pa->pa_id) == PCI_PRODUCT_SUN_HME)
		return (1);

	return (0);
}

#define	PCI_EBUS2_BOOTROM	0x10
#define	PROMHDR_PTR_DATA	0x18
#define	PROMDATA_PTR_VPD	0x08
#define	PROMDATA_DATA2		0x0a

static const u_int8_t hme_promhdr[] = { 0x55, 0xaa };
static const u_int8_t hme_promdat[] = {
	'P', 'C', 'I', 'R',
	PCI_VENDOR_SUN & 0xff, PCI_VENDOR_SUN >> 8,
	PCI_PRODUCT_SUN_HME & 0xff, PCI_PRODUCT_SUN_HME >> 8
};
static const u_int8_t hme_promdat2[] = {
	0x18, 0x00,			/* structure length */
	0x00,				/* structure revision */
	0x00,				/* interface revision */
	PCI_SUBCLASS_NETWORK_ETHERNET,	/* subclass code */
	PCI_CLASS_NETWORK		/* class code */
};

int
hme_pci_enaddr(struct hme_softc *sc, struct pci_attach_args *hpa)
{
	struct pci_attach_args epa;
	struct pci_vpd *vpd;
	pcireg_t cl, id;
	bus_space_handle_t romh;
	bus_space_tag_t romt;
	bus_size_t romsize = 0;
	u_int8_t buf[32];
	int dataoff, vpdoff;

	/*
	 * Dig out VPD (vital product data) and acquire Ethernet address.
	 * The VPD of hme resides in the Boot PROM (PCI FCode) attached
	 * to the EBus interface.
	 * ``Writing FCode 3.x Programs'' (newer ones, dated 1997 and later)
	 * chapter 2 describes the data structure.
	 */

	/* get a PCI tag for the EBus bridge (function 0 of the same device) */
	epa = *hpa;
	epa.pa_tag = pci_make_tag(hpa->pa_pc, hpa->pa_bus, hpa->pa_device, 0);
	cl = pci_conf_read(epa.pa_pc, epa.pa_tag, PCI_CLASS_REG);
	id = pci_conf_read(epa.pa_pc, epa.pa_tag, PCI_ID_REG);

	if (PCI_CLASS(cl) != PCI_CLASS_BRIDGE ||
	    PCI_PRODUCT(id) != PCI_PRODUCT_SUN_EBUS)
		goto fail;

	if (pci_mapreg_map(&epa, PCI_EBUS2_BOOTROM, PCI_MAPREG_TYPE_MEM, 0,
	    &romt, &romh, 0, &romsize, 0))
		goto fail;

	bus_space_read_region_1(romt, romh, 0, buf, sizeof(buf));
	if (bcmp(buf, hme_promhdr, sizeof(hme_promhdr)))
		goto fail;

	dataoff = buf[PROMHDR_PTR_DATA] | (buf[PROMHDR_PTR_DATA + 1] << 8);
	if (dataoff < 0x1c)
		goto fail;

	bus_space_read_region_1(romt, romh, dataoff, buf, sizeof(buf));
	if (bcmp(buf, hme_promdat, sizeof(hme_promdat)) ||
	    bcmp(buf + PROMDATA_DATA2, hme_promdat2, sizeof(hme_promdat2)))
		goto fail;

	vpdoff = buf[PROMDATA_PTR_VPD] | (buf[PROMDATA_PTR_VPD + 1] << 8);
	if (vpdoff < 0x1c)
		goto fail;

	/*
	 * The VPD of hme is not in PCI 2.2 standard format.  The length
	 * in the resource header is in big endian, and resources are not
	 * properly terminated (only one resource and no end tag).
	 */
	bus_space_read_region_1(romt, romh, vpdoff, buf, sizeof(buf));

	/* XXX TODO: Get the data from VPD */
	vpd = (struct pci_vpd *)(buf + 3);
	if (!PCI_VPDRES_ISLARGE(buf[0]) ||
	    PCI_VPDRES_LARGE_NAME(buf[0]) != PCI_VPDRES_TYPE_VPD)
		goto fail;
	if (vpd->vpd_key0 != 'N' || vpd->vpd_key1 != 'A')
		goto fail;

	bcopy(buf + 6, sc->sc_enaddr, ETHER_ADDR_LEN);
	sc->sc_enaddr[5] += hpa->pa_device;
	bus_space_unmap(romt, romh, romsize);
	return (0);

fail:
	if (romsize != 0)
		bus_space_unmap(romt, romh, romsize);
	return (-1);
}

void
hmeattach_pci(parent, self, aux)
	struct device *parent, *self;
	void *aux;
{
	struct pci_attach_args *pa = aux;
	struct hme_pci_softc *hsc = (void *)self;
	struct hme_softc *sc = &hsc->hsc_hme;
	pci_intr_handle_t intrhandle;
	/* XXX the following declarations should be elsewhere */
	extern void myetheraddr(u_char *);
	pcireg_t csr;
	const char *intrstr;
	int type, gotenaddr = 0;

	/*
	 * enable io/memory-space accesses.  this is kinda of gross; but
	 * the hme comes up with neither IO space enabled, or memory space.
	 */
	if (pa->pa_memt)
		pa->pa_flags |= PCI_FLAGS_MEM_ENABLED;
	if (pa->pa_iot)
		pa->pa_flags |= PCI_FLAGS_IO_ENABLED;
	csr = pci_conf_read(pa->pa_pc, pa->pa_tag, PCI_COMMAND_STATUS_REG);
	if (pa->pa_memt) {
		type = PCI_MAPREG_TYPE_MEM;
		csr |= PCI_COMMAND_MEM_ENABLE;
		sc->sc_bustag = pa->pa_memt;
	} else {
		type = PCI_MAPREG_TYPE_IO;
		csr |= PCI_COMMAND_IO_ENABLE;
		sc->sc_bustag = pa->pa_iot;
	}
	pci_conf_write(pa->pa_pc, pa->pa_tag, PCI_COMMAND_STATUS_REG,
	    csr | PCI_COMMAND_MEM_ENABLE);

	sc->sc_dmatag = pa->pa_dmat;

	sc->sc_pci = 1; /* XXXXX should all be done in bus_dma. */
	/*
	 * Map five register banks:
	 *
	 *	bank 0: HME SEB registers:	+0x0000
	 *	bank 1: HME ETX registers:	+0x2000
	 *	bank 2: HME ERX registers:	+0x4000
	 *	bank 3: HME MAC registers:	+0x6000
	 *	bank 4: HME MIF registers:	+0x7000
	 *
	 */

#define PCI_HME_BASEADDR	0x10
	if (pci_mapreg_map(pa, PCI_HME_BASEADDR, type, 0,
	    &hsc->hsc_memt, &hsc->hsc_memh, NULL, NULL, 0) != 0) {
		printf(": could not map hme registers\n");
		return;
	}
	sc->sc_seb = hsc->hsc_memh;
	bus_space_subregion(sc->sc_bustag, hsc->hsc_memh, 0x2000, 0x2000,
	    &sc->sc_etx);
	bus_space_subregion(sc->sc_bustag, hsc->hsc_memh, 0x4000, 0x2000,
	    &sc->sc_erx);
	bus_space_subregion(sc->sc_bustag, hsc->hsc_memh, 0x6000, 0x1000,
	    &sc->sc_mac);
	bus_space_subregion(sc->sc_bustag, hsc->hsc_memh, 0x7000, 0x1000,
	    &sc->sc_mif);

	if (hme_pci_enaddr(sc, pa) == 0)
		gotenaddr = 1;

#ifdef __sparc__
	if (!gotenaddr) {
		if (OF_getprop(PCITAG_NODE(pa->pa_tag), "local-mac-address",
		    sc->sc_enaddr, ETHER_ADDR_LEN) <= 0)
		myetheraddr(sc->sc_enaddr);
		gotenaddr = 1;
	}
#endif
#ifdef __powerpc__
	if (!gotenaddr) {
		pci_ether_hw_addr(pa->pa_pc, sc->sc_enaddr);
		gotenaddr = 1;
	}
#endif


	sc->sc_burst = 16;	/* XXX */

	if (pci_intr_map(pa, &intrhandle) != 0) {
		printf(": couldn't map interrupt\n");
		return;	/* bus_unmap ? */
	}	
	intrstr = pci_intr_string(pa->pa_pc, intrhandle);
	hsc->hsc_ih = pci_intr_establish(pa->pa_pc, intrhandle, IPL_NET,
	    hme_intr, sc, self->dv_xname);
	if (hsc->hsc_ih != NULL) {
		printf(": %s", intrstr ? intrstr : "unknown interrupt");
	} else {
		printf(": couldn't establish interrupt");
		if (intrstr != NULL)
			printf(" at %s", intrstr);
		printf("\n");
		return;	/* bus_unmap ? */
	}

	/*
	 * call the main configure
	 */
	hme_config(sc);
}
