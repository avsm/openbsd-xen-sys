/*	$OpenBSD$ */

/*
 * Copyright (c) 2006 Jon Simola <jsimola@gmail.com>
 * Copyright (c) 2003 Clifford Wright. All rights reserved.
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
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR `AS IS'' AND ANY EXPRESS OR
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
 * PCI shim for Broadcom BCM43xx Wireless network chipsets (broadcom.com)
 * SiliconBackplane is technology from Sonics, Inc.(sonicsinc.com)
 */
 
/* standard includes, probably some extras */

#include <sys/cdefs.h>
#include <sys/param.h>
#include <sys/systm.h>
#include <sys/timeout.h>
#include <sys/sockio.h>
#include <sys/mbuf.h>
#include <sys/malloc.h>
#include <sys/kernel.h>
#include <sys/device.h>
#include <sys/socket.h>

#include <machine/endian.h>

#include <net/if.h>
#include <net/if_dl.h>
#include <net/if_media.h>

#ifdef INET
#include <netinet/in.h>
//#include <netinet/in_systm.h>
//#include <netinet/in_var.h>
//#include <netinet/ip.h>
#include <netinet/if_ether.h>
#endif
#if NBPFILTER > 0
#include <net/bpf.h>
#endif

#include <net80211/ieee80211_var.h>
#include <net80211/ieee80211_radiotap.h>

#include <machine/bus.h>
#include <machine/intr.h>

#include <dev/pci/pcireg.h>
#include <dev/pci/pcivar.h>
#include <dev/pci/pcidevs.h>

#include <dev/ic/bcwreg.h>
#include <dev/ic/bcwvar.h>

#include <uvm/uvm_extern.h>

const struct pci_matchid bcw_pci_devices[]  = {
	{ PCI_VENDOR_BROADCOM, PCI_PRODUCT_BROADCOM_BCM4303 },
	{ PCI_VENDOR_BROADCOM, PCI_PRODUCT_BROADCOM_BCM4306 },
	{ PCI_VENDOR_BROADCOM, PCI_PRODUCT_BROADCOM_BCM4306_2 },
	{ PCI_VENDOR_BROADCOM, PCI_PRODUCT_BROADCOM_BCM4307 },
	{ PCI_VENDOR_BROADCOM, PCI_PRODUCT_BROADCOM_BCM4309 },
	{ PCI_VENDOR_BROADCOM, PCI_PRODUCT_BROADCOM_BCM4311 },
	{ PCI_VENDOR_BROADCOM, PCI_PRODUCT_BROADCOM_BCM4318 },
	{ PCI_VENDOR_BROADCOM, PCI_PRODUCT_BROADCOM_BCM4319 },
	{ PCI_VENDOR_BROADCOM, PCI_PRODUCT_BROADCOM_BCM4322 },
	{ PCI_VENDOR_BROADCOM, PCI_PRODUCT_BROADCOM_BCM43XG }
};


struct bcw_pci_softc {
	struct bcw_softc	psc_bcw;	/* Real softc */

	pci_intr_handle_t	psc_ih;		/* interrupt handle */
	void			*psc_intrcookie;
	
	pci_chipset_tag_t	psc_pc;		/* our PCI chipset */
	pcitag_t		psc_pcitag;	/* our PCI tag */
};

int		bcw_pci_match(struct device *, void *, void *);
int		bcw_pci_enable(struct bcw_softc *sc);
void		bcw_pci_disable(struct bcw_softc *sc);
void		bcw_pci_attach(struct device *, struct device *, void *);

struct cfattach bcw_pci_ca = {
	sizeof(struct bcw_pci_softc), bcw_pci_match, bcw_pci_attach
};


int
bcw_pci_match(struct device *parent, void *match, void *aux)
{
	return pci_matchbyid((struct pci_attach_args *)aux, bcw_pci_devices,
	    sizeof (bcw_pci_devices) / sizeof (bcw_pci_devices[0]));
}

int
bcw_pci_enable(struct bcw_softc *sc)
{
	struct bcw_pci_softc *psc = (void *)sc;
	
	/* Establish PCI interrupt */
	psc->psc_intrcookie = pci_intr_establish(psc->psc_pc, psc->psc_ih,
	    IPL_NET, bcw_intr, sc, sc->bcw_dev.dv_xname);
	if(psc->psc_intrcookie == NULL) {
		printf("%s: unable to establish interrupt\n",
		    sc->bcw_dev.dv_xname);
		return (1);
	}
	
	return (0);
}

void
bcw_pci_disable(struct bcw_softc *sc)
{
	struct bcw_pci_softc *psc = (void *)sc;
	
	/* Remove PCI interrupt */
	pci_intr_disestablish(psc->psc_pc, psc->psc_intrcookie);
	psc->psc_intrcookie = NULL;
}

void
bcw_pci_attach(parent, self, aux)
	struct device  *parent, *self;
	void           *aux;
{
	struct bcw_pci_softc *psc = (void *) self;
	struct bcw_softc *sc = &psc->psc_bcw;
	struct bcw_regs *regs = &sc->bcw_regs;
	struct pci_attach_args *pa = (struct pci_attach_args *)aux;
	pci_chipset_tag_t pc = pa->pa_pc;
//	const char     *intrstr = NULL;
//	caddr_t         kva;
//	bus_dma_segment_t seg;
//	int             rseg;
	pcireg_t        memtype;
	bus_addr_t      memaddr;
	bus_size_t      memsize;
//	bus_space_tag_t	memt;
//	bus_space_handle_t	memh;
//	int		ioh_valid, memh_valid;
	int             pmreg;
	pcireg_t        pmode;

//	int             error;
//	int             i,j;
	u_int32_t	sbval;

	psc->psc_pc = pa->pa_pc;
	psc->psc_pcitag = pa->pa_tag;

	/* Get it out of power save mode if needed. */
	if (pci_get_capability(pc, pa->pa_tag, PCI_CAP_PWRMGMT, &pmreg, 0)) {
		pmode = pci_conf_read(pc, pa->pa_tag, pmreg + 4) & 0x3;
		if (pmode == 3) {
			/*
			 * The card has lost all configuration data in
			 * this state, so punt.
			 */
			printf("%s: unable to wake up from power state D3\n",
			       sc->bcw_dev.dv_xname);
			return;
		}
		if (pmode != 0) {
			printf("%s: waking up from power state D%d\n",
			       sc->bcw_dev.dv_xname, pmode);
			pci_conf_write(pc, pa->pa_tag, pmreg + 4, 0);
		}
	}


	/*
	 * Map control/status registers.
	 */
	/* Copied from pre-abstraction, via if_bce.c */
	memtype = pci_mapreg_type(pa->pa_pc, pa->pa_tag, BCW_PCI_BAR0);
	switch (memtype) {
	case PCI_MAPREG_TYPE_MEM | PCI_MAPREG_MEM_TYPE_32BIT:
	case PCI_MAPREG_TYPE_MEM | PCI_MAPREG_MEM_TYPE_64BIT:
		if (pci_mapreg_map(pa, BCW_PCI_BAR0, memtype, 0, &sc->bcw_btag,
		    &sc->bcw_bhandle, &memaddr, &memsize, 0) == 0)
			break;
	default:
		printf("%s: unable to find mem space\n",
		    sc->bcw_dev.dv_xname);
		return;
	}
	regs->r_bt = sc->bcw_btag;
	regs->r_bh = sc->bcw_bhandle;

	sc->bcw_dmatag = pa->pa_dmat;


	/* Map the PCI interrupt */
	if (pci_intr_map(pa, &psc->psc_ih)) {
		printf("%s: couldn't map interrupt\n",
		    sc->bcw_dev.dv_xname);
		return;
	}

	sc->bcw_intrstr = pci_intr_string(pc, psc->psc_ih);

	psc->psc_intrcookie = pci_intr_establish(pc, psc->psc_ih, IPL_NET, 
	    bcw_intr, sc, sc->bcw_dev.dv_xname);

	if (psc->psc_intrcookie == NULL) {
		printf("%s: couldn't establish interrupt",
		    sc->bcw_dev.dv_xname);
		if (sc->bcw_intrstr != NULL)
			printf(" at %s", sc->bcw_intrstr);
		printf("\n");
		return;
	}

	printf(": %s\n", sc->bcw_intrstr);

	sc->sc_enable = bcw_pci_enable;
	sc->sc_disable = bcw_pci_disable;

	/*
	 * Get some PCI based info into the softc
	 */
	sc->bcw_chiprev=PCI_REVISION(pa->pa_class);
	sc->bcw_prodid=PCI_PRODUCT(pa->pa_id);

	/*
	 * Start the card up while we're in PCI land
	 */

	/* Turn the Crystal On */
	sbval = bus_space_read_4(sc->bcw_btag, sc->bcw_bhandle, BCW_GPIOI);
	if ((sbval & BCW_XTALPOWERUP) != BCW_XTALPOWERUP) {
	    sbval = bus_space_read_4(sc->bcw_btag, sc->bcw_bhandle, BCW_GPIOO);
	    sbval |= (BCW_XTALPOWERUP & BCW_PLLPOWERDOWN);
	    bus_space_write_4(sc->bcw_btag, sc->bcw_bhandle, BCW_GPIOO, sbval);
	    delay(1000);
	    sbval = bus_space_read_4(sc->bcw_btag, sc->bcw_bhandle, BCW_GPIOO);
	    sbval &= ~BCW_PLLPOWERDOWN;
	    bus_space_write_4(sc->bcw_btag, sc->bcw_bhandle, BCW_GPIOO, sbval);
	    delay(5000);
	}
	
	/*
	 * Clear PCI_STATUS_TARGET_TARGET_ABORT, Docs and Linux call it 
	 * PCI_STATUS_SIG_TARGET_ABORT - should use pci_conf_read/write?
	 */
	bus_space_write_4(sc->bcw_btag, sc->bcw_bhandle,
	    PCI_COMMAND_STATUS_REG,
	    bus_space_read_4(sc->bcw_btag, sc->bcw_bhandle,
	        PCI_COMMAND_STATUS_REG)
	    & ~PCI_STATUS_TARGET_TARGET_ABORT);
	

	/*
	 * Finish the attach
	 */

	bcw_attach(sc);
}