/*	$OpenBSD: if_epic_pci.c,v 1.1 2005/05/10 01:16:32 brad Exp $	*/
/*	$NetBSD: if_epic_pci.c,v 1.28 2005/02/27 00:27:32 perry Exp $	*/

/*-
 * Copyright (c) 1998, 1999 The NetBSD Foundation, Inc.
 * All rights reserved.
 *
 * This code is derived from software contributed to The NetBSD Foundation
 * by Jason R. Thorpe of the Numerical Aerospace Simulation Facility,
 * NASA Ames Research Center.
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
 *	This product includes software developed by the NetBSD
 *	Foundation, Inc. and its contributors.
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

/*
 * PCI bus front-end for the Standard Microsystems Corp. 83C170
 * Ethernet PCI Integrated Controller (EPIC/100) driver.
 */

#if 0
#include <sys/cdefs.h>
__KERNEL_RCSID(0, "$NetBSD: if_epic_pci.c,v 1.28 2005/02/27 00:27:32 perry Exp $");
#endif

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/mbuf.h>
#include <sys/malloc.h>
#include <sys/kernel.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/errno.h>
#include <sys/device.h>

#include <net/if.h>
#include <net/if_dl.h>
#include <net/if_types.h>

#ifdef INET
#include <netinet/in.h>
#include <netinet/in_systm.h>
#include <netinet/in_var.h>
#include <netinet/ip.h>
#include <netinet/if_ether.h>
#endif

#include <net/if_media.h>

#include <machine/bus.h>
#include <machine/intr.h>

#include <dev/mii/miivar.h>

#include <dev/ic/smc83c170reg.h>
#include <dev/ic/smc83c170var.h>

#include <dev/pci/pcivar.h>
#include <dev/pci/pcireg.h>
#include <dev/pci/pcidevs.h>

/*
 * PCI configuration space registers used by the EPIC.
 */
#define	EPIC_PCI_IOBA		0x10	/* i/o mapped base */
#define	EPIC_PCI_MMBA		0x14	/* memory mapped base */

struct epic_pci_softc {
	struct epic_softc sc_epic;	/* real EPIC softc */

	/* PCI-specific goo. */
	void	*sc_ih;			/* interrupt handle */
};

static int	epic_pci_match(struct device *, void *, void *);
static void	epic_pci_attach(struct device *, struct device *, void *);

struct cfattach epic_pci_ca = {
	sizeof(struct epic_pci_softc), epic_pci_match, epic_pci_attach
};

const struct pci_matchid epic_pci_devices[] = {
	{ PCI_VENDOR_SMC, PCI_PRODUCT_SMC_83C170 },
	{ PCI_VENDOR_SMC, PCI_PRODUCT_SMC_83C175 },
};

static const struct epic_pci_subsys_info {
	pcireg_t subsysid;
	int flags;
} epic_pci_subsys_info[] = {
	{ PCI_ID_CODE(PCI_VENDOR_SMC, 0xa015), /* SMC9432BTX */
	  EPIC_HAS_BNC },
	{ PCI_ID_CODE(PCI_VENDOR_SMC, 0xa024), /* SMC9432BTX1 */
	  EPIC_HAS_BNC },
	{ PCI_ID_CODE(PCI_VENDOR_SMC, 0xa016), /* SMC9432FTX */
	  EPIC_HAS_MII_FIBER | EPIC_DUPLEXLED_ON_694 },
	{ 0xffffffff,
	  0 }
};

static const struct epic_pci_subsys_info *
epic_pci_subsys_lookup(const struct pci_attach_args *pa)
{
	pci_chipset_tag_t pc = pa->pa_pc;
	pcireg_t reg;
	const struct epic_pci_subsys_info *esp;

	reg = pci_conf_read(pc, pa->pa_tag, PCI_SUBSYS_ID_REG);

	for (esp = epic_pci_subsys_info; esp->subsysid != 0xffffffff; esp++)
		if (esp->subsysid == reg)
			return (esp);

	return (NULL);
}

static int
epic_pci_match(struct device *parent, void *match, void *aux)
{
	return (pci_matchbyid((struct pci_attach_args *)aux, epic_pci_devices,
	    sizeof(epic_pci_devices)/sizeof(epic_pci_devices[0])));
}

static void
epic_pci_attach(struct device *parent, struct device *self, void *aux)
{
	struct epic_pci_softc *psc = (struct epic_pci_softc *)self;
	struct epic_softc *sc = &psc->sc_epic;
	struct pci_attach_args *pa = aux;
	pci_chipset_tag_t pc = pa->pa_pc;
	pci_intr_handle_t ih;
	const char *intrstr = NULL;
	const struct epic_pci_subsys_info *esp;
	bus_space_tag_t iot, memt;
	bus_space_handle_t ioh, memh;
	pcireg_t reg;
	int pmreg, ioh_valid, memh_valid;

	if (pci_get_capability(pc, pa->pa_tag, PCI_CAP_PWRMGMT, &pmreg, 0)) {
		reg = pci_conf_read(pc, pa->pa_tag, pmreg + PCI_PMCSR);
		switch (reg & PCI_PMCSR_STATE_MASK) {
		case PCI_PMCSR_STATE_D1:
		case PCI_PMCSR_STATE_D2:
			printf(": waking up from power state D%d\n",
			    sc->sc_dev.dv_xname, reg & PCI_PMCSR_STATE_MASK);
			pci_conf_write(pc, pa->pa_tag, pmreg + PCI_PMCSR,
			    (reg & ~PCI_PMCSR_STATE_MASK) |
			    PCI_PMCSR_STATE_D0);
			break;
		case PCI_PMCSR_STATE_D3:
			/*
			 * IO and MEM are disabled. We can't enable
			 * the card because the BARs might be invalid.
			 */
			printf(
			    ": unable to wake up from power state D3, "
			    "reboot required.\n", sc->sc_dev.dv_xname);
			pci_conf_write(pc, pa->pa_tag, pmreg + PCI_PMCSR,
			    (reg & ~PCI_PMCSR_STATE_MASK) |
			    PCI_PMCSR_STATE_D0);
			return;
		}
	}

	/*
	 * Map the device.
	 */
	ioh_valid = (pci_mapreg_map(pa, EPIC_PCI_IOBA,
	    PCI_MAPREG_TYPE_IO, 0,
	    &iot, &ioh, NULL, NULL, 0) == 0);
	memh_valid = (pci_mapreg_map(pa, EPIC_PCI_MMBA,
	    PCI_MAPREG_TYPE_MEM|PCI_MAPREG_MEM_TYPE_32BIT, 0,
	    &memt, &memh, NULL, NULL, 0) == 0);

	if (memh_valid) {
		sc->sc_st = memt;
		sc->sc_sh = memh;
	} else if (ioh_valid) {
		sc->sc_st = iot;
		sc->sc_sh = ioh;
	} else {
		printf(": unable to map device registers\n",
		    sc->sc_dev.dv_xname);
		return;
	}

	sc->sc_dmat = pa->pa_dmat;

	/* Make sure bus mastering is enabled. */
	pci_conf_write(pc, pa->pa_tag, PCI_COMMAND_STATUS_REG,
	    pci_conf_read(pc, pa->pa_tag, PCI_COMMAND_STATUS_REG) |
	    PCI_COMMAND_MASTER_ENABLE);

	/*
	 * Map and establish our interrupt.
	 */
	if (pci_intr_map(pa, &ih)) {
		printf(": unable to map interrupt\n",
		    sc->sc_dev.dv_xname);
		return;
	}
	intrstr = pci_intr_string(pc, ih);
	psc->sc_ih = pci_intr_establish(pc, ih, IPL_NET, epic_intr, sc,
	    self->dv_xname);
	if (psc->sc_ih == NULL) {
		printf("%s: unable to establish interrupt",
		    sc->sc_dev.dv_xname);
		if (intrstr != NULL)
			printf(" at %s", intrstr);
		printf("\n");
		return;
	}

	esp = epic_pci_subsys_lookup(pa);
	if (esp)
		sc->sc_hwflags = esp->flags;

	/*
	 * Finish off the attach.
	 */
	epic_attach(sc, intrstr);
}
