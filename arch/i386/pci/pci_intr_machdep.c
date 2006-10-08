/*	$OpenBSD: pci_machdep.c,v 1.32 2006/06/02 19:39:49 pascoe Exp $	*/
/*	$NetBSD: pci_machdep.c,v 1.28 1997/06/06 23:29:17 thorpej Exp $	*/

/*-
 * Copyright (c) 1997 The NetBSD Foundation, Inc.
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
 * Copyright (c) 1996 Christopher G. Demetriou.  All rights reserved.
 * Copyright (c) 1994 Charles Hannum.  All rights reserved.
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
 *	This product includes software developed by Charles Hannum.
 * 4. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
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
 * Machine-specific functions for PCI autoconfiguration.
 *
 * On PCs, there are two methods of generating PCI configuration cycles.
 * We try to detect the appropriate mechanism for this machine and set
 * up a few function pointers to access the correct method directly.
 *
 * The configuration method can be hard-coded in the config file by
 * using `options PCI_CONF_MODE=N', where `N' is the configuration mode
 * as defined section 3.6.4.1, `Generating Configuration Cycles'.
 */

#include <sys/types.h>
#include <sys/param.h>
#include <sys/time.h>
#include <sys/systm.h>
#include <sys/errno.h>
#include <sys/device.h>

#include <uvm/uvm_extern.h>

#define _I386_BUS_DMA_PRIVATE
#include <machine/bus.h>
#include <machine/pio.h>
#include <machine/i8259.h>

#include "bios.h"
#if NBIOS > 0
#include <machine/biosvar.h>
extern bios_pciinfo_t *bios_pciinfo;
#endif

#include <dev/isa/isavar.h>
#include <dev/pci/pcivar.h>
#include <dev/pci/pcireg.h>
#include <dev/pci/pcidevs.h>

#include "ioapic.h"

#include <machine/i82093var.h>
#if NIOAPIC > 0
#include <machine/mpbiosvar.h>
#endif

#include "pcibios.h"
#include <i386/pci/pcibiosvar.h>


int
pci_intr_map(struct pci_attach_args *pa, pci_intr_handle_t *ihp)
{
#if NIOAPIC > 0
	struct mp_intr_map *mip;
	int bus, dev, func;
#endif

#if (NPCIBIOS > 0) || (NIOAPIC > 0)
	pci_chipset_tag_t pc = pa->pa_pc;
	pcitag_t intrtag = pa->pa_intrtag;
#endif
	int pin = pa->pa_intrpin;
	int line = pa->pa_intrline;

	if (pin == 0) {
		/* No IRQ used. */
		goto bad;
	}

	if (pin > 4) {
		printf("pci_intr_map: bad interrupt pin %d\n", pin);
		goto bad;
	}

	ihp->line = line;
	ihp->pin = pin;
#if NPCIBIOS > 0
	pci_intr_header_fixup(pc, intrtag, ihp);
	line = ihp->line & APIC_INT_LINE_MASK;
#endif

	/*
	 * Section 6.2.4, `Miscellaneous Functions', says that 255 means
	 * `unknown' or `no connection' on a PC.  We assume that a device with
	 * `no connection' either doesn't have an interrupt (in which case the
	 * pin number should be 0, and would have been noticed above), or
	 * wasn't configured by the BIOS (in which case we punt, since there's
	 * no real way we can know how the interrupt lines are mapped in the
	 * hardware).
	 *
	 * XXX
	 * Since IRQ 0 is only used by the clock, and we can't actually be sure
	 * that the BIOS did its job, we also recognize that as meaning that
	 * the BIOS has not configured the device.
	 */
	if (line == 0 || line == 255) {
		printf("pci_intr_map: no mapping for pin %c\n", '@' + pin);
		goto bad;
	} else {
		if (line >= ICU_LEN) {
			printf("pci_intr_map: bad interrupt line %d\n", line);
			goto bad;
		}
		if (line == 2) {
			printf("pci_intr_map: changed line 2 to line 9\n");
			line = 9;
		}
	}
#if NIOAPIC > 0
	pci_decompose_tag (pc, intrtag, &bus, &dev, &func);

	if (!(ihp->line & PCI_INT_VIA_ISA) && mp_busses != NULL) {
		/*
		 * Assumes 1:1 mapping between PCI bus numbers and
		 * the numbers given by the MP bios.
		 * XXX Is this a valid assumption?
		 */
		int mpspec_pin = (dev<<2)|(pin-1);

		for (mip = mp_busses[bus].mb_intrs; mip != NULL; mip=mip->next) {
			if (mip->bus_pin == mpspec_pin) {
				ihp->line = mip->ioapic_ih | line;
				return 0;
			}
		}
		if (mip == NULL && mp_isa_bus != -1) {
			for (mip = mp_busses[mp_isa_bus].mb_intrs; mip != NULL;
			    mip=mip->next) {
				if (mip->bus_pin == line) {
					ihp->line = mip->ioapic_ih | line;
					return 0;
				}
			}
		}
		if (mip == NULL && mp_eisa_bus != -1) {
			for (mip = mp_busses[mp_eisa_bus].mb_intrs;
			    mip != NULL; mip=mip->next) {
				if (mip->bus_pin == line) {
					ihp->line = mip->ioapic_ih | line;
					return 0;
				}
			}
		}
		if (mip == NULL) {
			printf("pci_intr_map: "
			    "bus %d dev %d func %d pin %d; line %d\n",
			    bus, dev, func, pin, line);
			printf("pci_intr_map: no MP mapping found\n");
		}
	}
#endif

	return 0;

bad:
	ihp->line = -1;
	return 1;
}

const char *
pci_intr_string(pci_chipset_tag_t pc, pci_intr_handle_t ih)
{
	static char irqstr[64];
	int line = ih.line & APIC_INT_LINE_MASK;

	if (line == 0 || line >= ICU_LEN || line == 2)
		panic("pci_intr_string: bogus handle 0x%x", line);

#if NIOAPIC > 0
	if (ih.line & APIC_INT_VIA_APIC) {
		snprintf(irqstr, sizeof irqstr, "apic %d int %d (irq %d)",
		     APIC_IRQ_APIC(ih.line), APIC_IRQ_PIN(ih.line), line);
		return (irqstr);
	}
#endif

	snprintf(irqstr, sizeof irqstr, "irq %d", line);
	return (irqstr);
}

void *
pci_intr_establish(pci_chipset_tag_t pc, pci_intr_handle_t ih,
		int level, int (*func)(void *), void *arg, char *what)
{
	void *ret;
	int l = ih.line & APIC_INT_LINE_MASK;

#if NIOAPIC > 0
	if (l != -1 && ih.line & APIC_INT_VIA_APIC)
		return (apic_intr_establish(ih.line, IST_LEVEL, level, func, 
		    arg, what));
#endif
	if (l == 0 || l >= ICU_LEN || l == 2)
		panic("pci_intr_establish: bogus handle 0x%x", l);

	ret = isa_intr_establish(NULL, l, IST_LEVEL, level, func, arg, what);
#if NPCIBIOS > 0
	if (ret)
		pci_intr_route_link(pc, &ih);
#endif
	return (ret);
}

void
pci_intr_disestablish(pci_chipset_tag_t pc, void *cookie)
{
	/* XXX oh, unroute the pci int link? */
	isa_intr_disestablish(NULL, cookie);
}
