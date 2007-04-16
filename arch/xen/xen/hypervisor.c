/* $NetBSD: hypervisor.c,v 1.14 2005/04/18 21:33:21 bouyer Exp $ */

/*
 * Copyright (c) 2005 Manuel Bouyer.
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
 *      This product includes software developed by Manuel Bouyer.
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
 *
 */

/*
 *
 * Copyright (c) 2004 Christian Limpach.
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
 * 3. All advertising materials mentioning features or use of this software
 *    must display the following acknowledgement:
 *      This product includes software developed by Christian Limpach.
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


#include <sys/cdefs.h>

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/device.h>
#include <sys/malloc.h>

#include "xencons.h"
#include "npx.h"
#include "isa.h"
#include "pci.h"

#include <machine/xen.h>
#include <machine/hypervisor.h>
#include <machine/evtchn.h>

#include <sys/dirent.h>
#include <sys/stat.h>
#include <sys/tree.h>
#include <sys/vnode.h>
#include <miscfs/specfs/specdev.h>
#if NISA > 0
#include <dev/isa/isavar.h>
#endif
#if NPCI > 0
#include <dev/pci/pcivar.h>
#if NPCIBIOS > 0
#include <arch/i386/pci/pcibiosvar.h>
#endif
#endif

#include <machine/granttables.h>
#include <machine/xenbus.h>

#if NPCI > 0
extern u_int32_t pci_bus_attached[];
#endif

#if !defined(DOM0OPS) && NXENCONS == 0
#error DomU kernel will not boot without xencons 
#endif
/* #define DEBUG */

int	hypervisor_match(struct device *, void *, void *);
void	hypervisor_attach(struct device *, struct device *, void *);

struct cfattach hypervisor_ca = {
	sizeof(struct device), hypervisor_match, hypervisor_attach
};

struct cfdriver hypervisor_cd = {
	NULL, "hypervisor", 0
};
static int hypervisor_print(void *, const char *);

union hypervisor_attach_cookie {
	const char *hac_device;		/* first elem of all */

	struct xenbus_attach_args hac_xenbus;
#if NXENCONS > 0
	struct xencons_attach_args hac_xencons;
#endif
#if NNPX > 0
	struct xen_npx_attach_args hac_xennpx;
#endif
#if NPCI > 0
	struct pcibus_attach_args hac_pba;
#if defined(DOM0OPS) && NISA > 0
	struct isabus_attach_args hac_iba;
#endif
#endif /* NPCI */
};

/*
 * This is set when the ISA bus is attached.  If it's not set by the
 * time it's checked below, then mainbus attempts to attach an ISA.
 */
#ifdef DOM0OPS
int     isa_has_been_seen;
#if 0
#if NISA > 0
struct  x86_isa_chipset x86_isa_chipset;
#endif
#endif	/* 0 */
#endif


/*
 * Probe for the hypervisor; always succeeds.
 */
int
hypervisor_match(struct device *parent, void *match, void *aux)
{
	struct hypervisor_attach_args *haa = aux;

	if (strcmp(haa->haa_busname, "hypervisor") == 0)
		return 1;
	return 0;
}

/*
 * Attach the hypervisor.
 */
void
hypervisor_attach(struct device *parent, struct device *self, void *aux)
{
#if NPCI > 0
#if NACPI > 0
	int acpi_present = 0;
#endif
#ifdef NPCIBIOS
	int pci_maxbus = 0;
#endif
#endif
	union hypervisor_attach_cookie hac;

	printf("\n");

	init_events();
	xengnt_init();

	hac.hac_xenbus.xa_device = "xenbus";
	config_found(self, &hac.hac_xenbus, hypervisor_print);

#if NXENCONS > 0
	hac.hac_xencons.xa_device = "xencons";
	config_found(self, &hac.hac_xencons, hypervisor_print);
#endif
#if NNPX > 0
	hac.hac_xennpx.xa_device = "npx";
	config_found(self, &hac.hac_xennpx, hypervisor_print);
#endif
#if NPCI > 0
	pci_mode = pci_mode_detect();
#ifdef NPCIBIOS
	pci_maxbus = pci_bus_fixup(NULL, 0);
	printf("PCI bus max, after pci_bus_fixup: %i\n", pci_maxbus);
	pciaddr.extent_port = NULL;
	pciaddr.extent_mem = NULL;
	pci_addr_fixup(NULL, pci_maxbus);
#endif
#endif /* NPCI */


#ifdef DOM0OPS
	if (xen_start_info.flags & SIF_PRIVILEGED) {
#if 0
		xenkernfs_init();
		xenprivcmd_init();
#endif	/* 0 */
		xen_shm_init();
	}
#endif	/* DOM0OPS */
}

static int
hypervisor_print(void *aux, const char *parent)
{
	union hypervisor_attach_cookie *hac = aux;

	if (parent)
		printf("%s at %s", hac->hac_device, parent);
	return (UNCONF);
}


#ifdef DOM0OPS
#if 0
#define DIR_MODE	(S_IRUSR|S_IXUSR|S_IRGRP|S_IXGRP|S_IROTH|S_IXOTH)

struct kern_target *kernxen_pkt;

void
xenkernfs_init(void)
{
	kernfs_entry_t *dkt;

	KERNFS_ALLOCENTRY(dkt, M_TEMP, M_WAITOK);
	KERNFS_INITENTRY(dkt, DT_DIR, "xen", NULL, KFSsubdir, VDIR, DIR_MODE);
	kernfs_addentry(NULL, dkt);
	kernxen_pkt = KERNFS_ENTOPARENTDIR(dkt);
}
#endif	/* 0 */
#endif
