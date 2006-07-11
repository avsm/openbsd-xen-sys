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
#include "xennet.h"
#include "xbc.h"
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
#include <machine/granttables.h>
#if NPCI > 0
#include <dev/pci/pcivar.h>
#endif

#if NXENNET > 0
#include <sys/socket.h>
#include <net/if.h>
#include <netinet/in.h>
#include <netinet/if_ether.h>
#include <machine/if_xennetvar.h>
#endif

#if NXBC > 0
#include <scsi/scsi_all.h>
#include <scsi/scsiconf.h>
#include <machine/xbcvar.h>
#endif

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
#if NXENCONS > 0 || NXENNET > 0 || NXBC > 0 || NNPX > 0
static int hypervisor_print(void *, const char *);
#endif

union hypervisor_attach_cookie {
	const char *hac_device;		/* first elem of all */
#if NXENCONS > 0
	struct xencons_attach_args hac_xencons;
#endif
#if NXENNET > 0
	struct xennet_attach_args hac_xennet;
#endif
#if NXBC > 0
	struct xbc_attach_args hac_xbc;
#endif
#if NNPX > 0
	struct xen_npx_attach_args hac_xennpx;
#endif
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

#if 0
/* shutdown/reboot message stuff */
static void hypervisor_shutdown_handler(ctrl_msg_t *, unsigned long);
static struct sysmon_pswitch hysw_shutdown = {
	.smpsw_type = PSWITCH_TYPE_POWER,
	.smpsw_name = "hypervisor",
};
static struct sysmon_pswitch hysw_reboot = {
	.smpsw_type = PSWITCH_TYPE_RESET,
	.smpsw_name = "hypervisor",
};
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
	//struct pcibus_attach_args pba;
#if defined(DOM0OPS) && NISA > 0
	struct isabus_attach_args iba;
#endif
#endif
#if NXENCONS > 0 || NXENNET > 0 || NXBC > 0 || NNPX > 0 
	union hypervisor_attach_cookie hac;
#endif
	printf("\n");

	init_events();
	xengnt_init();

#if NXENCONS > 0
	hac.hac_xencons.xa_device = "xencons";
	config_found(self, &hac.hac_xencons, hypervisor_print);
#endif
#if NXENNET > 0
	hac.hac_xennet.xa_device = "xennet";
	xennet_scan(self, &hac.hac_xennet, hypervisor_print);
#endif
#if NXBC > 0
	hac.hac_xbc.xa_device = "xbc";
	xbc_scan(self, &hac.hac_xbc, hypervisor_print);
#endif
#if NNPX > 0
	hac.hac_xennpx.xa_device = "npx";
	config_found(self, &hac.hac_xennpx, hypervisor_print);
#endif
#if NPCI > 0
	/* XXX: todo PCI for Xen3 */
#if defined(DOM0OPS) && NISA > 0
	if (isa_has_been_seen == 0) {
		iba.iba_busname = "isa";
		iba.iba_iot = I386_BUS_SPACE_IO;
		iba.iba_memt = I386_BUS_SPACE_MEM;
#if NISADMA > 0
		iba.iba_dmat = &isa_bus_dma_tag;
#endif
		iba.iba_ic = NULL; /* No isa DMA yet */
		config_found(self, &iba, hypervisor_print);
	}
#endif	/* DOM0OPS && NISA */
#endif	/* NPCI */
#ifdef DOM0OPS
	if (xen_start_info.flags & SIF_PRIVILEGED) {
#if 0
		xenkernfs_init();
		xenprivcmd_init();
		xen_shm_init();
		xbdback_init();
		xennetback_init();
#endif	/* 0 */
	}
#endif	/* DOM0OPS */
#if 0
	if (sysmon_pswitch_register(&hysw_reboot) != 0 ||
	    sysmon_pswitch_register(&hysw_shutdown) != 0)
		printf("%s: unable to register with sysmon\n",
		    self->dv_xname);
	else
		ctrl_if_register_receiver(CMSG_SHUTDOWN,
		    hypervisor_shutdown_handler, CALLBACK_IN_BLOCKING_CONTEXT);
#endif	/* 0 */
}

#if NXENCONS > 0 || NXENNET > 0 || NXBC > 0 || NNPX > 0
static int
hypervisor_print(void *aux, const char *parent)
{
	union hypervisor_attach_cookie *hac = aux;

	if (parent)
		printf("%s at %s", hac->hac_device, parent);
	return (UNCONF);
}
#endif


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

#if 0
/* handler for the shutdown messages */
static void
hypervisor_shutdown_handler(ctrl_msg_t *msg, unsigned long id)
{
	switch(msg->subtype) {
	case CMSG_SHUTDOWN_POWEROFF:
		sysmon_pswitch_event(&hysw_shutdown, PSWITCH_EVENT_PRESSED);
		break;
	case CMSG_SHUTDOWN_REBOOT:
		sysmon_pswitch_event(&hysw_reboot, PSWITCH_EVENT_PRESSED);
		break;
	default:
		printf("shutdown_handler: unknwon message %d\n",
		    msg->type);
	}
}
#endif	/* 0 */
