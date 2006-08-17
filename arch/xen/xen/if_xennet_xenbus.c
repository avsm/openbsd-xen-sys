/*	$NetBSD: if_xennet.c,v 1.37 2005/10/02 21:39:41 bouyer Exp $	*/

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


#include <sys/param.h>
#include <sys/systm.h>
#include <sys/mbuf.h>
#include <sys/syslog.h>
#include <sys/mount.h>
#include <sys/socket.h>
#include <sys/socketvar.h>
#include <sys/device.h>
#include <sys/ioctl.h>
#include <sys/errno.h>
#include <sys/kernel.h>
#if NRND > 0
#include <sys/rnd.h>
#endif

#include <net/if.h>
#include <net/if_types.h>
#include <net/if_dl.h>
#include <net/if_arp.h>

#include <machine/intr.h>	/* for softintr_establish */

#include <net/if_media.h>

#ifdef INET
#include <netinet/in.h>
#include <netinet/in_systm.h>
#include <netinet/in_var.h>
#include <netinet/ip.h>
#endif

#include <netinet/if_ether.h>

#if defined(NFS_BOOT_BOOTSTATIC)
#include <nfs/rpcv2.h>

#include <nfs/nfsproto.h>
#include <nfs/nfs.h>
#include <nfs/nfsmount.h>
#include <nfs/nfsdiskless.h>
#endif /* defined(NFS_BOOT_BOOTSTATIC) */

#include "bpfilter.h"
#if NBPFILTER > 0
#include <net/bpf.h>
#endif

#include <uvm/uvm_extern.h>
#include <uvm/uvm_page.h>

#include <machine/xen.h>
#include <machine/hypervisor.h>
#include <machine/evtchn.h>

#include <machine/xen-public/io/ring.h>

#include <machine/granttables.h>
#include <machine/xenbus.h>
#include "../xenbus/strtoul.h"

#ifdef DEBUG
#define XENNET_DEBUG
#endif
#if defined(XENNET_DEBUG) && !defined(DEBUG)
#define DEBUG
#endif
/* #define XENNET_DEBUG_DUMP */

#ifdef XENNET_DEBUG
#define XEDB_FOLLOW	0x01
#define XEDB_INIT	0x02
#define XEDB_RXEVENT	0x04
#define XEDB_MBUF	0x08
#define XEDB_MEM	0x10
#define XEDB_TXEVENT	0x20
#define XEDB_EVENT	XEDB_RXEVENT | XEDB_TXEVENT
int xennet_debug = 0x0;
#define DPRINTF(x) if (xennet_debug) printf x;
#define DPRINTFN(n,x) if (xennet_debug & (n)) printf x;
#else
#define DPRINTF(x)
#define DPRINTFN(n,x)
#endif
#define PRINTF(x) printf x;

#define GRANT_INVALID_REF -1 /* entry is free */
#define GRANT_STACK_REF   -2 /* entry owned by the network stack */

#define NET_TX_RING_SIZE __RING_SIZE((netif_tx_sring_t *)0, PAGE_SIZE)
#define NET_RX_RING_SIZE __RING_SIZE((netif_rx_sring_t *)0, PAGE_SIZE)


#ifndef __HAVE_GENERIC_SOFT_INTERRUPTS
#error This driver assumes, generic softinterrupts are implemented
#endif


struct xennet_txreq {
	SLIST_ENTRY(xennet_txreq) txreq_next;
	uint16_t txreq_id; /* ID passed to backed */
	grant_ref_t txreq_gntref; /* grant ref of this request */
	struct mbuf *txreq_m; /* mbuf being transmitted */
};

struct xennet_rxreq {
	SLIST_ENTRY(xennet_rxreq) rxreq_next;
	uint16_t rxreq_id; /* ID passed to backed */
	grant_ref_t rxreq_gntref; /* grant ref of this request */
	/* va/pa for this receive buf. ma will be provided by backend */
	paddr_t rxreq_pa;
	vaddr_t rxreq_va;
	struct xennet_xenbus_softc *rxreq_sc; /* pointer to our interface */
};

struct xennet_xenbus_softc {
	struct device sc_dev;
	struct arpcom sc_arpcom;
	struct xenbus_device *sc_xbusd;

	int sc_ifno;
	struct ifmedia sc_media;

	netif_tx_front_ring_t sc_tx_ring;
	netif_rx_front_ring_t sc_rx_ring;

	unsigned int sc_evtchn;
	void *sc_softintr;

	grant_ref_t sc_tx_ring_gntref;
	grant_ref_t sc_rx_ring_gntref;

	struct xennet_txreq sc_txreqs[NET_TX_RING_SIZE];
	struct xennet_rxreq sc_rxreqs[NET_RX_RING_SIZE];
	SLIST_HEAD(,xennet_txreq) sc_txreq_head; /* list of free TX requests */
	SLIST_HEAD(,xennet_rxreq) sc_rxreq_head; /* list of free RX requests */
	int sc_free_rxreql; /* number of free receive request struct */

	int sc_backend_status; /* our status with backend */
#define BEST_CLOSED		0
#define BEST_DISCONNECTED	1
#define BEST_CONNECTED		2
#if NRND > 0
	rndsource_element_t	sc_rnd_source;
#endif
};


/* too big to be on stack */
static multicall_entry_t rx_mcl[NET_RX_RING_SIZE+1];
static paddr_t xennet_pages[NET_RX_RING_SIZE];

int  xennet_xenbus_match(struct device *, void *, void *);
void xennet_xenbus_attach(struct device *, struct device *, void *);
int  xennet_xenbus_detach(struct device *, int);
void xennet_backend_changed(void *, XenbusState);

int  xennet_xenbus_resume(void *);
void xennet_alloc_rx_buffer(struct xennet_xenbus_softc *);
void xennet_free_rx_buffer(struct xennet_xenbus_softc *);
void xennet_tx_complete(struct xennet_xenbus_softc *);
void xennet_rx_mbuf_free(caddr_t, u_int, void *);
int  xennet_handler(void *);
#ifdef XENNET_DEBUG_DUMP
static void xennet_hex_dump(unsigned char *, size_t, char *, int);
#endif

int  xennet_init(struct ifnet *);
void xennet_stop(struct ifnet *, int);
void xennet_reset(struct xennet_xenbus_softc *);
void xennet_softstart(void *);
int xennet_mediachange (struct ifnet *);
void xennet_mediastatus(struct ifnet *, struct ifmediareq *);
void xennet_start(struct ifnet *);
int  xennet_ioctl(struct ifnet *, u_long, caddr_t);
void xennet_watchdog(struct ifnet *);



struct cfattach xennet_ca = {
	sizeof(struct xennet_xenbus_softc),
	xennet_xenbus_match, xennet_xenbus_attach, xennet_xenbus_detach, NULL
};

struct cfdriver xennet_cd = {
	NULL, "xennet", DV_IFNET
};



#define RX_MAX_ENTRIES (NETIF_RX_RING_SIZE - 2)
#define RX_ENTRIES 128


static int xennet_media[] = {
	IFM_ETHER|IFM_AUTO,
};
static int nxennet_media = (sizeof(xennet_media)/sizeof(xennet_media[0]));

int in_autoconf = 0;


int
xennet_xenbus_match(struct device *parent, void *match, void *aux)
{
	struct xenbusdev_attach_args *xa = aux;

	if (strcmp(xa->xa_type, "vif") != 0)
		return 0;
	
	return 1;
}

void
xennet_xenbus_attach(struct device *parent, struct device *self, void *aux)
{
	struct xennet_xenbus_softc *sc = (struct xennet_xenbus_softc *)self;
	struct xenbusdev_attach_args *xa = (struct xenbusdev_attach_args *)aux;	
	struct ifnet *ifp = &sc->sc_arpcom.ac_if;
	int err, idx;
	RING_IDX i;
	char *val, *e, *p;
	int s;
	extern int ifqmaxlen; /* XXX */
#ifdef XENNET_DEBUG
	char **dir;
	int dir_n = 0;
	char id_str[20];
#endif


	printf(": Xen Virtual Network Interface\n");
#ifdef XENNET_DEBUG
	printf("path: %s\n", xa->xa_xbusd->xbusd_path);
	snprintf(id_str, sizeof(id_str), "%d", xa->xa_id);
	err = xenbus_directory(NULL, "device/vif", id_str, &dir_n, &dir);
	if (err) {
		printf("%s: xenbus_directory err %d\n",
		    sc->sc_dev.dv_xname, err);
	} else {
		printf("%s/\n", xa->xa_xbusd->xbusd_path);
		for (i = 0; i < dir_n; i++) {
			printf("\t/%s", dir[i]);
			err = xenbus_read(NULL, xa->xa_xbusd->xbusd_path, dir[i],
			    NULL, &val);
			if (err) {
				printf("%s: xenbus_read err %d\n",
				sc->sc_dev.dv_xname, err);
			} else {
				printf(" = %s\n", val);
				free(val, M_DEVBUF);
			}
		}
	}
#endif /* XENNET_DEBUG */
	sc->sc_xbusd = xa->xa_xbusd;
	sc->sc_xbusd->xbusd_otherend_changed = xennet_backend_changed;

	/* initialize free RX and RX request lists */
	SLIST_INIT(&sc->sc_txreq_head);
	for (i = 0; i < NET_TX_RING_SIZE; i++) {
		sc->sc_txreqs[i].txreq_id = i;
		SLIST_INSERT_HEAD(&sc->sc_txreq_head, &sc->sc_txreqs[i],
		    txreq_next);
	}
	SLIST_INIT(&sc->sc_rxreq_head);
	s = splvm();
	for (i = 0; i < NET_RX_RING_SIZE; i++) {
                struct xennet_rxreq *rxreq = &sc->sc_rxreqs[i];
                rxreq->rxreq_id = i;
                rxreq->rxreq_sc = sc;
                rxreq->rxreq_va = uvm_km_zalloc(kernel_map,
                    PAGE_SIZE);
                if (rxreq->rxreq_va == 0)
                        break;
                if (!pmap_extract(pmap_kernel(), rxreq->rxreq_va,
                    &rxreq->rxreq_pa))
                        panic("xennet: no pa for mapped va ?");
                rxreq->rxreq_gntref = GRANT_INVALID_REF;
                SLIST_INSERT_HEAD(&sc->sc_rxreq_head, rxreq, rxreq_next);
        }
        splx(s);
        sc->sc_free_rxreql = i;
	if (sc->sc_free_rxreql == 0) {
		printf("%s: failed to allocate rx memory\n",
		    sc->sc_dev.dv_xname);
		return;
	}

	/* read mac address */
	err = xenbus_read(NULL, xa->xa_xbusd->xbusd_path, "mac", NULL, &val);
	if (err) {
		printf("%s: can't read mac address, err %d\n",
		    sc->sc_dev.dv_xname, err);
		return;
	}
	/* read mac address */
	for (i = 0, p = val; i < 6; i++) {
		sc->sc_arpcom.ac_enaddr[i] = strtoul(p, &e, 16);
		if ((e[0] == '\0' && i != 5) && e[0] != ':') {
			printf("%s: %s is not a valid mac address\n",
			    sc->sc_dev.dv_xname, val);
			free(val, M_DEVBUF);
			return;
		}
		p = &e[1];
	}
	free(val, M_DEVBUF);
	printf("%s: MAC address %s\n", sc->sc_dev.dv_xname,
	    ether_sprintf(sc->sc_arpcom.ac_enaddr));

	/* Initialize ifnet structure. */
	memset(ifp, 0, sizeof(struct ifnet));
	memcpy(ifp->if_xname, sc->sc_dev.dv_xname, IFNAMSIZ);
	ifp->if_softc = sc;
	ifp->if_start = xennet_start;
	ifp->if_ioctl = xennet_ioctl;
	ifp->if_watchdog = xennet_watchdog;
	ifp->if_init = xennet_init;
#if 0
	ifp->if_reset = xennet_reset;
#endif
	ifp->if_flags =
	    IFF_BROADCAST|IFF_SIMPLEX|IFF_NOTRAILERS|IFF_MULTICAST;
	ifp->if_timer = 0;
	ifp->if_snd.ifq_maxlen = max(ifqmaxlen, NET_TX_RING_SIZE * 2);
	IFQ_SET_READY(&ifp->if_snd);

	ifmedia_init(&sc->sc_media, 0, xennet_mediachange,
	    xennet_mediastatus);
	for (idx = 0; idx < nxennet_media; idx++)
		ifmedia_add(&sc->sc_media, xennet_media[idx], 0, NULL);
	ifmedia_set(&sc->sc_media, xennet_media[0]);

	if_attach(ifp);
	ether_ifattach(ifp);

	sc->sc_softintr = softintr_establish(IPL_SOFTNET,
	    xennet_softstart, sc);
	if (sc->sc_softintr == NULL)
		panic(" xennet: can't establish soft interrupt");


	/* initialise shared structures and tell backend that we are ready */
        xennet_xenbus_resume(sc);
}


int
xennet_xenbus_detach(struct device *self, int flags)
{
	struct xennet_xenbus_softc *sc = (void *)self;
	struct ifnet *ifp = &sc->sc_arpcom.ac_if;
	int s0, s1;
	RING_IDX i;

	DPRINTF(("%s: xennet_xenbus_detach\n", sc->sc_dev.dv_xname));
	s0 = splnet();
	xennet_stop(ifp, 1);
	/* wait for pending TX to complete, and collect pending RX packets */
	xennet_handler(sc);
	while (sc->sc_tx_ring.sring->rsp_prod != sc->sc_tx_ring.rsp_cons) {
		tsleep(xennet_xenbus_detach, PRIBIO, "xnet_detach", hz/2);
		xennet_handler(sc);
	}
	xennet_free_rx_buffer(sc);

	s1 = splvm();
	for (i = 0; i < NET_RX_RING_SIZE; i++) {
		struct xennet_rxreq *rxreq = &sc->sc_rxreqs[i];
		uvm_km_free(kernel_map, rxreq->rxreq_va, PAGE_SIZE);
	}
	splx(s1);

	ether_ifdetach(ifp);
	if_detach(ifp);
	while (xengnt_status(sc->sc_tx_ring_gntref)) {
		tsleep(xennet_xenbus_detach, PRIBIO, "xnet_txref", hz/2);
	}
	xengnt_revoke_access(sc->sc_tx_ring_gntref);
	uvm_km_free(kernel_map, (vaddr_t)sc->sc_tx_ring.sring, PAGE_SIZE);
	while (xengnt_status(sc->sc_rx_ring_gntref)) {
		tsleep(xennet_xenbus_detach, PRIBIO, "xnet_rxref", hz/2);
	}
	xengnt_revoke_access(sc->sc_rx_ring_gntref);
	uvm_km_free(kernel_map, (vaddr_t)sc->sc_rx_ring.sring, PAGE_SIZE);

	softintr_disestablish(sc->sc_softintr);
	event_remove_handler(sc->sc_evtchn, &xennet_handler, sc);
	splx(s0);
	DPRINTF(("%s: xennet_xenbus_detach done\n", sc->sc_dev.dv_xname));
	return 0;
}


int
xennet_xenbus_resume(void *p)
{
	struct xennet_xenbus_softc *sc = p;
	struct xenbus_transaction *xbt;
	int error;
	netif_tx_sring_t *tx_ring;
	netif_rx_sring_t *rx_ring;
	paddr_t ma;
	const char *errmsg;

	sc->sc_tx_ring_gntref = GRANT_INVALID_REF;
	sc->sc_rx_ring_gntref = GRANT_INVALID_REF;


	/* setup device: alloc event channel and shared rings */
	tx_ring = (void *)uvm_km_zalloc(kernel_map, PAGE_SIZE);
	rx_ring = (void *)uvm_km_zalloc(kernel_map, PAGE_SIZE);
	if (tx_ring == NULL || rx_ring == NULL)
		panic("xennet_xenbus_resume: can't alloc rings");

	SHARED_RING_INIT(tx_ring);
	FRONT_RING_INIT(&sc->sc_tx_ring, tx_ring, PAGE_SIZE);
	SHARED_RING_INIT(rx_ring);
	FRONT_RING_INIT(&sc->sc_rx_ring, rx_ring, PAGE_SIZE);

	(void)pmap_extract_ma(pmap_kernel(), (vaddr_t)tx_ring, &ma);
	error = xenbus_grant_ring(sc->sc_xbusd, ma, &sc->sc_tx_ring_gntref);
	if (error)
		return error;
	(void)pmap_extract_ma(pmap_kernel(), (vaddr_t)rx_ring, &ma);
	error = xenbus_grant_ring(sc->sc_xbusd, ma, &sc->sc_rx_ring_gntref);
	if (error)
		return error;
	error = xenbus_alloc_evtchn(sc->sc_xbusd, &sc->sc_evtchn);
	if (error)
		return error;
	printf("%s: using event channel %d\n",
	    sc->sc_dev.dv_xname, sc->sc_evtchn);
	event_set_handler(sc->sc_evtchn, &xennet_handler, sc,
	    IPL_NET, sc->sc_dev.dv_xname);

again:
	xbt = xenbus_transaction_start();
	if (xbt == NULL)
		return ENOMEM;
	error = xenbus_printf(xbt, sc->sc_xbusd->xbusd_path,
	    "tx-ring-ref","%u", sc->sc_tx_ring_gntref);
	if (error) {
		errmsg = "writing tx ring-ref";
		goto abort_transaction;
	}
	error = xenbus_printf(xbt, sc->sc_xbusd->xbusd_path,
	    "rx-ring-ref","%u", sc->sc_rx_ring_gntref);
	if (error) {
		errmsg = "writing rx ring-ref";
		goto abort_transaction;
	}
	error = xenbus_printf(xbt, sc->sc_xbusd->xbusd_path,
	    "event-channel", "%u", sc->sc_evtchn);
	if (error) {
		errmsg = "writing event channel";
		goto abort_transaction;
	}
	error = xenbus_printf(xbt, sc->sc_xbusd->xbusd_path,
	    "state", "%d", XenbusStateConnected);
	if (error) {
		errmsg = "writing frontend XenbusStateConnected";
		goto abort_transaction;
	}
	error = xenbus_transaction_end(xbt, 0);
	if (error == EAGAIN)
		goto again;
	if (error) {
		xenbus_dev_fatal(sc->sc_xbusd, error, "completing transaction");
		return -1;
	}
	xennet_alloc_rx_buffer(sc);
	sc->sc_backend_status = BEST_CONNECTED;
	return 0;

abort_transaction:
	xenbus_transaction_end(xbt, 1);
	xenbus_dev_fatal(sc->sc_xbusd, error, "%s", errmsg);
	return error;
}

void xennet_backend_changed(void *arg, XenbusState new_state)
{
        struct xennet_xenbus_softc *sc = arg;
        DPRINTF(("%s: new backend state %d\n", sc->sc_dev.dv_xname, new_state));

        switch (new_state) {
        case XenbusStateInitialising:
        case XenbusStateInitWait:
        case XenbusStateInitialised:
                break;
        case XenbusStateClosing:
                sc->sc_backend_status = BEST_CLOSED;
                xenbus_switch_state(sc->sc_xbusd, NULL, XenbusStateClosed);
                break;
        case XenbusStateConnected:
                break;
        case XenbusStateUnknown:
        default:
                panic("bad backend state %d", new_state);
        }
}

void
xennet_alloc_rx_buffer(struct xennet_xenbus_softc *sc)
{
	RING_IDX req_prod = sc->sc_rx_ring.req_prod_pvt;
	RING_IDX i;
	struct xennet_rxreq *req;
	struct xen_memory_reservation reservation;
	int s1, s2;
	paddr_t pfn;

	s1 = splnet();
	for (i = 0; sc->sc_free_rxreql != 0; i++) {
		req  = SLIST_FIRST(&sc->sc_rxreq_head);
		KASSERT(req != NULL);
		KASSERT(req == &sc->sc_rxreqs[req->rxreq_id]);
		RING_GET_REQUEST(&sc->sc_rx_ring, req_prod + i)->id =
		    req->rxreq_id;
		if (xengnt_grant_transfer(sc->sc_xbusd->xbusd_otherend_id,
		    &req->rxreq_gntref) != 0) {
			break;
		}
		RING_GET_REQUEST(&sc->sc_rx_ring, req_prod + i)->gref =
		    req->rxreq_gntref;

		SLIST_REMOVE_HEAD(&sc->sc_rxreq_head, rxreq_next);
		sc->sc_free_rxreql--;

		/* unmap the page */
		MULTI_update_va_mapping(&rx_mcl[i], req->rxreq_va, 0, 0);
		/*
		 * Remove this page from pseudo phys map before
		 * passing back to Xen.
		 */
		pfn = (req->rxreq_pa - XPMAP_OFFSET) >> PAGE_SHIFT;
		xennet_pages[i] = xpmap_phys_to_machine_mapping[pfn];
		xpmap_phys_to_machine_mapping[pfn] = INVALID_P2M_ENTRY;
	}
	if (i == 0) {
		splx(s1);
		return;
	}
	/* also make sure to flush all TLB entries */
	rx_mcl[i-1].args[MULTI_UVMFLAGS_INDEX] = UVMF_TLB_FLUSH|UVMF_ALL;
	/*
	 * We may have allocated buffers which have entries
	 * outstanding in the page update queue -- make sure we flush
	 * those first!
	 */
	s2 = splvm();
	xpq_flush_queue();
	splx(s2);
	/* now decrease reservation */
	reservation.extent_start = xennet_pages;
	reservation.nr_extents = i;
	reservation.extent_order = 0;
	reservation.address_bits = 0;
	reservation.domid = DOMID_SELF;
	rx_mcl[i].op = __HYPERVISOR_memory_op;
	rx_mcl[i].args[0] = XENMEM_decrease_reservation;
	rx_mcl[i].args[1] = (unsigned long)&reservation;
	HYPERVISOR_multicall(rx_mcl, i+1);
	if (__predict_false(rx_mcl[i].result != i)) {
		panic("xennet_alloc_rx_buffer: XENMEM_decrease_reservation");
	}
	sc->sc_rx_ring.req_prod_pvt = req_prod + i;
	RING_PUSH_REQUESTS(&sc->sc_rx_ring);

	splx(s1);
	return;
}


void
xennet_free_rx_buffer(struct xennet_xenbus_softc *sc)
{
	paddr_t ma, pa;
	vaddr_t va;
	RING_IDX i;
	mmu_update_t mmu[1];
	multicall_entry_t mcl[2];

	int s = splbio();

	DPRINTF(("%s: xennet_free_rx_buffer\n", sc->sc_dev.dv_xname));
	/* get back memory from RX ring */
	for (i = 0; i < NET_RX_RING_SIZE; i++) {
		struct xennet_rxreq *rxreq = &sc->sc_rxreqs[i];

		/*
		 * if the buffer is in transit in the network stack, wait for
		 * the network stack to free it.
		 */
		while ((volatile grant_ref_t)rxreq->rxreq_gntref ==
		    GRANT_STACK_REF)
			tsleep(xennet_xenbus_detach, PRIBIO, "xnet_free", hz/2);

		if (rxreq->rxreq_gntref != GRANT_INVALID_REF) {
			/*
			 * this req is still granted. Get back the page or
			 * allocate a new one, and remap it.
			 */
			SLIST_INSERT_HEAD(&sc->sc_rxreq_head, rxreq,
			    rxreq_next);
			sc->sc_free_rxreql++;
			ma = xengnt_revoke_transfer(rxreq->rxreq_gntref);
			rxreq->rxreq_gntref = GRANT_INVALID_REF;
			if (ma == 0) {
				struct xen_memory_reservation xenres;
				/*
				 * transfer not complete, we lost the page.
				 * Get one from hypervisor
				 */
				xenres.extent_start = &ma;
				xenres.nr_extents = 1;
				xenres.extent_order = 0;
				xenres.address_bits = 31;
				xenres.domid = DOMID_SELF;
				if (HYPERVISOR_memory_op(
				    XENMEM_increase_reservation, &xenres) < 0) {
					panic("xennet_free_rx_buffer: "
					    "can't get memory back");
				}
				KASSERT(ma != 0);
			}
			pa = rxreq->rxreq_pa;
			va = rxreq->rxreq_va;
			/* remap the page */
			mmu[0].ptr = (ma << PAGE_SHIFT) | MMU_MACHPHYS_UPDATE;
			mmu[0].val = ((pa - XPMAP_OFFSET) >> PAGE_SHIFT);
			MULTI_update_va_mapping(&mcl[0], va,
			    (ma << PAGE_SHIFT) | PG_V | PG_KW,
			    UVMF_TLB_FLUSH|UVMF_ALL);
			xpmap_phys_to_machine_mapping[
			    (pa - XPMAP_OFFSET) >> PAGE_SHIFT] = ma;
			mcl[1].op = __HYPERVISOR_mmu_update;
			mcl[1].args[0] = (unsigned long)mmu;
			mcl[1].args[1] = 1;
			mcl[1].args[2] = 0;
			mcl[1].args[3] = DOMID_SELF;
			HYPERVISOR_multicall(mcl, 2);
		}

	}
	splx(s);
	DPRINTF(("%s: xennet_free_rx_buffer done\n", sc->sc_dev.dv_xname));
}


void
xennet_rx_mbuf_free(caddr_t buf, u_int size, void *arg)
{
	struct xennet_rxreq *req = arg;
	struct xennet_xenbus_softc *sc = req->rxreq_sc;

	int s = splnet();

	SLIST_INSERT_HEAD(&sc->sc_rxreq_head, req, rxreq_next);
	sc->sc_free_rxreql++;

	req->rxreq_gntref = GRANT_INVALID_REF;
	if (sc->sc_free_rxreql >= NET_RX_RING_SIZE / 2 &&
	    __predict_true(sc->sc_backend_status == BEST_CONNECTED)) {
		xennet_alloc_rx_buffer(sc);
	}

	splx(s);
}


void
xennet_tx_complete(struct xennet_xenbus_softc *sc)
{
	struct xennet_txreq *req;
	struct ifnet *ifp = &sc->sc_arpcom.ac_if;
	RING_IDX resp_prod, i;

	DPRINTFN(XEDB_EVENT, ("xennet_tx_complete prod %d cons %d\n",
	    sc->sc_tx_ring.sring->rsp_prod, sc->sc_tx_ring.rsp_cons));

again:
	resp_prod = sc->sc_tx_ring.sring->rsp_prod;
	x86_lfence();
	for (i = sc->sc_tx_ring.rsp_cons; i != resp_prod; i++) {
		req = &sc->sc_txreqs[RING_GET_RESPONSE(&sc->sc_tx_ring, i)->id];
		KASSERT(req->txreq_id ==
		    RING_GET_RESPONSE(&sc->sc_tx_ring, i)->id);
		if (__predict_false(xengnt_status(req->txreq_gntref))) {
			printf("%s: grant still used by backend\n",
			    sc->sc_dev.dv_xname);
			sc->sc_tx_ring.rsp_cons = i;
			goto end;
		}
		if (__predict_false(
		    RING_GET_RESPONSE(&sc->sc_tx_ring, i)->status !=
		    NETIF_RSP_OKAY))
			ifp->if_oerrors++;
		else
			ifp->if_opackets++;
		xengnt_revoke_access(req->txreq_gntref);
		m_freem(req->txreq_m);
		SLIST_INSERT_HEAD(&sc->sc_txreq_head, req, txreq_next);
	}
	sc->sc_tx_ring.rsp_cons = resp_prod;
	/* set new event and check fopr race with rsp_cons update */
	sc->sc_tx_ring.sring->rsp_event =
	    resp_prod + ((sc->sc_tx_ring.sring->req_prod - resp_prod) >> 1) + 1;
	ifp->if_timer = 0;
	x86_sfence();
	if (resp_prod != sc->sc_tx_ring.sring->rsp_prod)
		goto again;
end:
	if (ifp->if_flags & IFF_OACTIVE) {
		ifp->if_flags &= ~IFF_OACTIVE;
		xennet_softstart(sc);
	}
}


int
xennet_handler(void *arg)
{
	struct xennet_xenbus_softc *sc = arg;
	struct ifnet *ifp = &sc->sc_arpcom.ac_if;
	RING_IDX resp_prod, i;
	struct xennet_rxreq *req;
	paddr_t ma, pa;
	vaddr_t va;
	mmu_update_t mmu[1];
	multicall_entry_t mcl[2];
	struct mbuf *m;
	void *pktp;
	int more_to_do;

	if (sc->sc_backend_status != BEST_CONNECTED)
		return 1;

	xennet_tx_complete(sc);

again:
	DPRINTFN(XEDB_EVENT, ("xennet_handler prod %d cons %d\n",
	    sc->sc_rx_ring.sring->rsp_prod, sc->sc_rx_ring.rsp_cons));

	resp_prod = sc->sc_rx_ring.sring->rsp_prod;
	x86_lfence(); /* ensure we see replies up to resp_prod */
	for (i = sc->sc_rx_ring.rsp_cons; i != resp_prod; i++) {
		netif_rx_response_t *rx = RING_GET_RESPONSE(&sc->sc_rx_ring, i);
		req = &sc->sc_rxreqs[rx->id];
		KASSERT(req->rxreq_gntref != GRANT_INVALID_REF);
		KASSERT(req->rxreq_id == rx->id);
		ma = xengnt_revoke_transfer(req->rxreq_gntref);
		if (ma == 0) {
			DPRINTFN(XEDB_EVENT, ("xennet_handler ma == 0\n"));
			/*
			 * the remote could't send us a packet.
			 * we can't free this rxreq as no page will be mapped
			 * here. Instead give it back immediatly to backend.
			 */
			ifp->if_ierrors++;
			RING_GET_REQUEST(&sc->sc_rx_ring,
			    sc->sc_rx_ring.req_prod_pvt)->id = req->rxreq_id;
			RING_GET_REQUEST(&sc->sc_rx_ring,
			    sc->sc_rx_ring.req_prod_pvt)->gref =
				req->rxreq_gntref;
			sc->sc_rx_ring.req_prod_pvt++;
			RING_PUSH_REQUESTS(&sc->sc_rx_ring);
			continue;
		}
		req->rxreq_gntref = GRANT_INVALID_REF;

		pa = req->rxreq_pa;
		va = req->rxreq_va;
		/* remap the page */
		mmu[0].ptr = (ma << PAGE_SHIFT) | MMU_MACHPHYS_UPDATE;
		mmu[0].val = ((pa - XPMAP_OFFSET) >> PAGE_SHIFT);
		MULTI_update_va_mapping(&mcl[0], va,
		    (ma << PAGE_SHIFT) | PG_V | PG_KW, UVMF_TLB_FLUSH|UVMF_ALL);
		xpmap_phys_to_machine_mapping[
		    (pa - XPMAP_OFFSET) >> PAGE_SHIFT] = ma;
		mcl[1].op = __HYPERVISOR_mmu_update;
		mcl[1].args[0] = (unsigned long)mmu;
		mcl[1].args[1] = 1;
		mcl[1].args[2] = 0;
		mcl[1].args[3] = DOMID_SELF;
		HYPERVISOR_multicall(mcl, 2);
		pktp = (void *)(va + rx->offset);
#ifdef XENNET_DEBUG_DUMP
		xennet_hex_dump(pktp, rx->status, "r", rx->id);
#endif
		if ((ifp->if_flags & IFF_PROMISC) == 0) {
			struct ether_header *eh = pktp;
			if (ETHER_IS_MULTICAST(eh->ether_dhost) == 0 &&
			    memcmp(LLADDR(ifp->if_sadl), eh->ether_dhost,
			    ETHER_ADDR_LEN) != 0) {
				DPRINTFN(XEDB_EVENT,
				    ("xennet_handler bad dest\n"));
				/* packet not for us */
				xennet_rx_mbuf_free((void *)va, PAGE_SIZE,
				    req);
				continue;
			}
		}
		MGETHDR(m, M_DONTWAIT, MT_DATA);
		if (__predict_false(m == NULL)) {
			printf("xennet: rx no mbuf\n");
			ifp->if_ierrors++;
			xennet_rx_mbuf_free((void *)va, PAGE_SIZE, req);
			continue;
		}
		MCLAIM(m, &sc->sc_arpcom.ac_rx_mowner);

		m->m_pkthdr.rcvif = ifp;
		if (__predict_true(sc->sc_rx_ring.req_prod_pvt !=
		    sc->sc_rx_ring.sring->rsp_prod)) {
			m->m_len = m->m_pkthdr.len = rx->status;
			MEXTADD(m, pktp, rx->status,
			    M_DEVBUF, xennet_rx_mbuf_free, req);
			m->m_flags |= M_EXT_RW; /* we own the buffer */
			req->rxreq_gntref = GRANT_STACK_REF;
		} else {
			/*
			 * This was our last receive buffer, allocate
			 * memory, copy data and push the receive
			 * buffer back to the hypervisor.
			 */
			m->m_len = min(MHLEN, rx->status);
			m->m_pkthdr.len = 0;
			m_copyback(m, 0, rx->status, pktp);
			xennet_rx_mbuf_free((void *)va, PAGE_SIZE, req);
			if (m->m_pkthdr.len < rx->status) {
				/* out of memory, just drop packets */
				ifp->if_ierrors++;
				m_freem(m);
				continue;
			}
		}
		if ((rx->flags & NETRXF_csum_blank) != 0) {
			xennet_checksum_fill(&m);
			if (m == NULL) {
				ifp->if_ierrors++;
				continue;
			}
		}
#if NBPFILTER > 0
		/*
		 * Pass packet to bpf if there is a listener.
		 */
		if (ifp->if_bpf)
			bpf_mtap(ifp->if_bpf, m, BPF_DIRECTION_IN);
#endif

		ifp->if_ipackets++;

		/* Pass the packet up. */
		ether_input_mbuf(ifp, m);
	}
	x86_lfence();
	sc->sc_rx_ring.rsp_cons = i;
	RING_FINAL_CHECK_FOR_RESPONSES(&sc->sc_rx_ring, more_to_do);
	if (more_to_do)
		goto again;
        return 1;
}










static int
xennet_driver_count_connected(void)
{
	struct device *dv;
	struct xennet_softc *xs = NULL;

	netctrl.xc_interfaces = netctrl.xc_connected = 0;
	TAILQ_FOREACH(dv, &alldevs, dv_list) {
		if (dv->dv_cfdata == NULL ||
		    dv->dv_cfdata->cf_attach == NULL ||
		    dv->dv_cfdata->cf_attach->ca_attach != xennet_attach)
			continue;
		xs = (struct xennet_softc *)dv;
		netctrl.xc_interfaces++;
		if (xs->sc_backend_state == BEST_CONNECTED)
			netctrl.xc_connected++;
	}

	return netctrl.xc_connected;
}

static void
xennet_interface_status_change(netif_fe_interface_status_t *status)
{
	ctrl_msg_t cmsg;
	netif_fe_interface_connect_t up;
	struct xennet_softc *sc;
	struct ifnet *ifp;
	struct xennet_attach_args xneta;

	DPRINTFN(XEDB_EVENT, ("xennet_interface_status_change(%d,%d,%02x:%02x:%02x:%02x:%02x:%02x)\n",
	    status->status,
	    status->handle,
	    status->mac[0], status->mac[1], status->mac[2],
	    status->mac[3], status->mac[4], status->mac[5]));

	sc = find_device(status->handle);
	if (sc == NULL) {
		xneta.xa_device = "xennet";
		xneta.xa_handle = status->handle;
		config_found_sm(netctrl.xc_parent, &xneta,
			netctrl.xc_cfprint, netctrl.xc_cfmatch);
		sc = find_device(status->handle);
		if (sc == NULL) {
			if (in_autoconf) {
				in_autoconf = 0;
				config_pending_decr();
			}
			printf("Status change: invalid netif handle %u\n",
			    status->handle);
			return;
		}
	}
	ifp = &sc->sc_arpcom.ac_if;

	DPRINTFN(XEDB_EVENT, ("xennet_interface_status_change(%d,%p,%02x:%02x:%02x:%02x:%02x:%02x)\n",
		     status->handle, sc,
		     status->mac[0], status->mac[1], status->mac[2],
		     status->mac[3], status->mac[4], status->mac[5]));

	switch (status->status) {
	case NETIF_INTERFACE_STATUS_CLOSED:
		printf("Unexpected netif-CLOSED message in state %d\n",
		    sc->sc_backend_state);
		break;

	case NETIF_INTERFACE_STATUS_DISCONNECTED:

		if (sc->sc_backend_state == BEST_CLOSED) {
			/* Move from CLOSED to DISCONNECTED state. */
			sc->sc_tx = (netif_tx_interface_t *)
			    uvm_km_valloc_align(kernel_map, PAGE_SIZE,
				PAGE_SIZE);
			if (sc->sc_tx == NULL)
				panic("netif: no tx va");
			sc->sc_rx = (netif_rx_interface_t *)
			    uvm_km_valloc_align(kernel_map, PAGE_SIZE, PAGE_SIZE);
			if (sc->sc_rx == NULL)
				panic("netif: no rx va");
			sc->sc_pg_tx = uvm_pagealloc(NULL, 0, NULL, UVM_PGA_ZERO);
			if (sc->sc_pg_tx == NULL) {
				panic("netif: no tx pages");
			}
			pmap_kenter_pa((vaddr_t)sc->sc_tx, VM_PAGE_TO_PHYS(sc->sc_pg_tx),
			    VM_PROT_READ | VM_PROT_WRITE);
			sc->sc_pg_rx = uvm_pagealloc(NULL, 0, NULL, UVM_PGA_ZERO);
			if (sc->sc_pg_rx == NULL) {
				panic("netif: no rx pages");
			}
			pmap_kenter_pa((vaddr_t)sc->sc_rx, VM_PAGE_TO_PHYS(sc->sc_pg_rx),
			    VM_PROT_READ | VM_PROT_WRITE);
			sc->sc_backend_state = BEST_DISCONNECTED;
		}

		/* Construct an interface-CONNECT message for the
		 * domain controller. */
		cmsg.type      = CMSG_NETIF_FE;
		cmsg.subtype   = CMSG_NETIF_FE_INTERFACE_CONNECT;
		cmsg.length    = sizeof(netif_fe_interface_connect_t);
		up.handle      = status->handle;
		up.tx_shmem_frame = xpmap_ptom(VM_PAGE_TO_PHYS(sc->sc_pg_tx)) >> PAGE_SHIFT;
		up.rx_shmem_frame = xpmap_ptom(VM_PAGE_TO_PHYS(sc->sc_pg_rx)) >> PAGE_SHIFT;
		memcpy(cmsg.msg, &up, sizeof(up));

		/* Tell the controller to bring up the interface. */
		ctrl_if_send_message_block(&cmsg, NULL, 0, 0);
		break;

	case NETIF_INTERFACE_STATUS_CONNECTED:
		if (sc->sc_backend_state == BEST_CLOSED) {
			printf("Unexpected netif-CONNECTED message"
			    " in state %d\n", sc->sc_backend_state);
			break;
		}

		memcpy(sc->sc_arpcom.ac_enaddr, status->mac, ETHER_ADDR_LEN);

		/* Recovery procedure: */

		/* Step 1: Reinitialise variables. */
		sc->sc_rx_resp_cons = sc->sc_tx_resp_cons = /* sc->sc_tx_full = */ 0;
		sc->sc_rx->event = sc->sc_tx->event = 1;

		/* Step 2: Rebuild the RX ring contents. */
		network_alloc_rx_buffers(sc);

		/* Step 3: All public and private state should now be
		 * sane.  Get ready to start sending and receiving
		 * packets and give the driver domain a kick because
		 * we've probably just requeued some packets.
		 */
		sc->sc_backend_state = BEST_CONNECTED;
		x86_sfence();
		hypervisor_notify_via_evtchn(status->evtchn);
		network_tx_buf_gc(sc);

		if_attach(ifp);
		ether_ifattach(ifp);

#ifdef __HAVE_GENERIC_SOFT_INTERRUPTS
		sc->sc_softintr = softintr_establish(IPL_SOFTNET,
		    xennet_softstart, sc);
		if (sc->sc_softintr == NULL)
			panic(" xennet: can't establish soft interrupt");
#endif
		sc->sc_evtchn = status->evtchn;
		printf("%s: using event channel %d\n",
		    sc->sc_dev.dv_xname, sc->sc_evtchn);
		event_set_handler(sc->sc_evtchn,
		    &xen_network_handler, sc, IPL_NET, sc->sc_dev.dv_xname);
		hypervisor_enable_event(sc->sc_evtchn);
		xennet_driver_count_connected();

		printf("%s: MAC address %s\n", sc->sc_dev.dv_xname,
		    ether_sprintf(sc->sc_arpcom.ac_enaddr));

#if NRND > 0
		rnd_attach_source(&sc->sc_rnd_source, sc->sc_dev.dv_xname,
		    RND_TYPE_NET, 0);
#endif
		if (in_autoconf) {
			in_autoconf = 0;
			config_pending_decr();
		}
		break;

	default:
		printf("Status change to unknown value %d\n",
		    status->status);
		break;
	}
	DPRINTFN(XEDB_EVENT, ("xennet_interface_status_change()\n"));
}

static void
xennet_rx_push_buffer(struct xennet_softc *sc, int id)
{
	NETIF_RING_IDX ringidx;
	int nr_pfns;
	int s;

	ringidx = sc->sc_rx->req_prod;
	nr_pfns = 0;

	DPRINTFN(XEDB_MEM, ("readding page va %p pa %p ma %p/%p to rx_ring "
		     "at %d with id %d\n",
		     (void *)sc->sc_rx_bufa[id].xb_rx.xbrx_va,
		     (void *)sc->sc_rx_bufa[id].xb_rx.xbrx_pa,
		     (void *)(PTE_BASE[i386_btop
				  (sc->sc_rx_bufa[id].xb_rx.xbrx_va)] &
			 PG_FRAME),
		     (void *)xpmap_ptom(sc->sc_rx_bufa[id].xb_rx.xbrx_pa),
		     ringidx, id));

	sc->sc_rx->ring[MASK_NETIF_RX_IDX(ringidx)].req.id = id;

	rx_pfn_array[nr_pfns] = xpmap_ptom(sc->sc_rx_bufa[id].xb_rx.xbrx_pa)
		>> PAGE_SHIFT;

	/* Remove this page from pseudo phys map before
	 * passing back to Xen. */
	xpmap_phys_to_machine_mapping[(sc->sc_rx_bufa[id].xb_rx.xbrx_pa - XPMAP_OFFSET) >> PAGE_SHIFT] =
		INVALID_P2M_ENTRY;

	rx_mcl[nr_pfns].op = __HYPERVISOR_update_va_mapping;
	rx_mcl[nr_pfns].args[0] = sc->sc_rx_bufa[id].xb_rx.xbrx_va >> PAGE_SHIFT;
	rx_mcl[nr_pfns].args[1] = 0;
	rx_mcl[nr_pfns].args[2] = 0;

	nr_pfns++;

	sc->sc_rx_bufs_to_notify++;

	ringidx++;

	/*
	 * We may have allocated buffers which have entries
	 * outstanding in the page update queue -- make sure we flush
	 * those first!
	 */
	s = splvm();
	xpq_flush_queue();
	splx(s);

	/* After all PTEs have been zapped we blow away stale TLB entries. */
	rx_mcl[nr_pfns-1].args[2] = UVMF_FLUSH_TLB;

	/* Give away a batch of pages. */
	rx_mcl[nr_pfns].op = __HYPERVISOR_dom_mem_op;
	rx_mcl[nr_pfns].args[0] = MEMOP_decrease_reservation;
	rx_mcl[nr_pfns].args[1] = (unsigned long)rx_pfn_array;
	rx_mcl[nr_pfns].args[2] = (unsigned long)nr_pfns;
	rx_mcl[nr_pfns].args[3] = 0;
	rx_mcl[nr_pfns].args[4] = DOMID_SELF;

	/* Zap PTEs and give away pages in one big multicall. */
	(void)HYPERVISOR_multicall(rx_mcl, nr_pfns+1);

	/* Check return status of HYPERVISOR_dom_mem_op(). */
	if ( rx_mcl[nr_pfns].args[5] != nr_pfns )
		panic("Unable to reduce memory reservation\n");

	/* Above is a suitable barrier to ensure backend will see requests. */
	sc->sc_rx->req_prod = ringidx;
}


static int
xen_network_handler(void *arg)
{
	struct xennet_softc *sc = arg;
	struct ifnet *ifp = &sc->sc_arpcom.ac_if;
	netif_rx_response_t *rx;
	paddr_t pa;
	NETIF_RING_IDX ringidx, resp_prod;
	mmu_update_t *mmu = rx_mmu;
	multicall_entry_t *mcl = rx_mcl;
	struct mbuf *m;
	void *pktp;

	xennet_start(ifp); /* to cleanup TX bufs and keep the ifq_send going */

#if NRND > 0
	rnd_add_uint32(&sc->sc_rnd_source, sc->sc_rx_resp_cons);
#endif

 again:
	resp_prod = sc->sc_rx->resp_prod;
	x86_lfence(); /* ensure we see all requests up to resp_prod */
	for (ringidx = sc->sc_rx_resp_cons;
	     ringidx != resp_prod;
	     ringidx++) {
		rx = &sc->sc_rx->ring[MASK_NETIF_RX_IDX(ringidx)].resp;

		if (rx->status < 0)
			panic("rx->status < 0");
		/* XXXcl check rx->status for error */

		MGETHDR(m, M_DONTWAIT, MT_DATA);
		if (m == NULL) {
			printf("xennet: rx no mbuf\n");
			ifp->if_ierrors++;
			break;
		}

		pa = sc->sc_rx_bufa[rx->id].xb_rx.xbrx_pa;

		DPRINTFN(XEDB_RXEVENT, ("rx event %d for id %d, size %d, "
			     "status %d, ma %08lx, pa %08lx\n", ringidx,
			     rx->id, rx->status, rx->status, rx->addr, pa));

		/* Remap the page. */
		mmu->ptr  = (rx->addr & PG_FRAME) | MMU_MACHPHYS_UPDATE;
		mmu->val  = (pa - XPMAP_OFFSET) >> PAGE_SHIFT;
		mmu++;
		mcl->op = __HYPERVISOR_update_va_mapping;
		mcl->args[0] = sc->sc_rx_bufa[rx->id].xb_rx.xbrx_va >> PAGE_SHIFT;
		mcl->args[1] = (rx->addr & PG_FRAME) | PG_V|PG_KW;
		mcl->args[2] = UVMF_FLUSH_TLB; // 0;
		mcl++;

		xpmap_phys_to_machine_mapping
			[(pa - XPMAP_OFFSET) >> PAGE_SHIFT] =
			rx->addr >> PAGE_SHIFT;

		/* Do all the remapping work, and M->P updates, in one
		 * big hypercall. */
		mcl->op = __HYPERVISOR_mmu_update;
		mcl->args[0] = (unsigned long)rx_mmu;
		mcl->args[1] = mmu - rx_mmu;
		mcl->args[2] = 0;
		mcl++;
		(void)HYPERVISOR_multicall(rx_mcl, mcl - rx_mcl);
		if (0)
		printf("page mapped at va %08lx -> %08x/%08lx\n",
		    sc->sc_rx_bufa[rx->id].xb_rx.xbrx_va,
		    PTE_BASE[i386_btop(sc->sc_rx_bufa[rx->id].xb_rx.xbrx_va)],
		    rx->addr);
		mmu = rx_mmu;
		mcl = rx_mcl;

		DPRINTFN(XEDB_MBUF, ("rx packet mbuf %p va %p pa %p/%p "
		    "ma %p\n", m,
		    (void *)sc->sc_rx_bufa[rx->id].xb_rx.xbrx_va,
		    (void *)(xpmap_mtop(PTE_BASE[i386_btop
					    (sc->sc_rx_bufa[rx->id].xb_rx.xbrx_va)] & PG_FRAME)), (void *)pa,
		    (void *)(PTE_BASE[i386_btop
			(sc->sc_rx_bufa[rx->id].xb_rx.xbrx_va)] & PG_FRAME)));

		pktp = (void *)(sc->sc_rx_bufa[rx->id].xb_rx.xbrx_va +
			(rx->addr & PAGE_MASK));
		if ((ifp->if_flags & IFF_PROMISC) == 0) {
			struct ether_header *eh = pktp;
			if (ETHER_IS_MULTICAST(eh->ether_dhost) == 0 &&
			    memcmp(LLADDR(ifp->if_sadl), eh->ether_dhost,
			    ETHER_ADDR_LEN) != 0) {
				xennet_rx_push_buffer(sc, rx->id);
				m_freem(m);
				continue; /* packet not for us */
			}
		}

		m->m_pkthdr.rcvif = ifp;
		if (sc->sc_rx->req_prod != sc->sc_rx->resp_prod) {
			m->m_len = m->m_pkthdr.len = rx->status;
			MEXTADD(m, (caddr_t)pktp, rx->status,
			    M_DEVBUF, xennet_rx_mbuf_free,
			    (void *)&sc->sc_rx_bufa[rx->id]);
		} else {
			/*
			 * This was our last receive buffer, allocate
			 * memory, copy data and push the receive
			 * buffer back to the hypervisor.
			 */
			if (rx->status > MHLEN) {
				MCLGET(m, M_DONTWAIT);
				if ((m->m_flags & M_EXT) == 0) {
					/* out of memory, just drop packets */
					ifp->if_ierrors++;
					m_free(m);
					xennet_rx_push_buffer(sc, rx->id);
					continue;
				}
			}

			m->m_len = m->m_pkthdr.len = rx->status;
			memcpy(mtod(m, void *), pktp, rx->status);
			xennet_rx_push_buffer(sc, rx->id);
		}

#ifdef XENNET_DEBUG_DUMP
		xennet_hex_dump(mtod(m, u_char *), m->m_pkthdr.len, "r", rx->id);
#endif

#if NBPFILTER > 0
		/*
		 * Pass packet to bpf if there is a listener.
		 */
		if (ifp->if_bpf)
			bpf_mtap(ifp->if_bpf, m, BPF_DIRECTION_IN);
#endif

		ifp->if_ipackets++;

		/* Pass the packet up. */
		ether_input_mbuf(ifp, m);
	}

	sc->sc_rx_resp_cons = ringidx;
	sc->sc_rx->event = resp_prod + 1;
	x86_lfence();
	  /* ensure backend see the new sc_rx->event before we start again */

	if (sc->sc_rx->resp_prod != resp_prod)
		goto again;

	return 0;
}

static void
network_tx_buf_gc(struct xennet_softc *sc)
{
	struct ifnet *ifp = &sc->sc_arpcom.ac_if;
	NETIF_RING_IDX idx, prod;

	do {
		prod = sc->sc_tx->resp_prod;

		for (idx = sc->sc_tx_resp_cons; idx != prod; idx++) {
			DPRINTFN(XEDB_TXEVENT, ("tx event at pos %d, status: "
				     "%d, id: %d, mbuf %p, buf %p\n", idx,
				     sc->sc_tx->ring[MASK_NETIF_TX_IDX(idx)].resp.status,
				     sc->sc_tx->ring[MASK_NETIF_TX_IDX(idx)].resp.id,
				     sc->sc_tx_bufa[sc->sc_tx->ring[MASK_NETIF_TX_IDX(idx)].resp.id].xb_tx.xbtx_m,
				     mtod(sc->sc_tx_bufa[sc->sc_tx->ring[MASK_NETIF_TX_IDX(idx)].resp.id].xb_tx.xbtx_m, void *)));

			m_freem(sc->sc_tx_bufa[sc->sc_tx->ring[MASK_NETIF_TX_IDX(idx)].resp.id].xb_tx.xbtx_m);
			put_bufarray_entry(sc->sc_tx_bufa,
			    sc->sc_tx->ring[MASK_NETIF_TX_IDX(idx)].resp.id);
			sc->sc_tx_entries--; /* atomic */

			if (sc->sc_tx->ring[MASK_NETIF_TX_IDX(idx)].resp.status
			    == NETIF_RSP_OKAY) {
				ifp->if_opackets++;
			} else {
				ifp->if_oerrors++;
			}
		}
		if (sc->sc_tx_resp_cons != prod)
			ifp->if_flags &= ~IFF_OACTIVE;
		sc->sc_tx_resp_cons = prod;

		/*
		 * Set a new event, then check for race with update of
		 * tx_cons.
		 */
		sc->sc_tx->event = /* atomic */
			prod + (sc->sc_tx_entries >> 1) + 1;
		x86_lfence();
	} while (prod != sc->sc_tx->resp_prod);

	if (sc->sc_tx->resp_prod == sc->sc_tx->req_prod)
		ifp->if_timer = 0;
	/* KDASSERT(sc->sc_net_idx->tx_req_prod == */
	/* TX_RING_ADD(sc->sc_net_idx->tx_resp_prod, sc->sc_tx_entries)); */
}

static void
network_alloc_rx_buffers(struct xennet_softc *sc)
{
	vaddr_t rxpages, va;
	paddr_t pa;
	struct vm_page *pg;
	int id, nr_pfns;
	NETIF_RING_IDX ringidx;
	int snet, svm;

	ringidx = sc->sc_rx->req_prod;
	if ((ringidx - sc->sc_rx_resp_cons) > (RX_MAX_ENTRIES / 2))
		return;

	nr_pfns = 0;

	rxpages = uvm_km_valloc_align(kernel_map, RX_ENTRIES * PAGE_SIZE, PAGE_SIZE);
	KASSERT(rxpages);

	snet = splnet();
	for (va = rxpages; va < rxpages + RX_ENTRIES * PAGE_SIZE;
	     va += PAGE_SIZE) {
		pg = uvm_pagealloc(NULL, 0, NULL, 0);
		if (pg == NULL)
			panic("network_alloc_rx_buffers: no pages");
		pmap_kenter_pa(va, VM_PAGE_TO_PHYS(pg),
		    VM_PROT_READ | VM_PROT_WRITE);

		id = get_bufarray_entry(sc->sc_rx_bufa);
		KASSERT(id < NETIF_RX_RING_SIZE);
		sc->sc_rx_bufa[id].xb_rx.xbrx_va = va;
		sc->sc_rx_bufa[id].xb_rx.xbrx_sc = sc;

		pa = VM_PAGE_TO_PHYS(pg);
		DPRINTFN(XEDB_MEM, ("adding page va %p pa %p/%p "
		    "ma %p/%p to rx_ring at %d with id %d\n", (void *)va,
			     (void *)(VM_PAGE_TO_PHYS(pg) & PG_FRAME), (void *)xpmap_mtop(PTE_BASE[i386_btop(va)]),
		    (void *)(PTE_BASE[i386_btop(va)] & PG_FRAME),
			     (void *)xpmap_ptom(VM_PAGE_TO_PHYS(pg)),
		    ringidx, id));
		sc->sc_rx_bufa[id].xb_rx.xbrx_pa = pa;
		sc->sc_rx->ring[MASK_NETIF_RX_IDX(ringidx)].req.id = id;

		rx_pfn_array[nr_pfns] = xpmap_ptom(pa) >> PAGE_SHIFT;

		/* Remove this page from pseudo phys map before
		 * passing back to Xen. */
		xpmap_phys_to_machine_mapping[(pa - XPMAP_OFFSET) >> PAGE_SHIFT] =
			INVALID_P2M_ENTRY;

		rx_mcl[nr_pfns].op = __HYPERVISOR_update_va_mapping;
		rx_mcl[nr_pfns].args[0] = va >> PAGE_SHIFT;
		rx_mcl[nr_pfns].args[1] = 0;
		rx_mcl[nr_pfns].args[2] = 0;

		nr_pfns++;

		sc->sc_rx_bufs_to_notify++;

		ringidx++;
		if ((ringidx - sc->sc_rx_resp_cons) == RX_MAX_ENTRIES)
			break;
	}

	if (nr_pfns == 0) {
		splx(snet);
		return;
	}

	/*
	 * We may have allocated buffers which have entries
	 * outstanding in the page update queue -- make sure we flush
	 * those first!
	 */
	svm = splvm();
	xpq_flush_queue();
	splx(svm);

	/* After all PTEs have been zapped we blow away stale TLB entries. */
	rx_mcl[nr_pfns-1].args[2] = UVMF_FLUSH_TLB;

	/* Give away a batch of pages. */
	rx_mcl[nr_pfns].op = __HYPERVISOR_dom_mem_op;
	rx_mcl[nr_pfns].args[0] = MEMOP_decrease_reservation;
	rx_mcl[nr_pfns].args[1] = (unsigned long)rx_pfn_array;
	rx_mcl[nr_pfns].args[2] = (unsigned long)nr_pfns;
	rx_mcl[nr_pfns].args[3] = 0;
	rx_mcl[nr_pfns].args[4] = DOMID_SELF;

	/* Zap PTEs and give away pages in one big multicall. */
	(void)HYPERVISOR_multicall(rx_mcl, nr_pfns+1);

	/* Check return status of HYPERVISOR_dom_mem_op(). */
	if (rx_mcl[nr_pfns].args[5] != nr_pfns)
		panic("Unable to reduce memory reservation\n");

	/* Above is a suitable barrier to ensure backend will see requests. */
	sc->sc_rx->req_prod = ringidx;

	splx(snet);

}

/*
 * Called at splnet.
 */
void
xennet_start(struct ifnet *ifp)
{
	struct xennet_softc *sc = ifp->if_softc;

	DPRINTFN(XEDB_FOLLOW, ("%s: xennet_start()\n", sc->sc_dev.dv_xname));

#if NRND > 0
	rnd_add_uint32(&sc->sc_rnd_source, sc->sc_tx->req_prod);
#endif

	network_tx_buf_gc(sc);

	if (__predict_false(
	    (ifp->if_flags & (IFF_RUNNING | IFF_OACTIVE)) != IFF_RUNNING))
		return;

#ifdef __HAVE_GENERIC_SOFT_INTERRUPTS
	/*
	 * The Xen communication channel is much more efficient if we can
	 * schedule batch of packets for domain0. To achieve this, we
	 * schedule a soft interrupt, and just return. This way, the network
	 * stack will enqueue all pending mbufs in the interface's send queue
	 * before it is processed by xennet_softstart().
	 */
	softintr_schedule(sc->sc_softintr);
#else
	xennet_softstart(sc);
#endif
	DPRINTFN(XEDB_FOLLOW, ("%s: xennet_start() done\n",
	    sc->sc_dev.dv_xname));
	return;
}


/*
 * Helper function that "emulates" NetBSD's
 * ext_paddr mbuf member
 */
static inline paddr_t
_mbuf_ext_paddr(struct mbuf *m)
{
	pt_entry_t pte;
	vaddr_t va;
	paddr_t pa;

	va = (vaddr_t)m->m_ext.ext_buf;
	pte = PTE_GET(&PTE_BASE[i386_btop(va)]);
	pa = (paddr_t)((pte & PG_FRAME) | (va & ~PG_FRAME));
	return pa;
}


/*
 * Helper function that "emulates" NetBSD's
 * m_paddr mbuf member
 */
static inline paddr_t
_mbuf_m_paddr(struct mbuf *m)
{
	pt_entry_t pte;
	vaddr_t va;
	paddr_t pa;

	va = (vaddr_t)m;
	pte = PTE_GET(&PTE_BASE[i386_btop(va)]);
	pa = (paddr_t)((pte & PG_FRAME) | (va & ~PG_FRAME));
	return pa;
}


#ifndef M_BUFADDR
/*
 * Compute the address of an mbuf's data area.
 */
#define M_BUFADDR(m)							\
	(((m)->m_flags & M_PKTHDR) ? (m)->m_pktdat : (m)->m_dat)
#endif


#ifndef M_BUFOFFSET
/*
 * Compute the offset of the beginning of the data buffer of a
 * non-ext mbuf.
 */
#define M_BUFOFFSET(m)							\
	(((m)->m_flags & M_PKTHDR) ?					\
	 offsetof(struct mbuf, m_pktdat) : offsetof(struct mbuf, m_dat))
#endif



/*
 * called at splsoftnet
 */
void
xennet_softstart(void *arg)
{
	struct xennet_softc *sc = arg;
	struct ifnet *ifp = &sc->sc_arpcom.ac_if;
	struct mbuf *m, *new_m;
	netif_tx_request_t *txreq;
	NETIF_RING_IDX idx;
	paddr_t pa;
	int bufid;
	int s;

	s = splnet();
	if (__predict_false(
	    (ifp->if_flags & (IFF_RUNNING | IFF_OACTIVE)) != IFF_RUNNING)) {
		splx(s);
		return;
	}

	DPRINTFN(XEDB_FOLLOW, ("%s: xennet_softstart()\n",
		sc->sc_dev.dv_xname));

	idx = sc->sc_tx->req_prod;
	while (/*CONSTCOND*/1) {
		if (__predict_false(
		    sc->sc_tx_entries >= NETIF_TX_RING_SIZE - 1)) {
			ifp->if_flags |= IFF_OACTIVE;
			break;
		}
		IFQ_POLL(&ifp->if_snd, m);
		if (m == NULL)
			break;

#define M_PADDR_INVALID		-1

		switch (m->m_flags & (M_EXT|M_CLUSTER)) {
		case M_EXT|M_CLUSTER:
			pa = _mbuf_ext_paddr(m);
			KASSERT(pa != M_PADDR_INVALID);
			pa += (m->m_data - m->m_ext.ext_buf);
			break;
		case 0:
			pa = _mbuf_m_paddr(m);
			KASSERT(pa != M_PADDR_INVALID);
			pa += M_BUFOFFSET(m) +
				(m->m_data - M_BUFADDR(m));
			break;
		default:
			if (__predict_false(
			    !pmap_extract(pmap_kernel(), (vaddr_t)m->m_data,
			    &pa))) {
				panic("xennet_start: no pa");
			}
			break;
		}

		if (m->m_pkthdr.len != m->m_len ||
		    (pa ^ (pa + m->m_pkthdr.len - 1)) & PG_FRAME) {

			MGETHDR(new_m, M_DONTWAIT, MT_DATA);
			if (__predict_false(new_m == NULL)) {
				printf("xennet: no mbuf\n");
				break;
			}
			if (m->m_pkthdr.len > MHLEN) {
				MCLGET(new_m, M_DONTWAIT);
				if (__predict_false(
				    (new_m->m_flags & M_EXT) == 0)) {
					DPRINTF(("xennet: no mbuf cluster\n"));
					m_freem(new_m);
					break;
				}
			}
			IFQ_DEQUEUE(&ifp->if_snd, m);

			m_copydata(m, 0, m->m_pkthdr.len, mtod(new_m, caddr_t));
			new_m->m_len = new_m->m_pkthdr.len = m->m_pkthdr.len;

			if ((new_m->m_flags & M_EXT) != 0) {
				pa = _mbuf_ext_paddr(new_m);
				KASSERT(new_m->m_data == new_m->m_ext.ext_buf);
				KASSERT(pa != M_PADDR_INVALID);
			} else {
				pa = _mbuf_m_paddr(new_m);
				KASSERT(pa != M_PADDR_INVALID);
				KASSERT(new_m->m_data == M_BUFADDR(new_m));
				pa += M_BUFOFFSET(new_m);
			}
#ifdef XENNET_DEBUG
			if (((pa ^ (pa + new_m->m_pkthdr.len - 1)) & PG_FRAME) != 0) {
				printf("pa %p len %d m_flags 0x%x\n",
				    (void *)pa, new_m->m_pkthdr.len, new_m->m_flags);
				printf("new_m %p m_paddr %lx M_BUFOFFSET %d\n",
				   new_m,
				   ((new_m->m_flags & M_EXT) != 0) ?
					_mbuf_ext_paddr(new_m) : _mbuf_m_paddr(new_m),
				   ((new_m->m_flags & M_EXT) != 0) ?
					0 : M_BUFOFFSET(new_m));
				panic("xennet_start1: buffer crosses page");
			}
#endif
			m_freem(m);
			m = new_m;
		} else
			IFQ_DEQUEUE(&ifp->if_snd, m);

#undef M_PADDR_INVALID
#ifdef XENNET_DEBUG
		if (((pa ^ (pa + m->m_pkthdr.len - 1)) & PG_FRAME) != 0) {
			printf("pa %p len %d m_flags 0x%x\n",
			    (void *)pa, m->m_pkthdr.len, m->m_flags);
			printf("m %p m_paddr %lx M_BUFOFFSET %d\n",
			   m,
			   ((m->m_flags & M_EXT) != 0) ?
				_mbuf_ext_paddr(m) : _mbuf_m_paddr(m),
			   ((m->m_flags & M_EXT) != 0) ?
				0 : M_BUFOFFSET(m));
			panic("xennet_start2: buffer crosses page");
		}
#endif

		bufid = get_bufarray_entry(sc->sc_tx_bufa);
		KASSERT(bufid < NETIF_TX_RING_SIZE);
		sc->sc_tx_bufa[bufid].xb_tx.xbtx_m = m;

		DPRINTFN(XEDB_MBUF, ("xennet_start id %d, mbuf %p, buf %p/%p, "
			     "size %d\n", bufid, m, mtod(m, void *),
			     (void *)pa, m->m_pkthdr.len));
#ifdef XENNET_DEBUG_DUMP
		xennet_hex_dump(mtod(m, u_char *), m->m_pkthdr.len, "s", bufid);
#endif

		txreq = &sc->sc_tx->ring[MASK_NETIF_TX_IDX(idx)].req;
		txreq->id = bufid;
		txreq->addr = xpmap_ptom(pa);
		txreq->size = m->m_pkthdr.len;

		x86_lfence();
		idx++;
		sc->sc_tx->req_prod = idx;

		sc->sc_tx_entries++; /* XXX atomic */
		x86_lfence();

#ifdef XENNET_DEBUG
		DPRINTFN(XEDB_MEM, ("packet addr %p/%p, physical %p/%p, "
		    "m_paddr %p, len %d/%d\n", M_BUFADDR(m), mtod(m, void *),
		    (void *)*kvtopte(mtod(m, vaddr_t)),
		    (void *)xpmap_mtop(*kvtopte(mtod(m, vaddr_t))),
		    (void *)_mbuf_m_paddr(m), m->m_pkthdr.len, m->m_len));
#endif

#if NBPFILTER > 0
		/*
		 * Pass packet to bpf if there is a listener.
		 */
		if (ifp->if_bpf) {
			bpf_mtap(ifp->if_bpf, m, BPF_DIRECTION_OUT);
		}
#endif
	}

	x86_lfence();
	if (sc->sc_tx->resp_prod != idx) {
		hypervisor_notify_via_evtchn(sc->sc_evtchn);
		ifp->if_timer = 5;
	}
	splx(s);

	DPRINTFN(XEDB_FOLLOW, ("%s: xennet_softstart() done\n",
	    sc->sc_dev.dv_xname));
}

int
xennet_ioctl(struct ifnet *ifp, u_long cmd, caddr_t data)
{
	struct xennet_softc *sc = ifp->if_softc;
	struct ifreq *ifr = (struct ifreq *)data;
	struct ifaddr *ifa = (struct ifaddr *)data;
	int s, error = 0;

	s = splnet();

	DPRINTFN(XEDB_FOLLOW, ("%s: xennet_ioctl()\n", sc->sc_dev.dv_xname));
	error = ether_ioctl(ifp, &sc->sc_arpcom, cmd, data);
#ifdef __NetBSD__
	/* XXX I'm unsure if we also need this,
	 *  so I leave this marked as NetBSD specific for now
	 */
	if (error == ENETRESET) {
		error = 0;
	}
#endif

	switch (cmd) {
	case SIOCSIFADDR:
		ifp->if_flags |= IFF_UP;
		switch (ifa->ifa_addr->sa_family) {
#ifdef INET
		case AF_INET:
			xennet_init(ifp);
			arp_ifinit(&sc->sc_arpcom, ifa);
			break;

#endif
		default:
			break;
		}
		break;

	case SIOCSIFMEDIA:
	case SIOCGIFMEDIA:
		error = ifmedia_ioctl(ifp, ifr, &sc->sc_media, cmd);
		break;

	case SIOCSIFMTU:
		if (ifr->ifr_mtu > ETHERMTU || ifr->ifr_mtu < ETHERMIN) {
			error = EINVAL;
		} else {
			ifp->if_mtu = ifr->ifr_mtu;
		}
		break;

	case SIOCSIFFLAGS:
		/*
		 * If interface is marked up and not running, then start it.
		 * If it is marked down and running, stop it.
		 * XXX If it's up then re-initialize it. This is so flags
		 * such as IFF_PROMISC are handled.
		 */
		if (ifp->if_flags & IFF_UP) {
			xennet_init(ifp);
		} else if (ifp->if_flags & IFF_RUNNING) {
			xennet_stop(ifp);
		}
		break;

	case SIOCADDMULTI:
	case SIOCDELMULTI:
		error = (cmd == SIOCADDMULTI) ?
			ether_addmulti(ifr, &sc->sc_arpcom) :
			ether_delmulti(ifr, &sc->sc_arpcom);
		if (error == ENETRESET) {
			/*
			 * Multicast list has changed; set the hardware
			 * filter accordingly.
			 */
			xennet_init(ifp);
			error = 0;
		}
		break;

	case SIOCAIFADDR:
		if (error == ENETRESET)
			error = 0;
		break;

	default:
		error = EINVAL;
		break;
	}
	splx(s);

	DPRINTFN(XEDB_FOLLOW, ("%s: xennet_ioctl() returning %d\n",
	    sc->sc_dev.dv_xname, error));

	return error;
}

void
xennet_watchdog(struct ifnet *ifp)
{
	struct xennet_softc *sc = ifp->if_softc;

	printf("%s: xennet_watchdog\n", sc->sc_dev.dv_xname);
}

int
xennet_init(struct ifnet *ifp)
{
	int s;
#ifdef XENNET_DEBUG
	struct xennet_softc *sc = ifp->if_softc;

	DPRINTFN(XEDB_INIT, ("%s: xennet_init()\n", sc->sc_dev.dv_xname));
#endif
	s = splnet();
	if (ifp->if_flags & IFF_UP) {
		if ((ifp->if_flags & IFF_RUNNING) == 0)
			xennet_reset(ifp);

		ifp->if_flags |= IFF_RUNNING;
		ifp->if_flags &= ~IFF_OACTIVE;
		ifp->if_timer = 0;
	} else {
		ifp->if_flags &= ~IFF_RUNNING;
		xennet_reset(ifp);
	}
	splx(s);
	return 0;
}

int
xennet_stop(struct ifnet *ifp)
{
	ifp->if_flags &= ~(IFF_RUNNING | IFF_OACTIVE);
	return 0;
}

int
xennet_reset(struct ifnet *ifp)
{
#ifdef XENNET_DEBUG
	struct xennet_softc *sc = ifp->if_softc;

	DPRINTFN(XEDB_FOLLOW, ("%s: xennet_reset()\n", sc->sc_dev.dv_xname));
#endif
	return 0;
}

/*
 * Media change callback.
 */
static int
xennet_mediachange(struct ifnet *ifp)
{
	struct xennet_softc *sc = ifp->if_softc;

	switch IFM_SUBTYPE(sc->sc_media.ifm_media) {
	case IFM_AUTO:
		break;
	default:
		return (1);
		break;
	}

	return (0);
}

/*
 * Media status callback.
 */
static void
xennet_mediastatus(struct ifnet *ifp, struct ifmediareq *ifmr)
{
	struct xennet_softc *sc = ifp->if_softc;

	if (IFM_SUBTYPE(ifmr->ifm_active) == IFM_AUTO)
		ifmr->ifm_active = sc->sc_media.ifm_cur->ifm_data;

	ifmr->ifm_status &= ~IFM_AVALID;
}

#if defined(NFS_BOOT_BOOTSTATIC)
int
xennet_bootstatic_callback(struct nfs_diskless *nd)
{
	struct ifnet *ifp = nd->nd_ifp;
	struct xennet_softc *sc = (struct xennet_softc *)ifp->if_softc;
	union xen_cmdline_parseinfo xcp;
	struct sockaddr_in *sin;

	memset(&xcp, 0, sizeof(xcp.xcp_netinfo));
	xcp.xcp_netinfo.xi_ifno = sc->sc_ifno;
	xcp.xcp_netinfo.xi_root = nd->nd_root.ndm_host;
	xen_parse_cmdline(XEN_PARSE_NETINFO, &xcp);

	nd->nd_myip.s_addr = ntohl(xcp.xcp_netinfo.xi_ip[0]);
	nd->nd_gwip.s_addr = ntohl(xcp.xcp_netinfo.xi_ip[2]);
	nd->nd_mask.s_addr = ntohl(xcp.xcp_netinfo.xi_ip[3]);

	sin = (struct sockaddr_in *) &nd->nd_root.ndm_saddr;
	memset((caddr_t)sin, 0, sizeof(*sin));
	sin->sin_len = sizeof(*sin);
	sin->sin_family = AF_INET;
	sin->sin_addr.s_addr = ntohl(xcp.xcp_netinfo.xi_ip[1]);

	return (NFS_BOOTSTATIC_HAS_MYIP|NFS_BOOTSTATIC_HAS_GWIP|
	    NFS_BOOTSTATIC_HAS_MASK|NFS_BOOTSTATIC_HAS_SERVADDR|
	    NFS_BOOTSTATIC_HAS_SERVER);
}
#endif /* defined(NFS_BOOT_BOOTSTATIC) */


#ifdef XENNET_DEBUG_DUMP
const char hexdigits[] = {
 '0', '1', '2', '3',
 '4', '5', '6', '7',
 '8', '9', 'a', 'b',
 'c', 'd', 'e', 'f'
};
#define XCHR(x) hexdigits[(x) & 0xf]
static void
xennet_hex_dump(unsigned char *pkt, size_t len, char *type, int id)
{
	size_t i, j;

	printf("pkt %p len %d/%x type %s id %d\n", pkt, len, len, type, id);
	printf("00000000  ");
	for(i=0; i<len; i++) {
		printf("%c%c ", XCHR(pkt[i]>>4), XCHR(pkt[i]));
		if ((i+1) % 16 == 8)
			printf(" ");
		if ((i+1) % 16 == 0) {
			printf(" %c", '|');
			for(j=0; j<16; j++)
				printf("%c", pkt[i-15+j]>=32 &&
				    pkt[i-15+j]<127?pkt[i-15+j]:'.');
			printf("%c\n%c%c%c%c%c%c%c%c  ", '|',
			    XCHR((i+1)>>28), XCHR((i+1)>>24),
			    XCHR((i+1)>>20), XCHR((i+1)>>16),
			    XCHR((i+1)>>12), XCHR((i+1)>>8),
			    XCHR((i+1)>>4), XCHR(i+1));
		}
	}
	printf("\n");
}
#undef XCHR
#endif
