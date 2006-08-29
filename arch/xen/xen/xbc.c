/* $NetBSD: xbd.c,v 1.24 2005/10/18 00:14:43 yamt Exp $ */

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


#include <sys/types.h>
#include <sys/param.h>
#include <sys/systm.h>
#include <sys/proc.h>
#include <sys/errno.h>
#include <sys/buf.h>
#include <sys/malloc.h>
#include <sys/pool.h>
#include <sys/ioctl.h>
#include <sys/device.h>
#include <sys/disk.h>
#include <sys/disklabel.h>
#include <sys/fcntl.h>
#include <sys/vnode.h>
#include <sys/lock.h>
#include <sys/conf.h>
#include <sys/queue.h>
#include <sys/stat.h>
#include <sys/kernel.h>

#include <uvm/uvm.h>
#include <machine/uvm_km_kmemalloc.h>

#include <machine/xen-public/io/ring.h>
#include <machine/xen-public/io/blkif.h>

#include <machine/granttables.h>
#include <machine/xenbus.h>

#include <scsi/scsi_all.h>
#include <scsi/scsi_disk.h>
#include <scsi/scsiconf.h>

#include <machine/xen.h>
#include <machine/hypervisor.h>
#include <machine/evtchn.h>

#define XBC_DEBUG

#ifdef XBC_DEBUG
#define DPRINTF(x) printf x;
#else
#define DPRINTF(x)
#endif

#define GRANT_INVALID_REF	-1
#define XBC_INITED      0x00010000 /* unit has been initialized */

#define XBC_RING_SIZE __RING_SIZE((blkif_sring_t *)0, PAGE_SIZE)

#define XEN_BSHIFT	9	/* log2(XEN_BSIZE) */
#define XEN_BSIZE	(1 << XEN_BSHIFT)

struct xbc_req {
	SLIST_ENTRY(xbc_req) req_next;
	uint16_t req_id; /* ID passed to backed */
	grant_ref_t req_gntref[BLKIF_MAX_SEGMENTS_PER_REQUEST];
	int req_nr_segments; /* number of segments in this request */
	struct buf *req_bp; /* buffer associated with this request */
	void *req_aligned; /* pointer to aligned data buffer */
	void *req_data; /* pointer to the data buffer */
};

#if 0
struct xbc_vdisk_t {
	struct device vd_dev;
	SLIST_ENTRY(xbc_vdisk_t) vdisk_next;
	struct xenbus_device *vd_xbusd;

	blkif_front_ring_t vd_ring;
	unsigned int vd_evtchn;

	grant_ref_t vd_ring_gntref;

	uint8_t scsi_target;
	struct xbc_req vd_reqs[XBC_RING_SIZE];
	SLIST_HEAD(,xbc_req) vd_xbcreq_head; /* list of free requests */

	int vd_backend_status;		/* our status with backend */
#define BLKIF_STATE_DISCONNECTED	0
#define BLKIF_STATE_CONNECTED		1
#define BLKIF_STATE_SUSPENDED		2
	int vd_shutdown;

	u_long vd_sectors;		/* number of sectors for this device */
	u_long vd_secsize;		/* sector size */
	u_long vd_info;			/* VDISK_* */
	u_long vd_handle;		/* from backend */
};


struct xbc_xenbus_softc {
	struct device sc_dev;		/* base device glue */
	struct scsi_link sc_link;	/* scsi link */
	struct scsi_xfer *sc_xs;

	SLIST_HEAD(,xbc_vdisk_t) sc_vdisk_head; /* Managed vdisks */

	unsigned int sc_type; 
	struct xbc_vdisk_t *sc_vd;	/* backpointer to vdisk,
					 * not valid if this is the controller */
};
#endif


struct xbc_vdisk_softc {
	struct device vd_sc_dev;	/* base device glue */
	struct xenbus_device *vd_sc_xbusd;

	blkif_front_ring_t vd_sc_ring;
	unsigned int vd_sc_evtchn;

	grant_ref_t vd_sc_ring_gntref;

	uint8_t vd_sc_scsi_target;
	struct xbc_req vd_sc_reqs[XBC_RING_SIZE];
	SLIST_HEAD(,xbc_req) vd_sc_xbcreq_head; /* list of free requests */

	int vd_sc_backend_status;	/* our status with backend */
#define BLKIF_STATE_DISCONNECTED	0
#define BLKIF_STATE_CONNECTED		1
#define BLKIF_STATE_SUSPENDED		2
	int vd_sc_shutdown;

	u_long vd_sc_sectors;		/* number of sectors for this device */
	u_long vd_sc_secsize;		/* sector size */
	u_long vd_sc_info;		/* VDISK_* */
	u_long vd_sc_handle;		/* from backend */

	int sc_type;
};



/* Global variable for controller */
struct xbc_controller_softc {
	struct device c_sc_dev;		/* base device glue */
	struct scsi_link c_sc_link;	/* scsi link */
	struct scsi_xfer *c_sc_xs;

	int sc_type;
	int nr_vbds;
};

static struct xbc_controller_softc *controller_sc;


int xbc_xenbus_match(struct device *, void *, void *);
void xbc_xenbus_attach(struct device *, struct device *, void *);
int xbc_xenbus_detach(struct device *, int);

int xbc_xenbus_resume(void *);
int xbc_handler(void *);
//int xbcstart(struct xbc_xenbus_softc *, struct buf *);
void xbc_backend_changed(void *, XenbusState);
void xbc_connect(struct xbc_vdisk_softc *);

int xbc_map_align(struct xbc_req *);
void xbc_unmap_align(struct xbc_req *);

uint8_t xbc_vdisk_used(void);
int xbc_vdisk_init(struct xbc_vdisk_softc *vd_sc,
		struct xenbusdev_attach_args *xa);
int xbc_vdisk_destroy(struct xbc_vdisk_softc *vd_sc);

int xbc_scsi_cmd(struct scsi_xfer *);
int xbc_cmd(struct xbc_vdisk_softc *sc, int command, void *data,
		int datasize, /* struct xbc_vdisk_t *vd, */ int blkno, int flags,
		struct scsi_xfer *xs);


struct cfdriver xbc_cd = {
	NULL, "xbc", DV_DULL
};

struct scsi_adapter xbc_switch = {
	xbc_scsi_cmd, minphys,
	NULL,	/* open_target_lu */
	NULL,	/* close_target_lu */
	NULL	/* ioctl */
};

struct scsi_device xbc_dev = {
	NULL,	/* err_handler  */
	NULL,	/* start */
	NULL,	/* async */
	NULL	/* done */
};


struct cfattach xbc_ca = {
	sizeof(struct xbc_vdisk_softc),	/* take the larger one */
	xbc_xenbus_match, xbc_xenbus_attach,
	xbc_xenbus_detach,	/* detach */
	NULL	/* activate */
};


#if 0
int	xbc_response_handler(void *);
void	xbc_iodone(struct xbc_xenbus_softc *);
#endif

//int xbcinit(struct xbc_xenbus_softc *, struct xenbusdev_attach_args *);
void xbc_copy_internal_data(struct scsi_xfer *xs, void *v, size_t size);
void vbd_update(void);
void signal_requests_to_xen(void);

#ifdef DIAGNOSTIC
#define DIAGPANIC(x)		panic x
#define DIAGCONDPANIC(x,y)	if (x) panic y
#else
#define DIAGPANIC(x)
#define DIAGCONDPANIC(x,y)
#endif


#define XBC_BLK_CONTROLLER	0
#define XBC_BLK_DEVICE		1


static int
xbc_is_disk(struct xenbusdev_attach_args *xa)
{
	if (strcmp(xa->xa_type, "vbd") == 0) {
		return 1;
	}

	return 0;
}

static int
xbc_is_controller(struct xenbusdev_attach_args *xa)
{
	if (strcmp(xa->xa_type, "vbc") == 0) {
		return 1;
	}

	return 0;
}


int
xbc_xenbus_match(struct device *parent, void *match, void *aux)
{
	struct xenbusdev_attach_args *xa = aux;

	if (xbc_is_controller(xa) || xbc_is_disk(xa)) {
		return 1;
	}

	return 0;
}


void
xbc_xenbus_attach(struct device *parent, struct device *self, void *aux)
{
	struct xbc_controller_softc *c_sc = NULL;
	struct xbc_vdisk_softc *vd_sc = NULL;
	struct xenbusdev_attach_args *xa = aux;
	struct scsi_link *sl = NULL; 
	int nr_vbds;
	RING_IDX i;
#ifdef XBC_DEBUG
	char **dir, *val;
	int dir_n = 0;
	char id_str[20];
	int err;
#endif

	if (xbc_is_controller(xa)) {
		KASSERT(controller_sc == NULL);
		controller_sc = c_sc = (struct xbc_controller_softc *)self;
		c_sc->sc_type = XBC_BLK_CONTROLLER;
		c_sc->nr_vbds = 0;

		nr_vbds = 0;

		sl = &controller_sc->c_sc_link;
		sl->adapter_softc = controller_sc;
		sl->adapter = &xbc_switch;
		sl->adapter_buswidth = nr_vbds;
		sl->adapter_target = 255;
		sl->device = &xbc_dev;
		sl->openings = XBC_RING_SIZE - 1;

#if 0
		config_found(&sc->sc_dev, &sc->sc_link, scsiprint);

		/* initialise shared structures and tell backend that we are ready */
		xbc_xenbus_resume(sc);
#endif
//		xbcinit(sc, xa);
		return;
	}

	vd_sc = (struct xbc_vdisk_softc *)self;
	KASSERT(xbc_is_disk(xa));
	KASSERT(controller_sc != NULL); /* Assert, controller has already been attached */
	KASSERT((void *)vd_sc != (void *)controller_sc);
	vd_sc->sc_type = XBC_BLK_DEVICE;

	config_pending_incr();
#ifdef XBC_DEBUG
	printf("xbc: path: %s\n", xa->xa_xbusd->xbusd_path);
	snprintf(id_str, sizeof(id_str), "%d", xa->xa_id);
	err = xenbus_directory(NULL, "device/vbd", id_str, &dir_n, &dir);
	if (err) {
		printf("%s: xenbus_directory err %d\n",
			vd_sc->vd_sc_dev.dv_xname, err);
	} else {
		printf("%s/\n", xa->xa_xbusd->xbusd_path);
		for (i = 0; i < dir_n; i++) {
			printf("\t/%s", dir[i]);
			err = xenbus_read(NULL, xa->xa_xbusd->xbusd_path, dir[i],
				NULL, &val);
			if (err) {
				printf("%s: xenbus_read err %d\n",
					vd_sc->vd_sc_dev.dv_xname, err);
			} else {
				printf(" = %s\n", val);
				free(val, M_DEVBUF);
			}

		} 
	}
#endif /* XBC_DEBUG */			 

	xbc_vdisk_init(vd_sc, xa);
	nr_vbds = xbc_vdisk_used();

	sl = &controller_sc->c_sc_link;
	sl->adapter_buswidth = nr_vbds;

	config_found(&controller_sc->c_sc_dev, &controller_sc->c_sc_link, scsiprint);	

	/* initialize shared structures and tell backend that we are ready */
	xbc_xenbus_resume(vd_sc);

	return;
}



int
xbc_xenbus_detach(struct device *dev, int flags)
{
	struct xbc_vdisk_softc *vd_sc = (void *)dev;
	int s;

	KASSERT(vd_sc->sc_type == XBC_BLK_DEVICE);
	KASSERT((void *)dev != controller_sc);

	s = splbio();

	DPRINTF(("%s: xbc_detach\n", dev->dv_xname));
	if (vd_sc->vd_sc_shutdown == 0) {
		vd_sc->vd_sc_shutdown = 1;
		/* wait for requests to complete */
		while (vd_sc->vd_sc_backend_status == BLKIF_STATE_CONNECTED)
			tsleep(xbc_xenbus_detach, PRIBIO, "xbcdetach", hz/2);
	}
	splx(s);

	if (vd_sc->vd_sc_backend_status == BLKIF_STATE_CONNECTED) {
#ifdef __NetBSD__
		s = splbio();
		/* Kill off any queued buffers. */
		bufq_drain(sc->sc_dksc.sc_bufq);
		bufq_free(sc->sc_dksc.sc_bufq);
		splx(s);

		/* detach disk */
		disk_detach(&sc->sc_dksc.sc_dkdev);
#endif
	}

	event_remove_handler(vd_sc->vd_sc_evtchn, &xbc_handler, vd_sc);
	while (xengnt_status(vd_sc->vd_sc_ring_gntref)) {
		tsleep(xbc_xenbus_detach, PRIBIO, "xbc_ref", hz/2);
	}
	xengnt_revoke_access(vd_sc->vd_sc_ring_gntref);
	uvm_km_free(kernel_map, (vaddr_t)vd_sc->vd_sc_ring.sring,
	    PAGE_SIZE);
	return 0;
}



int
xbc_xenbus_resume(void *p)
{
	struct xbc_vdisk_softc *vd_sc = p;
	struct xenbus_transaction *xbt;
	int error;
	blkif_sring_t *ring;
	paddr_t ma;
	const char *errmsg;

	KASSERT(vd_sc->sc_type == XBC_BLK_DEVICE);
	KASSERT((void *)vd_sc != (void *)controller_sc);

	vd_sc->vd_sc_ring_gntref = GRANT_INVALID_REF;


	/* setup device: alloc event channel and shared ring */
	ring = (void *)uvm_km_zalloc(kernel_map, PAGE_SIZE);
	if (ring == NULL)
		panic("xbd_xenbus_resume: can't alloc rings");

	SHARED_RING_INIT(ring);
	FRONT_RING_INIT(&vd_sc->vd_sc_ring, ring, PAGE_SIZE);

	(void)pmap_extract_ma(pmap_kernel(), (vaddr_t)ring, &ma);
	error = xenbus_grant_ring(vd_sc->vd_sc_xbusd, ma, &vd_sc->vd_sc_ring_gntref);
	if (error)
		return error;
	error = xenbus_alloc_evtchn(vd_sc->vd_sc_xbusd, &vd_sc->vd_sc_evtchn);
	if (error)
		return error;
	printf("%s: using event channel %d\n",
	    vd_sc->vd_sc_dev.dv_xname, vd_sc->vd_sc_evtchn);
	event_set_handler(vd_sc->vd_sc_evtchn, &xbc_handler, vd_sc,
	    IPL_BIO, vd_sc->vd_sc_dev.dv_xname);

again:
	xbt = xenbus_transaction_start();
	if (xbt == NULL)
		return ENOMEM;
	error = xenbus_printf(xbt, vd_sc->vd_sc_xbusd->xbusd_path,
	    "ring-ref","%u", vd_sc->vd_sc_ring_gntref);
	if (error) {
		errmsg = "writing ring-ref";
		goto abort_transaction;
	}
	error = xenbus_printf(xbt, vd_sc->vd_sc_xbusd->xbusd_path,
	    "event-channel", "%u", vd_sc->vd_sc_evtchn);
	if (error) {
		errmsg = "writing event channel";
		goto abort_transaction;
	}
	error = xenbus_switch_state(vd_sc->vd_sc_xbusd, xbt, XenbusStateInitialised);
	if (error) {
		errmsg = "writing frontend XenbusStateInitialised";
		goto abort_transaction;
	}
	error = xenbus_transaction_end(xbt, 0);
	if (error == EAGAIN)
		goto again;
	if (error) {
		xenbus_dev_fatal(vd_sc->vd_sc_xbusd, error, "completing transaction");
		return -1;
	}
	return 0;

abort_transaction:
	xenbus_transaction_end(xbt, 1);
	xenbus_dev_fatal(vd_sc->vd_sc_xbusd, error, "%s", errmsg);
	return error;
}


void xbc_backend_changed(void *arg, XenbusState new_state)
{
	struct xbc_vdisk_softc *vd_sc = arg;
#ifdef __NetBSD__
	char buf[9];
#endif
	int s;

	KASSERT(vd_sc->sc_type == XBC_BLK_DEVICE);
	KASSERT((void *)vd_sc != controller_sc);

	DPRINTF(("%s: new backend state %d\n", vd_sc->vd_sc_dev.dv_xname, new_state));

	switch (new_state) {
	case XenbusStateUnknown:
	case XenbusStateInitialising:
	case XenbusStateInitWait:
	case XenbusStateInitialised:
		break;
	case XenbusStateClosing:
		s = splbio();
		vd_sc->vd_sc_shutdown = 1;
		/* wait for requests to complete */
		while (vd_sc->vd_sc_backend_status == BLKIF_STATE_CONNECTED /* &&
		    vd_sc->vd_sc_dksc.sc_dkdev.dk_stats->io_busy > 0 */)
			tsleep(xbc_xenbus_detach, PRIBIO, "xbcdetach",
			    hz/2);
		splx(s);
		xenbus_switch_state(vd_sc->vd_sc_xbusd, NULL, XenbusStateClosed);
		break;
	case XenbusStateConnected:
		s = splbio();
		if (vd_sc->vd_sc_backend_status == BLKIF_STATE_CONNECTED)
			/* already connected */
			return;
		vd_sc->vd_sc_backend_status = BLKIF_STATE_CONNECTED;
		splx(s);
		xbc_connect(vd_sc);
		vd_sc->vd_sc_shutdown = 0;
		hypervisor_enable_event(vd_sc->vd_sc_evtchn);

#ifdef __NetBSD__
		vd_sc->vd_dksc.vd_sc_size =
		    (uint64_t)vd_sc->vd_sc_sectors * (uint64_t)vd_sc->vd_sc_secsize /
		    DEV_BSIZE;
		pdg = &vd_sc->vd_sc_dksc.vd_geom;
		pdg->pdg_secsize = DEV_BSIZE;
		pdg->pdg_ntracks = 1;
		pdg->pdg_nsectors = 1024 * (1024 / pdg->pdg_secsize);
		pdg->pdg_ncylinders = vd_sc->sc_dksc.sc_size / pdg->pdg_nsectors;

		bufq_alloc(&vd_sc->vd_sc_dksc.vd_bufq, "fcfs", 0);
		vd_sc->vd_sc_dksc.vd_sc_flags |= XBC_INITED;

		disk_attach(&vd_sc->vd_sc_dksc.sc_dkdev);
		/* try to read the disklabel */
		dk_getdisklabel(vd_sc->vd_sc_di, &vd->sc_dksc, 0 /* XXX ? */);
		format_bytes(buf, sizeof(buf), (uint64_t)vd_sc->vd_sc_dksc.vd_size *
		    pdg->pdg_secsize);
		printf("%s: %s, %d bytes/sect x %llu sectors\n",
		    vd_sc->vd_sc_dev.dv_xname, buf, (int)pdg->pdg_secsize,
		    (unsigned long long)vd_sc->vd_dksc.vd_sc_size);
		/* Discover wedges on this disk. */
		dkwedge_discover(&vd_sc->vd_dksc.vd_dkdev);
#endif

		/* the disk should be working now */
		config_pending_decr();
		break;
	default:
		panic("bad backend state %d", new_state);
	}
}


void
xbc_connect(struct xbc_vdisk_softc *vd_sc)
{
	int err;

	err = xenbus_read_ul(NULL,
	    vd_sc->vd_sc_xbusd->xbusd_path, "virtual-device", &vd_sc->vd_sc_handle, 10);
	if (err)
	panic("%s: can't read number from %s/virtual-device\n",
	    vd_sc->vd_sc_dev.dv_xname, vd_sc->vd_sc_xbusd->xbusd_otherend);
	err = xenbus_read_ul(NULL,
	    vd_sc->vd_sc_xbusd->xbusd_otherend, "sectors", &vd_sc->vd_sc_sectors, 10);
	if (err)
		panic("%s: can't read number from %s/sectors\n",
		    vd_sc->vd_sc_dev.dv_xname, vd_sc->vd_sc_xbusd->xbusd_otherend);
	err = xenbus_read_ul(NULL,
	    vd_sc->vd_sc_xbusd->xbusd_otherend, "info", &vd_sc->vd_sc_info, 10);
	if (err)
		panic("%s: can't read number from %s/info\n",
		    vd_sc->vd_sc_dev.dv_xname, vd_sc->vd_sc_xbusd->xbusd_otherend);
	err = xenbus_read_ul(NULL,
	    vd_sc->vd_sc_xbusd->xbusd_otherend, "sector-size", &vd_sc->vd_sc_secsize, 10);
	if (err)
		panic("%s: can't read number from %s/sector-size\n",
		    vd_sc->vd_sc_dev.dv_xname, vd_sc->vd_sc_xbusd->xbusd_otherend);

	xenbus_switch_state(vd_sc->vd_sc_xbusd, NULL, XenbusStateConnected);
}


int
xbc_handler(void *arg)
{
#if 0
	struct xbd_xenbus_softc *sc = arg;
	struct buf *bp;
	RING_IDX resp_prod, i;
	int more_to_do;
	int seg;

	DPRINTF(("xbd_handler(%s)\n", sc->sc_dev.dv_xname));

again:
	resp_prod = sc->sc_ring.sring->rsp_prod;
	x86_lfence(); /* ensure we see replies up to resp_prod */
	for (i = sc->sc_ring.rsp_cons; i != resp_prod; i++) {
		blkif_response_t *rep = RING_GET_RESPONSE(&sc->sc_ring, i);
		struct xbd_req *xbdreq = &sc->sc_reqs[rep->id];
		bp = xbdreq->req_bp;
		DPRINTF(("xbd_handler(%p): b_bcount = %ld\n",
		    bp, (long)bp->b_bcount));
		for (seg = xbdreq->req_nr_segments - 1; seg >= 0; seg--) {
			if (__predict_false(
			    xengnt_status(xbdreq->req_gntref[seg]))) {
				printf("%s: grant still used by backend\n",
				    sc->sc_dev.dv_xname);
				sc->sc_ring.rsp_cons = i;
				xbdreq->req_nr_segments = seg + 1;
				return 1;
			}
			xengnt_revoke_access(
			    xbdreq->req_gntref[seg]);
			xbdreq->req_nr_segments--;
		}
		if (rep->operation != BLKIF_OP_READ &&
		    rep->operation != BLKIF_OP_WRITE) {
			printf("%s: bad operation %d from backend\n",
			     sc->sc_dev.dv_xname, rep->operation);
				bp->b_flags |= B_ERROR;
				bp->b_error = EIO;
				bp->b_resid = bp->b_bcount;
				goto next;
		}
		if (rep->status != BLKIF_RSP_OKAY) {
				bp->b_flags |= B_ERROR;
				bp->b_error = EIO;
				bp->b_resid = bp->b_bcount;
				goto next;
		}
		/* b_resid was set in xbdstart */
next:
		if (bp->b_data != xbdreq->req_data)
			xbd_unmap_align(xbdreq);
		disk_unbusy(&sc->sc_dksc.sc_dkdev,
		    (bp->b_bcount - bp->b_resid),
		    (bp->b_flags & B_READ));
		biodone(bp);
		dk_iodone(sc->sc_di, &sc->sc_dksc);
		SLIST_INSERT_HEAD(&sc->sc_xbdreq_head, xbdreq, req_next);
	}
	x86_lfence();
	sc->sc_ring.rsp_cons = i;
	RING_FINAL_CHECK_FOR_RESPONSES(&sc->sc_ring, more_to_do);
	if (more_to_do)
		goto again;
	return 1;
#endif
	return 1;
}



uint8_t xbc_vdisk_used(void)
{
	return controller_sc->nr_vbds;
}

int xbc_vdisk_init(struct xbc_vdisk_softc *vd_sc,
		struct xenbusdev_attach_args *xa)
{
	RING_IDX i;

	vd_sc->vd_sc_xbusd = xa->xa_xbusd;
	vd_sc->vd_sc_xbusd->xbusd_otherend_changed = xbc_backend_changed;
	vd_sc->vd_sc_dev = vd_sc->vd_sc_dev;
	vd_sc->sc_type = XBC_BLK_DEVICE;

	/* initialize free requests list */
	SLIST_INIT(&vd_sc->vd_sc_xbcreq_head);
	for (i = 0; i < XBC_RING_SIZE; i++) {
		vd_sc->vd_sc_reqs[i].req_id = i;
		SLIST_INSERT_HEAD(&vd_sc->vd_sc_xbcreq_head, &vd_sc->vd_sc_reqs[i],
		    req_next);
	}

	vd_sc->vd_sc_backend_status = BLKIF_STATE_DISCONNECTED;
	vd_sc->vd_sc_shutdown = 1;
	controller_sc->nr_vbds++;

	return 0;
}

int xbc_vdisk_destroy(struct xbc_vdisk_softc *vd_sc)
{
	/* XXX Free SLISTS first */

	return 0;
}




#if 0 /* global disable */

/*
 * Execute a [polled] command.
 */
int
xbc_cmd(struct xbc_xenbus_softc *sc, int command, void *data,
	int datasize, struct vdisk_t *vd, int blkno, int flags,
	struct scsi_xfer *xs)
{
	struct buf *bp;
	int rv;

	DPRINTF(/* XBCB_IO, */ ("xbc_cmd: op=%x, xen dev=%i blk=%i data=%p[%x] xs=%p\n",
		command, /* vd->device */, blkno, data, datasize, xs));


	bp = xs->bp;
	if (bp == NULL)
		panic("No buf available");

	DPRINTF(/* XBCB_IO, */ ("xbc_cmd: bp: flags=%d, bufsize=%d, bcount=%d,"
		" error=%d, dev=%d, blkno=%d, data=%p\n",
			bp->b_flags, bp->b_bufsize, bp->b_bcount,
			bp->b_error, bp->b_dev, bp->b_blkno, bp->b_data));

	rv = xbcstart(sc, bp);

	bp->b_resid = bp->b_bcount;

	if (!xs || xs->flags & SCSI_POLL) {
		/* Synchronous commands mustn't wait. */
		DPRINTF(/* XBCB_IO, */
			("xbc_cmd: Synchronous commands mustn't wait.\n"));
		xbc_iodone(sc);
	}

	DPRINTF(/* XBCB_IO, */ ("xbc_cmd: leave with %i\n", rv));
	return rv;
}

#endif

int
xbc_scsi_cmd(struct scsi_xfer *xs)
{
#if 0
	struct scsi_link *link = xs->sc_link;
	struct xbc_xenbus_softc *sc = link->adapter_softc;
	struct scsi_inquiry_data inq;
	struct scsi_sense_data sd;
	struct scsi_read_cap_data rcd;
	uint8_t target = link->target;
	uint32_t blockno, blockcnt, size;
	struct scsi_rw *rw;
	struct scsi_rw_big *rwb;
	int op, flags, s, poll, error;
	struct vdisk_t *vd = get_vdisk(target);

	if (target >= nr_vbds || link->lun != 0) {
		xs->error = XS_DRIVER_STUFFUP;
		return (COMPLETE);
	}

	s = splbio();
	xs->error = XS_NOERROR;
	xs->free_list.le_next = NULL;

	switch (xs->cmd->opcode) {

	/*
	 * Common SCSI opcodes
	 */

	case TEST_UNIT_READY:
		DPRINTF(/* XBCB_SETUP, */ ("scsi cmd TEST_UNIT_READY tgt %d\n",
			target));
		break;
	case START_STOP:
		DPRINTF(/* XBCB_SETUP, */ ("scsi cmd START_STOP tgt %d\n",
			target));
		break;
#if 0
	case VERIFY:
#endif
		DPRINTF(/* XBCB_SETUP, */ ("scsi cmd VERIFY tgt %d\n", target));
		break;

	case MODE_SENSE:
		DPRINTF(/* XBCB_SETUP, */ ("scsi cmd MODE_SENSE tgt %d\n", target));
		break;

	case MODE_SENSE_BIG:
		DPRINTF(/* XBCB_SETUP, */ ("scsi cmd MODE_SENSE_BIG tgt %d\n",
			target));
		break;

	case REQUEST_SENSE:
		DPRINTF(/* XBCB_SETUP, */ ("scsi cmd REQUEST_SENSE tgt %d\n",
			target));
		bzero(&sd, sizeof(sd));
		sd.error_code = 0x70;
		sd.segment = 0;
		sd.flags = SKEY_NO_SENSE;
		*(uint32_t *)sd.info = htole32(0);
		sd.extra_len = 0;
		xbc_copy_internal_data(xs, &sd, sizeof(sd));
		break;

	case INQUIRY:
		DPRINTF(/* XBCB_SETUP, */ ("scsi cmd INCQUIRY tgt %d\n", target));
		DPRINTF(/* XBCB_SETUP, */ ("vdisk type: "));
		bzero(&inq, sizeof(inq));
		switch (VDISK_TYPE(vd->info)) {
		case VDISK_CDROM:
			inq.device = T_CDROM;
			DPRINTF(/* XBCB_SETUP, */ ("cdrom, "));
			break;
		case VDISK_TYPE_DISK:
		default:
			inq.device = T_DIRECT;
			DPRINTF(/* XBCB_SETUP, */ ("disk: %x, \n",
				VDISK_TYPE(vd->info)));
			break;
		} 
		DPRINTF(/* XBCB_SETUP, */ ("%s, ",
			VDISK_READONLY(vd->info) ? "readonly" : "writable"));
		DPRINTF(/* XBCB_SETUP, */ ("%s drive\n",
			VDISK_VIRTUAL(vd->info) ? "virtual" : "real"));
		inq.dev_qual2 = 0;
		inq.version = SID_ANSII_SCSI2;
		inq.response_format = 2;
		inq.additional_length = 32;
		strlcpy(inq.vendor, "Xen ", sizeof(inq.vendor));
		snprintf(inq.product, sizeof(inq.product), "blk %s dev #%02u",
			VDISK_READONLY(vd->info) ? "ro" : "rw", target);
		strlcpy(inq.revision, "1.0", sizeof(inq.revision));
		xbc_copy_internal_data(xs, &inq, sizeof(inq));
		break;

	case PREVENT_ALLOW:
		DPRINTF(/* XBCB_SETUP, */ ("scsi cmd PREVENT_ALLOW tgt %d\n",
			target));
		goto notyet;

	/*
	 * SCSI disk specific opcodes
	 */

	case READ_CAPACITY:
		DPRINTF(/* XBCB_SETUP, */ ("scsi cmd READ_CAPACITY tgt %d\n",
			target));
		bzero(&rcd, sizeof(rcd));
		_lto4b(vd->capacity - 1, rcd.addr);	/* disk size */
		_lto4b(DEV_BSIZE, rcd.length);		/* sector size */
		xbc_copy_internal_data(xs, &rcd, sizeof(rcd));
		break;

	case SYNCHRONIZE_CACHE:
		DPRINTF(/* XBCB_IO, */ ("scsi cmd SYNCHRONIZE_CACHE tgt %d\n",
			target));
		xbc_iodone(sc);
		break;

	case READ_COMMAND:
	case READ_BIG:
	case WRITE_COMMAND:
	case WRITE_BIG:

		/* These commands are called from within interrupt
		 * context and must call scsi_done(xs); when transfer
		 * done
		 */

		flags = 0;
		switch (xs->cmd->opcode) {
		case READ_COMMAND:
			DPRINTF(/* XBCB_IO, */ ("scsi cmd READ_COMMAND tgt %d\n",
				target));
			op = BLKIF_OP_READ;
			flags = B_READ;
			break;
		case READ_BIG:
			DPRINTF(/* XBCB_IO, */ ("scsi cmd READ_BIG tgt %d\n",
				target));
			op = BLKIF_OP_READ;
			flags = B_READ;
			break;
		case WRITE_COMMAND:
			DPRINTF(/* XBCB_IO, */ ("scsi cmd WRITE_COMMAND tgt %d\n",
				target));
			op = BLKIF_OP_WRITE;
			flags = B_WRITE;
			if (VDISK_READONLY(vd->info)) {
				printf("Write operation failed. Device is readonly\n");
				splx(s);
				xs->error = XS_DRIVER_STUFFUP;
				scsi_done(xs);
				return (COMPLETE);
			}
			break;
		case WRITE_BIG:
			DPRINTF(/* XBCB_IO, */ ("scsi cmd WRITE_BIG tgt %d\n",
				target));
			op = BLKIF_OP_WRITE;
			flags = B_WRITE;
			if (VDISK_READONLY(vd->info)) {
				printf("Write operation failed. Device is readonly\n");
				splx(s);
				xs->error = XS_DRIVER_STUFFUP;
				scsi_done(xs);
				return (COMPLETE);
			}
			break;
		}

		if (xs->cmdlen == 6) {
			rw = (struct scsi_rw *)xs->cmd;
			blockno = _3btol(rw->addr) &
				(SRW_TOPADDR << 16 | 0xffff);
			blockcnt = rw->length ? rw->length : 0x100;
		} else {
			rwb = (struct scsi_rw_big *)xs->cmd;
			blockno = _4btol(rwb->addr);
			blockcnt = _2btol(rwb->length);
		}

		size = vd->capacity;
		if (blockno >= size || blockno + blockcnt > size) {
			printf("%s: out of bounds %u-%u >= %u\n",
				sc->sc_dev.dv_xname, blockno, blockcnt, size);
			xs->error = XS_DRIVER_STUFFUP;
			scsi_done(xs);
			break;
		}

		poll = xs->flags & SCSI_POLL;
		sc->sc_xs = xs;

		error = xbc_cmd(sc, op, xs->data, blockcnt * DEV_BSIZE,
				vd, blockno, flags, xs);
		if (error) {
			if (error == ENOMEM) {
				splx(s);
				return (TRY_AGAIN_LATER);
			} else if (poll) {
				splx(s);
				return (TRY_AGAIN_LATER);
			} else {
				xs->error = XS_DRIVER_STUFFUP;
				scsi_done(xs);
				break;
			}
		}

		splx(s);
		if (poll) {
			DPRINTF(/* XBCB_IO, */
				("xbc_scsi_cmd: (polling) COMPLETE\n"));
			return (COMPLETE);
		} else {
			DPRINTF(/* XBCB_IO, */
				("xbc_scsi_cmd: SUCCESSFULLY_QUEUED\n"));
			return (SUCCESSFULLY_QUEUED);
		}

	default:
		printf("unsupported scsi command %#x tgt %d\n",
			xs->cmd->opcode, target);
		xs->error = XS_DRIVER_STUFFUP;
		break;
	}
	splx(s);

	return (COMPLETE);

notyet:
	printf("unimplemented scsi command %#x tgt %d\n",
		xs->cmd->opcode, target);
	xs->error = XS_DRIVER_STUFFUP;

	splx(s);
#endif
	return (COMPLETE);
}



void
xbc_copy_internal_data(struct scsi_xfer *xs, void *v, size_t size)
{
	size_t copy_cnt;

	if (!xs->datalen) {
		printf("uio move is not yet supported\n");
	} else {
		copy_cnt = MIN(size, xs->datalen);
		bcopy(v, xs->data, copy_cnt);
	}
}

#if 0 /* global disable */

static int
get_vbd_info(vdisk_t *disk_info)
{
	vdisk_t *buf;
	int nr;
	blkif_request_t req;
	blkif_response_t rsp;
	paddr_t pa;

	buf = (vdisk_t *)uvm_km_kmemalloc1(kmem_map, NULL,
		PAGE_SIZE, PAGE_SIZE, UVM_UNKNOWN_OFFSET, 0);
	pmap_extract(pmap_kernel(), (vaddr_t)buf, &pa);
	/* Probe for disk information. */
	memset(&req, 0, sizeof(req));
	req.operation = BLKIF_OP_PROBE;
	req.nr_segments = 1;
	req.frame_and_sects[0] = xpmap_ptom_masked(pa) | 7;

	control_send(&req, &rsp);
	nr = rsp.status > MAX_VBDS ? MAX_VBDS : rsp.status;

	if (rsp.status < 0)
		printf("WARNING: Could not probe disks (%d)\n", rsp.status);

	memcpy(disk_info, buf, nr * sizeof(vdisk_t));

	uvm_km_free(kmem_map, (vaddr_t)buf, PAGE_SIZE);

	return nr;
}


vdisk_t *
get_vdisk(uint8_t target)
{
	if (target >= nr_vbds)
		return NULL;

	return &vbd_info[target];
}

extern int hypervisor_print(void *, const char *);

void
vbd_update(void)
{
	struct xenbusdev_attach_args *xbda;
#ifdef todo
	struct device *dev;
#endif
	vdisk_t *vd;
	vdisk_t *vbd_info_update, *vbd_info_old;
	int i, j, new_nr_vbds;

	MALLOC(vbd_info_update, vdisk_t *, MAX_VBDS *
	    sizeof(vdisk_t), M_DEVBUF, M_WAITOK);
	bzero(vbd_info_update, sizeof(vdisk_t));

	new_nr_vbds  = get_vbd_info(vbd_info_update);

	if (memcmp(vbd_info, vbd_info_update, MAX_VBDS *
	    sizeof(vdisk_t)) == 0) {
		FREE(vbd_info_update, M_DEVBUF);
		return;
	}

	for (i = 0, j = 0; i < nr_vbds && j < new_nr_vbds;)  {
		if (vbd_info[i].device > vbd_info_update[j].device) {
			DPRINTF(/* XBCB_HOTPLUG, */
			    ("delete device %x size %lx\n",
				vbd_info[i].device,
				(u_long)vbd_info[i].capacity));
#if todo
			vd = &vbd_info[i];
			dev = find_device(vd);
			if (dev)
				config_detach(dev, DETACH_FORCE);
#endif
			i++;
		} else if (vbd_info_update[j].device > vbd_info[i].device) {
			DPRINTF(/* XBCB_HOTPLUG, */
			    ("add device %x size %lx\n",
				vbd_info_update[j].device,
				(u_long)vbd_info_update[j].capacity));
			vd = &vbd_info_update[j];
			xbda = blkctrl.mainbus_xbca;
			if (xbda) {
				xbda->xa_xd = vd;
				config_found_sm(blkctrl.xc_parent, xbda,
				    blkctrl.xc_cfprint, blkctrl.xc_cfmatch);
			}
			j++;
		} else {
			i++; j++;
			DPRINTF(/* XBCB_HOTPLUG, */
			    ("update device %x size %lx size %lx\n",
				vbd_info_update[i].device,
				(u_long)vbd_info[j].capacity,
				(u_long)vbd_info_update[i].capacity));
		}
	}
#ifdef todo
	for (; i < nr_vbds; i++) {
		DPRINTF(/* XBCB_HOTPLUG, */
		    ("delete device %x size %lx\n",
			vbd_info[i].device,
			(u_long)vbd_info[i].capacity));
		vd = &vbd_info[i];
		dev = find_device(vd);
		if (dev)
			config_detach(dev, DETACH_FORCE);
	}
#endif
	for (; j < new_nr_vbds; j++) {
		DPRINTF(/* XBCB_HOTPLUG, */
		    ("add device %x size %lx\n",
			vbd_info_update[j].device,
			(u_long)vbd_info_update[j].capacity));
		vd = &vbd_info_update[j];
		xbda = blkctrl.mainbus_xbca;
		if (xbda) {
			xbda->xa_xd = vd;
			config_found_sm(blkctrl.xc_parent, xbda,
			    blkctrl.xc_cfprint, blkctrl.xc_cfmatch);
		}
	}

	nr_vbds = new_nr_vbds;

	vbd_info_old = vbd_info;
	vbd_info = vbd_info_update;
	FREE(vbd_info_old, M_DEVBUF);
}

int
xbc_map_align(struct xbc_req *req)
{
	int s;

	s = splvm();
	req->req_aligned = uvm_km_kmemalloc1(kmem_map, NULL,
		req->req_bqueue, XEN_BSIZE, UVM_UNKNOWN_OFFSET, UVM_KMF_NOWAIT);
	splx(s);
	if (req->req_aligned == 0)
		return 0;
	DPRINTF(/* XBCB_IO, */ ("map_align(%p): bp %p addr %p align 0x%08lx "
	    "size 0x%04lx\n", req, req->xr_bp, req->req_bp->b_data,
	    req->req_aligned, req->req_bqueue));
	req->req_data = req->req_aligned;
	if ((req->req_bp->b_flags & B_READ) == 0)
		memcpy((void *)req->req_aligned, req->req_bp->b_data,
		    req->req_bqueue);
	return 1;
}

void
xbc_unmap_align(struct xbc_req *req)
{
	int s;

	if (req->req_bp->b_flags & B_READ)
		memcpy(req->req_bp->b_data, (void *)req->req_aligned,
		    req->req_bp->b_bcount);
	DPRINTF(/* XBCB_IO, */ ("unmap_align(%p): bp %p addr %p align 0x%08lx "
	    "size 0x%04x\n", req, req->req_bp, req->req_bp->b_data,
	    req->req_aligned, req->req_bp->b_bcount));
	s = splvm();
	uvm_km_free(kmem_map, req->req_aligned, req->req_bp->b_bcount);
	splx(s);
	xr->xr_aligned = (vaddr_t)0;
}


static void
fill_ring(struct xbcreq *xr)
{
	struct xbcreq *pxr = xr->xr_parent;
	paddr_t pa;
	unsigned long ma;
	vaddr_t addr, off;
	blkif_request_t *ring_req;
	int breq, nr_sectors, fsect, lsect;

	KASSERT(pxr->xr_bqueue > 0);
	KASSERT(xr != NULL);

	/* Fill out a communications ring structure. */
	ring_req = &blk_ring->ring[MASK_BLKIF_IDX(req_prod)].req;
	bzero(&blk_ring->ring[MASK_BLKIF_IDX(req_prod)], sizeof(blkif_ring_t));
	ring_req->id = (unsigned long)xr;
	ring_req->operation = pxr->xr_bp->b_flags & B_READ ? BLKIF_OP_READ :
		BLKIF_OP_WRITE;
	ring_req->sector_number = pxr->xr_bn;
	ring_req->device = pxr->xr_vdisk->device;

	DPRINTF(/* XBCB_IO, */ ("fill_ring(%d): bp %p sector %llu pxr %p xr %p\n",
		    MASK_BLKIF_IDX(req_prod), pxr->xr_bp,
		    (unsigned long long)pxr->xr_bn,
		    pxr, xr));

	xr->xr_breq = 0;
	ring_req->nr_segments = 0;
	addr = trunc_page(pxr->xr_data);
	off = pxr->xr_data - addr;
	while (pxr->xr_bqueue > 0) {
#if 0
		pmap_extract(vm_map_pmap(&bp->b_proc->p_vmspace->vm_map),
		    addr, &pa);
#else
		pmap_extract(pmap_kernel(), addr, &pa);
#endif
		ma = xpmap_ptom_masked(pa);
		DIAGCONDPANIC((ma & (XEN_BSIZE - 1)) != 0,
		    ("xbc request ma not sector aligned"));

		if (pxr->xr_bqueue > PAGE_SIZE - off)
			breq = PAGE_SIZE - off;
		else
			breq = pxr->xr_bqueue;

		nr_sectors = breq >> XEN_BSHIFT;
		DIAGCONDPANIC(nr_sectors >= XEN_BSIZE,
		    ("xbd request nr_sectors >= XEN_BSIZE"));

		fsect = off >> XEN_BSHIFT;
		lsect = fsect + nr_sectors - 1;
		DIAGCONDPANIC(fsect > 7, ("xbd request fsect > 7"));
		DIAGCONDPANIC(lsect > 7, ("xbd request lsect > 7"));

		DPRINTF(/* XBCB_IO, */ ("fill_ring(%d): va 0x%08lx pa 0x%08lx "
		    "ma 0x%08lx, sectors %d, left %ld/%ld\n",
		    MASK_BLKIF_IDX(req_prod), addr, pa, ma, nr_sectors,
		    pxr->xr_bqueue >> XEN_BSHIFT, pxr->xr_bqueue));

		ring_req->frame_and_sects[ring_req->nr_segments++] =
			ma | (fsect<<3) | lsect;
		addr += PAGE_SIZE;
		pxr->xr_bqueue -= breq;
		pxr->xr_bn += nr_sectors;
		xr->xr_breq += breq;
		off = 0;
		if (ring_req->nr_segments == BLKIF_MAX_SEGMENTS_PER_REQUEST)
			break;
	}
	KASSERT(ring_req->nr_segments > 0);
	pxr->xr_data = addr;

	req_prod++;
}

static void
xbcresume(void)
{
	struct xbcreq *pxr, *xr;
	struct xbc_xenbus_softc *sc;
	struct buf *bp;
	struct scsi_xfer *xs;

	while ((pxr = SIMPLEQ_FIRST(&xbdr_suspended)) != NULL) {
		DPRINTF(/* XBCB_IO, */ ("xbcresume: resuming xbcreq %p for bp %p\n",
		    pxr, pxr->xr_bp));
		bp = pxr->xr_bp;
		sc = pxr->xr_sc;
		xs = pxr->xr_xs;
		if (bp->b_flags & B_ERROR) {
			xs->error = XS_DRIVER_STUFFUP;
			pxr->xr_bdone -= pxr->xr_bqueue;
			pxr->xr_bqueue = 0;
			if (pxr->xr_bdone == 0) {
				bp->b_resid = bp->b_bcount;
				if (pxr->xr_aligned)
					unmap_align(pxr);
				PUT_XBCREQ(pxr);
				if (xs) {
#if NRND > 0
					rnd_add_uint32(&xs->sc_rnd_source,
					    bp->b_blkno);
#endif
				}
				xs->flags |= ITSDONE;
				scsi_done(xs);
			}
			continue;
		}
		while (__predict_true(pxr->xr_bqueue > 0)) {
			GET_XBCREQ(xr);
			if (__predict_false(xr == NULL))
				goto out;
			xr->xr_parent = pxr;
			fill_ring(xr);
		}
		DPRINTF(/* XBCB_IO, */ ("xbcresume: resumed xbcreq %p for bp %p\n",
		    pxr, bp));
		SIMPLEQ_REMOVE_HEAD(&xbdr_suspended, xr_suspended);
	}

 out:
	return;
}

void
signal_requests_to_xen(void)
{

	DPRINTF(/* XBCB_IO, */ ("signal_requests_to_xen: %x -> %x\n",
		    blk_ring->req_prod, req_prod));
	blk_ring->req_prod = req_prod;
	last_req_prod = req_prod;

	hypervisor_notify_via_evtchn(blkif_evtchn);
	return;
}


int
xbcstart(struct xbc_xenbus_softc *sc, struct buf *bp)
{
	struct xbcreq *pxr, *xr;
	daddr_t	bn;
	int ret, runqueue;
	vdisk_t *vd;
	struct scsi_xfer *xs;

	DPRINTF_FOLLOW(("xbcstart(%p)\n", bp));

	runqueue = 1;
	ret = -1;
	xs = sc->sc_xs;

	if (sc == NULL || sc->sc_shutdown) {
		xs->error = XS_DRIVER_STUFFUP;
		xs->flags |= ITSDONE;
		scsi_done(xs);
		return 0;
	}

	bn = bp->b_blkno;
	vd = get_vdisk(sc->sc_link.target);

	DPRINTF(/* XBCB_IO, */ ("xbcstart: addr %p, sector %llu, "
	    "count %d [%s]\n", bp->b_data, (unsigned long long)bn,
	    bp->b_bcount, bp->b_flags & B_READ ? "read" : "write"));

	GET_XBCREQ(pxr);
	if (__predict_false(pxr == NULL))
		goto out;

	/*
	 * We have a request slot, return 0 to make dk_start remove
	 * the bp from the work queue.
	 */
	ret = 0;

	pxr->xr_bp = bp;
	pxr->xr_parent = pxr;
	pxr->xr_bn = bn;
	pxr->xr_bqueue = bp->b_bcount;
	pxr->xr_bdone = bp->b_bcount;
	pxr->xr_data = (vaddr_t)bp->b_data;
	pxr->xr_sc = sc;
	pxr->xr_vdisk = vd;
	pxr->xr_xs = xs;

	if (pxr->xr_data & (XEN_BSIZE - 1)) {
		if (!map_align(pxr)) { /* No memory; try later. */
			DPRINTF(/* XBCB_IO, */ ("xbdstart: map_align failed\n"));
			ret = -1;
			PUT_XBCREQ(pxr);
			goto out;
		}
	}

	fill_ring(pxr);

	while (__predict_false(pxr->xr_bqueue > 0)) {
		GET_XBCREQ(xr);
		if (__predict_false(xr == NULL))
			break;
		xr->xr_parent = pxr;
		fill_ring(xr);
	}

	if (__predict_false(pxr->xr_bqueue > 0)) {
		SIMPLEQ_INSERT_TAIL(&xbdr_suspended, pxr,
		    xr_suspended);
		DPRINTF(/* XBCB_IO, */ ("xbcstart: suspended xbcreq %p "
		    "for bp %p\n", pxr, bp));
	} else if (CANGET_XBCREQ() && BUFQ_GET(bufq) != NULL) {
		/*
		 * We have enough resources to start another bp and
		 * there are additional bps on the queue, dk_start
		 * will call us again and we'll run the queue then.
		 */
		runqueue = 0;
	}

 out:
	if (runqueue && last_req_prod != req_prod)
		signal_requests_to_xen();

	return ret;
}

void
xbc_iodone(struct xbc_xenbus_softc *xs)
{
	struct buf *bp;

	DPRINTF_FOLLOW(("xbc_iodone(%p)\n", xs));

	/* Process the work queue */
	while ((bp = BUFQ_GET(bufq)) != NULL) {
		printf("xbc_iodone: processing bp = %p\n", bp);
		if (xbcstart(xs, bp) != 0) {
			DPRINTF(/* XBCB_IO, */ ("xbc_iodone: re-add  bp = %p and abort\n",
				bp));
			BUFQ_ADD(bufq, bp);
			break;
		}
	}
}


int
xbc_response_handler(void *arg)
{
	struct buf *bp;
	struct xbc_xenbus_softc *sc;
	struct scsi_xfer *xs;
	blkif_response_t *ring_resp;
	struct xbcreq *pxr, *xr;
	BLKIF_RING_IDX i, rp;

	rp = blk_ring->resp_prod;
	x86_lfence(); /* Ensure we see queued responses up to 'rp'. */

	for (i = resp_cons; i != rp; i++) {
		ring_resp = &blk_ring->ring[MASK_BLKIF_IDX(i)].resp;
		xr = (struct xbcreq *)ring_resp->id;
		KASSERT(xr != NULL);

		switch (ring_resp->operation) {
		case BLKIF_OP_READ:
		case BLKIF_OP_WRITE:
			pxr = xr->xr_parent;

			DPRINTF(/* XBCB_IO, */ ("xbc_response_handler(%d): pxr %p "
				    "xr %p bdone %04lx breq %04lx\n", i, pxr,
				    xr, pxr->xr_bdone, xr->xr_breq));
			pxr->xr_bdone -= xr->xr_breq;
			DIAGCONDPANIC(pxr->xr_bdone < 0,
			    ("xbc_response_handler: pxr->xr_bdone < 0"));

			if (__predict_false(ring_resp->status)) {
				DPRINTF(/* XBCB_IO, */ ("xbc_response_handler(%d): Operation failed, Xen return code: %i\n",
					i, (int)ring_resp->status));
				pxr->xr_bp->b_flags |= B_ERROR;
				pxr->xr_bp->b_error = EIO;
			}

			if (xr != pxr) {
				PUT_XBCREQ(xr);
				if (!SIMPLEQ_EMPTY(&xbdr_suspended))
					xbcresume();
			}

			if (pxr->xr_bdone == 0) {
				bp = pxr->xr_bp;
				sc = pxr->xr_sc;
				xs = pxr->xr_xs;
				DPRINTF(/* XBCB_IO, */ ("xbc_response_handler(%d): "
					    "completed bp %p\n", i, bp));
				if (bp->b_flags & B_ERROR) {
					bp->b_resid = bp->b_bcount;
					xs->error = XS_DRIVER_STUFFUP;
				} else {
					bp->b_resid = 0;
					xs->resid = 0;
				}

				if (pxr->xr_aligned) {
					unmap_align(pxr);
				}

				PUT_XBCREQ(pxr);
				if (sc) {
#if NRND > 0
					rnd_add_uint32(&sc->sc_rnd_source,
					    bp->b_blkno);
#endif
				}
				xs->flags |= ITSDONE;
				scsi_done(xs);
				if (!SIMPLEQ_EMPTY(&xbdr_suspended))
					xbcresume();
				/* XXX possible lockup if this was the only
				 * active device and requests were held back in
				 * the queue.
				 */
				if (sc) {
					xbc_iodone(sc);
				}
			}
			break;
		case BLKIF_OP_PROBE:
			PUT_XBCREQ(xr);
			memcpy(&blkif_control_rsp, ring_resp,
			    sizeof(*ring_resp));
			blkif_control_rsp_valid = 1;
			wakeup((caddr_t)&blkif_control_rsp_valid);
			break;
		default:
			panic("unknown response");
		}
	}
	resp_cons = i;
	/* check if xbcresume queued any requests */
	if (last_req_prod != req_prod)
		signal_requests_to_xen();
	return 0;
}

static void
connect_interface(blkif_fe_interface_status_t *status)
{
	// unsigned long flags;
	struct xenbusdev_attach_args *xbda;
	vdisk_t *vd;
	int i;

	blkif_evtchn = status->evtchn;

	printf("xbc: using event channel %d\n", blkif_evtchn);

	event_set_handler(blkif_evtchn, &xbc_response_handler, NULL, IPL_BIO,
	    "xbc");
	hypervisor_enable_event(blkif_evtchn);

	/* Transition to connected in case we need to do
	 *  a partition probe on a whole disk. */
	state = STATE_CONNECTED;

	/* Probe for discs attached to the interface. */
	// xlvbd_init();
	MALLOC(vbd_info, vdisk_t *, MAX_VBDS * sizeof(vdisk_t),
	    M_DEVBUF, M_WAITOK);
	memset(vbd_info, 0, MAX_VBDS * sizeof(vdisk_t));
	nr_vbds  = get_vbd_info(vbd_info);

	if (in_autoconf) {
		/* call xbc_attach() via config_attach() here,
		 * we must attach xbc0 at hypervisor0 before we can
		 * attach sd* - even when nr_vbds <= 0 just to see at least
		 * "xbc0 at hypervisor0: Xen Virtual Block Controller"
		 * in dmesg
		 */
		config_attach(blkctrl.xc_parent, blkctrl.match, xbda,
				blkctrl.xc_cfprint);
		if (nr_vbds > 0) {
			in_autoconf = 0;
			config_pending_decr();
			return;
		}
	}

	if (nr_vbds <= 0)
		goto out;

	for (i = 0; i < nr_vbds; i++) {
		vd = &vbd_info[i];
		xbda = blkctrl.mainbus_xbca;
		if (xbda) {
			xbda->xa_xd = vd;
			config_found_sm(blkctrl.xc_parent, xbda,
			    blkctrl.xc_cfprint, blkctrl.xc_cfmatch);
		}
	}

#if 0
	/* Kick pending requests. */
	save_and_cli(flags);
	// simple_lock(&blkif_io_lock);
	kick_pending_request_queues();
	// simple_unlock(&blkif_io_lock);
	restore_flags(flags);
#endif

	return;

 out:
	FREE(vbd_info, M_DEVBUF);
	vbd_info = NULL;
	if (in_autoconf) {
		in_autoconf = 0;
		config_pending_decr();
	}

	return;
}


static void
disconnect_interface(void)
{
	if (blk_ring == NULL)
		blk_ring = (blkif_ring_t *)uvm_km_kmemalloc1(kmem_map, NULL,
			PAGE_SIZE, PAGE_SIZE, UVM_UNKNOWN_OFFSET, 0);
	memset(blk_ring, 0, PAGE_SIZE);
	blk_ring->req_prod = blk_ring->resp_prod = resp_cons = req_prod =
		last_req_prod = 0;
	state = STATE_DISCONNECTED;
	send_interface_connect();
}


static void
free_interface(void)
{

	/* Prevent new requests being issued until we fix things up. */
	// simple_lock(&blkif_io_lock);
	// recovery = 1;
	state = STATE_DISCONNECTED;
	// simple_unlock(&blkif_io_lock);

	/* Free resources associated with old device channel. */
	if (blk_ring) {
		uvm_km_free(kmem_map, (vaddr_t)blk_ring, PAGE_SIZE);
		blk_ring = NULL;
	}

	if (blkif_evtchn) {
		event_remove_handler(blkif_evtchn, &xbc_response_handler, NULL);
		blkif_evtchn = 0;
	}
}


static void
reset_interface(void)
{
	printf("Recovering virtual block device driver\n");
	free_interface();
	disconnect_interface();
}


static void
close_interface(void)
{
}


static void
unexpected(blkif_fe_interface_status_t *status)
{
	printf("Unexpected blkif status %d in state %d\n",
	    status->status, state);
}


static void
blkif_status(blkif_fe_interface_status_t *status)
{
	if (status->handle != blkif_handle) {
		printf("Invalid blkif: handle=%u", status->handle);
		return;
	}

	switch (status->status) {
	case BLKIF_INTERFACE_STATUS_CLOSED:
		switch (state) {
		case STATE_CLOSED:
			unexpected(status);
			break;
		case STATE_DISCONNECTED:
		case STATE_CONNECTED:
			unexpected(status);
			close_interface();
			break;
		}
		break;

	case BLKIF_INTERFACE_STATUS_DISCONNECTED:
		switch (state) {
		case STATE_CLOSED:
			disconnect_interface();
			break;
		case STATE_DISCONNECTED:
		case STATE_CONNECTED:
			unexpected(status);
			reset_interface();
			break;
		}
		break;

	case BLKIF_INTERFACE_STATUS_CONNECTED:
		switch (state) {
		case STATE_CLOSED:
			unexpected(status);
			disconnect_interface();
			connect_interface(status);
			break;
		case STATE_DISCONNECTED:
			connect_interface(status);
			break;
		case STATE_CONNECTED:
			unexpected(status);
			connect_interface(status);
			break;
		}
		break;

	case BLKIF_INTERFACE_STATUS_CHANGED:
		switch (state) {
		case STATE_CLOSED:
		case STATE_DISCONNECTED:
			unexpected(status);
			break;
		case STATE_CONNECTED:
			vbd_update();
			break;
		}
		break;

	default:
		printf(" Invalid blkif status: %d\n", status->status);
		break;
	}
}


int
xbcinit(struct xbc_xenbus_softc *sc, struct xenbusdev_attach_args *xbda)
{
	/*
	 * We have one shared bufq for all devices because otherwise
	 * requests can stall if there were no free request slots
	 * available in xbcstart and this device had no requests
	 * in-flight which would trigger a dk_start from the interrupt
	 * handler.
	 * XXX we reference count the usage in case so we can de-alloc
	 *     the bufq if all devices are deconfigured.
	 */
	if (bufq_users == 0) {
		bufq = BUFQ_ALLOC("fcfs");
		bufq_users = 1;
	}
	sc->sc_flags |= XBC_INITED;

	return 0;
}


void
control_send(blkif_request_t *req, blkif_response_t *rsp)
{
	unsigned long flags;
	struct xbcreq *xr;

 retry:
	while ((req_prod - resp_cons) == BLKIF_RING_SIZE) {
		/* XXX where is the wakeup ? */
		tsleep((caddr_t) &req_prod, PUSER | PCATCH,
		    "blkfront", 0);
	}

	save_and_cli(flags);
	// simple_lock(&blkif_io_lock);
	if ((req_prod - resp_cons) == BLKIF_RING_SIZE) {
		// simple_unlock(&blkif_io_lock);
		restore_flags(flags);
		goto retry;
	}

	blk_ring->ring[MASK_BLKIF_IDX(req_prod)].req = *req;

	GET_XBCREQ(xr);
	blk_ring->ring[MASK_BLKIF_IDX(req_prod)].req.id = (unsigned long)xr;
	// rec_ring[id].id = (unsigned long) req;

	// translate_req_to_pfn( &rec_ring[id], req );

	req_prod++;
	signal_requests_to_xen();

	// simple_unlock(&blkif_io_lock);
	restore_flags(flags);

	while (!blkif_control_rsp_valid) {
		tsleep((caddr_t)&blkif_control_rsp_valid, PUSER | PCATCH,
		    "blkfront", 0);
	}

	memcpy(rsp, &blkif_control_rsp, sizeof(*rsp));
	blkif_control_rsp_valid = 0;
}

#endif
