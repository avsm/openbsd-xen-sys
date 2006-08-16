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

#if NRND > 0
#include <sys/rnd.h>
#endif

#include <scsi/scsi_all.h>
#include <scsi/scsi_disk.h>
#include <scsi/scsiconf.h>

#include <machine/xbcvar.h>

#include <machine/xen.h>
#include <machine/hypervisor.h>
#include <machine/evtchn.h>
#include <machine/ctrl_if.h>

void	control_send(blkif_request_t *, blkif_response_t *);
void	send_interface_connect(void);

int xbc_match(struct device *, void *, void *);
void xbc_attach(struct device *, struct device *, void *);

void xbc_ctrlif_rx(ctrl_msg_t *msg, unsigned long id);
void send_driver_status(int ok);
vdisk_t *get_vdisk(uint8_t target);


int xbc_scsi_cmd(struct scsi_xfer *);
int xbc_cmd(struct xbc_softc *sc, int command, void *data,
		int datasize, vdisk_t *vd, int blkno, int flags,
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
	sizeof(struct xbc_softc),
	xbc_match, xbc_attach,
	NULL,	/* detach */
	NULL	/* activate */
};


int	xbcstart(struct xbc_softc *, struct buf *);
int	xbc_response_handler(void *);
void	xbc_iodone(struct xbc_softc *);

int xbcinit(struct xbc_softc *, struct xbc_attach_args *);
void xbc_copy_internal_data(struct scsi_xfer *xs, void *v, size_t size);
void vbd_update(void);
void signal_requests_to_xen(void);

#if defined(XBDDEBUG) && !defined(DEBUG)
#define DEBUG
#endif

#ifdef DEBUG
#define XBCB_FOLLOW	0x1
#define XBCB_IO		0x2
#define XBCB_SETUP	0x4
#define XBCB_HOTPLUG	0x8

#define IFDEBUG(x,y)		if (xbddebug & (x)) y
#define DPRINTF(x,y)		IFDEBUG(x, printf y)
#define DPRINTF_FOLLOW(y)	DPRINTF(XBCB_FOLLOW, y)

int xbddebug = 0x0;

#define	DEBUG_MARK_UNUSED(_xr)	(_xr)->xr_sc = (void *)0xdeadbeef

struct xbcreq *xbc_allxr;
#else
#define IFDEBUG(x,y)
#define DPRINTF(x,y)
#define DPRINTF_FOLLOW(y)
#define	DEBUG_MARK_UNUSED(_xr)
#endif

#ifdef DIAGNOSTIC
#define DIAGPANIC(x)		panic x
#define DIAGCONDPANIC(x,y)	if (x) panic y
#else
#define DIAGPANIC(x)
#define DIAGCONDPANIC(x,y)
#endif


struct xbcreq {
	union {
		SLIST_ENTRY(xbcreq) _unused;	/* ptr. to next free xbcreq */
		SIMPLEQ_ENTRY(xbcreq) _suspended;
					/* link when on suspended queue. */
	} _link;
	struct xbcreq		*xr_parent;	/* ptr. to parent xbcreq */
	struct buf		*xr_bp;		/* ptr. to original I/O buf */
	daddr_t			xr_bn;		/* block no. to process */
	long			xr_bqueue;	/* bytes left to queue */
	long			xr_bdone;	/* bytes left */
	vaddr_t			xr_data;	/* ptr. to data to be proc. */
	vaddr_t			xr_aligned;	/* ptr. to aligned data */
	long			xr_breq;	/* bytes in this req. */
	struct xbc_softc	*xr_sc;		/* ptr. to xbc softc */
	vdisk_t			*xr_vdisk;	/* ptr. to vdisk */
	struct scsi_xfer	*xr_xs;		/* ptr. to scsi xfer */
};
#define	xr_unused	_link._unused
#define	xr_suspended	_link._suspended

SLIST_HEAD(,xbcreq) xbcreqs =
	SLIST_HEAD_INITIALIZER(xbcreqs);
static SIMPLEQ_HEAD(, xbcreq) xbdr_suspended =
	SIMPLEQ_HEAD_INITIALIZER(xbdr_suspended);

#define	CANGET_XBCREQ() (!SLIST_EMPTY(&xbcreqs))

#define	GET_XBCREQ(_xr) do {				\
	(_xr) = SLIST_FIRST(&xbcreqs);			\
	if (__predict_true(_xr))			\
		SLIST_REMOVE_HEAD(&xbcreqs, xr_unused);	\
} while (/*CONSTCOND*/0)

#define	PUT_XBCREQ(_xr) do {				\
	DEBUG_MARK_UNUSED(_xr);				\
	SLIST_INSERT_HEAD(&xbcreqs, _xr, xr_unused);	\
} while (/*CONSTCOND*/0)

static struct bufq *bufq;
static int bufq_users = 0;

#define	XEN_BSHIFT	9		/* log2(XEN_BSIZE) */
#define	XEN_BSIZE	(1 << XEN_BSHIFT)

#define MAX_VBDS 64
static int nr_vbds;
static vdisk_t *vbd_info;

static blkif_ring_t *blk_ring = NULL;
static BLKIF_RING_IDX resp_cons; /* Response consumer for comms ring. */
static BLKIF_RING_IDX req_prod;  /* Private request producer.         */
static BLKIF_RING_IDX last_req_prod;  /* Request producer at last trap. */

#define STATE_CLOSED		0
#define STATE_DISCONNECTED	1
#define STATE_CONNECTED		2
static unsigned int state = STATE_CLOSED;

static int in_autoconf; /* we are still in autoconf ? */
static unsigned int blkif_evtchn = 0;
static unsigned int blkif_handle = 0;

static int blkif_control_rsp_valid = 0;
static blkif_response_t blkif_control_rsp;

/* block device interface info. */
struct xbd_ctrl {

	cfprint_t xc_cfprint;
	struct device *xc_parent;
	cfmatch_t xc_cfmatch;
	void *match;
	struct xbc_attach_args *mainbus_xbca;
};

static struct xbd_ctrl blkctrl;



int
xbc_scan(struct device *self, struct xbc_attach_args *mainbus_xbca,
    cfprint_t print)
{
	struct xbcreq *xr;
	int i;

	printf("Initializing Xen SCSI block controller frontend driver.\n");

	blkctrl.xc_parent = self;
	blkctrl.xc_cfprint = print;
	blkctrl.mainbus_xbca = mainbus_xbca;
	blkctrl.match = config_search(NULL, blkctrl.xc_parent, mainbus_xbca);

	MALLOC(xr, struct xbcreq *, BLKIF_RING_SIZE * sizeof(struct xbcreq),
	    M_DEVBUF, M_WAITOK);
	bzero(xr, BLKIF_RING_SIZE * sizeof(struct xbcreq));
#ifdef DEBUG
	xbc_allxr = xr;
#endif
	for (i = 0; i < BLKIF_RING_SIZE - 1; i++)
		PUT_XBCREQ(&xr[i]);

	(void)ctrl_if_register_receiver(CMSG_BLKIF_FE, xbc_ctrlif_rx,
	    CALLBACK_IN_BLOCKING_CONTEXT);

	in_autoconf = 1;
	config_pending_incr();

	send_driver_status(1);

	return 0;
}



int
xbc_match(struct device *parent, void *match, void *aux)
{
	struct xbc_attach_args *xa = (struct xbc_attach_args *)aux;

	if (strcmp(xa->xa_device, "xbc") == 0)
		return 1;
	return 0;
}


void
xbc_attach(struct device *parent, struct device *self, void *aux)
{
	struct xbc_attach_args *xbda = (struct xbc_attach_args *)aux;
	struct xbc_softc *xs = (struct xbc_softc *)self;
	struct scsi_link *sl = &xs->sc_link;

	printf(": Xen Virtual Block Controller\n");

	simple_lock_init(&xs->sc_slock);
	xbcinit(xs, xbda);

	/* No need to run vbd_update() here.
	 * connect_interface() probes for drives and
	 * initializes/updates nr_vbds properly
	 */
	/* vbd_update(); */

	if (nr_vbds <= 0) {
		printf("%s: No virtual block devices found/available\n",
			xs->sc_dev.dv_xname);
		return;
	}

	sl->adapter_softc = xs;
	sl->adapter = &xbc_switch;
	sl->adapter_buswidth = nr_vbds;
	sl->adapter_target = nr_vbds;
	sl->device = &xbc_dev;
	sl->openings = BLKIF_RING_SIZE - 1;

	config_found(&xs->sc_dev, &xs->sc_link, scsiprint);

	return;
}

/*
 * Execute a [polled] command.
 */
int
xbc_cmd(struct xbc_softc *sc, int command, void *data,
	int datasize, vdisk_t *vd, int blkno, int flags,
	struct scsi_xfer *xs)
{
	struct buf *bp;
	int rv;

	DPRINTF(XBCB_IO, ("xbc_cmd: op=%x, xen dev=%i blk=%i data=%p[%x] xs=%p\n",
		command, vd->device, blkno, data, datasize, xs));


	bp = xs->bp;
	if (bp == NULL)
		panic("No buf available");

	DPRINTF(XBCB_IO, ("xbc_cmd: bp: flags=%d, bufsize=%d, bcount=%d,"
		" error=%d, dev=%d, blkno=%d, data=%p\n",
			bp->b_flags, bp->b_bufsize, bp->b_bcount,
			bp->b_error, bp->b_dev, bp->b_blkno, bp->b_data));

	rv = xbcstart(sc, bp);

	bp->b_resid = bp->b_bcount;

	if (!xs || xs->flags & SCSI_POLL) {
		/* Synchronous commands mustn't wait. */
		DPRINTF(XBCB_IO,
			("xbc_cmd: Synchronous commands mustn't wait.\n"));
		xbc_iodone(sc);
	}

	DPRINTF(XBCB_IO, ("xbc_cmd: leave with %i\n", rv));
	return rv;
}



int
xbc_scsi_cmd(struct scsi_xfer *xs)
{
	struct scsi_link *link = xs->sc_link;
	struct xbc_softc *sc = link->adapter_softc;
	struct scsi_inquiry_data inq;
	struct scsi_sense_data sd;
	struct scsi_read_cap_data rcd;
	uint8_t target = link->target;
	uint32_t blockno, blockcnt, size;
	struct scsi_rw *rw;
	struct scsi_rw_big *rwb;
	int op, flags, s, poll, error;
	vdisk_t *vd = get_vdisk(target);

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
		DPRINTF(XBCB_SETUP, ("scsi cmd TEST_UNIT_READY tgt %d\n",
			target));
		break;
	case START_STOP:
		DPRINTF(XBCB_SETUP, ("scsi cmd START_STOP tgt %d\n",
			target));
		break;
#if 0
	case VERIFY:
#endif
		DPRINTF(XBCB_SETUP, ("scsi cmd VERIFY tgt %d\n", target));
		break;

	case MODE_SENSE:
		DPRINTF(XBCB_SETUP, ("scsi cmd MODE_SENSE tgt %d\n", target));
		break;

	case MODE_SENSE_BIG:
		DPRINTF(XBCB_SETUP, ("scsi cmd MODE_SENSE_BIG tgt %d\n",
			target));
		break;

	case REQUEST_SENSE:
		DPRINTF(XBCB_SETUP, ("scsi cmd REQUEST_SENSE tgt %d\n",
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
		DPRINTF(XBCB_SETUP, ("scsi cmd INCQUIRY tgt %d\n", target));
		DPRINTF(XBCB_SETUP, ("vdisk type: "));
		bzero(&inq, sizeof(inq));
		switch (VDISK_TYPE(vd->info)) {
		case VDISK_TYPE_TAPE:
			inq.device = T_SEQUENTIAL;
			DPRINTF(XBCB_SETUP, ("sequential, "));
			break;
		case VDISK_TYPE_CDROM:
			inq.device = T_CDROM;
			DPRINTF(XBCB_SETUP, ("cdrom, "));
			break;
		case VDISK_TYPE_OPTICAL:
			inq.device = T_OPTICAL;
			DPRINTF(XBCB_SETUP, ("optical, "));
			break;
		case VDISK_TYPE_DISK:
		default:
			inq.device = T_DIRECT;
			DPRINTF(XBCB_SETUP, ("disk: %x, \n",
				VDISK_TYPE(vd->info)));
			break;
		} 
		DPRINTF(XBCB_SETUP, ("%s, ",
			VDISK_READONLY(vd->info) ? "readonly" : "writable"));
		DPRINTF(XBCB_SETUP, ("%s drive\n",
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
		DPRINTF(XBCB_SETUP, ("scsi cmd PREVENT_ALLOW tgt %d\n",
			target));
		goto notyet;

	/*
	 * SCSI disk specific opcodes
	 */

	case READ_CAPACITY:
		DPRINTF(XBCB_SETUP, ("scsi cmd READ_CAPACITY tgt %d\n",
			target));
		bzero(&rcd, sizeof(rcd));
		_lto4b(vd->capacity - 1, rcd.addr);	/* disk size */
		_lto4b(DEV_BSIZE, rcd.length);		/* sector size */
		xbc_copy_internal_data(xs, &rcd, sizeof(rcd));
		break;

	case SYNCHRONIZE_CACHE:
		DPRINTF(XBCB_IO, ("scsi cmd SYNCHRONIZE_CACHE tgt %d\n",
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
			DPRINTF(XBCB_IO, ("scsi cmd READ_COMMAND tgt %d\n",
				target));
			op = BLKIF_OP_READ;
			flags = B_READ;
			break;
		case READ_BIG:
			DPRINTF(XBCB_IO, ("scsi cmd READ_BIG tgt %d\n",
				target));
			op = BLKIF_OP_READ;
			flags = B_READ;
			break;
		case WRITE_COMMAND:
			DPRINTF(XBCB_IO, ("scsi cmd WRITE_COMMAND tgt %d\n",
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
			DPRINTF(XBCB_IO, ("scsi cmd WRITE_BIG tgt %d\n",
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
			DPRINTF(XBCB_IO,
				("xbc_scsi_cmd: (polling) COMPLETE\n"));
			return (COMPLETE);
		} else {
			DPRINTF(XBCB_IO,
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


/* Tell the controller to bring up the interface. */
void
send_interface_connect(void)
{
	ctrl_msg_t cmsg = {
		.type    = CMSG_BLKIF_FE,
		.subtype = CMSG_BLKIF_FE_INTERFACE_CONNECT,
		.length  = sizeof(blkif_fe_interface_connect_t),
	};
	blkif_fe_interface_connect_t *msg =
		(blkif_fe_interface_connect_t *)cmsg.msg;
	paddr_t pa;

	pmap_extract(pmap_kernel(), (vaddr_t)blk_ring, &pa);

	msg->handle = 0;
	msg->shmem_frame = xpmap_ptom_masked(pa) >> PAGE_SHIFT;

	ctrl_if_send_message_block(&cmsg, NULL, 0, 0);
}


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

#ifdef todo
static struct device *
find_device(vdisk_t *vd)
{
	struct device *dv;
	struct device *dv_parent;
	struct xbc_softc *xs = NULL;

	for (dv = alldevs.tqh_first; dv != NULL; dv = dv->dv_list.tqe_next) {
		dv_parent = dv->dv_parent;
		/* Our SCSI children devices have us as parent
		 */
		if (dv_parent == NULL ||
		    dv_parent->dv_cfdata == NULL ||
		    dv_parent->dv_cfdata->cf_attach == NULL ||
		    dv_parent->dv_cfdata->cf_attach->ca_attach != xbc_attach)
			continue;
		xs = (struct xbc_softc *)dv;
		if (vd == NULL || xs->sc_xd_device == vd->device)
			break;
	}
	return dv;
}
#endif

extern int hypervisor_print(void *, const char *);

void
vbd_update(void)
{
	struct xbc_attach_args *xbda;
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
			DPRINTF(XBCB_HOTPLUG,
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
			DPRINTF(XBCB_HOTPLUG,
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
			DPRINTF(XBCB_HOTPLUG,
			    ("update device %x size %lx size %lx\n",
				vbd_info_update[i].device,
				(u_long)vbd_info[j].capacity,
				(u_long)vbd_info_update[i].capacity));
		}
	}
#ifdef todo
	for (; i < nr_vbds; i++) {
		DPRINTF(XBCB_HOTPLUG,
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
		DPRINTF(XBCB_HOTPLUG,
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

static int
map_align(struct xbcreq *xr)
{
	int s;

	s = splvm();
	xr->xr_aligned = uvm_km_kmemalloc1(kmem_map, NULL,
		xr->xr_bqueue, XEN_BSIZE, UVM_UNKNOWN_OFFSET, UVM_KMF_NOWAIT);
	splx(s);
	if (xr->xr_aligned == 0)
		return 0;
	DPRINTF(XBCB_IO, ("map_align(%p): bp %p addr %p align 0x%08lx "
	    "size 0x%04lx\n", xr, xr->xr_bp, xr->xr_bp->b_data,
	    xr->xr_aligned, xr->xr_bqueue));
	xr->xr_data = xr->xr_aligned;
	if ((xr->xr_bp->b_flags & B_READ) == 0)
		memcpy((void *)xr->xr_aligned, xr->xr_bp->b_data,
		    xr->xr_bqueue);
	return 1;
}

static void
unmap_align(struct xbcreq *xr)
{
	int s;

	if (xr->xr_bp->b_flags & B_READ)
		memcpy(xr->xr_bp->b_data, (void *)xr->xr_aligned,
		    xr->xr_bp->b_bcount);
	DPRINTF(XBCB_IO, ("unmap_align(%p): bp %p addr %p align 0x%08lx "
	    "size 0x%04x\n", xr, xr->xr_bp, xr->xr_bp->b_data,
	    xr->xr_aligned, xr->xr_bp->b_bcount));
	s = splvm();
	uvm_km_free(kmem_map, xr->xr_aligned, xr->xr_bp->b_bcount);
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

	DPRINTF(XBCB_IO, ("fill_ring(%d): bp %p sector %llu pxr %p xr %p\n",
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

		DPRINTF(XBCB_IO, ("fill_ring(%d): va 0x%08lx pa 0x%08lx "
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
	struct xbc_softc *sc;
	struct buf *bp;
	struct scsi_xfer *xs;

	while ((pxr = SIMPLEQ_FIRST(&xbdr_suspended)) != NULL) {
		DPRINTF(XBCB_IO, ("xbcresume: resuming xbcreq %p for bp %p\n",
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
		DPRINTF(XBCB_IO, ("xbcresume: resumed xbcreq %p for bp %p\n",
		    pxr, bp));
		SIMPLEQ_REMOVE_HEAD(&xbdr_suspended, xr_suspended);
	}

 out:
	return;
}

void
signal_requests_to_xen(void)
{

	DPRINTF(XBCB_IO, ("signal_requests_to_xen: %x -> %x\n",
		    blk_ring->req_prod, req_prod));
	blk_ring->req_prod = req_prod;
	last_req_prod = req_prod;

	hypervisor_notify_via_evtchn(blkif_evtchn);
	return;
}


int
xbcstart(struct xbc_softc *sc, struct buf *bp)
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

	DPRINTF(XBCB_IO, ("xbcstart: addr %p, sector %llu, "
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
			DPRINTF(XBCB_IO, ("xbdstart: map_align failed\n"));
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
		DPRINTF(XBCB_IO, ("xbcstart: suspended xbcreq %p "
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
xbc_iodone(struct xbc_softc *xs)
{
	struct buf *bp;

	DPRINTF_FOLLOW(("xbc_iodone(%p)\n", xs));

	/* Process the work queue */
	while ((bp = BUFQ_GET(bufq)) != NULL) {
		printf("xbc_iodone: processing bp = %p\n", bp);
		if (xbcstart(xs, bp) != 0) {
			DPRINTF(XBCB_IO, ("xbc_iodone: re-add  bp = %p and abort\n",
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
	struct xbc_softc *sc;
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

			DPRINTF(XBCB_IO, ("xbc_response_handler(%d): pxr %p "
				    "xr %p bdone %04lx breq %04lx\n", i, pxr,
				    xr, pxr->xr_bdone, xr->xr_breq));
			pxr->xr_bdone -= xr->xr_breq;
			DIAGCONDPANIC(pxr->xr_bdone < 0,
			    ("xbc_response_handler: pxr->xr_bdone < 0"));

			if (__predict_false(ring_resp->status)) {
				DPRINTF(XBCB_IO, ("xbc_response_handler(%d): Operation failed, Xen return code: %i\n",
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
				DPRINTF(XBCB_IO, ("xbc_response_handler(%d): "
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
	struct xbc_attach_args *xbda;
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


/* Send a driver status notification to the domain controller. */
void
send_driver_status(int ok)
{
	ctrl_msg_t cmsg = {
		.type    = CMSG_BLKIF_FE,
		.subtype = CMSG_BLKIF_FE_DRIVER_STATUS,
		.length  = sizeof(blkif_fe_driver_status_t),
	};
	blkif_fe_driver_status_t *msg = (blkif_fe_driver_status_t *)cmsg.msg;

	msg->status = ok ? BLKIF_DRIVER_STATUS_UP : BLKIF_DRIVER_STATUS_DOWN;

	ctrl_if_send_message_block(&cmsg, NULL, 0, 0);
}



void
xbc_ctrlif_rx(ctrl_msg_t *msg, unsigned long id)
{
	switch (msg->subtype) {
	case CMSG_BLKIF_FE_INTERFACE_STATUS:
		if (msg->length != sizeof(blkif_fe_interface_status_t))
			goto parse_error;
		blkif_status((blkif_fe_interface_status_t *)
		    &msg->msg[0]);
		break;
	default:
		goto parse_error;
	}

	ctrl_if_send_response(msg);
	return;

 parse_error:
	msg->length = 0;
	ctrl_if_send_response(msg);
}


int
xbcinit(struct xbc_softc *xs, struct xbc_attach_args *xbda)
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
	xs->sc_flags |= XBD_INITED;

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
