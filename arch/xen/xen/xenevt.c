/*      $NetBSD: xenevt.c,v 1.10 2006/07/25 05:14:38 riz Exp $      */

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

#include <sys/param.h>
#include <sys/kernel.h>
#include <sys/malloc.h>
#include <sys/systm.h>
#include <sys/device.h>
#include <sys/file.h>
#include <sys/filedesc.h>
#include <sys/poll.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <sys/proc.h>
#include <sys/conf.h>

#include <uvm/uvm_extern.h>

#include <machine/hypervisor.h>
#include <machine/xenpmap.h>
#include <machine/xenio.h>
#include <machine/xen.h>

extern struct evcount softxenevt_evtcnt;

/*
 * Interface between the event channel and userland.
 * Each process with a xenevt device instance open can regiter events it
 * wants to receive. It will get pending events by read(), eventually blocking
 * until some event is available. Pending events are ack'd by a bitmask
 * write()en to the device. Some special operations (such as events binding)
 * are done though ioctl().
 * Processes get a device instance by opening a cloning device.
 */

void		xenevtattach(int);
static int	xenevt_fread(struct file *, off_t *, struct uio *,
    struct ucred *);
static int	xenevt_fwrite(struct file *, off_t *, struct uio *,
    struct ucred *);
static int	xenevt_fioctl(struct file *, u_long, caddr_t, struct proc *);
static int	xenevt_fpoll(struct file *, int, struct proc *);
static int	xenevt_fclose(struct file *, struct proc *);
/* static int	xenevt_fkqfilter(struct file *, struct knote *); */

static struct fileops xenevt_fileops = {
	xenevt_fread,
	xenevt_fwrite,
	xenevt_fioctl,
	xenevt_fpoll,
	/* xenevt_fkqfilter */ NULL,
	/* xenevt_stat */ NULL,
	xenevt_fclose,
};

dev_type_open(xenevtopen);
dev_type_read(xenevtread);
dev_type_mmap(xenevtmmap);

const struct cdevsw xenevt_cdevsw = {
	xenevtopen, /* close */ NULL,
	xenevtread, /* write */ NULL, /* ioctl */ NULL,
	/* stop */ NULL, /* tty */ NULL, /* poll */ NULL,
	xenevtmmap, 0, 0, /* kqfilter */ NULL
};

/* minor numbers */
#define DEV_EVT 0
#define DEV_XSD 1

/* per-instance datas */
#define XENEVT_RING_SIZE 2048
#define XENEVT_RING_MASK 2047

#define BYTES_PER_PORT (sizeof(evtchn_port_t) / sizeof(uint8_t))

struct xenevt_d {
	struct simplelock lock;
	SIMPLEQ_ENTRY(xenevt_d) pendingq;
	boolean_t pending;
	evtchn_port_t ring[2048]; 
	u_int ring_read; /* pointer of the reader */
	u_int ring_write; /* pointer of the writer */
	u_int flags;
#define XENEVT_F_OVERFLOW 0x01 /* ring overflow */
	struct selinfo sel; /* used by poll */
};

/* event -> user device mapping */
static struct xenevt_d *devevent[NR_EVENT_CHANNELS];

/* pending events */
struct simplelock devevent_pending_lock = SIMPLELOCK_INITIALIZER;
SIMPLEQ_HEAD(, xenevt_d) devevent_pending =
    SIMPLEQ_HEAD_INITIALIZER(devevent_pending);

static void xenevt_donotify(struct xenevt_d *);
static void xenevt_record(struct xenevt_d *, evtchn_port_t);

/* called at boot time */
void
xenevtattach(int n)
{
	memset(devevent, 0, sizeof(devevent));
}

/* event callback */
void
xenevt_event(int port)
{
	struct xenevt_d *d;
	int ilevel;

	d = devevent[port];
	if (d != NULL) {
		xenevt_record(d, port);

		if (d->pending) {
			return;
		}

		ilevel = cpl;

		if (ilevel < IPL_SOFTXENEVT) {
			/* fast and common path */
			softxenevt_evtcnt.ec_count++;
			xenevt_donotify(d);
		} else {
			simple_lock(&devevent_pending_lock);
			SIMPLEQ_INSERT_TAIL(&devevent_pending, d, pendingq);
			simple_unlock(&devevent_pending_lock);
			d->pending = TRUE;
			softintr(SIR_XENEVT);
		}
	}
}

void
xenevt_notify()
{

	cli();
	simple_lock(&devevent_pending_lock);
	while (/* CONSTCOND */ 1) {
		struct xenevt_d *d;

		d = SIMPLEQ_FIRST(&devevent_pending);
		if (d == NULL) {
			break;
		}
		SIMPLEQ_REMOVE_HEAD(&devevent_pending, pendingq);
		simple_unlock(&devevent_pending_lock);
		sti();

		d->pending = FALSE;
		xenevt_donotify(d);

		cli();
		simple_lock(&devevent_pending_lock);
	}
	simple_unlock(&devevent_pending_lock);
	sti();
}

static void
xenevt_donotify(struct xenevt_d *d)
{
	int s;

	s = splsoftxenevt();
	simple_lock(&d->lock);
	 
#ifdef __NetBSD__
	selnotify(&d->sel, 1);
#endif
	wakeup(&d->ring_read);

	simple_unlock(&d->lock);
	splx(s);
}

static void
xenevt_record(struct xenevt_d *d, evtchn_port_t port)
{

	/*
	 * This algorithm overflows for one less slot than available.
	 * Not really an issue, and the correct algorithm would be more
	 * complex
	 */

	if (d->ring_read ==
	    ((d->ring_write + 1) & XENEVT_RING_MASK)) {
		d->flags |= XENEVT_F_OVERFLOW;
		printf("xenevt_event: ring overflow port %d\n", port);
	} else {
		d->ring[d->ring_write] = port;
		d->ring_write = (d->ring_write + 1) & XENEVT_RING_MASK;
	}
}

/* XXX Move this to src/sys/sys/file.h
 *   From NetBSD.
 */
#ifdef DIAGNOSTIC
#define FILE_USE_CHECK(fp, str)                                         \
do {                                                                    \
	if ((fp)->f_usecount < 0)                                       \
		panic(str);                                             \
} while (/* CONSTCOND */ 0)
#else
#define FILE_USE_CHECK(fp, str)         /* nothing */
#endif

/*
 * FILE_USE() must be called with the file lock held.
 * (Typical usage is: `fp = fd_getfile(..); FILE_USE(fp);'
 * and fd_getfile() returns the file locked)
 */
#define FILE_USE(fp)                                                    \
do {                                                                    \
	(fp)->f_usecount++;                                             \
	FILE_USE_CHECK((fp), "f_usecount overflow");                    \
	simple_unlock(&(fp)->f_slock);                                  \
} while (/* CONSTCOND */ 0)

#define FILE_UNUSE_WLOCK(fp, l, havelock)                               \
do {                                                                    \
	if (!(havelock))                                                \
		simple_lock(&(fp)->f_slock);                            \
	if ((fp)->f_iflags & FIF_WANTCLOSE) {                           \
		simple_unlock(&(fp)->f_slock);                          \
		/* Will drop usecount */                                \
		(void) closef((fp), (l));                               \
		break;                                                  \
	} else {                                                        \
		(fp)->f_usecount--;                                     \
		FILE_USE_CHECK((fp), "f_usecount underflow");           \
	}                                                               \
	simple_unlock(&(fp)->f_slock);                                  \
} while (/* CONSTCOND */ 0)
#define FILE_UNUSE(fp, l)               FILE_UNUSE_WLOCK(fp, l, 0)
#define FILE_UNUSE_HAVELOCK(fp, l)      FILE_UNUSE_WLOCK(fp, l, 1)



/*
 *   From NetBSD.
 */
static int
fdclone(struct proc *p, struct file *fp, int fd, int flag,
    struct fileops *fops, void *data)
{
	fp->f_flag = flag;
	fp->f_type = DTYPE_VNODE;
	fp->f_ops = fops;
	fp->f_data = data;

	p->p_dupfd = fd;

	FILE_SET_MATURE(fp);
	FILE_UNUSE(fp, p);

#ifdef __NetBSD__
	return EMOVEFD;
#endif
	return EJUSTRETURN;
}




/* open the xenevt device; this is where we clone */
int
xenevtopen(dev_t dev, int flags, int mode, struct proc *p)
{
	struct xenevt_d *d;
	struct file *fp;
	int fd, error;

	switch(minor(dev)) {
	case DEV_EVT:
		/* falloc() will use the descriptor for us. */
		if ((error = falloc(p, &fp, &fd)) != 0)
			return error;

		d = malloc(sizeof(*d), M_DEVBUF, M_WAITOK);
		memset(d, 0, sizeof(*d));
		simple_lock_init(&d->lock);
		return fdclone(p, fp, fd, flags, &xenevt_fileops, d);

	case DEV_XSD:
		/* no clone for /dev/xsd_kva */
		return (0);

	default:
		break;
	}
	return ENODEV;
}

/* read from device: only for /dev/xsd_kva, xenevt is done though fread */
int
xenevtread(dev_t dev, struct uio *uio, int flags)
{
#define LD_STRLEN 21 /* a 64bit integer needs 20 digits in base10 */
	if (minor(dev) == DEV_XSD) {
		char strbuf[LD_STRLEN], *bf;
		int off, error;
		size_t len;

		off = (int)uio->uio_offset;
		if (off < 0)
			return EINVAL;
		len  = snprintf(strbuf, sizeof(strbuf), "%ld\n",
		    xen_start_info.store_mfn);
		if (off >= len) {
			bf = strbuf;
			len = 0;
		} else {
			bf = &strbuf[off];
			len -= off;
		}
		error = uiomove(bf, len, uio);
		return error;
	}
	return ENODEV;
}

/* mmap: only for xsd_kva */
paddr_t
xenevtmmap(dev_t dev, off_t off, int prot)
{
	if (minor(dev) == DEV_XSD) {
		/* only one page, so off is always 0 */
		if (off != 0)
			return -1;
		return i386_btop(
		    xpmap_mtop(xen_start_info.store_mfn << PAGE_SHIFT));
	}
	return -1;
}

static int
xenevt_fclose(struct file *fp, struct proc *p)
{
	struct xenevt_d *d = fp->f_data;
	int i;

	for (i = 0; i < NR_EVENT_CHANNELS; i++ ) {
		if (devevent[i] == d) {
			evtchn_op_t op = { 0 };
			int error;
			hypervisor_mask_event(i);
			devevent[i] = NULL;

			op.cmd = EVTCHNOP_close;
			op.u.close.port = i;
			if ((error = HYPERVISOR_event_channel_op(&op))) {
				printf("xenevt_fclose: error %d from "
				    "hypervisor\n", -error);
			}
		}
	}
	free(d, M_DEVBUF);
	fp->f_data = NULL;

	return (0);
}

static int
xenevt_fread(struct file *fp, off_t *offp, struct uio *uio,
    struct ucred *cred)
{
	struct xenevt_d *d = fp->f_data;
	int error;
	size_t len, uio_len;
	int ring_read;
	int ring_write;
	int s;

	error = 0;
	s = splsoftxenevt();
	simple_lock(&d->lock);
	while (error == 0) {
		ring_read = d->ring_read;
		ring_write = d->ring_write;
		if (ring_read != ring_write) {
			break;
		}
		if (d->flags & XENEVT_F_OVERFLOW) {
			break;
		}

		/* nothing to read */
		if (fp->f_flag & FNONBLOCK) {
			error = EAGAIN;
		} else {
			error = ltsleep(&d->ring_read, PRIBIO | PCATCH,
			    "xenevt", 0, &d->lock);
		}
	}
	if (error == 0 && (d->flags & XENEVT_F_OVERFLOW)) {
		error = EFBIG;
	}
	simple_unlock(&d->lock);
	splx(s);

	if (error) {
		return error;
	}

	uio_len = uio->uio_resid / BYTES_PER_PORT;
	if (ring_read <= ring_write)
		len = ring_write - ring_read;
	else
		len = XENEVT_RING_SIZE - ring_read;
	if (len > uio_len)
		len = uio_len;
	error = uiomove(&d->ring[ring_read], len * BYTES_PER_PORT, uio);
	if (error)
		return error;
	ring_read = (ring_read + len) & XENEVT_RING_MASK;
	uio_len = uio->uio_resid / BYTES_PER_PORT;
	if (uio_len == 0)
		goto done;
	/* ring wrapped, read the second part */
	len = ring_write - ring_read;
	if (len > uio_len)
		len = uio_len;
	error = uiomove(&d->ring[ring_read], len * BYTES_PER_PORT, uio);
	if (error)
		return error;
	ring_read = (ring_read + len) & XENEVT_RING_MASK;

done:
	s = splsoftxenevt();
	simple_lock(&d->lock);
	d->ring_read = ring_read;
	simple_unlock(&d->lock);
	splx(s);

	return 0;
}

static int
xenevt_fwrite(struct file *fp, off_t *offp, struct uio *uio,
    struct ucred *cred)
{
	struct xenevt_d *d = fp->f_data;
	u_int16_t chans[NR_EVENT_CHANNELS];
	int i, nentries, error;

	if (uio->uio_resid == 0)
		return (0);
	nentries = uio->uio_resid / sizeof(u_int16_t);
	if (nentries > NR_EVENT_CHANNELS)
		return EMSGSIZE;
	error = uiomove(chans, uio->uio_resid, uio);
	if (error)
		return error;
	for (i = 0; i < nentries; i++) {
		if (chans[i] < NR_EVENT_CHANNELS &&
		    devevent[chans[i]] == d) {
			hypervisor_unmask_event(chans[i]);
		}
	}
	return 0;
}

static int
xenevt_fioctl(struct file *fp, u_long cmd, caddr_t addr, struct proc *p)
{
	struct xenevt_d *d = fp->f_data;
	evtchn_op_t op = { 0 };
	int error;

	switch(cmd) {
	case IOCTL_EVTCHN_RESET:
		d->ring_read = d->ring_write = 0;
		d->flags = 0;
		break;
	case IOCTL_EVTCHN_BIND_VIRQ:
	{
		struct ioctl_evtchn_bind_virq *bind_virq = (void *)addr;
		op.cmd = EVTCHNOP_bind_virq;
		op.u.bind_virq.virq = bind_virq->virq;
		op.u.bind_virq.vcpu = 0;
		if ((error = HYPERVISOR_event_channel_op(&op))) {
			printf("IOCTL_EVTCHN_BIND_VIRQ failed: virq %d error %d\n", bind_virq->virq, error);
			return -error;
		}
		bind_virq->port = op.u.bind_virq.port;
		devevent[bind_virq->port] = d;
		hypervisor_unmask_event(bind_virq->port);
		break;
	}
	case IOCTL_EVTCHN_BIND_INTERDOMAIN:
	{
		struct ioctl_evtchn_bind_interdomain *bind_intd = (void *)addr;
		op.cmd = EVTCHNOP_bind_interdomain;
		op.u.bind_interdomain.remote_dom = bind_intd->remote_domain;
		op.u.bind_interdomain.remote_port = bind_intd->remote_port;
		if ((error = HYPERVISOR_event_channel_op(&op)))
			return -error;
		bind_intd->port = op.u.bind_interdomain.local_port;
		devevent[bind_intd->port] = d;
		hypervisor_unmask_event(bind_intd->port);
		break;
	}
	case IOCTL_EVTCHN_BIND_UNBOUND_PORT:
	{
		struct ioctl_evtchn_bind_unbound_port *bind_unbound = (void *)addr;
		op.cmd = EVTCHNOP_alloc_unbound;
		op.u.alloc_unbound.dom = DOMID_SELF;
		op.u.alloc_unbound.remote_dom = bind_unbound->remote_domain;
		if ((error = HYPERVISOR_event_channel_op(&op)))
			return -error;
		bind_unbound->port = op.u.alloc_unbound.port;
		devevent[bind_unbound->port] = d;
		hypervisor_unmask_event(bind_unbound->port);
		break;
	}
	case IOCTL_EVTCHN_UNBIND:
	{
		struct ioctl_evtchn_unbind *unbind = (void *)addr;
		
		if (unbind->port > NR_EVENT_CHANNELS)
			return EINVAL;
		if (devevent[unbind->port] != d)
			return ENOTCONN;
		devevent[unbind->port] = NULL;
		hypervisor_mask_event(unbind->port);
		op.cmd = EVTCHNOP_close;
		op.u.close.port = unbind->port;
		if ((error = HYPERVISOR_event_channel_op(&op)))
			return -error;
		break;
	}
	case IOCTL_EVTCHN_NOTIFY:
	{
		struct ioctl_evtchn_notify *notify = (void *)addr;
		
		if (notify->port > NR_EVENT_CHANNELS)
			return EINVAL;
		if (devevent[notify->port] != d)
			return ENOTCONN;
		hypervisor_notify_via_evtchn(notify->port);
		break;
	}
	case FIONBIO:
		break;
	default:
		return EINVAL;
	}
	return 0;
}

/*      
 * Support for poll() system call  
 *
 * Return true if the specific operation will not block indefinitely.
 */      

static int
xenevt_fpoll(struct file *fp, int events, struct proc *p)
{
	struct xenevt_d *d = fp->f_data;
	int revents = events & (POLLOUT | POLLWRNORM); /* we can always write */

	if (events & (POLLIN | POLLRDNORM)) {
		if (d->ring_read != d->ring_write) {
			revents |= events & (POLLIN | POLLRDNORM);
		} else {
			/* Record that someone is waiting */
			selrecord(p, &d->sel);
		}
	}
	return (revents);
}
