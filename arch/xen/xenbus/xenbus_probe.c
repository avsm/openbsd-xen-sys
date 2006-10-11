/* $NetBSD: xenbus_probe.c,v 1.2 2006/03/06 20:21:35 bouyer Exp $ */
/******************************************************************************
 * Talks to Xen Store to figure out what devices we have.
 *
 * Copyright (C) 2005 Rusty Russell, IBM Corporation
 * Copyright (C) 2005 Mike Wray, Hewlett-Packard
 * Copyright (C) 2005 XenSource Ltd
 * 
 * This file may be distributed separately from the Linux kernel, or
 * incorporated into other software packages, subject to the following license:
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this source file (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use, copy, modify,
 * merge, publish, distribute, sublicense, and/or sell copies of the Software,
 * and to permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 */

#include <sys/types.h>
#include <sys/errno.h>
#include <sys/malloc.h>
#include <sys/systm.h>
#include <sys/param.h>
#include <sys/kthread.h>
#include "strtoul.h"

#include <machine/xen-public/grant_table.h>
#include <machine/hypervisor.h>
#include <machine/xenbus.h>
#include <machine/evtchn.h>
#include <machine/shutdown_xenbus.h>
#include <machine/sysrq_xenbus.h>
#include <machine/xen.h>
#include "xenbus_comms.h"


#if defined(DOM0OPS)
#include <uvm/uvm_extern.h>
#endif

#if 0
#define DPRINTK(fmt, args...) \
	printf("xenbus_probe (%s:%d) " fmt ".\n", __FUNCTION__, __LINE__, ##args)
#else
#define DPRINTK(fmt, args...) ((void)0)
#endif

static int  xenbus_match(struct device *, void *, void *);
static void xenbus_attach(struct device *, struct device *, void *);

struct cfattach xenbus_ca = {
	sizeof(struct device), xenbus_match, xenbus_attach
};

struct cfdriver xenbus_cd = {
	NULL, "xenbus", DV_DULL
};

/* A flag to determine if xenstored is 'ready' (i.e. has started) */
int xenstored_ready = 0;

struct device *xenbus_sc;
SLIST_HEAD(, xenbus_device) xenbus_device_list;
SLIST_HEAD(, xenbus_backend_driver) xenbus_backend_driver_list =
	SLIST_HEAD_INITIALIZER(xenbus_backend_driver);


static void xenbus_kthread_create(void *);
static void xenbus_probe_init(void *);
static int xenbus_print(void *, const char *);

struct xen_bus_type
{
	const char *root;
	unsigned int levels;
};

static int xenbus_probe_devices(struct xen_bus_type *);


#define __UNCONST(x)	((void *)(unsigned long)(const void *)(x))


static void
free_otherend_watch(struct xenbus_device *dev)
{
	if (dev->xbusd_otherend_watch.node) {
		unregister_xenbus_watch(&dev->xbusd_otherend_watch);
		free(dev->xbusd_otherend_watch.node, M_DEVBUF);
		dev->xbusd_otherend_watch.node = NULL;
	}
}

/* Frontend */
static struct xen_bus_type xenbus_frontend = {
	.root = "device",
	.levels = 2,	    /* device/type/<id> */
};

/* Backend */
static struct xen_bus_type xenbus_backend = {
	.root = "backend",
	.levels = 3,	    /* backend/type/<frontend>/<id> */
};

static void
otherend_changed(struct xenbus_watch *watch,
			     const char **vec, unsigned int len)
{
	struct xenbus_device *xdev = watch->xbw_dev;
	XenbusState state;

	/* Protect us against watches firing on old details when the otherend
	   details change, say immediately after a resume. */
	if (!xdev->xbusd_otherend ||
	    strncmp(xdev->xbusd_otherend, vec[XS_WATCH_PATH],
		    strlen(xdev->xbusd_otherend))) {
		DPRINTK("Ignoring watch at %s", vec[XS_WATCH_PATH]);
		return;
	}

	state = xenbus_read_driver_state(xdev->xbusd_otherend);

	DPRINTK("state is %d, %s, %s",
		state, xdev->xbusd_otherend_watch.node, vec[XS_WATCH_PATH]);
	if (state == XenbusStateClosed) {
		int error;
		error = config_detach(xdev->xbusd_u.f.f_dev, DETACH_FORCE);
		if (error) {
			printf("could not detach %s: %d\n",
			    xdev->xbusd_u.f.f_dev->dv_xname, error);
			return;
		}
		xenbus_free_device(xdev);
		return;
	}
	if (xdev->xbusd_otherend_changed)
		xdev->xbusd_otherend_changed(xdev->xbusd_u.f.f_dev, state);
}

static int
talk_to_otherend(struct xenbus_device *dev)
{
	free_otherend_watch(dev);

	return xenbus_watch_path2(dev, dev->xbusd_otherend, "state",
				  &dev->xbusd_otherend_watch,
				  otherend_changed);
}

static void
frontend_changed(struct xenbus_watch *watch,
	const char **vec, unsigned int len)
{
	DPRINTK("");
#if 0
	printf("frontend_changed %s\n", vec[XS_WATCH_PATH]);
#endif
	xenbus_probe_devices(&xenbus_frontend);
}

static void
backend_changed(struct xenbus_watch *watch,
	const char **vec, unsigned int len)
{
	DPRINTK("");

#if 0
	printf("backend_changed %s\n", vec[XS_WATCH_PATH]);
#endif
	xenbus_probe_devices(&xenbus_backend);
}

/* We watch for devices appearing and vanishing. */
static struct xenbus_watch fe_watch;
static struct xenbus_watch be_watch;

int
xenbus_match(struct device *parent, void *match, void *aux)
{
	struct xenbus_attach_args *xa = (struct xenbus_attach_args *)aux;

	if (strcmp(xa->xa_device, "xenbus") == 0)
		return 1;
	return 0;
}

static void
xenbus_attach(struct device *parent, struct device *self, void *aux)
{
	printf(": Xen Virtual Bus Interface\n");
	xenbus_sc = self;
	config_pending_incr();
	kthread_create_deferred(xenbus_kthread_create, NULL);
}

void
xenbus_backend_register(struct xenbus_backend_driver *xbakd)
{
        SLIST_INSERT_HEAD(&xenbus_backend_driver_list, xbakd, xbakd_entries);
}

void
xenbus_kthread_create(void *unused)
{
	struct proc *p;
	int err;
	err = kthread_create(xenbus_probe_init, NULL, &p, "xenbus_probe");
	if (err)
		printf("kthread_create1(xenbus_probe): %d\n", err);
}

static int
read_otherend_details(struct xenbus_device *xendev,
				 const char *id_node, const char *path_node)
{
	int err;
	char *val, *ep;

	err = xenbus_read(NULL, xendev->xbusd_path, id_node, NULL, &val);
	if (err) {
		printf("reading other end details %s from %s\n",
		    id_node, xendev->xbusd_path);
		xenbus_dev_fatal(xendev, err,
				 "reading other end details %s from %s",
				 id_node, xendev->xbusd_path);
		return err;
	}
	xendev->xbusd_otherend_id = strtoul(val, &ep, 10);
	if (val[0] == '\0' || *ep != '\0') {
		printf("reading other end details %s from %s: %s is not a number\n", id_node, xendev->xbusd_path, val);
		xenbus_dev_fatal(xendev, err,
		    "reading other end details %s from %s: %s is not a number",
		    id_node, xendev->xbusd_path, val);
		free(val, M_DEVBUF);
		return EFTYPE;
	}
	free(val, M_DEVBUF);
	err = xenbus_read(NULL, xendev->xbusd_path, path_node, NULL, &val);
	if (err) {
		printf("reading other end details %s from %s (%d)\n",
		    path_node, xendev->xbusd_path, err);
		xenbus_dev_fatal(xendev, err,
				 "reading other end details %s from %s",
				 path_node, xendev->xbusd_path);
		return err;
	}
	xendev->xbusd_otherend = val;

	if (strlen(xendev->xbusd_otherend) == 0 ||
	    !xenbus_exists(NULL, xendev->xbusd_otherend, "")) {
		printf("missing other end from %s\n", xendev->xbusd_path);
		xenbus_dev_fatal(xendev, -ENOENT, "missing other end from %s",
				 xendev->xbusd_path);
		free(xendev->xbusd_otherend, M_DEVBUF);
		xendev->xbusd_otherend = NULL;
		return ENOENT;
	}

	return 0;
}

static int
read_backend_details(struct xenbus_device *xendev)
{
	return read_otherend_details(xendev, "backend-id", "backend");
}

static struct xenbus_device *
xenbus_lookup_device_path(const char *path)
{
	struct xenbus_device *xbusd;

	SLIST_FOREACH(xbusd, &xenbus_device_list, xbusd_entries) {
	if (strcmp(xbusd->xbusd_path, path) == 0)
		return xbusd;
	}
	return NULL;
}


static struct xenbus_device *
xenbus_allocate_device(struct xen_bus_type *bus,
			const char *type, const char *dir)
{
	struct xenbus_device *xbusd;
	int msize;

	/*
	 * add size of path to size of xenbus_device. xenbus_device
	 * already has room for one char in xbusd_path.
	 */
	msize = sizeof(*xbusd) + strlen(bus->root) + strlen(type)
		+ strlen(dir) + 3;
	xbusd = malloc(msize, M_DEVBUF, M_WAITOK);
	memset (xbusd, 0, msize);

	if (xbusd == NULL)
		panic("can't malloc xbusd");

	snprintf(__UNCONST(xbusd->xbusd_path),
		msize - sizeof(*xbusd) + 1, "%s/%s/%s",
		bus->root, type, dir);

	return xbusd;
}

static int
xenbus_probe_device_type(struct xen_bus_type *bus, const char *type)
{
	int err;
	char **dir;
	unsigned int dir_n = 0;
	int i;
	struct xenbus_device *xbusd;
	struct xenbusdev_attach_args xa;
	char *ep;

	DPRINTK("probe %s type %s", bus->root, type);
	err = xenbus_directory(NULL, bus->root, type, &dir_n, &dir);
	DPRINTK("directory err %d dir_n %d", err, dir_n);
	if (err)
		return err;

	for (i = 0; i < dir_n; i++) {
		xbusd = xenbus_allocate_device(bus, type, dir[i]);
		KASSERT(xbusd != NULL);

		if (xenbus_lookup_device_path(xbusd->xbusd_path) != NULL) {
			/* device already registered */
			free(xbusd, M_DEVBUF);
			continue;
		}

		xbusd->xbusd_otherend_watch.xbw_dev = xbusd;

		DPRINTK("xenbus_probe_device_type probe %s\n",
		    xbusd->xbusd_path);
		xa.xa_xbusd = xbusd;
		xa.xa_type = type;
		xa.xa_id = strtoul(dir[i], &ep, 0);
		if (dir[i][0] == '\0' || *ep != '\0') {
			printf("xenbus device type %s: id %s is not a number\n",
			    type, dir[i]);
			err = EFTYPE;
			free(xbusd, M_DEVBUF);
			break;
		}
		err = read_backend_details(xbusd);
		if (err != 0) {
			printf("xenbus: can't get backend details "
			    "for %s (%d)\n", xbusd->xbusd_path, err);
			free(xbusd, M_DEVBUF);
			break;
		}
		xbusd->xbusd_u.f.f_dev = config_found(xenbus_sc, &xa, xenbus_print);
		if (xbusd->xbusd_u.f.f_dev == NULL)
			free(xbusd, M_DEVBUF);
		else {
			SLIST_INSERT_HEAD(&xenbus_device_list,
				xbusd, xbusd_entries);
			talk_to_otherend(xbusd);
		}
	}
	free(dir, M_DEVBUF);
	return err;
}

static int
xenbus_print(void *aux, const char *pnp)
{
	struct xenbusdev_attach_args *xa = aux;

	if (pnp) {
		if (strcmp(xa->xa_type, "vbd") == 0)
			printf("xbd");
		else if (strcmp(xa->xa_type, "vif") == 0)
			printf("xennet");
		else
			printf("unknown type %s", xa->xa_type);
		printf(" at %s", pnp);
	}
	printf(" id %d", xa->xa_id);
	return(UNCONF);
}

static int
xenbus_probe_devices(struct xen_bus_type *bus)
{
	int err;
	char **dir;
	unsigned int i, dir_n;

	DPRINTK("probe %s", bus->root);
	err = xenbus_directory(NULL, bus->root, "", &dir_n, &dir);
	DPRINTK("directory err %d dir_n %d", err, dir_n);
	if (err)
		return err;

	for (i = 0; i < dir_n; i++) {
		err = xenbus_probe_device_type(bus, dir[i]);
		if (err)
			break;
	}
	free(dir, M_DEVBUF);
	return err;
}

int
xenbus_free_device(struct xenbus_device *xbusd)
{
	KASSERT(xenbus_lookup_device_path(xbusd->xbusd_path) == xbusd);
	SLIST_REMOVE(&xenbus_device_list, xbusd, xenbus_device, xbusd_entries);
	free_otherend_watch(xbusd);
	free(xbusd->xbusd_otherend, M_DEVBUF);
	xenbus_switch_state(xbusd, NULL, XenbusStateClosed);
	free(xbusd, M_DEVBUF);
	return 0;
}

void
xenbus_probe(void *unused)
{
	KASSERT((xenstored_ready > 0)); 

	/* Enumerate devices in xenstore. */
	xenbus_probe_devices(&xenbus_frontend);
	xenbus_probe_devices(&xenbus_backend);

	/* Watch for changes. */
	fe_watch.node = malloc(strlen("device") + 1, M_DEVBUF, M_NOWAIT);
	strlcpy(fe_watch.node, "device", strlen("device") + 1);
	fe_watch.xbw_callback = frontend_changed;
	register_xenbus_watch(&fe_watch);
	be_watch.node = malloc(strlen("backend") + 1, M_DEVBUF, M_NOWAIT);
	strlcpy(be_watch.node, "backend", strlen("backend") + 1);
	be_watch.xbw_callback = backend_changed;
	register_xenbus_watch(&be_watch);
	shutdown_xenbus_setup();
	sysrq_xenbus_setup();

	/* Notify others that xenstore is up */
	//notifier_call_chain(&xenstore_chain, 0, NULL);
}

/* XXX: assuming domU, will need a bit more for dom0 */
static void
xenbus_probe_init(void *unused)
{
	int err = 0, dom0;

	SLIST_INIT(&xenbus_device_list);


	/*
	** Domain0 doesn't have a store_evtchn or store_mfn yet.
	*/
	dom0 = (xen_start_info.store_evtchn == 0);
	if (dom0) {
#if defined(DOM0OPS)
		vaddr_t page;
		paddr_t ma;
		evtchn_op_t op = { 0 };
		int ret;

		/* Allocate page. */
		page = uvm_km_zalloc(kernel_map, PAGE_SIZE);
		if (!page)
			panic("can't get xenstore page");

		(void)pmap_extract_ma(pmap_kernel(), page, &ma);
		xen_start_info.store_mfn = ma >> PAGE_SHIFT;
		xenstore_interface = (void *)page;

		/* Next allocate a local port which xenstored can bind to */
		op.cmd = EVTCHNOP_alloc_unbound;
		op.u.alloc_unbound.dom        = DOMID_SELF;
		op.u.alloc_unbound.remote_dom = 0;

		ret = HYPERVISOR_event_channel_op(&op);
		if (ret)
			panic("can't register xenstore event");
		xen_start_info.store_evtchn = op.u.alloc_unbound.port;

#ifdef __NetBSD__
		/* And finally publish the above info in /kern/xen */
		xenbus_kernfs_init();
#endif

#else /* DOM0OPS */
		return ; /* can't get a working xenstore in this case */
#endif /* DOM0OPS */
	}


	/* register event handler, map shared page */
	xb_init_comms(xenbus_sc);

	/* Initialize the interface to xenstore. */
	err = xs_init(); 
	if (err) {
		printf("xenbus_probe_init(): Error initializing xenstore comms: %d\n", err);
		kthread_exit(err);
	}

	xenstored_ready = 1;
	xenbus_probe(NULL);

	config_pending_decr();
	kthread_exit(0);
}

/*
 * Local variables:
 *  c-file-style: "linux"
 *  indent-tabs-mode: t
 *  c-indent-level: 8
 *  c-basic-offset: 8
 *  tab-width: 8
 * End:
 */
