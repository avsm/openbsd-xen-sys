/*	$OpenBSD: ami.c,v 1.176 2007/01/27 05:09:51 dlg Exp $	*/

/*
 * Copyright (c) 2001 Michael Shalayeff
 * Copyright (c) 2005 Marco Peereboom
 * Copyright (c) 2006 David Gwynne
 * All rights reserved.
 *
 * The SCSI emulation layer is derived from gdt(4) driver,
 * Copyright (c) 1999, 2000 Niklas Hallqvist. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR OR HIS RELATIVES BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF MIND, USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */
/*
 * American Megatrends Inc. MegaRAID controllers driver
 *
 * This driver was made because these ppl and organizations
 * donated hardware and provided documentation:
 *
 * - 428 model card
 *	John Kerbawy, Stephan Matis, Mark Stovall;
 *
 * - 467 and 475 model cards, docs
 *	American Megatrends Inc.;
 *
 * - uninterruptable electric power for cvs
 *	Theo de Raadt.
 */

#include "bio.h"

/* #define	AMI_DEBUG */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/buf.h>
#include <sys/ioctl.h>
#include <sys/device.h>
#include <sys/kernel.h>
#include <sys/malloc.h>
#include <sys/proc.h>
#include <sys/rwlock.h>

#include <machine/bus.h>

#include <scsi/scsi_all.h>
#include <scsi/scsi_disk.h>
#include <scsi/scsiconf.h>

#include <dev/ic/amireg.h>
#include <dev/ic/amivar.h>


#if NBIO > 0
#include <dev/biovar.h>
#include <sys/sensors.h>
#endif

#ifdef AMI_DEBUG
#define	AMI_DPRINTF(m,a)	do { if (ami_debug & (m)) printf a; } while (0)
#define	AMI_D_CMD	0x0001
#define	AMI_D_INTR	0x0002
#define	AMI_D_MISC	0x0004
#define	AMI_D_DMA	0x0008
#define	AMI_D_IOCTL	0x0010
int ami_debug = 0
	| AMI_D_CMD
	| AMI_D_INTR
	| AMI_D_MISC
/*	| AMI_D_DMA */
/*	| AMI_D_IOCTL */
	;
#else
#define	AMI_DPRINTF(m,a)	/* m, a */
#endif

struct cfdriver ami_cd = {
	NULL, "ami", DV_DULL
};

int	ami_scsi_cmd(struct scsi_xfer *);
int	ami_scsi_ioctl(struct scsi_link *, u_long, caddr_t, int, struct proc *);
void	amiminphys(struct buf *bp);

struct scsi_adapter ami_switch = {
	ami_scsi_cmd, amiminphys, 0, 0, ami_scsi_ioctl
};

struct scsi_device ami_dev = {
	NULL, NULL, NULL, NULL
};

int	ami_scsi_raw_cmd(struct scsi_xfer *);

struct scsi_adapter ami_raw_switch = {
	ami_scsi_raw_cmd, amiminphys, 0, 0,
};

struct scsi_device ami_raw_dev = {
	NULL, NULL, NULL, NULL
};

struct ami_ccb	*ami_get_ccb(struct ami_softc *);
void		ami_put_ccb(struct ami_ccb *);

u_int32_t	ami_read(struct ami_softc *, bus_size_t);
void		ami_write(struct ami_softc *, bus_size_t, u_int32_t);

void		ami_copyhds(struct ami_softc *, const u_int32_t *,
		    const u_int8_t *, const u_int8_t *);
struct ami_mem	*ami_allocmem(struct ami_softc *, size_t);
void		ami_freemem(struct ami_softc *, struct ami_mem *);
int		ami_alloc_ccbs(struct ami_softc *, int);

int		ami_poll(struct ami_softc *, struct ami_ccb *);
void		ami_start(struct ami_softc *, struct ami_ccb *);
void		ami_complete(struct ami_softc *, struct ami_ccb *, int);
int		ami_done(struct ami_softc *, int);
void		ami_runqueue_tick(void *);
void		ami_runqueue(struct ami_softc *);

int 		ami_start_xs(struct ami_softc *sc, struct ami_ccb *,
		    struct scsi_xfer *);
void		ami_done_xs(struct ami_softc *, struct ami_ccb *);
void		ami_done_pt(struct ami_softc *, struct ami_ccb *);
void		ami_done_flush(struct ami_softc *, struct ami_ccb *);
void		ami_done_sysflush(struct ami_softc *, struct ami_ccb *);
void		ami_stimeout(void *);

void		ami_done_ioctl(struct ami_softc *, struct ami_ccb *);
void		ami_done_init(struct ami_softc *, struct ami_ccb *);

void		ami_copy_internal_data(struct scsi_xfer *, void *, size_t);

int		ami_load_ptmem(struct ami_softc*, struct ami_ccb *,
		    void *, size_t, int, int);

#if NBIO > 0
int		ami_mgmt(struct ami_softc *, u_int8_t, u_int8_t, u_int8_t,
		    u_int8_t, size_t, void *);
int		ami_drv_inq(struct ami_softc *, u_int8_t, u_int8_t, u_int8_t,
		    void *);
int		ami_ioctl(struct device *, u_long, caddr_t);
int		ami_ioctl_inq(struct ami_softc *, struct bioc_inq *);
int		ami_vol(struct ami_softc *, struct bioc_vol *,
		    struct ami_big_diskarray *);
int		ami_disk(struct ami_softc *, struct bioc_disk *,
		    struct ami_big_diskarray *);
int		ami_ioctl_vol(struct ami_softc *, struct bioc_vol *);
int		ami_ioctl_disk(struct ami_softc *, struct bioc_disk *);
int		ami_ioctl_alarm(struct ami_softc *, struct bioc_alarm *);
int		ami_ioctl_setstate(struct ami_softc *, struct bioc_setstate *);

int		ami_create_sensors(struct ami_softc *);
void		ami_refresh_sensors(void *);
#endif /* NBIO > 0 */

#define DEVNAME(_s)	((_s)->sc_dev.dv_xname)

struct ami_ccb *
ami_get_ccb(struct ami_softc *sc)
{
	struct ami_ccb *ccb;

	ccb = TAILQ_FIRST(&sc->sc_ccb_freeq);
	if (ccb) {
		TAILQ_REMOVE(&sc->sc_ccb_freeq, ccb, ccb_link);
		ccb->ccb_state = AMI_CCB_READY;
	}

	return (ccb);
}

void
ami_put_ccb(struct ami_ccb *ccb)
{
	struct ami_softc *sc = ccb->ccb_sc;

	ccb->ccb_state = AMI_CCB_FREE;
	ccb->ccb_xs = NULL;
	ccb->ccb_flags = 0;
	ccb->ccb_done = NULL;
	TAILQ_INSERT_TAIL(&sc->sc_ccb_freeq, ccb, ccb_link);
}

u_int32_t
ami_read(struct ami_softc *sc, bus_size_t r)
{
	u_int32_t rv;

	bus_space_barrier(sc->sc_iot, sc->sc_ioh, r, 4,
	    BUS_SPACE_BARRIER_READ);
	rv = bus_space_read_4(sc->sc_iot, sc->sc_ioh, r);

	AMI_DPRINTF(AMI_D_CMD, ("ari 0x%x 0x08%x ", r, rv));
	return (rv);
}

void
ami_write(struct ami_softc *sc, bus_size_t r, u_int32_t v)
{
	AMI_DPRINTF(AMI_D_CMD, ("awo 0x%x 0x%08x", r, v));

	bus_space_write_4(sc->sc_iot, sc->sc_ioh, r, v);
	bus_space_barrier(sc->sc_iot, sc->sc_ioh, r, 4,
	    BUS_SPACE_BARRIER_WRITE);
}

struct ami_mem *
ami_allocmem(struct ami_softc *sc, size_t size)
{
	struct ami_mem		*am;
	int			nsegs;

	am = malloc(sizeof(struct ami_mem), M_DEVBUF, M_NOWAIT);
	if (am == NULL)
		return (NULL);

	memset(am, 0, sizeof(struct ami_mem));
	am->am_size = size;

	if (bus_dmamap_create(sc->sc_dmat, size, 1, size, 0,
	    BUS_DMA_NOWAIT | BUS_DMA_ALLOCNOW, &am->am_map) != 0)
		goto amfree; 

	if (bus_dmamem_alloc(sc->sc_dmat, size, PAGE_SIZE, 0, &am->am_seg, 1,
	    &nsegs, BUS_DMA_NOWAIT) != 0)
		goto destroy;

	if (bus_dmamem_map(sc->sc_dmat, &am->am_seg, nsegs, size, &am->am_kva,
	    BUS_DMA_NOWAIT) != 0)
		goto free;

	if (bus_dmamap_load(sc->sc_dmat, am->am_map, am->am_kva, size, NULL,
	    BUS_DMA_NOWAIT) != 0)
		goto unmap;

	memset(am->am_kva, 0, size);
	return (am);

unmap:
	bus_dmamem_unmap(sc->sc_dmat, am->am_kva, size);
free:
	bus_dmamem_free(sc->sc_dmat, &am->am_seg, 1);
destroy:
	bus_dmamap_destroy(sc->sc_dmat, am->am_map);
amfree:
	free(am, M_DEVBUF);

	return (NULL);
}

void
ami_freemem(struct ami_softc *sc, struct ami_mem *am)
{
	bus_dmamap_unload(sc->sc_dmat, am->am_map);
	bus_dmamem_unmap(sc->sc_dmat, am->am_kva, am->am_size);
	bus_dmamem_free(sc->sc_dmat, &am->am_seg, 1);
	bus_dmamap_destroy(sc->sc_dmat, am->am_map);
	free(am, M_DEVBUF);
}

void
ami_copyhds(struct ami_softc *sc, const u_int32_t *sizes,
    const u_int8_t *props, const u_int8_t *stats)
{
	int i;

	for (i = 0; i < sc->sc_nunits; i++) {
		sc->sc_hdr[i].hd_present = 1;
		sc->sc_hdr[i].hd_is_logdrv = 1;
		sc->sc_hdr[i].hd_size = letoh32(sizes[i]);
		sc->sc_hdr[i].hd_prop = props[i];
		sc->sc_hdr[i].hd_stat = stats[i];
	}
}

int
ami_alloc_ccbs(struct ami_softc *sc, int nccbs)
{
	struct ami_ccb *ccb;
	struct ami_ccbmem *ccbmem, *mem;
	int i, error;

	sc->sc_ccbs = malloc(sizeof(struct ami_ccb) * nccbs,
	    M_DEVBUF, M_NOWAIT);
	if (sc->sc_ccbs == NULL) {
		printf(": unable to allocate ccbs\n");
		return (1);
	}

	sc->sc_ccbmem_am = ami_allocmem(sc, sizeof(struct ami_ccbmem) * nccbs);
	if (sc->sc_ccbmem_am == NULL) {
		printf(": unable to allocate ccb dmamem\n");
		goto free_ccbs;
	}
	ccbmem = AMIMEM_KVA(sc->sc_ccbmem_am);

	TAILQ_INIT(&sc->sc_ccb_freeq);
	TAILQ_INIT(&sc->sc_ccb_preq);
	TAILQ_INIT(&sc->sc_ccb_runq);
	timeout_set(&sc->sc_run_tmo, ami_runqueue_tick, sc);

	for (i = 0; i < nccbs; i++) {
		ccb = &sc->sc_ccbs[i];
		mem = &ccbmem[i];

		error = bus_dmamap_create(sc->sc_dmat, AMI_MAXFER,
		    AMI_MAXOFFSETS, AMI_MAXFER, 0,
		    BUS_DMA_NOWAIT | BUS_DMA_ALLOCNOW, &ccb->ccb_dmamap);
		if (error) {
			printf(": cannot create ccb dmamap (%d)\n", error);
			goto free_list;
		}

		ccb->ccb_sc = sc;

		ccb->ccb_cmd.acc_id = i + 1;
		ccb->ccb_offset = sizeof(struct ami_ccbmem) * i;

		ccb->ccb_pt = &mem->cd_pt;
		ccb->ccb_ptpa = htole32(AMIMEM_DVA(sc->sc_ccbmem_am) +
		    ccb->ccb_offset);

		ccb->ccb_sglist = mem->cd_sg;
		ccb->ccb_sglistpa = htole32(AMIMEM_DVA(sc->sc_ccbmem_am) +
		    ccb->ccb_offset + sizeof(struct ami_passthrough));

		ami_put_ccb(ccb);
	}

	return (0);

free_list:
	while ((ccb = ami_get_ccb(sc)) != NULL)
		bus_dmamap_destroy(sc->sc_dmat, ccb->ccb_dmamap);

	ami_freemem(sc, sc->sc_ccbmem_am);
free_ccbs:
	free(sc->sc_ccbs, M_DEVBUF);

	return (1);
}

int
ami_attach(struct ami_softc *sc)
{
	struct scsibus_attach_args saa;
	struct ami_rawsoftc *rsc;
	struct ami_ccb iccb;
	struct ami_iocmd *cmd;
	struct ami_mem *am;
	const char *p;
	paddr_t	pa;
	int s;

	am = ami_allocmem(sc, NBPG);
	if (am == NULL) {
		printf(": unable to allocate init data\n");
		return (1);
	}
	pa = htole32(AMIMEM_DVA(am));

	sc->sc_mbox_am = ami_allocmem(sc, sizeof(struct ami_iocmd));
	if (sc->sc_mbox_am == NULL) {
		printf(": unable to allocate mbox\n");
		goto free_idata;
	}
	sc->sc_mbox = (volatile struct ami_iocmd *)AMIMEM_KVA(sc->sc_mbox_am);
	sc->sc_mbox_pa = htole32(AMIMEM_DVA(sc->sc_mbox_am));
	AMI_DPRINTF(AMI_D_CMD, ("mbox_pa=%llx ", sc->sc_mbox_pa));

	/* create a spartan ccb for use with ami_poll */
	bzero(&iccb, sizeof(iccb));
	iccb.ccb_sc = sc;
	iccb.ccb_done = ami_done_init;
	cmd = &iccb.ccb_cmd;

	(sc->sc_init)(sc);

	s = splbio();

	/* try FC inquiry first */
	cmd->acc_cmd = AMI_FCOP;
	cmd->acc_io.aio_channel = AMI_FC_EINQ3;
	cmd->acc_io.aio_param = AMI_FC_EINQ3_SOLICITED_FULL;
	cmd->acc_io.aio_data = pa;
	if (ami_poll(sc, &iccb) == 0) {
		struct ami_fc_einquiry *einq = AMIMEM_KVA(am);
		struct ami_fc_prodinfo *pi = AMIMEM_KVA(am);

		sc->sc_nunits = einq->ain_nlogdrv;
		ami_copyhds(sc, einq->ain_ldsize, einq->ain_ldprop,
		    einq->ain_ldstat);

		cmd->acc_cmd = AMI_FCOP;
		cmd->acc_io.aio_channel = AMI_FC_PRODINF;
		cmd->acc_io.aio_param = 0;
		cmd->acc_io.aio_data = pa;
		if (ami_poll(sc, &iccb) == 0) {
			sc->sc_maxunits = AMI_BIG_MAX_LDRIVES;

			bcopy (pi->api_fwver, sc->sc_fwver, 16);
			sc->sc_fwver[15] = '\0';
			bcopy (pi->api_biosver, sc->sc_biosver, 16);
			sc->sc_biosver[15] = '\0';
			sc->sc_channels = pi->api_channels;
			sc->sc_targets = pi->api_fcloops;
			sc->sc_memory = letoh16(pi->api_ramsize);
			sc->sc_maxcmds = pi->api_maxcmd;
			p = "FC loop";
		}
	}

	if (sc->sc_maxunits == 0) {
		struct ami_inquiry *inq = AMIMEM_KVA(am);

		cmd->acc_cmd = AMI_EINQUIRY;
		cmd->acc_io.aio_channel = 0;
		cmd->acc_io.aio_param = 0;
		cmd->acc_io.aio_data = pa;
		if (ami_poll(sc, &iccb) != 0) {
			cmd->acc_cmd = AMI_INQUIRY;
			cmd->acc_io.aio_channel = 0;
			cmd->acc_io.aio_param = 0;
			cmd->acc_io.aio_data = pa;
			if (ami_poll(sc, &iccb) != 0) {
				splx(s);
				printf(": cannot do inquiry\n");
				goto free_mbox;
			}
		}

		sc->sc_maxunits = AMI_MAX_LDRIVES;
		sc->sc_nunits = inq->ain_nlogdrv;
		ami_copyhds(sc, inq->ain_ldsize, inq->ain_ldprop,
		    inq->ain_ldstat);

		bcopy (inq->ain_fwver, sc->sc_fwver, 4);
		sc->sc_fwver[4] = '\0';
		bcopy (inq->ain_biosver, sc->sc_biosver, 4);
		sc->sc_biosver[4] = '\0';
		sc->sc_channels = inq->ain_channels;
		sc->sc_targets = inq->ain_targets;
		sc->sc_memory = inq->ain_ramsize;
		sc->sc_maxcmds = inq->ain_maxcmd;
		p = "target";
	}

	if (sc->sc_flags & AMI_BROKEN) {
		sc->sc_link.openings = 1;
		sc->sc_maxcmds = 1;
		sc->sc_maxunits = 1;
	} else {
		sc->sc_maxunits = AMI_BIG_MAX_LDRIVES;
		if (sc->sc_maxcmds > AMI_MAXCMDS)
			sc->sc_maxcmds = AMI_MAXCMDS;
		/*
		 * Reserve ccb's for ioctl's and raw commands to
		 * processors/enclosures by lowering the number of
		 * openings available for logical units.
		 */
		sc->sc_maxcmds -= AMI_MAXIOCTLCMDS + AMI_MAXPROCS *
		    AMI_MAXRAWCMDS * sc->sc_channels;

		if (sc->sc_nunits)
			sc->sc_link.openings =
			    sc->sc_maxcmds / sc->sc_nunits;
		else
			sc->sc_link.openings = sc->sc_maxcmds;
	}

	splx(s);

	ami_freemem(sc, am);

	if (ami_alloc_ccbs(sc, AMI_MAXCMDS) != 0) {
		/* error already printed */
		goto free_mbox;
	}

	/* hack for hp netraid version encoding */
	if ('A' <= sc->sc_fwver[2] && sc->sc_fwver[2] <= 'Z' &&
	    sc->sc_fwver[1] < ' ' && sc->sc_fwver[0] < ' ' &&
	    'A' <= sc->sc_biosver[2] && sc->sc_biosver[2] <= 'Z' &&
	    sc->sc_biosver[1] < ' ' && sc->sc_biosver[0] < ' ') {

		snprintf(sc->sc_fwver, sizeof sc->sc_fwver, "%c.%02d.%02d",
		    sc->sc_fwver[2], sc->sc_fwver[1], sc->sc_fwver[0]);
		snprintf(sc->sc_biosver, sizeof sc->sc_biosver, "%c.%02d.%02d",
		    sc->sc_biosver[2], sc->sc_biosver[1], sc->sc_biosver[0]);
	}

	/* TODO: fetch & print cache strategy */
	/* TODO: fetch & print scsi and raid info */

	sc->sc_link.device = &ami_dev;
	sc->sc_link.adapter_softc = sc;
	sc->sc_link.adapter = &ami_switch;
	sc->sc_link.adapter_target = sc->sc_maxunits;
	sc->sc_link.adapter_buswidth = sc->sc_maxunits;

#ifdef AMI_DEBUG
	printf(", FW %s, BIOS v%s, %dMB RAM\n"
	    "%s: %d channels, %d %ss, %d logical drives, "
	    "openings %d, max commands %d, quirks: %04x\n",
	    sc->sc_fwver, sc->sc_biosver, sc->sc_memory, DEVNAME(sc),
	    sc->sc_channels, sc->sc_targets, p, sc->sc_nunits,
	    sc->sc_link.openings, sc->sc_maxcmds, sc->sc_flags);
#else
	printf(", FW %s, BIOS v%s, %dMB RAM\n"
	    "%s: %d channels, %d %ss, %d logical drives\n",
	    sc->sc_fwver, sc->sc_biosver, sc->sc_memory, DEVNAME(sc),
	    sc->sc_channels, sc->sc_targets, p, sc->sc_nunits);
#endif /* AMI_DEBUG */

	if (sc->sc_flags & AMI_BROKEN && sc->sc_nunits > 1)
		printf("%s: firmware buggy, limiting access to first logical "
		    "disk\n", DEVNAME(sc));

	/* lock around ioctl requests */
	rw_init(&sc->sc_lock, NULL);

	bzero(&saa, sizeof(saa));
	saa.saa_sc_link = &sc->sc_link;

	config_found(&sc->sc_dev, &saa, scsiprint);

	/* can't do bioctls, sensors, or pass-through on broken devices */
	if (sc->sc_flags & AMI_BROKEN)
		return (0);

#if NBIO > 0
	if (bio_register(&sc->sc_dev, ami_ioctl) != 0)
		printf("%s: controller registration failed\n", DEVNAME(sc));
	else
		sc->sc_ioctl = ami_ioctl;

	if (ami_create_sensors(sc) != 0)
		printf("%s: unable to create sensors\n", DEVNAME(sc));
#endif

	rsc = malloc(sizeof(struct ami_rawsoftc) * sc->sc_channels,
	    M_DEVBUF, M_NOWAIT);
	if (!rsc) {
		printf("%s: no memory for raw interface\n", DEVNAME(sc));
		return (0);
	}

	bzero(rsc, sizeof(struct ami_rawsoftc) * sc->sc_channels);
	for (sc->sc_rawsoftcs = rsc;
	     rsc < &sc->sc_rawsoftcs[sc->sc_channels]; rsc++) {

		rsc->sc_softc = sc;
		rsc->sc_channel = rsc - sc->sc_rawsoftcs;
		rsc->sc_link.device = &ami_raw_dev;
		rsc->sc_link.openings = AMI_MAXRAWCMDS;
		rsc->sc_link.adapter_softc = rsc;
		rsc->sc_link.adapter = &ami_raw_switch;
		rsc->sc_proctarget = -1;
		/* TODO fetch it from the controller */
		rsc->sc_link.adapter_target = 16;
		rsc->sc_link.adapter_buswidth = 16;

		bzero(&saa, sizeof(saa));
		saa.saa_sc_link = &rsc->sc_link;

		config_found(&sc->sc_dev, &saa, scsiprint);
	}

	return (0);

free_mbox:
	ami_freemem(sc, sc->sc_mbox_am);
free_idata:
	ami_freemem(sc, am);

	return (1);
}

int
ami_quartz_init(struct ami_softc *sc)
{
	ami_write(sc, AMI_QIDB, 0);

	return (0);
}

int
ami_quartz_exec(struct ami_softc *sc, struct ami_iocmd *cmd)
{
	if (sc->sc_mbox->acc_busy) {
		AMI_DPRINTF(AMI_D_CMD, ("mbox_busy "));
		return (EBUSY);
	}

	memcpy((struct ami_iocmd *)sc->sc_mbox, cmd, 16);
	bus_dmamap_sync(sc->sc_dmat, AMIMEM_MAP(sc->sc_mbox_am), 0,
	    sizeof(struct ami_iocmd), BUS_DMASYNC_PREWRITE|BUS_DMASYNC_PREREAD);

	sc->sc_mbox->acc_busy = 1;
	sc->sc_mbox->acc_poll = 0;
	sc->sc_mbox->acc_ack = 0;

	ami_write(sc, AMI_QIDB, sc->sc_mbox_pa | htole32(AMI_QIDB_EXEC));

	return (0);
}

int
ami_quartz_done(struct ami_softc *sc, struct ami_iocmd *mbox)
{
	u_int32_t i, n;
	u_int8_t nstat, status;
	u_int8_t completed[AMI_MAXSTATACK];

	if (ami_read(sc, AMI_QODB) != AMI_QODB_READY)
		return (0); /* nothing to do */

	ami_write(sc, AMI_QODB, AMI_QODB_READY);

	/*
	 * The following sequence is not supposed to have a timeout clause
	 * since the firmware has a "guarantee" that all commands will
	 * complete.  The choice is either panic or hoping for a miracle
	 * and that the IOs will complete much later.
	 */
	i = 0;
	while ((nstat = sc->sc_mbox->acc_nstat) == 0xff) {
		bus_dmamap_sync(sc->sc_dmat, AMIMEM_MAP(sc->sc_mbox_am), 0,
		    sizeof(struct ami_iocmd), BUS_DMASYNC_POSTREAD);
		delay(1);
		if (i++ > 1000000)
			return (0); /* nothing to do */
	}
	sc->sc_mbox->acc_nstat = 0xff;
	bus_dmamap_sync(sc->sc_dmat, AMIMEM_MAP(sc->sc_mbox_am), 0,
	    sizeof(struct ami_iocmd), BUS_DMASYNC_POSTWRITE);

	/* wait until fw wrote out all completions */
	i = 0;
	AMI_DPRINTF(AMI_D_CMD, ("aqd %d ", nstat));
	for (n = 0; n < nstat; n++) {
		bus_dmamap_sync(sc->sc_dmat, AMIMEM_MAP(sc->sc_mbox_am), 0,
		    sizeof(struct ami_iocmd), BUS_DMASYNC_PREREAD);
		while ((completed[n] = sc->sc_mbox->acc_cmplidl[n]) == 0xff) {
			delay(1);
			if (i++ > 1000000)
				return (0); /* nothing to do */
		}
		sc->sc_mbox->acc_cmplidl[n] = 0xff;
		bus_dmamap_sync(sc->sc_dmat, AMIMEM_MAP(sc->sc_mbox_am), 0,
		    sizeof(struct ami_iocmd), BUS_DMASYNC_POSTWRITE);
	}

	/* this should never happen, someone screwed up the completion status */
	if ((status = sc->sc_mbox->acc_status) == 0xff)
		panic("%s: status 0xff from the firmware", DEVNAME(sc));

	sc->sc_mbox->acc_status = 0xff;

	/* copy mailbox to temporary one and fixup other changed values */
	bus_dmamap_sync(sc->sc_dmat, AMIMEM_MAP(sc->sc_mbox_am), 0, 16,
	    BUS_DMASYNC_POSTWRITE);
	memcpy(mbox, (struct ami_iocmd *)sc->sc_mbox, 16);
	mbox->acc_nstat = nstat;
	mbox->acc_status = status;
	for (n = 0; n < nstat; n++)
		mbox->acc_cmplidl[n] = completed[n];

	/* ack interrupt */
	ami_write(sc, AMI_QIDB, AMI_QIDB_ACK);

	return (1);	/* ready to complete all IOs in acc_cmplidl */
}

int
ami_quartz_poll(struct ami_softc *sc, struct ami_iocmd *cmd)
{
	/* struct scsi_xfer *xs = ccb->ccb_xs; */
	u_int32_t i;
	u_int8_t status;

	if (sc->sc_dis_poll)
		return (1); /* fail */

	i = 0;
	while (sc->sc_mbox->acc_busy && (i < AMI_MAX_BUSYWAIT)) {
		delay(1);
		i++;
	}
	if (sc->sc_mbox->acc_busy) {
		AMI_DPRINTF(AMI_D_CMD, ("mbox_busy "));
		return (EBUSY);
	}

	memcpy((struct ami_iocmd *)sc->sc_mbox, cmd, 16);
	bus_dmamap_sync(sc->sc_dmat, AMIMEM_MAP(sc->sc_mbox_am), 0, 16,
	    BUS_DMASYNC_PREWRITE|BUS_DMASYNC_PREREAD);

	sc->sc_mbox->acc_id = 0xfe;
	sc->sc_mbox->acc_busy = 1;
	sc->sc_mbox->acc_poll = 0;
	sc->sc_mbox->acc_ack = 0;

	sc->sc_mbox->acc_nstat = 0xff;
	sc->sc_mbox->acc_status = 0xff;

	/* send command to firmware */
	ami_write(sc, AMI_QIDB, sc->sc_mbox_pa | htole32(AMI_QIDB_EXEC));

	while ((sc->sc_mbox->acc_nstat == 0xff) && (i < AMI_MAX_POLLWAIT)) {
		delay(1);
		i++;
	}
	if (i >= AMI_MAX_POLLWAIT) {
		printf("%s: command not accepted, polling disabled\n",
		    DEVNAME(sc));
		sc->sc_dis_poll = 1;
		return (1);
	}

	sc->sc_mbox->acc_nstat = 0xff;

	while ((sc->sc_mbox->acc_status == 0xff) && (i < AMI_MAX_POLLWAIT)) {
		delay(1);
		i++;
	}
	if (i >= AMI_MAX_POLLWAIT) {
		printf("%s: bad status, polling disabled\n", DEVNAME(sc));
		sc->sc_dis_poll = 1;
		return (1);
	}
	status = sc->sc_mbox->acc_status;
	sc->sc_mbox->acc_status = 0xff;

	/* poll firmware */
	while ((sc->sc_mbox->acc_poll != 0x77) && (i < AMI_MAX_POLLWAIT)) {
		delay(1);
		i++;
	}
	if (i >= AMI_MAX_POLLWAIT) {
		printf("%s: firmware didn't reply, polling disabled\n",
		    DEVNAME(sc));
		sc->sc_dis_poll = 1;
		return 1;
	}

	sc->sc_mbox->acc_poll = 0;
	sc->sc_mbox->acc_ack = 0x77;

	/* ack */
	ami_write(sc, AMI_QIDB, sc->sc_mbox_pa | htole32(AMI_QIDB_ACK));

	while((ami_read(sc, AMI_QIDB) & AMI_QIDB_ACK) &&
	    (i < AMI_MAX_POLLWAIT)) {
		delay(1);
		i++;
	}
	if (i >= AMI_MAX_POLLWAIT) {
		printf("%s: firmware didn't ack the ack, polling disabled\n",
		    DEVNAME(sc));
		sc->sc_dis_poll = 1;
		return (1);
	}

	for (i = 0; i < AMI_MAXSTATACK; i++)
		sc->sc_mbox->acc_cmplidl[i] = 0xff;

	return (status);
}

int
ami_schwartz_init(struct ami_softc *sc)
{
	u_int32_t a = (u_int32_t)sc->sc_mbox_pa;

	bus_space_write_4(sc->sc_iot, sc->sc_ioh, AMI_SMBADDR, a);
	/* XXX 40bit address ??? */
	bus_space_write_1(sc->sc_iot, sc->sc_ioh, AMI_SMBENA, 0);

	bus_space_write_1(sc->sc_iot, sc->sc_ioh, AMI_SCMD, AMI_SCMD_ACK);
	bus_space_write_1(sc->sc_iot, sc->sc_ioh, AMI_SIEM, AMI_SEIM_ENA |
	    bus_space_read_1(sc->sc_iot, sc->sc_ioh, AMI_SIEM));

	return (0);
}

int
ami_schwartz_exec(struct ami_softc *sc, struct ami_iocmd *cmd)
{
	if (bus_space_read_1(sc->sc_iot, sc->sc_ioh, AMI_SMBSTAT) &
	    AMI_SMBST_BUSY) {
		AMI_DPRINTF(AMI_D_CMD, ("mbox_busy "));
		return (EBUSY);
	}

	memcpy((struct ami_iocmd *)sc->sc_mbox, cmd, 16);
	sc->sc_mbox->acc_busy = 1;
	sc->sc_mbox->acc_poll = 0;
	sc->sc_mbox->acc_ack = 0;

	bus_space_write_1(sc->sc_iot, sc->sc_ioh, AMI_SCMD, AMI_SCMD_EXEC);
	return (0);
}

int
ami_schwartz_done(struct ami_softc *sc, struct ami_iocmd *mbox)
{
	u_int8_t stat;

#if 0
	/* do not scramble the busy mailbox */
	if (sc->sc_mbox->acc_busy)
		return (0);
#endif
	if (bus_space_read_1(sc->sc_iot, sc->sc_ioh, AMI_SMBSTAT) &
	    AMI_SMBST_BUSY)
		return (0);

	stat = bus_space_read_1(sc->sc_iot, sc->sc_ioh, AMI_ISTAT);
	if (stat & AMI_ISTAT_PEND) {
		bus_space_write_1(sc->sc_iot, sc->sc_ioh, AMI_ISTAT, stat);

		*mbox = *sc->sc_mbox;
		AMI_DPRINTF(AMI_D_CMD, ("asd %d ", mbox->acc_nstat));

		bus_space_write_1(sc->sc_iot, sc->sc_ioh, AMI_SCMD,
		    AMI_SCMD_ACK);

		return (1);
	}

	return (0);
}

int
ami_schwartz_poll(struct ami_softc *sc, struct ami_iocmd *mbox)
{
	u_int8_t status;
	u_int32_t i;
	int rv;

	if (sc->sc_dis_poll)
		return (1); /* fail */

	for (i = 0; i < AMI_MAX_POLLWAIT; i++) {
		if (!(bus_space_read_1(sc->sc_iot, sc->sc_ioh, AMI_SMBSTAT) &
		    AMI_SMBST_BUSY))
			break;
		delay(1);
	}
	if (i >= AMI_MAX_POLLWAIT) {
		AMI_DPRINTF(AMI_D_CMD, ("mbox_busy "));
		return (EBUSY);
	}

	memcpy((struct ami_iocmd *)sc->sc_mbox, mbox, 16);
	bus_dmamap_sync(sc->sc_dmat, AMIMEM_MAP(sc->sc_mbox_am), 0, 16,
	    BUS_DMASYNC_PREWRITE|BUS_DMASYNC_PREREAD);

	sc->sc_mbox->acc_busy = 1;
	sc->sc_mbox->acc_poll = 0;
	sc->sc_mbox->acc_ack = 0;
	/* send command to firmware */
	bus_space_write_1(sc->sc_iot, sc->sc_ioh, AMI_SCMD, AMI_SCMD_EXEC);

	/* wait until no longer busy */
	for (i = 0; i < AMI_MAX_POLLWAIT; i++) {
		if (!(bus_space_read_1(sc->sc_iot, sc->sc_ioh, AMI_SMBSTAT) &
		    AMI_SMBST_BUSY))
			break;
		delay(1);
	}
	if (i >= AMI_MAX_POLLWAIT) {
		printf("%s: command not accepted, polling disabled\n",
		    DEVNAME(sc));
		sc->sc_dis_poll = 1;
		return (1); /* fail */
	}

	/* wait for interrupt bit */
	for (i = 0; i < AMI_MAX_POLLWAIT; i++) {
		status = bus_space_read_1(sc->sc_iot, sc->sc_ioh, AMI_ISTAT);
		if (status & AMI_ISTAT_PEND)
			break;
		delay(1);
	}
	if (i >= AMI_MAX_POLLWAIT) {
		printf("%s: interrupt didn't arrive, polling disabled\n",
		    DEVNAME(sc));
		sc->sc_dis_poll = 1;
		return (1); /* fail */
	}

	/* write ststus back to firmware */
	bus_space_write_1(sc->sc_iot, sc->sc_ioh, AMI_ISTAT, status);

	/* copy mailbox and status back */
	bus_dmamap_sync(sc->sc_dmat, AMIMEM_MAP(sc->sc_mbox_am), 0,
	    sizeof(struct ami_iocmd), BUS_DMASYNC_PREREAD);
	*mbox = *sc->sc_mbox;
	rv = sc->sc_mbox->acc_status;

	/* ack interrupt */
	bus_space_write_1(sc->sc_iot, sc->sc_ioh, AMI_SCMD, AMI_SCMD_ACK);

	return (rv);
}

int
ami_start_xs(struct ami_softc *sc, struct ami_ccb *ccb, struct scsi_xfer *xs)
{
	timeout_set(&xs->stimeout, ami_stimeout, ccb);

	if (xs->flags & SCSI_POLL) {
		ami_complete(sc, ccb, xs->timeout);
		return (COMPLETE);
	}
 
	timeout_add(&xs->stimeout, (xs->timeout * hz) / 1000);
	ami_start(sc, ccb);

	return (SUCCESSFULLY_QUEUED);
}

void
ami_start(struct ami_softc *sc, struct ami_ccb *ccb)
{
	int s;

	s = splbio();
	ccb->ccb_state = AMI_CCB_PREQUEUED;
	TAILQ_INSERT_TAIL(&sc->sc_ccb_preq, ccb, ccb_link);
	ami_runqueue(sc);
	splx(s);
}

void
ami_runqueue_tick(void *arg)
{
	struct ami_softc *sc = arg;
	int s;

	s = splbio();
	ami_runqueue(sc);
	splx(s);
}

void
ami_runqueue(struct ami_softc *sc)
{
	struct ami_ccb *ccb;

	while ((ccb = TAILQ_FIRST(&sc->sc_ccb_preq)) != NULL) {
		if (sc->sc_exec(sc, &ccb->ccb_cmd) != 0) {
			/* this is now raceable too with other incomming io */
			timeout_add(&sc->sc_run_tmo, 1);
			break;
		}

		TAILQ_REMOVE(&sc->sc_ccb_preq, ccb, ccb_link);
		ccb->ccb_state = AMI_CCB_QUEUED;
		TAILQ_INSERT_TAIL(&sc->sc_ccb_runq, ccb, ccb_link);
	}
}

int
ami_poll(struct ami_softc *sc, struct ami_ccb *ccb)
{
	int error;
	int s;

	/* XXX this is broken, shall drain IO or consider this
	 * a normal completion which can complete async and
	 * polled commands until the polled commands completes
	 */

	s = splbio();
	error = sc->sc_poll(sc, &ccb->ccb_cmd);
	if (error)
		ccb->ccb_flags |= AMI_CCB_F_ERR;

	ccb->ccb_done(sc, ccb);
	splx(s);

	return (error);
}

void
ami_complete(struct ami_softc *sc, struct ami_ccb *ccb, int timeout)
{
	struct ami_iocmd mbox;
	int i = 0, j, done = 0;
	int s;

	s = splbio();

	/*
	 * since exec will return if the mbox is busy we have to busy wait
	 * ourselves. once its in, jam it into the runq.
	 */
	while (i < AMI_MAX_BUSYWAIT) {
		if (sc->sc_exec(sc, &ccb->ccb_cmd) == 0) {
			ccb->ccb_state = AMI_CCB_QUEUED;
			TAILQ_INSERT_TAIL(&sc->sc_ccb_runq, ccb, ccb_link);
			break;
		}
		
		DELAY(1000);
		i++;
	}
	if (ccb->ccb_state != AMI_CCB_QUEUED)
		goto err;

	i = 0;
	while (i < timeout) {
		if (sc->sc_done(sc, &mbox) != 0) {
			for (j = 0; j < mbox.acc_nstat; j++) {
				int ready = mbox.acc_cmplidl[j];
				ami_done(sc, ready);
				if (ready == ccb->ccb_cmd.acc_id)
					done = 1;
			}
			if (done)
				break;
		}

		DELAY(1000);
		i++;
	}
	if (!done) {
		printf("%s: timeout ccb %d\n", DEVNAME(sc),
		    ccb->ccb_cmd.acc_id);
		TAILQ_REMOVE(&sc->sc_ccb_runq, ccb, ccb_link);
		goto err;
	}

	/* start the runqueue again */
	ami_runqueue(sc);

	splx(s);

	return;

err:
	ccb->ccb_flags |= AMI_CCB_F_ERR;
	ccb->ccb_state = AMI_CCB_READY;
	ccb->ccb_done(sc, ccb);
	splx(s);
}

void
ami_stimeout(void *v)
{
	struct ami_ccb *ccb = v;
	struct ami_softc *sc = ccb->ccb_sc;
	struct ami_iocmd *cmd = &ccb->ccb_cmd;
	int s;

	s = splbio();
	switch (ccb->ccb_state) {
	case AMI_CCB_PREQUEUED:
		/* command never ran, cleanup is easy */
		TAILQ_REMOVE(&sc->sc_ccb_preq, ccb, ccb_link);
		ccb->ccb_flags |= AMI_CCB_F_ERR;
		break;

	case AMI_CCB_QUEUED:
		/* XXX create a list to save ccb to and print the whole list */
		printf("%s: timeout ccb %d\n", DEVNAME(sc), cmd->acc_id);
		TAILQ_REMOVE(&sc->sc_ccb_runq, ccb, ccb_link);
		break;

	default:
		panic("%s: ami_stimeout(%d) botch", DEVNAME(sc), cmd->acc_id);
	}

	ccb->ccb_done(sc, ccb);
	splx(s);
}

int
ami_done(struct ami_softc *sc, int idx)
{
	struct ami_ccb *ccb = &sc->sc_ccbs[idx - 1];

	AMI_DPRINTF(AMI_D_CMD, ("done(%d) ", ccb->ccb_cmd.acc_id));

	if (ccb->ccb_state != AMI_CCB_QUEUED) {
		printf("%s: unqueued ccb %d ready, state = %d\n",
		    DEVNAME(sc), idx, ccb->ccb_state);
		return (1);
	}

	ccb->ccb_state = AMI_CCB_READY;
	TAILQ_REMOVE(&sc->sc_ccb_runq, ccb, ccb_link);

	ccb->ccb_done(sc, ccb);

	return (0);
}

void
ami_done_pt(struct ami_softc *sc, struct ami_ccb *ccb)
{
	struct scsi_xfer *xs = ccb->ccb_xs;
	struct scsi_link *link = xs->sc_link;
	struct ami_rawsoftc *rsc = link->adapter_softc;
	u_int8_t target = link->target, type;

	bus_dmamap_sync(sc->sc_dmat, AMIMEM_MAP(sc->sc_ccbmem_am),
	    ccb->ccb_offset, sizeof(struct ami_ccbmem),
	    BUS_DMASYNC_POSTREAD | BUS_DMASYNC_POSTWRITE);

	if (xs->data != NULL) {
		bus_dmamap_sync(sc->sc_dmat, ccb->ccb_dmamap, 0,
		    ccb->ccb_dmamap->dm_mapsize,
		    (xs->flags & SCSI_DATA_IN) ?
		    BUS_DMASYNC_POSTREAD : BUS_DMASYNC_POSTWRITE);

		bus_dmamap_unload(sc->sc_dmat, ccb->ccb_dmamap);
	}

	timeout_del(&xs->stimeout);
	xs->resid = 0;
	xs->flags |= ITSDONE;

	if (ccb->ccb_flags & AMI_CCB_F_ERR)
		xs->error = XS_DRIVER_STUFFUP;
 	else if (xs->flags & SCSI_POLL && xs->cmd->opcode == INQUIRY) {
		type = ((struct scsi_inquiry_data *)xs->data)->device &
		    SID_TYPE;
		if (!(type == T_PROCESSOR || type == T_ENCLOSURE))
			xs->error = XS_DRIVER_STUFFUP;
		else
			rsc->sc_proctarget = target;
	}

	ami_put_ccb(ccb);
	scsi_done(xs);
}

void
ami_done_xs(struct ami_softc *sc, struct ami_ccb *ccb)
{
	struct scsi_xfer *xs = ccb->ccb_xs;

	if (xs->data != NULL) {
		bus_dmamap_sync(sc->sc_dmat, ccb->ccb_dmamap, 0,
		    ccb->ccb_dmamap->dm_mapsize,
		    (xs->flags & SCSI_DATA_IN) ?
		    BUS_DMASYNC_POSTREAD : BUS_DMASYNC_POSTWRITE);

		bus_dmamap_sync(sc->sc_dmat, AMIMEM_MAP(sc->sc_ccbmem_am),
		    ccb->ccb_offset, sizeof(struct ami_ccbmem),
		    BUS_DMASYNC_POSTREAD | BUS_DMASYNC_POSTWRITE);

		bus_dmamap_unload(sc->sc_dmat, ccb->ccb_dmamap);
	}

	timeout_del(&xs->stimeout);
	xs->resid = 0;
	xs->flags |= ITSDONE;

	if (ccb->ccb_flags & AMI_CCB_F_ERR)
		xs->error = XS_DRIVER_STUFFUP;

	ami_put_ccb(ccb);
	scsi_done(xs);
}

void
ami_done_flush(struct ami_softc *sc, struct ami_ccb *ccb)
{
	struct scsi_xfer *xs = ccb->ccb_xs;
	struct ami_iocmd *cmd = &ccb->ccb_cmd;

	timeout_del(&xs->stimeout);
	if (ccb->ccb_flags & AMI_CCB_F_ERR) {
		xs->error = XS_DRIVER_STUFFUP;
		xs->resid = 0;
		xs->flags |= ITSDONE;

		ami_put_ccb(ccb);
		scsi_done(xs);
		return;
	}

	/* reuse the ccb for the sysflush command */
	ccb->ccb_done = ami_done_sysflush;
	cmd->acc_cmd = AMI_SYSFLUSH;

	ami_start_xs(sc, ccb, xs);
}

void
ami_done_sysflush(struct ami_softc *sc, struct ami_ccb *ccb)
{
	struct scsi_xfer *xs = ccb->ccb_xs;

	timeout_del(&xs->stimeout);
	xs->resid = 0;
	xs->flags |= ITSDONE;
	if (ccb->ccb_flags & AMI_CCB_F_ERR)
		xs->error = XS_DRIVER_STUFFUP;

	ami_put_ccb(ccb);
	scsi_done(xs);
}

void
ami_done_ioctl(struct ami_softc *sc, struct ami_ccb *ccb)
{
	wakeup(ccb);
}

void
ami_done_init(struct ami_softc *sc, struct ami_ccb *ccb)
{
	/* the ccb is going to be reused, so do nothing with it */
}

void
amiminphys(struct buf *bp)
{
	if (bp->b_bcount > AMI_MAXFER)
		bp->b_bcount = AMI_MAXFER;
	minphys(bp);
}

void
ami_copy_internal_data(struct scsi_xfer *xs, void *v, size_t size)
{
	size_t copy_cnt;

	AMI_DPRINTF(AMI_D_MISC, ("ami_copy_internal_data "));

	if (!xs->datalen)
		printf("uio move not yet supported\n");
	else {
		copy_cnt = MIN(size, xs->datalen);
		bcopy(v, xs->data, copy_cnt);
	}
}

int
ami_scsi_raw_cmd(struct scsi_xfer *xs)
{
	struct scsi_link *link = xs->sc_link;
	struct ami_rawsoftc *rsc = link->adapter_softc;
	struct ami_softc *sc = rsc->sc_softc;
	u_int8_t channel = rsc->sc_channel, target = link->target;
	struct device *dev = link->device_softc;
	struct ami_ccb *ccb;
	int s;

	AMI_DPRINTF(AMI_D_CMD, ("ami_scsi_raw_cmd "));

	if (!cold && target == rsc->sc_proctarget)
		strlcpy(rsc->sc_procdev, dev->dv_xname,
		    sizeof(rsc->sc_procdev));

	if (xs->cmdlen > AMI_MAX_CDB) {
		AMI_DPRINTF(AMI_D_CMD, ("CDB too big %p ", xs));
		bzero(&xs->sense, sizeof(xs->sense));
		xs->sense.error_code = SSD_ERRCODE_VALID | 0x70;
		xs->sense.flags = SKEY_ILLEGAL_REQUEST;
		xs->sense.add_sense_code = 0x20; /* illcmd, 0x24 illfield */
		xs->error = XS_SENSE;
		s = splbio();
		scsi_done(xs);
		splx(s);
		return (COMPLETE);
	}

	xs->error = XS_NOERROR;

	s = splbio();	
	ccb = ami_get_ccb(sc);
	splx(s);
	if (ccb == NULL) {
		xs->error = XS_DRIVER_STUFFUP;
		s = splbio();
		scsi_done(xs);
		splx(s);
		return (COMPLETE);
	}

	memset(ccb->ccb_pt, 0, sizeof(struct ami_passthrough));

	ccb->ccb_xs = xs;
	ccb->ccb_done = ami_done_pt;

	ccb->ccb_cmd.acc_cmd = AMI_PASSTHRU;
	ccb->ccb_cmd.acc_passthru.apt_data = ccb->ccb_ptpa;
	
	ccb->ccb_pt->apt_param = AMI_PTPARAM(AMI_TIMEOUT_6,1,0);
	ccb->ccb_pt->apt_channel = channel;
	ccb->ccb_pt->apt_target = target;
	bcopy(xs->cmd, ccb->ccb_pt->apt_cdb, AMI_MAX_CDB);
	ccb->ccb_pt->apt_ncdb = xs->cmdlen;
	ccb->ccb_pt->apt_nsense = AMI_MAX_SENSE;
	ccb->ccb_pt->apt_datalen = xs->datalen;
	ccb->ccb_pt->apt_data = 0;

	if (ami_load_ptmem(sc, ccb, xs->data, xs->datalen,
	    xs->flags & SCSI_DATA_IN, xs->flags & SCSI_NOSLEEP) != 0) {
		xs->error = XS_DRIVER_STUFFUP;
		s = splbio();
		ami_put_ccb(ccb);
		scsi_done(xs);
		splx(s);
		return (COMPLETE);
	}

	return (ami_start_xs(sc, ccb, xs));
}

int
ami_load_ptmem(struct ami_softc *sc, struct ami_ccb *ccb, void *data,
    size_t len, int read, int nowait)
{
	bus_dmamap_t dmap = ccb->ccb_dmamap;
	bus_dma_segment_t *sgd;
	int error, i;

	if (data != NULL) {
		error = bus_dmamap_load(sc->sc_dmat, dmap, data, len, NULL,
		    nowait ? BUS_DMA_NOWAIT : BUS_DMA_WAITOK);
		if (error) {
			if (error == EFBIG)
				printf("more than %d dma segs\n",
				    AMI_MAXOFFSETS);
			else
				printf("error %d loading dma map\n", error);

			return (1);
		}

		sgd = dmap->dm_segs;
		if (dmap->dm_nsegs > 1) {
			struct ami_sgent *sgl = ccb->ccb_sglist;

			ccb->ccb_pt->apt_nsge = dmap->dm_nsegs;
			ccb->ccb_pt->apt_data = ccb->ccb_sglistpa;

			for (i = 0; i < dmap->dm_nsegs; i++) {
				sgl[i].asg_addr = htole32(sgd[i].ds_addr);
				sgl[i].asg_len = htole32(sgd[i].ds_len);
			}
		} else {
			ccb->ccb_pt->apt_nsge = 0;
			ccb->ccb_pt->apt_data = htole32(sgd->ds_addr);
		}

		bus_dmamap_sync(sc->sc_dmat, dmap, 0, dmap->dm_mapsize,
		    read ? BUS_DMASYNC_PREREAD : BUS_DMASYNC_PREWRITE);
	}

	bus_dmamap_sync(sc->sc_dmat, AMIMEM_MAP(sc->sc_ccbmem_am),
	    ccb->ccb_offset, sizeof(struct ami_ccbmem),
	    BUS_DMASYNC_PREREAD | BUS_DMASYNC_PREWRITE);

	return (0);
}

int
ami_scsi_cmd(struct scsi_xfer *xs)
{
	struct scsi_link *link = xs->sc_link;
	struct ami_softc *sc = link->adapter_softc;
	struct device *dev = link->device_softc;
	struct ami_ccb *ccb;
	struct ami_iocmd *cmd;
	struct scsi_inquiry_data inq;
	struct scsi_sense_data sd;
	struct scsi_read_cap_data rcd;
	u_int8_t target = link->target;
	u_int32_t blockno, blockcnt;
	struct scsi_rw *rw;
	struct scsi_rw_big *rwb;
	bus_dma_segment_t *sgd;
	int error;
	int s;
	int i;

	AMI_DPRINTF(AMI_D_CMD, ("ami_scsi_cmd "));

	if (target >= sc->sc_nunits || !sc->sc_hdr[target].hd_present ||
	    link->lun != 0) {
		AMI_DPRINTF(AMI_D_CMD, ("no target %d ", target));
		/* XXX should be XS_SENSE and sense filled out */
		xs->error = XS_DRIVER_STUFFUP;
		xs->flags |= ITSDONE;
		s = splbio();
		scsi_done(xs);
		splx(s);
		return (COMPLETE);
	}

	error = 0;
	xs->error = XS_NOERROR;

	switch (xs->cmd->opcode) {
	case READ_COMMAND:
	case READ_BIG:
	case WRITE_COMMAND:
	case WRITE_BIG:
		/* deal with io outside the switch */
		break;

	case SYNCHRONIZE_CACHE:
		s = splbio();
		ccb = ami_get_ccb(sc);
		splx(s);
		if (ccb == NULL) {
			xs->error = XS_DRIVER_STUFFUP;
			s = splbio();
			scsi_done(xs);
			splx(s);
			return (COMPLETE);
		}

		ccb->ccb_xs = xs;
		ccb->ccb_done = ami_done_flush;
		if (xs->timeout < 30000)
			xs->timeout = 30000;	/* at least 30sec */

		cmd = &ccb->ccb_cmd;
		cmd->acc_cmd = AMI_FLUSH;

		return (ami_start_xs(sc, ccb, xs));

	case TEST_UNIT_READY:
		/* save off sd? after autoconf */
		if (!cold)	/* XXX bogus */
			strlcpy(sc->sc_hdr[target].dev, dev->dv_xname,
			    sizeof(sc->sc_hdr[target].dev));
	case START_STOP:
#if 0
	case VERIFY:
#endif
	case PREVENT_ALLOW:
		AMI_DPRINTF(AMI_D_CMD, ("opc %d tgt %d ", xs->cmd->opcode,
		    target));
		return (COMPLETE);

	case REQUEST_SENSE:
		AMI_DPRINTF(AMI_D_CMD, ("REQUEST SENSE tgt %d ", target));
		bzero(&sd, sizeof(sd));
		sd.error_code = 0x70;
		sd.segment = 0;
		sd.flags = SKEY_NO_SENSE;
		*(u_int32_t*)sd.info = htole32(0);
		sd.extra_len = 0;
		ami_copy_internal_data(xs, &sd, sizeof(sd));
		s = splbio();
		scsi_done(xs);
		splx(s);
		return (COMPLETE);

	case INQUIRY:
		AMI_DPRINTF(AMI_D_CMD, ("INQUIRY tgt %d ", target));
		bzero(&inq, sizeof(inq));
		inq.device = T_DIRECT;
		inq.dev_qual2 = 0;
		inq.version = 2;
		inq.response_format = 2;
		inq.additional_length = 32;
		strlcpy(inq.vendor, "AMI    ", sizeof(inq.vendor));
		snprintf(inq.product, sizeof(inq.product),
		    "Host drive  #%02d", target);
		strlcpy(inq.revision, "   ", sizeof(inq.revision));
		ami_copy_internal_data(xs, &inq, sizeof(inq));
		s = splbio();
		scsi_done(xs);
		splx(s);
		return (COMPLETE);

	case READ_CAPACITY:
		AMI_DPRINTF(AMI_D_CMD, ("READ CAPACITY tgt %d ", target));
		bzero(&rcd, sizeof(rcd));
		_lto4b(sc->sc_hdr[target].hd_size - 1, rcd.addr);
		_lto4b(AMI_SECTOR_SIZE, rcd.length);
		ami_copy_internal_data(xs, &rcd, sizeof(rcd));
		s = splbio();
		scsi_done(xs);
		splx(s);
		return (COMPLETE);

	default:
		AMI_DPRINTF(AMI_D_CMD, ("unsupported scsi command %#x tgt %d ",
		    xs->cmd->opcode, target));
		xs->error = XS_DRIVER_STUFFUP;
		s = splbio();
		scsi_done(xs);
		splx(s);
		return (COMPLETE);
	}

	/* A read or write operation. */
	if (xs->cmdlen == 6) {
		rw = (struct scsi_rw *)xs->cmd;
		blockno = _3btol(rw->addr) & (SRW_TOPADDR << 16 | 0xffff);
		blockcnt = rw->length ? rw->length : 0x100;
	} else {
		rwb = (struct scsi_rw_big *)xs->cmd;
		blockno = _4btol(rwb->addr);
		blockcnt = _2btol(rwb->length);
	}

	if (blockno >= sc->sc_hdr[target].hd_size ||
	    blockno + blockcnt > sc->sc_hdr[target].hd_size) {
		printf("%s: out of bounds %u-%u >= %u\n", DEVNAME(sc),
		    blockno, blockcnt, sc->sc_hdr[target].hd_size);
		xs->error = XS_DRIVER_STUFFUP;
		s = splbio();
		scsi_done(xs);
		splx(s);
		return (COMPLETE);
	}

	s = splbio();
	ccb = ami_get_ccb(sc);
	splx(s);
	if (ccb == NULL) {
		xs->error = XS_DRIVER_STUFFUP;
		s = splbio();
		scsi_done(xs);
		splx(s);
		return (COMPLETE);
	}

	ccb->ccb_xs = xs;
	ccb->ccb_done = ami_done_xs;

	cmd = &ccb->ccb_cmd;
	cmd->acc_cmd = (xs->flags & SCSI_DATA_IN) ? AMI_READ : AMI_WRITE;
	cmd->acc_mbox.amb_nsect = htole16(blockcnt);
	cmd->acc_mbox.amb_lba = htole32(blockno);
	cmd->acc_mbox.amb_ldn = target;

	error = bus_dmamap_load(sc->sc_dmat, ccb->ccb_dmamap,
	    xs->data, xs->datalen, NULL,
	    (xs->flags & SCSI_NOSLEEP) ? BUS_DMA_NOWAIT : BUS_DMA_WAITOK);
	if (error) {
		if (error == EFBIG)
			printf("more than %d dma segs\n", AMI_MAXOFFSETS);
		else
			printf("error %d loading dma map\n", error);

		xs->error = XS_DRIVER_STUFFUP;
		s = splbio();
		ami_put_ccb(ccb);
		scsi_done(xs);
		splx(s);
		return (COMPLETE);
	}

	sgd = ccb->ccb_dmamap->dm_segs;
	if (ccb->ccb_dmamap->dm_nsegs > 1) {
		struct ami_sgent *sgl = ccb->ccb_sglist;

		cmd->acc_mbox.amb_nsge = ccb->ccb_dmamap->dm_nsegs;
		cmd->acc_mbox.amb_data = ccb->ccb_sglistpa;

		for (i = 0; i < ccb->ccb_dmamap->dm_nsegs; i++) {
			sgl[i].asg_addr = htole32(sgd[i].ds_addr);
			sgl[i].asg_len = htole32(sgd[i].ds_len);
		}
	} else {
		cmd->acc_mbox.amb_nsge = 0;
		cmd->acc_mbox.amb_data = htole32(sgd->ds_addr);
	}

	bus_dmamap_sync(sc->sc_dmat, AMIMEM_MAP(sc->sc_ccbmem_am),
	    ccb->ccb_offset, sizeof(struct ami_ccbmem),
	    BUS_DMASYNC_PREREAD | BUS_DMASYNC_PREWRITE);

	bus_dmamap_sync(sc->sc_dmat, ccb->ccb_dmamap, 0,
	    ccb->ccb_dmamap->dm_mapsize, (xs->flags & SCSI_DATA_IN) ?
	    BUS_DMASYNC_PREREAD : BUS_DMASYNC_PREWRITE);

	return (ami_start_xs(sc, ccb, xs));
}

int
ami_intr(void *v)
{
	struct ami_softc *sc = v;
	struct ami_iocmd mbox;
	int i, rv = 0;

	if (TAILQ_EMPTY(&sc->sc_ccb_runq))
		return (0);

	AMI_DPRINTF(AMI_D_INTR, ("intr "));

	while ((sc->sc_done)(sc, &mbox)) {
		AMI_DPRINTF(AMI_D_CMD, ("got#%d ", mbox.acc_nstat));
		for (i = 0; i < mbox.acc_nstat; i++ ) {
			int ready = mbox.acc_cmplidl[i];

			AMI_DPRINTF(AMI_D_CMD, ("ready=%d ", ready));

			if (!ami_done(sc, ready))
				rv |= 1;
		}
	}

	if (rv)
		ami_runqueue(sc);

	AMI_DPRINTF(AMI_D_INTR, ("exit "));
	return (rv);
}

int
ami_scsi_ioctl(struct scsi_link *link, u_long cmd, caddr_t addr, int flag,
    struct proc *p)
{
	struct ami_softc *sc = (struct ami_softc *)link->adapter_softc;
	/* struct device *dev = (struct device *)link->device_softc; */
	/* u_int8_t target = link->target; */

	if (sc->sc_ioctl)
		return (sc->sc_ioctl(link->adapter_softc, cmd, addr));
	else
		return (ENOTTY);
}

#if NBIO > 0
int
ami_ioctl(struct device *dev, u_long cmd, caddr_t addr)
{
	struct ami_softc *sc = (struct ami_softc *)dev;
	int error = 0;

	AMI_DPRINTF(AMI_D_IOCTL, ("%s: ioctl ", DEVNAME(sc)));

	if (sc->sc_flags & AMI_BROKEN)
		return (ENODEV); /* can't do this to broken device for now */

	switch (cmd) {
	case BIOCINQ:
		AMI_DPRINTF(AMI_D_IOCTL, ("inq "));
		error = ami_ioctl_inq(sc, (struct bioc_inq *)addr);
		break;

	case BIOCVOL:
		AMI_DPRINTF(AMI_D_IOCTL, ("vol "));
		error = ami_ioctl_vol(sc, (struct bioc_vol *)addr);
		break;

	case BIOCDISK:
		AMI_DPRINTF(AMI_D_IOCTL, ("disk "));
		error = ami_ioctl_disk(sc, (struct bioc_disk *)addr);
		break;

	case BIOCALARM:
		AMI_DPRINTF(AMI_D_IOCTL, ("alarm "));
		error = ami_ioctl_alarm(sc, (struct bioc_alarm *)addr);
		break;

	case BIOCSETSTATE:
		AMI_DPRINTF(AMI_D_IOCTL, ("setstate "));
		error = ami_ioctl_setstate(sc, (struct bioc_setstate *)addr);
		break;

	default:
		AMI_DPRINTF(AMI_D_IOCTL, (" invalid ioctl\n"));
		error = EINVAL;
	}

	return (error);
}

int
ami_drv_inq(struct ami_softc *sc, u_int8_t ch, u_int8_t tg, u_int8_t page,
    void *inqbuf)
{
	struct ami_ccb *ccb;
	struct ami_passthrough *pt;
	struct scsi_inquiry_data *inq = inqbuf;
	int error = 0;
	int s;

	rw_enter_write(&sc->sc_lock);

	s = splbio();
	ccb = ami_get_ccb(sc);
	splx(s);
	if (ccb == NULL) {
		error = ENOMEM;
		goto err;
	}

	ccb->ccb_done = ami_done_ioctl;

	ccb->ccb_cmd.acc_cmd = AMI_PASSTHRU;
	ccb->ccb_cmd.acc_passthru.apt_data = ccb->ccb_ptpa;

	pt = ccb->ccb_pt;
	memset(pt, 0, sizeof(struct ami_passthrough));
	pt->apt_channel = ch;
	pt->apt_target = tg;
	pt->apt_ncdb = sizeof(struct scsi_inquiry);
	pt->apt_nsense = sizeof(struct scsi_sense_data);
	pt->apt_datalen = sizeof(struct scsi_inquiry_data);
	pt->apt_data = 0;

	pt->apt_cdb[0] = INQUIRY;
	pt->apt_cdb[1] = 0;
	pt->apt_cdb[2] = 0;
	pt->apt_cdb[3] = 0;
	pt->apt_cdb[4] = sizeof(struct scsi_inquiry_data); /* INQUIRY length */
	pt->apt_cdb[5] = 0;

	if (page != 0) {
		pt->apt_cdb[1] = SI_EVPD;
		pt->apt_cdb[2] = page;
	}

	if (ami_load_ptmem(sc, ccb, inqbuf, sizeof(struct scsi_inquiry_data),
	    1, 0) != 0) {
		error = ENOMEM;
		goto ptmemerr;
	}

	ami_start(sc, ccb);

	while (ccb->ccb_state != AMI_CCB_READY)
		tsleep(ccb, PRIBIO, "ami_drv_inq", 0);

	bus_dmamap_sync(sc->sc_dmat, ccb->ccb_dmamap, 0,
	    ccb->ccb_dmamap->dm_mapsize, BUS_DMASYNC_POSTREAD);
	bus_dmamap_sync(sc->sc_dmat, AMIMEM_MAP(sc->sc_ccbmem_am),
	    ccb->ccb_offset, sizeof(struct ami_ccbmem),
	    BUS_DMASYNC_POSTREAD | BUS_DMASYNC_POSTWRITE);
	bus_dmamap_unload(sc->sc_dmat, ccb->ccb_dmamap);

	if (ccb->ccb_flags & AMI_CCB_F_ERR)
		error = EIO;
	else if (pt->apt_scsistat != 0x00)
		error = EIO;
	else if ((inq->device & SID_TYPE) != T_DIRECT)
		error = EINVAL;

ptmemerr:
	s = splbio();
	ami_put_ccb(ccb);
	splx(s);

err:
	rw_exit_write(&sc->sc_lock);
	return (error);
}

int
ami_mgmt(struct ami_softc *sc, u_int8_t opcode, u_int8_t par1, u_int8_t par2,
    u_int8_t par3, size_t size, void *buffer)
{
	struct ami_ccb *ccb;
	struct ami_iocmd *cmd;
	struct ami_mem *am = NULL;
	char *idata = NULL;
	int error = 0;
	int s;

	rw_enter_write(&sc->sc_lock);

	s = splbio();
	ccb = ami_get_ccb(sc);
	splx(s);
	if (ccb == NULL) {
		error = ENOMEM;
		goto err;
	}

	if (size) {
		if ((am = ami_allocmem(sc, size)) == NULL) {
			error = ENOMEM;
			goto memerr;
		}
		idata = AMIMEM_KVA(am);
	}

	ccb->ccb_done = ami_done_ioctl;
	cmd = &ccb->ccb_cmd;

	cmd->acc_cmd = opcode;

	/*
	 * some commands require data to be written to idata before sending
	 * command to fw
	 */
	switch (opcode) {
	case AMI_SPEAKER:
		*idata = par1;
		break;
	default:
		cmd->acc_io.aio_channel = par1;
		cmd->acc_io.aio_param = par2;
		cmd->acc_io.aio_pad[0] = par3;
		break;
	};

	cmd->acc_io.aio_data = am ? htole32(AMIMEM_DVA(am)) : 0;

	ami_start(sc, ccb);
	while (ccb->ccb_state != AMI_CCB_READY)
		tsleep(ccb, PRIBIO,"ami_mgmt", 0);

	if (ccb->ccb_flags & AMI_CCB_F_ERR)
		error = EIO;
	else if (buffer && size)
		memcpy(buffer, idata, size);

	if (am)
		ami_freemem(sc, am);

memerr:
	s = splbio();
	ami_put_ccb(ccb);
	splx(s);

err:
	rw_exit_write(&sc->sc_lock);
	return (error);
}

int
ami_ioctl_inq(struct ami_softc *sc, struct bioc_inq *bi)
{
	struct ami_big_diskarray *p; /* struct too large for stack */
	char *plist;
	int i, s, t;
	int off;
	int error = 0;
	struct scsi_inquiry_data inqbuf;
	u_int8_t ch, tg;

	p = malloc(sizeof *p, M_DEVBUF, M_NOWAIT);
	if (!p)
		return (ENOMEM);

	plist = malloc(AMI_BIG_MAX_PDRIVES, M_DEVBUF, M_NOWAIT);
	if (!plist) {
		error = ENOMEM;
		goto bail;
	}

	if (ami_mgmt(sc, AMI_FCOP, AMI_FC_RDCONF, 0, 0, sizeof *p, p)) {
		error = EINVAL;
		goto bail2;
	}

	memset(plist, 0, AMI_BIG_MAX_PDRIVES);

	bi->bi_novol = p->ada_nld;
	bi->bi_nodisk = 0;

	strlcpy(bi->bi_dev, DEVNAME(sc), sizeof(bi->bi_dev));

	/* do we actually care how many disks we have at this point? */
	for (i = 0; i < p->ada_nld; i++)
		for (s = 0; s < p->ald[i].adl_spandepth; s++)
			for (t = 0; t < p->ald[i].adl_nstripes; t++) {
				off = p->ald[i].asp[s].adv[t].add_channel *
				    AMI_MAX_TARGET +
				    p->ald[i].asp[s].adv[t].add_target;

				if (!plist[off]) {
					plist[off] = 1;
					bi->bi_nodisk++;
				}
			}

	/*
	 * hack warning!
	 * Megaraid cards sometimes return a size in the PD structure
	 * even though there is no disk in that slot.  Work around
	 * that by issuing an INQUIRY to determine if there is
	 * an actual disk in the slot.
	 */
	for(i = 0; i < ((sc->sc_flags & AMI_QUARTZ) ?
	    AMI_BIG_MAX_PDRIVES : AMI_MAX_PDRIVES); i++) {
	    	/* skip claimed drives */
	    	if (plist[i])
			continue;

	    	/*
		 * poke drive to make sure its there.  If it is it is either
		 * unused or a hot spare; at this point we dont care which it is
		 */
		if (p->apd[i].adp_size) {
			ch = (i & 0xf0) >> 4;
			tg = i & 0x0f;

			if (!ami_drv_inq(sc, ch, tg, 0, &inqbuf)) {
				bi->bi_novol++;
				bi->bi_nodisk++;
				plist[i] = 1;
			}
		}
	}

bail2:
	free(plist, M_DEVBUF);
bail:
	free(p, M_DEVBUF);

	return (error);
}

int
ami_vol(struct ami_softc *sc, struct bioc_vol *bv, struct ami_big_diskarray *p)
{
	struct scsi_inquiry_data inqbuf;
	char *plist;
	int i, s, t, off;
	int ld = p->ada_nld, error = EINVAL;
	u_int8_t ch, tg;

	plist = malloc(AMI_BIG_MAX_PDRIVES, M_DEVBUF, M_NOWAIT);
	if (!plist)
		return (ENOMEM);

	memset(plist, 0, AMI_BIG_MAX_PDRIVES);

	/* setup plist */
	for (i = 0; i < p->ada_nld; i++)
		for (s = 0; s < p->ald[i].adl_spandepth; s++)
			for (t = 0; t < p->ald[i].adl_nstripes; t++) {
				off = p->ald[i].asp[s].adv[t].add_channel *
				    AMI_MAX_TARGET +
				    p->ald[i].asp[s].adv[t].add_target;

				if (!plist[off])
					plist[off] = 1;
			}

	for(i = 0; i < ((sc->sc_flags & AMI_QUARTZ) ?
	    AMI_BIG_MAX_PDRIVES : AMI_MAX_PDRIVES); i++) {
	    	/* skip claimed drives */
	    	if (plist[i])
			continue;

	    	/*
		 * poke drive to make sure its there.  If it is it is either
		 * unused or a hot spare; at this point we dont care which it is
		 */
		if (p->apd[i].adp_size) {
			ch = (i & 0xf0) >> 4;
			tg = i & 0x0f;

			if (!ami_drv_inq(sc, ch, tg, 0, &inqbuf)) {
				if (ld != bv->bv_volid) {
					ld++;
					continue;
				}

				bv->bv_status = BIOC_SVONLINE;
				bv->bv_size = (u_quad_t)p->apd[i].adp_size *
				    (u_quad_t)512;
				bv->bv_nodisk = 1;
				strlcpy(bv->bv_dev,
				    sc->sc_hdr[bv->bv_volid].dev,
				    sizeof(bv->bv_dev));

				if (p->apd[i].adp_ostatus == AMI_PD_HOTSPARE
				    && p->apd[i].adp_type == 0)
					bv->bv_level = -1;
				else
					bv->bv_level = -2;

				error = 0;
				goto bail;
			}
		}
	}

bail:
	free(plist, M_DEVBUF);

	return (error);
}

int
ami_disk(struct ami_softc *sc, struct bioc_disk *bd,
    struct ami_big_diskarray *p)
{
	struct scsi_inquiry_data inqbuf;
	struct scsi_inquiry_vpd vpdbuf;
	char *plist;
	int i, s, t, off;
	int ld = p->ada_nld, error = EINVAL;
	u_int8_t ch, tg;

	plist = malloc(AMI_BIG_MAX_PDRIVES, M_DEVBUF, M_NOWAIT);
	if (!plist)
		return (ENOMEM);

	memset(plist, 0, AMI_BIG_MAX_PDRIVES);

	/* setup plist */
	for (i = 0; i < p->ada_nld; i++)
		for (s = 0; s < p->ald[i].adl_spandepth; s++)
			for (t = 0; t < p->ald[i].adl_nstripes; t++) {
				off = p->ald[i].asp[s].adv[t].add_channel *
				    AMI_MAX_TARGET +
				    p->ald[i].asp[s].adv[t].add_target;

				if (!plist[off])
					plist[off] = 1;
			}

	for(i = 0; i < ((sc->sc_flags & AMI_QUARTZ) ?
	    AMI_BIG_MAX_PDRIVES : AMI_MAX_PDRIVES); i++) {
		char vend[8+16+4+1];

	    	/* skip claimed drives */
	    	if (plist[i])
			continue;

		/* no size no disk, most of the times */
		if (!p->apd[i].adp_size)
			continue;

		ch = (i & 0xf0) >> 4;
		tg = i & 0x0f;

	    	/*
		 * poke drive to make sure its there.  If it is it is either
		 * unused or a hot spare; at this point we dont care which it is
		 */
		if (ami_drv_inq(sc, ch, tg, 0, &inqbuf)) 
			continue;
		
		if (ld != bd->bd_volid) {
			ld++;
			continue;
		}

		bcopy(inqbuf.vendor, vend, sizeof vend - 1);

		vend[sizeof vend - 1] = '\0';
		strlcpy(bd->bd_vendor, vend, sizeof(bd->bd_vendor));

		if (!ami_drv_inq(sc, ch, tg, 0x80, &vpdbuf)) {
			char ser[32 + 1];

			bcopy(vpdbuf.serial, ser, sizeof ser - 1);

			ser[sizeof ser - 1] = '\0';
			if (vpdbuf.page_length < sizeof ser)
				ser[vpdbuf.page_length] = '\0';

			strlcpy(bd->bd_serial, ser, sizeof(bd->bd_serial));
		}

		bd->bd_size = (u_quad_t)p->apd[i].adp_size * (u_quad_t)512;

		bd->bd_channel = ch;
		bd->bd_target = tg;

		strlcpy(bd->bd_procdev, sc->sc_rawsoftcs[ch].sc_procdev,
		    sizeof(bd->bd_procdev));

		if (p->apd[i].adp_ostatus == AMI_PD_HOTSPARE)
			bd->bd_status = BIOC_SDHOTSPARE;
		else
			bd->bd_status = BIOC_SDUNUSED;

#ifdef AMI_DEBUG
		if (p->apd[i].adp_type != 0)
			printf("invalid disk type: %d %d %x inquiry type: %x\n",
			    ch, tg, p->apd[i].adp_type, inqbuf.device);
#endif /* AMI_DEBUG */

		error = 0;
		goto bail;
	}

bail:
	free(plist, M_DEVBUF);

	return (error);
}

int
ami_ioctl_vol(struct ami_softc *sc, struct bioc_vol *bv)
{
	struct ami_big_diskarray *p; /* struct too large for stack */
	int i, s, t, off;
	int error = 0;
	struct ami_progress perc;
	u_int8_t bgi[5]; /* 40 LD, 1 bit per LD if BGI is active */

	p = malloc(sizeof *p, M_DEVBUF, M_NOWAIT);
	if (!p)
		return (ENOMEM);

	if (ami_mgmt(sc, AMI_FCOP, AMI_FC_RDCONF, 0, 0, sizeof *p, p)) {
		error = EINVAL;
		goto bail;
	}

	if (bv->bv_volid >= p->ada_nld) {
		error = ami_vol(sc, bv, p);
		goto bail;
	}

	i = bv->bv_volid;

	switch (p->ald[i].adl_status) {
	case AMI_RDRV_OFFLINE:
		bv->bv_status = BIOC_SVOFFLINE;
		break;

	case AMI_RDRV_DEGRADED:
		bv->bv_status = BIOC_SVDEGRADED;
		break;

	case AMI_RDRV_OPTIMAL:
		bv->bv_status = BIOC_SVONLINE;
		bv->bv_percent = -1;

		/* get BGI progress here and over-ride status if so */
		memset(bgi, 0, sizeof bgi);
		if (ami_mgmt(sc, AMI_MISC, AMI_GET_BGI, 0, 0, sizeof bgi, &bgi))
			break;

		if ((bgi[i / 8] & (1 << i % 8)) == 0)
			break;

		if (!ami_mgmt(sc, AMI_GCHECKPROGR, i, 0, 0, sizeof perc, &perc))
		    	if (perc.apr_progress < 100) {
				bv->bv_status = BIOC_SVSCRUB;
				bv->bv_percent = perc.apr_progress >= 100 ? -1 :
				    perc.apr_progress;
			}
		break;

	default:
		bv->bv_status = BIOC_SVINVALID;
	}

	/* over-ride status if a pd is in rebuild status for this ld */
	for (s = 0; s < p->ald[i].adl_spandepth; s++)
		for (t = 0; t < p->ald[i].adl_nstripes; t++) {
			off = p->ald[i].asp[s].adv[t].add_channel *
			    AMI_MAX_TARGET +
			    p->ald[i].asp[s].adv[t].add_target;

			if (p->apd[off].adp_ostatus != AMI_PD_RBLD)
				continue;

			/* get rebuild progress here */
			bv->bv_status = BIOC_SVREBUILD;
			if (ami_mgmt(sc, AMI_GRBLDPROGR,
			    p->ald[i].asp[s].adv[t].add_channel,
			    p->ald[i].asp[s].adv[t].add_target, 0,
			    sizeof perc, &perc))
				bv->bv_percent = -1;
			else
				bv->bv_percent = perc.apr_progress >= 100 ? -1 :
				    perc.apr_progress;

			/* XXX fix this, we should either use lowest percentage
			 * of all disks in rebuild state or an average
			 */
			break;
		}

	bv->bv_size = 0;
	bv->bv_level = p->ald[i].adl_raidlvl;
	bv->bv_nodisk = 0;

	for (s = 0; s < p->ald[i].adl_spandepth; s++) {
		for (t = 0; t < p->ald[i].adl_nstripes; t++)
			bv->bv_nodisk++;

		switch (bv->bv_level) {
		case 0:
			bv->bv_size += p->ald[i].asp[s].ads_length *
			    p->ald[i].adl_nstripes;
			break;

		case 1:
			bv->bv_size += p->ald[i].asp[s].ads_length;
			break;

		case 5:
			bv->bv_size += p->ald[i].asp[s].ads_length *
			    (p->ald[i].adl_nstripes - 1);
			break;
		}
	}

	if (p->ald[i].adl_spandepth > 1)
		bv->bv_level *= 10;

	bv->bv_size *= (u_quad_t)512;

	strlcpy(bv->bv_dev, sc->sc_hdr[i].dev, sizeof(bv->bv_dev));
	
bail:
	free(p, M_DEVBUF);

	return (error);
}

int
ami_ioctl_disk(struct ami_softc *sc, struct bioc_disk *bd)
{
	struct scsi_inquiry_data inqbuf;
	struct scsi_inquiry_vpd vpdbuf;
	struct ami_big_diskarray *p; /* struct too large for stack */
	int i, s, t, d;
	int off;
	int error = 0;
	u_int16_t ch, tg;

	p = malloc(sizeof *p, M_DEVBUF, M_NOWAIT);
	if (!p)
		return (ENOMEM);

	if (ami_mgmt(sc, AMI_FCOP, AMI_FC_RDCONF, 0, 0, sizeof *p, p)) {
		error = EINVAL;
		goto bail;
	}

	if (bd->bd_volid >= p->ada_nld) {
		error = ami_disk(sc, bd, p);
		goto bail;
	}

	i = bd->bd_volid;
	error = EINVAL;

	for (s = 0, d = 0; s < p->ald[i].adl_spandepth; s++)
		for (t = 0; t < p->ald[i].adl_nstripes; t++) {
			if (d != bd->bd_diskid) {
				d++;
				continue;
			}

			off = p->ald[i].asp[s].adv[t].add_channel *
			    AMI_MAX_TARGET +
			    p->ald[i].asp[s].adv[t].add_target;

			switch (p->apd[off].adp_ostatus) {
			case AMI_PD_UNCNF:
				bd->bd_status = BIOC_SDUNUSED;
				break;

			case AMI_PD_ONLINE:
				bd->bd_status = BIOC_SDONLINE;
				break;

			case AMI_PD_FAILED:
				bd->bd_status = BIOC_SDFAILED;
				break;

			case AMI_PD_RBLD:
				bd->bd_status = BIOC_SDREBUILD;
				break;

			case AMI_PD_HOTSPARE:
				bd->bd_status = BIOC_SDHOTSPARE;
				break;

			default:
				bd->bd_status = BIOC_SDINVALID;
			}

			bd->bd_size = (u_quad_t)p->apd[off].adp_size *
			    (u_quad_t)512;

			ch = p->ald[i].asp[s].adv[t].add_target >> 4;
			tg = p->ald[i].asp[s].adv[t].add_target & 0x0f;

			if (!ami_drv_inq(sc, ch, tg, 0, &inqbuf)) {
				char vend[8+16+4+1];

				bcopy(inqbuf.vendor, vend, sizeof vend - 1);

				vend[sizeof vend - 1] = '\0';
				strlcpy(bd->bd_vendor, vend,
				    sizeof(bd->bd_vendor));
			}

			if (!ami_drv_inq(sc, ch, tg, 0x80, &vpdbuf)) {
				char ser[32 + 1];

				bcopy(vpdbuf.serial, ser, sizeof ser - 1);

				ser[sizeof ser - 1] = '\0';
				if (vpdbuf.page_length < sizeof ser)
					ser[vpdbuf.page_length] = '\0';
				strlcpy(bd->bd_serial, ser,
				    sizeof(bd->bd_serial));
			}

			bd->bd_channel = ch;
			bd->bd_target = tg;

			strlcpy(bd->bd_procdev, sc->sc_rawsoftcs[ch].sc_procdev,
			    sizeof(bd->bd_procdev));

			error = 0;
			goto bail;
		}

	/* XXX if we reach this do dedicated hotspare magic*/
bail:
	free(p, M_DEVBUF);

	return (error);
}

int ami_ioctl_alarm(struct ami_softc *sc, struct bioc_alarm *ba)
{
	int error = 0;
	u_int8_t func, ret;

	switch(ba->ba_opcode) {
	case BIOC_SADISABLE:
		func = AMI_SPKR_OFF;
		break;

	case BIOC_SAENABLE:
		func = AMI_SPKR_ON;
		break;

	case BIOC_SASILENCE:
		func = AMI_SPKR_SHUT;
		break;

	case BIOC_GASTATUS:
		func = AMI_SPKR_GVAL;
		break;

	case BIOC_SATEST:
		func = AMI_SPKR_TEST;
		break;

	default:
		AMI_DPRINTF(AMI_D_IOCTL, ("%s: biocalarm invalid opcode %x\n",
		    DEVNAME(sc), ba->ba_opcode));
		return (EINVAL);
	}

	if (ami_mgmt(sc, AMI_SPEAKER, func, 0, 0, sizeof ret, &ret))
		error = EINVAL;
	else
		if (ba->ba_opcode == BIOC_GASTATUS)
			ba->ba_status = ret;
		else
			ba->ba_status = 0;

	return (error);
}

int
ami_ioctl_setstate(struct ami_softc *sc, struct bioc_setstate *bs)
{
	struct scsi_inquiry_data inqbuf;
	int func;
	int off;

	switch (bs->bs_status) {
	case BIOC_SSONLINE:
		func = AMI_STATE_ON;
		break;

	case BIOC_SSOFFLINE:
		func = AMI_STATE_FAIL;
		break;

	case BIOC_SSHOTSPARE:
		off = bs->bs_channel * AMI_MAX_TARGET + bs->bs_target;

		if (ami_drv_inq(sc, bs->bs_channel, bs->bs_target, 0,
		    &inqbuf))
			return (EINVAL);

		func = AMI_STATE_SPARE;
		break;

	default:
		AMI_DPRINTF(AMI_D_IOCTL, ("%s: biocsetstate invalid opcode %x\n"
		    , DEVNAME(sc), bs->bs_status));
		return (EINVAL);
	}

	if (ami_mgmt(sc, AMI_CHSTATE, bs->bs_channel, bs->bs_target, func,
	    0, NULL))
		return (EINVAL);

	return (0);
}

int
ami_create_sensors(struct ami_softc *sc)
{
	struct device *dev;
	struct scsibus_softc *ssc;
	int i;

	TAILQ_FOREACH(dev, &alldevs, dv_list) {
		if (dev->dv_parent != &sc->sc_dev)
			continue;

		/* check if this is the scsibus for the logical disks */
		ssc = (struct scsibus_softc *)dev;
		if (ssc->adapter_link == &sc->sc_link)
			break;
	}

	if (ssc == NULL)
		return (1);

	sc->sc_sensors = malloc(sizeof(struct ksensor) * sc->sc_nunits,
	    M_DEVBUF, M_WAITOK);
	if (sc->sc_sensors == NULL)
		return (1);
	bzero(sc->sc_sensors, sizeof(struct ksensor) * sc->sc_nunits);	

	strlcpy(sc->sc_sensordev.xname, DEVNAME(sc),
	    sizeof(sc->sc_sensordev.xname));

	for (i = 0; i < sc->sc_nunits; i++) {
		if (ssc->sc_link[i][0] == NULL)
			goto bad;

		dev = ssc->sc_link[i][0]->device_softc;

		sc->sc_sensors[i].type = SENSOR_DRIVE;
		sc->sc_sensors[i].status = SENSOR_S_UNKNOWN;

		strlcpy(sc->sc_sensors[i].desc, dev->dv_xname,
		    sizeof(sc->sc_sensors[i].desc));

		sensor_attach(&sc->sc_sensordev, &sc->sc_sensors[i]);
	}

	sc->sc_bd = malloc(sizeof(*sc->sc_bd), M_DEVBUF, M_WAITOK);
	if (sc->sc_bd == NULL)
		goto bad;

	if (sensor_task_register(sc, ami_refresh_sensors, 10) != 0)
		goto freebd;

	sensordev_install(&sc->sc_sensordev);

	return (0);

freebd:
	free(sc->sc_bd, M_DEVBUF);
bad:
	free(sc->sc_sensors, M_DEVBUF);

	return (1);
}

void
ami_refresh_sensors(void *arg)
{
	struct ami_softc *sc = arg;
	int i;

	if (ami_mgmt(sc, AMI_FCOP, AMI_FC_RDCONF, 0, 0, sizeof(*sc->sc_bd),
	    sc->sc_bd))
		return;

	for (i = 0; i < sc->sc_nunits; i++) {
		switch (sc->sc_bd->ald[i].adl_status) {
		case AMI_RDRV_OFFLINE:
			sc->sc_sensors[i].value = SENSOR_DRIVE_FAIL;
			sc->sc_sensors[i].status = SENSOR_S_CRIT;
			break;

		case AMI_RDRV_DEGRADED:
			sc->sc_sensors[i].value = SENSOR_DRIVE_PFAIL;
			sc->sc_sensors[i].status = SENSOR_S_WARN;
			break;

		case AMI_RDRV_OPTIMAL:
			sc->sc_sensors[i].value = SENSOR_DRIVE_ONLINE;
			sc->sc_sensors[i].status = SENSOR_S_OK;
			break;

		default:
			sc->sc_sensors[i].value = 0; /* unknown */
			sc->sc_sensors[i].status = SENSOR_S_UNKNOWN;
		}
	}
}
#endif /* NBIO > 0 */

#ifdef AMI_DEBUG
void
ami_print_mbox(struct ami_iocmd *mbox)
{
	int i;

	printf("acc_cmd: %d  aac_id: %d  acc_busy: %d  acc_nstat: %d",
	    mbox->acc_cmd, mbox->acc_id, mbox->acc_busy, mbox->acc_nstat);
	printf("acc_status: %d  acc_poll: %d  acc_ack: %d\n",
	    mbox->acc_status, mbox->acc_poll, mbox->acc_ack);

	printf("acc_cmplidl: ");
	for (i = 0; i < AMI_MAXSTATACK; i++) {
		printf("[%d] = %d  ", i, mbox->acc_cmplidl[i]);
	}

	printf("\n");
}
#endif /* AMI_DEBUG */
