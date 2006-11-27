/*	$OpenBSD: ips.c,v 1.1 2006/11/27 16:47:05 grange Exp $	*/

/*
 * Copyright (c) 2006 Alexander Yurchenko <grange@openbsd.org>
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

/*
 * IBM ServeRAID controller driver.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/buf.h>
#include <sys/device.h>
#include <sys/malloc.h>

#include <machine/bus.h>

#include <scsi/scsi_all.h>
#include <scsi/scsi_disk.h>
#include <scsi/scsiconf.h>

#include <dev/pci/pcidevs.h>
#include <dev/pci/pcireg.h>
#include <dev/pci/pcivar.h>

/*
 * Register definitions.
 */
#define IPS_BAR0		0x10	/* I/O space base address */
#define IPS_BAR1		0x14	/* I/O space base address */

#define IPS_MORPHEUS_OISR	0x0030	/* outbound IRQ status */
#define	IPS_MORPHEUS_OISR_CMD		(1 << 3)
#define IPS_MORPHEUS_OIMR	0x0034	/* outbound IRQ mask */
#define IPS_MORPHEUS_IQPR	0x0040	/* inbound queue port */
#define IPS_MORPHEUS_OQPR	0x0044	/* outbound queue port */

/* Commands */
#define IPS_CMD_READ		0x02
#define IPS_CMD_WRITE		0x03
#define IPS_CMD_ADAPTERINFO	0x05
#define IPS_CMD_DRIVEINFO	0x19

#define IPS_MAXCMDSZ		256	/* XXX: for now */
#define IPS_MAXDATASZ		64 * 1024
#define IPS_MAXSEGS		32

#define IPS_MAXDRIVES		8
#define IPS_MAXCHANS		4
#define IPS_MAXTARGETS		15
#define IPS_MAXCMDS		32

/* Command frames */
struct ips_cmd_adapterinfo {
	u_int8_t	command;
	u_int8_t	id;
	u_int8_t	reserve1;
	u_int8_t	commandtype;
	u_int32_t	reserve2;
	u_int32_t	buffaddr;
	u_int32_t	reserve3;
} __packed;

struct ips_cmd_driveinfo {
	u_int8_t	command;
	u_int8_t	id;
	u_int8_t	drivenum;
	u_int8_t	reserve1;
	u_int32_t	reserve2;
	u_int32_t	buffaddr;
	u_int32_t	reserve3;
} __packed;

struct ips_cmd_io {
	u_int8_t	command;
	u_int8_t	id;
	u_int8_t	drivenum;
	u_int8_t	segnum;
	u_int32_t	lba;
	u_int32_t	buffaddr;
	u_int16_t	length;
	u_int16_t	reserve1;
} __packed;

/* Data frames */
struct ips_adapterinfo {
	u_int8_t	drivecount;
	u_int8_t	miscflags;
	u_int8_t	SLTflags;
	u_int8_t	BSTflags;
	u_int8_t	pwr_chg_count;
	u_int8_t	wrong_addr_count;
	u_int8_t	unident_count;
	u_int8_t	nvram_dev_chg_count;
	u_int8_t	codeblock_version[8];
	u_int8_t	bootblock_version[8];
	u_int32_t	drive_sector_count[IPS_MAXDRIVES];
	u_int8_t	max_concurrent_cmds;
	u_int8_t	max_phys_devices;
	u_int16_t	flash_prog_count;
	u_int8_t	defunct_disks;
	u_int8_t	rebuildflags;
	u_int8_t	offline_drivecount;
	u_int8_t	critical_drivecount;
	u_int16_t	config_update_count;
	u_int8_t	blockedflags;
	u_int8_t	psdn_error;
	u_int16_t	addr_dead_disk[IPS_MAXCHANS][IPS_MAXTARGETS];
} __packed;

struct ips_drive {
	u_int8_t	drivenum;
	u_int8_t	merge_id;
	u_int8_t	raid_lvl;
	u_int8_t	state;
	u_int32_t	sector_count;
} __packed;

struct ips_driveinfo {
	u_int8_t	drivecount;
	u_int8_t	reserve1;
	u_int16_t	reserve2;
	struct ips_drive drives[IPS_MAXDRIVES];
} __packed;

/* I/O access helper macros */
#define IPS_READ_4(s, r) \
	bus_space_read_4((s)->sc_iot, (s)->sc_ioh, (r))
#define IPS_WRITE_4(s, r, v) \
	bus_space_write_4((s)->sc_iot, (s)->sc_ioh, (r), (v))

struct ccb {
	int			c_run;
	struct dmamem *		c_dm;
	struct scsi_xfer *	c_xfer;
};

struct dmamem {
	bus_dma_tag_t		dm_tag;
	bus_dmamap_t		dm_map;
	bus_dma_segment_t	dm_seg;
	bus_size_t		dm_size;
	void *			dm_kva;
};

struct ips_softc {
	struct device		sc_dev;

	struct scsi_link	sc_scsi_link;
	struct scsibus_softc *	sc_scsi_bus;

	pci_chipset_tag_t	sc_pc;
	pcitag_t		sc_tag;

	bus_space_tag_t		sc_iot;
	bus_space_handle_t	sc_ioh;
	bus_dma_tag_t		sc_dmat;

	struct dmamem *		sc_cmdm;
	struct ccb		sc_ccb[IPS_MAXCMDS];

	void *			sc_ih;

	void			(*sc_exec)(struct ips_softc *);
	void			(*sc_inten)(struct ips_softc *);
	int			(*sc_intr)(void *);

	struct ips_adapterinfo	sc_ai;
	struct ips_driveinfo	sc_di;
};

int	ips_match(struct device *, void *, void *);
void	ips_attach(struct device *, struct device *, void *);

int	ips_scsi_cmd(struct scsi_xfer *);
int	ips_scsi_io(struct scsi_xfer *);
int	ips_scsi_ioctl(struct scsi_link *, u_long, caddr_t, int,
	    struct proc *);
void	ips_scsi_minphys(struct buf *);

int	ips_getadapterinfo(struct ips_softc *, struct ips_adapterinfo *);
int	ips_getdriveinfo(struct ips_softc *, struct ips_driveinfo *);

void	ips_copperhead_exec(struct ips_softc *);
void	ips_copperhead_inten(struct ips_softc *);
int	ips_copperhead_intr(void *);

void	ips_morpheus_exec(struct ips_softc *);
void	ips_morpheus_inten(struct ips_softc *);
int	ips_morpheus_intr(void *);

struct dmamem *	ips_dmamem_alloc(bus_dma_tag_t, bus_size_t);
void		ips_dmamem_free(struct dmamem *);

struct cfattach ips_ca = {
	sizeof(struct ips_softc),
	ips_match,
	ips_attach
};

struct cfdriver ips_cd = {
	NULL, "ips", DV_DULL
};

static const struct pci_matchid ips_ids[] = {
	{ PCI_VENDOR_IBM,	PCI_PRODUCT_IBM_SERVERAID },
	{ PCI_VENDOR_IBM,	PCI_PRODUCT_IBM_SERVERAID2 },
	{ PCI_VENDOR_ADP2,	PCI_PRODUCT_ADP2_SERVERAID }
};

static struct scsi_adapter ips_scsi_adapter = {
	ips_scsi_cmd,
	ips_scsi_minphys,
	NULL,
	NULL,
	ips_scsi_ioctl
};

static struct scsi_device ips_scsi_device = {
	NULL,
	NULL,
	NULL,
	NULL
};

int
ips_match(struct device *parent, void *match, void *aux)
{
	return (pci_matchbyid(aux, ips_ids,
	    sizeof(ips_ids) / sizeof(ips_ids[0])));
}

void
ips_attach(struct device *parent, struct device *self, void *aux)
{
	struct ips_softc *sc = (struct ips_softc *)self;
	struct pci_attach_args *pa = aux;
	int bar;
	pcireg_t maptype;
	bus_size_t iosize;
	pci_intr_handle_t ih;
	const char *intrstr;
	int i;

	sc->sc_pc = pa->pa_pc;
	sc->sc_tag = pa->pa_tag;
	sc->sc_dmat = pa->pa_dmat;

	/* Identify the chipset */
	switch (PCI_PRODUCT(pa->pa_id)) {
	case PCI_PRODUCT_IBM_SERVERAID:
		printf(": Copperhead");
		sc->sc_exec = ips_copperhead_exec;
		sc->sc_inten = ips_copperhead_inten;
		sc->sc_intr = ips_copperhead_intr;
		break;
	case PCI_PRODUCT_IBM_SERVERAID2:
	case PCI_PRODUCT_ADP2_SERVERAID:
		printf(": Morpheus");
		sc->sc_exec = ips_morpheus_exec;
		sc->sc_inten = ips_morpheus_inten;
		sc->sc_intr = ips_morpheus_intr;
		break;
	}

	/* Map I/O space */
	if (PCI_PRODUCT(pa->pa_id) == PCI_PRODUCT_IBM_SERVERAID)
		bar = IPS_BAR1;
	else
		bar = IPS_BAR0;
	maptype = pci_mapreg_type(sc->sc_pc, sc->sc_tag, bar);
	if (pci_mapreg_map(pa, bar, maptype, 0, &sc->sc_iot, &sc->sc_ioh,
	    NULL, &iosize, 0)) {
		printf(": can't map I/O space\n");
		return;
	}

	/* Allocate command DMA buffer */
	if ((sc->sc_cmdm = ips_dmamem_alloc(sc->sc_dmat,
	    IPS_MAXCMDSZ)) == NULL) {
		printf(": can't alloc command DMA buffer\n");
		goto fail1;
	}

	for (i = 0; i < IPS_MAXCMDS; i++)
		if ((sc->sc_ccb[i].c_dm = ips_dmamem_alloc(sc->sc_dmat,
		    IPS_MAXDATASZ)) == NULL) {
			printf(": can't alloc ccb\n");
			goto fail2;
		}

	/* Get adapter info */
	if (ips_getadapterinfo(sc, &sc->sc_ai)) {
		printf(": can't get adapter info\n");
		goto fail2;
	}

	/* Get logical drives info */
	if (ips_getdriveinfo(sc, &sc->sc_di)) {
		printf(": can't get drives info\n");
		goto fail2;
	}

	/* Install interrupt handler */
	if (pci_intr_map(pa, &ih)) {
		printf(": can't map interrupt\n");
		goto fail2;
	}
	intrstr = pci_intr_string(sc->sc_pc, ih);
	if ((sc->sc_ih = pci_intr_establish(sc->sc_pc, ih, IPL_BIO,
	    sc->sc_intr, sc, sc->sc_dev.dv_xname)) == NULL) {
		printf(": can't establish interrupt");
		if (intrstr != NULL)
			printf(" at %s", intrstr);
		printf("\n");
		goto fail2;
	}
	printf(", %s\n", intrstr);

	/* Enable interrupts */
	(*sc->sc_inten)(sc);

	/* Attach SCSI bus */
	sc->sc_scsi_link.openings = IPS_MAXCMDS;	/* XXX: for now */
	sc->sc_scsi_link.adapter_target = IPS_MAXTARGETS;
	sc->sc_scsi_link.adapter_buswidth = IPS_MAXTARGETS;
	sc->sc_scsi_link.device = &ips_scsi_device;
	sc->sc_scsi_link.adapter = &ips_scsi_adapter;
	sc->sc_scsi_link.adapter_softc = sc;

	sc->sc_scsi_bus = (struct scsibus_softc *)config_found(self,
	    &sc->sc_scsi_link, scsiprint);

	return;
fail2:
	ips_dmamem_free(sc->sc_cmdm);
fail1:
	bus_space_unmap(sc->sc_iot, sc->sc_ioh, iosize);
}

int
ips_scsi_cmd(struct scsi_xfer *xs)
{
	struct scsi_link *link = xs->sc_link;
	struct ips_softc *sc = link->adapter_softc;
	struct scsi_inquiry_data *inq;
	struct scsi_read_cap_data *cap;
	struct scsi_sense_data *sns;
	int target = link->target;
	int s;

	if (target >= sc->sc_di.drivecount || link->lun != 0)
		goto error;

	switch (xs->cmd->opcode) {
	case READ_BIG:
	case READ_COMMAND:
	case WRITE_BIG:
	case WRITE_COMMAND:
		return (ips_scsi_io(xs));
	case INQUIRY:
		inq = (void *)xs->data;
		bzero(inq, sizeof(*inq));
		inq->device = T_DIRECT;
		inq->version = 2;
		inq->response_format = 2;
		inq->additional_length = 32;
		strlcpy(inq->vendor, "IBM", sizeof(inq->vendor));
		snprintf(inq->product, sizeof(inq->product),
		    "ServeRAID LD %02d", target);
		goto done;
	case READ_CAPACITY:
		cap = (void *)xs->data;
		bzero(cap, sizeof(*cap));
		_lto4b(sc->sc_di.drives[target].sector_count - 1, cap->addr);
		_lto4b(512, cap->length);
		goto done;
	case REQUEST_SENSE:
		sns = (void *)xs->data;
		bzero(sns, sizeof(*sns));
		sns->error_code = 0x70;
		sns->flags = SKEY_NO_SENSE;
		goto done;
	case PREVENT_ALLOW:
	case START_STOP:
	case TEST_UNIT_READY:
		return (COMPLETE);
	}

error:
	xs->error = XS_DRIVER_STUFFUP;
done:
	s = splbio();
	scsi_done(xs);
	splx(s);
	return (COMPLETE);
}

int
ips_scsi_io(struct scsi_xfer *xs)
{
	struct scsi_link *link = xs->sc_link;
	struct ips_softc *sc = link->adapter_softc;
	struct scsi_rw *rw;
	struct scsi_rw_big *rwb;
	struct ips_cmd_io *cmd;
	u_int32_t blkno, blkcnt;
	int i;
	struct ccb *ccb;

	for (i = 0; i < IPS_MAXCMDS; i++) {
		ccb = &sc->sc_ccb[i];
		if (!ccb->c_run)
			break;
	}
	ccb->c_run = 1;
	if (xs->flags & SCSI_DATA_OUT)
		memcpy(ccb->c_dm->dm_kva, xs->data, xs->datalen);
	ccb->c_xfer = xs;

	if (xs->cmd->opcode == READ_COMMAND ||
	    xs->cmd->opcode == WRITE_COMMAND) {
		rw = (void *)xs->cmd;
		blkno = _3btol(rw->addr) & (SRW_TOPADDR << 16 | 0xffff);
		blkcnt = rw->length > 0 ? rw->length : 0x100;
	} else {
		rwb = (void *)xs->cmd;
		blkno = _4btol(rwb->addr);
		blkcnt = _2btol(rwb->length);
	}

	cmd = sc->sc_cmdm->dm_kva;
	bzero(cmd, sizeof(*cmd));
	cmd->command = (xs->flags & SCSI_DATA_IN) ? IPS_CMD_READ :
	    IPS_CMD_WRITE;
	cmd->id = i;
	cmd->drivenum = link->target;
	cmd->segnum = 0;
	cmd->lba = blkno;
	cmd->buffaddr = ccb->c_dm->dm_seg.ds_addr;
	cmd->length = blkcnt;

	(*sc->sc_exec)(sc);
	return (SUCCESSFULLY_QUEUED);
}

int
ips_scsi_ioctl(struct scsi_link *link, u_long cmd, caddr_t addr, int flags,
    struct proc *p)
{
	return (ENOTTY);
}

void
ips_scsi_minphys(struct buf *bp)
{
	minphys(bp);
}

int
ips_getadapterinfo(struct ips_softc *sc, struct ips_adapterinfo *ai)
{
	struct dmamem *dm;
	struct ips_cmd_adapterinfo *cmd;

	if ((dm = ips_dmamem_alloc(sc->sc_dmat, sizeof(*ai))) == NULL)
		return (1);

	cmd = sc->sc_cmdm->dm_kva;
	bzero(cmd, sizeof(*cmd));
	cmd->command = IPS_CMD_ADAPTERINFO;
	cmd->buffaddr = dm->dm_seg.ds_addr;

	(*sc->sc_exec)(sc);
	DELAY(1000);
	(*sc->sc_intr)(sc);
	bcopy(dm->dm_kva, ai, sizeof(*ai));
	ips_dmamem_free(dm);

	return (0);
}

int
ips_getdriveinfo(struct ips_softc *sc, struct ips_driveinfo *di)
{
	struct dmamem *dm;
	struct ips_cmd_driveinfo *cmd;

	if ((dm = ips_dmamem_alloc(sc->sc_dmat, sizeof(*di))) == NULL)
		return (1);

	cmd = sc->sc_cmdm->dm_kva;
	bzero(cmd, sizeof(*cmd));
	cmd->command = IPS_CMD_DRIVEINFO;
	cmd->buffaddr = dm->dm_seg.ds_addr;

	(*sc->sc_exec)(sc);
	DELAY(1000);
	(*sc->sc_intr)(sc);
	bcopy(dm->dm_kva, di, sizeof(*di));
	ips_dmamem_free(dm);

	return (0);
}

void
ips_copperhead_exec(struct ips_softc *sc)
{
}

void
ips_copperhead_inten(struct ips_softc *sc)
{
}

int
ips_copperhead_intr(void *arg)
{
	return (0);
}

void
ips_morpheus_exec(struct ips_softc *sc)
{
	IPS_WRITE_4(sc, IPS_MORPHEUS_IQPR, sc->sc_cmdm->dm_seg.ds_addr);
}

void
ips_morpheus_inten(struct ips_softc *sc)
{
	u_int32_t reg;

	reg = IPS_READ_4(sc, IPS_MORPHEUS_OIMR);
	reg &= ~0x08;
	IPS_WRITE_4(sc, IPS_MORPHEUS_OIMR, reg);
}

int
ips_morpheus_intr(void *arg)
{
	struct ips_softc *sc = arg;
	u_int32_t reg;
	int id;
	struct scsi_xfer *xs;
	int s;
	struct ccb *ccb;

	reg = IPS_READ_4(sc, IPS_MORPHEUS_OISR);
	if (!(reg & IPS_MORPHEUS_OISR_CMD))
		return (0);

	while ((reg = IPS_READ_4(sc, IPS_MORPHEUS_OQPR)) != 0xffffffff) {
		id = (reg >> 8) & 0xff;
		ccb = &sc->sc_ccb[id];
		if (!ccb->c_run)
			continue;
		xs = ccb->c_xfer;
		if (xs->flags & SCSI_DATA_IN)
			memcpy(xs->data, ccb->c_dm->dm_kva, xs->datalen);
		xs->resid = 0;
		xs->flags |= ITSDONE;
		s = splbio();
		scsi_done(xs);
		splx(s);
		ccb->c_run = 0;
	}

	return (1);
}

struct dmamem *
ips_dmamem_alloc(bus_dma_tag_t tag, bus_size_t size)
{
	struct dmamem *dm;
	int nsegs;

	if ((dm = malloc(sizeof(*dm), M_DEVBUF, M_NOWAIT)) == NULL)
		return (NULL);

	dm->dm_tag = tag;
	dm->dm_size = size;

	if (bus_dmamap_create(tag, size, 1, size, 0,
	    BUS_DMA_NOWAIT | BUS_DMA_ALLOCNOW, &dm->dm_map))
		goto fail1;
	if (bus_dmamem_alloc(tag, size, 0, 0, &dm->dm_seg, 1, &nsegs,
	    BUS_DMA_NOWAIT))
		goto fail2;
	if (bus_dmamem_map(tag, &dm->dm_seg, 1, size, (caddr_t *)&dm->dm_kva,
	    BUS_DMA_NOWAIT))
		goto fail3;
	bzero(dm->dm_kva, size);
	if (bus_dmamap_load(tag, dm->dm_map, dm->dm_kva, size, NULL,
	    BUS_DMA_NOWAIT))
		goto fail4;

	return (dm);

fail4:
	bus_dmamem_unmap(tag, dm->dm_kva, size);
fail3:
	bus_dmamem_free(tag, &dm->dm_seg, 1);
fail2:
	bus_dmamap_destroy(tag, dm->dm_map);
fail1:
	free(dm, M_DEVBUF);
	return (NULL);
}

void
ips_dmamem_free(struct dmamem *dm)
{
	bus_dmamap_unload(dm->dm_tag, dm->dm_map);
	bus_dmamem_unmap(dm->dm_tag, dm->dm_kva, dm->dm_size);
	bus_dmamem_free(dm->dm_tag, &dm->dm_seg, 1);
	bus_dmamap_destroy(dm->dm_tag, dm->dm_map);
	free(dm, M_DEVBUF);
}