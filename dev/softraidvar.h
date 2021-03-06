/* $OpenBSD: softraidvar.h,v 1.6 2007/04/11 22:05:09 marco Exp $ */
/*
 * Copyright (c) 2006 Marco Peereboom <sro@peereboom.us>
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

#include <dev/biovar.h>

#include <sys/buf.h>
#include <sys/queue.h>
#include <sys/rwlock.h>

#include <scsi/scsi_all.h>
#include <scsi/scsi_disk.h>
#include <scsi/scsiconf.h>

#define DEVNAME(_s)     ((_s)->sc_dev.dv_xname)

/* #define SR_DEBUG */
#ifdef SR_DEBUG
extern u_int32_t		sr_debug;
#define DPRINTF(x...)		do { if (sr_debug) printf(x); } while(0)
#define DNPRINTF(n,x...)	do { if (sr_debug & n) printf(x); } while(0)
#define	SR_D_CMD		0x0001
#define	SR_D_INTR		0x0002
#define	SR_D_MISC		0x0004
#define	SR_D_IOCTL		0x0008
#define	SR_D_CCB		0x0010
#define	SR_D_WU			0x0020
#define	SR_D_META		0x0040
#define	SR_D_DIS		0x0080
#define	SR_D_STATE		0x0100
#else
#define DPRINTF(x...)
#define DNPRINTF(n,x...)
#endif

#define	SR_MAXFER		MAXPHYS
#define	SR_MAX_LD		1
#define	SR_MAX_CMDS		16
#define	SR_MAX_STATES		7

/* forward define to prevent dependency goo */
struct sr_softc;

#define SR_UUID_MAX		4
struct sr_uuid {
	u_int32_t		sui_id[SR_UUID_MAX];
} __packed;

struct sr_ccb {
	struct buf		ccb_buf;	/* MUST BE FIRST!! */

	struct sr_workunit	*ccb_wu;
	struct sr_discipline	*ccb_dis;

	int			ccb_target;
	int			ccb_state;
#define SR_CCB_FREE		0
#define SR_CCB_INPROGRESS	1
#define SR_CCB_OK		2
#define SR_CCB_FAILED		3

	TAILQ_ENTRY(sr_ccb)	ccb_link;
} __packed;

TAILQ_HEAD(sr_ccb_list, sr_ccb);

struct sr_workunit {
	struct scsi_xfer	*swu_xs;
	struct sr_discipline	*swu_dis;

	int			swu_state;
#define SR_WU_FREE		0
#define SR_WU_INPROGRESS	1
#define SR_WU_OK		2
#define SR_WU_FAILED		3
#define SR_WU_PARTIALLYFAILED	4
#define SR_WU_DEFERRED		5
#define SR_WU_PENDING		6

	/* workunit io range */
	daddr64_t		swu_blk_start;
	daddr64_t		swu_blk_end;

	/* in flight totals */
	u_int32_t		swu_ios_complete;
	u_int32_t		swu_ios_failed;
	u_int32_t		swu_ios_succeeded;

	/* number of ios that makes up the whole work unit */
	u_int32_t		swu_io_count;

	/* colliding wu */
	struct sr_workunit	*swu_collider;

	/* all ios that make up this workunit */
	struct sr_ccb_list	swu_ccb;

	TAILQ_ENTRY(sr_workunit) swu_link;
};

TAILQ_HEAD(sr_wu_list, sr_workunit);

#define SR_META_PD		1   /* only 1 PD per metadata block */
#define SR_META_VD		1   /* only 1 VD per metadata block */
#define SR_META_DATA		1   /* only 1 metadata block per disk */
#define SR_META_FUDGE		32  /* save some space at the end of chunk */
#define SR_META_VERSION		1 /* bump when sr_metadata changes */
struct sr_metadata {
	/* do not change order of ssd_magic, ssd_version and ssd_big_endian */
	u_int64_t		ssd_magic;	/* magic id */
#define	SR_MAGIC		0x4d4152436372616dllu
	u_int8_t		ssd_version;	/* meta data version */
	u_int8_t		ssd_big_endian;	/* set if big endian */
	u_int8_t		ssd_pad[2];

	/* meta-data */
	u_int32_t		ssd_checksum;	/* xor of the structure */
	u_int32_t		ssd_size;	/* sizeof(sr_metadata) */
	u_int32_t		ssd_ondisk;	/* on disk version counter */
	struct sr_uuid	ssd_uuid;	/* unique identifier */

	/* virtual disk data */
	u_int32_t		ssd_vd_ver;	/* vd structure version */
	u_int32_t		ssd_vd_size;	/* vd structure size */
	u_int32_t		ssd_vd_chk;	/* vd structure xor */

	/* chunk data */
	u_int32_t		ssd_chunk_ver;	/* chunk structure version */
	u_int32_t		ssd_chunk_no;	/* number of chunks */
	u_int32_t		ssd_chunk_size;	/* chunk structure size */
	u_int32_t		ssd_chunk_chk;	/* chunk structure xor */
} __packed;

#define SR_CHUNK_VERSION	1	/* bump when sr_chunk_meta changes */
struct sr_chunk_meta {
	int			scm_volid;	/* vd we belong to */
	int			scm_chunk_id;	/* chunk id */
	char			scm_devname[32];/* /dev/XXXXX */
	int			scm_status;	/* use bio bioc_disk status */
	u_quad_t		scm_size;	/* size of partition */
	u_quad_t		scm_coerced_size; /* coerced size of part */
	struct sr_uuid		scm_uuid;	/* unique identifier */
} __packed;

struct sr_chunk {
	struct sr_chunk_meta	src_meta;	/* chunk meta data */

	/* runtime data */
	struct vnode		*src_dev_vn;	/* vnode */
	dev_t			src_dev_mm;	/* major/minor */

	SLIST_ENTRY(sr_chunk)	src_link;
};

SLIST_HEAD(sr_chunk_head, sr_chunk);

#define SR_VOL_VERSION	1	/* bump when sr_vol_meta changes */
struct sr_vol_meta {
	int			svm_volid;	/* volume id */
	int			svm_status; 	/* use bioc_vol status */
	int			svm_flags;	/* flags */
#define SR_VDF_DIRTY		0x01
	u_quad_t		svm_size;	/* virtual disk size */
	int			svm_level;	/* raid level */
	char			svm_vendor[8];	/* scsi vendor */
	char			svm_product[16];/* scsi product */
	char			svm_revision[4];/* scsi revision */
	char			svm_pad[4];
	char			svm_devname[32];/* /dev/XXXXX */
	int			svm_no_chunk;	/* number of chunks */
	struct sr_uuid 		svm_uuid;	/* volume unique identifier */
} __packed;

struct sr_volume {
	struct sr_vol_meta	sv_meta;	/* meta data */

	/* runtime data */
	struct sr_chunk_head	sv_chunk_list;	/* linked list of all chunks */
	struct sr_chunk		**sv_chunks;	/* array to same chunks */
};

/* RAID 1 */
#define SR_RAID1_NOWU		16
struct sr_raid1 {
	u_int32_t		sr1_counter;
};

struct sr_discipline {
	struct sr_softc		*sd_sc;		/* link back to sr softc */
	u_int8_t		sd_type;	/* type of discipline */
#define	SR_MD_RAID0		0
#define	SR_MD_RAID1		1
#define	SR_MD_RAID5		2
#define	SR_MD_CACHE		3
	char			sd_name[10];	/* human readable dis name */
	u_int8_t		sd_scsibus;	/* scsibus discipline uses */
	struct scsi_link	sd_link;	/* link to midlayer */

	union {
	    struct sr_raid1	mdd_raid1;
	}			sd_dis_specific;/* dis specific members */
#define mds			sd_dis_specific

	/* discipline volume */
	struct sr_volume	sd_vol;		/* volume associated */

	/* discipline resources */
	struct sr_ccb		*sd_ccb;
	struct sr_ccb_list	sd_ccb_freeq;
	u_int32_t		sd_max_ccb_per_wu;

	struct sr_workunit	*sd_wu;		/* all workunits */
	u_int32_t		sd_max_wu;

	struct sr_wu_list	sd_wu_freeq;	/* free wu queue */
	struct sr_wu_list	sd_wu_pendq;	/* pending wu queue */
	struct sr_wu_list	sd_wu_defq;	/* deferred wu queue */

	/* discipline functions */
	int			(*sd_alloc_resources)(struct sr_discipline *);
	int			(*sd_assemble_volume)(void *);
	int			(*sd_bringup_volume)(void *);
	int			(*sd_shutdown_volume)(void *);
	int			(*sd_free_resources)(struct sr_discipline *);
	int			(*sd_quiesce_io)(struct sr_discipline *);
	void			(*sd_set_chunk_state)(struct sr_discipline *,
				    int, int);
	void			(*sd_set_vol_state)(struct sr_discipline *);

	/* SCSI emulation */
	struct scsi_sense_data	sd_scsi_sense;
	int			(*sd_scsi_rw)(struct sr_workunit *);
	int			(*sd_scsi_sync)(struct sr_workunit *);
	int			(*sd_scsi_tur)(struct sr_workunit *);
	int			(*sd_scsi_start_stop)(struct sr_workunit *);
	int			(*sd_scsi_inquiry)(struct sr_workunit *);
	int			(*sd_scsi_read_cap)(struct sr_workunit *);
	int			(*sd_scsi_req_sense)(struct sr_workunit *);
};

struct sr_softc {
	struct device		sc_dev;

	int			(*sc_ioctl)(struct device *, u_long, caddr_t);

	struct rwlock		sc_lock;
	/*
	 * during scsibus attach this is the discipline that is in use
	 * this variable is protected by sc_lock and splhigh
	 */
	struct sr_discipline	*sc_attach_dis;
	/*
	 * XXX expensive, alternative would be nice but has to be cheap
	 * since the scsibus lookup happens on each IO
	 */
#define SR_MAXSCSIBUS		256
	struct sr_discipline	*sc_dis[SR_MAXSCSIBUS]; /* scsibus is u_int8_t */
};
