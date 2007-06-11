/* $NetBSD: dksubr.c,v 1.23 2006/07/21 16:48:47 ad Exp $ */

/*-
 * Copyright (c) 1996, 1997, 1998, 1999, 2002 The NetBSD Foundation, Inc.
 * All rights reserved.
 *
 * This code is derived from software contributed to The NetBSD Foundation
 * by Jason R. Thorpe and Roland C. Dowdeswell.
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
 *        This product includes software developed by the NetBSD
 *        Foundation, Inc. and its contributors.
 * 4. Neither the name of The NetBSD Foundation nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE NETBSD FOUNDATION, INC. AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE FOUNDATION OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */


#include <sys/param.h>
#include <sys/systm.h>
#include <sys/stat.h>
#include <sys/proc.h>
#include <sys/ioctl.h>
#include <sys/conf.h>
#include <sys/device.h>
#include <sys/disk.h>
#include <sys/disklabel.h>
#include <sys/buf.h>
#include <sys/vnode.h>
#include <sys/fcntl.h>
#include <sys/namei.h>

#include "dkvar.h"

int	dkdebug = 0;

#ifdef DEBUG
#define DKDB_FOLLOW	0x1
#define DKDB_INIT	0x2
#define DKDB_VNODE	0x4

#define IFDEBUG(x,y)		if (dkdebug & (x)) y
#define DPRINTF(x,y)		IFDEBUG(x, printf y)
#define DPRINTF_FOLLOW(y)	DPRINTF(DKDB_FOLLOW, y)
#else
#define IFDEBUG(x,y)
#define DPRINTF(x,y)
#define DPRINTF_FOLLOW(y)
#endif

#define DKLABELDEV(dev)	\
	(MAKEDISKDEV(major((dev)), DISKUNIT((dev)), RAW_PART))

static void	dk_makedisklabel(struct dk_intf *, struct dk_softc *);


void
dk_sc_init(struct dk_softc *dksc, void *osc, char *xname)
{

	memset(dksc, 0x0, sizeof(*dksc));
	dksc->sc_osc = osc;
	strncpy(dksc->sc_xname, xname, DK_XNAME_SIZE);
	dksc->sc_dkdev.dk_name = dksc->sc_xname;
}

/* ARGSUSED */
int
dk_open(struct dk_intf *di, struct dk_softc *dksc, dev_t dev,
	   int flags, int fmt, struct proc *l)
{
	struct	disklabel *lp = dksc->sc_dkdev.dk_label;
	int	part = DISKPART(dev);
	int	pmask = 1 << part;
	int	ret = 0;
	struct disk *dk = &dksc->sc_dkdev;

	DPRINTF_FOLLOW(("dk_open(%s, %p, 0x%x, 0x%x)\n",
	    di->di_dkname, dksc, dev, flags));

	if (disk_lock(dk) != 0)
		return ret;

	part = DISKPART(dev);

#ifdef __NetBSD__
	/*
	 * If there are wedges, and this is not RAW_PART, then we
	 * need to fail.
	 */
	if (part != RAW_PART) {
		ret = EBUSY;
		goto done;
	}
#endif

	pmask = 1 << part;

	/*
	 * If we're init'ed and there are no other open partitions then
	 * update the in-core disklabel.
	 */
	if ((dksc->sc_flags & DKF_INITED)) {
		if (dk->dk_openmask == 0) {
			dk_getdisklabel(di, dksc, dev, 0);
		}
		/* XXX re-discover wedges? */
	}

	/* Fail if we can't find the partition. */
	if ((part != RAW_PART) &&
	    (((dksc->sc_flags & DKF_INITED) == 0) ||
	    ((part >= lp->d_npartitions) ||
	    (lp->d_partitions[part].p_fstype == FS_UNUSED)))) {
		ret = ENXIO;
		goto done;
	}

	/* Mark our unit as open. */
	switch (fmt) {
	case S_IFCHR:
		dk->dk_copenmask |= pmask;
		break;
	case S_IFBLK:
		dk->dk_bopenmask |= pmask;
		break;
	}

	dk->dk_openmask = dk->dk_copenmask | dk->dk_bopenmask;

done:
	disk_unlock(dk);
	return ret;
}

/* ARGSUSED */
int
dk_close(struct dk_intf *di, struct dk_softc *dksc, dev_t dev,
	    int flags, int fmt, struct proc *l)
{
	int	part = DISKPART(dev);
	int	pmask = 1 << part;
	int	ret;
	struct disk *dk = &dksc->sc_dkdev;

	DPRINTF_FOLLOW(("dk_close(%s, %p, 0x%x, 0x%x)\n",
	    di->di_dkname, dksc, dev, flags));

	if (disk_lock(dk) != 0)
		return ret;

	switch (fmt) {
	case S_IFCHR:
		dk->dk_copenmask &= ~pmask;
		break;
	case S_IFBLK:
		dk->dk_bopenmask &= ~pmask;
		break;
	}
	dk->dk_openmask = dk->dk_copenmask | dk->dk_bopenmask;

	disk_unlock(dk);
	return 0;
}

void
dk_strategy(struct dk_intf *di, struct dk_softc *dksc, struct buf *bp)
{
	int	s;
	int	wlabel;
	daddr_t	blkno;

	DPRINTF_FOLLOW(("dk_strategy(%s, %p, %p)\n",
	    di->di_dkname, dksc, bp));

	if (!(dksc->sc_flags & DKF_INITED)) {
		DPRINTF_FOLLOW(("dk_stragy: not inited\n"));
		bp->b_error  = ENXIO;
		bp->b_flags |= B_ERROR;
		biodone(bp);
		return;
	}

	/* XXX look for some more errors, c.f. ld.c */

	bp->b_resid = bp->b_bcount;

	/* If there is nothing to do, then we are done */
	if (bp->b_bcount == 0) {
		biodone(bp);
		return;
	}

	wlabel = dksc->sc_flags & (DKF_WLABEL|DKF_LABELLING);
	if (DISKPART(bp->b_dev) != RAW_PART &&
	    bounds_check_with_label(bp, dksc->sc_dkdev.dk_label,
				dksc->sc_dkdev.dk_cpulabel, wlabel) <= 0) {
		biodone(bp);
		return;
	}

	blkno = bp->b_blkno;
	if (DISKPART(bp->b_dev) != RAW_PART) {
		struct partition *pp;

		pp =
		    &dksc->sc_dkdev.dk_label->d_partitions[DISKPART(bp->b_dev)];
		blkno += pp->p_offset;
	}
	bp->b_blkno = blkno;

	/*
	 * Start the unit by calling the start routine
	 * provided by the individual driver.
	 */
	s = splbio();
	BUFQ_ADD(dksc->sc_bufq, bp);
	dk_start(di, dksc);
	splx(s);
	return;
}

void
dk_start(struct dk_intf *di, struct dk_softc *dksc)
{
	struct	buf *bp;

	DPRINTF_FOLLOW(("dk_start(%s, %p)\n", di->di_dkname, dksc));

	/* Process the work queue */
	while ((bp = BUFQ_GET(dksc->sc_bufq)) != NULL) {
		if (di->di_diskstart(dksc, bp) != 0) {
			BUFQ_ADD(dksc->sc_bufq, bp);
			break;
		}
	}
}

void
dk_iodone(struct dk_intf *di, struct dk_softc *dksc)
{

	DPRINTF_FOLLOW(("dk_iodone(%s, %p)\n", di->di_dkname, dksc));

	/* We kick the queue in case we are able to get more work done */
	dk_start(di, dksc);
}

int
dk_size(struct dk_intf *di, struct dk_softc *dksc, dev_t dev)
{
	struct	disklabel *lp;
	int	is_open;
	int	part;
	int	size;

	if ((dksc->sc_flags & DKF_INITED) == 0)
		return -1;

	part = DISKPART(dev);
	is_open = dksc->sc_dkdev.dk_openmask & (1 << part);

	if (!is_open && di->di_open(dev, 0, S_IFBLK, curproc))
		return -1;

	lp = dksc->sc_dkdev.dk_label;
	if (lp->d_partitions[part].p_fstype != FS_SWAP)
		size = -1;
	else
		size = lp->d_partitions[part].p_size *
		    (lp->d_secsize / DEV_BSIZE);

	if (!is_open && di->di_close(dev, 0, S_IFBLK, curproc))
		return 1;

	return size;
}

int
dk_ioctl(struct dk_intf *di, struct dk_softc *dksc, dev_t dev,
	    u_long cmd, caddr_t data, int flag, struct proc *l)
{
	struct	disklabel *lp;
	struct	disk *dk;
	int	error = 0;

	DPRINTF_FOLLOW(("dk_ioctl(%s, %p, 0x%x, 0x%lx)\n",
	    di->di_dkname, dksc, dev, cmd));

	/* ensure that the pseudo disk is open for writes for these commands */
	switch (cmd) {
	case DIOCSDINFO:
	case DIOCWDINFO:
	case DIOCWLABEL:
		if ((flag & FWRITE) == 0) {
			return EBADF;
		}
	}

	/* ensure that the pseudo-disk is initialized for these */
	switch (cmd) {
	case DIOCGDINFO:
	case DIOCSDINFO:
	case DIOCWDINFO:
	case DIOCGPART:
	case DIOCWLABEL:
#ifdef __NetBSD__
	case DIOCGDEFLABEL:
#endif
		if ((dksc->sc_flags & DKF_INITED) == 0) {
			return ENXIO;
		}
	}

	switch (cmd) {
	case DIOCGDINFO:
		*(struct disklabel *)data = *(dksc->sc_dkdev.dk_label);
		break;

	case DIOCGPART:
		((struct partinfo *)data)->disklab = dksc->sc_dkdev.dk_label;
		((struct partinfo *)data)->part =
		    &dksc->sc_dkdev.dk_label->d_partitions[DISKPART(dev)];
		break;

	case DIOCWDINFO:
	case DIOCSDINFO:
		lp = (struct disklabel *)data;

		dk = &dksc->sc_dkdev;
		error = disk_lock(dk);
		if (error) {
			break;
		}

		dksc->sc_flags |= DKF_LABELLING;

		error = setdisklabel(dksc->sc_dkdev.dk_label,
		    lp, 0, dksc->sc_dkdev.dk_cpulabel);
		if (error == 0) {
			if (cmd == DIOCWDINFO)
				error = writedisklabel(DKLABELDEV(dev),
				    di->di_strategy, dksc->sc_dkdev.dk_label,
				    dksc->sc_dkdev.dk_cpulabel);
		}

		dksc->sc_flags &= ~DKF_LABELLING;
		disk_unlock(dk);
		break;

	case DIOCWLABEL:
		if (*(int *)data != 0)
			dksc->sc_flags |= DKF_WLABEL;
		else
			dksc->sc_flags &= ~DKF_WLABEL;
		break;

#ifdef __NetBSD__
	case DIOCGDEFLABEL:
		dk_getdefaultlabel(di, dksc, (struct disklabel *)data);
		break;

	case DIOCGSTRATEGY:
	    {
		struct disk_strategy *dks = (void *)data;
		int s;

		s = splbio();
		strlcpy(dks->dks_name, bufq_getstrategyname(dksc->sc_bufq),
		    sizeof(dks->dks_name));
		splx(s);
		dks->dks_paramlen = 0;

		return 0;
	    }
	
	case DIOCSSTRATEGY:
	    {
		struct disk_strategy *dks = (void *)data;
		struct bufq *new;
		struct bufq *old;
		int s;

		if ((flag & FWRITE) == 0) {
			return EBADF;
		}
		if (dks->dks_param != NULL) {
			return EINVAL;
		}
		dks->dks_name[sizeof(dks->dks_name) - 1] = 0; /* ensure term */
		error = bufq_alloc(&new, dks->dks_name,
		    BUFQ_EXACT|BUFQ_SORT_RAWBLOCK);
		if (error) {
			return error;
		}
		s = splbio();
		old = dksc->sc_bufq;
		bufq_move(new, old);
		dksc->sc_bufq = new;
		splx(s);
		bufq_free(old);

		return 0;
	    }
#endif

	default:
		printf("dk_ioctl: ioctl cmd 0x%X not implemented\n", cmd);
		error = ENOTTY;
	}

	return error;
}

/*
 * dk_dump dumps all of physical memory into the partition specified.
 * This requires substantially more framework than {s,w}ddump, and hence
 * is probably much more fragile.
 *
 * XXX: we currently do not implement this.
 */

#define DKF_READYFORDUMP	(DKF_INITED|DKF_TAKEDUMP)
#define DKFF_READYFORDUMP(x)	(((x) & DKF_READYFORDUMP) == DKF_READYFORDUMP)
static volatile int	dk_dumping = 0;

/* ARGSUSED */
int
dk_dump(struct dk_intf *di, struct dk_softc *dksc, dev_t dev,
	   daddr_t blkno, caddr_t va, size_t size)
{

	/*
	 * ensure that we consider this device to be safe for dumping,
	 * and that the device is configured.
	 */
	if (!DKFF_READYFORDUMP(dksc->sc_flags))
		return ENXIO;

	/* ensure that we are not already dumping */
	if (dk_dumping)
		return EFAULT;
	dk_dumping = 1;

	/* XXX: unimplemented */

	dk_dumping = 0;

	/* XXX: actually for now, we are going to leave this alone */
	return ENXIO;
}

/* ARGSUSED */
void
dk_getdefaultlabel(struct dk_intf *di, struct dk_softc *dksc,
		      struct disklabel *lp)
{
	struct dk_geom *pdg = &dksc->sc_geom;

	memset(lp, 0, sizeof(*lp));

	lp->d_secperunit = dksc->sc_size;
	lp->d_secsize = pdg->pdg_secsize;
	lp->d_nsectors = pdg->pdg_nsectors;
	lp->d_ntracks = pdg->pdg_ntracks;
	lp->d_ncylinders = pdg->pdg_ncylinders;
	lp->d_secpercyl = lp->d_ntracks * lp->d_nsectors;

	strncpy(lp->d_typename, di->di_dkname, sizeof(lp->d_typename));
	lp->d_type = di->di_dtype;
	strncpy(lp->d_packname, "fictitious", sizeof(lp->d_packname));
	lp->d_rpm = 3600;
	lp->d_interleave = 1;
	lp->d_flags = 0;

	lp->d_partitions[RAW_PART].p_offset = 0;
	lp->d_partitions[RAW_PART].p_size = dksc->sc_size;
	lp->d_partitions[RAW_PART].p_fstype = FS_UNUSED;
	lp->d_npartitions = RAW_PART + 1;

	lp->d_magic = DISKMAGIC;
	lp->d_magic2 = DISKMAGIC;
	lp->d_checksum = dkcksum(dksc->sc_dkdev.dk_label);
}

/* ARGSUSED */
void
dk_getdisklabel(struct dk_intf *di, struct dk_softc *dksc, dev_t dev, int spoofonly)
{
	struct	 disklabel *lp = dksc->sc_dkdev.dk_label;
	struct	 cpu_disklabel *clp = dksc->sc_dkdev.dk_cpulabel;
	struct	 partition *pp;
	int	 i;
	const char	*errstring;

	memset(clp, 0x0, sizeof(*clp));
	dk_getdefaultlabel(di, dksc, lp);
	errstring = readdisklabel(DKLABELDEV(dev), di->di_strategy,
	    dksc->sc_dkdev.dk_label, dksc->sc_dkdev.dk_cpulabel, spoofonly);
	if (errstring) {
		dk_makedisklabel(di, dksc);
		if (dksc->sc_flags & DKF_WARNLABEL)
			printf("%s: %s\n", dksc->sc_xname, errstring);
		return;
	}

	if ((dksc->sc_flags & DKF_LABELSANITY) == 0)
		return;

	/* Sanity check */
	if (lp->d_secperunit != dksc->sc_size)
		printf("WARNING: %s: total sector size in disklabel (%d) "
		    "!= the size of %s (%lu)\n", dksc->sc_xname,
		    lp->d_secperunit, di->di_dkname, (u_long)dksc->sc_size);

	for (i=0; i < lp->d_npartitions; i++) {
		pp = &lp->d_partitions[i];
		if (pp->p_offset + pp->p_size > dksc->sc_size)
			printf("WARNING: %s: end of partition `%c' exceeds "
			    "the size of %s (%lu)\n", dksc->sc_xname,
			    'a' + i, di->di_dkname, (u_long)dksc->sc_size);
	}
}

/* ARGSUSED */
static void
dk_makedisklabel(struct dk_intf *di, struct dk_softc *dksc)
{
	struct	disklabel *lp = dksc->sc_dkdev.dk_label;

	lp->d_partitions[RAW_PART].p_fstype = FS_BSDFFS;
	strncpy(lp->d_packname, "default label", sizeof(lp->d_packname));
	lp->d_checksum = dkcksum(lp);
}

/* This function is taken from ccd.c:1.76  --rcd */

/*
 * XXX this function looks too generic for dksubr.c, shouldn't we
 *     put it somewhere better?
 */

/*
 * Lookup the provided name in the filesystem.  If the file exists,
 * is a valid block device, and isn't being used by anyone else,
 * set *vpp to the file's vnode.
 */
int
dk_lookup(const char *path, struct proc *p, struct vnode **vpp)
{
	struct nameidata nd;
	struct vnode *vp;
	struct vattr va;
	int error;

	NDINIT(&nd, LOOKUP, FOLLOW, UIO_USERSPACE, path, p);
	if ((error = vn_open(&nd, FREAD|FWRITE, 0)) != 0) {
		DPRINTF((DKDB_FOLLOW|DKDB_INIT),
		    ("dk_lookup: vn_open error = %d\n", error));
		return (error);
	}
	vp = nd.ni_vp;

	if (vp->v_usecount > 1) {
		VOP_UNLOCK(vp, 0, p);
		(void)vn_close(vp, FREAD|FWRITE, p->p_ucred, p);
		return (EBUSY);
	}

	if ((error = VOP_GETATTR(vp, &va, p->p_ucred, p)) != 0) {
		DPRINTF((DKDB_FOLLOW|DKDB_INIT),
		    ("dk_lookup: getattr error = %d\n", error));
		VOP_UNLOCK(vp, 0, p);
		(void)vn_close(vp, FREAD|FWRITE, p->p_ucred, p);
		return (error);
	}

	/* XXX: eventually we should handle VREG, too. */
	if (va.va_type != VBLK) {
		VOP_UNLOCK(vp, 0, p);
		(void)vn_close(vp, FREAD|FWRITE, p->p_ucred, p);
		return (ENOTBLK);
	}

	IFDEBUG(DKDB_VNODE, vprint("dk_lookup: vnode info", vp));

	VOP_UNLOCK(vp, 0, p);
	*vpp = vp;
	return (0);
}



/*
 * snprintf() `bytes' into `buf', reformatting it so that the number,
 * plus a possible `x' + suffix extension) fits into len bytes (including
 * the terminating NUL).
 * Returns the number of bytes stored in buf, or -1 if there was a problem.
 * E.g, given a len of 9 and a suffix of `B':
 *      bytes           result
 *      -----           ------
 *      99999           `99999 B'
 *      100000          `97 kB'
 *      66715648        `65152 kB'
 *      252215296       `240 MB'
 */
int
humanize_number(char *buf, size_t len, uint64_t bytes, const char *suffix,
    int divisor)
{
	/* prefixes are: (none), kilo, Mega, Giga, Tera, Peta, Exa */
	const char *prefixes;
	int             r;
	uint64_t        umax;
	size_t          i, suffixlen;

	if (buf == NULL || suffix == NULL)
		return (-1);
	if (len > 0)
		buf[0] = '\0';
	suffixlen = strlen(suffix);
	/* check if enough room for `x y' + suffix + `\0' */
	if (len < 4 + suffixlen)
		return (-1);

	if (divisor == 1024) {
		/*
		 * binary multiplies
		 * XXX IEC 60027-2 recommends Ki, Mi, Gi...
		 */
		prefixes = " KMGTPE";
	} else
		prefixes = " kMGTPE"; /* SI for decimal multiplies */

	umax = 1;
	for (i = 0; i < len - suffixlen - 3; i++)
		umax *= 10;
	for (i = 0; bytes >= umax && prefixes[i + 1]; i++)
		bytes /= divisor;

	r = snprintf(buf, len, "%qu%s%c%s", (unsigned long long)bytes,
	    i == 0 ? "" : " ", prefixes[i], suffix);

	return (r);
}


int
format_bytes(char *buf, size_t len, uint64_t bytes)
{
	int     rv;
	size_t  nlen;

	rv = humanize_number(buf, len, bytes, "B", 1024);
	if (rv != -1) {
			/* nuke the trailing ` B' if it exists */
		nlen = strlen(buf) - 2;
		if (strcmp(&buf[nlen], " B") == 0)
			buf[nlen] = '\0';
	}
	return (rv);
}


int
bdevsw_lookup_major(const struct bdevsw *bdev)
{
	int bmajor;

	for (bmajor = 0 ; bmajor < nblkdev; bmajor++) {
		if (&bdevsw[bmajor] == bdev)
			return (bmajor);
	}

	return (-1);
}

int
cdevsw_lookup_major(const struct cdevsw *cdev)
{
	int cmajor;

	for (cmajor = 0 ; cmajor < nchrdev; cmajor++) {
		if (&cdevsw[cmajor] == cdev)
			return (cmajor);
	}

	return (-1);
}

