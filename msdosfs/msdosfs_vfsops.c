/*	$OpenBSD: msdosfs_vfsops.c,v 1.24 2001/11/21 21:16:18 csapuntz Exp $	*/
/*	$NetBSD: msdosfs_vfsops.c,v 1.48 1997/10/18 02:54:57 briggs Exp $	*/

/*-
 * Copyright (C) 1994, 1995, 1997 Wolfgang Solfrank.
 * Copyright (C) 1994, 1995, 1997 TooLs GmbH.
 * All rights reserved.
 * Original code by Paul Popelka (paulp@uts.amdahl.com) (see below).
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
 *	This product includes software developed by TooLs GmbH.
 * 4. The name of TooLs GmbH may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY TOOLS GMBH ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL TOOLS GMBH BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/*
 * Written by Paul Popelka (paulp@uts.amdahl.com)
 * 
 * You can do anything you want with this software, just don't say you wrote
 * it, and don't remove this notice.
 * 
 * This software is provided "as is".
 * 
 * The author supplies this software to be publicly redistributed on the
 * understanding that the author is not responsible for the correct
 * functioning of this software in any circumstances and is not liable for
 * any damages caused by this software.
 * 
 * October 1992
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/namei.h>
#include <sys/proc.h>
#include <sys/kernel.h>
#include <sys/vnode.h>
#include <miscfs/specfs/specdev.h> /* XXX */	/* defines v_rdev */
#include <sys/mount.h>
#include <sys/buf.h>
#include <sys/file.h>
#include <sys/disklabel.h>
#include <sys/ioctl.h>
#include <sys/malloc.h>
#include <sys/dirent.h>

#include <msdosfs/bpb.h>
#include <msdosfs/bootsect.h>
#include <msdosfs/direntry.h>
#include <msdosfs/denode.h>
#include <msdosfs/msdosfsmount.h>
#include <msdosfs/fat.h>

int msdosfs_mount __P((struct mount *, const char *, void *, struct nameidata *,
		       struct proc *));
int msdosfs_start __P((struct mount *, int, struct proc *));
int msdosfs_unmount __P((struct mount *, int, struct proc *));
int msdosfs_root __P((struct mount *, struct vnode **));
int msdosfs_statfs __P((struct mount *, struct statfs *, struct proc *));
int msdosfs_sync __P((struct mount *, int, struct ucred *, struct proc *));
int msdosfs_fhtovp __P((struct mount *, struct fid *, struct vnode **));
int msdosfs_vptofh __P((struct vnode *, struct fid *));
int msdosfs_check_export __P((struct mount *mp, struct mbuf *nam,
			      int *extflagsp, struct ucred **credanonp));

int msdosfs_mountfs __P((struct vnode *, struct mount *, struct proc *,
			 struct msdosfs_args *));

int msdosfs_sync_vnode(struct vnode *, void *);

/*
 * mp - path - addr in user space of mount point (ie /usr or whatever) 
 * data - addr in user space of mount params including the name of the block
 * special file to treat as a filesystem. 
 */
int
msdosfs_mount(mp, path, data, ndp, p)
	struct mount *mp;
	const char *path;
	void *data;
	struct nameidata *ndp;
	struct proc *p;
{
	struct vnode *devvp;	  /* vnode for blk device to mount */
	struct msdosfs_args args; /* will hold data from mount request */
	/* msdosfs specific mount control block */
	struct msdosfsmount *pmp = NULL;
	size_t size;
	int error, flags;
	mode_t accessmode;

	error = copyin(data, &args, sizeof(struct msdosfs_args));
	if (error)
		return (error);
	/*
	 * If updating, check whether changing from read-only to
	 * read/write; if there is no device name, that's all we do.
	 */
	if (mp->mnt_flag & MNT_UPDATE) {
		pmp = VFSTOMSDOSFS(mp);
		error = 0;
		if (!(pmp->pm_flags & MSDOSFSMNT_RONLY) && (mp->mnt_flag & MNT_RDONLY)) {
			flags = WRITECLOSE;
			if (mp->mnt_flag & MNT_FORCE)
				flags |= FORCECLOSE;
			error = vflush(mp, NULLVP, flags);
		}
		if (!error && (mp->mnt_flag & MNT_RELOAD))
			/* not yet implemented */
			error = EOPNOTSUPP;
		if (error)
			return (error);
		if ((pmp->pm_flags & MSDOSFSMNT_RONLY) && (mp->mnt_flag & MNT_WANTRDWR)) {
			/*
			 * If upgrade to read-write by non-root, then verify
			 * that user has necessary permissions on the device.
			 */
			if (p->p_ucred->cr_uid != 0) {
				devvp = pmp->pm_devvp;
				vn_lock(devvp, LK_EXCLUSIVE | LK_RETRY, p);
				error = VOP_ACCESS(devvp, VREAD | VWRITE,
						   p->p_ucred, p);
				if (error) {
					VOP_UNLOCK(devvp, 0, p);
					return (error);
				}
				VOP_UNLOCK(devvp, 0, p);
			}
			pmp->pm_flags &= ~MSDOSFSMNT_RONLY;
		}
		if (args.fspec == 0) {
#ifdef	__notyet__		/* doesn't work correctly with current mountd	XXX */
			if (args.flags & MSDOSFSMNT_MNTOPT) {
				pmp->pm_flags &= ~MSDOSFSMNT_MNTOPT;
				pmp->pm_flags |= args.flags & MSDOSFSMNT_MNTOPT;
				if (pmp->pm_flags & MSDOSFSMNT_NOWIN95)
					pmp->pm_flags |= MSDOSFSMNT_SHORTNAME;
			}
#endif
			/*
			 * Process export requests.
			 */
			return (vfs_export(mp, &pmp->pm_export, &args.export));
		}
	}
	/*
	 * Not an update, or updating the name: look up the name
	 * and verify that it refers to a sensible block device.
	 */
	NDINIT(ndp, LOOKUP, FOLLOW, UIO_USERSPACE, args.fspec, p);
	if ((error = namei(ndp)) != 0)
		return (error);
	devvp = ndp->ni_vp;

	if (devvp->v_type != VBLK) {
		vrele(devvp);
		return (ENOTBLK);
	}
	if (major(devvp->v_rdev) >= nblkdev) {
		vrele(devvp);
		return (ENXIO);
	}
	/*
	 * If mount by non-root, then verify that user has necessary
	 * permissions on the device.
	 */
	if (p->p_ucred->cr_uid != 0) {
		accessmode = VREAD;
		if ((mp->mnt_flag & MNT_RDONLY) == 0)
			accessmode |= VWRITE;
		vn_lock(devvp, LK_EXCLUSIVE | LK_RETRY, p);
		error = VOP_ACCESS(devvp, accessmode, p->p_ucred, p);
		if (error) {
			vput(devvp);
			return (error);
		}
		VOP_UNLOCK(devvp, 0, p);
	}
	if ((mp->mnt_flag & MNT_UPDATE) == 0)
		error = msdosfs_mountfs(devvp, mp, p, &args);
	else {
		if (devvp != pmp->pm_devvp)
			error = EINVAL;	/* XXX needs translation */
		else
			vrele(devvp);
	}
	if (error) {
		vrele(devvp);
		return (error);
	}
	pmp = VFSTOMSDOSFS(mp);
	pmp->pm_gid = args.gid;
	pmp->pm_uid = args.uid;
	pmp->pm_mask = args.mask;
	pmp->pm_flags |= args.flags & MSDOSFSMNT_MNTOPT;

	/*
	 * GEMDOS knows nothing (yet) about win95
	 */
	if (pmp->pm_flags & MSDOSFSMNT_GEMDOSFS)
		pmp->pm_flags |= MSDOSFSMNT_NOWIN95;
	
	if (pmp->pm_flags & MSDOSFSMNT_NOWIN95)
		pmp->pm_flags |= MSDOSFSMNT_SHORTNAME;
	else if (!(pmp->pm_flags & (MSDOSFSMNT_SHORTNAME | MSDOSFSMNT_LONGNAME))) {
		struct vnode *rootvp;
		
		/*
		 * Try to divine whether to support Win'95 long filenames
		 */
		if (FAT32(pmp))
		        pmp->pm_flags |= MSDOSFSMNT_LONGNAME;
		else {
		        if ((error = msdosfs_root(mp, &rootvp)) != 0) {
			        msdosfs_unmount(mp, MNT_FORCE, p);
			        return (error);
			}
			pmp->pm_flags |= findwin95(VTODE(rootvp))
			     ? MSDOSFSMNT_LONGNAME
			     : MSDOSFSMNT_SHORTNAME;
			vput(rootvp);
		}
	}
	(void) copyinstr(path, mp->mnt_stat.f_mntonname, MNAMELEN - 1, &size);
	bzero(mp->mnt_stat.f_mntonname + size, MNAMELEN - size);
	(void) copyinstr(args.fspec, mp->mnt_stat.f_mntfromname, MNAMELEN - 1,
	    &size);
	bzero(mp->mnt_stat.f_mntfromname + size, MNAMELEN - size);
	bcopy(&args, &mp->mnt_stat.mount_info.msdosfs_args, sizeof(args));
#ifdef MSDOSFS_DEBUG
	printf("msdosfs_mount(): mp %x, pmp %x, inusemap %x\n", mp, pmp, pmp->pm_inusemap);
#endif
	return (0);
}

int
msdosfs_mountfs(devvp, mp, p, argp)
	struct vnode *devvp;
	struct mount *mp;
	struct proc *p;
	struct msdosfs_args *argp;
{
	struct msdosfsmount *pmp;
	struct buf *bp;
	dev_t dev = devvp->v_rdev;
	struct partinfo dpart;
	union bootsector *bsp;
	struct byte_bpb33 *b33;
	struct byte_bpb50 *b50;
	struct byte_bpb710 *b710;
	extern struct vnode *rootvp;
	u_int8_t SecPerClust;
	int	ronly, error;
	int	bsize = 0, dtype = 0, tmp;
	u_long dirsperblk;

	/*
	 * Disallow multiple mounts of the same device.
	 * Disallow mounting of a device that is currently in use
	 * (except for root, which might share swap device for miniroot).
	 * Flush out any old buffers remaining from a previous use.
	 */
	if ((error = vfs_mountedon(devvp)) != 0)
		return (error);
	if (vcount(devvp) > 1 && devvp != rootvp)
		return (EBUSY);
	vn_lock(devvp, LK_EXCLUSIVE | LK_RETRY, p);
	error = vinvalbuf(devvp, V_SAVE, p->p_ucred, p, 0, 0);
	VOP_UNLOCK(devvp, 0, p);
	if (error)
		return (error);

	ronly = (mp->mnt_flag & MNT_RDONLY) != 0;
	error = VOP_OPEN(devvp, ronly ? FREAD : FREAD|FWRITE, FSCRED, p);
	if (error)
		return (error);

	bp  = NULL; /* both used in error_exit */
	pmp = NULL;

	if (argp->flags & MSDOSFSMNT_GEMDOSFS) {
		/*
	 	 * We need the disklabel to calculate the size of a FAT entry
		 * later on. Also make sure the partition contains a filesystem
		 * of type FS_MSDOS. This doesn't work for floppies, so we have
		 * to check for them too.
	 	 *
	 	 * At least some parts of the msdos fs driver seem to assume
		 * that the size of a disk block will always be 512 bytes.
		 * Let's check it...
		 */
		error = VOP_IOCTL(devvp, DIOCGPART, (caddr_t)&dpart,
				  FREAD, NOCRED, p);
		if (error)
			goto error_exit;
		tmp   = dpart.part->p_fstype;
		dtype = dpart.disklab->d_type;
		bsize = dpart.disklab->d_secsize;
		if (bsize != 512 || (dtype!=DTYPE_FLOPPY && tmp!=FS_MSDOS)) {
			error = EFTYPE;
			goto error_exit;
		}
	}

	/*
	 * Read the boot sector of the filesystem, and then check the
	 * boot signature.  If not a dos boot sector then error out.
	 */
	if ((error = bread(devvp, 0, 512, NOCRED, &bp)) != 0)
		goto error_exit;
	bp->b_flags |= B_AGE;
	bsp = (union bootsector *)bp->b_data;
	b33 = (struct byte_bpb33 *)bsp->bs33.bsBPB;
	b50 = (struct byte_bpb50 *)bsp->bs50.bsBPB;
	b710 = (struct byte_bpb710 *)bsp->bs710.bsPBP;

	pmp = malloc(sizeof *pmp, M_MSDOSFSMNT, M_WAITOK);
	bzero((caddr_t)pmp, sizeof *pmp);
	pmp->pm_mountp = mp;

	/*
	 * Compute several useful quantities from the bpb in the
	 * bootsector.  Copy in the dos 5 variant of the bpb then fix up
	 * the fields that are different between dos 5 and dos 3.3.
	 */
	SecPerClust = b50->bpbSecPerClust;
	pmp->pm_BytesPerSec = getushort(b50->bpbBytesPerSec);
	pmp->pm_ResSectors = getushort(b50->bpbResSectors);
	pmp->pm_FATs = b50->bpbFATs;
	pmp->pm_RootDirEnts = getushort(b50->bpbRootDirEnts);
	pmp->pm_Sectors = getushort(b50->bpbSectors);
	pmp->pm_FATsecs = getushort(b50->bpbFATsecs);
	pmp->pm_SecPerTrack = getushort(b50->bpbSecPerTrack);
	pmp->pm_Heads = getushort(b50->bpbHeads);
	pmp->pm_Media = b50->bpbMedia;

	if (!(argp->flags & MSDOSFSMNT_GEMDOSFS)) {
		/* XXX - We should probably check more values here */
    		if (!pmp->pm_BytesPerSec || !SecPerClust
	    		|| pmp->pm_Heads > 255 || pmp->pm_SecPerTrack > 63) {
			error = EFTYPE;
			goto error_exit;
		}
	}

	if (pmp->pm_Sectors == 0) {
		pmp->pm_HiddenSects = getulong(b50->bpbHiddenSecs);
		pmp->pm_HugeSectors = getulong(b50->bpbHugeSectors);
	} else {
		pmp->pm_HiddenSects = getushort(b33->bpbHiddenSecs);
		pmp->pm_HugeSectors = pmp->pm_Sectors;
	}

	dirsperblk = pmp->pm_BytesPerSec / sizeof(struct direntry);
	if (pmp->pm_HugeSectors > 0xffffffff / dirsperblk + 1) {
	        /*
		 * We cannot deal currently with this size of disk
		 * due to fileid limitations (see msdosfs_getattr and
		 * msdosfs_readdir)
		 */
	        error = EINVAL;
		goto error_exit;
	}

	if (pmp->pm_RootDirEnts == 0) {
		if (pmp->pm_Sectors || pmp->pm_FATsecs ||
		    getushort(b710->bpbFSVers)) {
		        error = EINVAL;
			goto error_exit;
		}
		pmp->pm_fatmask = FAT32_MASK;
		pmp->pm_fatmult = 4;
		pmp->pm_fatdiv = 1;
		pmp->pm_FATsecs = getulong(b710->bpbBigFATsecs);
		if (getushort(b710->bpbExtFlags) & FATMIRROR)
		        pmp->pm_curfat = getushort(b710->bpbExtFlags) & FATNUM;
		else
		        pmp->pm_flags |= MSDOSFS_FATMIRROR;
	} else
	        pmp->pm_flags |= MSDOSFS_FATMIRROR;

	if (argp->flags & MSDOSFSMNT_GEMDOSFS) {
	        if (FAT32(pmp)) {
		        /*
			 * GEMDOS doesn't know fat32.
			 */
		        error = EINVAL;
			goto error_exit;
		}

		/*
		 * Check a few values (could do some more):
		 * - logical sector size: power of 2, >= block size
		 * - sectors per cluster: power of 2, >= 1
		 * - number of sectors:   >= 1, <= size of partition
		 */
		if ( (SecPerClust == 0)
		  || (SecPerClust & (SecPerClust - 1))
		  || (pmp->pm_BytesPerSec < bsize)
		  || (pmp->pm_BytesPerSec & (pmp->pm_BytesPerSec - 1))
		  || (pmp->pm_HugeSectors == 0)
		  || (pmp->pm_HugeSectors * (pmp->pm_BytesPerSec / bsize)
							> dpart.part->p_size)
		   ) {
			error = EFTYPE;
			goto error_exit;
		}
		/*
		 * XXX - Many parts of the msdos fs driver seem to assume that
		 * the number of bytes per logical sector (BytesPerSec) will
		 * always be the same as the number of bytes per disk block
		 * Let's pretend it is.
		 */
		tmp = pmp->pm_BytesPerSec / bsize;
		pmp->pm_BytesPerSec  = bsize;
		pmp->pm_HugeSectors *= tmp;
		pmp->pm_HiddenSects *= tmp;
		pmp->pm_ResSectors  *= tmp;
		pmp->pm_Sectors     *= tmp;
		pmp->pm_FATsecs     *= tmp;
		SecPerClust         *= tmp;
	}
	pmp->pm_fatblk = pmp->pm_ResSectors;
	if (FAT32(pmp)) {
	        pmp->pm_rootdirblk = getulong(b710->bpbRootClust);
		pmp->pm_firstcluster = pmp->pm_fatblk
		        + (pmp->pm_FATs * pmp->pm_FATsecs);
		pmp->pm_fsinfo = getushort(b710->bpbFSInfo);
	} else {
	        pmp->pm_rootdirblk = pmp->pm_fatblk +
		        (pmp->pm_FATs * pmp->pm_FATsecs);
		pmp->pm_rootdirsize = (pmp->pm_RootDirEnts * sizeof(struct direntry)
				       + pmp->pm_BytesPerSec - 1)
		        / pmp->pm_BytesPerSec;/* in sectors */
		pmp->pm_firstcluster = pmp->pm_rootdirblk + pmp->pm_rootdirsize;
	}

	pmp->pm_nmbrofclusters = (pmp->pm_HugeSectors - pmp->pm_firstcluster) /
	    SecPerClust;
	pmp->pm_maxcluster = pmp->pm_nmbrofclusters + 1;
	pmp->pm_fatsize = pmp->pm_FATsecs * pmp->pm_BytesPerSec;

	if (argp->flags & MSDOSFSMNT_GEMDOSFS) {
		if ((pmp->pm_nmbrofclusters <= (0xff0 - 2))
		      && ((dtype == DTYPE_FLOPPY) || ((dtype == DTYPE_VNODE)
		      && ((pmp->pm_Heads == 1) || (pmp->pm_Heads == 2))))
		     ) {
		        pmp->pm_fatmask = FAT12_MASK;
			pmp->pm_fatmult = 3;
			pmp->pm_fatdiv = 2;
		} else {
		        pmp->pm_fatmask = FAT16_MASK;
			pmp->pm_fatmult = 2;
			pmp->pm_fatdiv = 1;
		}
	} else if (pmp->pm_fatmask == 0) {
		if (pmp->pm_maxcluster
		    <= ((CLUST_RSRVD - CLUST_FIRST) & FAT12_MASK)) {
			/*
			 * This will usually be a floppy disk. This size makes
			 * sure that one fat entry will not be split across
			 * multiple blocks.
			 */
			pmp->pm_fatmask = FAT12_MASK;
			pmp->pm_fatmult = 3;
			pmp->pm_fatdiv = 2;
		} else {
			pmp->pm_fatmask = FAT16_MASK;
			pmp->pm_fatmult = 2;
			pmp->pm_fatdiv = 1;
		}
	}
	if (FAT12(pmp))
		pmp->pm_fatblocksize = 3 * pmp->pm_BytesPerSec;
	else
		pmp->pm_fatblocksize = MAXBSIZE;

	pmp->pm_fatblocksec = pmp->pm_fatblocksize / pmp->pm_BytesPerSec;
	pmp->pm_bnshift = ffs(pmp->pm_BytesPerSec) - 1;

	/*
	 * Compute mask and shift value for isolating cluster relative byte
	 * offsets and cluster numbers from a file offset.
	 */
	pmp->pm_bpcluster = SecPerClust * pmp->pm_BytesPerSec;
	pmp->pm_crbomask = pmp->pm_bpcluster - 1;
	pmp->pm_cnshift = ffs(pmp->pm_bpcluster) - 1;

	/*
	 * Check for valid cluster size
	 * must be a power of 2
	 */
	if (pmp->pm_bpcluster ^ (1 << pmp->pm_cnshift)) {
		error = EFTYPE;
		goto error_exit;
	}

	/*
	 * Release the bootsector buffer.
	 */
	brelse(bp);
	bp = NULL;

	/*
	 * Check FSInfo
	 */
	if (pmp->pm_fsinfo) {
	        struct fsinfo *fp;

		if ((error = bread(devvp, pmp->pm_fsinfo, 1024, NOCRED, &bp)) != 0)
		        goto error_exit;
		fp = (struct fsinfo *)bp->b_data;
		if (!bcmp(fp->fsisig1, "RRaA", 4)
		    && !bcmp(fp->fsisig2, "rrAa", 4)
		    && !bcmp(fp->fsisig3, "\0\0\125\252", 4)
		    && !bcmp(fp->fsisig4, "\0\0\125\252", 4))
		        pmp->pm_nxtfree = getulong(fp->fsinxtfree);
		else
		        pmp->pm_fsinfo = 0;
		brelse(bp);
		bp = NULL;
	}

	/*
	 * Check and validate (or perhaps invalidate?) the fsinfo structure? XXX
	 */

	/*
	 * Allocate memory for the bitmap of allocated clusters, and then
	 * fill it in.
	 */
	pmp->pm_inusemap = malloc(((pmp->pm_maxcluster + N_INUSEBITS - 1)
				   / N_INUSEBITS)
				  * sizeof(*pmp->pm_inusemap),
				  M_MSDOSFSFAT, M_WAITOK);

	/*
	 * fillinusemap() needs pm_devvp.
	 */
	pmp->pm_dev = dev;
	pmp->pm_devvp = devvp;

	/*
	 * Have the inuse map filled in.
	 */
	if ((error = fillinusemap(pmp)) != 0)
		goto error_exit;

	/*
	 * If they want fat updates to be synchronous then let them suffer
	 * the performance degradation in exchange for the on disk copy of
	 * the fat being correct just about all the time.  I suppose this
	 * would be a good thing to turn on if the kernel is still flakey.
	 */
	if (mp->mnt_flag & MNT_SYNCHRONOUS)
		pmp->pm_flags |= MSDOSFSMNT_WAITONFAT;

	/*
	 * Finish up.
	 */
	if (ronly)
		pmp->pm_flags |= MSDOSFSMNT_RONLY;
	else
		pmp->pm_fmod = 1;
	mp->mnt_data = (qaddr_t)pmp;
        mp->mnt_stat.f_fsid.val[0] = (long)dev;
        mp->mnt_stat.f_fsid.val[1] = mp->mnt_vfc->vfc_typenum;
#ifdef QUOTA
	/*
	 * If we ever do quotas for DOS filesystems this would be a place
	 * to fill in the info in the msdosfsmount structure. You dolt,
	 * quotas on dos filesystems make no sense because files have no
	 * owners on dos filesystems. of course there is some empty space
	 * in the directory entry where we could put uid's and gid's.
	 */
#endif
	devvp->v_specmountpoint = mp;

	return (0);

error_exit:
	devvp->v_specmountpoint = NULL;
	if (bp)
		brelse(bp);
	(void) VOP_CLOSE(devvp, ronly ? FREAD : FREAD|FWRITE, NOCRED, p);
	if (pmp) {
		if (pmp->pm_inusemap)
			free(pmp->pm_inusemap, M_MSDOSFSFAT);
		free(pmp, M_MSDOSFSMNT);
		mp->mnt_data = (qaddr_t)0;
	}
	return (error);
}

int
msdosfs_start(mp, flags, p)
	struct mount *mp;
	int flags;
	struct proc *p;
{

	return (0);
}

/*
 * Unmount the filesystem described by mp.
 */
int
msdosfs_unmount(mp, mntflags, p)
	struct mount *mp;
	int mntflags;
	struct proc *p;
{
	struct msdosfsmount *pmp;
	int error, flags;
	struct vnode *vp;

	flags = 0;
	if (mntflags & MNT_FORCE)
		flags |= FORCECLOSE;
#ifdef QUOTA
#endif
	if ((error = vflush(mp, NULLVP, flags)) != 0)
		return (error);
	pmp = VFSTOMSDOSFS(mp);
	pmp->pm_devvp->v_specmountpoint = NULL;
	vp = pmp->pm_devvp;
#ifdef MSDOSFS_DEBUG
	vprint("msdosfs_umount(): just before calling VOP_CLOSE()\n", vp);
#endif
	error = VOP_CLOSE(vp,
	   pmp->pm_flags & MSDOSFSMNT_RONLY ? FREAD : FREAD|FWRITE, NOCRED, p);
	vrele(vp);
	free(pmp->pm_inusemap, M_MSDOSFSFAT);
	free(pmp, M_MSDOSFSMNT);
	mp->mnt_data = (qaddr_t)0;
	mp->mnt_flag &= ~MNT_LOCAL;
	return (error);
}

int
msdosfs_root(mp, vpp)
	struct mount *mp;
	struct vnode **vpp;
{
	struct msdosfsmount *pmp = VFSTOMSDOSFS(mp);
	struct denode *ndep;
	int error;

	if ((error = deget(pmp, MSDOSFSROOT, MSDOSFSROOT_OFS, &ndep)) != 0)
		return (error);

#ifdef MSDOSFS_DEBUG
	printf("msdosfs_root(); mp %08x, pmp %08x, ndep %08x, vp %08x\n",
	    mp, pmp, ndep, DETOV(ndep));
#endif

	*vpp = DETOV(ndep);
	return (0);
}

int
msdosfs_statfs(mp, sbp, p)
	struct mount *mp;
	struct statfs *sbp;
	struct proc *p;
{
	struct msdosfsmount *pmp;

	pmp = VFSTOMSDOSFS(mp);
	sbp->f_bsize = pmp->pm_bpcluster;
	sbp->f_iosize = pmp->pm_bpcluster;
	sbp->f_blocks = pmp->pm_nmbrofclusters;
	sbp->f_bfree = pmp->pm_freeclustercount;
	sbp->f_bavail = pmp->pm_freeclustercount;
	sbp->f_files = pmp->pm_RootDirEnts;			/* XXX */
	sbp->f_ffree = 0;	/* what to put in here? */
	if (sbp != &mp->mnt_stat) {
		bcopy(mp->mnt_stat.f_mntonname, sbp->f_mntonname, MNAMELEN);
		bcopy(mp->mnt_stat.f_mntfromname, sbp->f_mntfromname, MNAMELEN);
		bcopy(&mp->mnt_stat.mount_info.msdosfs_args,
		    &sbp->mount_info.msdosfs_args, sizeof(struct msdosfs_args));
	}
	strncpy(sbp->f_fstypename, mp->mnt_vfc->vfc_name, MFSNAMELEN);
	return (0);
}


struct msdosfs_sync_arg {
	struct proc *p;
	struct ucred *cred;
	int allerror;
	int waitfor;
};

int
msdosfs_sync_vnode(struct vnode *vp, void *arg)
{
	struct msdosfs_sync_arg *msa = arg;
	int error;
	struct denode *dep;

	dep = VTODE(vp);
	if (vp->v_type == VNON || 
	    ((dep->de_flag & (DE_ACCESS | DE_CREATE | DE_UPDATE | DE_MODIFIED)) == 0
		&& vp->v_dirtyblkhd.lh_first == NULL) ||
	    msa->waitfor == MNT_LAZY) {
		simple_unlock(&vp->v_interlock);
		return (0);
	}

	if (vget(vp, LK_EXCLUSIVE | LK_NOWAIT | LK_INTERLOCK, msa->p))
		return (0);

	if ((error = VOP_FSYNC(vp, msa->cred, msa->waitfor, msa->p)) != 0)
		msa->allerror = error;
	VOP_UNLOCK(vp, 0, msa->p);
	vrele(vp);

	return (0);
}


int
msdosfs_sync(mp, waitfor, cred, p)
	struct mount *mp;
	int waitfor;
	struct ucred *cred;
	struct proc *p;
{
	struct msdosfsmount *pmp = VFSTOMSDOSFS(mp);
	struct msdosfs_sync_arg msa;
	int error;

	msa.allerror = 0;
	msa.p = p;
	msa.cred = cred;
	msa.waitfor = waitfor;

	/*
	 * If we ever switch to not updating all of the fats all the time,
	 * this would be the place to update them from the first one.
	 */
	if (pmp->pm_fmod != 0) {
		if (pmp->pm_flags & MSDOSFSMNT_RONLY)
			panic("msdosfs_sync: rofs mod");
		else {
			/* update fats here */
		}
	}
	/*
	 * Write back each (modified) denode.
	 */
	vfs_mount_foreach_vnode(mp, msdosfs_sync_vnode, &msa);

	/*
	 * Force stale file system control information to be flushed.
	 */
	if (waitfor != MNT_LAZY) {
		vn_lock(pmp->pm_devvp, LK_EXCLUSIVE | LK_RETRY, p);
		if ((error = VOP_FSYNC(pmp->pm_devvp, cred, waitfor, p)) != 0)
			msa.allerror = error;
		VOP_UNLOCK(pmp->pm_devvp, 0, p);
	}

	return (msa.allerror);
}

int
msdosfs_fhtovp(mp, fhp, vpp)
	struct mount *mp;
	struct fid *fhp;
	struct vnode **vpp;
{
	struct msdosfsmount *pmp = VFSTOMSDOSFS(mp);
	struct defid *defhp = (struct defid *) fhp;
	struct denode *dep;
	int error;

	error = deget(pmp, defhp->defid_dirclust, defhp->defid_dirofs, &dep);
	if (error) {
		*vpp = NULLVP;
		return (error);
	}
	*vpp = DETOV(dep);
	return (0);
}

int
msdosfs_vptofh(vp, fhp)
	struct vnode *vp;
	struct fid *fhp;
{
	struct denode *dep;
	struct defid *defhp;

	dep = VTODE(vp);
	defhp = (struct defid *)fhp;
	defhp->defid_len = sizeof(struct defid);
	defhp->defid_dirclust = dep->de_dirclust;
	defhp->defid_dirofs = dep->de_diroffset;
	/* defhp->defid_gen = dep->de_gen; */
	return (0);
}

int
msdosfs_check_export(mp, nam, exflagsp, credanonp)
	register struct mount *mp;
	struct mbuf *nam;
	int *exflagsp;
	struct ucred **credanonp;
{
	register struct netcred *np;
	register struct msdosfsmount *pmp = VFSTOMSDOSFS(mp);

	/*
	 * Get the export permission structure for this <mp, client> tuple.
	 */
	np = vfs_export_lookup(mp, &pmp->pm_export, nam);
	if (np == NULL)
		return (EACCES);

	*exflagsp = np->netc_exflags;
	*credanonp = &np->netc_anon;
	return (0);
}

#define msdosfs_vget ((int (*) __P((struct mount *, ino_t, struct vnode **))) \
		      eopnotsupp)

#define msdosfs_quotactl ((int (*) __P((struct mount *, int, uid_t, caddr_t, \
					struct proc *)))eopnotsupp)

#define msdosfs_sysctl ((int (*) __P((int *, u_int, void *, size_t *, void *, \
                                    size_t, struct proc *)))eopnotsupp)

struct vfsops msdosfs_vfsops = {
	msdosfs_mount,
	msdosfs_start,
	msdosfs_unmount,
	msdosfs_root,
	msdosfs_quotactl,
	msdosfs_statfs,
	msdosfs_sync,
	msdosfs_vget,
	msdosfs_fhtovp,
	msdosfs_vptofh,
	msdosfs_init,
	msdosfs_sysctl,
	msdosfs_check_export
};
