/*
 * Warning: This file is generated automatically.
 * (Modifications made here may easily be lost!)
 *
 * Created from the file:
 *	OpenBSD: vnode_if.src,v 1.32 2007/01/16 17:52:18 thib Exp 
 * by the script:
 *	OpenBSD: vnode_if.sh,v 1.15 2006/01/02 05:05:11 jsg Exp 
 */

/*
 * Copyright (c) 1992, 1993
 *	The Regents of the University of California.  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the University nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#include <sys/param.h>
#include <sys/mount.h>
#include <sys/vnode.h>

struct vnodeop_desc vop_default_desc = {
	0,
	"default",
	0,
	NULL,
	VDESC_NO_OFFSET,
	VDESC_NO_OFFSET,
	VDESC_NO_OFFSET,
	VDESC_NO_OFFSET,
	NULL,
};


int vop_islocked_vp_offsets[] = {
	VOPARG_OFFSETOF(struct vop_islocked_args,a_vp),
	VDESC_NO_OFFSET
};
struct vnodeop_desc vop_islocked_desc = {
	0,
	"vop_islocked",
	0,
	vop_islocked_vp_offsets,
	VDESC_NO_OFFSET,
	VDESC_NO_OFFSET,
	VDESC_NO_OFFSET,
	VDESC_NO_OFFSET,
	NULL,
};

int VOP_ISLOCKED(struct vnode *vp)
{
	struct vop_islocked_args a;
	a.a_desc = VDESC(vop_islocked);
	a.a_vp = vp;
	return (VCALL(vp, VOFFSET(vop_islocked), &a));
}

int vop_lookup_vp_offsets[] = {
	VOPARG_OFFSETOF(struct vop_lookup_args,a_dvp),
	VDESC_NO_OFFSET
};
struct vnodeop_desc vop_lookup_desc = {
	0,
	"vop_lookup",
	0,
	vop_lookup_vp_offsets,
	VOPARG_OFFSETOF(struct vop_lookup_args, a_vpp),
	VDESC_NO_OFFSET,
	VDESC_NO_OFFSET,
	VOPARG_OFFSETOF(struct vop_lookup_args, a_cnp),
	NULL,
};

int VOP_LOOKUP(struct vnode *dvp, struct vnode **vpp, 
    struct componentname *cnp)
{
	struct vop_lookup_args a;
	a.a_desc = VDESC(vop_lookup);
	a.a_dvp = dvp;
	a.a_vpp = vpp;
	a.a_cnp = cnp;
	return (VCALL(dvp, VOFFSET(vop_lookup), &a));
}

int vop_create_vp_offsets[] = {
	VOPARG_OFFSETOF(struct vop_create_args,a_dvp),
	VDESC_NO_OFFSET
};
struct vnodeop_desc vop_create_desc = {
	0,
	"vop_create",
	0 | VDESC_VP0_WILLPUT,
	vop_create_vp_offsets,
	VOPARG_OFFSETOF(struct vop_create_args, a_vpp),
	VDESC_NO_OFFSET,
	VDESC_NO_OFFSET,
	VOPARG_OFFSETOF(struct vop_create_args, a_cnp),
	NULL,
};

int VOP_CREATE(struct vnode *dvp, struct vnode **vpp, 
    struct componentname *cnp, struct vattr *vap)
{
	struct vop_create_args a;
#ifdef VFSDEBUG
	int islocked_dvp;
#endif
	a.a_desc = VDESC(vop_create);
	a.a_dvp = dvp;
#ifdef VFSDEBUG
	islocked_dvp = (dvp->v_flag & VLOCKSWORK) ? (VOP_ISLOCKED(dvp) == LK_EXCLUSIVE) : 1;
	if (islocked_dvp != 1)
		panic("vop_create: dvp: locked %d, expected %d", islocked_dvp, 1);
#endif
	a.a_vpp = vpp;
	a.a_cnp = cnp;
	a.a_vap = vap;
	return (VCALL(dvp, VOFFSET(vop_create), &a));
}

int vop_mknod_vp_offsets[] = {
	VOPARG_OFFSETOF(struct vop_mknod_args,a_dvp),
	VDESC_NO_OFFSET
};
struct vnodeop_desc vop_mknod_desc = {
	0,
	"vop_mknod",
	0 | VDESC_VP0_WILLPUT | VDESC_VPP_WILLRELE,
	vop_mknod_vp_offsets,
	VOPARG_OFFSETOF(struct vop_mknod_args, a_vpp),
	VDESC_NO_OFFSET,
	VDESC_NO_OFFSET,
	VOPARG_OFFSETOF(struct vop_mknod_args, a_cnp),
	NULL,
};

int VOP_MKNOD(struct vnode *dvp, struct vnode **vpp, 
    struct componentname *cnp, struct vattr *vap)
{
	struct vop_mknod_args a;
#ifdef VFSDEBUG
	int islocked_dvp;
#endif

	a.a_desc = VDESC(vop_mknod);
	a.a_dvp = dvp;
#ifdef VFSDEBUG
	islocked_dvp = (dvp->v_flag & VLOCKSWORK) ? (VOP_ISLOCKED(dvp) == LK_EXCLUSIVE) : 1;
	if (islocked_dvp != 1)
		panic("vop_mknod: dvp: locked %d, expected %d", islocked_dvp, 1);
#endif
	a.a_vpp = vpp;
	a.a_cnp = cnp;
	a.a_vap = vap;
	return (VCALL(dvp, VOFFSET(vop_mknod), &a));
}

int vop_open_vp_offsets[] = {
	VOPARG_OFFSETOF(struct vop_open_args,a_vp),
	VDESC_NO_OFFSET
};
struct vnodeop_desc vop_open_desc = {
	0,
	"vop_open",
	0,
	vop_open_vp_offsets,
	VDESC_NO_OFFSET,
	VOPARG_OFFSETOF(struct vop_open_args, a_cred),
	VOPARG_OFFSETOF(struct vop_open_args, a_p),
	VDESC_NO_OFFSET,
	NULL,
};

int VOP_OPEN(struct vnode *vp, int mode, struct ucred *cred, struct proc *p)
{
	struct vop_open_args a;
	a.a_desc = VDESC(vop_open);
	a.a_vp = vp;
	a.a_mode = mode;
	a.a_cred = cred;
	a.a_p = p;
	return (VCALL(vp, VOFFSET(vop_open), &a));
}

int vop_close_vp_offsets[] = {
	VOPARG_OFFSETOF(struct vop_close_args,a_vp),
	VDESC_NO_OFFSET
};
struct vnodeop_desc vop_close_desc = {
	0,
	"vop_close",
	0,
	vop_close_vp_offsets,
	VDESC_NO_OFFSET,
	VOPARG_OFFSETOF(struct vop_close_args, a_cred),
	VOPARG_OFFSETOF(struct vop_close_args, a_p),
	VDESC_NO_OFFSET,
	NULL,
};

int VOP_CLOSE(struct vnode *vp, int fflag, struct ucred *cred, struct proc *p)
{
	struct vop_close_args a;
#ifdef VFSDEBUG
	int islocked_vp;
#endif

	a.a_desc = VDESC(vop_close);
	a.a_vp = vp;
#ifdef VFSDEBUG
	islocked_vp = (vp->v_flag & VLOCKSWORK) ? (VOP_ISLOCKED(vp) == LK_EXCLUSIVE) : 1;
	if (islocked_vp != 1)
		panic("vop_close: vp: locked %d, expected %d", islocked_vp, 1);
#endif
	a.a_fflag = fflag;
	a.a_cred = cred;
	a.a_p = p;
	return (VCALL(vp, VOFFSET(vop_close), &a));
}

int vop_access_vp_offsets[] = {
	VOPARG_OFFSETOF(struct vop_access_args,a_vp),
	VDESC_NO_OFFSET
};
struct vnodeop_desc vop_access_desc = {
	0,
	"vop_access",
	0,
	vop_access_vp_offsets,
	VDESC_NO_OFFSET,
	VOPARG_OFFSETOF(struct vop_access_args, a_cred),
	VOPARG_OFFSETOF(struct vop_access_args, a_p),
	VDESC_NO_OFFSET,
	NULL,
};

int VOP_ACCESS(struct vnode *vp, int mode, struct ucred *cred, struct proc *p)
{
	struct vop_access_args a;
#ifdef VFSDEBUG
	int islocked_vp;
#endif
	a.a_desc = VDESC(vop_access);
	a.a_vp = vp;
#ifdef VFSDEBUG
	islocked_vp = (vp->v_flag & VLOCKSWORK) ? (VOP_ISLOCKED(vp) == LK_EXCLUSIVE) : 1;
	if (islocked_vp != 1)
		panic("vop_access: vp: locked %d, expected %d", islocked_vp, 1);
#endif
	a.a_mode = mode;
	a.a_cred = cred;
	a.a_p = p;
	return (VCALL(vp, VOFFSET(vop_access), &a));
}

int vop_getattr_vp_offsets[] = {
	VOPARG_OFFSETOF(struct vop_getattr_args,a_vp),
	VDESC_NO_OFFSET
};
struct vnodeop_desc vop_getattr_desc = {
	0,
	"vop_getattr",
	0,
	vop_getattr_vp_offsets,
	VDESC_NO_OFFSET,
	VOPARG_OFFSETOF(struct vop_getattr_args, a_cred),
	VOPARG_OFFSETOF(struct vop_getattr_args, a_p),
	VDESC_NO_OFFSET,
	NULL,
};

int VOP_GETATTR(struct vnode *vp, struct vattr *vap, struct ucred *cred, 
    struct proc *p)
{
	struct vop_getattr_args a;
	a.a_desc = VDESC(vop_getattr);
	a.a_vp = vp;
	a.a_vap = vap;
	a.a_cred = cred;
	a.a_p = p;
	return (VCALL(vp, VOFFSET(vop_getattr), &a));
}

int vop_setattr_vp_offsets[] = {
	VOPARG_OFFSETOF(struct vop_setattr_args,a_vp),
	VDESC_NO_OFFSET
};
struct vnodeop_desc vop_setattr_desc = {
	0,
	"vop_setattr",
	0,
	vop_setattr_vp_offsets,
	VDESC_NO_OFFSET,
	VOPARG_OFFSETOF(struct vop_setattr_args, a_cred),
	VOPARG_OFFSETOF(struct vop_setattr_args, a_p),
	VDESC_NO_OFFSET,
	NULL,
};

int VOP_SETATTR(struct vnode *vp, struct vattr *vap, struct ucred *cred, 
    struct proc *p)
{
	struct vop_setattr_args a;
#ifdef VFSDEBUG
	int islocked_vp;
#endif

	a.a_desc = VDESC(vop_setattr);
	a.a_vp = vp;
#ifdef VFSDEBUG
	islocked_vp = (vp->v_flag & VLOCKSWORK) ? (VOP_ISLOCKED(vp) == LK_EXCLUSIVE) : 1;
	if (islocked_vp != 1)
		panic("vop_setattr: vp: locked %d, expected %d", islocked_vp, 1);
#endif
	a.a_vap = vap;
	a.a_cred = cred;
	a.a_p = p;
	return (VCALL(vp, VOFFSET(vop_setattr), &a));
}

int vop_read_vp_offsets[] = {
	VOPARG_OFFSETOF(struct vop_read_args,a_vp),
	VDESC_NO_OFFSET
};
struct vnodeop_desc vop_read_desc = {
	0,
	"vop_read",
	0,
	vop_read_vp_offsets,
	VDESC_NO_OFFSET,
	VOPARG_OFFSETOF(struct vop_read_args, a_cred),
	VDESC_NO_OFFSET,
	VDESC_NO_OFFSET,
	NULL,
};

int VOP_READ(struct vnode *vp, struct uio *uio, int ioflag, struct ucred *cred)
{
	struct vop_read_args a;
#ifdef VFSDEBUG
	int islocked_vp;
#endif

	a.a_desc = VDESC(vop_read);
	a.a_vp = vp;
#ifdef VFSDEBUG
	islocked_vp = (vp->v_flag & VLOCKSWORK) ? (VOP_ISLOCKED(vp) == LK_EXCLUSIVE) : 1;
	if (islocked_vp != 1)
		panic("vop_read: vp: locked %d, expected %d", islocked_vp, 1);
#endif
	a.a_uio = uio;
	a.a_ioflag = ioflag;
	a.a_cred = cred;
	return (VCALL(vp, VOFFSET(vop_read), &a));
}

int vop_write_vp_offsets[] = {
	VOPARG_OFFSETOF(struct vop_write_args,a_vp),
	VDESC_NO_OFFSET
};
struct vnodeop_desc vop_write_desc = {
	0,
	"vop_write",
	0,
	vop_write_vp_offsets,
	VDESC_NO_OFFSET,
	VOPARG_OFFSETOF(struct vop_write_args, a_cred),
	VDESC_NO_OFFSET,
	VDESC_NO_OFFSET,
	NULL,
};

int VOP_WRITE(struct vnode *vp, struct uio *uio, int ioflag, 
    struct ucred *cred)
{
	struct vop_write_args a;
#ifdef VFSDEBUG
	int islocked_vp;
#endif

	a.a_desc = VDESC(vop_write);
	a.a_vp = vp;
#ifdef VFSDEBUG
	islocked_vp = (vp->v_flag & VLOCKSWORK) ? (VOP_ISLOCKED(vp) == LK_EXCLUSIVE) : 1;
	if (islocked_vp != 1)
		panic("vop_write: vp: locked %d, expected %d", islocked_vp, 1);
#endif
	a.a_uio = uio;
	a.a_ioflag = ioflag;
	a.a_cred = cred;
	return (VCALL(vp, VOFFSET(vop_write), &a));
}

int vop_ioctl_vp_offsets[] = {
	VOPARG_OFFSETOF(struct vop_ioctl_args,a_vp),
	VDESC_NO_OFFSET
};
struct vnodeop_desc vop_ioctl_desc = {
	0,
	"vop_ioctl",
	0,
	vop_ioctl_vp_offsets,
	VDESC_NO_OFFSET,
	VOPARG_OFFSETOF(struct vop_ioctl_args, a_cred),
	VOPARG_OFFSETOF(struct vop_ioctl_args, a_p),
	VDESC_NO_OFFSET,
	NULL,
};

int VOP_IOCTL(struct vnode *vp, u_long command, void *data, int fflag, 
    struct ucred *cred, struct proc *p)
{
	struct vop_ioctl_args a;
	a.a_desc = VDESC(vop_ioctl);
	a.a_vp = vp;
	a.a_command = command;
	a.a_data = data;
	a.a_fflag = fflag;
	a.a_cred = cred;
	a.a_p = p;
	return (VCALL(vp, VOFFSET(vop_ioctl), &a));
}

int vop_poll_vp_offsets[] = {
	VOPARG_OFFSETOF(struct vop_poll_args,a_vp),
	VDESC_NO_OFFSET
};
struct vnodeop_desc vop_poll_desc = {
	0,
	"vop_poll",
	0,
	vop_poll_vp_offsets,
	VDESC_NO_OFFSET,
	VDESC_NO_OFFSET,
	VOPARG_OFFSETOF(struct vop_poll_args, a_p),
	VDESC_NO_OFFSET,
	NULL,
};

int VOP_POLL(struct vnode *vp, int events, struct proc *p)
{
	struct vop_poll_args a;
	a.a_desc = VDESC(vop_poll);
	a.a_vp = vp;
	a.a_events = events;
	a.a_p = p;
	return (VCALL(vp, VOFFSET(vop_poll), &a));
}

int vop_kqfilter_vp_offsets[] = {
	VOPARG_OFFSETOF(struct vop_kqfilter_args,a_vp),
	VDESC_NO_OFFSET
};
struct vnodeop_desc vop_kqfilter_desc = {
	0,
	"vop_kqfilter",
	0,
	vop_kqfilter_vp_offsets,
	VDESC_NO_OFFSET,
	VDESC_NO_OFFSET,
	VDESC_NO_OFFSET,
	VDESC_NO_OFFSET,
	NULL,
};

int VOP_KQFILTER(struct vnode *vp, struct knote *kn)
{
	struct vop_kqfilter_args a;
	a.a_desc = VDESC(vop_kqfilter);
	a.a_vp = vp;
	a.a_kn = kn;
	return (VCALL(vp, VOFFSET(vop_kqfilter), &a));
}

int vop_revoke_vp_offsets[] = {
	VOPARG_OFFSETOF(struct vop_revoke_args,a_vp),
	VDESC_NO_OFFSET
};
struct vnodeop_desc vop_revoke_desc = {
	0,
	"vop_revoke",
	0,
	vop_revoke_vp_offsets,
	VDESC_NO_OFFSET,
	VDESC_NO_OFFSET,
	VDESC_NO_OFFSET,
	VDESC_NO_OFFSET,
	NULL,
};

int VOP_REVOKE(struct vnode *vp, int flags)
{
	struct vop_revoke_args a;
	a.a_desc = VDESC(vop_revoke);
	a.a_vp = vp;
	a.a_flags = flags;
	return (VCALL(vp, VOFFSET(vop_revoke), &a));
}

int vop_fsync_vp_offsets[] = {
	VOPARG_OFFSETOF(struct vop_fsync_args,a_vp),
	VDESC_NO_OFFSET
};
struct vnodeop_desc vop_fsync_desc = {
	0,
	"vop_fsync",
	0,
	vop_fsync_vp_offsets,
	VDESC_NO_OFFSET,
	VOPARG_OFFSETOF(struct vop_fsync_args, a_cred),
	VOPARG_OFFSETOF(struct vop_fsync_args, a_p),
	VDESC_NO_OFFSET,
	NULL,
};

int VOP_FSYNC(struct vnode *vp, struct ucred *cred, int waitfor, 
    struct proc *p)
{
	struct vop_fsync_args a;
#ifdef VFSDEBUG
	int islocked_vp;
#endif

	a.a_desc = VDESC(vop_fsync);
	a.a_vp = vp;
#ifdef VFSDEBUG
        islocked_vp = (vp->v_flag & VLOCKSWORK) ? (VOP_ISLOCKED(vp) == LK_EXCLUSIVE) : 1;
        if (islocked_vp != 1)
                panic("vop_fsync: vp: locked %d, expected %d", islocked_vp, 1);
#endif
	a.a_cred = cred;
	a.a_waitfor = waitfor;
	a.a_p = p;
	return (VCALL(vp, VOFFSET(vop_fsync), &a));
}

int vop_remove_vp_offsets[] = {
	VOPARG_OFFSETOF(struct vop_remove_args,a_dvp),
	VOPARG_OFFSETOF(struct vop_remove_args,a_vp),
	VDESC_NO_OFFSET
};
struct vnodeop_desc vop_remove_desc = {
	0,
	"vop_remove",
	0 | VDESC_VP0_WILLPUT | VDESC_VP1_WILLPUT,
	vop_remove_vp_offsets,
	VDESC_NO_OFFSET,
	VDESC_NO_OFFSET,
	VDESC_NO_OFFSET,
	VOPARG_OFFSETOF(struct vop_remove_args, a_cnp),
	NULL,
};

int VOP_REMOVE(struct vnode *dvp, struct vnode *vp, struct componentname *cnp)
{
	struct vop_remove_args a;
#ifdef VFSDEBUG
	int islocked_dvp;
	int islocked_vp;
#endif

	a.a_desc = VDESC(vop_remove);
	a.a_dvp = dvp;
#ifdef VFSDEBUG
	islocked_dvp = (dvp->v_flag & VLOCKSWORK) ? (VOP_ISLOCKED(dvp) == LK_EXCLUSIVE) : 1;
	if (islocked_dvp != 1)
		panic("vop_remove: dvp: locked %d, expected %d", islocked_dvp, 1);
#endif
	a.a_vp = vp;
#ifdef VFSDEBUG
	islocked_vp = (vp->v_flag & VLOCKSWORK) ? (VOP_ISLOCKED(vp) == LK_EXCLUSIVE) : 1;
	if (islocked_vp != 1)
		panic("vop_remove: vp: locked %d, expected %d", islocked_vp, 1);
#endif
	a.a_cnp = cnp;
	return (VCALL(dvp, VOFFSET(vop_remove), &a));
}

int vop_link_vp_offsets[] = {
	VOPARG_OFFSETOF(struct vop_link_args,a_dvp),
	VOPARG_OFFSETOF(struct vop_link_args,a_vp),
	VDESC_NO_OFFSET
};
struct vnodeop_desc vop_link_desc = {
	0,
	"vop_link",
	0 | VDESC_VP0_WILLPUT,
	vop_link_vp_offsets,
	VDESC_NO_OFFSET,
	VDESC_NO_OFFSET,
	VDESC_NO_OFFSET,
	VOPARG_OFFSETOF(struct vop_link_args, a_cnp),
	NULL,
};

int VOP_LINK(struct vnode *dvp, struct vnode *vp, struct componentname *cnp)
{
	struct vop_link_args a;
#ifdef VFSDEBUG
	int islocked_dvp;
	int islocked_vp;
#endif

	a.a_desc = VDESC(vop_link);
	a.a_dvp = dvp;
#ifdef VFSDEBUG
	islocked_dvp = (dvp->v_flag & VLOCKSWORK) ? (VOP_ISLOCKED(dvp) == LK_EXCLUSIVE) : 1;
	if (islocked_dvp != 1)
		panic("vop_link: dvp: locked %d, expected %d", islocked_dvp, 1);
#endif
	a.a_vp = vp;
#ifdef VFSDEBUG
	islocked_vp = (vp->v_flag & VLOCKSWORK) ? (VOP_ISLOCKED(vp) == LK_EXCLUSIVE) : 0;
	if (islocked_vp != 0)
		panic("vop_link: vp: locked %d, expected %d", islocked_vp, 0);
#endif
	a.a_cnp = cnp;
	return (VCALL(dvp, VOFFSET(vop_link), &a));
}

int vop_rename_vp_offsets[] = {
	VOPARG_OFFSETOF(struct vop_rename_args,a_fdvp),
	VOPARG_OFFSETOF(struct vop_rename_args,a_fvp),
	VOPARG_OFFSETOF(struct vop_rename_args,a_tdvp),
	VOPARG_OFFSETOF(struct vop_rename_args,a_tvp),
	VDESC_NO_OFFSET
};
struct vnodeop_desc vop_rename_desc = {
	0,
	"vop_rename",
	0 | VDESC_VP0_WILLRELE | VDESC_VP1_WILLRELE | VDESC_VP2_WILLPUT | VDESC_VP3_WILLRELE,
	vop_rename_vp_offsets,
	VDESC_NO_OFFSET,
	VDESC_NO_OFFSET,
	VDESC_NO_OFFSET,
	VOPARG_OFFSETOF(struct vop_rename_args, a_fcnp),
	NULL,
};

int VOP_RENAME(struct vnode *fdvp, struct vnode *fvp, 
    struct componentname *fcnp, struct vnode *tdvp, struct vnode *tvp, 
    struct componentname *tcnp)
{
	struct vop_rename_args a;
#ifdef VFSDEBUG
	int islocked_fdvp;
	int islocked_fvp;
	int islocked_tdvp;
#endif

	a.a_desc = VDESC(vop_rename);
	a.a_fdvp = fdvp;
#ifdef VFSDEBUG
	islocked_fdvp = (fdvp->v_flag & VLOCKSWORK) ? (VOP_ISLOCKED(fdvp) == LK_EXCLUSIVE) : 0;
	if (islocked_fdvp != 0)
		panic("vop_rename: fdvp: locked %d, expected %d", islocked_fdvp, 0);
#endif
	a.a_fvp = fvp;
#ifdef VFSDEBUG
	islocked_fvp = (fvp->v_flag & VLOCKSWORK) ? (VOP_ISLOCKED(fvp) == LK_EXCLUSIVE) : 0;
	if (islocked_fvp != 0)
		panic("vop_rename: fvp: locked %d, expected %d", islocked_fvp, 0);
#endif
	a.a_fcnp = fcnp;
	a.a_tdvp = tdvp;
#ifdef VFSDEBUG
	islocked_tdvp = (tdvp->v_flag & VLOCKSWORK) ? (VOP_ISLOCKED(tdvp) == LK_EXCLUSIVE) : 1;
	if (islocked_tdvp != 1)
		panic("vop_rename: tdvp: locked %d, expected %d", islocked_tdvp, 1);
#endif
	a.a_tvp = tvp;
	a.a_tcnp = tcnp;
	return (VCALL(fdvp, VOFFSET(vop_rename), &a));
}

int vop_mkdir_vp_offsets[] = {
	VOPARG_OFFSETOF(struct vop_mkdir_args,a_dvp),
	VDESC_NO_OFFSET
};
struct vnodeop_desc vop_mkdir_desc = {
	0,
	"vop_mkdir",
	0 | VDESC_VP0_WILLPUT,
	vop_mkdir_vp_offsets,
	VOPARG_OFFSETOF(struct vop_mkdir_args, a_vpp),
	VDESC_NO_OFFSET,
	VDESC_NO_OFFSET,
	VOPARG_OFFSETOF(struct vop_mkdir_args, a_cnp),
	NULL,
};

int VOP_MKDIR(struct vnode *dvp, struct vnode **vpp, 
    struct componentname *cnp, struct vattr *vap)
{
	struct vop_mkdir_args a;
#ifdef VFSDEBUG
	int islocked_dvp;
#endif

	a.a_desc = VDESC(vop_mkdir);
	a.a_dvp = dvp;
#ifdef VFSDEBUG
	islocked_dvp = (dvp->v_flag & VLOCKSWORK) ? (VOP_ISLOCKED(dvp) == LK_EXCLUSIVE) : 1;
	if (islocked_dvp != 1)
		panic("vop_mkdir: dvp: locked %d, expected %d", islocked_dvp, 1);
#endif
	a.a_vpp = vpp;
	a.a_cnp = cnp;
	a.a_vap = vap;
	return (VCALL(dvp, VOFFSET(vop_mkdir), &a));
}

int vop_rmdir_vp_offsets[] = {
	VOPARG_OFFSETOF(struct vop_rmdir_args,a_dvp),
	VOPARG_OFFSETOF(struct vop_rmdir_args,a_vp),
	VDESC_NO_OFFSET
};
struct vnodeop_desc vop_rmdir_desc = {
	0,
	"vop_rmdir",
	0 | VDESC_VP0_WILLPUT | VDESC_VP1_WILLPUT,
	vop_rmdir_vp_offsets,
	VDESC_NO_OFFSET,
	VDESC_NO_OFFSET,
	VDESC_NO_OFFSET,
	VOPARG_OFFSETOF(struct vop_rmdir_args, a_cnp),
	NULL,
};

int VOP_RMDIR(struct vnode *dvp, struct vnode *vp, struct componentname *cnp)
{
	struct vop_rmdir_args a;
#ifdef VFSDEBUG
	int islocked_dvp;
	int islocked_vp;
#endif

	a.a_desc = VDESC(vop_rmdir);
	a.a_dvp = dvp;
#ifdef VFSDEBUG
	islocked_dvp = (dvp->v_flag & VLOCKSWORK) ? (VOP_ISLOCKED(dvp) == LK_EXCLUSIVE) : 1;
	if (islocked_dvp != 1)
		panic("vop_rmdir: dvp: locked %d, expected %d", islocked_dvp, 1);
#endif
	a.a_vp = vp;
#ifdef VFSDEBUG
	islocked_vp = (vp->v_flag & VLOCKSWORK) ? (VOP_ISLOCKED(vp) == LK_EXCLUSIVE) : 1;
	if (islocked_vp != 1)
		panic("vop_rmdir: vp: locked %d, expected %d", islocked_vp, 1);
#endif
	a.a_cnp = cnp;
	return (VCALL(dvp, VOFFSET(vop_rmdir), &a));
}

int vop_symlink_vp_offsets[] = {
	VOPARG_OFFSETOF(struct vop_symlink_args,a_dvp),
	VDESC_NO_OFFSET
};
struct vnodeop_desc vop_symlink_desc = {
	0,
	"vop_symlink",
	0 | VDESC_VP0_WILLPUT | VDESC_VPP_WILLRELE,
	vop_symlink_vp_offsets,
	VOPARG_OFFSETOF(struct vop_symlink_args, a_vpp),
	VDESC_NO_OFFSET,
	VDESC_NO_OFFSET,
	VOPARG_OFFSETOF(struct vop_symlink_args, a_cnp),
	NULL,
};

int VOP_SYMLINK(struct vnode *dvp, struct vnode **vpp, 
    struct componentname *cnp, struct vattr *vap, char *target)
{
	struct vop_symlink_args a;
#ifdef VFSDEBUG
	int islocked_dvp;
#endif

	a.a_desc = VDESC(vop_symlink);
	a.a_dvp = dvp;
#ifdef VFSDEBUG
	islocked_dvp = (dvp->v_flag & VLOCKSWORK) ? (VOP_ISLOCKED(dvp) == LK_EXCLUSIVE) : 1;
	if (islocked_dvp != 1)
		panic("vop_symlink: dvp: locked %d, expected %d", islocked_dvp, 1);
#endif
	a.a_vpp = vpp;
	a.a_cnp = cnp;
	a.a_vap = vap;
	a.a_target = target;
	return (VCALL(dvp, VOFFSET(vop_symlink), &a));
}

int vop_readdir_vp_offsets[] = {
	VOPARG_OFFSETOF(struct vop_readdir_args,a_vp),
	VDESC_NO_OFFSET
};
struct vnodeop_desc vop_readdir_desc = {
	0,
	"vop_readdir",
	0,
	vop_readdir_vp_offsets,
	VDESC_NO_OFFSET,
	VOPARG_OFFSETOF(struct vop_readdir_args, a_cred),
	VDESC_NO_OFFSET,
	VDESC_NO_OFFSET,
	NULL,
};

int VOP_READDIR(struct vnode *vp, struct uio *uio, struct ucred *cred, 
    int *eofflag, int *ncookies, u_long **cookies)
{
	struct vop_readdir_args a;
#ifdef VFSDEBUG
	int islocked_vp;
#endif

	a.a_desc = VDESC(vop_readdir);
	a.a_vp = vp;
#ifdef VFSDEBUG
	islocked_vp = (vp->v_flag & VLOCKSWORK) ? (VOP_ISLOCKED(vp) == LK_EXCLUSIVE) : 1;
	if (islocked_vp != 1)
		panic("vop_readdir: vp: locked %d, expected %d", islocked_vp, 1);
#endif
	a.a_uio = uio;
	a.a_cred = cred;
	a.a_eofflag = eofflag;
	a.a_ncookies = ncookies;
	a.a_cookies = cookies;
	return (VCALL(vp, VOFFSET(vop_readdir), &a));
}

int vop_readlink_vp_offsets[] = {
	VOPARG_OFFSETOF(struct vop_readlink_args,a_vp),
	VDESC_NO_OFFSET
};
struct vnodeop_desc vop_readlink_desc = {
	0,
	"vop_readlink",
	0,
	vop_readlink_vp_offsets,
	VDESC_NO_OFFSET,
	VOPARG_OFFSETOF(struct vop_readlink_args, a_cred),
	VDESC_NO_OFFSET,
	VDESC_NO_OFFSET,
	NULL,
};

int VOP_READLINK(struct vnode *vp, struct uio *uio, struct ucred *cred)
{
	struct vop_readlink_args a;
#ifdef VFSDEBUG
	int islocked_vp;
#endif

	a.a_desc = VDESC(vop_readlink);
	a.a_vp = vp;
#ifdef VFSDEBUG
	islocked_vp = (vp->v_flag & VLOCKSWORK) ? (VOP_ISLOCKED(vp) == LK_EXCLUSIVE) : 1;
	if (islocked_vp != 1)
		panic("vop_readlink: vp: locked %d, expected %d", islocked_vp, 1);
#endif
	a.a_uio = uio;
	a.a_cred = cred;
	return (VCALL(vp, VOFFSET(vop_readlink), &a));
}

int vop_abortop_vp_offsets[] = {
	VOPARG_OFFSETOF(struct vop_abortop_args,a_dvp),
	VDESC_NO_OFFSET
};
struct vnodeop_desc vop_abortop_desc = {
	0,
	"vop_abortop",
	0,
	vop_abortop_vp_offsets,
	VDESC_NO_OFFSET,
	VDESC_NO_OFFSET,
	VDESC_NO_OFFSET,
	VOPARG_OFFSETOF(struct vop_abortop_args, a_cnp),
	NULL,
};

int VOP_ABORTOP(struct vnode *dvp, struct componentname *cnp)
{
	struct vop_abortop_args a;
	a.a_desc = VDESC(vop_abortop);
	a.a_dvp = dvp;
	a.a_cnp = cnp;
	return (VCALL(dvp, VOFFSET(vop_abortop), &a));
}

int vop_inactive_vp_offsets[] = {
	VOPARG_OFFSETOF(struct vop_inactive_args,a_vp),
	VDESC_NO_OFFSET
};
struct vnodeop_desc vop_inactive_desc = {
	0,
	"vop_inactive",
	0 | VDESC_VP0_WILLUNLOCK,
	vop_inactive_vp_offsets,
	VDESC_NO_OFFSET,
	VDESC_NO_OFFSET,
	VOPARG_OFFSETOF(struct vop_inactive_args, a_p),
	VDESC_NO_OFFSET,
	NULL,
};

int VOP_INACTIVE(struct vnode *vp, struct proc *p)
{
	struct vop_inactive_args a;
#ifdef VFSDEBUG
	int islocked_vp;
#endif

	a.a_desc = VDESC(vop_inactive);
	a.a_vp = vp;
#ifdef VFSDEBUG
	islocked_vp = (vp->v_flag & VLOCKSWORK) ? (VOP_ISLOCKED(vp) == LK_EXCLUSIVE) : 1;
	if (islocked_vp != 1)
		panic("vop_inactive: vp: locked %d, expected %d", islocked_vp, 1);
#endif
	a.a_p = p;
	return (VCALL(vp, VOFFSET(vop_inactive), &a));
}

int vop_reclaim_vp_offsets[] = {
	VOPARG_OFFSETOF(struct vop_reclaim_args,a_vp),
	VDESC_NO_OFFSET
};
struct vnodeop_desc vop_reclaim_desc = {
	0,
	"vop_reclaim",
	0,
	vop_reclaim_vp_offsets,
	VDESC_NO_OFFSET,
	VDESC_NO_OFFSET,
	VOPARG_OFFSETOF(struct vop_reclaim_args, a_p),
	VDESC_NO_OFFSET,
	NULL,
};

int VOP_RECLAIM(struct vnode *vp, struct proc *p)
{
	struct vop_reclaim_args a;
	a.a_desc = VDESC(vop_reclaim);
	a.a_vp = vp;
	a.a_p = p;
	return (VCALL(vp, VOFFSET(vop_reclaim), &a));
}

int vop_lock_vp_offsets[] = {
	VOPARG_OFFSETOF(struct vop_lock_args,a_vp),
	VDESC_NO_OFFSET
};
struct vnodeop_desc vop_lock_desc = {
	0,
	"vop_lock",
	0,
	vop_lock_vp_offsets,
	VDESC_NO_OFFSET,
	VDESC_NO_OFFSET,
	VOPARG_OFFSETOF(struct vop_lock_args, a_p),
	VDESC_NO_OFFSET,
	NULL,
};

int VOP_LOCK(struct vnode *vp, int flags, struct proc *p)
{
	struct vop_lock_args a;
	a.a_desc = VDESC(vop_lock);
	a.a_vp = vp;
	a.a_flags = flags;
	a.a_p = p;
	return (VCALL(vp, VOFFSET(vop_lock), &a));
}

int vop_unlock_vp_offsets[] = {
	VOPARG_OFFSETOF(struct vop_unlock_args,a_vp),
	VDESC_NO_OFFSET
};
struct vnodeop_desc vop_unlock_desc = {
	0,
	"vop_unlock",
	0,
	vop_unlock_vp_offsets,
	VDESC_NO_OFFSET,
	VDESC_NO_OFFSET,
	VOPARG_OFFSETOF(struct vop_unlock_args, a_p),
	VDESC_NO_OFFSET,
	NULL,
};

int VOP_UNLOCK(struct vnode *vp, int flags, struct proc *p)
{
	struct vop_unlock_args a;
	a.a_desc = VDESC(vop_unlock);
	a.a_vp = vp;
	a.a_flags = flags;
	a.a_p = p;
	return (VCALL(vp, VOFFSET(vop_unlock), &a));
}

int vop_bmap_vp_offsets[] = {
	VOPARG_OFFSETOF(struct vop_bmap_args,a_vp),
	VDESC_NO_OFFSET
};
struct vnodeop_desc vop_bmap_desc = {
	0,
	"vop_bmap",
	0,
	vop_bmap_vp_offsets,
	VOPARG_OFFSETOF(struct vop_bmap_args, a_vpp),
	VDESC_NO_OFFSET,
	VDESC_NO_OFFSET,
	VDESC_NO_OFFSET,
	NULL,
};

int VOP_BMAP(struct vnode *vp, daddr64_t bn, struct vnode **vpp, 
    daddr64_t *bnp, int *runp)
{
	struct vop_bmap_args a;
#ifdef VFSDEBUG
	int islocked_vp;
#endif

	a.a_desc = VDESC(vop_bmap);
	a.a_vp = vp;
#ifdef VFSDEBUG
        islocked_vp = (vp->v_flag & VLOCKSWORK) ? (VOP_ISLOCKED(vp) == LK_EXCLUSIVE) : 1;
        if (islocked_vp != 1)
                panic("vop_bmap: vp: locked %d, expected %d", islocked_vp, 1);
#endif
	a.a_bn = bn;
	a.a_vpp = vpp;
	a.a_bnp = bnp;
	a.a_runp = runp;
	return (VCALL(vp, VOFFSET(vop_bmap), &a));
}

int vop_print_vp_offsets[] = {
	VOPARG_OFFSETOF(struct vop_print_args,a_vp),
	VDESC_NO_OFFSET
};
struct vnodeop_desc vop_print_desc = {
	0,
	"vop_print",
	0,
	vop_print_vp_offsets,
	VDESC_NO_OFFSET,
	VDESC_NO_OFFSET,
	VDESC_NO_OFFSET,
	VDESC_NO_OFFSET,
	NULL,
};

int VOP_PRINT(struct vnode *vp)
{
	struct vop_print_args a;
	a.a_desc = VDESC(vop_print);
	a.a_vp = vp;
	return (VCALL(vp, VOFFSET(vop_print), &a));
}

int vop_pathconf_vp_offsets[] = {
	VOPARG_OFFSETOF(struct vop_pathconf_args,a_vp),
	VDESC_NO_OFFSET
};
struct vnodeop_desc vop_pathconf_desc = {
	0,
	"vop_pathconf",
	0,
	vop_pathconf_vp_offsets,
	VDESC_NO_OFFSET,
	VDESC_NO_OFFSET,
	VDESC_NO_OFFSET,
	VDESC_NO_OFFSET,
	NULL,
};

int VOP_PATHCONF(struct vnode *vp, int name, register_t *retval)
{
	struct vop_pathconf_args a;
#ifdef VFSDEBUG
	int islocked_vp;
#endif

	a.a_desc = VDESC(vop_pathconf);
	a.a_vp = vp;
#ifdef VFSDEBUG
        islocked_vp = (vp->v_flag & VLOCKSWORK) ? (VOP_ISLOCKED(vp) == LK_EXCLUSIVE) : 1;
        if (islocked_vp != 1)
                panic("vop_pathconf: vp: locked %d, expected %d", islocked_vp, 1);
#endif
	a.a_name = name;
	a.a_retval = retval;
	return (VCALL(vp, VOFFSET(vop_pathconf), &a));
}

int vop_advlock_vp_offsets[] = {
	VOPARG_OFFSETOF(struct vop_advlock_args,a_vp),
	VDESC_NO_OFFSET
};
struct vnodeop_desc vop_advlock_desc = {
	0,
	"vop_advlock",
	0,
	vop_advlock_vp_offsets,
	VDESC_NO_OFFSET,
	VDESC_NO_OFFSET,
	VDESC_NO_OFFSET,
	VDESC_NO_OFFSET,
	NULL,
};

int VOP_ADVLOCK(struct vnode *vp, void *id, int op, struct flock *fl, int flags)
{
	struct vop_advlock_args a;
	a.a_desc = VDESC(vop_advlock);
	a.a_vp = vp;
	a.a_id = id;
	a.a_op = op;
	a.a_fl = fl;
	a.a_flags = flags;
	return (VCALL(vp, VOFFSET(vop_advlock), &a));
}

int vop_reallocblks_vp_offsets[] = {
	VOPARG_OFFSETOF(struct vop_reallocblks_args,a_vp),
	VDESC_NO_OFFSET
};
struct vnodeop_desc vop_reallocblks_desc = {
	0,
	"vop_reallocblks",
	0,
	vop_reallocblks_vp_offsets,
	VDESC_NO_OFFSET,
	VDESC_NO_OFFSET,
	VDESC_NO_OFFSET,
	VDESC_NO_OFFSET,
	NULL,
};

int VOP_REALLOCBLKS(struct vnode *vp, struct cluster_save *buflist)
{
	struct vop_reallocblks_args a;
#ifdef VFSDEBUG
	int islocked_vp;
#endif

	a.a_desc = VDESC(vop_reallocblks);
	a.a_vp = vp;
#ifdef VFSDEBUG
	islocked_vp = (vp->v_flag & VLOCKSWORK) ? (VOP_ISLOCKED(vp) == LK_EXCLUSIVE) : 1;
	if (islocked_vp != 1)
		panic("vop_reallocblks: vp: locked %d, expected %d", islocked_vp, 1);
#endif
	a.a_buflist = buflist;
	return (VCALL(vp, VOFFSET(vop_reallocblks), &a));
}

/* Special cases: */

int vop_strategy_vp_offsets[] = {
	VDESC_NO_OFFSET
};
struct vnodeop_desc vop_strategy_desc = {
	0,
	"vop_strategy",
	0,
	vop_strategy_vp_offsets,
	VDESC_NO_OFFSET,
	VDESC_NO_OFFSET,
	VDESC_NO_OFFSET,
	VDESC_NO_OFFSET,
	NULL,
};

int VOP_STRATEGY(struct buf *bp)
{
	struct vop_strategy_args a;
	a.a_desc = VDESC(vop_strategy);
	a.a_bp = bp;
	return (VCALL(bp->b_vp, VOFFSET(vop_strategy), &a));
}

int vop_bwrite_vp_offsets[] = {
	VDESC_NO_OFFSET
};
struct vnodeop_desc vop_bwrite_desc = {
	0,
	"vop_bwrite",
	0,
	vop_bwrite_vp_offsets,
	VDESC_NO_OFFSET,
	VDESC_NO_OFFSET,
	VDESC_NO_OFFSET,
	VDESC_NO_OFFSET,
	NULL,
};

int VOP_BWRITE(struct buf *bp)
{
	struct vop_bwrite_args a;
	a.a_desc = VDESC(vop_bwrite);
	a.a_bp = bp;
	return (VCALL(bp->b_vp, VOFFSET(vop_bwrite), &a));
}

/* End of special cases. */

struct vnodeop_desc *vfs_op_descs[] = {
	&vop_default_desc,	/* MUST BE FIRST */
	&vop_strategy_desc,	/* XXX: SPECIAL CASE */
	&vop_bwrite_desc,	/* XXX: SPECIAL CASE */

	&vop_islocked_desc,
	&vop_lookup_desc,
	&vop_create_desc,
	&vop_mknod_desc,
	&vop_open_desc,
	&vop_close_desc,
	&vop_access_desc,
	&vop_getattr_desc,
	&vop_setattr_desc,
	&vop_read_desc,
	&vop_write_desc,
	&vop_ioctl_desc,
	&vop_poll_desc,
	&vop_kqfilter_desc,
	&vop_revoke_desc,
	&vop_fsync_desc,
	&vop_remove_desc,
	&vop_link_desc,
	&vop_rename_desc,
	&vop_mkdir_desc,
	&vop_rmdir_desc,
	&vop_symlink_desc,
	&vop_readdir_desc,
	&vop_readlink_desc,
	&vop_abortop_desc,
	&vop_inactive_desc,
	&vop_reclaim_desc,
	&vop_lock_desc,
	&vop_unlock_desc,
	&vop_bmap_desc,
	&vop_print_desc,
	&vop_pathconf_desc,
	&vop_advlock_desc,
	&vop_reallocblks_desc,
	NULL
};

