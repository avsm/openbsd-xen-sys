/*	$OpenBSD: systrace.h,v 1.19 2006/05/23 22:28:22 alek Exp $	*/
/*
 * Copyright 2002 Niels Provos <provos@citi.umich.edu>
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
 *      This product includes software developed by Niels Provos.
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

#ifndef _SYSTRACE_H_
#define _SYSTRACE_H_

#include <sys/ioccom.h>
#include <sys/rwlock.h>

#define SYSTR_EMULEN	8	/* sync with sys proc */

struct str_msg_emul {
	char emul[SYSTR_EMULEN];
};

struct str_msg_ugid {
	uid_t uid;
	gid_t gid;
};

struct str_msg_execve {
	char path[MAXPATHLEN];
};

#define SYSTR_MAX_POLICIES	64
#define SYSTR_MAXARGS		64
#define SYSTR_MAXFNAME		8
#define SYSTR_MAXINJECTS        8
#define SYSTR_MAXREPLEN		2048

struct str_msg_ask {
	int code;
	int argsize;
	register_t args[SYSTR_MAXARGS];
	register_t rval[2];
	int result;
};

/* Queued on fork or exit of a process */

struct str_msg_child {
	pid_t new_pid;
};

#define SYSTR_MSG_ASK		1
#define SYSTR_MSG_RES		2
#define SYSTR_MSG_EMUL		3
#define SYSTR_MSG_CHILD		4
#define SYSTR_MSG_UGID		5
#define SYSTR_MSG_POLICYFREE	6
#define SYSTR_MSG_EXECVE	7

#define SYSTR_MSG_NOPROCESS(x) \
	((x)->msg.msg_type == SYSTR_MSG_CHILD || \
	 (x)->msg.msg_type == SYSTR_MSG_POLICYFREE)

struct str_message {
	int msg_type;
	pid_t msg_pid;
	u_int16_t msg_seqnr;	/* answer has to match seqnr */
	short msg_policy;
	union {
		struct str_msg_emul msg_emul;
		struct str_msg_ugid msg_ugid;
		struct str_msg_ask msg_ask;
		struct str_msg_child msg_child;
		struct str_msg_execve msg_execve;
	} msg_data;
};

struct systrace_answer {
	pid_t stra_pid;
	u_int16_t stra_seqnr;
	short reserved;
	uid_t stra_seteuid;	/* elevated privileges for system call */
	uid_t stra_setegid;
	int stra_policy;
	int stra_error;
	int stra_flags;
};

struct systrace_scriptname {
	pid_t sn_pid;
	char  sn_scriptname[MAXPATHLEN];
};

#define SYSTR_READ		1
#define SYSTR_WRITE		2

struct systrace_io {
	pid_t strio_pid;
	int strio_op;
	void *strio_offs;
	void *strio_addr;
	size_t strio_len;
};

#define SYSTR_POLICY_NEW	1
#define SYSTR_POLICY_ASSIGN	2
#define SYSTR_POLICY_MODIFY	3

struct systrace_policy {
	int strp_op;
	int strp_num;
	union {
		struct {
			short code;
			short policy;
		} assign;
		pid_t pid;
		int maxents;
	} strp_data;
};

#define strp_pid	strp_data.pid
#define strp_maxents	strp_data.maxents
#define strp_code	strp_data.assign.code
#define strp_policy	strp_data.assign.policy

#define	SYSTR_NOLINKS	1

struct systrace_replace {
	pid_t strr_pid;
	u_int16_t strr_seqnr;
	int16_t reserved;
	int strr_nrepl;
	caddr_t	strr_base;	/* Base memory */
	size_t strr_len;	/* Length of memory */
	int strr_argind[SYSTR_MAXARGS];
	size_t strr_off[SYSTR_MAXARGS];
	size_t strr_offlen[SYSTR_MAXARGS];
	int32_t strr_flags[SYSTR_MAXARGS];
};

struct systrace_inject {
	/* On return, this contains the stackgap address. */
	caddr_t stri_addr;
	size_t  stri_len;
	pid_t   stri_pid;
};

#define STRIOCCLONE		_IOR('s', 100, int)
#define SYSTR_CLONE		STRIOCCLONE
#define STRIOCATTACH		_IOW('s', 101, pid_t)
#define STRIOCDETACH		_IOW('s', 102, pid_t)
#define STRIOCANSWER		_IOW('s', 103, struct systrace_answer)
#define STRIOCIO		_IOWR('s', 104, struct systrace_io)
#define STRIOCPOLICY		_IOWR('s', 105, struct systrace_policy)
#define STRIOCGETCWD		_IOW('s', 106, pid_t)
#define STRIOCRESCWD		_IO('s', 107)
#define STRIOCREPORT		_IOW('s', 108, pid_t)
#define STRIOCREPLACE		_IOW('s', 109, struct systrace_replace)
#define STRIOCSCRIPTNAME	_IOW('s', 110, struct systrace_scriptname)
#define STRIOCINJECT		_IOWR('s', 111, struct systrace_inject)

#define SYSTR_POLICY_ASK	0
#define SYSTR_POLICY_PERMIT	1
#define SYSTR_POLICY_NEVER	2

#define SYSTR_FLAGS_RESULT	0x001
#define SYSTR_FLAGS_SETEUID	0x002
#define SYSTR_FLAGS_SETEGID	0x004

#ifdef _KERNEL
#include <sys/namei.h>

struct str_process;
struct fsystrace {
	struct rwlock lock;
	struct selinfo si;

	TAILQ_HEAD(strprocessq, str_process) processes;
	int nprocesses;

	TAILQ_HEAD(strpolicyq, str_policy) policies;

	struct strprocessq messages;

	int npolicynr;
	int npolicies;

	int issuser;
	uid_t p_ruid;
	gid_t p_rgid;

	/* cwd magic */
	pid_t fd_pid;
	struct vnode *fd_cdir;
	struct vnode *fd_rdir;
};

/* Internal prototypes */

void systrace_namei(struct nameidata *);
int  systrace_redirect(int, struct proc *, void *, register_t *);
void systrace_exit(struct proc *);
void systrace_fork(struct proc *, struct proc *);
void systrace_execve0(struct proc *);
void systrace_execve1(char *, struct proc *);
int systrace_scriptname(struct proc *, char *);

#endif /* _KERNEL */
#endif /* _SYSTRACE_H_ */
