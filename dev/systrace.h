#ifndef _SYSTRACE_H_
#define _SYSTRACE_H_

#include <sys/ioccom.h>

#define	SYSTR_CLONE	_IOR('s', 1, int)

#define SYSTR_EMULEN	8	/* sync with sys proc */

struct str_msg_emul {
	char emul[SYSTR_EMULEN];
};

#define SYSTR_MAX_POLICIES	64
#define SYSTR_MAXARGS		64

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

#define SYSTR_MSG_ASK	1
#define SYSTR_MSG_RES	2
#define SYSTR_MSG_EMUL	3
#define SYSTR_MSG_CHILD	4

#define SYSTR_MSG_NOPROCESS(x) \
	((x)->msg.msg_type == SYSTR_MSG_CHILD)

struct str_message {
	int msg_type;
	pid_t msg_pid;
	short msg_policy;
	union {
		struct str_msg_emul msg_emul;
		struct str_msg_ask msg_ask;
		struct str_msg_child msg_child;
	} msg_data;
};

struct systrace_answer {
	pid_t stra_pid;
	int stra_policy;
	int stra_error;
	int stra_flags;
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

#define STRIOCATTACH	_IOW('s', 101, pid_t)
#define STRIOCDETACH	_IOW('s', 102, pid_t)
#define STRIOCANSWER	_IOW('s', 103, struct systrace_answer)
#define STRIOCIO	_IOWR('s', 104, struct systrace_io)
#define STRIOCPOLICY	_IOWR('s', 105, struct systrace_policy)
#define STRIOCGETCWD	_IOW('s', 106, pid_t)
#define STRIOCRESCWD	_IO('s', 107)

#define SYSTR_POLICY_ASK	0
#define SYSTR_POLICY_PERMIT	1
#define SYSTR_POLICY_NEVER	2

#define SYSTR_FLAGS_RESULT	0x001

#ifdef _KERNEL
/* Internal prototypes */

int systrace_redirect(int, struct proc *, void *, register_t *);
void systrace_exit(struct proc *);
void systrace_fork(struct proc *, struct proc *);

#endif /* _KERNEL */
#endif /* _SYSTRACE_H_ */
