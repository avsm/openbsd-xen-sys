/*	$OpenBSD$	*/

/*
 * System call switch table.
 *
 * DO NOT EDIT-- this file is automatically generated.
 * created from	OpenBSD: syscalls.master,v 1.7 2004/07/15 20:07:41 mickey Exp 
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/signal.h>
#include <sys/mount.h>
#include <sys/exec.h>
#include <sys/syscallargs.h>
#include <compat/hpux/hpux.h>
#include <compat/hpux/hppa/hpux_syscallargs.h>
#include <machine/hpux_machdep.h>

#define	s(type)	sizeof(type)

struct sysent hpux_sysent[] = {
	{ 0, 0,
	    sys_nosys },			/* 0 = syscall */
	{ 1, s(struct sys_exit_args),
	    sys_exit },				/* 1 = exit */
	{ 0, 0,
	    hpux_sys_fork },			/* 2 = fork */
	{ 3, s(struct hpux_sys_read_args),
	    hpux_sys_read },			/* 3 = read */
	{ 3, s(struct hpux_sys_write_args),
	    hpux_sys_write },			/* 4 = write */
	{ 3, s(struct hpux_sys_open_args),
	    hpux_sys_open },			/* 5 = open */
	{ 1, s(struct sys_close_args),
	    sys_close },			/* 6 = close */
	{ 1, s(struct hpux_sys_wait_args),
	    hpux_sys_wait },			/* 7 = wait */
	{ 2, s(struct hpux_sys_creat_args),
	    hpux_sys_creat },			/* 8 = creat */
	{ 2, s(struct sys_link_args),
	    sys_link },				/* 9 = link */
	{ 1, s(struct hpux_sys_unlink_args),
	    hpux_sys_unlink },			/* 10 = unlink */
	{ 2, s(struct hpux_sys_execv_args),
	    hpux_sys_execv },			/* 11 = execv */
	{ 1, s(struct hpux_sys_chdir_args),
	    hpux_sys_chdir },			/* 12 = chdir */
	{ 1, s(struct hpux_sys_time_6x_args),
	    hpux_sys_time_6x },			/* 13 = time_6x */
	{ 3, s(struct hpux_sys_mknod_args),
	    hpux_sys_mknod },			/* 14 = mknod */
	{ 2, s(struct hpux_sys_chmod_args),
	    hpux_sys_chmod },			/* 15 = chmod */
	{ 3, s(struct hpux_sys_chown_args),
	    hpux_sys_chown },			/* 16 = chown */
	{ 1, s(struct sys_obreak_args),
	    sys_obreak },			/* 17 = obreak */
	{ 0, 0,
	    sys_nosys },			/* 18 = unimplemented lchmod */
	{ 3, s(struct compat_43_sys_lseek_args),
	    compat_43_sys_lseek },		/* 19 = lseek */
	{ 0, 0,
	    sys_getpid },			/* 20 = getpid */
	{ 0, 0,
	    sys_nosys },			/* 21 = unimplemented mount */
	{ 0, 0,
	    sys_nosys },			/* 22 = unimplemented umount */
	{ 1, s(struct sys_setuid_args),
	    sys_setuid },			/* 23 = setuid */
	{ 0, 0,
	    sys_getuid },			/* 24 = getuid */
	{ 1, s(struct hpux_sys_stime_6x_args),
	    hpux_sys_stime_6x },		/* 25 = stime_6x */
#ifdef PTRACE
	{ 4, s(struct hpux_sys_ptrace_args),
	    hpux_sys_ptrace },			/* 26 = ptrace */
#else
	{ 0, 0,
	    sys_nosys },			/* 26 = unimplemented ptrace */
#endif
	{ 1, s(struct hpux_sys_alarm_6x_args),
	    hpux_sys_alarm_6x },		/* 27 = alarm_6x */
	{ 0, 0,
	    sys_nosys },			/* 28 = unimplemented cnx_lw_pmon_read */
	{ 0, 0,
	    hpux_sys_pause_6x },		/* 29 = pause_6x */
	{ 2, s(struct hpux_sys_utime_6x_args),
	    hpux_sys_utime_6x },		/* 30 = utime_6x */
	{ 2, s(struct hpux_sys_stty_6x_args),
	    hpux_sys_stty_6x },			/* 31 = stty_6x */
	{ 2, s(struct hpux_sys_gtty_6x_args),
	    hpux_sys_gtty_6x },			/* 32 = gtty_6x */
	{ 2, s(struct hpux_sys_access_args),
	    hpux_sys_access },			/* 33 = access */
	{ 1, s(struct hpux_sys_nice_6x_args),
	    hpux_sys_nice_6x },			/* 34 = nice_6x */
	{ 1, s(struct hpux_sys_ftime_6x_args),
	    hpux_sys_ftime_6x },		/* 35 = ftime_6x */
	{ 0, 0,
	    sys_sync },				/* 36 = sync */
	{ 2, s(struct hpux_sys_kill_args),
	    hpux_sys_kill },			/* 37 = kill */
	{ 2, s(struct hpux_sys_stat_args),
	    hpux_sys_stat },			/* 38 = stat */
	{ 0, 0,
	    hpux_sys_setpgrp_6x },		/* 39 = setpgrp_6x */
	{ 2, s(struct hpux_sys_lstat_args),
	    hpux_sys_lstat },			/* 40 = lstat */
	{ 1, s(struct sys_dup_args),
	    sys_dup },				/* 41 = dup */
	{ 0, 0,
	    sys_opipe },			/* 42 = opipe */
	{ 1, s(struct hpux_sys_times_6x_args),
	    hpux_sys_times_6x },		/* 43 = times_6x */
	{ 4, s(struct sys_profil_args),
	    sys_profil },			/* 44 = profil */
	{ 0, 0,
	    sys_nosys },			/* 45 = unimplemented ki_syscall */
	{ 1, s(struct sys_setgid_args),
	    sys_setgid },			/* 46 = setgid */
	{ 0, 0,
	    sys_getgid },			/* 47 = getgid */
	{ 0, 0,
	    sys_nosys },			/* 48 = unimplemented */
	{ 0, 0,
	    sys_nosys },			/* 49 = unimplemented reserved for USG */
	{ 0, 0,
	    sys_nosys },			/* 50 = unimplemented reserved for USG */
	{ 0, 0,
	    sys_nosys },			/* 51 = unimplemented acct */
	{ 0, 0,
	    sys_nosys },			/* 52 = unimplemented set_userthreadid */
	{ 0, 0,
	    sys_nosys },			/* 53 = unimplemented lwp_mutex_unlock_2 */
	{ 3, s(struct hpux_sys_ioctl_args),
	    hpux_sys_ioctl },			/* 54 = ioctl */
	{ 0, 0,
	    sys_nosys },			/* 55 = unimplemented reboot */
	{ 2, s(struct hpux_sys_symlink_args),
	    hpux_sys_symlink },			/* 56 = symlink */
	{ 3, s(struct hpux_sys_utssys_args),
	    hpux_sys_utssys },			/* 57 = utssys */
	{ 3, s(struct hpux_sys_readlink_args),
	    hpux_sys_readlink },		/* 58 = readlink */
	{ 3, s(struct hpux_sys_execve_args),
	    hpux_sys_execve },			/* 59 = execve */
	{ 1, s(struct sys_umask_args),
	    sys_umask },			/* 60 = umask */
	{ 1, s(struct sys_chroot_args),
	    sys_chroot },			/* 61 = chroot */
	{ 3, s(struct hpux_sys_fcntl_args),
	    hpux_sys_fcntl },			/* 62 = fcntl */
	{ 2, s(struct hpux_sys_ulimit_args),
	    hpux_sys_ulimit },			/* 63 = ulimit */
	{ 0, 0,
	    sys_nosys },			/* 64 = unimplemented */
	{ 0, 0,
	    sys_nosys },			/* 65 = unimplemented */
	{ 0, 0,
	    hpux_sys_vfork },			/* 66 = vfork */
	{ 0, 0,
	    sys_nosys },			/* 67 = unimplemented lwp_getprivate */
	{ 0, 0,
	    sys_nosys },			/* 68 = unimplemented lwp_setprivate */
	{ 0, 0,
	    sys_nosys },			/* 69 = unimplemented */
	{ 0, 0,
	    sys_nosys },			/* 70 = unimplemented */
	{ 6, s(struct hpux_sys_mmap_args),
	    hpux_sys_mmap },			/* 71 = mmap */
	{ 0, 0,
	    sys_nosys },			/* 72 = unimplemented */
	{ 2, s(struct sys_munmap_args),
	    sys_munmap },			/* 73 = munmap */
	{ 3, s(struct sys_mprotect_args),
	    sys_mprotect },			/* 74 = mprotect */
	{ 3, s(struct sys_madvise_args),
	    sys_madvise },			/* 75 = madvise */
	{ 0, 0,
	    sys_nosys },			/* 76 = unimplemented vhangup */
	{ 0, 0,
	    sys_nosys },			/* 77 = unimplemented swapoff */
	{ 0, 0,
	    sys_nosys },			/* 78 = unimplemented */
	{ 2, s(struct sys_getgroups_args),
	    sys_getgroups },			/* 79 = getgroups */
	{ 2, s(struct sys_setgroups_args),
	    sys_setgroups },			/* 80 = setgroups */
	{ 1, s(struct hpux_sys_getpgrp2_args),
	    hpux_sys_getpgrp2 },		/* 81 = getpgrp2 */
	{ 2, s(struct hpux_sys_setpgrp2_args),
	    hpux_sys_setpgrp2 },		/* 82 = setpgrp2 */
	{ 3, s(struct sys_setitimer_args),
	    sys_setitimer },			/* 83 = setitimer */
	{ 3, s(struct hpux_sys_wait3_args),
	    hpux_sys_wait3 },			/* 84 = wait3 */
	{ 0, 0,
	    sys_nosys },			/* 85 = unimplemented swapon */
	{ 2, s(struct sys_getitimer_args),
	    sys_getitimer },			/* 86 = getitimer */
	{ 0, 0,
	    sys_nosys },			/* 87 = unimplemented */
	{ 0, 0,
	    sys_nosys },			/* 88 = unimplemented */
	{ 0, 0,
	    sys_nosys },			/* 89 = unimplemented */
	{ 2, s(struct sys_dup2_args),
	    sys_dup2 },				/* 90 = dup2 */
	{ 0, 0,
	    sys_nosys },			/* 91 = unimplemented */
	{ 2, s(struct hpux_sys_fstat_args),
	    hpux_sys_fstat },			/* 92 = fstat */
	{ 5, s(struct sys_select_args),
	    sys_select },			/* 93 = select */
	{ 0, 0,
	    sys_nosys },			/* 94 = unimplemented */
	{ 1, s(struct sys_fsync_args),
	    sys_fsync },			/* 95 = fsync */
	{ 3, s(struct sys_setpriority_args),
	    sys_setpriority },			/* 96 = setpriority */
	{ 0, 0,
	    sys_nosys },			/* 97 = unimplemented */
	{ 0, 0,
	    sys_nosys },			/* 98 = unimplemented */
	{ 0, 0,
	    sys_nosys },			/* 99 = unimplemented */
	{ 2, s(struct sys_getpriority_args),
	    sys_getpriority },			/* 100 = getpriority */
	{ 0, 0,
	    sys_nosys },			/* 101 = unimplemented */
	{ 0, 0,
	    sys_nosys },			/* 102 = unimplemented */
	{ 0, 0,
	    sys_nosys },			/* 103 = unimplemented lf_send */
	{ 0, 0,
	    sys_nosys },			/* 104 = unimplemented */
	{ 0, 0,
	    sys_nosys },			/* 105 = unimplemented */
	{ 0, 0,
	    sys_nosys },			/* 106 = unimplemented */
	{ 0, 0,
	    sys_nosys },			/* 107 = unimplemented */
	{ 3, s(struct hpux_sys_sigvec_args),
	    hpux_sys_sigvec },			/* 108 = sigvec */
	{ 1, s(struct hpux_sys_sigblock_args),
	    hpux_sys_sigblock },		/* 109 = sigblock */
	{ 1, s(struct hpux_sys_sigsetmask_args),
	    hpux_sys_sigsetmask },		/* 110 = sigsetmask */
	{ 1, s(struct hpux_sys_sigpause_args),
	    hpux_sys_sigpause },		/* 111 = sigpause */
	{ 2, s(struct compat_43_sys_sigstack_args),
	    compat_43_sys_sigstack },		/* 112 = sigstack */
	{ 0, 0,
	    sys_nosys },			/* 113 = unimplemented */
	{ 0, 0,
	    sys_nosys },			/* 114 = unimplemented */
	{ 0, 0,
	    sys_nosys },			/* 115 = unimplemented regctl */
	{ 1, s(struct sys_gettimeofday_args),
	    sys_gettimeofday },			/* 116 = gettimeofday */
	{ 0, 0,
	    sys_nosys },			/* 117 = unimplemented getrusage */
	{ 0, 0,
	    sys_nosys },			/* 118 = unimplemented */
	{ 0, 0,
	    sys_nosys },			/* 119 = unimplemented */
	{ 3, s(struct hpux_sys_readv_args),
	    hpux_sys_readv },			/* 120 = readv */
	{ 3, s(struct hpux_sys_writev_args),
	    hpux_sys_writev },			/* 121 = writev */
	{ 2, s(struct sys_settimeofday_args),
	    sys_settimeofday },			/* 122 = settimeofday */
	{ 3, s(struct sys_fchown_args),
	    sys_fchown },			/* 123 = fchown */
	{ 2, s(struct sys_fchmod_args),
	    sys_fchmod },			/* 124 = fchmod */
	{ 0, 0,
	    sys_nosys },			/* 125 = unimplemented */
	{ 3, s(struct sys_setresuid_args),
	    sys_setresuid },			/* 126 = setresuid */
	{ 3, s(struct sys_setresgid_args),
	    sys_setresgid },			/* 127 = setresgid */
	{ 2, s(struct hpux_sys_rename_args),
	    hpux_sys_rename },			/* 128 = rename */
	{ 2, s(struct hpux_sys_truncate_args),
	    hpux_sys_truncate },		/* 129 = truncate */
	{ 2, s(struct compat_43_sys_ftruncate_args),
	    compat_43_sys_ftruncate },		/* 130 = ftruncate */
	{ 0, 0,
	    sys_nosys },			/* 131 = unimplemented */
	{ 1, s(struct hpux_sys_sysconf_args),
	    hpux_sys_sysconf },			/* 132 = sysconf */
	{ 0, 0,
	    sys_nosys },			/* 133 = unimplemented */
	{ 0, 0,
	    sys_nosys },			/* 134 = unimplemented */
	{ 0, 0,
	    sys_nosys },			/* 135 = unimplemented */
	{ 2, s(struct hpux_sys_mkdir_args),
	    hpux_sys_mkdir },			/* 136 = mkdir */
	{ 1, s(struct hpux_sys_rmdir_args),
	    hpux_sys_rmdir },			/* 137 = rmdir */
	{ 0, 0,
	    sys_nosys },			/* 138 = unimplemented */
	{ 0, 0,
	    sys_nosys },			/* 139 = unimplemented sigcleanup */
	{ 0, 0,
	    sys_nosys },			/* 140 = unimplemented setcore */
	{ 0, 0,
	    sys_nosys },			/* 141 = unimplemented */
	{ 0, 0,
	    sys_nosys },			/* 142 = unimplemented */
	{ 0, 0,
	    sys_nosys },			/* 143 = unimplemented */
	{ 2, s(struct hpux_sys_getrlimit_args),
	    hpux_sys_getrlimit },		/* 144 = getrlimit */
	{ 2, s(struct hpux_sys_setrlimit_args),
	    hpux_sys_setrlimit },		/* 145 = setrlimit */
	{ 0, 0,
	    sys_nosys },			/* 146 = unimplemented */
	{ 0, 0,
	    sys_nosys },			/* 147 = unimplemented lwp_self */
	{ 0, 0,
	    sys_nosys },			/* 148 = unimplemented quotactl */
	{ 0, 0,
	    sys_nosys },			/* 149 = unimplemented get_sysinfo */
	{ 0, 0,
	    sys_nosys },			/* 150 = unimplemented */
	{ 0, 0,
	    sys_nosys },			/* 151 = unimplemented privgrp */
	{ 2, s(struct hpux_sys_rtprio_args),
	    hpux_sys_rtprio },			/* 152 = rtprio */
	{ 0, 0,
	    sys_nosys },			/* 153 = unimplemented plock */
	{ 0, 0,
	    sys_nosys },			/* 154 = unimplemented lf_next_scn */
	{ 3, s(struct hpux_sys_lockf_args),
	    hpux_sys_lockf },			/* 155 = lockf */
#ifdef SYSVSEM
	{ 3, s(struct sys_semget_args),
	    sys_semget },			/* 156 = semget */
	{ 4, s(struct sys___semctl_args),
	    sys___semctl },			/* 157 = __semctl */
	{ 3, s(struct sys_semop_args),
	    sys_semop },			/* 158 = semop */
#else
	{ 0, 0,
	    sys_nosys },			/* 156 = unimplemented semget */
	{ 0, 0,
	    sys_nosys },			/* 157 = unimplemented semctl */
	{ 0, 0,
	    sys_nosys },			/* 158 = unimplemented semop */
#endif
#ifdef SYSVMSG
	{ 2, s(struct sys_msgget_args),
	    sys_msgget },			/* 159 = msgget */
	{ 3, s(struct sys_msgctl_args),
	    sys_msgctl },			/* 160 = msgctl */
	{ 4, s(struct sys_msgsnd_args),
	    sys_msgsnd },			/* 161 = msgsnd */
	{ 5, s(struct sys_msgrcv_args),
	    sys_msgrcv },			/* 162 = msgrcv */
#else
	{ 0, 0,
	    sys_nosys },			/* 159 = unimplemented msgget */
	{ 0, 0,
	    sys_nosys },			/* 160 = unimplemented msgctl */
	{ 0, 0,
	    sys_nosys },			/* 161 = unimplemented msgsnd */
	{ 0, 0,
	    sys_nosys },			/* 162 = unimplemented msgrcv */
#endif
#ifdef SYSVSHM
	{ 3, s(struct sys_shmget_args),
	    sys_shmget },			/* 163 = shmget */
	{ 3, s(struct hpux_sys_shmctl_args),
	    hpux_sys_shmctl },			/* 164 = shmctl */
	{ 3, s(struct sys_shmat_args),
	    sys_shmat },			/* 165 = shmat */
	{ 1, s(struct sys_shmdt_args),
	    sys_shmdt },			/* 166 = shmdt */
#else
	{ 0, 0,
	    sys_nosys },			/* 163 = unimplemented shmget */
	{ 0, 0,
	    sys_nosys },			/* 164 = unimplemented shmctl */
	{ 0, 0,
	    sys_nosys },			/* 165 = unimplemented shmat */
	{ 0, 0,
	    sys_nosys },			/* 166 = unimplemented shmdt */
#endif
	{ 0, 0,
	    sys_nosys },			/* 167 = unimplemented set_mem_window */
	{ 0, 0,
	    sys_nosys },			/* 168 = unimplemented nsp_init */
	{ 0, 0,
	    sys_nosys },			/* 169 = unimplemented */
	{ 0, 0,
	    sys_nosys },			/* 170 = unimplemented mkrnod */
	{ 0, 0,
	    sys_nosys },			/* 171 = unimplemented test */
	{ 0, 0,
	    sys_nosys },			/* 172 = unimplemented unsp_open */
	{ 0, 0,
	    sys_nosys },			/* 173 = unimplemented */
	{ 2, s(struct hpux_sys_getcontext_args),
	    hpux_sys_getcontext },		/* 174 = getcontext */
	{ 0, 0,
	    sys_nosys },			/* 175 = unimplemented osetcontext */
	{ 0, 0,
	    sys_nosys },			/* 176 = unimplemented bigio */
	{ 0, 0,
	    sys_nosys },			/* 177 = unimplemented pipenode */
	{ 0, 0,
	    sys_nosys },			/* 178 = unimplemented lsync */
	{ 0, 0,
	    sys_nosys },			/* 179 = unimplemented */
	{ 0, 0,
	    sys_nosys },			/* 180 = unimplemented mysite */
	{ 0, 0,
	    sys_nosys },			/* 181 = unimplemented sitels */
	{ 0, 0,
	    sys_nosys },			/* 182 = unimplemented swapclients */
	{ 0, 0,
	    sys_nosys },			/* 183 = unimplemented rmtprocess */
	{ 0, 0,
	    sys_nosys },			/* 184 = unimplemented dskless_stats */
	{ 3, s(struct hpux_sys_sigprocmask_args),
	    hpux_sys_sigprocmask },		/* 185 = sigprocmask */
	{ 1, s(struct hpux_sys_sigpending_args),
	    hpux_sys_sigpending },		/* 186 = sigpending */
	{ 1, s(struct hpux_sys_sigsuspend_args),
	    hpux_sys_sigsuspend },		/* 187 = sigsuspend */
	{ 3, s(struct hpux_sys_sigaction_args),
	    hpux_sys_sigaction },		/* 188 = sigaction */
	{ 0, 0,
	    sys_nosys },			/* 189 = unimplemented lw_get_thread_times */
	{ 0, 0,
	    sys_nosys },			/* 190 = unimplemented nfssvc */
	{ 0, 0,
	    sys_nosys },			/* 191 = unimplemented getfh */
	{ 2, s(struct compat_09_sys_getdomainname_args),
	    compat_09_sys_getdomainname },	/* 192 = getdomainname */
	{ 2, s(struct compat_09_sys_setdomainname_args),
	    compat_09_sys_setdomainname },	/* 193 = setdomainname */
	{ 0, 0,
	    sys_nosys },			/* 194 = unimplemented async_daemon */
	{ 4, s(struct compat_43_sys_getdirentries_args),
	    compat_43_sys_getdirentries },	/* 195 = getdirentries */
	{ 0, 0,
	    sys_nosys },			/* 196 = unimplemented statfs */
	{ 0, 0,
	    sys_nosys },			/* 197 = unimplemented fstatfs */
	{ 0, 0,
	    sys_nosys },			/* 198 = unimplemented vfsmount */
	{ 0, 0,
	    sys_nosys },			/* 199 = unimplemented qmml */
	{ 4, s(struct hpux_sys_waitpid_args),
	    hpux_sys_waitpid },			/* 200 = waitpid */
	{ 0, 0,
	    sys_nosys },			/* 201 = unimplemented */
	{ 0, 0,
	    sys_nosys },			/* 202 = unimplemented */
	{ 0, 0,
	    sys_nosys },			/* 203 = unimplemented */
	{ 0, 0,
	    sys_nosys },			/* 204 = unimplemented */
	{ 0, 0,
	    sys_nosys },			/* 205 = unimplemented */
	{ 0, 0,
	    sys_nosys },			/* 206 = unimplemented */
	{ 0, 0,
	    sys_nosys },			/* 207 = unimplemented */
	{ 0, 0,
	    sys_nosys },			/* 208 = unimplemented */
	{ 0, 0,
	    sys_nosys },			/* 209 = unimplemented */
	{ 0, 0,
	    sys_nosys },			/* 210 = unimplemented */
	{ 0, 0,
	    sys_nosys },			/* 211 = unimplemented */
	{ 0, 0,
	    sys_nosys },			/* 212 = unimplemented */
	{ 0, 0,
	    sys_nosys },			/* 213 = unimplemented */
	{ 0, 0,
	    sys_nosys },			/* 214 = unimplemented */
	{ 0, 0,
	    sys_nosys },			/* 215 = unimplemented */
	{ 0, 0,
	    sys_nosys },			/* 216 = unimplemented */
	{ 0, 0,
	    sys_nosys },			/* 217 = unimplemented */
	{ 0, 0,
	    sys_nosys },			/* 218 = unimplemented */
	{ 0, 0,
	    sys_nosys },			/* 219 = unimplemented */
	{ 0, 0,
	    sys_nosys },			/* 220 = unimplemented */
	{ 0, 0,
	    sys_nosys },			/* 221 = unimplemented */
	{ 0, 0,
	    sys_nosys },			/* 222 = unimplemented */
	{ 0, 0,
	    sys_nosys },			/* 223 = unimplemented */
	{ 1, s(struct hpux_sigsetreturn_args),
	    hpux_sigsetreturn },		/* 224 = hpux_sigsetreturn */
	{ 0, 0,
	    sys_nosys },			/* 225 = unimplemented sigsetstatemask */
	{ 0, 0,
	    sys_nosys },			/* 226 = unimplemented */
	{ 0, 0,
	    sys_nosys },			/* 227 = unimplemented cs */
	{ 0, 0,
	    sys_nosys },			/* 228 = unimplemented cds */
	{ 0, 0,
	    sys_nosys },			/* 229 = unimplemented set_no_trunc */
	{ 0, 0,
	    sys_nosys },			/* 230 = unimplemented pathconf */
	{ 0, 0,
	    sys_nosys },			/* 231 = unimplemented fpathconf */
	{ 0, 0,
	    sys_nosys },			/* 232 = unimplemented */
	{ 0, 0,
	    sys_nosys },			/* 233 = unimplemented */
	{ 0, 0,
	    sys_nosys },			/* 234 = unimplemented nfs_fcntl */
	{ 0, 0,
	    sys_nosys },			/* 235 = unimplemented ogetacl */
	{ 0, 0,
	    sys_nosys },			/* 236 = unimplemented ofgetctl */
	{ 0, 0,
	    sys_nosys },			/* 237 = unimplemented osetacl */
	{ 0, 0,
	    sys_nosys },			/* 238 = unimplemented ofsetacl */
	{ 0, 0,
	    sys_nosys },			/* 239 = unimplemented pstat */
	{ 0, 0,
	    sys_nosys },			/* 240 = unimplemented getaudid */
	{ 0, 0,
	    sys_nosys },			/* 241 = unimplemented setaudid */
	{ 0, 0,
	    sys_nosys },			/* 242 = unimplemented getaudproc */
	{ 0, 0,
	    sys_nosys },			/* 243 = unimplemented setaudproc */
	{ 0, 0,
	    sys_nosys },			/* 244 = unimplemented getevent */
	{ 0, 0,
	    sys_nosys },			/* 245 = unimplemented setevent */
	{ 0, 0,
	    sys_nosys },			/* 246 = unimplemented audwrite */
	{ 0, 0,
	    sys_nosys },			/* 247 = unimplemented audswitch */
	{ 0, 0,
	    sys_nosys },			/* 248 = unimplemented audctl */
	{ 0, 0,
	    sys_nosys },			/* 249 = unimplemented ogetaccess */
	{ 0, 0,
	    sys_nosys },			/* 250 = unimplemented fsctl */
	{ 0, 0,
	    sys_nosys },			/* 251 = unimplemented */
	{ 0, 0,
	    sys_nosys },			/* 252 = unimplemented getmount_entries */
	{ 0, 0,
	    sys_nosys },			/* 253 = unimplemented lwp_mutex_init2 */
	{ 0, 0,
	    sys_nosys },			/* 254 = unimplemented */
	{ 0, 0,
	    sys_nosys },			/* 255 = unimplemented */
	{ 0, 0,
	    sys_nosys },			/* 256 = unimplemented */
	{ 0, 0,
	    sys_nosys },			/* 257 = unimplemented */
	{ 0, 0,
	    sys_nosys },			/* 258 = unimplemented */
	{ 0, 0,
	    sys_nosys },			/* 259 = unimplemented swapfs */
	{ 0, 0,
	    sys_nosys },			/* 260 = unimplemented fss */
	{ 0, 0,
	    sys_nosys },			/* 261 = unimplemented */
	{ 0, 0,
	    sys_nosys },			/* 262 = unimplemented */
	{ 0, 0,
	    sys_nosys },			/* 263 = unimplemented */
	{ 0, 0,
	    sys_nosys },			/* 264 = unimplemented */
	{ 0, 0,
	    sys_nosys },			/* 265 = unimplemented */
	{ 0, 0,
	    sys_nosys },			/* 266 = unimplemented */
	{ 0, 0,
	    sys_nosys },			/* 267 = unimplemented tsync */
	{ 0, 0,
	    sys_nosys },			/* 268 = unimplemented getnumfds */
	{ 3, s(struct sys_poll_args),
	    sys_poll },				/* 269 = poll */
	{ 0, 0,
	    sys_nosys },			/* 270 = unimplemented getmsg */
	{ 0, 0,
	    sys_nosys },			/* 271 = unimplemented putmsg */
	{ 1, s(struct sys_fchdir_args),
	    sys_fchdir },			/* 272 = fchdir */
	{ 0, 0,
	    sys_nosys },			/* 273 = unimplemented getmount_cnt */
	{ 0, 0,
	    sys_nosys },			/* 274 = unimplemented getmount_entry */
	{ 3, s(struct compat_43_sys_accept_args),
	    compat_43_sys_accept },		/* 275 = accept */
	{ 3, s(struct sys_bind_args),
	    sys_bind },				/* 276 = bind */
	{ 3, s(struct sys_connect_args),
	    sys_connect },			/* 277 = connect */
	{ 3, s(struct sys_getpeername_args),
	    sys_getpeername },			/* 278 = getpeername */
	{ 3, s(struct sys_getsockname_args),
	    sys_getsockname },			/* 279 = getsockname */
	{ 5, s(struct sys_getsockopt_args),
	    sys_getsockopt },			/* 280 = getsockopt */
	{ 2, s(struct sys_listen_args),
	    sys_listen },			/* 281 = listen */
	{ 4, s(struct compat_43_sys_recv_args),
	    compat_43_sys_recv },		/* 282 = recv */
	{ 6, s(struct compat_43_sys_recvfrom_args),
	    compat_43_sys_recvfrom },		/* 283 = recvfrom */
	{ 3, s(struct compat_43_sys_recvmsg_args),
	    compat_43_sys_recvmsg },		/* 284 = recvmsg */
	{ 4, s(struct compat_43_sys_send_args),
	    compat_43_sys_send },		/* 285 = send */
	{ 3, s(struct compat_43_sys_sendmsg_args),
	    compat_43_sys_sendmsg },		/* 286 = sendmsg */
	{ 6, s(struct sys_sendto_args),
	    sys_sendto },			/* 287 = sendto */
	{ 5, s(struct sys_setsockopt_args),
	    sys_setsockopt },			/* 288 = setsockopt */
	{ 2, s(struct sys_shutdown_args),
	    sys_shutdown },			/* 289 = shutdown */
	{ 3, s(struct sys_socket_args),
	    sys_socket },			/* 290 = socket */
	{ 4, s(struct sys_socketpair_args),
	    sys_socketpair },			/* 291 = socketpair */
	{ 0, 0,
	    sys_nosys },			/* 292 = unimplemented proc_open */
	{ 0, 0,
	    sys_nosys },			/* 293 = unimplemented proc_close */
	{ 0, 0,
	    sys_nosys },			/* 294 = unimplemented proc_send */
	{ 0, 0,
	    sys_nosys },			/* 295 = unimplemented proc_recv */
	{ 0, 0,
	    sys_nosys },			/* 296 = unimplemented proc_sendrecv */
	{ 0, 0,
	    sys_nosys },			/* 297 = unimplemented proc_syscall */
	{ 0, 0,
	    sys_nosys },			/* 298 = unimplemented */
	{ 0, 0,
	    sys_nosys },			/* 299 = unimplemented */
	{ 0, 0,
	    sys_nosys },			/* 300 = unimplemented */
	{ 0, 0,
	    sys_nosys },			/* 301 = unimplemented */
	{ 0, 0,
	    sys_nosys },			/* 302 = unimplemented */
	{ 0, 0,
	    sys_nosys },			/* 303 = unimplemented */
	{ 0, 0,
	    sys_nosys },			/* 304 = unimplemented */
	{ 0, 0,
	    sys_nosys },			/* 305 = unimplemented */
	{ 0, 0,
	    sys_nosys },			/* 306 = unimplemented */
	{ 0, 0,
	    sys_nosys },			/* 307 = unimplemented */
	{ 0, 0,
	    sys_nosys },			/* 308 = unimplemented */
	{ 0, 0,
	    sys_nosys },			/* 309 = unimplemented */
	{ 0, 0,
	    sys_nosys },			/* 310 = unimplemented */
	{ 0, 0,
	    sys_nosys },			/* 311 = unimplemented */
#ifdef SYSVSEM
	{ 4, s(struct sys___semctl_args),
	    sys___semctl },			/* 312 = nsemctl */
#else
	{ 0, 0,
	    sys_nosys },			/* 312 = unimplemented semctl */
#endif
#ifdef SYSVMSG
	{ 3, s(struct sys_msgctl_args),
	    sys_msgctl },			/* 313 = nmsgctl */
#else
	{ 0, 0,
	    sys_nosys },			/* 313 = unimplemented msgctl */
#endif
#ifdef SYSVSHM
	{ 3, s(struct hpux_sys_nshmctl_args),
	    hpux_sys_nshmctl },			/* 314 = nshmctl */
#else
	{ 0, 0,
	    sys_nosys },			/* 314 = unimplemented shmctl */
#endif
	{ 0, 0,
	    sys_nosys },			/* 315 = unimplemented mpctl */
	{ 0, 0,
	    sys_nosys },			/* 316 = unimplemented exportfs */
	{ 0, 0,
	    sys_nosys },			/* 317 = unimplemented getpmsg */
	{ 0, 0,
	    sys_nosys },			/* 318 = unimplemented putpmsg */
	{ 0, 0,
	    sys_nosys },			/* 319 = unimplemented */
	{ 3, s(struct sys_msync_args),
	    sys_msync },			/* 320 = msync */
	{ 0, 0,
	    sys_nosys },			/* 321 = unimplemented msleep */
	{ 0, 0,
	    sys_nosys },			/* 322 = unimplemented mwakeup */
	{ 0, 0,
	    sys_nosys },			/* 323 = unimplemented msem_init */
	{ 0, 0,
	    sys_nosys },			/* 324 = unimplemented msem_remove */
	{ 0, 0,
	    sys_nosys },			/* 325 = unimplemented adjtime */
	{ 0, 0,
	    sys_nosys },			/* 326 = unimplemented kload */
	{ 0, 0,
	    sys_nosys },			/* 327 = unimplemented fattach */
	{ 0, 0,
	    sys_nosys },			/* 328 = unimplemented fdetach */
	{ 0, 0,
	    sys_nosys },			/* 329 = unimplemented serialize */
	{ 0, 0,
	    sys_nosys },			/* 330 = unimplemented statvfs */
	{ 0, 0,
	    sys_nosys },			/* 331 = unimplemented fstatvfs */
	{ 3, s(struct sys_lchown_args),
	    sys_lchown },			/* 332 = lchown */
	{ 0, 0,
	    sys_nosys },			/* 333 = unimplemented getsid */
	{ 0, 0,
	    sys_nosys },			/* 334 = unimplemented sysfs */
	{ 0, 0,
	    sys_nosys },			/* 335 = unimplemented */
	{ 0, 0,
	    sys_nosys },			/* 336 = unimplemented */
	{ 0, 0,
	    sys_nosys },			/* 337 = unimplemented sched_setparam */
	{ 0, 0,
	    sys_nosys },			/* 338 = unimplemented sched_getparam */
	{ 0, 0,
	    sys_nosys },			/* 339 = unimplemented sched_setscheduler */
	{ 0, 0,
	    sys_nosys },			/* 340 = unimplemented sched_getscheduler */
	{ 0, 0,
	    sys_nosys },			/* 341 = unimplemented sched_yield */
	{ 0, 0,
	    sys_nosys },			/* 342 = unimplemented sched_get_priority_max */
	{ 0, 0,
	    sys_nosys },			/* 343 = unimplemented sched_get_priority_min */
	{ 0, 0,
	    sys_nosys },			/* 344 = unimplemented sched_rr_get_interval */
	{ 0, 0,
	    sys_nosys },			/* 345 = unimplemented clock_settime */
	{ 0, 0,
	    sys_nosys },			/* 346 = unimplemented clock_gettime */
	{ 0, 0,
	    sys_nosys },			/* 347 = unimplemented clock_getres */
	{ 0, 0,
	    sys_nosys },			/* 348 = unimplemented timer_create */
	{ 0, 0,
	    sys_nosys },			/* 349 = unimplemented timer_delete */
	{ 0, 0,
	    sys_nosys },			/* 350 = unimplemented timer_settime */
	{ 0, 0,
	    sys_nosys },			/* 351 = unimplemented timer_gettime */
	{ 0, 0,
	    sys_nosys },			/* 352 = unimplemented timer_getoverrun */
	{ 2, s(struct sys_nanosleep_args),
	    sys_nanosleep },			/* 353 = nanosleep */
	{ 0, 0,
	    sys_nosys },			/* 354 = unimplemented toolbox */
	{ 0, 0,
	    sys_nosys },			/* 355 = unimplemented */
	{ 0, 0,
	    sys_nosys },			/* 356 = unimplemented getdents */
	{ 0, 0,
	    sys_nosys },			/* 357 = unimplemented getcontext */
	{ 0, 0,
	    sys_nosys },			/* 358 = unimplemented sysinfo */
	{ 0, 0,
	    sys_nosys },			/* 359 = unimplemented fcntl64 */
	{ 0, 0,
	    sys_nosys },			/* 360 = unimplemented ftruncate64 */
	{ 0, 0,
	    sys_nosys },			/* 361 = unimplemented fstat64 */
	{ 0, 0,
	    sys_nosys },			/* 362 = unimplemented getdirentries64 */
	{ 0, 0,
	    sys_nosys },			/* 363 = unimplemented getrlimit64 */
	{ 0, 0,
	    sys_nosys },			/* 364 = unimplemented lockf64 */
	{ 0, 0,
	    sys_nosys },			/* 365 = unimplemented lseek64 */
	{ 0, 0,
	    sys_nosys },			/* 366 = unimplemented lstat64 */
	{ 0, 0,
	    sys_nosys },			/* 367 = unimplemented mmap64 */
	{ 0, 0,
	    sys_nosys },			/* 368 = unimplemented setrlimit64 */
	{ 0, 0,
	    sys_nosys },			/* 369 = unimplemented stat64 */
	{ 0, 0,
	    sys_nosys },			/* 370 = unimplemented truncate64 */
	{ 0, 0,
	    sys_nosys },			/* 371 = unimplemented ulimit64 */
	{ 0, 0,
	    sys_nosys },			/* 372 = unimplemented pread */
	{ 0, 0,
	    sys_nosys },			/* 373 = unimplemented preadv */
	{ 0, 0,
	    sys_nosys },			/* 374 = unimplemented pwrite */
	{ 0, 0,
	    sys_nosys },			/* 375 = unimplemented pwritev */
	{ 0, 0,
	    sys_nosys },			/* 376 = unimplemented pread64 */
	{ 0, 0,
	    sys_nosys },			/* 377 = unimplemented preadv64 */
	{ 0, 0,
	    sys_nosys },			/* 378 = unimplemented pwrite64 */
	{ 0, 0,
	    sys_nosys },			/* 379 = unimplemented pwritev64 */
	{ 0, 0,
	    sys_nosys },			/* 380 = unimplemented setcontext */
	{ 2, s(struct hpux_sys_sigaltstack_args),
	    hpux_sys_sigaltstack },		/* 381 = sigaltstack */
	{ 0, 0,
	    sys_nosys },			/* 382 = unimplemented waitid */
	{ 0, 0,
	    sys_nosys },			/* 383 = unimplemented setpgrp */
	{ 0, 0,
	    sys_nosys },			/* 384 = unimplemented recvmsg2 */
	{ 0, 0,
	    sys_nosys },			/* 385 = unimplemented sendmsg2 */
	{ 0, 0,
	    sys_nosys },			/* 386 = unimplemented socket2 */
	{ 0, 0,
	    sys_nosys },			/* 387 = unimplemented socketpair2 */
	{ 0, 0,
	    sys_nosys },			/* 388 = unimplemented setregid */
	{ 0, 0,
	    sys_nosys },			/* 389 = unimplemented lwp_create */
	{ 0, 0,
	    sys_nosys },			/* 390 = unimplemented lwp_terminate */
	{ 0, 0,
	    sys_nosys },			/* 391 = unimplemented lwp_wait */
	{ 0, 0,
	    sys_nosys },			/* 392 = unimplemented lwp_suspend */
	{ 0, 0,
	    sys_nosys },			/* 393 = unimplemented lwp_resume */
	{ 0, 0,
	    sys_nosys },			/* 394 = unimplemented */
	{ 0, 0,
	    sys_nosys },			/* 395 = unimplemented lwp_abort_syscall */
	{ 0, 0,
	    sys_nosys },			/* 396 = unimplemented lwp_info */
	{ 0, 0,
	    sys_nosys },			/* 397 = unimplemented lwp_kill */
	{ 0, 0,
	    sys_nosys },			/* 398 = unimplemented ksleep */
	{ 0, 0,
	    sys_nosys },			/* 399 = unimplemented kwakeup */
	{ 0, 0,
	    sys_nosys },			/* 400 = unimplemented */
	{ 0, 0,
	    sys_nosys },			/* 401 = unimplemented pstat_getlwp */
	{ 0, 0,
	    sys_nosys },			/* 402 = unimplemented lwp_exit */
	{ 0, 0,
	    sys_nosys },			/* 403 = unimplemented lwp_continue */
	{ 0, 0,
	    sys_nosys },			/* 404 = unimplemented getacl */
	{ 0, 0,
	    sys_nosys },			/* 405 = unimplemented fgetacl */
	{ 0, 0,
	    sys_nosys },			/* 406 = unimplemented setacl */
	{ 0, 0,
	    sys_nosys },			/* 407 = unimplemented fsetacl */
	{ 0, 0,
	    sys_nosys },			/* 408 = unimplemented getaccess */
	{ 0, 0,
	    sys_nosys },			/* 409 = unimplemented lwp_mutex_init */
	{ 0, 0,
	    sys_nosys },			/* 410 = unimplemented lwp_mutex_lock_sys */
	{ 0, 0,
	    sys_nosys },			/* 411 = unimplemented lwp_mutex_unlock */
	{ 0, 0,
	    sys_nosys },			/* 412 = unimplemented lwp_cond_init */
	{ 0, 0,
	    sys_nosys },			/* 413 = unimplemented lwp_cond_signal */
	{ 0, 0,
	    sys_nosys },			/* 414 = unimplemented lwp_cond_broadcast */
	{ 0, 0,
	    sys_nosys },			/* 415 = unimplemented lwp_cond_wait_sys */
	{ 0, 0,
	    sys_nosys },			/* 416 = unimplemented lwp_getscheduler */
	{ 0, 0,
	    sys_nosys },			/* 417 = unimplemented lwp_setscheduler */
	{ 0, 0,
	    sys_nosys },			/* 418 = unimplemented lwp_getstate */
	{ 0, 0,
	    sys_nosys },			/* 419 = unimplemented lwp_setstate */
	{ 0, 0,
	    sys_nosys },			/* 420 = unimplemented lwp_detach */
	{ 2, s(struct sys_mlock_args),
	    sys_mlock },			/* 421 = mlock */
	{ 2, s(struct sys_munlock_args),
	    sys_munlock },			/* 422 = munlock */
	{ 1, s(struct sys_mlockall_args),
	    sys_mlockall },			/* 423 = mlockall */
	{ 0, 0,
	    sys_munlockall },			/* 424 = munlockall */
	{ 0, 0,
	    sys_nosys },			/* 425 = unimplemented shm_open */
	{ 0, 0,
	    sys_nosys },			/* 426 = unimplemented shm_unlink */
	{ 0, 0,
	    sys_nosys },			/* 427 = unimplemented sigqueue */
	{ 0, 0,
	    sys_nosys },			/* 428 = unimplemented sigwaitinfo */
	{ 0, 0,
	    sys_nosys },			/* 429 = unimplemented sigtimedwait */
	{ 0, 0,
	    sys_nosys },			/* 430 = unimplemented sigwait */
	{ 0, 0,
	    sys_nosys },			/* 431 = unimplemented aio_read */
	{ 0, 0,
	    sys_nosys },			/* 432 = unimplemented aio_write */
	{ 0, 0,
	    sys_nosys },			/* 433 = unimplemented lio_listio */
	{ 0, 0,
	    sys_nosys },			/* 434 = unimplemented aio_error */
	{ 0, 0,
	    sys_nosys },			/* 435 = unimplemented aio_return */
	{ 0, 0,
	    sys_nosys },			/* 436 = unimplemented aio_cancel */
	{ 0, 0,
	    sys_nosys },			/* 437 = unimplemented aio_suspend */
	{ 0, 0,
	    sys_nosys },			/* 438 = unimplemented aio_fsync */
	{ 0, 0,
	    sys_nosys },			/* 439 = unimplemented mq_open */
	{ 0, 0,
	    sys_nosys },			/* 440 = unimplemented mq_close */
	{ 0, 0,
	    sys_nosys },			/* 441 = unimplemented mq_unlink */
	{ 0, 0,
	    sys_nosys },			/* 442 = unimplemented mq_send */
	{ 0, 0,
	    sys_nosys },			/* 443 = unimplemented mq_receive */
	{ 0, 0,
	    sys_nosys },			/* 444 = unimplemented mq_notify */
	{ 0, 0,
	    sys_nosys },			/* 445 = unimplemented mq_setattr */
	{ 0, 0,
	    sys_nosys },			/* 446 = unimplemented mq_getattr */
	{ 0, 0,
	    sys_nosys },			/* 447 = unimplemented ksem_open */
	{ 0, 0,
	    sys_nosys },			/* 448 = unimplemented ksem_unlink */
	{ 0, 0,
	    sys_nosys },			/* 449 = unimplemented ksem_close */
	{ 0, 0,
	    sys_nosys },			/* 450 = unimplemented ksem_post */
	{ 0, 0,
	    sys_nosys },			/* 451 = unimplemented ksem_wait */
	{ 0, 0,
	    sys_nosys },			/* 452 = unimplemented ksem_read */
	{ 0, 0,
	    sys_nosys },			/* 453 = unimplemented ksem_trywait */
	{ 0, 0,
	    sys_nosys },			/* 454 = unimplemented lwp_rwlock_init */
	{ 0, 0,
	    sys_nosys },			/* 455 = unimplemented lwp_rwlock_destroy */
	{ 0, 0,
	    sys_nosys },			/* 456 = unimplemented lwp_rwlock_rdlock_sys */
	{ 0, 0,
	    sys_nosys },			/* 457 = unimplemented lwp_rwlock_wrlock_sys */
	{ 0, 0,
	    sys_nosys },			/* 458 = unimplemented lwp_rwlock_tryrdlock */
	{ 0, 0,
	    sys_nosys },			/* 459 = unimplemented lwp_rwlock_trywrlock */
	{ 0, 0,
	    sys_nosys },			/* 460 = unimplemented lwp_rwlock_unlock */
	{ 0, 0,
	    sys_nosys },			/* 461 = unimplemented ttrace */
	{ 0, 0,
	    sys_nosys },			/* 462 = unimplemented ttrace_wait */
	{ 0, 0,
	    sys_nosys },			/* 463 = unimplemented lf_wire_mem */
	{ 0, 0,
	    sys_nosys },			/* 464 = unimplemented lf_unwire_mem */
	{ 0, 0,
	    sys_nosys },			/* 465 = unimplemented lf_send_pin_map */
	{ 0, 0,
	    sys_nosys },			/* 466 = unimplemented lf_free_buf */
	{ 0, 0,
	    sys_nosys },			/* 467 = unimplemented lf_wait_nq */
	{ 0, 0,
	    sys_nosys },			/* 468 = unimplemented lf_wakeup_conn_q */
	{ 0, 0,
	    sys_nosys },			/* 469 = unimplemented lf_unused */
	{ 0, 0,
	    sys_nosys },			/* 470 = unimplemented lwp_sema_init */
	{ 0, 0,
	    sys_nosys },			/* 471 = unimplemented lwp_sema_post */
	{ 0, 0,
	    sys_nosys },			/* 472 = unimplemented lwp_sema_wait */
	{ 0, 0,
	    sys_nosys },			/* 473 = unimplemented lwp_sema_trywait */
	{ 0, 0,
	    sys_nosys },			/* 474 = unimplemented lwp_sema_destroy */
	{ 0, 0,
	    sys_nosys },			/* 475 = unimplemented statvfs64 */
	{ 0, 0,
	    sys_nosys },			/* 476 = unimplemented fstatvfs64 */
	{ 0, 0,
	    sys_nosys },			/* 477 = unimplemented msh_register */
	{ 0, 0,
	    sys_nosys },			/* 478 = unimplemented ptrace64 */
	{ 0, 0,
	    sys_nosys },			/* 479 = unimplemented sendfile */
	{ 0, 0,
	    sys_nosys },			/* 480 = unimplemented sendpath */
	{ 0, 0,
	    sys_nosys },			/* 481 = unimplemented sendfile64 */
	{ 0, 0,
	    sys_nosys },			/* 482 = unimplemented sendpath64 */
	{ 0, 0,
	    sys_nosys },			/* 483 = unimplemented modload */
	{ 0, 0,
	    sys_nosys },			/* 484 = unimplemented moduload */
	{ 0, 0,
	    sys_nosys },			/* 485 = unimplemented modpath */
	{ 0, 0,
	    sys_nosys },			/* 486 = unimplemented getksym */
	{ 0, 0,
	    sys_nosys },			/* 487 = unimplemented modadm */
	{ 0, 0,
	    sys_nosys },			/* 488 = unimplemented modstat */
	{ 0, 0,
	    sys_nosys },			/* 489 = unimplemented lwp_detached_exit */
	{ 0, 0,
	    sys_nosys },			/* 490 = unimplemented crashconf */
	{ 0, 0,
	    sys_nosys },			/* 491 = unimplemented siginhibit */
	{ 0, 0,
	    sys_nosys },			/* 492 = unimplemented sigenable */
	{ 0, 0,
	    sys_nosys },			/* 493 = unimplemented spuctl */
	{ 0, 0,
	    sys_nosys },			/* 494 = unimplemented zerokernelsum */
	{ 0, 0,
	    sys_nosys },			/* 495 = unimplemented nfs_kstat */
	{ 0, 0,
	    sys_nosys },			/* 496 = unimplemented aio_read64 */
	{ 0, 0,
	    sys_nosys },			/* 497 = unimplemented aio_write64 */
	{ 0, 0,
	    sys_nosys },			/* 498 = unimplemented aio_error64 */
	{ 0, 0,
	    sys_nosys },			/* 499 = unimplemented aio_return64 */
	{ 0, 0,
	    sys_nosys },			/* 500 = unimplemented aio_cancel64 */
	{ 0, 0,
	    sys_nosys },			/* 501 = unimplemented aio_suspend64 */
	{ 0, 0,
	    sys_nosys },			/* 502 = unimplemented aio_fsync64 */
	{ 0, 0,
	    sys_nosys },			/* 503 = unimplemented lio_listio64 */
	{ 0, 0,
	    sys_nosys },			/* 504 = unimplemented recv2 */
	{ 0, 0,
	    sys_nosys },			/* 505 = unimplemented recvfrom2 */
	{ 0, 0,
	    sys_nosys },			/* 506 = unimplemented send2 */
	{ 0, 0,
	    sys_nosys },			/* 507 = unimplemented sendto2 */
	{ 0, 0,
	    sys_nosys },			/* 508 = unimplemented acl */
	{ 0, 0,
	    sys_nosys },			/* 509 = unimplemented __cnx_p2p_ctl */
	{ 0, 0,
	    sys_nosys },			/* 510 = unimplemented __cnx_gsched_ctl */
	{ 0, 0,
	    sys_nosys },			/* 511 = unimplemented __cnx_pmon_ctl */
	{ 0, 0,
	    sys_nosys },			/* 512 = unimplemented mem_res_grp */
	{ 0, 0,
	    sys_nosys },			/* 513 = unimplemented fabric */
	{ 0, 0,
	    sys_nosys },			/* 514 = unimplemented diagsyscall */
	{ 0, 0,
	    sys_nosys },			/* 515 = unimplemented tuneinfo */
	{ 0, 0,
	    sys_nosys },			/* 516 = unimplemented gettune */
	{ 0, 0,
	    sys_nosys },			/* 517 = unimplemented settune */
	{ 0, 0,
	    sys_nosys },			/* 518 = unimplemented pset_create */
	{ 0, 0,
	    sys_nosys },			/* 519 = unimplemented pset_destroy */
	{ 0, 0,
	    sys_nosys },			/* 520 = unimplemented pset_assign */
	{ 0, 0,
	    sys_nosys },			/* 521 = unimplemented pset_bind */
	{ 0, 0,
	    sys_nosys },			/* 522 = unimplemented pset_getattr */
	{ 0, 0,
	    sys_nosys },			/* 523 = unimplemented pset_setattr */
	{ 0, 0,
	    sys_nosys },			/* 524 = unimplemented pset_ctl */
	{ 0, 0,
	    sys_nosys },			/* 525 = unimplemented pset_rtctl */
};

