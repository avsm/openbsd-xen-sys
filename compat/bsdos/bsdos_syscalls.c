/*	$OpenBSD$	*/

/*
 * System call names.
 *
 * DO NOT EDIT-- this file is automatically generated.
 * created from	OpenBSD: syscalls.master,v 1.4 1999/05/31 17:34:44 millert Exp 
 */

char *bsdos_syscallnames[] = {
	"syscall",			/* 0 = syscall */
	"exit",			/* 1 = exit */
	"fork",			/* 2 = fork */
	"read",			/* 3 = read */
	"write",			/* 4 = write */
	"open",			/* 5 = open */
	"close",			/* 6 = close */
	"wait4",			/* 7 = wait4 */
	"ocreat",			/* 8 = ocreat */
	"link",			/* 9 = link */
	"unlink",			/* 10 = unlink */
	"#11 (obsolete execv)",		/* 11 = obsolete execv */
	"chdir",			/* 12 = chdir */
	"fchdir",			/* 13 = fchdir */
	"mknod",			/* 14 = mknod */
	"chmod",			/* 15 = chmod */
	"chown",			/* 16 = chown */
	"break",			/* 17 = break */
	"ogetfsstat",			/* 18 = ogetfsstat */
	"olseek",			/* 19 = olseek */
	"getpid",			/* 20 = getpid */
	"mount",			/* 21 = mount */
	"unmount",			/* 22 = unmount */
	"setuid",			/* 23 = setuid */
	"getuid",			/* 24 = getuid */
	"geteuid",			/* 25 = geteuid */
	"ptrace",			/* 26 = ptrace */
	"recvmsg",			/* 27 = recvmsg */
	"sendmsg",			/* 28 = sendmsg */
	"recvfrom",			/* 29 = recvfrom */
	"accept",			/* 30 = accept */
	"getpeername",			/* 31 = getpeername */
	"getsockname",			/* 32 = getsockname */
	"access",			/* 33 = access */
	"chflags",			/* 34 = chflags */
	"fchflags",			/* 35 = fchflags */
	"sync",			/* 36 = sync */
	"kill",			/* 37 = kill */
	"ostat",			/* 38 = ostat */
	"getppid",			/* 39 = getppid */
	"olstat",			/* 40 = olstat */
	"dup",			/* 41 = dup */
	"pipe",			/* 42 = pipe */
	"getegid",			/* 43 = getegid */
	"profil",			/* 44 = profil */
#ifdef KTRACE
	"ktrace",			/* 45 = ktrace */
#else
	"#45 (unimplemented ktrace)",		/* 45 = unimplemented ktrace */
#endif
	"sigaction",			/* 46 = sigaction */
	"getgid",			/* 47 = getgid */
	"sigprocmask",			/* 48 = sigprocmask */
	"getlogin",			/* 49 = getlogin */
	"setlogin",			/* 50 = setlogin */
	"acct",			/* 51 = acct */
	"sigpending",			/* 52 = sigpending */
	"sigaltstack",			/* 53 = sigaltstack */
	"ioctl",			/* 54 = ioctl */
	"reboot",			/* 55 = reboot */
	"revoke",			/* 56 = revoke */
	"symlink",			/* 57 = symlink */
	"readlink",			/* 58 = readlink */
	"execve",			/* 59 = execve */
	"umask",			/* 60 = umask */
	"chroot",			/* 61 = chroot */
	"ofstat",			/* 62 = ofstat */
	"ogetkerninfo",			/* 63 = ogetkerninfo */
	"ogetpagesize",			/* 64 = ogetpagesize */
	"msync",			/* 65 = msync */
	"vfork",			/* 66 = vfork */
	"#67 (obsolete vread)",		/* 67 = obsolete vread */
	"#68 (obsolete vwrite)",		/* 68 = obsolete vwrite */
	"sbrk",			/* 69 = sbrk */
	"sstk",			/* 70 = sstk */
	"ommap",			/* 71 = ommap */
	"vadvise",			/* 72 = vadvise */
	"munmap",			/* 73 = munmap */
	"mprotect",			/* 74 = mprotect */
	"madvise",			/* 75 = madvise */
	"#76 (obsolete vhangup)",		/* 76 = obsolete vhangup */
	"#77 (obsolete vlimit)",		/* 77 = obsolete vlimit */
	"mincore",			/* 78 = mincore */
	"getgroups",			/* 79 = getgroups */
	"setgroups",			/* 80 = setgroups */
	"getpgrp",			/* 81 = getpgrp */
	"setpgid",			/* 82 = setpgid */
	"setitimer",			/* 83 = setitimer */
	"owait",			/* 84 = owait */
	"swapon",			/* 85 = swapon */
	"getitimer",			/* 86 = getitimer */
	"ogethostname",			/* 87 = ogethostname */
	"osethostname",			/* 88 = osethostname */
	"ogetdtablesize",			/* 89 = ogetdtablesize */
	"dup2",			/* 90 = dup2 */
	"#91 (unimplemented getdopt)",		/* 91 = unimplemented getdopt */
	"fcntl",			/* 92 = fcntl */
	"select",			/* 93 = select */
	"#94 (unimplemented setdopt)",		/* 94 = unimplemented setdopt */
	"fsync",			/* 95 = fsync */
	"setpriority",			/* 96 = setpriority */
	"socket",			/* 97 = socket */
	"connect",			/* 98 = connect */
	"oaccept",			/* 99 = oaccept */
	"getpriority",			/* 100 = getpriority */
	"osend",			/* 101 = osend */
	"orecv",			/* 102 = orecv */
	"sigreturn",			/* 103 = sigreturn */
	"bind",			/* 104 = bind */
	"setsockopt",			/* 105 = setsockopt */
	"listen",			/* 106 = listen */
	"#107 (obsolete vtimes)",		/* 107 = obsolete vtimes */
	"osigvec",			/* 108 = osigvec */
	"osigblock",			/* 109 = osigblock */
	"osigsetmask",			/* 110 = osigsetmask */
	"sigsuspend",			/* 111 = sigsuspend */
	"osigstack",			/* 112 = osigstack */
	"orecvmsg",			/* 113 = orecvmsg */
	"osendmsg",			/* 114 = osendmsg */
#ifdef TRACE
	"vtrace",			/* 115 = vtrace */
#else
	"#115 (obsolete vtrace)",		/* 115 = obsolete vtrace */
#endif
	"gettimeofday",			/* 116 = gettimeofday */
	"getrusage",			/* 117 = getrusage */
	"getsockopt",			/* 118 = getsockopt */
	"#119 (obsolete resuba)",		/* 119 = obsolete resuba */
	"readv",			/* 120 = readv */
	"writev",			/* 121 = writev */
	"settimeofday",			/* 122 = settimeofday */
	"fchown",			/* 123 = fchown */
	"fchmod",			/* 124 = fchmod */
	"orecvfrom",			/* 125 = orecvfrom */
	"osetreuid",			/* 126 = osetreuid */
	"osetregid",			/* 127 = osetregid */
	"rename",			/* 128 = rename */
	"otruncate",			/* 129 = otruncate */
	"oftruncate",			/* 130 = oftruncate */
	"flock",			/* 131 = flock */
	"mkfifo",			/* 132 = mkfifo */
	"sendto",			/* 133 = sendto */
	"shutdown",			/* 134 = shutdown */
	"socketpair",			/* 135 = socketpair */
	"mkdir",			/* 136 = mkdir */
	"rmdir",			/* 137 = rmdir */
	"utimes",			/* 138 = utimes */
	"#139 (obsolete 4.2 sigreturn)",		/* 139 = obsolete 4.2 sigreturn */
	"adjtime",			/* 140 = adjtime */
	"ogetpeername",			/* 141 = ogetpeername */
	"ogethostid",			/* 142 = ogethostid */
	"osethostid",			/* 143 = osethostid */
	"ogetrlimit",			/* 144 = ogetrlimit */
	"osetrlimit",			/* 145 = osetrlimit */
	"okillpg",			/* 146 = okillpg */
	"setsid",			/* 147 = setsid */
	"quotactl",			/* 148 = quotactl */
	"oquota",			/* 149 = oquota */
	"ogetsockname",			/* 150 = ogetsockname */
	"#151 (unimplemented sem_lock)",		/* 151 = unimplemented sem_lock */
	"#152 (unimplemented sem_wakeup)",		/* 152 = unimplemented sem_wakeup */
	"#153 (unimplemented asyncdaemon)",		/* 153 = unimplemented asyncdaemon */
	"#154 (unimplemented)",		/* 154 = unimplemented */
#if defined(NFSCLIENT) || defined(NFSSERVER)
	"nfssvc",			/* 155 = nfssvc */
#else
	"#155 (unimplemented)",		/* 155 = unimplemented */
#endif
	"ogetdirentries",			/* 156 = ogetdirentries */
	"ostatfs",			/* 157 = ostatfs */
	"ofstatfs",			/* 158 = ofstatfs */
	"#159 (unimplemented)",		/* 159 = unimplemented */
	"#160 (unimplemented)",		/* 160 = unimplemented */
#ifdef NFSCLIENT
	"getfh",			/* 161 = getfh */
#else
	"#161 (unimplemented getfh)",		/* 161 = unimplemented getfh */
#endif
	"#162 (unimplemented)",		/* 162 = unimplemented */
	"#163 (unimplemented)",		/* 163 = unimplemented */
	"#164 (unimplemented)",		/* 164 = unimplemented */
	"#165 (unimplemented)",		/* 165 = unimplemented */
	"#166 (unimplemented)",		/* 166 = unimplemented */
	"#167 (unimplemented)",		/* 167 = unimplemented */
	"#168 (unimplemented)",		/* 168 = unimplemented */
	"#169 (unimplemented)",		/* 169 = unimplemented */
	"#170 (unimplemented)",		/* 170 = unimplemented */
#if defined(SYSVSHM) && !defined(alpha)
	"shmsys",			/* 171 = shmsys */
#else
	"#171 (unimplemented shmsys)",		/* 171 = unimplemented shmsys */
#endif
	"#172 (unimplemented)",		/* 172 = unimplemented */
	"#173 (unimplemented)",		/* 173 = unimplemented */
	"#174 (unimplemented)",		/* 174 = unimplemented */
	"#175 (unimplemented)",		/* 175 = unimplemented */
	"#176 (unimplemented)",		/* 176 = unimplemented */
	"#177 (unimplemented sfork)",		/* 177 = unimplemented sfork */
	"#178 (unimplemented)",		/* 178 = unimplemented */
	"#179 (unimplemented getdescriptor)",		/* 179 = unimplemented getdescriptor */
	"#180 (unimplemented setdescriptor)",		/* 180 = unimplemented setdescriptor */
	"setgid",			/* 181 = setgid */
	"setegid",			/* 182 = setegid */
	"seteuid",			/* 183 = seteuid */
#ifdef LFS
	"lfs_bmapv",			/* 184 = lfs_bmapv */
	"lfs_markv",			/* 185 = lfs_markv */
	"lfs_segclean",			/* 186 = lfs_segclean */
	"lfs_segwait",			/* 187 = lfs_segwait */
#else
	"#184 (unimplemented)",		/* 184 = unimplemented */
	"#185 (unimplemented)",		/* 185 = unimplemented */
	"#186 (unimplemented)",		/* 186 = unimplemented */
	"#187 (unimplemented)",		/* 187 = unimplemented */
#endif
	"stat",			/* 188 = stat */
	"fstat",			/* 189 = fstat */
	"lstat",			/* 190 = lstat */
	"pathconf",			/* 191 = pathconf */
	"fpathconf",			/* 192 = fpathconf */
	"#193 (unimplemented)",		/* 193 = unimplemented */
	"getrlimit",			/* 194 = getrlimit */
	"setrlimit",			/* 195 = setrlimit */
	"getdirentries",			/* 196 = getdirentries */
	"mmap",			/* 197 = mmap */
	"__syscall",			/* 198 = __syscall */
	"lseek",			/* 199 = lseek */
	"truncate",			/* 200 = truncate */
	"ftruncate",			/* 201 = ftruncate */
	"__sysctl",			/* 202 = __sysctl */
	"mlock",			/* 203 = mlock */
	"munlock",			/* 204 = munlock */
	"undelete",			/* 205 = undelete */
	"#206 (unimplemented)",		/* 206 = unimplemented */
	"#207 (unimplemented)",		/* 207 = unimplemented */
	"#208 (unimplemented)",		/* 208 = unimplemented */
	"#209 (unimplemented)",		/* 209 = unimplemented */
	"#210 (unimplemented)",		/* 210 = unimplemented */
	"#211 (unimplemented)",		/* 211 = unimplemented */
	"#212 (unimplemented)",		/* 212 = unimplemented */
	"#213 (unimplemented)",		/* 213 = unimplemented */
	"#214 (unimplemented)",		/* 214 = unimplemented */
	"#215 (unimplemented)",		/* 215 = unimplemented */
	"#216 (unimplemented)",		/* 216 = unimplemented */
	"#217 (unimplemented)",		/* 217 = unimplemented */
	"#218 (unimplemented)",		/* 218 = unimplemented */
	"#219 (unimplemented)",		/* 219 = unimplemented */
#ifdef SYSVSEM
	"__semctl",			/* 220 = __semctl */
	"semget",			/* 221 = semget */
	"semop",			/* 222 = semop */
	"semconfig",			/* 223 = semconfig */
#else
	"#220 (unimplemented semctl)",		/* 220 = unimplemented semctl */
	"#221 (unimplemented semget)",		/* 221 = unimplemented semget */
	"#222 (unimplemented semop)",		/* 222 = unimplemented semop */
	"#223 (unimplemented semconfig)",		/* 223 = unimplemented semconfig */
#endif
#ifdef SYSVMSG
	"msgctl",			/* 224 = msgctl */
	"msgget",			/* 225 = msgget */
	"msgsnd",			/* 226 = msgsnd */
	"msgrcv",			/* 227 = msgrcv */
#else
	"#224 (unimplemented msgctl)",		/* 224 = unimplemented msgctl */
	"#225 (unimplemented msgget)",		/* 225 = unimplemented msgget */
	"#226 (unimplemented msgsnd)",		/* 226 = unimplemented msgsnd */
	"#227 (unimplemented msgrcv)",		/* 227 = unimplemented msgrcv */
#endif
#ifdef SYSVSHM
	"shmat",			/* 228 = shmat */
	"shmctl",			/* 229 = shmctl */
	"shmdt",			/* 230 = shmdt */
	"shmget",			/* 231 = shmget */
#else
	"#228 (unimplemented shmat)",		/* 228 = unimplemented shmat */
	"#229 (unimplemented shmctl)",		/* 229 = unimplemented shmctl */
	"#230 (unimplemented shmdt)",		/* 230 = unimplemented shmdt */
	"#231 (unimplemented shmget)",		/* 231 = unimplemented shmget */
#endif
};
