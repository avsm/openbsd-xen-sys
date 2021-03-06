/*	$OpenBSD: sys_machdep.c,v 1.18 2005/08/01 11:54:25 miod Exp $ */

/*
 * Copyright (c) 1982, 1986, 1993
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
 * THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND
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
 *
 *	@(#)sys_machdep.c	8.2 (Berkeley) 1/13/94
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/ioctl.h>
#include <sys/file.h>
#include <sys/time.h>
#include <sys/proc.h>
#include <sys/signalvar.h>
#include <sys/uio.h>
#include <sys/kernel.h>
#include <sys/mtio.h>
#include <sys/buf.h>
#include <sys/mount.h>

#include <sys/syscallargs.h>

#include <uvm/uvm_extern.h>

#include <machine/cpu.h>

/*
 * DMA cache control
 */
/*ARGSUSED1*/
void
dma_cachectl(addr, len)
	caddr_t  addr;
	int len;
{
#if defined(M68040) || defined(M68060)
	if (mmutype <= MMU_68040) {
		register int inc = 0;
		int pa = 0;
		caddr_t end;

		end = addr + len;
		if (len <= 1024 || mmutype == MMU_68060) { /* always line push line for 060 */
			addr = (caddr_t)((int)addr & ~0xF);
			inc = 16;
		} else {
			addr = (caddr_t)((int)addr & ~PGOFSET);
			inc = NBPG;
		}
		do {
			/*
			 * Convert to physical address.
			 */
			if (pa == 0 || ((int)addr & PGOFSET) == 0) {
				pa = kvtop((vaddr_t)addr);
			}
			if (inc == 16) {
				DCFL(pa);
				ICPL(pa);
			} else {
				DCFP(pa);
				ICPP(pa);
			}
			pa += inc;
			addr += inc;
		} while (addr < end);
	}
#endif	/* M68040 */
}

int
sys_sysarch(p, v, retval)
	struct proc *p;
	void *v;
	register_t *retval;
{
#if 0
	struct sys_sysarch_args /* {
		syscallarg(int) op;
		syscallarg(char *) parms;
	} */ *uap = v;
#endif

	return ENOSYS;
}
