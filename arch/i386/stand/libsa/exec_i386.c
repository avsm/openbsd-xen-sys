/*	$OpenBSD: exec_i386.c,v 1.16 1997/09/17 17:56:10 mickey Exp $	*/

/*
 * Copyright (c) 1997 Michael Shalayeff
 * Copyright (c) 1997 Tobias Weingartner
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
 *	This product includes software developed by Michael Shalayeff.
 *	This product includes software developed by Tobias Weingartner.
 * 4. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHORS ``AS IS'' AND ANY EXPRESS OR 
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 */

#include <sys/param.h>
#include <sys/exec.h>
#include <sys/reboot.h>
#include <dev/cons.h>
#include <machine/biosvar.h>
#include "libsa.h"

#define round_to_size(x) (((int)(x) + sizeof(int) - 1) & ~(sizeof(int) - 1))

typedef void (*startfuncp)(int, int, int, int, int, int, int, int)
    __attribute__ ((noreturn));

void
machdep_start(startaddr, howto, loadaddr, ssym, esym)
	char *startaddr, *loadaddr, *ssym, *esym;
	int howto;
{
#ifdef EXEC_DEBUG
	struct exec *x;

	x = (void *)loadaddr;
	printf("exec {\n\ta_midmag = %lx\n\ta_text = %lx\n\ta_data = %lx\n"
	       "\ta_bss = %lx\n\ta_syms = %lx\n\ta_entry = %lx\n"
	       "\ta_trsize = %lx\n\ta_drsize = %lx\n}\n",
	       x->a_midmag, x->a_text, x->a_data, x->a_bss, x->a_syms,
	       x->a_entry, x->a_trsize, x->a_drsize);

	printf("/bsd(%x,%x,%x,%x,%x,%x,%x)\n",
	       howto, bootdev, BOOT_APIVER, round_to_size(esym),
	       extmem, cnvmem, (int)&BIOS_vars, sizeof(BIOS_vars));
	getchar();
#endif
	(int)startaddr &= 0xffffff;

	printf("entry point at 0x%x\n", (int)startaddr);
	/* stack and the gung is ok at this point, so, no need for asm setup */
	(*(startfuncp)startaddr)(howto, bootdev, BOOT_APIVER,
	    round_to_size(esym), extmem, cnvmem, (int)&BIOS_vars,
	    sizeof (BIOS_vars));
	/* not reached */
}
