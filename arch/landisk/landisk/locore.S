/*	$OpenBSD: locore.S,v 1.1.1.1 2006/10/06 21:16:15 miod Exp $	*/
/*	$NetBSD: locore.S,v 1.1 2006/09/01 21:26:18 uwe Exp $	*/

/*-
 * Copyright (c) 2002 The NetBSD Foundation, Inc.
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

#include "assym.h"

#include <sh/asm.h>
#include <sh/locore.h>
#include <sh/trap.h>
#include <sh/mmu_sh4.h>

/*
 * void start(int howto, void *bootinfo);
 *	Kernel entry point.
 */	
NENTRY(start)
ALTENTRY(kernel_text)
	/* Set bootstrap stack pointer */
	mov.l	_L.bootstrap_stack, r15

	/* Mask all interrupt */
	__INTR_MASK(r0, r1)

	/* MMU off */
	mov.l	_L.MMUCR, r0
	xor	r1, r1
	mov.l	r1, @r0
	
	/* Make sure to change register bank 0 */
	mov	#0x20, r1
	stc	sr, r0
	swap.b	r1, r1
	swap.w	r1, r1
	not	r1, r1
	and	r1, r0
	ldc	r0, sr	/* SR.RB = 0 */

	/* Go! */
	mov.l	_L.landisk_startup, r0
	jmp	@r0
	 nop

	/* NOTREACHED */	
	.align	2
_L.landisk_startup:	.long	_C_LABEL(landisk_startup)
_L.bootstrap_stack:	.long	_C_LABEL(start)
_L.MMUCR:		.long	SH4_MMUCR

ENTRY(getramsize)
	mov	#4, r0
	trapa	#0x3f
	rts
	 nop
