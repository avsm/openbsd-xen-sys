/*	$OpenBSD: prf.c,v 1.1 1997/07/14 08:14:27 downsj Exp $	*/
/*	$NetBSD: prf.c,v 1.5 1994/10/26 07:27:50 cgd Exp $	*/

/*
 * Copyright (c) 1982, 1986, 1990, 1993
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
 *	@(#)prf.c	8.1 (Berkeley) 6/10/93
 */

getchar()
{
	register int c;

	while((c = cngetc()) == 0)
		;
	if (c == '\r')
		c = '\n';
	else if (c == ('c'&037)) {
		panic("^C");
		/* NOTREACHED */
	}
	return(c);
}

tgetchar()
{
	register int c;

	if ((c = cngetc()) == 0)
        	return(0);
        
	if (c == '\r')
		c = '\n';
	else if (c == ('c'&037)) {
		panic("^C");
		/* NOTREACHED */
	}
	return(c);
}

putchar(c)
	register int c;
{
	cnputc(c);
	if (c == '\n')
		cnputc('\r');
}
