/*	$Id: shutdown_xenbus.c,v 1.1 2006/08/11 13:22:43 yamt Exp $	*/

/*-
 * Copyright (c)2006 YAMAMOTO Takashi,
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
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

/*
 * Copyright (c) 2005 Manuel Bouyer.
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
 *      This product includes software developed by Manuel Bouyer.
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
 *
 */

/*
 * watch "control/break" and generate sysmon events.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/malloc.h>

#include <machine/xenbus.h>
#include <machine/break_xenbus.h>

#define	BREAK_PATH	"control"
#define	BREAK_NAME	"break"


static void
xenbus_break_handler(struct xenbus_watch *watch, const char **vec,
    unsigned int len)
{

	struct xenbus_transaction *xbt;
	int error;
	char *reqstr;
	unsigned int reqstrlen;

again:
	xbt = xenbus_transaction_start();
	if (xbt == NULL) {
		return;
	}
	error = xenbus_read(xbt, BREAK_PATH, BREAK_NAME,
	    &reqstrlen, &reqstr);
	if (error) {
		if (error != ENOENT) {
			printf("%s: xenbus_read %d\n", __func__, error);
		}
		error = xenbus_transaction_end(xbt, 1);
		if (error != 0) {
			printf("%s: xenbus_transaction_end 1 %d\n",
			    __func__, error);
		}
		return;
	}
	KASSERT(strlen(reqstr) == reqstrlen);
	error = xenbus_rm(xbt, BREAK_PATH, BREAK_NAME);
	if (error) {
		printf("%s: xenbus_rm %d\n", __func__, error);
	}
	error = xenbus_transaction_end(xbt, 0);
	if (error == EAGAIN) {
		free(reqstr, M_DEVBUF);
		goto again;
	}
	if (error != 0) {
		printf("%s: xenbus_transaction_end 2 %d\n", __func__, error);
	}

	free(reqstr, M_DEVBUF);
	Debugger();
}

static struct xenbus_watch xenbus_break_watch = {
	.node = BREAK_PATH "/" BREAK_NAME, /* XXX */
	.xbw_callback = xenbus_break_handler,
};

void
break_xenbus_setup(void)
{

	if (register_xenbus_watch(&xenbus_break_watch)) {
		printf("%s: unable to watch control/break\n", __func__);
	}
}
