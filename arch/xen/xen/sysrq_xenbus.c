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
 * watch "control/sysrq" and generate sysmon events.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/malloc.h>

#include <machine/xenbus.h>
#include <machine/sysrq_xenbus.h>

#define	SYSRQ_PATH	"control"
#define	SYSRQ_NAME	"sysrq"


static void
xenbus_sysrq_handler(struct xenbus_watch *watch, const char **vec,
    unsigned int len)
{

	struct xenbus_transaction *xbt;
	int error;
	char *reqstr;
	unsigned int reqstrlen;
	char letter;

again:
	xbt = xenbus_transaction_start();
	if (xbt == NULL) {
		return;
	}
	error = xenbus_read(xbt, SYSRQ_PATH, SYSRQ_NAME,
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
	error = xenbus_rm(xbt, SYSRQ_PATH, SYSRQ_NAME);
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

	/* We know, xm sysrq only sends one character. So just take the one
	 * and forget the rest.
	 */
	letter = reqstr[0];
	free(reqstr, M_DEVBUF);

	switch (letter) {
	case 'k':
		/* Kills all programs on the current virtual console.
		 */
		printf("%s: letter '%c' not implemented\n", __func__, letter);
		break;
	case 'b':
		/* Will immediately reboot the system without syncing or unmounting
		 * your disks.
		 */
		printf("%s: letter '%c' not implemented\n", __func__, letter);
		break;
	case 'c':
		/* Perform a crashdump and reboot */
		printf("%s: letter '%c' not implemented\n", __func__, letter);
		break;
	case 'o':
		/* Will shut your system off */
		printf("%s: letter '%c' not implemented\n", __func__, letter);
		break;
	case 's':
		/* Will attempt to sync all mounted filesystems. */
		printf("%s: letter '%c' not implemented\n", __func__, letter);
		break;
	case 'u':
		/* Will attempt to re-mount all filesystems read-only */
		printf("%s: letter '%c' not implemented\n", __func__, letter);
		break;
	case 'p':
		/* Will dump your current registers and flags to your console. */
		printf("%s: letter '%c' not implemented\n", __func__, letter);
		break;
	case 't':
		/* Will dump a list of current tasks and their information
		 * to your console.
		 */
		printf("%s: letter '%c' not implemented\n", __func__, letter);
		break;
	case 'e':
		/* Send a SIGTERM to all processes, except for init */
		printf("%s: letter '%c' not implemented\n", __func__, letter);
		break;
	case 'f':
		/* Send a SIGKILL to all processes, except for init */
		printf("%s: letter '%c' not implemented\n", __func__, letter);
		break;
	case 'l':
		/* Send a SIGKILL to all processes, INCLUDING init
		 * (Your system will be non-functional after this.)
		 */
		printf("%s: letter '%c' not implemented\n", __func__, letter);
		break;

	case 'D':
		Debugger();
		break;
	default:
		printf("%s: unknown letter '%c'\n", __func__, letter);
		break;
	}

}

static struct xenbus_watch xenbus_sysrq_watch = {
	.node = SYSRQ_PATH "/" SYSRQ_NAME, /* XXX */
	.xbw_callback = xenbus_sysrq_handler,
};

void
sysrq_xenbus_setup(void)
{

	if (register_xenbus_watch(&xenbus_sysrq_watch)) {
		printf("%s: unable to watch control/sysrq\n", __func__);
	}
}
