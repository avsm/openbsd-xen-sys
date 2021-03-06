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
 * watch "control/shutdown" and generate sysmon events.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/malloc.h>

#ifdef __NetBSD__
#include <dev/sysmon/sysmonvar.h>
#endif

#include <machine/xenbus.h>
#include <machine/shutdown_xenbus.h>

#define	SHUTDOWN_PATH	"control"
#define	SHUTDOWN_NAME	"shutdown"

#ifdef __NetBSD__
static struct sysmon_pswitch xenbus_power = {
	.smpsw_type = PSWITCH_TYPE_POWER,
	.smpsw_name = "xenbus",
};
static struct sysmon_pswitch xenbus_reset = {
	.smpsw_type = PSWITCH_TYPE_RESET,
	.smpsw_name = "xenbus",
};
#endif

static void
xenbus_shutdown_handler(struct xenbus_watch *watch, const char **vec,
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
	error = xenbus_read(xbt, SHUTDOWN_PATH, SHUTDOWN_NAME,
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
	error = xenbus_rm(xbt, SHUTDOWN_PATH, SHUTDOWN_NAME);
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
#ifdef __NetBSD__
	if (strcmp(reqstr, "poweroff") == 0) {
		sysmon_pswitch_event(&xenbus_power, PSWITCH_EVENT_PRESSED);
	} else if (strcmp(reqstr, "reboot") == 0) {
		sysmon_pswitch_event(&xenbus_reset, PSWITCH_EVENT_PRESSED);
	} else {
		/* XXX halt, suspend */
		printf("ignore shutdown request: %s\n", reqstr);
	}
#endif
	free(reqstr, M_DEVBUF);
}

static struct xenbus_watch xenbus_shutdown_watch = {
	.node = SHUTDOWN_PATH "/" SHUTDOWN_NAME, /* XXX */
	.xbw_callback = xenbus_shutdown_handler,
};

void
shutdown_xenbus_setup(void)
{

#ifdef __NetBSD__
	if (sysmon_pswitch_register(&xenbus_power) != 0 ||
	    sysmon_pswitch_register(&xenbus_reset) != 0) {
		printf("%s: unable to register with sysmon\n", __func__);
		return;
	}
#endif
	if (register_xenbus_watch(&xenbus_shutdown_watch)) {
		printf("%s: unable to watch control/shutdown\n", __func__);
	}
}
