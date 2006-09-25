/*	$OpenBSD: dkcsum.c,v 1.3 2005/11/28 18:47:03 hshoexer Exp $	*/

/*-
 * Copyright (c) 1997 Niklas Hallqvist.  All rights reserved.
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
 */

/*
 * A checksumming pseudo device used to get unique labels of each disk
 * that needs to be matched to BIOS disks.
 */

#include <sys/param.h>
#include <sys/buf.h>
#include <sys/conf.h>
#include <sys/device.h>
#include <sys/disklabel.h>
#include <sys/fcntl.h>
#include <sys/proc.h>
#include <sys/reboot.h>
#include <sys/stat.h>
#include <sys/systm.h>

#include <machine/biosvar.h>
#include <machine/xen.h>

dev_t dev_rawpart(struct device *);	/* XXX */
extern dev_t bootdev;

void
dkcsumattach(void)
{
	struct device *dv;
	union xen_cmdline_parseinfo xcp;

	xen_parse_cmdline(XEN_PARSE_BOOTDEV, &xcp);

	TAILQ_FOREACH(dv, &alldevs, dv_list) {
		if (dv->dv_class != DV_DISK)
			continue;

		if (xcp.xcp_bootdev[0] == 0) {
			bootdev = dev_rawpart(dv);
			break;
		}

		if (strncmp(xcp.xcp_bootdev, dv->dv_xname,
		    strlen(dv->dv_xname)))
			continue;
		bootdev = dev_rawpart(dv);
	}
	bootdev = MAKEBOOTDEV(major(bootdev), 0, 0, DISKUNIT(bootdev), 0);
}
