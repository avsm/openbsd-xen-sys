/*	$OpenBSD: autoconf.c,v 1.33 2006/01/11 07:22:01 miod Exp $	*/
/*
 * Copyright (c) 1998 Steve Murphree, Jr.
 * Copyright (c) 1996 Nivas Madhur
 * Copyright (c) 1994 Christian E. Hopps
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
 *      This product includes software developed by Christian E. Hopps.
 * 4. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission
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
#include <sys/param.h>
#include <sys/systm.h>
#include <sys/buf.h>
#include <sys/dkstat.h>
#include <sys/reboot.h>
#include <sys/conf.h>
#include <sys/device.h>
#include <sys/disklabel.h>
#include <sys/kernel.h>

#include <machine/asm_macro.h>
#include <machine/autoconf.h>
#include <machine/cpu.h>
#include <machine/disklabel.h>
#include <machine/vmparam.h>

#include <scsi/scsi_all.h>
#include <scsi/scsiconf.h>

#include <dev/cons.h>

/*
 * The following several variables are related to
 * the configuration process, and are used in initializing
 * the machine.
 */

void	dumpconf(void);
int	findblkmajor(struct device *);
struct device *getdisk(char *, int, int, dev_t *);
struct device *parsedisk(char *, int, int, dev_t *);
void	setroot(void);

int cold = 1;   /* 1 if still booting */

struct device *bootdv;	/* set by device drivers (if found) */

u_int bootdevtype;

#define	BT_CIEN		0x6369656e
#define	BT_DGEN		0x6467656e
#define	BT_HKEN		0x686b656e
#define	BT_INEN		0x696e656e
#define	BT_INSC		0x696e7363
#define	BT_NCSC		0x6e637363

/*
 * called at boot time, configure all devices on the system.
 */
void
cpu_configure()
{
	printf("bootpath: '%s' dev %u unit %u part %u\n",
	    bootargs, bootdev, bootunit, bootpart);

	if (config_rootfound("mainbus", "mainbus") == 0)
		panic("no mainbus found");

	/*
	 * Turn external interrupts on.
	 */
	set_psr(get_psr() & ~PSR_IND);
	spl0();

	setroot();
	dumpconf();

	cold = 0;
}

struct nam2blk {
	char *name;
	int maj;
} nam2blk[] = {
	{ "sd",	4 },
	{ "cd", 6 },
	{ "rd",	7 },
};

int
findblkmajor(dv)
	struct device *dv;
{
	char *name = dv->dv_xname;
	int i;

	for (i = 0; i < sizeof(nam2blk)/sizeof(nam2blk[0]); ++i)
		if (strncmp(name, nam2blk[i].name, strlen(nam2blk[0].name)) == 0)
			return (nam2blk[i].maj);
	return (-1);
}

struct device *
getdisk(str, len, defpart, devp)
	char *str;
	int len, defpart;
	dev_t *devp;
{
	struct device *dv;

	if ((dv = parsedisk(str, len, defpart, devp)) == NULL) {
		printf("use one of:");
		TAILQ_FOREACH(dv, &alldevs, dv_list) {
			if (dv->dv_class == DV_DISK)
				printf(" %s[a-p]", dv->dv_xname);
#ifdef NFSCLIENT
			if (dv->dv_class == DV_IFNET)
				printf(" %s", dv->dv_xname);
#endif
		}
		printf("\n");
	}
	return (dv);
}

struct device *
parsedisk(str, len, defpart, devp)
	char *str;
	int len, defpart;
	dev_t *devp;
{
	struct device *dv;
	char *cp, c;
	int majdev, unit, part;

	if (len == 0)
		return (NULL);
	cp = str + len - 1;
	c = *cp;
	if (c >= 'a' && (c - 'a') < MAXPARTITIONS) {
		part = c - 'a';
		*cp = '\0';
	} else
		part = defpart;

	TAILQ_FOREACH(dv, &alldevs, dv_list) {
		if (dv->dv_class == DV_DISK &&
		    strcmp(str, dv->dv_xname) == 0) {
			majdev = findblkmajor(dv);
			unit = dv->dv_unit;
			if (majdev < 0)
				panic("parsedisk");
			*devp = MAKEDISKDEV(majdev, unit, part);
			break;
		}
#ifdef NFSCLIENT
		if (dv->dv_class == DV_IFNET &&
		    strcmp(str, dv->dv_xname) == 0) {
			*devp = NODEV;
			break;
		}
#endif
	}

	*cp = c;
	return (dv);
}

/*
 * Attempt to find the device from which we were booted.
 * If we can do so, and not instructed not to do so,
 * change rootdev to correspond to the load device.
 *
 * XXX Actually, swap and root must be on the same type of device,
 * (ie. DV_DISK or DV_IFNET) because of how (*mountroot) is written.
 * That should be fixed.
 */
void
setroot()
{
	struct swdevt *swp;
	struct device *dv;
	int len, majdev, unit;
	dev_t nrootdev, nswapdev = NODEV;
	char buf[128];
	dev_t temp;
#if defined(NFSCLIENT)
	extern char *nfsbootdevname;
#endif
	extern u_int bootpart;

	printf("boot device: %s\n",
	    (bootdv) ? bootdv->dv_xname : "<unknown>");

	/*
	 * If 'swap generic' and we could not determine the boot device,
	 * ask the user.
	 */
	if (mountroot == NULL && bootdv == NULL)
		boothowto |= RB_ASKNAME;

	if (boothowto & RB_ASKNAME) {
		for (;;) {
			printf("root device ");
			if (bootdv != NULL)
				printf("(default %s%c)",
				    bootdv->dv_xname,
				    bootdv->dv_class == DV_DISK ? 'a' : ' ');
			printf(": ");
			len = getsn(buf, sizeof(buf));
			if (len == 0 && bootdv != NULL) {
				strlcpy(buf, bootdv->dv_xname, sizeof buf);
				len = strlen(buf);
			}
			if (len > 0 && buf[len - 1] == '*') {
				buf[--len] = '\0';
				dv = getdisk(buf, len, 1, &nrootdev);
				if (dv) {
					bootdv = dv;
					nswapdev = nrootdev;
					goto gotswap;
				}
			}
			dv = getdisk(buf, len, 0, &nrootdev);
			if (dv) {
				bootdv = dv;
				break;
			}
		}

		/*
		 * because swap must be on same device as root, for
		 * network devices this is easy.
		 */
		if (bootdv->dv_class == DV_IFNET) {
			goto gotswap;
		}
		for (;;) {
			printf("swap device ");
			if (bootdv != NULL)
				printf("(default %s%c)",
				    bootdv->dv_xname,
				    bootdv->dv_class == DV_DISK ? 'b' : ' ');
			printf(": ");
			len = getsn(buf, sizeof(buf));
			if (len == 0 && bootdv != NULL) {
				switch (bootdv->dv_class) {
				case DV_IFNET:
					nswapdev = NODEV;
					break;
				case DV_DISK:
					nswapdev = MAKEDISKDEV(major(nrootdev),
					    DISKUNIT(nrootdev), 1);
					break;
				case DV_TAPE:
				case DV_TTY:
				case DV_DULL:
				case DV_CPU:
					break;
				}
				break;
			}
			dv = getdisk(buf, len, 1, &nswapdev);
			if (dv) {
				if (dv->dv_class == DV_IFNET)
					nswapdev = NODEV;
				break;
			}
		}
gotswap:
		rootdev = nrootdev;
		dumpdev = nswapdev;
		swdevt[0].sw_dev = nswapdev;
		swdevt[1].sw_dev = NODEV;
	} else if (mountroot == NULL) {
		/*
		 * `swap generic': Use the device the ROM told us to use.
		 */
		if (bootdv == NULL)
			panic("boot device not known");

		majdev = findblkmajor(bootdv);
		if (majdev >= 0) {
			/*
			 * Root and swap are on a disk.
			 * val[2] of the boot device is the partition number.
			 * Assume swap is on partition b.
			 */
			unit = bootdv->dv_unit;
			rootdev = MAKEDISKDEV(majdev, unit, bootpart);
			nswapdev = dumpdev = MAKEDISKDEV(major(rootdev),
			    DISKUNIT(rootdev), 1);
		} else {
			/*
			 * Root and swap are on a net.
			 */
			nswapdev = dumpdev = NODEV;
		}
		swdevt[0].sw_dev = nswapdev;
		swdevt[1].sw_dev = NODEV;
	} else {
		/*
		 * `root DEV swap DEV': honour rootdev/swdevt.
		 * rootdev/swdevt/mountroot already properly set.
		 */
		return;
	}

	switch (bootdv->dv_class) {
#if defined(NFSCLIENT)
	case DV_IFNET:
		mountroot = nfs_mountroot;
		nfsbootdevname = bootdv->dv_xname;
		return;
#endif
#if defined(FFS)
	case DV_DISK:
		mountroot = dk_mountroot;
		majdev = major(rootdev);
		unit = DISKUNIT(rootdev);
		printf("root on %s%c\n", bootdv->dv_xname,
		    DISKPART(rootdev) + 'a');
		break;
#endif
	default:
		printf("can't figure root, hope your kernel is right\n");
		return;
	}

	/*
	 * Make the swap partition on the root drive the primary swap.
	 */
	temp = NODEV;
	for (swp = swdevt; swp->sw_dev != NODEV; swp++) {
		if (majdev == major(swp->sw_dev) &&
		    unit == DISKUNIT(swp->sw_dev)) {
			temp = swdevt[0].sw_dev;
			swdevt[0].sw_dev = swp->sw_dev;
			swp->sw_dev = temp;
			break;
		}
	}
	if (swp->sw_dev == NODEV)
		return;

	/*
	 * If dumpdev was the same as the old primary swap device, move
	 * it to the new primary swap device.
	 */
	if (temp == dumpdev)
		dumpdev = swdevt[0].sw_dev;
}

/*
 * Parse the commandline.
 *
 * This has two goals: first, turn the necessary options into boothowto
 * flags; second, convert the PROM boot device into the matching OpenBSD
 * driver name.
 */

static char *stws(char *);
static char *
stws(char *p)
{
	while (*p != ' ' && *p != '\0')
		p++;

	while (*p == ' ')
		p++;

	return (p);
}

void
cmdline_parse(void)
{
	char *p;

	/*
	 * Skip boot device ``foo(ctrl,dev,lun)'' and filename,
	 * i.e. eat everything until whitespace.
	 */
	p = stws(bootargs);
	while (*p != '\0') {
		if (*p++ == '-')
			switch (*p) {
			case 'a':
				boothowto |= RB_ASKNAME;
				break;
			case 'b':
				boothowto |= RB_KDB;
				break;
			case 'c':
				boothowto |= RB_CONFIG;
				break;
			case 's':
				boothowto |= RB_SINGLE;
				break;
			}
		p = stws(p);
	}

	/*
	 * Now parse the boot device. We are only interested in the
	 * device type, since the PROM has cracked the controller, unit
	 * and partition numbers for us already, and we do not care about
	 * our own filename...
	 *
	 * Actually we rely upon the fact that all device strings are
	 * exactly 4 characters long, and appears at the beginning of the
	 * commandline, so we can simply use its numerical value, as a
	 * word, to tell device types apart.
	 */
	bootdevtype = *(int *)bootargs;
}

void
device_register(struct device *dev, void *aux)
{
	if (bootdv != NULL)
		return;

	switch (bootdevtype) {
	case BT_INEN:
		/*
		 * Internal ethernet is le at syscon only, and we do not
		 * care about controller and unit numbers.
		 */
		if (strncmp("le", dev->dv_xname, 2) == 0 &&
		    strncmp("syscon", dev->dv_parent->dv_xname, 6) == 0)
			bootdv = dev;
		break;
	}
}
