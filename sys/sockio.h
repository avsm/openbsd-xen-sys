/*	$OpenBSD: sockio.h,v 1.16 2000/04/26 18:37:37 chris Exp $	*/
/*	$NetBSD: sockio.h,v 1.5 1995/08/23 00:40:47 thorpej Exp $	*/

/*-
 * Copyright (c) 1982, 1986, 1990, 1993, 1994
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
 * 3. All advertising materials mentioning features or use of this software
 *    must display the following acknowledgement:
 *	This product includes software developed by the University of
 *	California, Berkeley and its contributors.
 * 4. Neither the name of the University nor the names of its contributors
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
 *	@(#)sockio.h	8.1 (Berkeley) 3/28/94
 */

#ifndef	_SYS_SOCKIO_H_
#define	_SYS_SOCKIO_H_

#include <sys/ioccom.h>

/* Socket ioctl's. */
#define	SIOCSHIWAT	 _IOW('s',  0, int)		/* set high watermark */
#define	SIOCGHIWAT	 _IOR('s',  1, int)		/* get high watermark */
#define	SIOCSLOWAT	 _IOW('s',  2, int)		/* set low watermark */
#define	SIOCGLOWAT	 _IOR('s',  3, int)		/* get low watermark */
#define	SIOCATMARK	 _IOR('s',  7, int)		/* at oob mark? */
#define	SIOCSPGRP	 _IOW('s',  8, int)		/* set process group */
#define	SIOCGPGRP	 _IOR('s',  9, int)		/* get process group */

#define	SIOCADDRT	 _IOW('r', 10, struct ortentry)	/* add route */
#define	SIOCDELRT	 _IOW('r', 11, struct ortentry)	/* delete route */

#define	SIOCSIFADDR	 _IOW('i', 12, struct ifreq)	/* set ifnet address */
#define	OSIOCGIFADDR	_IOWR('i', 13, struct ifreq)	/* get ifnet address */
#define	SIOCGIFADDR	_IOWR('i', 33, struct ifreq)	/* get ifnet address */
#define	SIOCSIFDSTADDR	 _IOW('i', 14, struct ifreq)	/* set p-p address */
#define	OSIOCGIFDSTADDR	_IOWR('i', 15, struct ifreq)	/* get p-p address */
#define	SIOCGIFDSTADDR	_IOWR('i', 34, struct ifreq)	/* get p-p address */
#define	SIOCSIFFLAGS	 _IOW('i', 16, struct ifreq)	/* set ifnet flags */
#define	SIOCGIFFLAGS	_IOWR('i', 17, struct ifreq)	/* get ifnet flags */
#define	OSIOCGIFBRDADDR	_IOWR('i', 18, struct ifreq)	/* get broadcast addr */
#define	SIOCGIFBRDADDR	_IOWR('i', 35, struct ifreq)	/* get broadcast addr */
#define	SIOCSIFBRDADDR	 _IOW('i', 19, struct ifreq)	/* set broadcast addr */
#define	OSIOCGIFCONF	_IOWR('i', 20, struct ifconf)	/* get ifnet list */
#define	SIOCGIFCONF	_IOWR('i', 36, struct ifconf)	/* get ifnet list */
#define	OSIOCGIFNETMASK	_IOWR('i', 21, struct ifreq)	/* get net addr mask */
#define	SIOCGIFNETMASK	_IOWR('i', 37, struct ifreq)	/* get net addr mask */
#define	SIOCSIFNETMASK	 _IOW('i', 22, struct ifreq)	/* set net addr mask */
#define	SIOCGIFMETRIC	_IOWR('i', 23, struct ifreq)	/* get IF metric */
#define	SIOCSIFMETRIC	 _IOW('i', 24, struct ifreq)	/* set IF metric */
#define	SIOCDIFADDR	 _IOW('i', 25, struct ifreq)	/* delete IF addr */
#define	SIOCAIFADDR	 _IOW('i', 26, struct ifaliasreq)/* add/chg IF alias */
#define	SIOCGIFDATA	_IOWR('i', 27, struct ifreq)	/* get if_data */

/* KAME IPv6 */
/* SIOCAIFALIAS? */
#define SIOCALIFADDR	 _IOW('i', 28, struct if_laddrreq) /* add IF addr */
#define SIOCGLIFADDR	_IOWR('i', 29, struct if_laddrreq) /* get IF addr */
#define SIOCDLIFADDR	 _IOW('i', 30, struct if_laddrreq) /* delete IF addr */

#define	SIOCADDMULTI	 _IOW('i', 49, struct ifreq)	/* add m'cast addr */
#define	SIOCDELMULTI	 _IOW('i', 50, struct ifreq)	/* del m'cast addr */
#define	SIOCGETVIFCNT	_IOWR('u', 51, struct sioc_vif_req)/* vif pkt cnt */
#define	SIOCGETSGCNT	_IOWR('u', 52, struct sioc_sg_req) /* sg pkt cnt */

#define	SIOCSIFMEDIA	_IOWR('i', 53, struct ifreq)	/* set net media */
#define	SIOCGIFMEDIA	_IOWR('i', 54, struct ifmediareq) /* get net media */

#define	SIOCSIFGENERIC	 _IOW('i', 57, struct ifreq)	/* generic IF set op */
#define	SIOCGIFGENERIC	_IOWR('i', 58, struct ifreq)	/* generic IF get op */

#define SIOCSIFPHYADDR   _IOW('i', 70, struct ifaliasreq) /* set gif addres */
#define	SIOCGIFPSRCADDR	_IOWR('i', 71, struct ifreq)	/* get gif psrc addr */
#define	SIOCGIFPDSTADDR	_IOWR('i', 72, struct ifreq)	/* get gif pdst addr */

#define	SIOCBRDGADD	 _IOW('i', 60, struct ifbreq)	/* add bridge ifs */
#define	SIOCBRDGDEL	 _IOW('i', 61, struct ifbreq)	/* del bridge ifs */
#define	SIOCBRDGGIFFLGS	_IOWR('i', 62, struct ifbreq)	/* get brdg if flags */
#define	SIOCBRDGSIFFLGS	 _IOW('i', 63, struct ifbreq)	/* set brdg if flags */
#define	SIOCBRDGSCACHE	 _IOW('i', 64, struct ifbrparam)/* set cache size */
#define	SIOCBRDGGCACHE	_IOWR('i', 65, struct ifbrparam)/* get cache size */
#define	SIOCBRDGIFS	_IOWR('i', 66, struct ifbreq)	/* get member ifs */
#define	SIOCBRDGRTS	_IOWR('i', 67, struct ifbaconf)	/* get addresses */
#define	SIOCBRDGSADDR	_IOWR('i', 68, struct ifbareq)	/* set addr flags */
#define	SIOCBRDGSTO	 _IOW('i', 69, struct ifbrparam)/* cache timeout */
#define	SIOCBRDGGTO	_IOWR('i', 70, struct ifbrparam)/* cache timeout */
#define	SIOCBRDGDADDR	 _IOW('i', 71, struct ifbareq)	/* delete addr */
#define	SIOCBRDGFLUSH	 _IOW('i', 72, struct ifbreq)	/* flush addr cache */

#define SIOCGENCSA	_IOWR('i', 73, struct ifsa)	/* get enc sa */
#define SIOCSENCDSTSA	 _IOW('i', 74, struct ifsa)	/* set enc sa */
#define SIOCSENCSRCSA	 _IOW('i', 75, struct ifsa)	/* set enc sa */
#define SIOCSENCCLEARSA	 _IOW('i', 76, struct ifsa)	/* set enc sa */

#define SIOCBRDGARL	 _IOW('i', 77, struct ifbrlreq)	/* add bridge rule */
#define SIOCBRDGFRL	 _IOW('i', 78, struct ifbrlreq)	/* flush brdg rules */
#define SIOCBRDGGRL	_IOWR('i', 79, struct ifbrlconf)/* get bridge rules */
#define	SIOCBRDGGPRI	_IOWR('i', 80, struct ifbrparam)/* get priority */
#define	SIOCBRDGSPRI	 _IOW('i', 80, struct ifbrparam)/* set priority */
#define	SIOCBRDGGHT	_IOWR('i', 81, struct ifbrparam)/* get hello time */
#define	SIOCBRDGSHT	 _IOW('i', 81, struct ifbrparam)/* set hello time */
#define	SIOCBRDGGFD	_IOWR('i', 82, struct ifbrparam)/* get forward delay */
#define	SIOCBRDGSFD	 _IOW('i', 82, struct ifbrparam)/* set forward delay */
#define	SIOCBRDGGMA	_IOWR('i', 83, struct ifbrparam)/* get max age */
#define	SIOCBRDGSMA	 _IOW('i', 83, struct ifbrparam)/* set max age */
#define	SIOCBRDGSIFPRIO	 _IOW('i', 84, struct ifbreq)	/* set if priority */

#define	SIOCBRDGS
#define GRESADDRS        _IOW('i', 101, struct ifreq)
#define GRESADDRD        _IOW('i', 102, struct ifreq)   
#define GREGADDRS       _IOWR('i', 103, struct ifreq)
#define GREGADDRD       _IOWR('i', 104, struct ifreq)
#define GRESPROTO        _IOW('i', 105, struct ifreq)
#define GREGPROTO       _IOWR('i', 106, struct ifreq)

#define	SIOCSIFMTU	 _IOW('i', 127, struct ifreq)	/* set ifnet mtu */
#define	SIOCGIFMTU	_IOWR('i', 126, struct ifreq)	/* get ifnet mtu */
#define	SIOCSIFASYNCMAP  _IOW('i', 125, struct ifreq)	/* set ppp asyncmap */
#define	SIOCGIFASYNCMAP _IOWR('i', 124, struct ifreq)	/* get ppp asyncmap */
#endif /* !_SYS_SOCKIO_H_ */
