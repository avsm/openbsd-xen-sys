/*	$OpenBSD: conf.h,v 1.6 2005/01/22 04:04:32 uwe Exp $	*/
/*	$NetBSD: conf.h,v 1.8 2002/02/10 12:26:03 chris Exp $	*/

#ifndef _ARMISH_CONF_H
#define	_ARMISH_CONF_H

#include <sys/conf.h>

/*
 * ARMISH specific device includes go in here
 */

#define	CONF_HAVE_USB
#define	CONF_HAVE_GPIO

#include <arm/conf.h>

#endif	/* _ARMISH_CONF_H */
