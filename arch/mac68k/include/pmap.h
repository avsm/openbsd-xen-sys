/*	$OpenBSD: pmap.h,v 1.12 2001/11/28 16:13:28 art Exp $	*/

#ifndef	_MAC68K_PMAP_H_
#define	_MAC68K_PMAP_H_

#include <m68k/pmap_motorola.h>

#ifdef	_KERNEL

void mac68k_set_pte __P((vm_offset_t va, vm_offset_t pge));

void pmap_init_md __P((void));
#define	PMAP_INIT_MD()	pmap_init_md()

#endif	/* _KERNEL */

#endif	/* _MAC68K_PMAP_H_ */
