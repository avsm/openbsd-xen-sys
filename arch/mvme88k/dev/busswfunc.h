/*	$OpenBSD: pcctwofunc.h,v 1.1 2001/03/08 00:03:14 miod Exp $ */

#ifndef _MVME88K_BUSSWF_H_
#define _MVME88K_BUSSWF_H_

int busswintr_establish __P((int vec, struct intrhand *ih));

#endif	/* _MVME88K_PCCTWO_H_ */

