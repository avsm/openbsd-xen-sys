/*      $OpenBSD: $       */

/*
 *
 * Copyright (c) 1996 Charles D. Cranor
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
 *      This product includes software developed by Charles D. Cranor.
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
 */

/*
 * net/if_atm.h
 */

/*
 * pseudo header for packet transmission
 */

struct atm_pseudohdr {
  u_int8_t atm_ph[4];	/* flags+VPI+VCI1(msb)+VCI2(lsb) */
};

#define ATM_PH_FLAGS(X)	((X)->atm_ph[0])
#define ATM_PH_VPI(X)	((X)->atm_ph[1])
#define ATM_PH_VCI(X)	((((X)->atm_ph[2]) << 8) | ((X)->atm_ph[3]))
#define ATM_PH_SETVCI(X,V) { \
	(X)->atm_ph[2] = ((V) >> 8) & 0xff; \
	(X)->atm_ph[3] = ((V) & 0xff); \
}

#define ATM_PH_AAL5    0x01	/* use AAL5? (0 == aal0) */
#define ATM_PH_LLCSNAP 0x02	/* use the LLC SNAP encoding (iff aal5) */

#define ATM_PH_DRIVER7  0x40	/* reserve for driver's use */
#define ATM_PH_DRIVER8  0x80	/* reserve for driver's use */

#define ATMMTU		9180	/* ATM MTU size for IP */
				/* XXX: could be 9188 with LLC/SNAP according
					to comer */

/* pseudo ioctl */
#define SIOCATMENA	_IOWR('a', 122, struct atm_pseudohdr) /* enable */
#define SIOCATMDIS	_IOWR('a', 123, struct atm_pseudohdr) /* disable */

/*
 * XXX forget all the garbage in if_llc.h and do it the easy way
 */

#define ATMLLC_HDR "\252\252\3\0\0\0"
struct atmllc {
  u_int8_t llchdr[6];	/* aa.aa.03.00.00.00 */
  u_int8_t type[2];	/* "ethernet" type */
};

#define ATM_LLC_TYPE(X) (((X)->type[1] << 8) | ((X)->type[0]))
#define ATM_LLC_SETTYPE(X,V) { \
	(X)->type[1] = ((V) >> 8) & 0xff; \
	(X)->type[0] = ((V) & 0xff); \
}

#ifdef _KERNEL
void atm_ifattach __P((struct ifnet *));
int atm_output __P((struct ifnet *, struct mbuf *, struct sockaddr *, 
	struct rtentry *));
#endif
