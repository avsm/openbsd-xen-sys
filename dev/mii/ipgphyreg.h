/*	$OpenBSD$	*/

/*-
 * Copyright (c) 2006, Pyun YongHyeon
 * All rights reserved.
 *              
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:             
 * 1. Redistributions of source code must retain the above copyright
 *    notice unmodified, this list of conditions, and the following
 *    disclaimer.  
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
 *
 */

#ifndef _DEV_MII_IPGPHYREG_H_
#define _DEV_MII_IPGPHYREG_H_

/*
 * Registers for the IC Plus IPGA internal PHY.
 */

/* Control register */
#define IPGPHY_MII_BMCR		0x00
#define IPGPHY_BMCR_FDX		0x0100
#define IPGPHY_BMCR_STARTNEG		0x0200
#define IPGPHY_BMCR_ISO		0x0400
#define IPGPHY_BMCR_PDOWN		0x0800
#define IPGPHY_BMCR_AUTOEN		0x1000
#define IPGPHY_BMCR_LOOP		0x4000
#define IPGPHY_BMCR_RESET		0x8000

#define IPGPHY_BMCR_10		0x0000
#define IPGPHY_BMCR_100		0x2000
#define IPGPHY_BMCR_1000		0x0040

/* Status register */
#define IPGPHY_MII_BMSR		0x01
#define IPGPHY_BMSR_EXT		0x0001
#define IPGPHY_BMSR_LINK		0x0004
#define IPGPHY_BMSR_ANEG		0x0008
#define IPGPHY_BMSR_RFAULT		0x0010
#define IPGPHY_BMSR_ANEGCOMP		0x0020
#define IPGPHY_BMSR_EXTSTS		0x0100

#define IPGPHY_MII_ID1		0x02

/* Autonegotiation advertisement register */
#define IPGPHY_MII_ANAR		0x04
#define IPGPHY_ANAR_10T		0x0020
#define IPGPHY_ANAR_10T_FDX		0x0040
#define IPGPHY_ANAR_100TX		0x0080
#define IPGPHY_ANAR_100TX_FDX	0x0100
#define IPGPHY_ANAR_100T4		0x0200
#define IPGPHY_ANAR_PAUSE		0x0400
#define IPGPHY_ANAR_APAUSE		0x0800
#define IPGPHY_ANAR_RFAULT		0x2000
#define IPGPHY_ANAR_NP		0x8000

/* Autonegotiation link parnet ability register */
#define IPGPHY_MII_ANLPAR		0x05
#define IPGPHY_ANLPAR_10T		0x0020
#define IPGPHY_ANLPAR_10T_FDX	0x0040
#define IPGPHY_ANLPAR_100TX		0x0080
#define IPGPHY_ANLPAR_100TX_FDX	0x0100
#define IPGPHY_ANLPAR_100T4		0x0200
#define IPGPHY_ANLPAR_PAUSE		0x0400
#define IPGPHY_ANLPAR_APAUSE		0x0800
#define IPGPHY_ANLPAR_RFAULT		0x2000
#define IPGPHY_ANLPAR_ACK		0x4000
#define IPGPHY_ANLPAR_NP		0x8000

/* Autonegotiation expansion register */
#define IPGPHY_MII_ANER		0x06
#define IPGPHY_ANER_LPNWAY		0x0001
#define IPGPHY_ANER_PRCVD		0x0002
#define IPGPHY_ANER_NEXTP		0x0004
#define IPGPHY_ANER_LPNEXTP		0x0008
#define IPGPHY_ANER_PDF		0x0100

/* Autonegotiation next page transmit register */
#define IPGPHY_MII_NEXTP		0x07
#define IPGPHY_NEXTP_MSGC		0x0001
#define IPGPHY_NEXTP_TOGGLE		0x0800
#define IPGPHY_NEXTP_ACK2		0x1000
#define IPGPHY_NEXTP_MSGP		0x2000
#define IPGPHY_NEXTP_NEXTP		0x8000

/* Autonegotiation link partner next page register */
#define IPGPHY_MII_NEXTPLP		0x08
#define IPGPHY_NEXTPLP_MSGC		0x0001
#define IPGPHY_NEXTPLP_TOGGLE	0x0800
#define IPGPHY_NEXTPLP_ACK2		0x1000
#define IPGPHY_NEXTPLP_MSGP		0x2000
#define IPGPHY_NEXTPLP_ACK		0x4000
#define IPGPHY_NEXTPLP_NEXTP		0x8000

/* 1000baseT control register */
#define IPGPHY_MII_1000CR		0x09
#define IPGPHY_1000CR_1000T		0x0100
#define IPGPHY_1000CR_1000T_FDX	0x0200
#define IPGPHY_1000CR_MASTER		0x0400
#define IPGPHY_1000CR_MMASTER	0x0800
#define IPGPHY_1000CR_MANUAL		0x1000
#define IPGPHY_1000CR_TMNORMAL	0x0000
#define IPGPHY_1000CR_TM1		0x2000
#define IPGPHY_1000CR_TM2		0x4000
#define IPGPHY_1000CR_TM3		0x6000
#define IPGPHY_1000CR_TM4		0x8000

/* 1000baseT status register */
#define IPGPHY_MII_1000SR		0x0A
#define IPGPHY_1000SR_LP		0x0400
#define IPGPHY_1000SR_LP_FDX		0x0800
#define IPGPHY_1000SR_RXSTAT		0x1000
#define IPGPHY_1000SR_LRXSTAT	0x2000
#define IPGPHY_1000SR_MASTER		0x4000
#define IPGPHY_1000SR_MASTERF	0x8000

/* Extended status register */
#define IPGPHY_MII_EXTSTS		0x0F
#define IPGPHY_EXTSTS_1000T		0x1000
#define IPGPHY_EXTSTS_1000T_FDX	0x2000
#define IPGPHY_EXTSTS_1000X		0x4000
#define IPGPHY_EXTSTS_1000X_FDX	0x8000

#endif /* _DEV_MII_IPGPHYREG_H_ */
