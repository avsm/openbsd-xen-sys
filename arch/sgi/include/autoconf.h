/*	$OpenBSD: autoconf.h,v 1.9 2004/10/20 12:49:15 pefo Exp $ */

/*
 * Copyright (c) 2001-2003 Opsycon AB  (www.opsycon.se / www.opsycon.com)
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
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 */

/*
 * Definitions used by autoconfiguration.
 */

#ifndef _MACHINE_AUTOCONF_H_
#define _MACHINE_AUTOCONF_H_

#include <machine/bus.h>

/*
 *  Structure holding all misc config information.
 */
#define MAX_CPUS	4

struct sys_rec {
	int	system_type;
	struct cpuinfo {
		u_int16_t type;
		u_int8_t  vers_maj;
		u_int8_t  vers_min;
		u_int16_t fptype;
		u_int8_t  fpvers_maj;
		u_int8_t  fpvers_min;
		u_int32_t clock;
		u_int32_t clock_bus;
		u_int32_t tlbsize;
		u_int32_t tlbwired;
		u_int32_t cfg_reg;
		u_int32_t stat_reg;
	} cpu[MAX_CPUS];
	/* Published Cache OPS */
	void    (*_SyncCache)(void);
	void    (*_InvalidateICache)(vaddr_t, int);
	void    (*_InvalidateICachePage)(vaddr_t);
	void    (*_SyncDCachePage)(vaddr_t);
	void    (*_HitSyncDCache)(vaddr_t, int);
	void    (*_IOSyncDCache)(vaddr_t, int, int);
	void    (*_HitInvalidateDCache)(vaddr_t, int);
	/* BUS mappings */
	struct mips_bus_space local;
	struct mips_bus_space isa_io;
	struct mips_bus_space isa_mem;
	struct mips_bus_space pci_io[2];
	struct mips_bus_space pci_mem[2];
	/* Console/Serial configuration */
	int	cons_baudclk;
	struct mips_bus_space console_io;	/* for stupid map designs */
	struct mips_bus_space *cons_iot;
	bus_addr_t cons_ioaddr[8];		/* up to eight loclbus tty's */
};

extern struct sys_rec sys_config;

/*
 *  Give com.c method to find console address and speeds
 */
#define	COM_FREQ	(sys_config.cons_baudclk)
#define	CONCOM_FREQ	(sys_config.cons_baudclk)
#define	CONADDR		(sys_config.cons_ioaddr[0])


/**/
struct confargs;

typedef int (*intr_handler_t)(void *);

struct abus {
	struct	device *ab_dv;		/* back-pointer to device */
	int	ab_type;		/* bus type (see below) */
	void	*(*ab_intr_establish)	/* bus's set-handler function */
		    (void *, u_long, int, int, int (*)(void *), void *, char *);
	void	(*ab_intr_disestablish)	/* bus's unset-handler function */
		    (void *, void *);
	caddr_t	(*ab_cvtaddr)		/* convert slot/offset to address */
		    (struct confargs *);
	int	(*ab_matchname)		/* see if name matches driver */
		    (struct confargs *, char *);
};

#define	BUS_MAIN	1		/* mainbus */
#define	BUS_LOCAL	2		/* localbus */
#define	BUS_ISABR	3		/* ISA Bridge Bus */
#define	BUS_PLCHLDR	4		/* placeholder */
#define	BUS_PCIBR	5		/* PCI bridge Bus */

#define BUS_INTR_ESTABLISH(ca, a, b, c, d, e, f, h)			\
	    (*(ca)->ca_bus->ab_intr_establish)((a),(b),(c),(d),(e),(f),(h))
#define BUS_INTR_DISESTABLISH(ca)					\
	    (*(ca)->ca_bus->ab_intr_establish)(ca)
#define BUS_MATCHNAME(ca, name)						\
	    (((ca)->ca_bus->ab_matchname) ?				\
	    (*(ca)->ca_bus->ab_matchname)((ca), (name)) :		\
	    -1)

struct confargs {
	char		*ca_name;	/* Device name. */
	struct abus	*ca_bus;	/* Bus device resides on. */
	bus_space_tag_t ca_iot;
	bus_space_tag_t ca_memt;
	bus_dma_tag_t	ca_dmat;
	u_int32_t	ca_num;		/* which system */
	u_int32_t	ca_sys;		/* which system */
	int		ca_nreg;
	u_int32_t	*ca_reg;
	int		ca_nintr;
	int32_t		ca_intr;
	bus_addr_t	ca_baseaddr;
};

int	badaddr(void *, u_int64_t);
void	enaddr_aton(const char *, u_int8_t *);

#endif /* _MACHINE_AUTOCONF_H_ */
