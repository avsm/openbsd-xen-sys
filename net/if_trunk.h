/*	$OpenBSD$	*/

/*
 * Copyright (c) 2005 Reyk Floeter <reyk@vantronix.net>
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#ifndef _NET_TRUNK_H
#define _NET_TRUNK_H

/*
 * Global definitions
 */

#define TRUNK_MAX_PORTS		32	/* logically */
#define TRUNK_MAX_NAMESIZE	32	/* name of a protocol */

/* Port flags */
#define TRUNK_PORT_SLAVE	0x00000000	/* normal enslaved port */
#define TRUNK_PORT_MASTER	0x00000001	/* primary port */
#define TRUNK_PORT_GLOBAL	0x80000000	/* IOCTL: global flag */
#define TRUNK_PORT_BITS		"\20\01MASTER"

/* Supported trunk PROTOs */
enum trunk_proto {
	TRUNK_PROTO_NONE	= 0,		/* no trunk protocol defined */
	TRUNK_PROTO_ROUNDROBIN	= 1,		/* simple round robin */
	TRUNK_PROTO_MAX		= 3,
};

struct trunk_protos {
	const char		*tpr_name;
	enum trunk_proto	tpr_proto;
};

#define	TRUNK_PROTO_DEFAULT	TRUNK_PROTO_ROUNDROBIN
#define TRUNK_PROTOS	{						\
	{ "roundrobin",	TRUNK_PROTO_ROUNDROBIN },			\
	{ "none",	TRUNK_PROTO_NONE },				\
	{ "default",	TRUNK_PROTO_DEFAULT }				\
}

/*
 * Trunk ioctls.
 */

/* Trunk port settings */
struct trunk_reqport {
	char			rp_ifname[IFNAMSIZ];	/* name of the trunk */
	char			rp_portname[IFNAMSIZ];	/* name of the port */
	u_int32_t		rp_flags;		/* port flags */
};

#define SIOCGTRUNKPORT		_IOWR('i', 140, struct trunk_reqport)
#define SIOCSTRUNKPORT		 _IOW('i', 141, struct trunk_reqport)
#define SIOCSTRUNKDELPORT	 _IOW('i', 142, struct trunk_reqport)

/* Trunk, ports and options */
struct trunk_reqall {
	char			ra_ifname[IFNAMSIZ];	/* name of the trunk */
	u_int			ra_proto;		/* trunk protocol */

	size_t			ra_size;		/* size of buffer */
	struct trunk_reqport	*ra_port;		/* allocated buffer */
	int			ra_ports;		/* total port count */
};

#define SIOCGTRUNK		_IOWR('i', 143, struct trunk_reqall)
#define SIOCSTRUNK		 _IOW('i', 144, struct trunk_reqall)

#ifdef _KERNEL
/*
 * Internal kernel part
 */

struct trunk_port {
	struct ifnet			*tp_if;		/* physical interface */
	caddr_t				tp_trunk;	/* parent trunk */

	u_char				tp_iftype;	/* interface type */
	u_int32_t			tp_flags;	/* port flags */

	/* Redirected callbacks */
	void	(*tp_watchdog)(struct ifnet *);
	int	(*tp_ioctl)(struct ifnet *, u_long, caddr_t);

	SLIST_ENTRY(trunk_port)		tp_entries;
};

#define tp_ifname		tp_if->if_xname		/* interface name */
#define tp_link_state		tp_if->if_link_state	/* link state */

struct trunk_softc {
	struct arpcom			tr_ac;		/* virtual interface */
	int				tr_unit;	/* trunk unit */
	enum trunk_proto		tr_proto;	/* trunk protocol */
	u_int				tr_count;	/* number of ports */
	struct trunk_port		*tr_primary;	/* primary port */
	struct ifmedia			tr_media;	/* media config */
	caddr_t				tr_psc;		/* protocol data */

	SLIST_HEAD(__tplhd, trunk_port)	tr_ports;	/* list of interfaces */
	SLIST_ENTRY(trunk_softc)	tr_entries;

	/* Trunk protocol callbacks */
	int	(*tr_detach)(struct trunk_softc *);
	int	(*tr_start)(struct trunk_softc *, struct mbuf *);
	int	(*tr_watchdog)(struct trunk_softc *);
	int	(*tr_input)(struct trunk_softc *, struct trunk_port *,
		    struct ether_header *, struct mbuf *);
};

#define tr_ifflags		tr_ac.ac_if.if_flags	/* interface flags */
#define tr_ifname		tr_ac.ac_if.if_xname	/* interface name */

void	 trunk_port_ifdetach(struct ifnet *);
int	 trunk_input(struct ifnet *, struct ether_header *, struct mbuf *);
#endif /* _KERNEL */

#endif /* _NET_TRUNK_H */
