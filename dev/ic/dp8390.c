/*	$OpenBSD: dp8390.c,v 1.5 1999/08/13 21:10:14 deraadt Exp $	*/
/*	$NetBSD: dp8390.c,v 1.13 1998/07/05 06:49:11 jonathan Exp $	*/

/*
 * Device driver for National Semiconductor DS8390/WD83C690 based ethernet
 * adapters.
 *
 * Copyright (c) 1994, 1995 Charles M. Hannum.  All rights reserved.
 *
 * Copyright (C) 1993, David Greenman.  This software may be used, modified,
 * copied, distributed, and sold, in both source and binary form provided that
 * the above copyright and these terms are retained.  Under no circumstances is
 * the author responsible for the proper functioning of this software, nor does
 * the author assume any responsibility for damages incurred with its use.
 */

#include "bpfilter.h"

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/device.h>
#include <sys/errno.h>
#include <sys/ioctl.h>
#include <sys/mbuf.h>
#include <sys/socket.h>
#include <sys/syslog.h>

#include <net/if.h>
#include <net/if_dl.h>
#include <net/if_types.h>
#include <net/if_media.h>
#ifdef __NetBSD__
#include <net/if_ether.h>
#endif

#ifdef INET
#include <netinet/in.h>
#include <netinet/in_systm.h>
#include <netinet/in_var.h>
#include <netinet/ip.h>
#ifdef __NetBSD__
#include <netinet/if_inarp.h>
#else
#include <netinet/if_ether.h>
#endif
#endif

#ifdef NS
#include <netns/ns.h>
#include <netns/ns_if.h>
#endif

#if NBPFILTER > 0
#include <net/bpf.h>
#include <net/bpfdesc.h>
#endif

#include <machine/bus.h>

#include <dev/ic/dp8390reg.h>
#include <dev/ic/dp8390var.h>

#ifdef DEBUG
#define __inline__	/* XXX for debugging porpoises */
#endif

static __inline__ void	dp8390_xmit __P((struct dp8390_softc *));

static __inline__ void	dp8390_read_hdr __P((struct dp8390_softc *,
			    int, struct dp8390_ring *));
static __inline__ int	dp8390_ring_copy __P((struct dp8390_softc *,
			    int, caddr_t, u_short));
static __inline__ int	dp8390_write_mbuf __P((struct dp8390_softc *,
			    struct mbuf *, int));

static int		dp8390_test_mem __P((struct dp8390_softc *));

int	dp8390_enable __P((struct dp8390_softc *));
void	dp8390_disable __P((struct dp8390_softc *));

int	dp8390_mediachange __P((struct ifnet *));
void	dp8390_mediastatus __P((struct ifnet *, struct ifmediareq *));

#define	ETHER_MIN_LEN	64
#define ETHER_MAX_LEN	1518
#define	ETHER_ADDR_LEN	6

int	dp8390_debug = 0;

/*
 * Do bus-independent setup.
 */
int
dp8390_config(sc, media, nmedia, defmedia)
	struct dp8390_softc *sc;
	int *media, nmedia, defmedia;
{
#ifdef __NetBSD__
	struct ifnet *ifp = &sc->sc_ec.ec_if;
#else
	struct ifnet *ifp = &sc->sc_arpcom.ac_if;
#endif
	int i, rv;

	rv = 1;

	if (!sc->test_mem)
		sc->test_mem = dp8390_test_mem;

	/* Allocate one xmit buffer if < 16k, two buffers otherwise. */
	if ((sc->mem_size < 16384) ||
	    (sc->sc_flags & DP8390_NO_MULTI_BUFFERING))
		sc->txb_cnt = 1;
	else if (sc->mem_size < 8192 * 3)
		sc->txb_cnt = 2;
	else
		sc->txb_cnt = 3;

	sc->tx_page_start = sc->mem_start >> ED_PAGE_SHIFT;
	sc->rec_page_start = sc->tx_page_start + sc->txb_cnt * ED_TXBUF_SIZE;
	sc->rec_page_stop = sc->tx_page_start + (sc->mem_size >> ED_PAGE_SHIFT);
	sc->mem_ring = sc->mem_start + (sc->rec_page_start << ED_PAGE_SHIFT);
	sc->mem_end = sc->mem_start + sc->mem_size;

	/* Now zero memory and verify that it is clear. */
	if ((*sc->test_mem)(sc))
		goto out;

	/* Set interface to stopped condition (reset). */
	dp8390_stop(sc);

	/* Initialize ifnet structure. */
	bcopy(sc->sc_dev.dv_xname, ifp->if_xname, IFNAMSIZ);
	ifp->if_softc = sc;
	ifp->if_start = dp8390_start;
	ifp->if_ioctl = dp8390_ioctl;
	if (!ifp->if_watchdog)
		ifp->if_watchdog = dp8390_watchdog;
	ifp->if_flags =
	    IFF_BROADCAST | IFF_SIMPLEX | IFF_NOTRAILERS | IFF_MULTICAST;
	ifp->if_snd.ifq_maxlen = IFQ_MAXLEN;

	/* Initialize media goo. */
	ifmedia_init(&sc->sc_media, 0, dp8390_mediachange, dp8390_mediastatus);
	if (media != NULL) {
		for (i = 0; i < nmedia; i++)
			ifmedia_add(&sc->sc_media, media[i], 0, NULL);
		ifmedia_set(&sc->sc_media, defmedia);
	} else {
		ifmedia_add(&sc->sc_media, IFM_ETHER|IFM_MANUAL, 0, NULL);
		ifmedia_set(&sc->sc_media, IFM_ETHER|IFM_MANUAL);
	}

	/* Attach the interface. */
	if_attach(ifp);
#ifdef __NetBSD__
	ether_ifattach(ifp, sc->sc_enaddr);
#else
	ether_ifattach(ifp);
#endif
#if NBPFILTER > 0
#ifdef __NetBSD__
	bpfattach(&ifp->if_bpf, ifp, DLT_EN10MB, sizeof(struct ether_header));
#else
	bpfattach(&sc->sc_arpcom.ac_if.if_bpf, ifp, DLT_EN10MB,
		sizeof(struct ether_header));
#endif
#endif

	/* Print additional info when attached. */
	printf("%s: address %s\n", sc->sc_dev.dv_xname,
#ifdef __NetBSD__
	    ether_sprintf(sc->sc_enaddr));
#else
	    ether_sprintf(sc->sc_arpcom.ac_enaddr));
#endif

	rv = 0;
out:
	return (rv);
}

/*
 * Media change callback.
 */
int
dp8390_mediachange(ifp)
	struct ifnet *ifp;
{
	struct dp8390_softc *sc = ifp->if_softc;

	if (sc->sc_mediachange)
		return ((*sc->sc_mediachange)(sc));
	return (EINVAL);
}

/*
 * Media status callback.
 */
void
dp8390_mediastatus(ifp, ifmr)
	struct ifnet *ifp;
	struct ifmediareq *ifmr;
{
	struct dp8390_softc *sc = ifp->if_softc;

	if (sc->sc_enabled == 0) {
		ifmr->ifm_active = IFM_ETHER | IFM_NONE;
		ifmr->ifm_status = 0;
		return;
	}

	if (sc->sc_mediastatus)
		(*sc->sc_mediastatus)(sc, ifmr);
}

/*
 * Reset interface.
 */
void
dp8390_reset(sc)
	struct dp8390_softc *sc;
{
	int     s;

	s = splnet();
	dp8390_stop(sc);
	dp8390_init(sc);
	splx(s);
}

/*
 * Take interface offline.
 */
void
dp8390_stop(sc)
	struct dp8390_softc *sc;
{
	bus_space_tag_t regt = sc->sc_regt;
	bus_space_handle_t regh = sc->sc_regh;
	int n = 5000;

	/* Stop everything on the interface, and select page 0 registers. */
	NIC_PUT(regt, regh, ED_P0_CR,
	    sc->cr_proto | ED_CR_PAGE_0 | ED_CR_STP);

	/*
	 * Wait for interface to enter stopped state, but limit # of checks to
	 * 'n' (about 5ms).  It shouldn't even take 5us on modern DS8390's, but
	 * just in case it's an old one.
	 */
	while (((NIC_GET(regt, regh,
	    ED_P0_ISR) & ED_ISR_RST) == 0) && --n)
		;
}

/*
 * Device timeout/watchdog routine.  Entered if the device neglects to generate
 * an interrupt after a transmit has been started on it.
 */

void
dp8390_watchdog(ifp)
	struct ifnet *ifp;
{
	struct dp8390_softc *sc = ifp->if_softc;

	log(LOG_ERR, "%s: device timeout\n", sc->sc_dev.dv_xname);
#ifdef __NetBSD__
	++sc->sc_ec.ec_if.if_oerrors;
#else
	++sc->sc_arpcom.ac_if.if_oerrors;
#endif

	dp8390_reset(sc);
}

/*
 * Initialize device.
 */
void
dp8390_init(sc)
	struct dp8390_softc *sc;
{
	bus_space_tag_t regt = sc->sc_regt;
	bus_space_handle_t regh = sc->sc_regh;
#ifdef __NetBSD__
	struct ifnet *ifp = &sc->sc_ec.ec_if;
#else
	struct ifnet *ifp = &sc->sc_arpcom.ac_if;
#endif
	u_int8_t mcaf[8];
	int i;

	/*
	 * Initialize the NIC in the exact order outlined in the NS manual.
	 * This init procedure is "mandatory"...don't change what or when
	 * things happen.
	 */

	/* Reset transmitter flags. */
	ifp->if_timer = 0;

	sc->txb_inuse = 0;
	sc->txb_new = 0;
	sc->txb_next_tx = 0;

	/* Set interface for page 0, remote DMA complete, stopped. */
	NIC_PUT(regt, regh, ED_P0_CR,
	    sc->cr_proto | ED_CR_PAGE_0 | ED_CR_STP);

	if (sc->dcr_reg & ED_DCR_LS) {
		NIC_PUT(regt, regh, ED_P0_DCR, sc->dcr_reg);
	} else {
		/*
		 * Set FIFO threshold to 8, No auto-init Remote DMA, byte
		 * order=80x86, byte-wide DMA xfers,
		 */
		NIC_PUT(regt, regh, ED_P0_DCR, ED_DCR_FT1 | ED_DCR_LS);
	}

	/* Clear remote byte count registers. */
	NIC_PUT(regt, regh, ED_P0_RBCR0, 0);
	NIC_PUT(regt, regh, ED_P0_RBCR1, 0);

	/* Tell RCR to do nothing for now. */
	NIC_PUT(regt, regh, ED_P0_RCR, ED_RCR_MON);

	/* Place NIC in internal loopback mode. */
	NIC_PUT(regt, regh, ED_P0_TCR, ED_TCR_LB0);

	/* Set lower bits of byte addressable framing to 0. */
	if (sc->is790)
		NIC_PUT(regt, regh, 0x09, 0);

	/* Initialize receive buffer ring. */
	NIC_PUT(regt, regh, ED_P0_BNRY, sc->rec_page_start);
	NIC_PUT(regt, regh, ED_P0_PSTART, sc->rec_page_start);
	NIC_PUT(regt, regh, ED_P0_PSTOP, sc->rec_page_stop);

	/*
	 * Enable the following interrupts: receive/transmit complete,
	 * receive/transmit error, and Receiver OverWrite.
	 *
	 * Counter overflow and Remote DMA complete are *not* enabled.
	 */
	NIC_PUT(regt, regh, ED_P0_IMR,
	    ED_IMR_PRXE | ED_IMR_PTXE | ED_IMR_RXEE | ED_IMR_TXEE |
	    ED_IMR_OVWE);

	/*
	 * Clear all interrupts.  A '1' in each bit position clears the
	 * corresponding flag.
	 */
	NIC_PUT(regt, regh, ED_P0_ISR, 0xff);

	/* Program command register for page 1. */
	NIC_PUT(regt, regh, ED_P0_CR,
	    sc->cr_proto | ED_CR_PAGE_1 | ED_CR_STP);

	/* Copy out our station address. */
	for (i = 0; i < ETHER_ADDR_LEN; ++i)
#ifdef __NetBSD__
		NIC_PUT(regt, regh, ED_P1_PAR0 + i,
		    LLADDR(ifp->if_sadl)[i]);
#else
		NIC_PUT(regt, regh, ED_P1_PAR0 + i,
		    sc->sc_arpcom.ac_enaddr[i]);
#endif

	/* Set multicast filter on chip. */
#ifdef __NetBSD__
	dp8390_getmcaf(&sc->sc_ec, mcaf);
#else
	dp8390_getmcaf(&sc->sc_arpcom, mcaf);
#endif
	for (i = 0; i < 8; i++)
		NIC_PUT(regt, regh, ED_P1_MAR0 + i, mcaf[i]);

	/*
	 * Set current page pointer to one page after the boundary pointer, as
	 * recommended in the National manual.
	 */
	sc->next_packet = sc->rec_page_start + 1;
	NIC_PUT(regt, regh, ED_P1_CURR, sc->next_packet);

	/* Program command register for page 0. */
	NIC_PUT(regt, regh, ED_P1_CR,
	    sc->cr_proto | ED_CR_PAGE_0 | ED_CR_STP);

	/* Accept broadcast and multicast packets by default. */
	i = ED_RCR_AB | ED_RCR_AM;
	if (ifp->if_flags & IFF_PROMISC) {
		/*
		 * Set promiscuous mode.  Multicast filter was set earlier so
		 * that we should receive all multicast packets.
		 */
		i |= ED_RCR_PRO | ED_RCR_AR | ED_RCR_SEP;
	}
	NIC_PUT(regt, regh, ED_P0_RCR, i);

	/* Take interface out of loopback. */
	NIC_PUT(regt, regh, ED_P0_TCR, 0);

	/* Do any card-specific initialization, if applicable. */
	if (sc->init_card)
		(*sc->init_card)(sc);

	/* Fire up the interface. */
	NIC_PUT(regt, regh, ED_P0_CR,
	    sc->cr_proto | ED_CR_PAGE_0 | ED_CR_STA);

	/* Set 'running' flag, and clear output active flag. */
	ifp->if_flags |= IFF_RUNNING;
	ifp->if_flags &= ~IFF_OACTIVE;

	/* ...and attempt to start output. */
	dp8390_start(ifp);
}

/*
 * This routine actually starts the transmission on the interface.
 */
static __inline__ void
dp8390_xmit(sc)
	struct dp8390_softc *sc;
{
	bus_space_tag_t regt = sc->sc_regt;
	bus_space_handle_t regh = sc->sc_regh;
#ifdef __NetBSD__
	struct ifnet *ifp = &sc->sc_ec.ec_if;
#else
	struct ifnet *ifp = &sc->sc_arpcom.ac_if;
#endif
	u_short len;

#ifdef DIAGNOSTIC
	if ((sc->txb_next_tx + sc->txb_inuse) % sc->txb_cnt != sc->txb_new)
		panic("dp8390_xmit: desync, next_tx=%d inuse=%d cnt=%d new=%d",
		    sc->txb_next_tx, sc->txb_inuse, sc->txb_cnt, sc->txb_new);

	if (sc->txb_inuse == 0)
		panic("dp8390_xmit: no packets to xmit\n");
#endif

	len = sc->txb_len[sc->txb_next_tx];

	/* Set NIC for page 0 register access. */
	NIC_PUT(regt, regh, ED_P0_CR,
	    sc->cr_proto | ED_CR_PAGE_0 | ED_CR_STA);

	/* Set TX buffer start page. */
	NIC_PUT(regt, regh, ED_P0_TPSR, sc->tx_page_start +
	    sc->txb_next_tx * ED_TXBUF_SIZE);

	/* Set TX length. */
	NIC_PUT(regt, regh, ED_P0_TBCR0, len);
	NIC_PUT(regt, regh, ED_P0_TBCR1, len >> 8);

	/* Set page 0, remote DMA complete, transmit packet, and *start*. */
	NIC_PUT(regt, regh, ED_P0_CR,
	    sc->cr_proto | ED_CR_PAGE_0 | ED_CR_TXP | ED_CR_STA);

	/* Point to next transmit buffer slot and wrap if necessary. */
	if (++sc->txb_next_tx == sc->txb_cnt)
		sc->txb_next_tx = 0;

	/* Set a timer just in case we never hear from the board again. */
	ifp->if_timer = 2;
}

/*
 * Start output on interface.
 * We make two assumptions here:
 *  1) that the current priority is set to splnet _before_ this code
 *     is called *and* is returned to the appropriate priority after
 *     return
 *  2) that the IFF_OACTIVE flag is checked before this code is called
 *     (i.e. that the output part of the interface is idle)
 */
void
dp8390_start(ifp)
	struct ifnet *ifp;
{
	struct dp8390_softc *sc = ifp->if_softc;
	struct mbuf *m0;
	int buffer;
	int len;

	if ((ifp->if_flags & (IFF_RUNNING | IFF_OACTIVE)) != IFF_RUNNING)
		return;

outloop:
	/* See if there is room to put another packet in the buffer. */
	if (sc->txb_inuse == sc->txb_cnt) {
		/* No room.  Indicate this to the outside world and exit. */
		ifp->if_flags |= IFF_OACTIVE;
		return;
	}
	IF_DEQUEUE(&ifp->if_snd, m0);
	if (m0 == 0)
		return;

	/* We need to use m->m_pkthdr.len, so require the header */
	if ((m0->m_flags & M_PKTHDR) == 0)
		panic("dp8390_start: no header mbuf");

#if NBPFILTER > 0
	/* Tap off here if there is a BPF listener. */
	if (ifp->if_bpf)
		bpf_mtap(ifp->if_bpf, m0);
#endif

	/* txb_new points to next open buffer slot. */
	buffer = sc->mem_start +
	    ((sc->txb_new * ED_TXBUF_SIZE) << ED_PAGE_SHIFT);

	if (sc->write_mbuf)
		len = (*sc->write_mbuf)(sc, m0, buffer);
	else
		len = dp8390_write_mbuf(sc, m0, buffer);

	m_freem(m0);
	sc->txb_len[sc->txb_new] = max(len, ETHER_MIN_LEN - ETHER_CRC_LEN);

	/* Point to next buffer slot and wrap if necessary. */
	if (++sc->txb_new == sc->txb_cnt)
		sc->txb_new = 0;

	/* Start the first packet transmitting. */
	if (sc->txb_inuse++ == 0)
		dp8390_xmit(sc);

	/* Loop back to the top to possibly buffer more packets. */
	goto outloop;
}

/*
 * Ethernet interface receiver interrupt.
 */
void
dp8390_rint(sc)
	struct dp8390_softc *sc;
{
	bus_space_tag_t regt = sc->sc_regt;
	bus_space_handle_t regh = sc->sc_regh;
	struct dp8390_ring packet_hdr;
	int packet_ptr;
	u_short len;
	u_char boundary, current;
	u_char nlen;

loop:
	/* Set NIC to page 1 registers to get 'current' pointer. */
	NIC_PUT(regt, regh, ED_P0_CR,
	    sc->cr_proto | ED_CR_PAGE_1 | ED_CR_STA);

	/*
	 * 'sc->next_packet' is the logical beginning of the ring-buffer - i.e.
	 * it points to where new data has been buffered.  The 'CURR' (current)
	 * register points to the logical end of the ring-buffer - i.e. it
	 * points to where additional new data will be added.  We loop here
	 * until the logical beginning equals the logical end (or in other
	 * words, until the ring-buffer is empty).
	 */
	current = NIC_GET(regt, regh, ED_P1_CURR);
	if (sc->next_packet == current)
		return;

	/* Set NIC to page 0 registers to update boundary register. */
	NIC_PUT(regt, regh, ED_P1_CR,
	    sc->cr_proto | ED_CR_PAGE_0 | ED_CR_STA);

	do {
		/* Get pointer to this buffer's header structure. */
		packet_ptr = sc->mem_ring +
		    ((sc->next_packet - sc->rec_page_start) << ED_PAGE_SHIFT);

		if (sc->read_hdr)
			(*sc->read_hdr)(sc, packet_ptr, &packet_hdr);
		else
			dp8390_read_hdr(sc, packet_ptr, &packet_hdr);
		len = packet_hdr.count;

		/*
		 * Try do deal with old, buggy chips that sometimes duplicate
		 * the low byte of the length into the high byte.  We do this
		 * by simply ignoring the high byte of the length and always
		 * recalculating it.
		 *
		 * NOTE: sc->next_packet is pointing at the current packet.
		 */
		if (packet_hdr.next_packet >= sc->next_packet)
			nlen = (packet_hdr.next_packet - sc->next_packet);
		else
			nlen = ((packet_hdr.next_packet - sc->rec_page_start) +
			    (sc->rec_page_stop - sc->next_packet));
		--nlen;
		if ((len & ED_PAGE_MASK) + sizeof(packet_hdr) > ED_PAGE_SIZE)
			--nlen;
		len = (len & ED_PAGE_MASK) | (nlen << ED_PAGE_SHIFT);
#ifdef DIAGNOSTIC
		if (len != packet_hdr.count) {
			printf("%s: length does not match "
			    "next packet pointer\n", sc->sc_dev.dv_xname);
			printf("%s: len %04x nlen %04x start %02x "
			    "first %02x curr %02x next %02x stop %02x\n",
			    sc->sc_dev.dv_xname, packet_hdr.count, len,
			    sc->rec_page_start, sc->next_packet, current,
			    packet_hdr.next_packet, sc->rec_page_stop);
		}
#endif

		/*
		 * Be fairly liberal about what we allow as a "reasonable"
		 * length so that a [crufty] packet will make it to BPF (and
		 * can thus be analyzed).  Note that all that is really
		 * important is that we have a length that will fit into one
		 * mbuf cluster or less; the upper layer protocols can then
		 * figure out the length from their own length field(s).
		 */
		if (len <= MCLBYTES &&
		    packet_hdr.next_packet >= sc->rec_page_start &&
		    packet_hdr.next_packet < sc->rec_page_stop) {
			/* Go get packet. */
			dp8390_read(sc,
			    packet_ptr + sizeof(struct dp8390_ring),
			    len - sizeof(struct dp8390_ring));
#ifdef __NetBSD__
			++sc->sc_ec.ec_if.if_ipackets;
#else
			++sc->sc_arpcom.ac_if.if_ipackets;
#endif
		} else {
			/* Really BAD.  The ring pointers are corrupted. */
			log(LOG_ERR, "%s: NIC memory corrupt - "
			    "invalid packet length %d\n",
			    sc->sc_dev.dv_xname, len);
#ifdef __NetBSD__
			++sc->sc_ec.ec_if.if_ierrors;
#else
			++sc->sc_arpcom.ac_if.if_ierrors;
#endif
			dp8390_reset(sc);
			return;
		}

		/* Update next packet pointer. */
		sc->next_packet = packet_hdr.next_packet;

		/*
		 * Update NIC boundary pointer - being careful to keep it one
		 * buffer behind (as recommended by NS databook).
		 */
		boundary = sc->next_packet - 1;
		if (boundary < sc->rec_page_start)
			boundary = sc->rec_page_stop - 1;
		NIC_PUT(regt, regh, ED_P0_BNRY, boundary);
	} while (sc->next_packet != current);

	goto loop;
}

/* Ethernet interface interrupt processor. */
int
dp8390_intr(arg)
	void *arg;
{
	struct dp8390_softc *sc = (struct dp8390_softc *)arg;
	bus_space_tag_t regt = sc->sc_regt;
	bus_space_handle_t regh = sc->sc_regh;
#ifdef __NetBSD__
	struct ifnet *ifp = &sc->sc_ec.ec_if;
#else
	struct ifnet *ifp = &sc->sc_arpcom.ac_if;
#endif
	u_char isr;

	if (sc->sc_enabled == 0)
		return (0);

	/* Set NIC to page 0 registers. */
	NIC_PUT(regt, regh, ED_P0_CR,
	    sc->cr_proto | ED_CR_PAGE_0 | ED_CR_STA);

	isr = NIC_GET(regt, regh, ED_P0_ISR);
	if (!isr)
		return (0);

	/* Loop until there are no more new interrupts. */
	for (;;) {
		/*
		 * Reset all the bits that we are 'acknowledging' by writing a
		 * '1' to each bit position that was set.
		 * (Writing a '1' *clears* the bit.)
		 */
		NIC_PUT(regt, regh, ED_P0_ISR, isr);

		/*
		 * Handle transmitter interrupts.  Handle these first because
		 * the receiver will reset the board under some conditions.
		 *
		 * If the chip was reset while a packet was transmitting, it
		 * may still deliver a TX interrupt.  In this case, just ignore
		 * the interrupt.
		 */
		if (isr & (ED_ISR_PTX | ED_ISR_TXE) &&
		    sc->txb_inuse != 0) {
			u_char collisions =
			    NIC_GET(regt, regh, ED_P0_NCR) & 0x0f;

			/*
			 * Check for transmit error.  If a TX completed with an
			 * error, we end up throwing the packet away.  Really
			 * the only error that is possible is excessive
			 * collisions, and in this case it is best to allow the
			 * automatic mechanisms of TCP to backoff the flow.  Of
			 * course, with UDP we're screwed, but this is expected
			 * when a network is heavily loaded.
			 */
			if (isr & ED_ISR_TXE) {
				/*
				 * Excessive collisions (16).
				 */
				if ((NIC_GET(regt, regh, ED_P0_TSR)
				    & ED_TSR_ABT) && (collisions == 0)) {
					/*
					 * When collisions total 16, the P0_NCR
					 * will indicate 0, and the TSR_ABT is
					 * set.
					 */
					collisions = 16;
				}

				/* Update output errors counter. */
				++ifp->if_oerrors;
			} else {
				/* Throw away the non-error status bits. */
				(void)NIC_GET(regt, regh, ED_P0_TSR);

				/*
				 * Update total number of successfully
				 * transmitted packets.
				 */
				++ifp->if_opackets;
			}

			/* Clear watchdog timer. */
			ifp->if_timer = 0;
			ifp->if_flags &= ~IFF_OACTIVE;

			/*
			 * Add in total number of collisions on last
			 * transmission.
			 */
			ifp->if_collisions += collisions;

			/*
			 * Decrement buffer in-use count if not zero (can only
			 * be zero if a transmitter interrupt occured while not
			 * actually transmitting).
			 * If data is ready to transmit, start it transmitting,
			 * otherwise defer until after handling receiver.
			 */
			if (--sc->txb_inuse != 0)
				dp8390_xmit(sc);
		}

		/* Handle receiver interrupts. */
		if (isr & (ED_ISR_PRX | ED_ISR_RXE | ED_ISR_OVW)) {
			/*
			 * Overwrite warning.  In order to make sure that a
			 * lockup of the local DMA hasn't occurred, we reset
			 * and re-init the NIC.  The NSC manual suggests only a
			 * partial reset/re-init is necessary - but some chips
			 * seem to want more.  The DMA lockup has been seen
			 * only with early rev chips - Methinks this bug was
			 * fixed in later revs.  -DG
			 */
			if (isr & ED_ISR_OVW) {
				++ifp->if_ierrors;
#ifdef DIAGNOSTIC
				log(LOG_WARNING, "%s: warning - receiver "
				    "ring buffer overrun\n",
				    sc->sc_dev.dv_xname);
#endif
				/* Stop/reset/re-init NIC. */
				dp8390_reset(sc);
			} else {
				/*
				 * Receiver Error.  One or more of: CRC error,
				 * frame alignment error FIFO overrun, or
				 * missed packet.
				 */
				if (isr & ED_ISR_RXE) {
					++ifp->if_ierrors;
#ifdef DEBUG
					if (dp8390_debug) {
						printf("%s: receive error %x\n",
						    sc->sc_dev.dv_xname,
						    NIC_GET(regt, regh,
							ED_P0_RSR));
					}
#endif
				}

				/*
				 * Go get the packet(s)
				 * XXX - Doing this on an error is dubious
				 * because there shouldn't be any data to get
				 * (we've configured the interface to not
				 * accept packets with errors).
				 */
				if (sc->recv_int)
					(*sc->recv_int)(sc);
				else
					dp8390_rint(sc);
			}
		}

		/*
		 * If it looks like the transmitter can take more data, attempt
		 * to start output on the interface.  This is done after
		 * handling the receiver to give the receiver priority.
		 */
		dp8390_start(ifp);

		/*
		 * Return NIC CR to standard state: page 0, remote DMA
		 * complete, start (toggling the TXP bit off, even if was just
		 * set in the transmit routine, is *okay* - it is 'edge'
		 * triggered from low to high).
		 */
		NIC_PUT(regt, regh, ED_P0_CR,
		    sc->cr_proto | ED_CR_PAGE_0 | ED_CR_STA);

		/*
		 * If the Network Talley Counters overflow, read them to reset
		 * them.  It appears that old 8390's won't clear the ISR flag
		 * otherwise - resulting in an infinite loop.
		 */
		if (isr & ED_ISR_CNT) {
			(void)NIC_GET(regt, regh, ED_P0_CNTR0);
			(void)NIC_GET(regt, regh, ED_P0_CNTR1);
			(void)NIC_GET(regt, regh, ED_P0_CNTR2);
		}

		isr = NIC_GET(regt, regh, ED_P0_ISR);
		if (!isr)
			return (1);
	}
}

/*
 * Process an ioctl request.  This code needs some work - it looks pretty ugly.
 */
int
dp8390_ioctl(ifp, cmd, data)
	struct ifnet *ifp;
	u_long cmd;
	caddr_t data;
{
	struct dp8390_softc *sc = ifp->if_softc;
	struct ifaddr *ifa = (struct ifaddr *) data;
	struct ifreq *ifr = (struct ifreq *) data;
	int s, error = 0;

	s = splnet();

	switch (cmd) {

	case SIOCSIFADDR:
		if ((error = dp8390_enable(sc)) != 0)
			break;
		ifp->if_flags |= IFF_UP;

		switch (ifa->ifa_addr->sa_family) {
#ifdef INET
		case AF_INET:
			dp8390_init(sc);
#ifdef __NetBSD__
			arp_ifinit(ifp, ifa);
#else
			arp_ifinit(&sc->sc_arpcom, ifa);
#endif
			break;
#endif
#ifdef NS
			/* XXX - This code is probably wrong. */
		case AF_NS:
		    {
			struct ns_addr *ina = &IA_SNS(ifa)->sns_addr;

			if (ns_nullhost(*ina))
				ina->x_host =
				    *(union ns_host *)LLADDR(ifp->if_sadl);
			else
				bcopy(ina->x_host.c_host, LLADDR(ifp->if_sadl),
				    ETHER_ADDR_LEN);
			/* Set new address. */
			dp8390_init(sc);
			break;
		    }
#endif
		default:
			dp8390_init(sc);
			break;
		}
		break;

	case SIOCSIFFLAGS:
		if ((ifp->if_flags & IFF_UP) == 0 &&
		    (ifp->if_flags & IFF_RUNNING) != 0) {
			/*
			 * If interface is marked down and it is running, then
			 * stop it.
			 */
			dp8390_stop(sc);
			ifp->if_flags &= ~IFF_RUNNING;
			dp8390_disable(sc);
		} else if ((ifp->if_flags & IFF_UP) != 0 &&
		    (ifp->if_flags & IFF_RUNNING) == 0) {
			/*
			 * If interface is marked up and it is stopped, then
			 * start it.
			 */
			if ((error = dp8390_enable(sc)) != 0)
				break;
			dp8390_init(sc);
		} else if ((ifp->if_flags & IFF_UP) != 0) {
			/*
			 * Reset the interface to pick up changes in any other
			 * flags that affect hardware registers.
			 */
			dp8390_stop(sc);
			dp8390_init(sc);
		}
		break;

	case SIOCADDMULTI:
	case SIOCDELMULTI:
		if (sc->sc_enabled == 0) {
			error = EIO;
			break;
		}

		/* Update our multicast list. */
		error = (cmd == SIOCADDMULTI) ?
#ifdef __NetBSD__
		    ether_addmulti(ifr, &sc->sc_ec) :
		    ether_delmulti(ifr, &sc->sc_ec);
#else
		    ether_addmulti(ifr, &sc->sc_arpcom) :
		    ether_delmulti(ifr, &sc->sc_arpcom);
#endif

		if (error == ENETRESET) {
			/*
			 * Multicast list has changed; set the hardware filter
			 * accordingly.
			 */
			dp8390_stop(sc);	/* XXX for ds_setmcaf? */
			dp8390_init(sc);
			error = 0;
		}
		break;

	case SIOCGIFMEDIA:
	case SIOCSIFMEDIA:
		error = ifmedia_ioctl(ifp, ifr, &sc->sc_media, cmd);
		break;

	default:
		error = EINVAL;
		break;
	}

	splx(s);
	return (error);
}

/*
 * Retrieve packet from buffer memory and send to the next level up via
 * ether_input().  If there is a BPF listener, give a copy to BPF, too.
 */
void
dp8390_read(sc, buf, len)
	struct dp8390_softc *sc;
	int buf;
	u_short len;
{
#ifdef __NetBSD__
	struct ifnet *ifp = &sc->sc_ec.ec_if;
#else
	struct ifnet *ifp = &sc->sc_arpcom.ac_if;
#endif
	struct mbuf *m;
	struct ether_header *eh;

	/* Pull packet off interface. */
	m = dp8390_get(sc, buf, len);
	if (m == 0) {
		ifp->if_ierrors++;
		return;
	}

	ifp->if_ipackets++;

	/* We assume that the header fits entirely in one mbuf. */
	eh = mtod(m, struct ether_header *);

#if NBPFILTER > 0
	/*
	 * Check if there's a BPF listener on this interface.
	 * If so, hand off the raw packet to bpf.
	 */
	if (ifp->if_bpf)
		bpf_mtap(ifp->if_bpf, m);
#endif

	/* Fix up data start offset in mbuf to point past ether header. */
	m_adj(m, sizeof(struct ether_header));
	ether_input(ifp, eh, m);
}


/*
 * Supporting routines.
 */

/*
 * Compute the multicast address filter from the list of multicast addresses we
 * need to listen to.
 */
void
dp8390_getmcaf(ec, af)
#ifdef __NetBSD__
	struct ethercom *ec;
#else
	struct arpcom *ec;
#endif
	u_int8_t *af;
{
#ifdef __NetBSD__
	struct ifnet *ifp = &ec->ec_if;
#else
	struct ifnet *ifp = &ec->ac_if;
#endif
	struct ether_multi *enm;
	u_int8_t *cp, c;
	u_int32_t crc;
	int i, len;
	struct ether_multistep step;

	/*
	 * Set up multicast address filter by passing all multicast addresses
	 * through a crc generator, and then using the high order 6 bits as an
	 * index into the 64 bit logical address filter.  The high order bit
	 * selects the word, while the rest of the bits select the bit within
	 * the word.
	 */

	if (ifp->if_flags & IFF_PROMISC) {
		ifp->if_flags |= IFF_ALLMULTI;
		for (i = 0; i < 8; i++)
			af[i] = 0xff;
		return;
	}
	for (i = 0; i < 8; i++)
		af[i] = 0;
	ETHER_FIRST_MULTI(step, ec, enm);
	while (enm != NULL) {
		if (bcmp(enm->enm_addrlo, enm->enm_addrhi,
		    sizeof(enm->enm_addrlo)) != 0) {
			/*
			 * We must listen to a range of multicast addresses.
			 * For now, just accept all multicasts, rather than
			 * trying to set only those filter bits needed to match
			 * the range.  (At this time, the only use of address
			 * ranges is for IP multicast routing, for which the
			 * range is big enough to require all bits set.)
			 */
			ifp->if_flags |= IFF_ALLMULTI;
			for (i = 0; i < 8; i++)
				af[i] = 0xff;
			return;
		}
		cp = enm->enm_addrlo;
		crc = 0xffffffff;
		for (len = sizeof(enm->enm_addrlo); --len >= 0;) {
			c = *cp++;
			for (i = 8; --i >= 0;) {
				if (((crc & 0x80000000) ? 1 : 0) ^ (c & 0x01)) {
					crc <<= 1;
					crc ^= 0x04c11db6 | 1;
				} else
					crc <<= 1;
				c >>= 1;
			}
		}
		/* Just want the 6 most significant bits. */
		crc >>= 26;

		/* Turn on the corresponding bit in the filter. */
		af[crc >> 3] |= 1 << (crc & 0x7);

		ETHER_NEXT_MULTI(step, enm);
	}
	ifp->if_flags &= ~IFF_ALLMULTI;
}

/*
 * Copy data from receive buffer to end of mbuf chain allocate additional mbufs
 * as needed.  Return pointer to last mbuf in chain.
 * sc = dp8390 info (softc)
 * src = pointer in dp8390 ring buffer
 * dst = pointer to last mbuf in mbuf chain to copy to
 * amount = amount of data to copy
 */
struct mbuf *
dp8390_get(sc, src, total_len)
	struct dp8390_softc *sc;
	int src;
	u_short total_len;
{
#ifdef __NetBSD__
	struct ifnet *ifp = &sc->sc_ec.ec_if;
#else
	struct ifnet *ifp = &sc->sc_arpcom.ac_if;
#endif
	struct mbuf *top, **mp, *m;
	u_short len;

	MGETHDR(m, M_DONTWAIT, MT_DATA);
	if (m == 0)
		return 0;
	m->m_pkthdr.rcvif = ifp;
	m->m_pkthdr.len = total_len;
	len = MHLEN;
	top = 0;
	mp = &top;

	while (total_len > 0) {
		if (top) {
			MGET(m, M_DONTWAIT, MT_DATA);
			if (m == 0) {
				m_freem(top);
				return 0;
			}
			len = MLEN;
		}
		if (total_len >= MINCLSIZE) {
			MCLGET(m, M_DONTWAIT);
			if ((m->m_flags & M_EXT) == 0) {
				m_freem(m);
				m_freem(top);
				return 0;
			}
			len = MCLBYTES;
		}

		/*
		 * Make sure the data after the Ethernet header is aligned.
		 */
		if (top == NULL) {
			caddr_t newdata = (caddr_t)
			    ALIGN(m->m_data + sizeof(struct ether_header)) -
			    sizeof(struct ether_header);
			len -= newdata - m->m_data;
			m->m_data = newdata;
		}

		m->m_len = len = min(total_len, len);
		if (sc->ring_copy)
			src = (*sc->ring_copy)(sc, src, mtod(m, caddr_t), len);
		else
			src = dp8390_ring_copy(sc, src, mtod(m, caddr_t), len);
		total_len -= len;
		*mp = m;
		mp = &m->m_next;
	}

	return top;
}


/*
 * Default driver support functions.
 *
 * NOTE: all support functions assume 8-bit shared memory.
 */
/*
 * Zero NIC buffer memory and verify that it is clear.
 */
static int
dp8390_test_mem(sc)
	struct dp8390_softc *sc;
{
	bus_space_tag_t buft = sc->sc_buft;
	bus_space_handle_t bufh = sc->sc_bufh;
	int i;

	bus_space_set_region_1(buft, bufh, sc->mem_start, 0, sc->mem_size);

	for (i = 0; i < sc->mem_size; ++i) {
		if (bus_space_read_1(buft, bufh, sc->mem_start + i)) {
			printf(": failed to clear NIC buffer at offset %x - "
			    "check configuration\n", (sc->mem_start + i));
			return 1;
		}
	}

	return 0;
}

/*
 * Read a packet header from the ring, given the source offset.
 */
static __inline__ void
dp8390_read_hdr(sc, src, hdrp)
	struct dp8390_softc *sc;
	int src;
	struct dp8390_ring *hdrp;
{
	bus_space_tag_t buft = sc->sc_buft;
	bus_space_handle_t bufh = sc->sc_bufh;

	/*
	 * The byte count includes a 4 byte header that was added by
	 * the NIC.
	 */
	hdrp->rsr = bus_space_read_1(buft, bufh, src);
	hdrp->next_packet = bus_space_read_1(buft, bufh, src + 1);
	hdrp->count = bus_space_read_1(buft, bufh, src + 2) |
	    (bus_space_read_1(buft, bufh, src + 3) << 8);
}

/*
 * Copy `amount' bytes from a packet in the ring buffer to a linear
 * destination buffer, given a source offset and destination address.
 * Takes into account ring-wrap.
 */
static __inline__ int
dp8390_ring_copy(sc, src, dst, amount)
	struct dp8390_softc *sc;
	int src;
	caddr_t dst;
	u_short amount;
{
	bus_space_tag_t buft = sc->sc_buft;
	bus_space_handle_t bufh = sc->sc_bufh;
	u_short tmp_amount;

	/* Does copy wrap to lower addr in ring buffer? */
	if (src + amount > sc->mem_end) {
		tmp_amount = sc->mem_end - src;

		/* Copy amount up to end of NIC memory. */
		bus_space_read_region_1(buft, bufh, src, dst, tmp_amount);

		amount -= tmp_amount;
		src = sc->mem_ring;
		dst += tmp_amount;
	}
	bus_space_read_region_1(buft, bufh, src, dst, amount);

	return (src + amount);
}

/*
 * Copy a packet from an mbuf to the transmit buffer on the card.
 *
 * Currently uses an extra buffer/extra memory copy, unless the whole
 * packet fits in one mbuf.
 */
static __inline__ int
dp8390_write_mbuf(sc, m, buf)
	struct dp8390_softc *sc;
	struct mbuf *m;
	int buf;
{
	bus_space_tag_t buft = sc->sc_buft;
	bus_space_handle_t bufh = sc->sc_bufh;
	u_char *data;
	int len, totlen = 0;

	for (; m ; m = m->m_next) {
		data = mtod(m, u_char *);
		len = m->m_len;
		if (len > 0) {
			bus_space_write_region_1(buft, bufh, buf, data, len);
			totlen += len;
			buf += len;
		}
	}

	return (totlen);
}

/*
 * Enable power on the interface.
 */
int
dp8390_enable(sc)
	struct dp8390_softc *sc;
{

	if (sc->sc_enabled == 0 && sc->sc_enable != NULL) {
		if ((*sc->sc_enable)(sc) != 0) {
			printf("%s: device enable failed\n",
			    sc->sc_dev.dv_xname);
			return (EIO);
		}
	}

	sc->sc_enabled = 1;
	return (0);
}

/*
 * Disable power on the interface.
 */
void
dp8390_disable(sc)
	struct dp8390_softc *sc;
{

	if (sc->sc_enabled != 0 && sc->sc_disable != NULL) {
		(*sc->sc_disable)(sc);
		sc->sc_enabled = 0;
	}
}
