/*	$NetBSD: xencons.c,v 1.8 2005/04/20 22:01:24 bouyer Exp $	*/

/*
 *
 * Copyright (c) 2004 Christian Limpach.
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
 *      This product includes software developed by Christian Limpach.
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


#include <sys/param.h>
#include <sys/proc.h>
#include <sys/tty.h>
#include <sys/systm.h>
#include <sys/device.h>
#include <sys/conf.h>
#include <sys/poll.h>

#include <machine/xen.h>
#include <machine/hypervisor.h>
#include <machine/evtchn.h>
#include <uvm/uvm.h>
#include <machine/pmap.h>
#include <machine/xen-public/io/console.h>
#include <dev/cons.h>

#ifdef DDB
#include <ddb/db_var.h>	/* XXX for db_max_line */
#endif

#undef XENDEBUG

#ifdef XENDEBUG
#define XENPRINTK(x) printk x
#else
#define XENPRINTK(x)
#endif

#define __UNVOLATILE(a)     ((void *)(unsigned long)(volatile void *)(a))

static int xencons_isconsole = 0;
static struct xencons_softc *xencons_console_device = NULL;

#define	XENCONS_UNIT(x)	(minor(x))
#define XENCONS_BURST 128

int		 xencons_match(struct device *, void*, void *);
void		 xencons_attach(struct device *, struct device *, void *);
int		 xencons_intr (void *);
void		 xencons_tty_input(struct xencons_softc *, char*, int);

int		 xenconsopen(dev_t, int, int, struct proc *);
int		 xenconsclose(dev_t, int, int, struct proc *);
int		 xenconsread(dev_t, struct uio *, int);
int		 xenconswrite(dev_t, struct uio *, int);
int		 xenconsioctl(dev_t, u_long, caddr_t, int, struct proc *);
int		 xenconsstop(struct tty *, int);
struct tty	*xenconstty(dev_t);
int		 xenconspoll(dev_t, int, struct proc *);

struct xencons_softc {
	struct device	 sc_dev;
	struct tty	*sc_tty;
	int		 polling;
};
volatile struct xencons_interface *xencons_interface;

struct cfattach	xencons_ca = {
	sizeof(struct xencons_softc), xencons_match, xencons_attach,
	NULL, NULL
};

struct cfdriver xencons_cd = {
	NULL, "xencons", DV_TTY, 0, 0
};

const struct cdevsw xencons_cdevsw = {
	xenconsopen, xenconsclose, xenconsread, xenconswrite,
	xenconsioctl, xenconsstop, xenconstty, xenconspoll,
	NULL, D_TTY, D_KQFILTER, ttkqfilter
};


static int	xencons_handler(void *);
int		xenconscn_getc(dev_t);
void		xenconscn_putc(dev_t, int);
void		xenconscn_pollc(dev_t, int);

static struct consdev xencons = {
	NULL, NULL, xenconscn_getc, xenconscn_putc, xenconscn_pollc,
	NULL, NODEV, CN_NORMAL
};


/* XXXXXXXX - this is in MI code in NetBSD */
/*
 * Stuff to handle debugger magic key sequences.
 */
#define CNS_LEN                 128
#define CNS_MAGIC_VAL(x)        ((x)&0x1ff)
#define CNS_MAGIC_NEXT(x)       (((x)>>9)&0x7f)
#define CNS_TERM                0x7f    /* End of sequence */

typedef struct cnm_state {
        int     cnm_state;
        u_short *cnm_magic;
} cnm_state_t;

#ifdef DDB
#include <ddb/db_var.h>
#define cn_trap()       do { if (db_console) Debugger(); } while (0)
#else
#define cn_trap()
#endif
#define cn_isconsole(d) ((d) == cn_tab->cn_dev)
void cn_init_magic(cnm_state_t *cnm);
void cn_destroy_magic(cnm_state_t *cnm);
int cn_set_magic(char *magic);
int cn_get_magic(char *magic, int len);
/* This should be called for each byte read */
#ifndef cn_check_magic
#define cn_check_magic(d, k, s)                                         \
	do {                                                            \
		if (cn_isconsole(d)) {                                  \
			int v = (s).cnm_magic[(s).cnm_state];           \
			if ((k) == CNS_MAGIC_VAL(v)) {                  \
				(s).cnm_state = CNS_MAGIC_NEXT(v);      \
				if ((s).cnm_state == CNS_TERM) {        \
					cn_trap();                      \
					(s).cnm_state = 0;              \
				}                                       \
			} else {                                        \
				(s).cnm_state = 0;                      \
			}                                               \
		}                                                       \
	} while (/* CONSTCOND */ 0)
#endif

/* Encode out-of-band events this way when passing to cn_check_magic() */
#define CNC_BREAK               0x100

/* XXXXXXXXXX - end of this part of cnmagic, more at the end of this file. */


static struct cnm_state xencons_cnm_state;

void		xenconsstart (struct tty *);
int		xenconsparam (struct tty *, struct termios *);

int
xencons_match(struct device *parent, void *match, void *aux)
{
	struct xencons_attach_args *xa = (struct xencons_attach_args *)aux;

	if (strcmp(xa->xa_device, "xencons") == 0)
		return 1;
	return 0;
}

void
xencons_attach(struct device *parent, struct device *self, void *aux)
{
	struct xencons_softc *sc = (void *)self;

	printf(": Xen Virtual Console Driver\n");

	sc->sc_tty = ttymalloc();
#if 0
	tty_attach(sc->sc_tty);
#endif
	sc->sc_tty->t_oproc = xenconsstart;
	sc->sc_tty->t_param = xenconsparam;

	if (xencons_isconsole) {
		int maj;

		/* Locate the major number. */
		for (maj = 0; maj < nchrdev; maj++)
			if (cdevsw[maj].d_open == xenconsopen)
				break;

		/* There can be only one, but it can have any unit number. */
		cn_tab->cn_dev = makedev(maj, sc->sc_dev.dv_unit);

		printf("%s: console major %d, unit %d\n", sc->sc_dev.dv_xname,
		    maj, sc->sc_dev.dv_unit);

		sc->sc_tty->t_dev = cn_tab->cn_dev;

#ifdef DDB
		/* Set db_max_line to avoid paging. */
		db_max_line = 0x7fffffff;
#endif

		if (xen_start_info.flags & SIF_INITDOMAIN) {
			int evtch = bind_virq_to_evtch(VIRQ_CONSOLE);
			printf("%s: using event channel %d\n",
			    sc->sc_dev.dv_xname, evtch);
			if (event_set_handler(evtch, xencons_intr, sc,
			    IPL_TTY, "xencons") != 0)
				printf("console: can't register xencons_intr\n");
			hypervisor_enable_event(evtch);
		} else {
			printf("%s: using event channel %d\n",
			    sc->sc_dev.dv_xname, xen_start_info.console_evtchn);
			event_set_handler(xen_start_info.console_evtchn,
			    xencons_handler, sc, IPL_TTY, "xencons");
			hypervisor_enable_event(xen_start_info.console_evtchn);
		}
		xencons_console_device = sc;
	}
	sc->polling = 0;
}

int
xenconsopen(dev_t dev, int flag, int mode, struct proc *p)
{
	struct tty *tp;

	tp = xenconstty(dev);
	if (tp == NULL)
		return (ENXIO);

	if ((tp->t_state & TS_ISOPEN) == 0) {
		tp->t_dev = dev;
		ttychars(tp);
		tp->t_iflag = TTYDEF_IFLAG;
		tp->t_oflag = TTYDEF_OFLAG;
		tp->t_cflag = TTYDEF_CFLAG;
		tp->t_lflag = TTYDEF_LFLAG;
		tp->t_ispeed = tp->t_ospeed = TTYDEF_SPEED;
		xenconsparam(tp, &tp->t_termios);
		ttsetwater(tp);
	} else if (tp->t_state&TS_XCLUDE && p->p_ucred->cr_uid != 0)
		return (EBUSY);
	tp->t_state |= TS_CARR_ON;

	return ((*linesw[tp->t_line].l_open)(dev, tp));
}

int
xenconsclose(dev_t dev, int flag, int mode, struct proc *p)
{
	struct tty *tp = xenconstty(dev);

	if (tp == NULL)
		return (0);
	(*linesw[tp->t_line].l_close)(tp, flag);
	ttyclose(tp);
#ifdef notyet /* XXX */
	ttyfree(tp);
#endif
	return (0);
}

int
xenconsread(dev_t dev, struct uio *uio, int flag)
{
	struct tty *tp = xenconstty(dev);

	return ((*linesw[tp->t_line].l_read)(tp, uio, flag));
}

int
xenconswrite(dev_t dev, struct uio *uio, int flag)
{
	struct tty *tp = xenconstty(dev);

	return ((*linesw[tp->t_line].l_write)(tp, uio, flag));
}


int
xenconspoll(dev_t dev, int events, struct proc *p)
{
#ifdef __NetBSD__
	struct tty *tp = xenconstty(dev);
	return ((*linesw[tp->t_line].l_poll)(tp, events, p));
#endif
#ifdef __OpenBSD__
	return ttpoll(dev, events, p);
#endif
}

struct tty *
xenconstty(dev_t dev)
{
	struct xencons_softc *sc =
	    (struct xencons_softc *)device_lookup(&xencons_cd,
	    XENCONS_UNIT(dev));
	struct tty *tp = sc->sc_tty;

	return (tp);
}

int
xenconsioctl(dev_t dev, u_long cmd, caddr_t data, int flag, struct proc *p)
{
	struct tty *tp = xenconstty(dev);
	int error;

	error = (*linesw[tp->t_line].l_ioctl)(tp, cmd, data, flag, p);
	if (error >= 0)
		return (error);

	error = ttioctl(tp, cmd, data, flag, p);
	if (error >= 0)
		return (error);

	switch (cmd) {
	default:
		return (ENOTTY);
	}

#ifdef DIAGNOSTIC
	panic("xencons_ioctl: impossible");
#endif
}

void
xenconsstart(struct tty *tp)
{
	struct clist *cl;
	int s, len;

	s = spltty();
	if (tp->t_state & (TS_TIMEOUT | TS_BUSY | TS_TTSTOP))
		goto out;
	tp->t_state |= TS_BUSY;
	splx(s);

	/*
	 * We need to do this outside spl since it could be fairly
	 * expensive and we don't want our serial ports to overflow.
	 */
	cl = &tp->t_outq;
	if (xen_start_info.flags & SIF_INITDOMAIN) {
		int r;
		u_char buf[XENCONS_BURST+1];

		len = q_to_b(cl, buf, XENCONS_BURST);
		while (len > 0) {
			r = HYPERVISOR_console_io(CONSOLEIO_write, len, buf);
			if (r <= 0)
				break;
			len -= r;
		}
	} else {
		XENCONS_RING_IDX cons, prod, len;

#define XNC_OUT (xencons_interface->out)
		cons = xencons_interface->out_cons;
		prod = xencons_interface->out_prod;
		x86_lfence();
		while (prod != cons + sizeof(xencons_interface->out)) {
			if (MASK_XENCONS_IDX(prod, XNC_OUT) <
			    MASK_XENCONS_IDX(cons, XNC_OUT)) {
				len = MASK_XENCONS_IDX(cons, XNC_OUT) -
					MASK_XENCONS_IDX(prod, XNC_OUT);
			} else {
				len = sizeof(XNC_OUT) -
					MASK_XENCONS_IDX(prod, XNC_OUT);
			}
			len = q_to_b(cl, __UNVOLATILE(
			    &XNC_OUT[MASK_XENCONS_IDX(prod, XNC_OUT)]), len);
			if (len == 0)
				break;
			prod += len;
		}
		x86_sfence();
		xencons_interface->out_prod = prod;
		x86_sfence();
		hypervisor_notify_via_evtchn(xen_start_info.console_evtchn);
#undef XNC_OUT
	}

	s = spltty();
	tp->t_state &= ~TS_BUSY;
	if (cl->c_cc) {
		tp->t_state |= TS_TIMEOUT;
#ifdef __NetBSD__
		callout_reset(&tp->t_rstrt_ch, 1, ttrstrt, tp);
#endif
#ifdef __OpenBSD__
		timeout_set(&tp->t_rstrt_to, ttrstrt, tp);
		timeout_add(&tp->t_rstrt_to, 1);
#endif
	}
	if (cl->c_cc <= tp->t_lowat) {
		if (tp->t_state & TS_ASLEEP) {
			tp->t_state &= ~TS_ASLEEP;
			wakeup(cl);
		}
		selwakeup(&tp->t_wsel);
	}
out:
	splx(s);
}

int
xenconsstop(struct tty *tp, int flag)
{
	return (0);
}


/* Non-privileged receive callback. */
static int
xencons_handler(void *arg)
{
	struct xencons_softc *sc = arg;
	XENCONS_RING_IDX cons, prod, len;
	int s = spltty();

	if (sc->polling) {
		splx(s);
		return 1;
	}

#define XNC_IN (xencons_interface->in)

	cons = xencons_interface->in_cons;
	prod = xencons_interface->in_prod;
	x86_lfence();
	while (cons != prod) {
		if (MASK_XENCONS_IDX(cons, XNC_IN) <
		    MASK_XENCONS_IDX(prod, XNC_IN)) {
			len = MASK_XENCONS_IDX(prod, XNC_IN) -
				MASK_XENCONS_IDX(cons, XNC_IN);
		} else {
			len = sizeof(XNC_IN) - MASK_XENCONS_IDX(cons, XNC_IN);
		}

		xencons_tty_input(sc, __UNVOLATILE(
		    &XNC_IN[MASK_XENCONS_IDX(cons, XNC_IN)]), len);
		if (__predict_false(xencons_interface->in_cons != cons)) {
			/* catch up with xenconscn_getc() */
			cons = xencons_interface->in_cons;
			prod = xencons_interface->in_prod;
			x86_lfence();
		} else {
			cons += len;
			x86_sfence();
			xencons_interface->in_cons = cons;
			x86_sfence();
		}
	}
	hypervisor_notify_via_evtchn(xen_start_info.console_evtchn);
	splx(s);
	return 1;
#undef XNC_IN
}	

void
xencons_tty_input(struct xencons_softc *sc, char* buf, int len)
{
	struct tty *tp;
	int i;

	tp = sc->sc_tty;
	if (tp == NULL)
		return;

	for (i = 0; i < len; i++) {
		cn_check_magic(sc->sc_tty->t_dev, buf[i], xencons_cnm_state);
		(*linesw[tp->t_line].l_rint)(buf[i], tp);
	}
}

/* privileged receive callback */
int
xencons_intr(void *p)
{
	static char rbuf[16];
	int len;
	struct xencons_softc *sc = p;

	if (sc->polling)
		return 1;

	while ((len =
	    HYPERVISOR_console_io(CONSOLEIO_read, sizeof(rbuf), rbuf)) > 0) {
		xencons_tty_input(sc, rbuf, len);
	}
	return 1;
}

void
xenconscn_attach(void)
{
	cn_tab = &xencons;

	/* console ring mapped in locore.S */

	cn_init_magic(&xencons_cnm_state);
	cn_set_magic("+++++");

	xencons_isconsole = 1;
}

int
xenconscn_getc(dev_t dev)
{
	char c;
	int s = spltty();
	XENCONS_RING_IDX cons, prod;

	if (xencons_console_device && xencons_console_device->polling == 0) {
		printf("xenconscn_getc() but not polling\n");
		splx(s);
		return 0;
	}
	if (xen_start_info.flags & SIF_INITDOMAIN) {
		while (HYPERVISOR_console_io(CONSOLEIO_read, 1, &c) == 0)
			;

		cn_check_magic(dev, c, xencons_cnm_state);
	
		splx(s);
		return c;
	}
	if (xencons_console_device == NULL) {
		printf("xenconscn_getc(): not console\n");
		for (;;) {
			/* loop here instead of in ddb */
			HYPERVISOR_yield();
		}
		splx(s);
		return 0;
	}

	cons = xencons_interface->in_cons;
	prod = xencons_interface->in_prod;
	x86_lfence();
	while (cons == prod) {
		HYPERVISOR_yield();
		prod = xencons_interface->in_prod;
	}
	x86_lfence();
	c = xencons_interface->in[MASK_XENCONS_IDX(xencons_interface->in_cons,
	    xencons_interface->in)];
	x86_lfence();
	xencons_interface->in_cons = cons + 1;

	cn_check_magic(dev, c, xencons_cnm_state);

	splx(s);
	return c;
}

void
xenconscn_putc(dev_t dev, int c)
{
	int s;
	XENCONS_RING_IDX cons, prod;

	s = spltty();

	if (xen_start_info.flags & SIF_INITDOMAIN) {
		u_char buf[1];

		buf[0] = c;
		(void)HYPERVISOR_console_io(CONSOLEIO_write, 1, buf);
	} else {
		XENPRINTK(("xenconscn_putc(%c)\n", c));

		cons = xencons_interface->out_cons;
		prod = xencons_interface->out_prod;
		x86_lfence();
		while (prod == cons + sizeof(xencons_interface->out)) {
			cons = xencons_interface->out_cons;
			prod = xencons_interface->out_prod;
			x86_lfence();
		}
		xencons_interface->out[MASK_XENCONS_IDX(xencons_interface->out_prod,
		    xencons_interface->out)] = c;
		x86_lfence();
		xencons_interface->out_prod++;
		x86_lfence();
		hypervisor_notify_via_evtchn(xen_start_info.console_evtchn);

	}
	splx(s);
}

void
xenconscn_pollc(dev_t dev, int on)
{
	if (xencons_console_device)
		xencons_console_device->polling = on;
}

/*
 * Set line parameters.
 */
int
xenconsparam(struct tty *tp, struct termios *t)
{
	tp->t_ispeed = t->c_ispeed;
	tp->t_ospeed = t->c_ospeed;
	tp->t_cflag = t->c_cflag;

	return (0);
}


/* XXXXXXXX --- more cnmagic stuff. */
#define ENCODE_STATE(c, n) (short)(((c)&0x1ff)|(((n)&0x7f)<<9))

static unsigned short cn_magic[CNS_LEN];

/*
 * Initialize a cnm_state_t.
 */
void
cn_init_magic(cnm_state_t *cnm)
{
	cnm->cnm_state = 0;
	cnm->cnm_magic = cn_magic;
}

/*
 * Destroy a cnm_state_t.
 */
void
cn_destroy_magic(cnm_state_t *cnm)
{
	cnm->cnm_state = 0;
	cnm->cnm_magic = NULL;
}

/*
 * Translate a magic string to a state
 * machine table.
 */
int
cn_set_magic(char *magic)
{
	unsigned int i, c, n;
	unsigned short m[CNS_LEN];

	for (i=0; i<CNS_LEN; i++) {
		c = (*magic++)&0xff;
		n = *magic ? i+1 : CNS_TERM;
		switch (c) {
		case 0:
			/* End of string */
			if (i == 0) {
				/* empty string? */
				cn_magic[0] = 0;
#ifdef DEBUG
				printf("cn_set_magic(): empty!\n");
#endif
				return (0);
			}
			do {
				cn_magic[i] = m[i];
			} while (i--);
			return(0);
		case 0x27:
			/* Escape sequence */
			c = (*magic++)&0xff;
			n = *magic ? i+1 : CNS_TERM;
			switch (c) {
			case 0x27:
				break;
			case 0x01:
				/* BREAK */
				c = CNC_BREAK;
				break;
			case 0x02:
				/* NUL */
				c = 0;
				break;
			}
			/* FALLTHROUGH */
		default:
			/* Transition to the next state. */
#ifdef DEBUG
			if (!cold)
				printf("mag %d %x:%x\n", i, c, n);
#endif
			m[i] = ENCODE_STATE(c, n);
			break;
		}
	}
	return (EINVAL);
}

/*
 * Translate a state machine table back to
 * a magic string.
 */
int
cn_get_magic(char *magic, int maglen)
{
	unsigned int i, c;

	for (i=0; i<CNS_LEN; i++) {
		c = cn_magic[i];
		/* Translate a character */
		switch (CNS_MAGIC_VAL(c)) {
		case CNC_BREAK:
			*magic++ = 0x27;
			*magic++ = 0x01;
			break;
		case 0:
			*magic++ = 0x27;
			*magic++ = 0x02;
			break;
		case 0x27:
			*magic++ = 0x27;
			*magic++ = 0x27;
			break;
		default:
			*magic++ = (c&0x0ff);
			break;
		}
		/* Now go to the next state */
		i = CNS_MAGIC_NEXT(c);
		if (i == CNS_TERM || i == 0) {
			/* Either termination state or empty machine */
			*magic++ = 0;
			return (0);
		}
	}
	return (EINVAL);
}

