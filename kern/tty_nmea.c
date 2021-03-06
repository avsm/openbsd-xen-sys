/*	$OpenBSD: tty_nmea.c,v 1.20 2007/03/20 20:14:29 deraadt Exp $ */

/*
 * Copyright (c) 2006, 2007 Marc Balmer <mbalmer@openbsd.org>
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

/* A tty line discipline to decode NMEA 0183 data to get the time. */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/queue.h>
#include <sys/proc.h>
#include <sys/malloc.h>
#include <sys/sensors.h>
#include <sys/tty.h>
#include <sys/conf.h>
#include <sys/time.h>

#ifdef NMEA_DEBUG
#define DPRINTFN(n, x)	do { if (nmeadebug > (n)) printf x; } while (0)
int nmeadebug = 0;
#else
#define DPRINTFN(n, x)
#endif
#define DPRINTF(x)	DPRINTFN(0, x)

int	nmeaopen(dev_t, struct tty *);
int	nmeaclose(struct tty *, int);
int	nmeainput(int, struct tty *);
void	nmeaattach(int);

#define NMEAMAX	82
#define MAXFLDS	32

int nmea_count;	/* this is wrong, it should really be a SLIST */

struct nmea {
	char			cbuf[NMEAMAX];	/* receive buffer */
	struct ksensor		time;		/* the timedelta sensor */
	struct ksensordev	timedev;
	struct timespec		ts;		/* current timestamp */
	struct timespec		lts;		/* timestamp of last '$' seen */
	int64_t			gap;		/* gap between two sentences */
#ifdef NMEA_DEBUG
	int			gapno;
#endif
	int64_t			last;		/* last time rcvd */
	int			sync;		/* if 1, waiting for '$' */
	int			pos;		/* positon in rcv buffer */
	int			no_pps;		/* no PPS although requested */
	char			mode;		/* GPS mode */
};

/* NMEA decoding */
void	nmea_scan(struct nmea *, struct tty *);
void	nmea_gprmc(struct nmea *, struct tty *, char *fld[], int fldcnt);

/* date and time conversion */
int	nmea_date_to_nano(char *s, int64_t *nano);
int	nmea_time_to_nano(char *s, int64_t *nano);

void
nmeaattach(int dummy)
{
}

int
nmeaopen(dev_t dev, struct tty *tp)
{
	struct proc *p = curproc;
	struct nmea *np;
	int error;

	if (tp->t_line == NMEADISC)
		return ENODEV;
	if ((error = suser(p, 0)) != 0)
		return error;
	np = malloc(sizeof(struct nmea), M_DEVBUF, M_WAITOK);
	bzero(np, sizeof(*np));
	snprintf(np->timedev.xname, sizeof(np->timedev.xname), "nmea%d",
	    nmea_count++);
	np->time.status = SENSOR_S_UNKNOWN;
	np->time.type = SENSOR_TIMEDELTA;
	np->time.flags = SENSOR_FINVALID;
	sensor_attach(&np->timedev, &np->time);
	np->sync = 1;
	tp->t_sc = (caddr_t)np;

	error = linesw[TTYDISC].l_open(dev, tp);
	if (error) {
		free(np, M_DEVBUF);
		tp->t_sc = NULL;
	} else
		sensordev_install(&np->timedev);
	return error;
}

int
nmeaclose(struct tty *tp, int flags)
{
	struct nmea *np = (struct nmea *)tp->t_sc;

	tp->t_line = TTYDISC;	/* switch back to termios */
	sensordev_deinstall(&np->timedev);
	free(np, M_DEVBUF);
	tp->t_sc = NULL;
	nmea_count--;
	return linesw[TTYDISC].l_close(tp, flags);
}

/* collect NMEA sentence from tty */
int
nmeainput(int c, struct tty *tp)
{
	struct nmea *np = (struct nmea *)tp->t_sc;
	struct timespec ts;
	int64_t gap;
	long tmin, tmax;

	switch (c) {
	case '$':
		nanotime(&ts);
		np->pos = np->sync = 0;
		gap = (ts.tv_sec * 1000000000LL + ts.tv_nsec) -
		    (np->lts.tv_sec * 1000000000LL + np->lts.tv_nsec);

		np->lts.tv_sec = ts.tv_sec;
		np->lts.tv_nsec = ts.tv_nsec;

		if (gap <= np->gap)
			break;

		np->ts.tv_sec = ts.tv_sec;
		np->ts.tv_nsec = ts.tv_nsec;

#ifdef NMEA_DEBUG
		if (nmeadebug > 0) {
			linesw[TTYDISC].l_rint('[', tp);
			linesw[TTYDISC].l_rint('0' + np->gapno++, tp);
			linesw[TTYDISC].l_rint(']', tp);
		}
#endif
		np->gap = gap;
	
		/*
		 * If a tty timestamp is available, make sure its value is
		 * reasonable by comparing against the timestamp just taken.
		 * If they differ by more than 2 seconds, assume no PPS signal
		 * is present, note the fact, and keep using the timestamp
		 * value.  When this happens, the sensor state is set to
		 * CRITICAL later when the GPRMC sentence is decoded.
		 */
		if (tp->t_flags & (TS_TSTAMPDCDSET | TS_TSTAMPDCDCLR |
		    TS_TSTAMPCTSSET | TS_TSTAMPCTSCLR)) {
			tmax = lmax(np->ts.tv_sec, tp->t_tv.tv_sec);
			tmin = lmin(np->ts.tv_sec, tp->t_tv.tv_sec);
			if (tmax - tmin > 1)
				np->no_pps = 1;
			else {
				np->ts.tv_sec = tp->t_tv.tv_sec;
				np->ts.tv_nsec = tp->t_tv.tv_usec *
				    1000L;
				np->no_pps = 0;
			}
		}
		break;
	case '\r':
	case '\n':
		if (!np->sync) {
			np->cbuf[np->pos] = '\0';
			nmea_scan(np, tp);
			np->sync = 1;
		}
		break;
	default:
		if (!np->sync && np->pos < (NMEAMAX - 1))
			np->cbuf[np->pos++] = c;
		break;
	}
	/* pass data to termios */
	return linesw[TTYDISC].l_rint(c, tp);
}

/* Scan the NMEA sentence just received */
void
nmea_scan(struct nmea *np, struct tty *tp)
{
	int fldcnt = 0, cksum = 0, msgcksum, n;
	char *fld[MAXFLDS], *cs;

	/* split into fields and calculate the checksum */
	fld[fldcnt++] = &np->cbuf[0];	/* message type */
	for (cs = NULL, n = 0; n < np->pos && cs == NULL; n++) {
		switch (np->cbuf[n]) {
		case '*':
			np->cbuf[n] = '\0';
			cs = &np->cbuf[n + 1];
			break;
		case ',':
			if (fldcnt < MAXFLDS) {
				cksum ^= np->cbuf[n];
				np->cbuf[n] = '\0';
				fld[fldcnt++] = &np->cbuf[n + 1];
			} else {
				DPRINTF(("nr of fields in %s sentence exceeds "
				    "maximum of %d\n", fld[0], MAXFLDS));
				return;
			}
			break;
		default:
			cksum ^= np->cbuf[n];
		}
	}

	/* if we have a checksum, verify it */
	if (cs != NULL) {
		msgcksum = 0;
		while (*cs) {
			if ((*cs >= '0' && *cs <= '9') ||
			    (*cs >= 'A' && *cs <= 'F')) {
				if (msgcksum)
					msgcksum <<= 4;
				if (*cs >= '0' && *cs<= '9')
					msgcksum += *cs - '0';
				else if (*cs >= 'A' && *cs <= 'F')
					msgcksum += 10 + *cs - 'A';
				cs++;
			} else {
				DPRINTF(("bad char %c in checksum\n", *cs));
				return;
			}
		}
		if (msgcksum != cksum) {
			DPRINTF(("checksum mismatch\n"));
			return;
		}
	}

	/* check message type */
	if (!strcmp(fld[0], "GPRMC"))
		nmea_gprmc(np, tp, fld, fldcnt);
}

/* Decode the recommended minimum specific GPS/TRANSIT data */
void
nmea_gprmc(struct nmea *np, struct tty *tp, char *fld[], int fldcnt)
{
	int64_t date_nano, time_nano, nmea_now;

	if (fldcnt != 12 && fldcnt != 13) {
		DPRINTF(("gprmc: field count mismatch, %d\n", fldcnt));
		return;
	}
	if (nmea_time_to_nano(fld[1], &time_nano)) {
		DPRINTF(("gprmc: illegal time, %s\n", fld[1]));
		return;
	}
	if (nmea_date_to_nano(fld[9], &date_nano)) {
		DPRINTF(("gprmc: illegal date, %s\n", fld[9]));
		return;
	}
	nmea_now = date_nano + time_nano;
	if (nmea_now <= np->last) {
		DPRINTF(("gprmc: time not monotonically increasing\n"));
		return;
	}
	np->last = nmea_now;
	np->gap = 0LL;
#ifdef NMEA_DEBUG
	np->gapno = 0;
	if (nmeadebug > 0) {
		linesw[TTYDISC].l_rint('[', tp);
		linesw[TTYDISC].l_rint('C', tp);
		linesw[TTYDISC].l_rint(']', tp);
	}
#endif

	np->time.value = np->ts.tv_sec * 1000000000LL +
	    np->ts.tv_nsec - nmea_now;
	np->time.tv.tv_sec = np->ts.tv_sec;
	np->time.tv.tv_usec = np->ts.tv_nsec / 1000L;
	if (np->time.status == SENSOR_S_UNKNOWN) {
		np->time.status = SENSOR_S_OK;
		np->time.flags &= ~SENSOR_FINVALID;
		if (fldcnt != 13)
			strlcpy(np->time.desc, "GPS", sizeof(np->time.desc));
	}
	if (fldcnt == 13 && *fld[12] != np->mode) {
		np->mode = *fld[12];
		switch (np->mode) {
		case 'S':
			strlcpy(np->time.desc, "GPS simulated",
			    sizeof(np->time.desc));
			break;
		case 'E':
			strlcpy(np->time.desc, "GPS estimated",
			    sizeof(np->time.desc));
			break;
		case 'A':
			strlcpy(np->time.desc, "GPS autonomous",
			    sizeof(np->time.desc));
			break;
		case 'D':
			strlcpy(np->time.desc, "GPS differential",
			    sizeof(np->time.desc));
			break;
		case 'N':
			strlcpy(np->time.desc, "GPS not valid",
			    sizeof(np->time.desc));
			break;
		default:
			strlcpy(np->time.desc, "GPS unknown",
			    sizeof(np->time.desc));
			DPRINTF(("gprmc: unknown mode '%c'\n", np->mode));
		}
	}
	switch (*fld[2]) {
	case 'A':
		np->time.status = SENSOR_S_OK;
		break;
	case 'V':
		np->time.status = SENSOR_S_WARN;
		break;
	default:
		DPRINTF(("gprmc: unknown warning indication\n"));
	}

	/*
	 * If tty timestamping is requested, but not PPS signal is present, set
	 * the sensor state to CRITICAL.
	 */
	if (np->no_pps)
		np->time.status = SENSOR_S_CRIT;
}

/*
 * Convert a NMEA 0183 formatted date string to seconds since the epoch.
 * The string must be of the form DDMMYY.
 * Return 0 on success, -1 if illegal characters are encountered.
 */
int
nmea_date_to_nano(char *s, int64_t *nano)
{
	struct clock_ymdhms ymd;
	time_t secs;
	char *p;
	int n;

	/* make sure the input contains only numbers and is six digits long */
	for (n = 0, p = s; n < 6 && *p && *p >= '0' && *p <= '9'; n++, p++)
		;
	if (n != 6 || (*p != '\0'))
		return -1;

	ymd.dt_year = 2000 + (s[4] - '0') * 10 + (s[5] - '0');
	ymd.dt_mon = (s[2] - '0') * 10 + (s[3] - '0');
	ymd.dt_day = (s[0] - '0') * 10 + (s[1] - '0');
	ymd.dt_hour = ymd.dt_min = ymd.dt_sec = 0;

	secs = clock_ymdhms_to_secs(&ymd);
	*nano = secs * 1000000000LL;
	return 0;
}

/*
 * Convert NMEA 0183 formatted time string to nanoseconds since midnight.
 * The string must be of the form HHMMSS[.[sss]] (e.g. 143724 or 143723.615).
 * Return 0 on success, -1 if illegal characters are encountered.
 */
int
nmea_time_to_nano(char *s, int64_t *nano)
{
	long fac = 36000L, div = 6L, secs = 0L, frac = 0L;
	char ul = '2';
	int n;

	for (n = 0, secs = 0; fac && *s && *s >= '0' && *s <= ul; s++, n++) {
		secs += (*s - '0') * fac;
		div = 16 - div;
		fac /= div;
		switch (n) {
		case 0:
			if (*s <= '1')
				ul = '9';
			else
				ul = '3';
			break;
		case 1:
		case 3:
			ul = '5';
			break;
		case 2:
		case 4:
			ul = '9';
			break;
		}
	}
	if (fac)
		return -1;

	/* Handle the fractions of a second, up to a maximum of 6 digits. */
	div = 1L;
	if (*s == '.') {
		for (++s; div < 1000000 && *s && *s >= '0' && *s <= '9'; s++) {
			frac *= 10;
			frac += (*s - '0');
			div *= 10;
		}
	}

	if (*s != '\0')
		return -1;

	*nano = secs * 1000000000LL + (int64_t)frac * (1000000000 / div);
	return 0;
}
