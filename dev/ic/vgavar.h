/* $OpenBSD: vgavar.h,v 1.7 2002/03/14 03:16:05 millert Exp $ */
/* $NetBSD: vgavar.h,v 1.4 2000/06/17 07:11:50 soda Exp $ */

/*
 * Copyright (c) 1995, 1996 Carnegie-Mellon University.
 * All rights reserved.
 *
 * Author: Chris G. Demetriou
 * 
 * Permission to use, copy, modify and distribute this software and
 * its documentation is hereby granted, provided that both the copyright
 * notice and this permission notice appear in all copies of the
 * software, derivative works or modified versions, and any portions
 * thereof, and that both notices appear in supporting documentation.
 * 
 * CARNEGIE MELLON ALLOWS FREE USE OF THIS SOFTWARE IN ITS "AS IS" 
 * CONDITION.  CARNEGIE MELLON DISCLAIMS ANY LIABILITY OF ANY KIND 
 * FOR ANY DAMAGES WHATSOEVER RESULTING FROM THE USE OF THIS SOFTWARE.
 * 
 * Carnegie Mellon requests users of this software to return to
 *
 *  Software Distribution Coordinator  or  Software.Distribution@CS.CMU.EDU
 *  School of Computer Science
 *  Carnegie Mellon University
 *  Pittsburgh PA 15213-3890
 *
 * any improvements or extensions that they make and grant Carnegie the
 * rights to redistribute these changes.
 */

#include <sys/timeout.h>

struct vga_handle {
	struct pcdisplay_handle vh_ph;
	bus_space_handle_t vh_ioh_vga, vh_allmemh;
	int vh_mono;
};
#define vh_iot vh_ph.ph_iot
#define vh_memt vh_ph.ph_memt
#define vh_ioh_6845 vh_ph.ph_ioh_6845
#define vh_memh vh_ph.ph_memh

struct vga_config {
	struct vga_handle hdl;

	struct device *vc_softc;
	int vc_type;
	int nscreens;
	LIST_HEAD(, vgascreen) screens;
	struct vgascreen *active; /* current display */
	const struct wsscreen_descr *currenttype;
	int currentfontset1, currentfontset2;

	struct vgafont *vc_fonts[8];

	struct vgascreen *wantedscreen;
	void (*switchcb)(void *, int, int);
	void *switchcbarg;

	paddr_t (*vc_mmap)(void *, off_t, int);

	struct timeout vc_switch_timeout;
};

static inline u_int8_t _vga_attr_read(struct vga_handle *, int);
static inline void _vga_attr_write(struct vga_handle *, int, u_int8_t);
static inline u_int8_t _vga_ts_read(struct vga_handle *, int);
static inline void _vga_ts_write(struct vga_handle *, int, u_int8_t);
static inline u_int8_t _vga_gdc_read(struct vga_handle *, int);
static inline void _vga_gdc_write(struct vga_handle *, int, u_int8_t);

static inline u_int8_t _vga_attr_read(vh, reg)
	struct vga_handle *vh;
	int reg;
{
	u_int8_t res;

	/* reset state */
	(void) bus_space_read_1(vh->vh_iot, vh->vh_ioh_6845, 10);

	bus_space_write_1(vh->vh_iot, vh->vh_ioh_vga, VGA_ATC_INDEX, reg);
	res = bus_space_read_1(vh->vh_iot, vh->vh_ioh_vga, VGA_ATC_DATAR);

	/* reset state XXX unneeded? */
	(void) bus_space_read_1(vh->vh_iot, vh->vh_ioh_6845, 10);

	/* enable */
	bus_space_write_1(vh->vh_iot, vh->vh_ioh_vga, 0, 0x20);

	return (res);
}

static inline void _vga_attr_write(vh, reg, val)
	struct vga_handle *vh;
	int reg;
	u_int8_t val;
{
	/* reset state */
	(void) bus_space_read_1(vh->vh_iot, vh->vh_ioh_6845, 10);

	bus_space_write_1(vh->vh_iot, vh->vh_ioh_vga, VGA_ATC_INDEX, reg);
	bus_space_write_1(vh->vh_iot, vh->vh_ioh_vga, VGA_ATC_DATAW, val);

	/* reset state XXX unneeded? */
	(void) bus_space_read_1(vh->vh_iot, vh->vh_ioh_6845, 10);

	/* enable */
	bus_space_write_1(vh->vh_iot, vh->vh_ioh_vga, 0, 0x20);
}

static inline u_int8_t _vga_ts_read(vh, reg)
	struct vga_handle *vh;
	int reg;
{
	bus_space_write_1(vh->vh_iot, vh->vh_ioh_vga, VGA_TS_INDEX, reg);
	return (bus_space_read_1(vh->vh_iot, vh->vh_ioh_vga, VGA_TS_DATA));
}

static inline void _vga_ts_write(vh, reg, val)
	struct vga_handle *vh;
	int reg;
	u_int8_t val;
{
	bus_space_write_1(vh->vh_iot, vh->vh_ioh_vga, VGA_TS_INDEX, reg);
	bus_space_write_1(vh->vh_iot, vh->vh_ioh_vga, VGA_TS_DATA, val);
}

static inline u_int8_t _vga_gdc_read(vh, reg)
	struct vga_handle *vh;
	int reg;
{
	bus_space_write_1(vh->vh_iot, vh->vh_ioh_vga, VGA_GDC_INDEX, reg);
	return (bus_space_read_1(vh->vh_iot, vh->vh_ioh_vga, VGA_GDC_DATA));
}

static inline void _vga_gdc_write(vh, reg, val)
	struct vga_handle *vh;
	int reg;
	u_int8_t val;
{
	bus_space_write_1(vh->vh_iot, vh->vh_ioh_vga, VGA_GDC_INDEX, reg);
	bus_space_write_1(vh->vh_iot, vh->vh_ioh_vga, VGA_GDC_DATA, val);
}

#define vga_attr_read(vh, reg) \
	_vga_attr_read(vh, offsetof(struct reg_vgaattr, reg))
#define vga_attr_write(vh, reg, val) \
	_vga_attr_write(vh, offsetof(struct reg_vgaattr, reg), val)
#define vga_ts_read(vh, reg) \
	_vga_ts_read(vh, offsetof(struct reg_vgats, reg))
#define vga_ts_write(vh, reg, val) \
	_vga_ts_write(vh, offsetof(struct reg_vgats, reg), val)
#define vga_gdc_read(vh, reg) \
	_vga_gdc_read(vh, offsetof(struct reg_vgagdc, reg))
#define vga_gdc_write(vh, reg, val) \
	_vga_gdc_write(vh, offsetof(struct reg_vgagdc, reg), val)

#define vga_6845_read(vh, reg) \
	pcdisplay_6845_read(&(vh)->vh_ph, reg)
#define vga_6845_write(vh, reg, val) \
	pcdisplay_6845_write(&(vh)->vh_ph, reg, val)

int	vga_common_probe(bus_space_tag_t, bus_space_tag_t);
void	vga_common_attach(struct device *, bus_space_tag_t,
			       bus_space_tag_t, int);
void	vga_extended_attach(struct device *, bus_space_tag_t,
    bus_space_tag_t, int, paddr_t (*)(void *, off_t, int));
int	vga_is_console(bus_space_tag_t, int);
int	vga_cnattach(bus_space_tag_t, bus_space_tag_t, int, int);

struct wsscreen_descr;
void vga_loadchars(struct vga_handle *, int, int, int, int, char *);
void vga_setfontset(struct vga_handle *, int, int);
void vga_setscreentype(struct vga_handle *,
		       const struct wsscreen_descr *);
#if NVGA_PCI > 0
int vga_pci_ioctl(void *, u_long, caddr_t, int, struct proc *); 
#endif
