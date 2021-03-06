/*	$OpenBSD: ar5212var.h,v 1.12 2007/03/05 15:13:26 reyk Exp $	*/

/*
 * Copyright (c) 2004, 2005, 2006, 2007 Reyk Floeter <reyk@openbsd.org>
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

/*
 * Specific definitions for the Atheros AR5001 Wireless LAN chipset
 * (AR5212/AR5311).
 */

#ifndef _AR5K_AR5212_VAR_H
#define _AR5K_AR5212_VAR_H

#include <dev/ic/ar5xxx.h>

/*
 * Define a "magic" code for the AR5212 (the HAL layer wants it)
 */

#define AR5K_AR5212_MAGIC		0x0000145c /* 5212 */
#define AR5K_AR5212_TX_NUM_QUEUES	10

#if BYTE_ORDER == BIG_ENDIAN
#define AR5K_AR5212_INIT_CFG	(					\
	AR5K_AR5212_CFG_SWTD | AR5K_AR5212_CFG_SWRD			\
)
#else
#define AR5K_AR5212_INIT_CFG	0x00000000
#endif

/*
 * Internal RX/TX descriptor structures
 * (rX: reserved fields possibily used by future versions of the ar5k chipset)
 */

struct ar5k_ar5212_rx_desc {
	/*
	 * RX control word 0
	 */
	u_int32_t	rx_control_0;

#define AR5K_AR5212_DESC_RX_CTL0			0x00000000

	/*
	 * RX control word 1
	 */
	u_int32_t	rx_control_1;

#define AR5K_AR5212_DESC_RX_CTL1_BUF_LEN		0x00000fff
#define AR5K_AR5212_DESC_RX_CTL1_INTREQ			0x00002000
} __packed;

struct ar5k_ar5212_rx_status {
	/*
	 * RX status word 0
	 */
	u_int32_t	rx_status_0;

#define AR5K_AR5212_DESC_RX_STATUS0_DATA_LEN		0x00000fff
#define AR5K_AR5212_DESC_RX_STATUS0_MORE		0x00001000
#define AR5K_AR5212_DESC_RX_STATUS0_DECOMP_CRC_ERROR	0x00002000
#define AR5K_AR5212_DESC_RX_STATUS0_RECEIVE_RATE	0x000f8000
#define AR5K_AR5212_DESC_RX_STATUS0_RECEIVE_RATE_S	15
#define AR5K_AR5212_DESC_RX_STATUS0_RECEIVE_SIGNAL	0x0ff00000
#define AR5K_AR5212_DESC_RX_STATUS0_RECEIVE_SIGNAL_S	20
#define AR5K_AR5212_DESC_RX_STATUS0_RECEIVE_ANTENNA	0xf0000000
#define AR5K_AR5212_DESC_RX_STATUS0_RECEIVE_ANTENNA_S	28

	/*
	 * RX status word 1
	 */
	u_int32_t	rx_status_1;

#define AR5K_AR5212_DESC_RX_STATUS1_DONE		0x00000001
#define AR5K_AR5212_DESC_RX_STATUS1_FRAME_RECEIVE_OK	0x00000002
#define AR5K_AR5212_DESC_RX_STATUS1_CRC_ERROR		0x00000004
#define AR5K_AR5212_DESC_RX_STATUS1_DECRYPT_CRC_ERROR	0x00000008
#define AR5K_AR5212_DESC_RX_STATUS1_PHY_ERROR		0x00000010
#define AR5K_AR5212_DESC_RX_STATUS1_MIC_ERROR		0x00000020
#define AR5K_AR5212_DESC_RX_STATUS1_KEY_INDEX_VALID	0x00000100
#define AR5K_AR5212_DESC_RX_STATUS1_KEY_INDEX		0x0000fe00
#define AR5K_AR5212_DESC_RX_STATUS1_KEY_INDEX_S		9
#define AR5K_AR5212_DESC_RX_STATUS1_RECEIVE_TIMESTAMP	0x7fff0000
#define AR5K_AR5212_DESC_RX_STATUS1_RECEIVE_TIMESTAMP_S	16
#define AR5K_AR5212_DESC_RX_STATUS1_KEY_CACHE_MISS	0x80000000
} __packed;

struct ar5k_ar5212_rx_error {
	/*
	 * RX error word 0
	 */
	u_int32_t	rx_error_0;

#define AR5K_AR5212_DESC_RX_ERROR0			0x00000000

	/*
	 * RX error word 1
	 */
	u_int32_t	rx_error_1;

#define AR5K_AR5212_DESC_RX_ERROR1_PHY_ERROR_CODE	0x0000ff00
#define AR5K_AR5212_DESC_RX_ERROR1_PHY_ERROR_CODE_S	8
} __packed;

#define AR5K_AR5212_DESC_RX_PHY_ERROR_NONE		0x00
#define AR5K_AR5212_DESC_RX_PHY_ERROR_TIMING		0x20
#define AR5K_AR5212_DESC_RX_PHY_ERROR_PARITY		0x40
#define AR5K_AR5212_DESC_RX_PHY_ERROR_RATE		0x60
#define AR5K_AR5212_DESC_RX_PHY_ERROR_LENGTH		0x80
#define AR5K_AR5212_DESC_RX_PHY_ERROR_64QAM		0xa0
#define AR5K_AR5212_DESC_RX_PHY_ERROR_SERVICE		0xc0
#define AR5K_AR5212_DESC_RX_PHY_ERROR_TRANSMITOVR	0xe0

struct ar5k_ar5212_tx_desc {
	/*
	 * TX control word 0
	 */
	u_int32_t	tx_control_0;

#define AR5K_AR5212_DESC_TX_CTL0_FRAME_LEN		0x00000fff
#define AR5K_AR5212_DESC_TX_CTL0_XMIT_POWER		0x003f0000
#define AR5K_AR5212_DESC_TX_CTL0_XMIT_POWER_S		16
#define AR5K_AR5212_DESC_TX_CTL0_RTSENA			0x00400000
#define AR5K_AR5212_DESC_TX_CTL0_VEOL			0x00800000
#define AR5K_AR5212_DESC_TX_CTL0_CLRDMASK		0x01000000
#define AR5K_AR5212_DESC_TX_CTL0_ANT_MODE_XMIT		0x1e000000
#define AR5K_AR5212_DESC_TX_CTL0_ANT_MODE_XMIT_S	25
#define AR5K_AR5212_DESC_TX_CTL0_INTREQ			0x20000000
#define AR5K_AR5212_DESC_TX_CTL0_ENCRYPT_KEY_VALID	0x40000000
#define AR5K_AR5212_DESC_TX_CTL0_CTSENA			0x80000000

	/*
	 * TX control word 1
	 */
	u_int32_t	tx_control_1;

#define AR5K_AR5212_DESC_TX_CTL1_BUF_LEN		0x00000fff
#define AR5K_AR5212_DESC_TX_CTL1_MORE			0x00001000
#define AR5K_AR5212_DESC_TX_CTL1_ENCRYPT_KEY_INDEX	0x000fe000
#define AR5K_AR5212_DESC_TX_CTL1_ENCRYPT_KEY_INDEX_S	13
#define AR5K_AR5212_DESC_TX_CTL1_FRAME_TYPE		0x00f00000
#define AR5K_AR5212_DESC_TX_CTL1_FRAME_TYPE_S		20
#define AR5K_AR5212_DESC_TX_CTL1_NOACK			0x01000000
#define AR5K_AR5212_DESC_TX_CTL1_COMP_PROC		0x06000000
#define AR5K_AR5212_DESC_TX_CTL1_COMP_PROC_S		25
#define AR5K_AR5212_DESC_TX_CTL1_COMP_IV_LEN		0x18000000
#define AR5K_AR5212_DESC_TX_CTL1_COMP_IV_LEN_S		27
#define AR5K_AR5212_DESC_TX_CTL1_COMP_ICV_LEN		0x60000000
#define AR5K_AR5212_DESC_TX_CTL1_COMP_ICV_LEN_S		29

	/*
	 * TX control word 2
	 */
	u_int32_t	tx_control_2;

#define AR5K_AR5212_DESC_TX_CTL2_RTS_DURATION		0x00007fff
#define AR5K_AR5212_DESC_TX_CTL2_DURATION_UPDATE_ENABLE	0x00008000
#define AR5K_AR5212_DESC_TX_CTL2_XMIT_TRIES0		0x000f0000
#define AR5K_AR5212_DESC_TX_CTL2_XMIT_TRIES0_S		16
#define AR5K_AR5212_DESC_TX_CTL2_XMIT_TRIES1		0x00f00000
#define AR5K_AR5212_DESC_TX_CTL2_XMIT_TRIES1_S		20
#define AR5K_AR5212_DESC_TX_CTL2_XMIT_TRIES2		0x0f000000
#define AR5K_AR5212_DESC_TX_CTL2_XMIT_TRIES2_S		24
#define AR5K_AR5212_DESC_TX_CTL2_XMIT_TRIES3		0xf0000000
#define AR5K_AR5212_DESC_TX_CTL2_XMIT_TRIES3_S		28

	/*
	 * TX control word 3
	 */
	u_int32_t	tx_control_3;

#define AR5K_AR5212_DESC_TX_CTL3_XMIT_RATE0		0x0000001f
#define AR5K_AR5212_DESC_TX_CTL3_XMIT_RATE1		0x000003e0
#define AR5K_AR5212_DESC_TX_CTL3_XMIT_RATE1_S		5
#define AR5K_AR5212_DESC_TX_CTL3_XMIT_RATE2		0x00007c00
#define AR5K_AR5212_DESC_TX_CTL3_XMIT_RATE2_S		10
#define AR5K_AR5212_DESC_TX_CTL3_XMIT_RATE3		0x000f8000
#define AR5K_AR5212_DESC_TX_CTL3_XMIT_RATE3_S		15
#define AR5K_AR5212_DESC_TX_CTL3_RTS_CTS_RATE		0x01f00000
#define AR5K_AR5212_DESC_TX_CTL3_RTS_CTS_RATE_S		20
} __packed;

struct ar5k_ar5212_tx_status {
	/*
	 * TX status word 0
	 */
	u_int32_t	tx_status_0;

#define AR5K_AR5212_DESC_TX_STATUS0_FRAME_XMIT_OK	0x00000001
#define AR5K_AR5212_DESC_TX_STATUS0_EXCESSIVE_RETRIES	0x00000002
#define AR5K_AR5212_DESC_TX_STATUS0_FIFO_UNDERRUN	0x00000004
#define AR5K_AR5212_DESC_TX_STATUS0_FILTERED		0x00000008
#define AR5K_AR5212_DESC_TX_STATUS0_RTS_FAIL_COUNT	0x000000f0
#define AR5K_AR5212_DESC_TX_STATUS0_RTS_FAIL_COUNT_S	4
#define AR5K_AR5212_DESC_TX_STATUS0_DATA_FAIL_COUNT	0x00000f00
#define AR5K_AR5212_DESC_TX_STATUS0_DATA_FAIL_COUNT_S	8
#define AR5K_AR5212_DESC_TX_STATUS0_VIRT_COLL_COUNT	0x0000f000
#define AR5K_AR5212_DESC_TX_STATUS0_VIRT_COLL_COUNT_S	12
#define AR5K_AR5212_DESC_TX_STATUS0_SEND_TIMESTAMP	0xffff0000
#define AR5K_AR5212_DESC_TX_STATUS0_SEND_TIMESTAMP_S	16

	/*
	 * TX status word 1
	 */
	u_int32_t	tx_status_1;

#define AR5K_AR5212_DESC_TX_STATUS1_DONE		0x00000001
#define AR5K_AR5212_DESC_TX_STATUS1_SEQ_NUM		0x00001ffe
#define AR5K_AR5212_DESC_TX_STATUS1_SEQ_NUM_S		1
#define AR5K_AR5212_DESC_TX_STATUS1_ACK_SIG_STRENGTH	0x001fe000
#define AR5K_AR5212_DESC_TX_STATUS1_ACK_SIG_STRENGTH_S	13
#define AR5K_AR5212_DESC_TX_STATUS1_FINAL_TS_INDEX	0x00600000
#define AR5K_AR5212_DESC_TX_STATUS1_FINAL_TS_INDEX_S	21
#define AR5K_AR5212_DESC_TX_STATUS1_COMP_SUCCESS	0x00800000
#define AR5K_AR5212_DESC_TX_STATUS1_XMIT_ANTENNA	0x01000000
} __packed;

/*
 * Public function prototypes
 */
extern ar5k_attach_t ar5k_ar5212_attach;

/*
 * Initial register values which have to be loaded into the
 * card at boot time and after each reset.
 */

struct ar5k_ar5212_ini {
	u_int8_t	ini_flags;
	u_int16_t	ini_register;
	u_int32_t	ini_value;

#define AR5K_INI_FLAG_511X	0x00
#define	AR5K_INI_FLAG_5111	0x01
#define AR5K_INI_FLAG_5112	0x02
#define AR5K_INI_FLAG_BOTH	(AR5K_INI_FLAG_5111 | AR5K_INI_FLAG_5112)
};

#define AR5K_AR5212_INI {						\
	{ AR5K_INI_FLAG_BOTH, 0x000c, 0x00000000 },			\
	{ AR5K_INI_FLAG_BOTH, 0x0034, 0x00000005 },			\
	{ AR5K_INI_FLAG_BOTH, 0x0040, 0x00000000 },			\
	{ AR5K_INI_FLAG_BOTH, 0x0044, 0x00000008 },			\
	{ AR5K_INI_FLAG_BOTH, 0x0048, 0x00000008 },			\
	{ AR5K_INI_FLAG_BOTH, 0x004c, 0x00000010 },			\
	{ AR5K_INI_FLAG_BOTH, 0x0050, 0x00000000 },			\
	{ AR5K_INI_FLAG_BOTH, 0x0054, 0x0000001f },			\
	{ AR5K_INI_FLAG_BOTH, 0x0800, 0x00000000 },			\
	{ AR5K_INI_FLAG_BOTH, 0x0804, 0x00000000 },			\
	{ AR5K_INI_FLAG_BOTH, 0x0808, 0x00000000 },			\
	{ AR5K_INI_FLAG_BOTH, 0x080c, 0x00000000 },			\
	{ AR5K_INI_FLAG_BOTH, 0x0810, 0x00000000 },			\
	{ AR5K_INI_FLAG_BOTH, 0x0814, 0x00000000 },			\
	{ AR5K_INI_FLAG_BOTH, 0x0818, 0x00000000 },			\
	{ AR5K_INI_FLAG_BOTH, 0x081c, 0x00000000 },			\
	{ AR5K_INI_FLAG_BOTH, 0x0820, 0x00000000 },			\
	{ AR5K_INI_FLAG_BOTH, 0x0824, 0x00000000 },			\
	{ AR5K_INI_FLAG_BOTH, 0x1230, 0x00000000 },			\
	{ AR5K_INI_FLAG_BOTH, 0x1270, 0x00000000 },			\
	{ AR5K_INI_FLAG_BOTH, 0x1038, 0x00000000 },			\
	{ AR5K_INI_FLAG_BOTH, 0x1078, 0x00000000 },			\
	{ AR5K_INI_FLAG_BOTH, 0x10b8, 0x00000000 },			\
	{ AR5K_INI_FLAG_BOTH, 0x10f8, 0x00000000 },			\
	{ AR5K_INI_FLAG_BOTH, 0x1138, 0x00000000 },			\
	{ AR5K_INI_FLAG_BOTH, 0x1178, 0x00000000 },			\
	{ AR5K_INI_FLAG_BOTH, 0x11b8, 0x00000000 },			\
	{ AR5K_INI_FLAG_BOTH, 0x11f8, 0x00000000 },			\
	{ AR5K_INI_FLAG_BOTH, 0x1238, 0x00000000 },			\
	{ AR5K_INI_FLAG_BOTH, 0x1278, 0x00000000 },			\
	{ AR5K_INI_FLAG_BOTH, 0x12b8, 0x00000000 },			\
	{ AR5K_INI_FLAG_BOTH, 0x12f8, 0x00000000 },			\
	{ AR5K_INI_FLAG_BOTH, 0x1338, 0x00000000 },			\
	{ AR5K_INI_FLAG_BOTH, 0x1378, 0x00000000 },			\
	{ AR5K_INI_FLAG_BOTH, 0x13b8, 0x00000000 },			\
	{ AR5K_INI_FLAG_BOTH, 0x13f8, 0x00000000 },			\
	{ AR5K_INI_FLAG_BOTH, 0x1438, 0x00000000 },			\
	{ AR5K_INI_FLAG_BOTH, 0x1478, 0x00000000 },			\
	{ AR5K_INI_FLAG_BOTH, 0x14b8, 0x00000000 },			\
	{ AR5K_INI_FLAG_BOTH, 0x14f8, 0x00000000 },			\
	{ AR5K_INI_FLAG_BOTH, 0x1538, 0x00000000 },			\
	{ AR5K_INI_FLAG_BOTH, 0x1578, 0x00000000 },			\
	{ AR5K_INI_FLAG_BOTH, 0x15b8, 0x00000000 },			\
	{ AR5K_INI_FLAG_BOTH, 0x15f8, 0x00000000 },			\
	{ AR5K_INI_FLAG_BOTH, 0x1638, 0x00000000 },			\
	{ AR5K_INI_FLAG_BOTH, 0x1678, 0x00000000 },			\
	{ AR5K_INI_FLAG_BOTH, 0x16b8, 0x00000000 },			\
	{ AR5K_INI_FLAG_BOTH, 0x16f8, 0x00000000 },			\
	{ AR5K_INI_FLAG_BOTH, 0x1738, 0x00000000 },			\
	{ AR5K_INI_FLAG_BOTH, 0x1778, 0x00000000 },			\
	{ AR5K_INI_FLAG_BOTH, 0x17b8, 0x00000000 },			\
	{ AR5K_INI_FLAG_BOTH, 0x17f8, 0x00000000 },			\
	{ AR5K_INI_FLAG_BOTH, 0x103c, 0x00000000 },			\
	{ AR5K_INI_FLAG_BOTH, 0x107c, 0x00000000 },			\
	{ AR5K_INI_FLAG_BOTH, 0x10bc, 0x00000000 },			\
	{ AR5K_INI_FLAG_BOTH, 0x10fc, 0x00000000 },			\
	{ AR5K_INI_FLAG_BOTH, 0x113c, 0x00000000 },			\
	{ AR5K_INI_FLAG_BOTH, 0x117c, 0x00000000 },			\
	{ AR5K_INI_FLAG_BOTH, 0x11bc, 0x00000000 },			\
	{ AR5K_INI_FLAG_BOTH, 0x11fc, 0x00000000 },			\
	{ AR5K_INI_FLAG_BOTH, 0x123c, 0x00000000 },			\
	{ AR5K_INI_FLAG_BOTH, 0x127c, 0x00000000 },			\
	{ AR5K_INI_FLAG_BOTH, 0x12bc, 0x00000000 },			\
	{ AR5K_INI_FLAG_BOTH, 0x12fc, 0x00000000 },			\
	{ AR5K_INI_FLAG_BOTH, 0x133c, 0x00000000 },			\
	{ AR5K_INI_FLAG_BOTH, 0x137c, 0x00000000 },			\
	{ AR5K_INI_FLAG_BOTH, 0x13bc, 0x00000000 },			\
	{ AR5K_INI_FLAG_BOTH, 0x13fc, 0x00000000 },			\
	{ AR5K_INI_FLAG_BOTH, 0x143c, 0x00000000 },			\
	{ AR5K_INI_FLAG_BOTH, 0x147c, 0x00000000 },			\
	{ AR5K_INI_FLAG_BOTH, 0x8004, 0x00000000 },			\
	{ AR5K_INI_FLAG_BOTH, 0x8008, 0x00000000 },			\
	{ AR5K_INI_FLAG_BOTH, 0x800c, 0x00000000 },			\
	{ AR5K_INI_FLAG_BOTH, 0x8018, 0x00000000 },			\
	{ AR5K_INI_FLAG_BOTH, 0x8020, 0x00000000 },			\
	{ AR5K_INI_FLAG_BOTH, 0x8024, 0x00000000 },			\
	{ AR5K_INI_FLAG_BOTH, 0x8028, 0x00000030 },			\
	{ AR5K_INI_FLAG_BOTH, 0x802c, 0x0007ffff },			\
	{ AR5K_INI_FLAG_BOTH, 0x8030, 0x01ffffff },			\
	{ AR5K_INI_FLAG_BOTH, 0x8034, 0x00000031 },			\
	{ AR5K_INI_FLAG_BOTH, 0x8038, 0x00000000 },			\
	{ AR5K_INI_FLAG_BOTH, 0x803c, 0x00000000 },			\
	{ AR5K_INI_FLAG_BOTH, 0x8048, 0x00000000 },			\
	{ AR5K_INI_FLAG_BOTH, 0x8054, 0x00000000 },			\
	{ AR5K_INI_FLAG_BOTH, 0x8058, 0x00000000 },			\
	{ AR5K_INI_FLAG_BOTH, 0x805c, 0xffffc7ff },			\
	{ AR5K_INI_FLAG_BOTH, 0x8080, 0x00000000 },			\
	{ AR5K_INI_FLAG_BOTH, 0x8084, 0x00000000 },			\
	{ AR5K_INI_FLAG_BOTH, 0x8088, 0x00000000 },			\
	{ AR5K_INI_FLAG_BOTH, 0x808c, 0x00000000 },			\
	{ AR5K_INI_FLAG_BOTH, 0x8090, 0x00000000 },			\
	{ AR5K_INI_FLAG_BOTH, 0x8094, 0x00000000 },			\
	{ AR5K_INI_FLAG_BOTH, 0x8098, 0x00000000 },			\
	{ AR5K_INI_FLAG_BOTH, 0x80c0, 0x2a82301a },			\
	{ AR5K_INI_FLAG_BOTH, 0x80c4, 0x05dc01e0 },			\
	{ AR5K_INI_FLAG_BOTH, 0x80c8, 0x1f402710 },			\
	{ AR5K_INI_FLAG_BOTH, 0x80cc, 0x01f40000 },			\
	{ AR5K_INI_FLAG_BOTH, 0x80d0, 0x00001e1c },			\
	{ AR5K_INI_FLAG_BOTH, 0x80d4, 0x0002aaaa },			\
	{ AR5K_INI_FLAG_BOTH, 0x80d8, 0x02005555 },			\
	{ AR5K_INI_FLAG_BOTH, 0x80dc, 0x00000000 },			\
	{ AR5K_INI_FLAG_BOTH, 0x80e0, 0xffffffff },			\
	{ AR5K_INI_FLAG_BOTH, 0x80e4, 0x0000ffff },			\
	{ AR5K_INI_FLAG_BOTH, 0x80e8, 0x00000000 },			\
	{ AR5K_INI_FLAG_BOTH, 0x80ec, 0x00000000 },			\
	{ AR5K_INI_FLAG_BOTH, 0x80f0, 0x00000000 },			\
	{ AR5K_INI_FLAG_BOTH, 0x80f4, 0x00000000 },			\
	{ AR5K_INI_FLAG_BOTH, 0x80f8, 0x00000000 },			\
	{ AR5K_INI_FLAG_BOTH, 0x80fc, 0x00000088 },			\
	{ AR5K_INI_FLAG_BOTH, 0x8700, 0x00000000 },			\
	{ AR5K_INI_FLAG_BOTH, 0x8704, 0x0000008c },			\
	{ AR5K_INI_FLAG_BOTH, 0x8708, 0x000000e4 },			\
	{ AR5K_INI_FLAG_BOTH, 0x870c, 0x000002d5 },			\
	{ AR5K_INI_FLAG_BOTH, 0x8710, 0x00000000 },			\
	{ AR5K_INI_FLAG_BOTH, 0x8714, 0x00000000 },			\
	{ AR5K_INI_FLAG_BOTH, 0x8718, 0x000000a0 },			\
	{ AR5K_INI_FLAG_BOTH, 0x871c, 0x000001c9 },			\
	{ AR5K_INI_FLAG_BOTH, 0x8720, 0x0000002c },			\
	{ AR5K_INI_FLAG_BOTH, 0x8724, 0x0000002c },			\
	{ AR5K_INI_FLAG_BOTH, 0x8728, 0x00000030 },			\
	{ AR5K_INI_FLAG_BOTH, 0x872c, 0x0000003c },			\
	{ AR5K_INI_FLAG_BOTH, 0x8730, 0x0000002c },			\
	{ AR5K_INI_FLAG_BOTH, 0x8734, 0x0000002c },			\
	{ AR5K_INI_FLAG_BOTH, 0x8738, 0x00000030 },			\
	{ AR5K_INI_FLAG_BOTH, 0x873c, 0x0000003c },			\
	{ AR5K_INI_FLAG_BOTH, 0x8740, 0x00000000 },			\
	{ AR5K_INI_FLAG_BOTH, 0x8744, 0x00000000 },			\
	{ AR5K_INI_FLAG_BOTH, 0x8748, 0x00000000 },			\
	{ AR5K_INI_FLAG_BOTH, 0x874c, 0x00000000 },			\
	{ AR5K_INI_FLAG_BOTH, 0x8750, 0x00000000 },			\
	{ AR5K_INI_FLAG_BOTH, 0x8754, 0x00000000 },			\
	{ AR5K_INI_FLAG_BOTH, 0x8758, 0x00000000 },			\
	{ AR5K_INI_FLAG_BOTH, 0x875c, 0x00000000 },			\
	{ AR5K_INI_FLAG_BOTH, 0x8760, 0x000000d5 },			\
	{ AR5K_INI_FLAG_BOTH, 0x8764, 0x000000df },			\
	{ AR5K_INI_FLAG_BOTH, 0x8768, 0x00000102 },			\
	{ AR5K_INI_FLAG_BOTH, 0x876c, 0x0000013a },			\
	{ AR5K_INI_FLAG_BOTH, 0x8770, 0x00000075 },			\
	{ AR5K_INI_FLAG_BOTH, 0x8774, 0x0000007f },			\
	{ AR5K_INI_FLAG_BOTH, 0x8778, 0x000000a2 },			\
	{ AR5K_INI_FLAG_BOTH, 0x877c, 0x00000000 },			\
	{ AR5K_INI_FLAG_BOTH, 0x8100, 0x00010002 },			\
	{ AR5K_INI_FLAG_BOTH, 0x8104, 0x00000001 },			\
	{ AR5K_INI_FLAG_BOTH, 0x8108, 0x000000c0 },			\
	{ AR5K_INI_FLAG_BOTH, 0x810c, 0x00000000 },			\
	{ AR5K_INI_FLAG_BOTH, 0x8110, 0x00000168 },			\
	{ AR5K_INI_FLAG_BOTH, 0x8114, 0x00000000 },			\
	{ AR5K_INI_FLAG_BOTH, 0x87c0, 0x03020100 },			\
	{ AR5K_INI_FLAG_BOTH, 0x87c4, 0x07060504 },			\
	{ AR5K_INI_FLAG_BOTH, 0x87c8, 0x0b0a0908 },			\
	{ AR5K_INI_FLAG_BOTH, 0x87cc, 0x0f0e0d0c },			\
	{ AR5K_INI_FLAG_BOTH, 0x87d0, 0x13121110 },			\
	{ AR5K_INI_FLAG_BOTH, 0x87d4, 0x17161514 },			\
	{ AR5K_INI_FLAG_BOTH, 0x87d8, 0x1b1a1918 },			\
	{ AR5K_INI_FLAG_BOTH, 0x87dc, 0x1f1e1d1c },			\
	{ AR5K_INI_FLAG_BOTH, 0x87e0, 0x03020100 },			\
	{ AR5K_INI_FLAG_BOTH, 0x87e4, 0x07060504 },			\
	{ AR5K_INI_FLAG_BOTH, 0x87e8, 0x0b0a0908 },			\
	{ AR5K_INI_FLAG_BOTH, 0x87ec, 0x0f0e0d0c },			\
	{ AR5K_INI_FLAG_BOTH, 0x87f0, 0x13121110 },			\
	{ AR5K_INI_FLAG_BOTH, 0x87f4, 0x17161514 },			\
	{ AR5K_INI_FLAG_BOTH, 0x87f8, 0x1b1a1918 },			\
	{ AR5K_INI_FLAG_BOTH, 0x87fc, 0x1f1e1d1c },			\
	/* PHY registers */						\
	{ AR5K_INI_FLAG_BOTH, 0x9808, 0x00000000 },			\
	{ AR5K_INI_FLAG_BOTH, 0x980c, 0xad848e19 },			\
	{ AR5K_INI_FLAG_BOTH, 0x9810, 0x7d28e000 },			\
	{ AR5K_INI_FLAG_BOTH, 0x9814, 0x9c0a9f6b },			\
	{ AR5K_INI_FLAG_BOTH, 0x981c, 0x00000000 },			\
	{ AR5K_INI_FLAG_BOTH, 0x982c, 0x00022ffe },			\
	{ AR5K_INI_FLAG_BOTH, 0x983c, 0x00020100 },			\
	{ AR5K_INI_FLAG_BOTH, 0x9840, 0x206a017a },			\
	{ AR5K_INI_FLAG_BOTH, 0x984c, 0x1284613c },			\
	{ AR5K_INI_FLAG_BOTH, 0x9854, 0x00000859 },			\
	{ AR5K_INI_FLAG_BOTH, 0x9900, 0x00000000 },			\
	{ AR5K_INI_FLAG_BOTH, 0x9904, 0x00000000 },			\
	{ AR5K_INI_FLAG_BOTH, 0x9908, 0x00000000 },			\
	{ AR5K_INI_FLAG_BOTH, 0x990c, 0x00800000 },			\
	{ AR5K_INI_FLAG_BOTH, 0x9910, 0x00000001 },			\
	{ AR5K_INI_FLAG_BOTH, 0x991c, 0x0000092a },			\
	{ AR5K_INI_FLAG_BOTH, 0x9920, 0x05100000 },			\
	{ AR5K_INI_FLAG_BOTH, 0x9928, 0x00000001 },			\
	{ AR5K_INI_FLAG_BOTH, 0x992c, 0x00000004 },			\
	{ AR5K_INI_FLAG_BOTH, 0x9934, 0x1e1f2022 },			\
	{ AR5K_INI_FLAG_BOTH, 0x9938, 0x0a0b0c0d },			\
	{ AR5K_INI_FLAG_BOTH, 0x993c, 0x0000003f },			\
	{ AR5K_INI_FLAG_BOTH, 0x9940, 0x00000004 },			\
	{ AR5K_INI_FLAG_BOTH, 0x9948, 0x9280b212 },			\
	{ AR5K_INI_FLAG_BOTH, 0x9954, 0x5d50e188 },			\
	{ AR5K_INI_FLAG_BOTH, 0x9958, 0x000000ff },			\
	{ AR5K_INI_FLAG_BOTH, 0x995c, 0x004b6a8e },			\
	{ AR5K_INI_FLAG_BOTH, 0x9968, 0x000003ce },			\
	{ AR5K_INI_FLAG_BOTH, 0x9970, 0x192fb515 },			\
	{ AR5K_INI_FLAG_BOTH, 0x9974, 0x00000000 },			\
	{ AR5K_INI_FLAG_BOTH, 0x9978, 0x00000001 },			\
	{ AR5K_INI_FLAG_BOTH, 0x997c, 0x00000000 },			\
	{ AR5K_INI_FLAG_BOTH, 0xa184, 0x10ff10ff },			\
	{ AR5K_INI_FLAG_BOTH, 0xa188, 0x10ff10ff },			\
	{ AR5K_INI_FLAG_BOTH, 0xa18c, 0x10ff10ff },			\
	{ AR5K_INI_FLAG_BOTH, 0xa190, 0x10ff10ff },			\
	{ AR5K_INI_FLAG_BOTH, 0xa194, 0x10ff10ff },			\
	{ AR5K_INI_FLAG_BOTH, 0xa198, 0x10ff10ff },			\
	{ AR5K_INI_FLAG_BOTH, 0xa19c, 0x10ff10ff },			\
	{ AR5K_INI_FLAG_BOTH, 0xa1a0, 0x10ff10ff },			\
	{ AR5K_INI_FLAG_BOTH, 0xa1a4, 0x10ff10ff },			\
	{ AR5K_INI_FLAG_BOTH, 0xa1a8, 0x10ff10ff },			\
	{ AR5K_INI_FLAG_BOTH, 0xa1ac, 0x10ff10ff },			\
	{ AR5K_INI_FLAG_BOTH, 0xa1b0, 0x10ff10ff },			\
	{ AR5K_INI_FLAG_BOTH, 0xa1b4, 0x10ff10ff },			\
	{ AR5K_INI_FLAG_BOTH, 0xa1b8, 0x10ff10ff },			\
	{ AR5K_INI_FLAG_BOTH, 0xa1bc, 0x10ff10ff },			\
	{ AR5K_INI_FLAG_BOTH, 0xa1c0, 0x10ff10ff },			\
	{ AR5K_INI_FLAG_BOTH, 0xa1c4, 0x10ff10ff },			\
	{ AR5K_INI_FLAG_BOTH, 0xa1c8, 0x10ff10ff },			\
	{ AR5K_INI_FLAG_BOTH, 0xa1cc, 0x10ff10ff },			\
	{ AR5K_INI_FLAG_BOTH, 0xa1d0, 0x10ff10ff },			\
	{ AR5K_INI_FLAG_BOTH, 0xa1d4, 0x10ff10ff },			\
	{ AR5K_INI_FLAG_BOTH, 0xa1d8, 0x10ff10ff },			\
	{ AR5K_INI_FLAG_BOTH, 0xa1dc, 0x10ff10ff },			\
	{ AR5K_INI_FLAG_BOTH, 0xa1e0, 0x10ff10ff },			\
	{ AR5K_INI_FLAG_BOTH, 0xa1e4, 0x10ff10ff },			\
	{ AR5K_INI_FLAG_BOTH, 0xa1e8, 0x10ff10ff },			\
	{ AR5K_INI_FLAG_BOTH, 0xa1ec, 0x10ff10ff },			\
	{ AR5K_INI_FLAG_BOTH, 0xa1f0, 0x10ff10ff },			\
	{ AR5K_INI_FLAG_BOTH, 0xa1f4, 0x10ff10ff },			\
	{ AR5K_INI_FLAG_BOTH, 0xa1f8, 0x10ff10ff },			\
	{ AR5K_INI_FLAG_BOTH, 0xa1fc, 0x10ff10ff },			\
	{ AR5K_INI_FLAG_BOTH, 0xa210, 0x0080a333 },			\
	{ AR5K_INI_FLAG_BOTH, 0xa214, 0x00206c10 },			\
	{ AR5K_INI_FLAG_BOTH, 0xa218, 0x009c4060 },			\
	{ AR5K_INI_FLAG_BOTH, 0xa21c, 0x1483800a },			\
	{ AR5K_INI_FLAG_BOTH, 0xa220, 0x01831061 },			\
	{ AR5K_INI_FLAG_BOTH, 0xa224, 0x00000400 },			\
	{ AR5K_INI_FLAG_BOTH, 0xa228, 0x000001b5 },			\
	{ AR5K_INI_FLAG_BOTH, 0xa22c, 0x00000000 },			\
	{ AR5K_INI_FLAG_BOTH, 0xa234, 0x20202020 },			\
	{ AR5K_INI_FLAG_BOTH, 0xa238, 0x20202020 },			\
	{ AR5K_INI_FLAG_BOTH, 0xa23c, 0x13c889af },			\
	{ AR5K_INI_FLAG_BOTH, 0xa240, 0x38490a20 },			\
	{ AR5K_INI_FLAG_BOTH, 0xa244, 0x00007bb6 },			\
	{ AR5K_INI_FLAG_BOTH, 0xa248, 0x0fff3ffc },			\
	{ AR5K_INI_FLAG_BOTH, 0x9b00, 0x00000000 },			\
	{ AR5K_INI_FLAG_BOTH, 0x9b28, 0x0000000c },			\
	{ AR5K_INI_FLAG_BOTH, 0x9b38, 0x00000012 },			\
	{ AR5K_INI_FLAG_BOTH, 0x9b64, 0x00000021 },			\
	{ AR5K_INI_FLAG_BOTH, 0x9b8c, 0x0000002d },			\
	{ AR5K_INI_FLAG_BOTH, 0x9b9c, 0x00000033 },			\
	/* AR5111 specific */						\
	{ AR5K_INI_FLAG_5111, 0x9930, 0x00004883 },			\
	{ AR5K_INI_FLAG_5111, 0xa204, 0x00000000 },			\
	{ AR5K_INI_FLAG_5111, 0xa208, 0xd03e6788 },			\
	{ AR5K_INI_FLAG_5111, 0xa20c, 0x6448416a },			\
	{ AR5K_INI_FLAG_5111, 0x9b04, 0x00000020 },			\
	{ AR5K_INI_FLAG_5111, 0x9b08, 0x00000010 },			\
	{ AR5K_INI_FLAG_5111, 0x9b0c, 0x00000030 },			\
	{ AR5K_INI_FLAG_5111, 0x9b10, 0x00000008 },			\
	{ AR5K_INI_FLAG_5111, 0x9b14, 0x00000028 },			\
	{ AR5K_INI_FLAG_5111, 0x9b18, 0x00000004 },			\
	{ AR5K_INI_FLAG_5111, 0x9b1c, 0x00000024 },			\
	{ AR5K_INI_FLAG_5111, 0x9b20, 0x00000014 },			\
	{ AR5K_INI_FLAG_5111, 0x9b24, 0x00000034 },			\
	{ AR5K_INI_FLAG_5111, 0x9b2c, 0x0000002c },			\
	{ AR5K_INI_FLAG_5111, 0x9b30, 0x00000002 },			\
	{ AR5K_INI_FLAG_5111, 0x9b34, 0x00000022 },			\
	{ AR5K_INI_FLAG_5111, 0x9b3c, 0x00000032 },			\
	{ AR5K_INI_FLAG_5111, 0x9b40, 0x0000000a },			\
	{ AR5K_INI_FLAG_5111, 0x9b44, 0x0000002a },			\
	{ AR5K_INI_FLAG_5111, 0x9b48, 0x00000006 },			\
	{ AR5K_INI_FLAG_5111, 0x9b4c, 0x00000026 },			\
	{ AR5K_INI_FLAG_5111, 0x9b50, 0x00000016 },			\
	{ AR5K_INI_FLAG_5111, 0x9b54, 0x00000036 },			\
	{ AR5K_INI_FLAG_5111, 0x9b58, 0x0000000e },			\
	{ AR5K_INI_FLAG_5111, 0x9b5c, 0x0000002e },			\
	{ AR5K_INI_FLAG_5111, 0x9b60, 0x00000001 },			\
	{ AR5K_INI_FLAG_5111, 0x9b68, 0x00000011 },			\
	{ AR5K_INI_FLAG_5111, 0x9b6c, 0x00000031 },			\
	{ AR5K_INI_FLAG_5111, 0x9b70, 0x00000009 },			\
	{ AR5K_INI_FLAG_5111, 0x9b74, 0x00000029 },			\
	{ AR5K_INI_FLAG_5111, 0x9b78, 0x00000005 },			\
	{ AR5K_INI_FLAG_5111, 0x9b7c, 0x00000025 },			\
	{ AR5K_INI_FLAG_5111, 0x9b80, 0x00000015 },			\
	{ AR5K_INI_FLAG_5111, 0x9b84, 0x00000035 },			\
	{ AR5K_INI_FLAG_5111, 0x9b88, 0x0000000d },			\
	{ AR5K_INI_FLAG_5111, 0x9b90, 0x00000003 },			\
	{ AR5K_INI_FLAG_5111, 0x9b94, 0x00000023 },			\
	{ AR5K_INI_FLAG_5111, 0x9b98, 0x00000013 },			\
	{ AR5K_INI_FLAG_5111, 0x9ba0, 0x0000000b },			\
	{ AR5K_INI_FLAG_5111, 0x9ba4, 0x0000002b },			\
	{ AR5K_INI_FLAG_5111, 0x9ba8, 0x0000002b },			\
	{ AR5K_INI_FLAG_5111, 0x9bac, 0x0000002b },			\
	{ AR5K_INI_FLAG_5111, 0x9bb0, 0x0000002b },			\
	{ AR5K_INI_FLAG_5111, 0x9bb4, 0x0000002b },			\
	{ AR5K_INI_FLAG_5111, 0x9bb8, 0x0000002b },			\
	{ AR5K_INI_FLAG_5111, 0x9bbc, 0x0000002b },			\
	{ AR5K_INI_FLAG_5111, 0x9bc0, 0x0000002b },			\
	{ AR5K_INI_FLAG_5111, 0x9bc4, 0x0000002b },			\
	{ AR5K_INI_FLAG_5111, 0x9bc8, 0x0000002b },			\
	{ AR5K_INI_FLAG_5111, 0x9bcc, 0x0000002b },			\
	{ AR5K_INI_FLAG_5111, 0x9bd0, 0x0000002b },			\
	{ AR5K_INI_FLAG_5111, 0x9bd4, 0x0000002b },			\
	{ AR5K_INI_FLAG_5111, 0x9bd8, 0x0000002b },			\
	{ AR5K_INI_FLAG_5111, 0x9bdc, 0x0000002b },			\
	{ AR5K_INI_FLAG_5111, 0x9be0, 0x0000002b },			\
	{ AR5K_INI_FLAG_5111, 0x9be4, 0x0000002b },			\
	{ AR5K_INI_FLAG_5111, 0x9be8, 0x0000002b },			\
	{ AR5K_INI_FLAG_5111, 0x9bec, 0x0000002b },			\
	{ AR5K_INI_FLAG_5111, 0x9bf0, 0x0000002b },			\
	{ AR5K_INI_FLAG_5111, 0x9bf4, 0x0000002b },			\
	{ AR5K_INI_FLAG_5111, 0x9bf8, 0x00000002 },			\
	{ AR5K_INI_FLAG_5111, 0x9bfc, 0x00000016 },			\
	/* AR5112 specific */						\
	{ AR5K_INI_FLAG_5112, 0x9930, 0x00004882 },			\
	{ AR5K_INI_FLAG_5112, 0x9b04, 0x00000001 },			\
	{ AR5K_INI_FLAG_5112, 0x9b08, 0x00000002 },			\
	{ AR5K_INI_FLAG_5112, 0x9b0c, 0x00000003 },			\
	{ AR5K_INI_FLAG_5112, 0x9b10, 0x00000004 },			\
	{ AR5K_INI_FLAG_5112, 0x9b14, 0x00000005 },			\
	{ AR5K_INI_FLAG_5112, 0x9b18, 0x00000008 },			\
	{ AR5K_INI_FLAG_5112, 0x9b1c, 0x00000009 },			\
	{ AR5K_INI_FLAG_5112, 0x9b20, 0x0000000a },			\
	{ AR5K_INI_FLAG_5112, 0x9b24, 0x0000000b },			\
	{ AR5K_INI_FLAG_5112, 0x9b2c, 0x0000000d },			\
	{ AR5K_INI_FLAG_5112, 0x9b30, 0x00000010 },			\
	{ AR5K_INI_FLAG_5112, 0x9b34, 0x00000011 },			\
	{ AR5K_INI_FLAG_5112, 0x9b3c, 0x00000013 },			\
	{ AR5K_INI_FLAG_5112, 0x9b40, 0x00000014 },			\
	{ AR5K_INI_FLAG_5112, 0x9b44, 0x00000015 },			\
	{ AR5K_INI_FLAG_5112, 0x9b48, 0x00000018 },			\
	{ AR5K_INI_FLAG_5112, 0x9b4c, 0x00000019 },			\
	{ AR5K_INI_FLAG_5112, 0x9b50, 0x0000001a },			\
	{ AR5K_INI_FLAG_5112, 0x9b54, 0x0000001b },			\
	{ AR5K_INI_FLAG_5112, 0x9b58, 0x0000001c },			\
	{ AR5K_INI_FLAG_5112, 0x9b5c, 0x0000001d },			\
	{ AR5K_INI_FLAG_5112, 0x9b60, 0x00000020 },			\
	{ AR5K_INI_FLAG_5112, 0x9b68, 0x00000022 },			\
	{ AR5K_INI_FLAG_5112, 0x9b6c, 0x00000023 },			\
	{ AR5K_INI_FLAG_5112, 0x9b70, 0x00000024 },			\
	{ AR5K_INI_FLAG_5112, 0x9b74, 0x00000025 },			\
	{ AR5K_INI_FLAG_5112, 0x9b78, 0x00000028 },			\
	{ AR5K_INI_FLAG_5112, 0x9b7c, 0x00000029 },			\
	{ AR5K_INI_FLAG_5112, 0x9b80, 0x0000002a },			\
	{ AR5K_INI_FLAG_5112, 0x9b84, 0x0000002b },			\
	{ AR5K_INI_FLAG_5112, 0x9b88, 0x0000002c },			\
	{ AR5K_INI_FLAG_5112, 0x9b90, 0x00000030 },			\
	{ AR5K_INI_FLAG_5112, 0x9b94, 0x00000031 },			\
	{ AR5K_INI_FLAG_5112, 0x9b98, 0x00000032 },			\
	{ AR5K_INI_FLAG_5112, 0x9ba0, 0x00000034 },			\
	{ AR5K_INI_FLAG_5112, 0x9ba4, 0x00000035 },			\
	{ AR5K_INI_FLAG_5112, 0x9ba8, 0x00000035 },			\
	{ AR5K_INI_FLAG_5112, 0x9bac, 0x00000035 },			\
	{ AR5K_INI_FLAG_5112, 0x9bb0, 0x00000035 },			\
	{ AR5K_INI_FLAG_5112, 0x9bb4, 0x00000035 },			\
	{ AR5K_INI_FLAG_5112, 0x9bb8, 0x00000035 },			\
	{ AR5K_INI_FLAG_5112, 0x9bbc, 0x00000035 },			\
	{ AR5K_INI_FLAG_5112, 0x9bc0, 0x00000035 },			\
	{ AR5K_INI_FLAG_5112, 0x9bc4, 0x00000035 },			\
	{ AR5K_INI_FLAG_5112, 0x9bc8, 0x00000035 },			\
	{ AR5K_INI_FLAG_5112, 0x9bcc, 0x00000035 },			\
	{ AR5K_INI_FLAG_5112, 0x9bd0, 0x00000035 },			\
	{ AR5K_INI_FLAG_5112, 0x9bd4, 0x00000035 },			\
	{ AR5K_INI_FLAG_5112, 0x9bd8, 0x00000035 },			\
	{ AR5K_INI_FLAG_5112, 0x9bdc, 0x00000035 },			\
	{ AR5K_INI_FLAG_5112, 0x9be0, 0x00000035 },			\
	{ AR5K_INI_FLAG_5112, 0x9be4, 0x00000035 },			\
	{ AR5K_INI_FLAG_5112, 0x9be8, 0x00000035 },			\
	{ AR5K_INI_FLAG_5112, 0x9bec, 0x00000035 },			\
	{ AR5K_INI_FLAG_5112, 0x9bf0, 0x00000035 },			\
	{ AR5K_INI_FLAG_5112, 0x9bf4, 0x00000035 },			\
	{ AR5K_INI_FLAG_5112, 0x9bf8, 0x00000010 },			\
	{ AR5K_INI_FLAG_5112, 0x9bfc, 0x0000001a },			\
}

struct ar5k_ar5212_ini_mode {
	u_int16_t	mode_register;
	u_int8_t	mode_flags;
	u_int32_t	mode_value[2][5];
};

#define AR5K_AR5212_INI_MODE {							\
	{ 0x0030, AR5K_INI_FLAG_511X, {						\
		{ 0, },								\
		{ 0x00008107, 0x00008107, 0x00008107, 0x00008107, 0x00008107 }	\
	} },									\
	{ 0x1040, AR5K_INI_FLAG_511X, {						\
		{ 0, },								\
		{ 0x002ffc0f, 0x002ffc0f, 0x002ffc1f, 0x002ffc0f, 0x002ffc0f }	\
	} },									\
	{ 0x1044, AR5K_INI_FLAG_511X, {						\
		{ 0, },								\
		{ 0x002ffc0f, 0x002ffc0f, 0x002ffc1f, 0x002ffc0f, 0x002ffc0f }	\
	} },									\
	{ 0x1048, AR5K_INI_FLAG_511X, {						\
		{ 0, },								\
		{ 0x002ffc0f, 0x002ffc0f, 0x002ffc1f, 0x002ffc0f, 0x002ffc0f }	\
	} },									\
	{ 0x104c, AR5K_INI_FLAG_511X, {						\
		{ 0, },								\
		{ 0x002ffc0f, 0x002ffc0f, 0x002ffc1f, 0x002ffc0f, 0x002ffc0f }	\
	} },									\
	{ 0x1050, AR5K_INI_FLAG_511X, {						\
		{ 0, },								\
		{ 0x002ffc0f, 0x002ffc0f, 0x002ffc1f, 0x002ffc0f, 0x002ffc0f }	\
	} },									\
	{ 0x1054, AR5K_INI_FLAG_511X, {						\
		{ 0, },								\
		{ 0x002ffc0f, 0x002ffc0f, 0x002ffc1f, 0x002ffc0f, 0x002ffc0f }	\
	} },									\
	{ 0x1058, AR5K_INI_FLAG_511X, {						\
		{ 0, },								\
		{ 0x002ffc0f, 0x002ffc0f, 0x002ffc1f, 0x002ffc0f, 0x002ffc0f }	\
	} },									\
	{ 0x105c, AR5K_INI_FLAG_511X, {						\
		{ 0, },								\
		{ 0x002ffc0f, 0x002ffc0f, 0x002ffc1f, 0x002ffc0f, 0x002ffc0f }	\
	} },									\
	{ 0x1060, AR5K_INI_FLAG_511X, {						\
		{ 0, },								\
		{ 0x002ffc0f, 0x002ffc0f, 0x002ffc1f, 0x002ffc0f, 0x002ffc0f }	\
	} },									\
	{ 0x1064, AR5K_INI_FLAG_511X, {						\
		{ 0, },								\
		{ 0x002ffc0f, 0x002ffc0f, 0x002ffc1f, 0x002ffc0f, 0x002ffc0f }	\
	} },									\
	{ 0x1030, AR5K_INI_FLAG_511X, {						\
		{ 0, },								\
		{ 0x00000230, 0x000001e0, 0x000000b0, 0x00000160, 0x000001e0 }	\
	} },									\
	{ 0x1070, AR5K_INI_FLAG_511X, {						\
		{ 0, },								\
		{ 0x00000168, 0x000001e0, 0x000001b8, 0x0000018c, 0x000001e0 }	\
	} },									\
	{ 0x10b0, AR5K_INI_FLAG_511X, {						\
		{ 0, },								\
		{ 0x00000e60, 0x00001180, 0x00001f1c, 0x00003e38, 0x00001180 }	\
	} },									\
	{ 0x10f0, AR5K_INI_FLAG_511X, {						\
		{ 0, },								\
		{ 0x0000a0e0, 0x00014068, 0x00005880, 0x0000b0e0, 0x00014068 }	\
	} },									\
	{ 0x8014, AR5K_INI_FLAG_511X, {						\
		{ 0, },								\
		{ 0x03e803e8, 0x06e006e0, 0x04200420, 0x08400840, 0x06e006e0 }	\
	} },									\
	{ 0x9804, AR5K_INI_FLAG_511X, {						\
		{ 0, },								\
		{ 0x00000000, 0x00000003, 0x00000000, 0x00000000, 0x00000003 }	\
	} },									\
	{ 0x9820, AR5K_INI_FLAG_511X, {						\
		{ 0, },								\
		{ 0x02020200, 0x02020200, 0x02010200, 0x02020200, 0x02020200 }	\
	} },									\
	{ 0x9834, AR5K_INI_FLAG_511X, {						\
		{ 0, },								\
		{ 0x00000e0e, 0x00000e0e, 0x00000e0e, 0x00000e0e, 0x00000e0e }	\
	} },									\
	{ 0x9838, AR5K_INI_FLAG_511X, {						\
		{ 0, },								\
		{ 0x00000007, 0x00000007, 0x0000000b, 0x0000000b, 0x0000000b }	\
	} },									\
	{ 0x9844, AR5K_INI_FLAG_511X, {						\
		{ 0, },								\
		{ 0x1372161c, 0x13721c25, 0x13721728, 0x137216a2, 0x13721c25 }	\
	} },									\
	{ 0x9850, AR5K_INI_FLAG_511X, {						\
		{ 0, },								\
		{ 0x0de8b4e0, 0x0de8b4e0, 0x0de8b4e0, 0x0de8b4e0, 0x0de8b4e0 }	\
	} },									\
	{ 0x9858, AR5K_INI_FLAG_511X, {						\
		{ 0, },								\
		{ 0x7e800d2e, 0x7e800d2e, 0x7ee84d2e, 0x7ee84d2e, 0x7e800d2e }	\
	} },									\
	{ 0x9860, AR5K_INI_FLAG_511X, {						\
		{ 0, },								\
		{ 0x00009d10, 0x00009d10, 0x00009d18, 0x00009d10, 0x00009d10 }	\
	} },									\
	{ 0x9864, AR5K_INI_FLAG_511X, {						\
		{ 0, },								\
		{ 0x0001ce00, 0x0001ce00, 0x0001ce00, 0x0001ce00, 0x0001ce00 }	\
	} },									\
	{ 0x9868, AR5K_INI_FLAG_511X, {						\
		{ 0, },								\
		{ 0x409a4190, 0x409a4190, 0x409a4190, 0x409a4190, 0x409a4190 }	\
	} },									\
	{ 0x9918, AR5K_INI_FLAG_511X, {						\
		{ 0, },								\
		{ 0x000001b8, 0x000001b8, 0x00000084, 0x00000108, 0x000001b8 }	\
	} },									\
	{ 0x9924, AR5K_INI_FLAG_511X, {						\
		{ 0, },								\
		{ 0x10058a05, 0x10058a05, 0x10058a05, 0x10058a05, 0x10058a05 }	\
	} },									\
	{ 0xa180, AR5K_INI_FLAG_511X, {						\
		{ 0, },								\
		{ 0x10ff14ff, 0x10ff14ff, 0x10ff10ff, 0x10ff19ff, 0x10ff19ff }	\
	} },									\
	{ 0xa230, AR5K_INI_FLAG_511X, {						\
		{ 0, },								\
		{ 0x00000000, 0x00000000, 0x00000000, 0x00000108, 0x00000000 }	\
	} },									\
	{ 0x801c, AR5K_INI_FLAG_BOTH, {						\
		{ 0x128d8fa7, 0x09880fcf, 0x04e00f95, 0x128d8fab, 0x09880fcf }, \
		{ 0x128d93a7, 0x098813cf, 0x04e01395, 0x128d93ab, 0x098813cf }	\
	} },									\
	{ 0x9824, AR5K_INI_FLAG_BOTH, {						\
		{ 0x00000e0e, 0x00000e0e, 0x00000707, 0x00000e0e, 0x00000e0e },	\
		{ 0x00000e0e, 0x00000e0e, 0x00000e0e, 0x00000e0e, 0x00000e0e }	\
	} },									\
	{ 0x9828, AR5K_INI_FLAG_BOTH, {						\
		{ 0x0a020001, 0x0a020001, 0x05010100, 0x0a020001, 0x0a020001 },	\
		{ 0x0a020001, 0x0a020001, 0x05020100, 0x0a020001, 0x0a020001 }	\
	} },									\
	{ 0x9848, AR5K_INI_FLAG_BOTH, {						\
		{ 0x0018da5a, 0x0018da5a, 0x0018ca69, 0x0018ca69, 0x0018ca69 },	\
		{ 0x0018da6d, 0x0018da6d, 0x0018ca75, 0x0018ca75, 0x0018ca75 }	\
	} },									\
	{ 0x985c, AR5K_INI_FLAG_BOTH, {						\
		{ 0x3137665e, 0x3137665e, 0x3137665e, 0x3137665e, 0x3137615e },	\
		{ 0x3137665e, 0x3137665e, 0x3137665e, 0x3137665e, 0x3137665e }	\
	} },									\
	{ 0x986c, AR5K_INI_FLAG_BOTH, {						\
		{ 0x050cb081, 0x050cb081, 0x050cb081, 0x050cb080, 0x050cb080 },	\
		{ 0x050cb081, 0x050cb081, 0x050cb081, 0x050cb081, 0x050cb081 }	\
	} },									\
	{ 0x9914, AR5K_INI_FLAG_BOTH, {						\
		{ 0x00002710, 0x00002710, 0x0000157c, 0x00002af8, 0x00002710 },	\
		{ 0x000007d0, 0x000007d0, 0x0000044c, 0x00000898, 0x000007d0 }	\
	} },									\
	{ 0x9944, AR5K_INI_FLAG_BOTH, {						\
		{ 0xffb81020, 0xffb81020, 0xffb80d20, 0xffb81020, 0xffb81020 },	\
		{ 0xffb81020, 0xffb81020, 0xffb80d10, 0xffb81010, 0xffb81010 }	\
	} },									\
	{ 0xa204, AR5K_INI_FLAG_5112, {						\
		{ 0, },								\
		{ 0x00000000, 0x00000000, 0x00000004, 0x00000004, 0x00000004 }	\
	} },									\
	{ 0xa208, AR5K_INI_FLAG_5112, {						\
		{ 0, },								\
		{ 0xd6be6788, 0xd6be6788, 0xd03e6788, 0xd03e6788, 0xd03e6788 }	\
	} },									\
	{ 0xa20c, AR5K_INI_FLAG_5112, {						\
		{ 0, },								\
		{ 0x642c0140, 0x642c0140, 0x6442c160, 0x6442c160, 0x6442c160 }	\
	} },									\
}

#endif /* _AR5K_AR5212_VAR_H */
