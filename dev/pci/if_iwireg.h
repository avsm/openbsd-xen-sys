/*	$OpenBSD: if_iwireg.h,v 1.20 2006/02/26 19:14:40 damien Exp $	*/

/*-
 * Copyright (c) 2004-2006
 *      Damien Bergamini <damien.bergamini@free.fr>. All rights reserved.
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
 */

#define IWI_CMD_RING_COUNT	16
#define IWI_TX_RING_COUNT	64
#define IWI_RX_RING_COUNT	32

#define IWI_CSR_INTR		0x0008
#define IWI_CSR_INTR_MASK	0x000c
#define IWI_CSR_INDIRECT_ADDR	0x0010
#define IWI_CSR_INDIRECT_DATA	0x0014
#define IWI_CSR_AUTOINC_ADDR	0x0018
#define IWI_CSR_AUTOINC_DATA	0x001c
#define IWI_CSR_RST		0x0020
#define IWI_CSR_CTL		0x0024
#define IWI_CSR_IO		0x0030
#define IWI_CSR_CMD_BASE	0x0200
#define IWI_CSR_CMD_SIZE	0x0204
#define IWI_CSR_TX1_BASE	0x0208
#define IWI_CSR_TX1_SIZE	0x020c
#define IWI_CSR_TX2_BASE	0x0210
#define IWI_CSR_TX2_SIZE	0x0214
#define IWI_CSR_TX3_BASE	0x0218
#define IWI_CSR_TX3_SIZE	0x021c
#define IWI_CSR_TX4_BASE	0x0220
#define IWI_CSR_TX4_SIZE	0x0224
#define IWI_CSR_CMD_RIDX	0x0280
#define IWI_CSR_TX1_RIDX	0x0284
#define IWI_CSR_TX2_RIDX	0x0288
#define IWI_CSR_TX3_RIDX	0x028c
#define IWI_CSR_TX4_RIDX	0x0290
#define IWI_CSR_RX_RIDX		0x02a0
#define IWI_CSR_RX_BASE		0x0500
#define IWI_CSR_TABLE0_SIZE	0x0700
#define IWI_CSR_TABLE0_BASE	0x0704
#define IWI_CSR_NODE_BASE	0x0c0c
#define IWI_CSR_CMD_WIDX	0x0f80
#define IWI_CSR_TX1_WIDX	0x0f84
#define IWI_CSR_TX2_WIDX	0x0f88
#define IWI_CSR_TX3_WIDX	0x0f8c
#define IWI_CSR_TX4_WIDX	0x0f90
#define IWI_CSR_RX_WIDX		0x0fa0
#define IWI_CSR_READ_INT	0x0ff4

/* aliases */
#define IWI_CSR_CURRENT_TX_RATE	IWI_CSR_TABLE0_BASE

/* possible flags for IWI_CSR_INTR */
#define IWI_INTR_RX_DONE	0x00000002
#define IWI_INTR_CMD_DONE	0x00000800
#define IWI_INTR_TX1_DONE	0x00001000
#define IWI_INTR_TX2_DONE	0x00002000
#define IWI_INTR_TX3_DONE	0x00004000
#define IWI_INTR_TX4_DONE	0x00008000
#define IWI_INTR_FW_INITED	0x01000000
#define IWI_INTR_RADIO_OFF	0x04000000
#define IWI_INTR_FATAL_ERROR	0x40000000
#define IWI_INTR_PARITY_ERROR	0x80000000

#define IWI_INTR_MASK							\
	(IWI_INTR_RX_DONE | IWI_INTR_CMD_DONE | IWI_INTR_TX1_DONE |	\
	 IWI_INTR_TX2_DONE | IWI_INTR_TX3_DONE | IWI_INTR_TX4_DONE |	\
	 IWI_INTR_FW_INITED | IWI_INTR_RADIO_OFF |			\
	 IWI_INTR_FATAL_ERROR | IWI_INTR_PARITY_ERROR)

/* possible flags for register IWI_CSR_RST */
#define IWI_RST_PRINCETON_RESET	0x00000001
#define IWI_RST_SW_RESET	0x00000080
#define IWI_RST_MASTER_DISABLED	0x00000100
#define IWI_RST_STOP_MASTER	0x00000200

/* possible flags for register IWI_CSR_CTL */
#define IWI_CTL_CLOCK_READY	0x00000001
#define IWI_CTL_ALLOW_STANDBY	0x00000002
#define IWI_CTL_INIT		0x00000004

/* possible flags for register IWI_CSR_IO */
#define IWI_IO_RADIO_ENABLED	0x00010000

/* possible flags for IWI_CSR_READ_INT */
#define IWI_READ_INT_INIT_HOST	0x20000000

/* table2 offsets */
#define IWI_INFO_ADAPTER_MAC	40

/* constants for command blocks */
#define IWI_CB_DEFAULT_CTL	0x8cea0000
#define IWI_CB_MAXDATALEN	8191

/* supported rates */
#define IWI_RATE_DS1	10
#define IWI_RATE_DS2	20
#define IWI_RATE_DS5	55
#define IWI_RATE_DS11	110
#define IWI_RATE_OFDM6	13
#define IWI_RATE_OFDM9	15
#define IWI_RATE_OFDM12	5
#define IWI_RATE_OFDM18	7
#define IWI_RATE_OFDM24	9
#define IWI_RATE_OFDM36	11
#define IWI_RATE_OFDM48	1
#define IWI_RATE_OFDM54	3

/* firmware binary image header */
struct iwi_firmware_hdr {
	uint32_t	version;
	uint32_t	bootsz;
	uint32_t	ucodesz;
	uint32_t	mainsz;
} __packed;

struct iwi_hdr {
	uint8_t	type;
#define IWI_HDR_TYPE_DATA	0
#define IWI_HDR_TYPE_COMMAND	1
#define IWI_HDR_TYPE_NOTIF	3
#define IWI_HDR_TYPE_FRAME	9

	uint8_t	seq;
	uint8_t	flags;
#define IWI_HDR_FLAG_IRQ	0x04

	uint8_t	reserved;
} __packed;

struct iwi_notif {
	uint32_t	reserved[2];
	uint8_t		type;
#define IWI_NOTIF_TYPE_ASSOCIATION	10
#define IWI_NOTIF_TYPE_AUTHENTICATION	11
#define IWI_NOTIF_TYPE_SCAN_CHANNEL	12
#define IWI_NOTIF_TYPE_SCAN_COMPLETE	13
#define IWI_NOTIF_TYPE_BEACON		17
#define IWI_NOTIF_TYPE_CALIBRATION	20
#define IWI_NOTIF_TYPE_NOISE		25

	uint8_t		flags;
	uint16_t	len;
} __packed;

/* structure for notification IWI_NOTIF_TYPE_AUTHENTICATION */
struct iwi_notif_authentication {
	uint8_t	state;
#define IWI_DEAUTHENTICATED	0
#define IWI_AUTHENTICATED	9
} __packed;

/* structure for notification IWI_NOTIF_TYPE_ASSOCIATION */
struct iwi_notif_association {
	uint8_t			state;
#define IWI_DEASSOCIATED	0
#define IWI_ASSOCIATED		12

	struct ieee80211_frame	frame;
	uint16_t		capinfo;
	uint16_t		status;
	uint16_t		associd;
} __packed;

/* structure for notification IWI_NOTIF_TYPE_SCAN_CHANNEL */
struct iwi_notif_scan_channel {
	uint8_t	nchan;
	uint8_t	reserved[47];
} __packed;

/* structure for notification IWI_NOTIF_TYPE_SCAN_COMPLETE */
struct iwi_notif_scan_complete {
	uint8_t	type;
	uint8_t	nchan;
	uint8_t	status;
	uint8_t	reserved;
} __packed;

/* received frame header */
struct iwi_frame {
	uint32_t	reserved1[2];
	uint8_t		chan;
	uint8_t		status;
	uint8_t		rate;
	uint8_t		rssi;	/* receiver signal strength indicator */
	uint8_t		agc;	/* automatic gain control */
	uint8_t		rssi_dbm;
	uint16_t	signal;
	uint16_t	noise;
	uint8_t		antenna;
	uint8_t		control;
	uint8_t		reserved2[2];
	uint16_t	len;
} __packed;

/* header for transmission */
struct iwi_tx_desc {
	struct iwi_hdr	hdr;
	uint32_t	reserved1;
	uint8_t		station;
	uint8_t		reserved2[3];
	uint8_t		cmd;
#define IWI_DATA_CMD_TX	0x0b

	uint8_t		seq;
	uint16_t	len;
	uint8_t		priority;
	uint8_t		flags;
#define IWI_DATA_FLAG_SHPREAMBLE	0x04
#define IWI_DATA_FLAG_NO_WEP		0x20
#define IWI_DATA_FLAG_NEED_ACK		0x80

	uint8_t		xflags;
	uint8_t		wep_txkey;
	uint8_t		wepkey[IEEE80211_KEYBUF_SIZE];
	uint8_t		rate;
	uint8_t		antenna;
	uint8_t		reserved3[10];

	struct ieee80211_qosframe_addr4	wh;
	uint32_t	iv[2];

	uint32_t	nseg;
#define IWI_MAX_NSEG	6
#define IWI_MAX_SCATTER	(IWI_MAX_NSEG - 2)

	uint32_t	seg_addr[IWI_MAX_NSEG];
	uint16_t	seg_len[IWI_MAX_NSEG];
} __packed;

/* command */
struct iwi_cmd_desc {
	struct iwi_hdr	hdr;
	uint8_t		type;
#define IWI_CMD_ENABLE			2
#define IWI_CMD_SET_CONFIG		6
#define IWI_CMD_SET_ESSID		8
#define IWI_CMD_SET_MAC_ADDRESS		11
#define IWI_CMD_SET_RTS_THRESHOLD	15
#define IWI_CMD_SET_FRAG_THRESHOLD	16
#define IWI_CMD_SET_POWER_MODE		17
#define IWI_CMD_SET_WEP_KEY		18
#define IWI_CMD_ASSOCIATE		21
#define IWI_CMD_SET_RATES		22
#define IWI_CMD_SCAN			26
#define IWI_CMD_DISABLE			33
#define IWI_CMD_SET_IV			34
#define IWI_CMD_SET_TX_POWER		35
#define IWI_CMD_SET_SENSITIVITY		42

	uint8_t		len;
	uint16_t	reserved;
	uint8_t		data[120];
} __packed;

/* node information (IBSS) */
struct iwi_node {
	uint8_t	bssid[IEEE80211_ADDR_LEN];
	uint8_t	reserved[2];
} __packed;

/* constants for 'mode' fields */
#define IWI_MODE_11A	0
#define IWI_MODE_11B	1
#define IWI_MODE_11G	2

/* possible values for command IWI_CMD_SET_POWER_MODE */
#define IWI_POWER_MODE_CAM	0

/* structure for command IWI_CMD_SET_RATES */
struct iwi_rateset {
	uint8_t	mode;
	uint8_t	nrates;
	uint8_t	type;
#define IWI_RATESET_TYPE_NEGOTIATED	0
#define IWI_RATESET_TYPE_SUPPORTED	1

	uint8_t	reserved;
	uint8_t	rates[12];
} __packed;

/* structure for command IWI_CMD_SET_TX_POWER */
struct iwi_txpower {
	uint8_t	nchan;
	uint8_t	mode;
	struct {
		uint8_t	chan;
		uint8_t	power;
#define IWI_TXPOWER_MAX		20
#define IWI_TXPOWER_RATIO	(IEEE80211_TXPOWER_MAX / IWI_TXPOWER_MAX)
	} __packed chan[37];
} __packed;

/* structure for command IWI_CMD_ASSOCIATE */
struct iwi_associate {
	uint8_t		chan;
	uint8_t		auth;
#define IWI_AUTH_OPEN	0
#define IWI_AUTH_SHARED	1
#define IWI_AUTH_NONE	3

	uint8_t		type;
	uint8_t		reserved1;
	uint16_t	reserved2;
	uint8_t		plen;
	uint8_t		mode;
	uint8_t		bssid[IEEE80211_ADDR_LEN];
	uint8_t		tstamp[8];
	uint16_t	capinfo;
	uint16_t	lintval;
	uint16_t	intval;
	uint8_t		dst[IEEE80211_ADDR_LEN];
	uint32_t	reserved3;
	uint16_t	reserved4;
} __packed;

/* structure for command IWI_CMD_SCAN */
struct iwi_scan {
	uint32_t	index;
	uint8_t		channels[54];
#define IWI_CHAN_5GHZ	(0 << 6)
#define IWI_CHAN_2GHZ	(1 << 6)

	uint8_t		type[27];
#define IWI_SCAN_TYPE_PASSIVE	0x11
#define IWI_SCAN_TYPE_DIRECTED	0x22
#define IWI_SCAN_TYPE_BROADCAST	0x33
#define IWI_SCAN_TYPE_BDIRECTED	0x44

	uint8_t		reserved1;
	uint16_t	reserved2;
	uint16_t	passive;	/* dwell time */
	uint16_t	directed;	/* dwell time */
	uint16_t	broadcast;	/* dwell time */
	uint16_t	bdirected;	/* dwell time */
} __packed;

/* structure for command IWI_CMD_SET_CONFIGURATION */
struct iwi_configuration {
	uint8_t	bluetooth_coexistence;
	uint8_t	reserved1;
	uint8_t	answer_pbreq;
	uint8_t	allow_invalid_frames;
	uint8_t	multicast_enabled;
	uint8_t	exclude_unicast_unencrypted;
	uint8_t	disable_unicast_decryption;
	uint8_t	exclude_multicast_unencrypted;
	uint8_t	disable_multicast_decryption;
	uint8_t	antenna;
	uint8_t	reserved2;
	uint8_t	bg_autodetection;
	uint8_t	reserved3;
	uint8_t	enable_multicast_filtering;
	uint8_t	bluetooth_threshold;
	uint8_t	reserved4;
	uint8_t	allow_beacon_and_probe_resp;
	uint8_t	allow_mgt;
	uint8_t	noise_reported;
	uint8_t	reserved5;
} __packed;

/* structure for command IWI_CMD_SET_WEP_KEY */
struct iwi_wep_key {
	uint8_t	cmd;
#define IWI_WEP_KEY_CMD_SETKEY	0x08

	uint8_t	seq;
	uint8_t	idx;
	uint8_t	len;
	uint8_t	key[IEEE80211_KEYBUF_SIZE];
} __packed;

#define IWI_MEM_EEPROM_CTL	0x00300040
#define IWI_MEM_EVENT_CTL	0x00300004

/* possible flags for register IWI_MEM_EVENT */
#define IWI_LED_ASSOC	(1 << 5)
#define IWI_LED_MASK	0xd9fffffb

/* EEPROM = Electrically Erasable Programmable Read-Only Memory */

#define IWI_EEPROM_MAC	0x21

#define IWI_EEPROM_DELAY	1	/* minimum hold time (microsecond) */

#define IWI_EEPROM_C	(1 << 0)	/* Serial Clock */
#define IWI_EEPROM_S	(1 << 1)	/* Chip Select */
#define IWI_EEPROM_D	(1 << 2)	/* Serial data input */
#define IWI_EEPROM_Q	(1 << 4)	/* Serial data output */

#define IWI_EEPROM_SHIFT_D	2
#define IWI_EEPROM_SHIFT_Q	4

/*
 * control and status registers access macros
 */
#define CSR_READ_1(sc, reg)						\
	bus_space_read_1((sc)->sc_st, (sc)->sc_sh, (reg))

#define CSR_READ_2(sc, reg)						\
	bus_space_read_2((sc)->sc_st, (sc)->sc_sh, (reg))

#define CSR_READ_4(sc, reg)						\
	bus_space_read_4((sc)->sc_st, (sc)->sc_sh, (reg))

#define CSR_READ_REGION_4(sc, offset, datap, count)			\
	bus_space_read_region_4((sc)->sc_st, (sc)->sc_sh, (offset),	\
	    (datap), (count))

#define CSR_WRITE_1(sc, reg, val)					\
	bus_space_write_1((sc)->sc_st, (sc)->sc_sh, (reg), (val))

#define CSR_WRITE_2(sc, reg, val)					\
	bus_space_write_2((sc)->sc_st, (sc)->sc_sh, (reg), (val))

#define CSR_WRITE_4(sc, reg, val)					\
	bus_space_write_4((sc)->sc_st, (sc)->sc_sh, (reg), (val))

#define CSR_WRITE_REGION_1(sc, offset, datap, count)			\
	bus_space_write_region_1((sc)->sc_st, (sc)->sc_sh, (offset),	\
	    (datap), (count))
/*
 * indirect memory space access macros
 */
#define MEM_WRITE_1(sc, addr, val) do {					\
	CSR_WRITE_4((sc), IWI_CSR_INDIRECT_ADDR, (addr));		\
	CSR_WRITE_1((sc), IWI_CSR_INDIRECT_DATA, (val));		\
} while (/* CONSTCOND */0)

#define MEM_WRITE_2(sc, addr, val) do {					\
	CSR_WRITE_4((sc), IWI_CSR_INDIRECT_ADDR, (addr));		\
	CSR_WRITE_2((sc), IWI_CSR_INDIRECT_DATA, (val));		\
} while (/* CONSTCOND */0)

#define MEM_WRITE_4(sc, addr, val) do {					\
	CSR_WRITE_4((sc), IWI_CSR_INDIRECT_ADDR, (addr));		\
	CSR_WRITE_4((sc), IWI_CSR_INDIRECT_DATA, (val));		\
} while (/* CONSTCOND */0)

#define MEM_WRITE_MULTI_1(sc, addr, buf, len) do {			\
	CSR_WRITE_4((sc), IWI_CSR_INDIRECT_ADDR, (addr));		\
	CSR_WRITE_MULTI_1((sc), IWI_CSR_INDIRECT_DATA, (buf), (len));	\
} while (/* CONSTCOND */0)

/*
 * EEPROM access macro
 */
#define IWI_EEPROM_CTL(sc, val) do {					\
	MEM_WRITE_4((sc), IWI_MEM_EEPROM_CTL, (val));			\
	DELAY(IWI_EEPROM_DELAY);					\
} while (/* CONSTCOND */0)
