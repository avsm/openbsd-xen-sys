/*	$OpenBSD: anreg.h,v 1.5 2001/02/27 06:41:11 tholo Exp $	*/

/*
 * Copyright (c) 1997, 1998, 1999
 *	Bill Paul <wpaul@ctr.columbia.edu>.  All rights reserved.
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
 *	This product includes software developed by Bill Paul.
 * 4. Neither the name of the author nor the names of any co-contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY Bill Paul AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL Bill Paul OR THE VOICES IN HIS HEAD
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 *
 * $FreeBSD: src/sys/dev/an/if_anreg.h,v 1.1 2000/01/14 20:40:56 wpaul Exp $
 */

#define AN_TIMEOUT	65536

/* Default network name: empty string */
#define AN_DEFAULT_NETNAME	""

/* The nodename must be less than 16 bytes */
#define AN_DEFAULT_NODENAME	"OpenBSD"

#define AN_DEFAULT_IBSS		"OpenBSD IBSS"

/*
 * register space access macros
 */
#define CSR_WRITE_2(sc, reg, val)	\
	bus_space_write_2(sc->an_btag, sc->an_bhandle, reg, val)

#define CSR_READ_2(sc, reg)		\
	bus_space_read_2(sc->an_btag, sc->an_bhandle, reg)

#define CSR_WRITE_1(sc, reg, val)	\
	bus_space_write_1(sc->an_btag, sc->an_bhandle, reg, val)

#define CSR_READ_1(sc, reg)		\
	bus_space_read_1(sc->an_btag, sc->an_bhandle, reg)

/*
 * Size of Aironet I/O space.
 */
#define AN_IOSIZ		0x40

/*
 * Hermes register definitions and what little I know about them.
 */

/* Hermes command/status registers. */
#define AN_COMMAND		0x00
#define AN_PARAM0		0x02
#define AN_PARAM1		0x04
#define AN_PARAM2		0x06
#define AN_STATUS		0x08
#define AN_RESP0		0x0A
#define AN_RESP1		0x0C
#define AN_RESP2		0x0E
#define AN_LINKSTAT		0x10

/* Command register */
#define AN_CMD_BUSY		0x8000 /* busy bit */
#define AN_CMD_NO_ACK		0x0080 /* don't acknowledge command */
#define AN_CMD_CODE_MASK	0x003F
#define AN_CMD_QUAL_MASK	0x7F00

/* Command codes */
#define AN_CMD_NOOP		0x0000 /* no-op */
#define AN_CMD_ENABLE		0x0001 /* enable */
#define AN_CMD_DISABLE		0x0002 /* disable */
#define AN_CMD_FORCE_SYNCLOSS	0x0003 /* force loss of sync */
#define AN_CMD_FW_RESTART	0x0004 /* firmware resrart */
#define AN_CMD_HOST_SLEEP	0x0005
#define AN_CMD_MAGIC_PKT	0x0006
#define AN_CMD_READCFG		0x0008
#define AN_CMD_ALLOC_MEM	0x000A /* allocate NIC memory */
#define AN_CMD_TX		0x000B /* transmit */
#define AN_CMD_DEALLOC_MEM	0x000C
#define AN_CMD_NOOP2		0x0010
#define AN_CMD_ACCESS		0x0021
#define AN_CMD_ALLOC_BUF	0x0028
#define AN_CMD_PSP_NODES	0x0030
#define AN_CMD_SET_PHYREG	0x003E
#define AN_CMD_TX_TEST		0x003F
#define AN_CMD_SLEEP		0x0085
#define AN_CMD_SAVECFG		0x0108

/*
 * Reclaim qualifier bit, applicable to the
 * TX command.
 */
#define AN_RECLAIM		0x0100 /* reclaim NIC memory */

/*
 * ACCESS command qualifier bits.
 */
#define AN_ACCESS_READ		0x0000
#define AN_ACCESS_WRITE		0x0100

/*
 * PROGRAM command qualifier bits.
 */
#define AN_PROGRAM_DISABLE	0x0000
#define AN_PROGRAM_ENABLE_RAM	0x0100
#define AN_PROGRAM_ENABLE_NVRAM	0x0200
#define AN_PROGRAM_NVRAM	0x0300

/* Status register values */
#define AN_STAT_CMD_CODE	0x003F
#define AN_STAT_CMD_RESULT	0x7F00

/* Linkstat register */
#define AN_LINKSTAT_ASSOCIATED		0x0400
#define AN_LINKSTAT_AUTHFAIL		0x0300
#define AN_LINKSTAT_ASSOC_FAIL		0x8400
#define AN_LINKSTAT_DISASSOC		0x8200
#define AN_LINKSTAT_DEAUTH		0x8100
#define AN_LINKSTAT_SYNCLOST_TSF	0x8004
#define AN_LINKSTAT_SYNCLOST_HOSTREQ	0x8003
#define AN_LINKSTAT_SYNCLOST_AVGRETRY	0x8002
#define AN_LINKSTAT_SYNCLOST_MAXRETRY	0x8001
#define AN_LINKSTAT_SYNCLOST_MISSBEACON	0x8000

/* memory handle management registers */
#define AN_RX_FID		0x20
#define AN_ALLOC_FID		0x22
#define AN_TX_CMP_FID		0x24

/*
 * Buffer Access Path (BAP) registers.
 * These are I/O channels. I believe you can use each one for
 * any desired purpose independently of the other. In general
 * though, we use BAP1 for reading and writing LTV records and
 * reading received data frames, and BAP0 for writing transmit
 * frames. This is a convention though, not a rule.
 */
#define AN_SEL0			0x18
#define AN_SEL1			0x1A
#define AN_OFF0			0x1C
#define AN_OFF1			0x1E
#define AN_DATA0		0x36
#define AN_DATA1		0x38
#define AN_BAP0			AN_DATA0
#define AN_BAP1			AN_DATA1

#define AN_OFF_BUSY		0x8000
#define AN_OFF_ERR		0x4000
#define AN_OFF_DONE		0x2000
#define AN_OFF_DATAOFF		0x0FFF

/* Event registers */
#define AN_EVENT_STAT		0x30	/* Event status */
#define AN_INT_EN		0x32	/* Interrupt enable/disable */
#define AN_EVENT_ACK		0x34	/* Ack event */

/* Events */
#define AN_EV_CLR_STUCK_BUSY	0x4000	/* clear stuck busy bit */
#define AN_EV_WAKEREQUEST	0x2000	/* awaken from PSP mode */
#define AN_EV_AWAKE		0x0100	/* station woke up from PSP mode*/
#define AN_EV_LINKSTAT		0x0080	/* link status available */
#define AN_EV_CMD		0x0010	/* command completed */
#define AN_EV_ALLOC		0x0008	/* async alloc/reclaim completed */
#define AN_EV_TX_EXC		0x0004	/* async xmit completed with failure */
#define AN_EV_TX		0x0002	/* async xmit completed succesfully */
#define AN_EV_RX		0x0001	/* async rx completed */

#define AN_INTRS	\
	(AN_EV_RX|AN_EV_TX|AN_EV_TX_EXC|AN_EV_ALLOC|AN_EV_LINKSTAT)

/* Host software registers */
#define AN_SW0			0x28
#define AN_SW1			0x2A
#define AN_SW2			0x2C
#define AN_SW3			0x2E

#define AN_CNTL			0x14

#define AN_CNTL_AUX_ENA		0xC000
#define AN_CNTL_AUX_ENA_STAT	0xC000
#define AN_CNTL_AUX_DIS_STAT	0x0000
#define AN_CNTL_AUX_ENA_CNTL	0x8000
#define AN_CNTL_AUX_DIS_CNTL	0x4000

#define AN_AUX_PAGE		0x3A
#define AN_AUX_OFFSET		0x3C
#define AN_AUX_DATA		0x3E

/*
 * Length, Type, Value (LTV) record definitions and RID values.
 */
struct an_ltv_gen {
	u_int16_t		an_len;
	u_int16_t		an_type;
	u_int16_t		an_val;
};

/*
 * General configuration information.
 */
#define AN_RID_GENCONFIG	0xFF10
struct an_ltv_genconfig {
	/* General configuration. */
	u_int16_t		an_len;			/* 0x00 */
	u_int16_t		an_type;		/* XXXX */
	u_int16_t		an_opmode;		/* 0x02 */
	u_int16_t		an_rxmode;		/* 0x04 */
	u_int16_t		an_fragthresh;		/* 0x06 */
	u_int16_t		an_rtsthresh;		/* 0x08 */
	u_int8_t		an_macaddr[6];		/* 0x0A */
	u_int8_t		an_rates[8];		/* 0x10 */
	u_int16_t		an_shortretry_limit;	/* 0x18 */
	u_int16_t		an_longretry_limit;	/* 0x1A */
	u_int16_t		an_tx_msdu_lifetime;	/* 0x1C */
	u_int16_t		an_rx_msdu_lifetime;	/* 0x1E */
	u_int16_t		an_stationary;		/* 0x20 */
	u_int16_t		an_ordering;		/* 0x22 */
	u_int16_t		an_devtype;		/* 0x24 */
	u_int16_t		an_rsvd0[5];		/* 0x26 */
	/* Scanning associating. */
	u_int16_t		an_scanmode;		/* 0x30 */
	u_int16_t		an_probedelay;		/* 0x32 */
	u_int16_t		an_probe_energy_timeout;/* 0x34 */
	u_int16_t		an_probe_response_timeout;/*0x36 */
	u_int16_t		an_beacon_listen_timeout;/*0x38 */
	u_int16_t		an_ibss_join_net_timeout;/*0x3A */
	u_int16_t		an_auth_timeout;	/* 0x3C */
	u_int16_t		an_authtype;		/* 0x3E */
	u_int16_t		an_assoc_timeout;	/* 0x40 */
	u_int16_t		an_specified_ap_timeout;/* 0x42 */
	u_int16_t		an_offline_scan_interval;/*0x44 */
	u_int16_t		an_offline_scan_duration;/*0x46 */
	u_int16_t		an_link_loss_delay;	/* 0x48 */
	u_int16_t		an_max_beacon_lost_time;/* 0x4A */
	u_int16_t		an_refresh_interval;	/* 0x4C */
	u_int16_t		an_rsvd1;		/* 0x4E */
	/* Power save operation */
	u_int16_t		an_psave_mode;		/* 0x50 */
	u_int16_t		an_sleep_for_dtims;	/* 0x52 */
	u_int16_t		an_listen_interval;	/* 0x54 */
	u_int16_t		an_fast_listen_interval;/* 0x56 */
	u_int16_t		an_listen_decay;	/* 0x58 */
	u_int16_t		an_fast_listen_decay;	/* 0x5A */
	u_int16_t		an_rsvd2[2];		/* 0x5C */
	/* Ad-hoc (or AP) operation. */
	u_int16_t		an_beacon_period;	/* 0x60 */
	u_int16_t		an_atim_duration;	/* 0x62 */
	u_int16_t		an_rsvd3;		/* 0x64 */
	u_int16_t		an_ds_channel;		/* 0x66 */
	u_int16_t		an_rsvd4;		/* 0x68 */
	u_int16_t		an_dtim_period;		/* 0x6A */
	u_int16_t		an_rsvd5[2];		/* 0x6C */
	/* Radio operation. */
	u_int16_t		an_radiotype;		/* 0x70 */
	u_int16_t		an_diversity;		/* 0x72 */
	u_int16_t		an_tx_power;		/* 0x74 */
	u_int16_t		an_rss_thresh;		/* 0x76 */
	u_int16_t		an_modulation_type;	/* 0x77 */
	u_int16_t		an_short_preamble;	/* 0x7A */
	u_int16_t		an_home_product;	/* 0x7C */
	u_int16_t		an_rsvd6;		/* 0x7E */
	/* Aironet extensions. */
	u_int8_t		an_nodename[16];	/* 0x80 */
	u_int16_t		an_arl_thresh;		/* 0x90 */
	u_int16_t		an_arl_decay;		/* 0x92 */
	u_int16_t		an_arl_delay;		/* 0x94 */
	u_int8_t		an_rsvd7;		/* 0x96 */
	u_int8_t		an_rsvd8;		/* 0x97 */
	u_int8_t		an_magic_packet_action;	/* 0x98 */
	u_int8_t		an_magic_packet_ctl;	/* 0x99 */
	u_int16_t		an_rsvd9;
};

#define AN_OPMODE_IBSS_ADHOC			0x0000
#define AN_OPMODE_INFRASTRUCTURE_STATION	0x0001
#define AN_OPMODE_AP				0x0002
#define AN_OPMODE_AP_REPEATER			0x0003
#define AN_OPMODE_UNMODIFIED_PAYLOAD		0x0100
#define AN_OPMODE_AIRONET_EXTENSIONS		0x0200
#define AN_OPMODE_AP_EXTENSIONS			0x0400

#define AN_RXMODE_BC_MC_ADDR			0x0000
#define AN_RXMODE_BC_ADDR			0x0001
#define AN_RXMODE_ADDR				0x0002
#define AN_RXMODE_80211_MONITOR_CURBSS		0x0003
#define AN_RXMODE_80211_MONITOR_ANYBSS		0x0004
#define AN_RXMODE_LAN_MONITOR_CURBSS		0x0005
#define AN_RXMODE_NO_8023_HEADER		0x0100

#define AN_RATE_1MBPS				0x0002
#define AN_RATE_2MBPS				0x0004
#define AN_RATE_5_5MBPS				0x000B
#define AN_RATE_11MBPS				0x0016

#define AN_DEVTYPE_PC4500			0x0065
#define AN_DEVTYPE_PC4800			0x006D

#define AN_SCANMODE_ACTIVE			0x0000
#define AN_SCANMODE_PASSIVE			0x0001
#define AN_SCANMODE_AIRONET_ACTIVE		0x0002

#define AN_AUTHTYPE_NONE			0x0000
#define AN_AUTHTYPE_OPEN			0x0001
#define AN_AUTHTYPE_SHAREDKEY			0x0002
#define AN_AUTHTYPE_PRIVACY_IN_USE		0x0100
#define AN_AUTHTYPE_ALLOW_UNENCRYPTED		0x0200

#define AN_PSAVE_NONE				0x0000
#define AN_PSAVE_CAM				0x0001
#define AN_PSAVE_PSP				0x0002
#define AN_PSAVE_PSP_CAM			0x0003

#define AN_RADIOTYPE_80211_FH			0x0001
#define AN_RADIOTYPE_80211_DS			0x0002
#define AN_RADIOTYPE_LM2000_DS			0x0004

#define AN_DIVERSITY_FACTORY_DEFAULT		0x0000
#define AN_DIVERSITY_ANTENNA_1_ONLY		0x0001
#define AN_DIVERSITY_ANTENNA_2_ONLY		0x0002
#define AN_DIVERSITY_ANTENNA_1_AND_2		0x0003

#define AN_TXPOWER_FACTORY_DEFAULT		0x0000
#define AN_TXPOWER_50MW				50
#define AN_TXPOWER_100MW			100
#define AN_TXPOWER_250MW			250

/*
 * Valid SSID list. You can specify up to three SSIDs denoting
 * the service sets that you want to join. The first SSID always
 * defaults to "tsunami" which is a handy way to detect the
 * card.
 */
#define AN_RID_SSIDLIST		0xFF11
struct an_ltv_ssidlist {
	u_int16_t		an_len;
	u_int16_t		an_type;
	u_int16_t		an_ssid1_len;
	char			an_ssid1[32];
	u_int16_t		an_ssid2_len;
	char			an_ssid2[32];
	u_int16_t		an_ssid3_len;
	char			an_ssid3[32];
};

#define AN_DEF_SSID_LEN		7
#define AN_DEF_SSID		"tsunami"

/*
 * Valid AP list.
 */
#define AN_RID_APLIST		0xFF12
struct an_ltv_aplist {
	u_int16_t		an_len;
	u_int16_t		an_type;
	u_int8_t		an_ap1[8];
	u_int8_t		an_ap2[8];
	u_int8_t		an_ap3[8];
	u_int8_t		an_ap4[8];
};

/*
 * Driver name.
 */
#define AN_RID_DRVNAME		0xFF13
struct an_ltv_drvname {
	u_int16_t		an_len;
	u_int16_t		an_type;
	u_int8_t		an_drvname[16];
};

/*
 * Frame encapsulation.
 */
#define AN_RID_ENCAP		0xFF14
struct an_rid_encap {
	u_int16_t		an_len;
	u_int16_t		an_type;
	u_int16_t		an_ethertype_default;
	u_int16_t		an_action_default;
	u_int16_t		an_ethertype0;
	u_int16_t		an_action0;
	u_int16_t		an_ethertype1;
	u_int16_t		an_action1;
	u_int16_t		an_ethertype2;
	u_int16_t		an_action2;
	u_int16_t		an_ethertype3;
	u_int16_t		an_action3;
	u_int16_t		an_ethertype4;
	u_int16_t		an_action4;
	u_int16_t		an_ethertype5;
	u_int16_t		an_action5;
	u_int16_t		an_ethertype6;
	u_int16_t		an_action6;
};

#define AN_ENCAP_ACTION_RX	0x0001
#define AN_ENCAP_ACTION_TX	0x0002

#define AN_RXENCAP_NONE		0x0000
#define AN_RXENCAP_RFC1024	0x0001

#define AN_TXENCAP_RFC1024	0x0000
#define AN_TXENCAP_80211	0x0002

/*
 * Actual config, same structure as general config (read only).
 */
#define AN_RID_ACTUALCFG	0xFF20

/*
 * Card capabilities (read only).
 */
#define AN_RID_CAPABILITIES	0xFF00
struct an_ltv_caps {
	u_int16_t		an_len;			/* 0x00 */
	u_int16_t		an_type;		/* XXXX */
	u_int8_t		an_oui[3];		/* 0x02 */
	u_int8_t		an_rsvd0;		/* 0x05 */
	u_int16_t		an_prodnum;		/* 0x06 */
	u_int8_t		an_manufname[32];	/* 0x08 */
	u_int8_t		an_prodname[16];	/* 0x28 */
	u_int8_t		an_prodvers[8];		/* 0x38 */
	u_int8_t		an_oemaddr[6];		/* 0x40 */
	u_int8_t		an_aironetaddr[6];	/* 0x46 */
	u_int16_t		an_radiotype;		/* 0x4C */
	u_int16_t		an_regdomain;		/* 0x4E */
	u_int8_t		an_callid[6];		/* 0x50 */
	u_int8_t		an_rates[8];		/* 0x56 */
	u_int8_t		an_rx_diversity;	/* 0x5E */
	u_int8_t		an_tx_diversity;	/* 0x5F */
	u_int16_t		an_tx_powerlevels[8];	/* 0x60 */
	u_int16_t		an_hwrev;		/* 0x70 */
	u_int16_t		an_hwcaps;		/* 0x72 */
	u_int16_t		an_temprange;		/* 0x74 */
	u_int16_t		an_fwrev;		/* 0x76 */
	u_int16_t		an_fwsubrev;		/* 0x78 */
	u_int16_t		an_ifacerev;		/* 0x7A */
	u_int16_t		an_softcaps;		/* 0x7C */
	u_int16_t		an_bootblockrev;	/* 0x7E */
	u_int16_t		an_req_hw_support;	/* 0x80 */
};

/*
 * Access point (read only)
 */
#define AN_RID_APINFO		0xFF01
struct an_ltv_apinfo {
	u_int16_t		an_len;
	u_int16_t		an_type;
	u_int16_t		an_tim_addr;
	u_int16_t		an_airo_addr;
};

/*
 * Radio info (read only).
 */
#define AN_RID_RADIOINFO	0xFF02
struct an_ltv_radioinfo {
	u_int16_t		an_len;
	u_int16_t		an_type;
	/* ??? */
};

/*
 * Status (read only). Note: the manual claims this RID is 108 bytes
 * long (0x6A is the last datum, which is 2 bytes long) however when
 * this RID is read from the NIC, it returns a length of 110. To be
 * on the safe side, this structure is padded with an extra 16-bit
 * word. (There is a misprint in the manual which says the macaddr
 * field is 8 bytes long.)
 *
 * Also, the channel_set and current_channel fields appear to be
 * reversed. Either that, or the hop_period field is unused.
 */
#define AN_RID_STATUS		0xFF50
struct an_ltv_status {
	u_int16_t		an_len;			/* 0x00 */
	u_int16_t		an_type;		/* 0xXX */
	u_int8_t		an_macaddr[6];		/* 0x02 */
	u_int16_t		an_opmode;		/* 0x08 */
	u_int16_t		an_errcode;		/* 0x0A */
	u_int16_t		an_cur_signal_strength;	/* 0x0C */
	u_int16_t		an_ssidlen;		/* 0x0E */
	u_int8_t		an_ssid[32];		/* 0x10 */
	u_int8_t		an_ap_name[16];		/* 0x30 */
	u_int8_t		an_cur_bssid[6];	/* 0x40 */
	u_int8_t		an_prev_bssid1[6];	/* 0x46 */
	u_int8_t		an_prev_bssid2[6];	/* 0x4C */
	u_int8_t		an_prev_bssid3[6];	/* 0x52 */
	u_int16_t		an_beacon_period;	/* 0x58 */
	u_int16_t		an_dtim_period;		/* 0x5A */
	u_int16_t		an_atim_duration;	/* 0x5C */
	u_int16_t		an_hop_period;		/* 0x5E */
	u_int16_t		an_cur_channel;		/* 0x62 */
	u_int16_t		an_channel_set;		/* 0x60 */
	u_int16_t		an_hops_to_backbone;	/* 0x64 */
	u_int16_t		an_ap_total_load;	/* 0x66 */
	u_int16_t		an_our_generated_load;	/* 0x68 */
	u_int16_t		an_accumulated_arl;	/* 0x6A */
	u_int16_t		an_cur_signal_quality;	/* 0x6C */
	u_int16_t		an_current_tx_rate;	/* 0x6E */
	u_int16_t		an_ap_device;		/* 0x70 */
	u_int16_t		an_normalized_rssi;	/* 0x72 */
	u_int16_t		an_short_pre_in_use;	/* 0x74 */
	u_int8_t		an_ap_ip_addr[4];	/* 0x76 */
	u_int16_t		an_max_noise_prev_sec;	/* 0x7A */
	u_int16_t		an_avg_noise_prev_min;	/* 0x7C */
	u_int16_t		an_max_noise_prev_min;	/* 0x7E */
	u_int16_t		an_rsvd0[3];		/* 0x80 */
};

#define AN_STATUS_OPMODE_CONFIGURED		0x0001
#define AN_STATUS_OPMODE_MAC_ENABLED		0x0002
#define AN_STATUS_OPMODE_RX_ENABLED		0x0004
#define AN_STATUS_OPMODE_IN_SYNC		0x0010
#define AN_STATUS_OPMODE_ASSOCIATED		0x0020
#define AN_STATUS_OPMODE_ERROR			0x8000


/*
 * Statistics
 */
#define AN_RID_16BITS_CUM	0xFF60	/* Cumulative 16-bit stats counters */
#define AN_RID_16BITS_DELTA	0xFF61	/* 16-bit stats (since last clear) */
#define AN_RID_16BITS_DELTACLR	0xFF62	/* 16-bit stats, clear on read */
#define AN_RID_32BITS_CUM	0xFF68	/* Cumulative 32-bit stats counters */
#define AN_RID_32BITS_DELTA	0xFF69	/* 32-bit stats (since last clear) */
#define AN_RID_32BITS_DELTACLR	0xFF6A	/* 32-bit stats, clear on read */

/*
 * Grrr. The manual says the statistics record is 384 bytes in length,
 * but the card says the record is 404 bytes. There's some padding left
 * at the end of this structure to account for any discrepancies.
 */
struct an_ltv_stats {
	u_int16_t		an_fudge;
	u_int16_t		an_len;			/* 0x00 */
	u_int16_t		an_type;		/* 0xXX */
	u_int16_t		an_spacer;		/* 0x02 */
	u_int32_t		an_rx_overruns;		/* 0x04 */
	u_int32_t		an_rx_plcp_csum_errs;	/* 0x08 */
	u_int32_t		an_rx_plcp_format_errs;	/* 0x0C */
	u_int32_t		an_rx_plcp_len_errs;	/* 0x10 */
	u_int32_t		an_rx_mac_crc_errs;	/* 0x14 */
	u_int32_t		an_rx_mac_crc_ok;	/* 0x18 */
	u_int32_t		an_rx_wep_errs;		/* 0x1C */
	u_int32_t		an_rx_wep_ok;		/* 0x20 */
	u_int32_t		an_retry_long;		/* 0x24 */
	u_int32_t		an_retry_short;		/* 0x28 */
	u_int32_t		an_retry_max;		/* 0x2C */
	u_int32_t		an_no_ack;		/* 0x30 */
	u_int32_t		an_no_cts;		/* 0x34 */
	u_int32_t		an_rx_ack_ok;		/* 0x38 */
	u_int32_t		an_rx_cts_ok;		/* 0x3C */
	u_int32_t		an_tx_ack_ok;		/* 0x40 */
	u_int32_t		an_tx_rts_ok;		/* 0x44 */
	u_int32_t		an_tx_cts_ok;		/* 0x48 */
	u_int32_t		an_tx_lmac_mcasts;	/* 0x4C */
	u_int32_t		an_tx_lmac_bcasts;	/* 0x50 */
	u_int32_t		an_tx_lmac_ucast_frags;	/* 0x54 */
	u_int32_t		an_tx_lmac_ucasts;	/* 0x58 */
	u_int32_t		an_tx_beacons;		/* 0x5C */
	u_int32_t		an_rx_beacons;		/* 0x60 */
	u_int32_t		an_tx_single_cols;	/* 0x64 */
	u_int32_t		an_tx_multi_cols;	/* 0x68 */
	u_int32_t		an_tx_defers_no;	/* 0x6C */
	u_int32_t		an_tx_defers_prot;	/* 0x70 */
	u_int32_t		an_tx_defers_energy;	/* 0x74 */
	u_int32_t		an_rx_dups;		/* 0x78 */
	u_int32_t		an_rx_partial;		/* 0x7C */
	u_int32_t		an_tx_too_old;		/* 0x80 */
	u_int32_t		an_rx_too_old;		/* 0x84 */
	u_int32_t		an_lostsync_max_retries;/* 0x88 */
	u_int32_t		an_lostsync_missed_beacons;/* 0x8C */
	u_int32_t		an_lostsync_arl_exceeded;/*0x90 */
	u_int32_t		an_lostsync_deauthed;	/* 0x94 */
	u_int32_t		an_lostsync_disassociated;/*0x98 */
	u_int32_t		an_lostsync_tsf_timing;	/* 0x9C */
	u_int32_t		an_tx_host_mcasts;	/* 0xA0 */
	u_int32_t		an_tx_host_bcasts;	/* 0xA4 */
	u_int32_t		an_tx_host_ucasts;	/* 0xA8 */
	u_int32_t		an_tx_host_failed;	/* 0xAC */
	u_int32_t		an_rx_host_mcasts;	/* 0xB0 */
	u_int32_t		an_rx_host_bcasts;	/* 0xB4 */
	u_int32_t		an_rx_host_ucasts;	/* 0xB8 */
	u_int32_t		an_rx_host_discarded;	/* 0xBC */
	u_int32_t		an_tx_hmac_mcasts;	/* 0xC0 */
	u_int32_t		an_tx_hmac_bcasts;	/* 0xC4 */
	u_int32_t		an_tx_hmac_ucasts;	/* 0xC8 */
	u_int32_t		an_tx_hmac_failed;	/* 0xCC */
	u_int32_t		an_rx_hmac_mcasts;	/* 0xD0 */
	u_int32_t		an_rx_hmac_bcasts;	/* 0xD4 */
	u_int32_t		an_rx_hmac_ucasts;	/* 0xD8 */
	u_int32_t		an_rx_hmac_discarded;	/* 0xDC */
	u_int32_t		an_tx_hmac_accepted;	/* 0xE0 */
	u_int32_t		an_ssid_mismatches;	/* 0xE4 */
	u_int32_t		an_ap_mismatches;	/* 0xE8 */
	u_int32_t		an_rates_mismatches;	/* 0xEC */
	u_int32_t		an_auth_rejects;	/* 0xF0 */
	u_int32_t		an_auth_timeouts;	/* 0xF4 */
	u_int32_t		an_assoc_rejects;	/* 0xF8 */
	u_int32_t		an_assoc_timeouts;	/* 0xFC */
	u_int32_t		an_reason_outside_table;/* 0x100 */
	u_int32_t		an_reason1;		/* 0x104 */
	u_int32_t		an_reason2;		/* 0x108 */
	u_int32_t		an_reason3;		/* 0x10C */
	u_int32_t		an_reason4;		/* 0x110 */
	u_int32_t		an_reason5;		/* 0x114 */
	u_int32_t		an_reason6;		/* 0x118 */
	u_int32_t		an_reason7;		/* 0x11C */
	u_int32_t		an_reason8;		/* 0x120 */
	u_int32_t		an_reason9;		/* 0x124 */
	u_int32_t		an_reason10;		/* 0x128 */
	u_int32_t		an_reason11;		/* 0x12C */
	u_int32_t		an_reason12;		/* 0x130 */
	u_int32_t		an_reason13;		/* 0x134 */
	u_int32_t		an_reason14;		/* 0x138 */
	u_int32_t		an_reason15;		/* 0x13C */
	u_int32_t		an_reason16;		/* 0x140 */
	u_int32_t		an_reason17;		/* 0x144 */
	u_int32_t		an_reason18;		/* 0x148 */
	u_int32_t		an_reason19;		/* 0x14C */
	u_int32_t		an_rx_mgmt_pkts;	/* 0x150 */
	u_int32_t		an_tx_mgmt_pkts;	/* 0x154 */
	u_int32_t		an_rx_refresh_pkts;	/* 0x158 */
	u_int32_t		an_tx_refresh_pkts;	/* 0x15C */
	u_int32_t		an_rx_poll_pkts;	/* 0x160 */
	u_int32_t		an_tx_poll_pkts;	/* 0x164 */
	u_int32_t		an_host_retries;	/* 0x168 */
	u_int32_t		an_lostsync_hostreq;	/* 0x16C */
	u_int32_t		an_host_tx_bytes;	/* 0x170 */
	u_int32_t		an_host_rx_bytes;	/* 0x174 */
	u_int32_t		an_uptime_usecs;	/* 0x178 */
	u_int32_t		an_uptime_secs;		/* 0x17C */
	u_int32_t		an_lostsync_better_ap;	/* 0x180 */
	u_int32_t		an_rsvd[10];
};

/*
 * WEP config
 */
#define AN_RID_WEP_VOLATILE	0xFF15
#define AN_RID_WEP_PERMANENT	0xFF16
struct an_wepkey {
	u_int16_t		an_len;			/* 0x00 */
	u_int16_t		an_type;		/* 0xXX */
	u_int16_t		an_key_index;		/* 0x02 */
	u_int8_t		an_mac_addr[6];		/* 0x04 */
	u_int16_t		an_key_len;		/* 0x0A */
	u_int8_t		an_key[13];		/* 0x0C */
};

/*
 * Receive frame structure.
 */
struct an_rxframe {
	u_int32_t		an_rx_time;		/* 0x00 */
	u_int16_t		an_rx_status;		/* 0x04 */
	u_int16_t		an_rx_payload_len;	/* 0x06 */
	u_int8_t		an_rsvd0;		/* 0x08 */
	u_int8_t		an_rx_signal_strength;	/* 0x09 */
	u_int8_t		an_rx_rate;		/* 0x0A */
	u_int8_t		an_rx_chan;		/* 0x0B */
	u_int8_t		an_rx_assoc_cnt;	/* 0x0C */
	u_int8_t		an_rsvd1[3];		/* 0x0D */
	u_int8_t		an_plcp_hdr[4];		/* 0x10 */
	u_int16_t		an_frame_ctl;		/* 0x14 */
	u_int16_t		an_duration;		/* 0x16 */
	u_int8_t		an_addr1[6];		/* 0x18 */
	u_int8_t		an_addr2[6];		/* 0x1E */
	u_int8_t		an_addr3[6];		/* 0x24 */
	u_int16_t		an_seq_ctl;		/* 0x2A */
	u_int8_t		an_addr4[6];		/* 0x2C */
	u_int16_t		an_gaplen;		/* 0x32 */
};

#define AN_RXGAP_MAX	8

/*
 * Transmit frame structure.
 */
struct an_txframe {
	u_int32_t		an_tx_sw;		/* 0x00 */
	u_int16_t		an_tx_status;		/* 0x04 */
	u_int16_t		an_tx_payload_len;	/* 0x06 */
	u_int16_t		an_tx_ctl;		/* 0x08 */
	u_int16_t		an_tx_assoc_id;		/* 0x0A */
	u_int16_t		an_tx_retry;		/* 0x0C */
	u_int8_t		an_tx_assoc_cnt;	/* 0x0E */
	u_int8_t		an_tx_rate;		/* 0x0F */
	u_int8_t		an_tx_max_long_retries;	/* 0x10 */
	u_int8_t		an_tx_max_short_retries; /*0x11 */
	u_int8_t		an_rsvd0[2];		/* 0x12 */
	u_int16_t		an_frame_ctl;		/* 0x14 */
	u_int16_t		an_duration;		/* 0x16 */
	u_int8_t		an_addr1[6];		/* 0x18 */
	u_int8_t		an_addr2[6];		/* 0x1E */
	u_int8_t		an_addr3[6];		/* 0x24 */
	u_int16_t		an_seq_ctl;		/* 0x2A */
	u_int8_t		an_addr4[6];		/* 0x2C */
	u_int16_t		an_gaplen;		/* 0x32 */
};

struct an_rxframe_802_3 {
	u_int16_t		an_rx_802_3_status;     /* 0x34 */
	u_int16_t		an_rx_802_3_payload_len;/* 0x36 */
	u_int8_t		an_rx_dst_addr[6];      /* 0x38 */
	u_int8_t		an_rx_src_addr[6];      /* 0x3E */
};
#define AN_RXGAP_MAX	8


struct an_txframe_802_3 {
/*
 * Transmit 802.3 header structure.
 */
	u_int16_t		an_tx_802_3_status;     /* 0x34 */
	u_int16_t		an_tx_802_3_payload_len;/* 0x36 */
	u_int8_t		an_tx_dst_addr[6];      /* 0x38 */
	u_int8_t		an_tx_src_addr[6];      /* 0x3E */
};

#define AN_TXSTAT_EXCESS_RETRY	0x0002
#define AN_TXSTAT_LIFE_EXCEEDED	0x0004
#define AN_TXSTAT_AID_FAIL	0x0008
#define AN_TXSTAT_MAC_DISABLED	0x0010
#define AN_TXSTAT_ASSOC_LOST	0x0020

#define AN_TXCTL_RSVD		0x0001
#define AN_TXCTL_TXOK_INTR	0x0002
#define AN_TXCTL_TXERR_INTR	0x0004
#define AN_TXCTL_HEADER_TYPE	0x0008
#define AN_TXCTL_PAYLOAD_TYPE	0x0010
#define AN_TXCTL_NORELEASE	0x0020
#define AN_TXCTL_NORETRIES	0x0040
#define AN_TXCTL_CLEAR_AID	0x0080
#define AN_TXCTL_STRICT_ORDER	0x0100
#define AN_TXCTL_USE_RTS	0x0200

#define AN_HEADERTYPE_8023	0x0000
#define AN_HEADERTYPE_80211	0x0008

#define AN_PAYLOADTYPE_ETHER	0x0000
#define AN_PAYLOADTYPE_LLC	0x0010

#define AN_TXCTL_80211	\
	(AN_TXCTL_TXOK_INTR|AN_TXCTL_TXERR_INTR|AN_HEADERTYPE_80211|	\
	AN_PAYLOADTYPE_LLC|AN_TXCTL_NORELEASE)

#define AN_TXCTL_8023	\
	(AN_TXCTL_TXOK_INTR|AN_TXCTL_TXERR_INTR|AN_HEADERTYPE_8023|	\
	AN_PAYLOADTYPE_ETHER|AN_TXCTL_NORELEASE)

#define AN_TXGAP_80211		0
#define AN_TXGAP_8023		0

struct an_802_3_hdr {
	u_int16_t		an_8023_status;
	u_int16_t		an_8023_payload_len;
	u_int8_t		an_8023_dst_addr[6];
	u_int8_t		an_8023_src_addr[6];
	u_int16_t		an_8023_dat[3];	/* SNAP header */
	u_int16_t		an_8023_type;
};

struct an_snap_hdr {
	u_int16_t		an_snap_dat[3];	/* SNAP header */
	u_int16_t		an_snap_type;
};

#define AN_TX_RING_CNT		4
#define AN_INC(x, y)		(x) = (x + 1) % y

struct an_tx_ring_data {
	u_int16_t		an_tx_fids[AN_TX_RING_CNT];
	u_int16_t		an_tx_ring[AN_TX_RING_CNT];
	int			an_tx_prod;
	int			an_tx_cons;
};

struct an_softc	{
	struct device	sc_dev;
	struct arpcom	arpcom;
	void		*sc_ih;

	bus_space_tag_t		an_btag;
	bus_space_handle_t	an_bhandle;

	struct an_ltv_genconfig	an_config;
	struct an_ltv_caps	an_caps;
	struct an_ltv_ssidlist	an_ssidlist;
	struct an_ltv_aplist	an_aplist;
	int			an_tx_rate;
	int			an_rxmode;
	int			an_gone;
	int			an_if_flags;
	u_int8_t		an_txbuf[1536];
	struct an_tx_ring_data	an_rdata;
	struct an_ltv_stats	an_stats;
	struct an_ltv_status	an_status;
	u_int8_t		an_associated;
#ifdef ANCACHE
	int			an_cache_iponly;
	int			an_cache_mcastonly;

	int			an_sigitems;
	struct an_sigcache	an_sigcache[MAXANCACHE];
	int			an_nextitem;
#endif
	struct timeout		an_stat_ch;
};

void	an_release_resources	__P((struct device *));
int	an_alloc_port		__P((struct device *, int, int));
int	an_alloc_memory		__P((struct device *, int, int));
int	an_alloc_irq		__P((struct device *, int, int));
int	an_probe	        __P((struct device *));
void	an_shutdown	        __P((void *));
int	an_attach		__P((struct an_softc *));
void	an_init			__P((struct an_softc *));
void    an_stop		        __P((struct an_softc *));
int	an_intr			__P((void *));

#define AN_802_3_OFFSET		0x2E
#define AN_802_11_OFFSET	0x44
#define AN_802_11_OFFSET_RAW	0x3C

#define AN_STAT_BADCRC		0x0001
#define AN_STAT_UNDECRYPTABLE	0x0002
#define AN_STAT_ERRSTAT		0x0003
#define AN_STAT_MAC_PORT	0x0700
#define AN_STAT_1042		0x2000	/* RFC1042 encoded */
#define AN_STAT_TUNNEL		0x4000	/* Bridge-tunnel encoded */
#define AN_STAT_WMP_MSG		0x6000	/* WaveLAN-II management protocol */
#define AN_RXSTAT_MSG_TYPE	0xE000

#define AN_ENC_TX_802_3		0x00
#define AN_ENC_TX_802_11	0x11
#define AN_ENC_TX_E_II		0x0E

#define AN_ENC_TX_1042		0x00
#define AN_ENC_TX_TUNNEL	0xF8

#define AN_TXCNTL_MACPORT	0x00FF
#define AN_TXCNTL_STRUCTTYPE	0xFF00

/*
 * SNAP (sub-network access protocol) constants for transmission
 * of IP datagrams over IEEE 802 networks, taken from RFC1042.
 * We need these for the LLC/SNAP header fields in the TX/RX frame
 * structure.
 */
#define AN_SNAP_K1		0xaa	/* assigned global SAP for SNAP */
#define AN_SNAP_K2		0x00
#define AN_SNAP_CONTROL		0x03	/* unnumbered information format */
#define AN_SNAP_WORD0		(AN_SNAP_K1 | (AN_SNAP_K1 << 8))
#define AN_SNAP_WORD1		(AN_SNAP_K2 | (AN_SNAP_CONTROL << 8))
#define AN_SNAPHDR_LEN		0x6
