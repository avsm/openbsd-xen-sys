/*	$OpenBSD: atapi.h,v 1.2 1996/06/09 08:59:48 downsj Exp $	*/

/*
 * Copyright (c) 1996 Manuel Bouyer.  All rights reserved.
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
 *  This product includes software developed by Manuel Bouyer.
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

/*
 * Definition of atapi commands and associated data structures
 */

/* 
 * TEST UNIT READY (mandatory)
 */
#define ATAPI_TEST_UNIT_READY		0x00

struct test_unit_ready {
	u_int8_t	operation_code;
	u_int8_t	reserved1[15];
};

/* 
 * START/STOP UNIT (mandatory)
 */ 
#define ATAPI_START_STOP_UNIT		0x1b
 
struct start_stop_unit {
	u_int8_t	operation_code;
	u_int8_t	flags;
#define START_STOP_IMMED	0x01
	u_int8_t	reserved2[2]; 
	u_int8_t	how;
#define SSS_STOP	0x00 
#define SSS_START	0x01
#define SSS_LOEJ	0x02
	u_int8_t	reserved4[11];
};

/* 
 * PREVENT/ALLOW MEDIUM REMOVAL (mandatory)
 */ 
#define ATAPI_PREVENT_ALLOW_MEDIUM_REMOVAL 0x1e
 
struct prevent_allow_medium_removal {
	u_int8_t	operation_code;
	u_int8_t	reserved1[3];
	u_int8_t	how;	
#define PR_PREVENT	0x01
#define PR_ALLOW	0x00
	u_int8_t	reserved3[11];
};


/* 
 * READ CD CAPACITY (mandatory)
 */
#define ATAPI_READ_CD_CAPACITY		0x25

struct read_cd_capacity {
	u_int8_t	operation_code;
	u_int8_t	reserved1[7];
	u_int8_t	len;
	u_int8_t	reserved2[7];
};

/* 
 * Volume size info.
 */
struct read_cd_capacity_data {
	u_long	size;		/* Volume size in blocks */
	u_long	blksize;	/* Block size in bytes */
};

/*
 * READ (10) (mandatory)
 */  
#define ATAPI_READ			0x28  

struct read {    
	u_int8_t	operation_code;
	u_int8_t	reserved1;
	u_int8_t	lba[4];
	u_int8_t	reserved2;
	u_int8_t	length[2];
	u_int8_t	reserved3[7];
}; 

/* 
 * PAUSE/RESUME (optional) 
 */ 
#define ATAPI_PAUSE_RESUME		0x4b
 
struct pause_resume {
	u_int8_t	operation_code;
	u_int8_t	reserved1[7];
	u_int8_t	resume;
#define PA_PAUSE	0x00
#define PA_RESUME	0x01
	u_int8_t	reserved3[7];
};

/*
 * MODE SENSE (mandatory)
 */
#define ATAPI_MODE_SENSE        0x5a

struct mode_sense {
	u_int8_t	operation_code;
	u_int8_t	reserved1;
	u_int8_t	page_code_control;
#define PAGE_CODE_MASK		0x3f
#define PAGE_CONTROL_MASK	0xc0
	u_int8_t	reserved2[4];
	u_int8_t	length[2];
	u_int8_t	reserved3[7];
};

struct cappage {
	/* Mode data header */
	u_int8_t	data_length[2];
	u_int8_t	medium_type;
#define MDT_UNKNOWN	0x00        
#define MDT_DATA_120	0x01
#define MDT_AUDIO_120	0x02
#define MDT_COMB_120	0x03    
#define MDT_PHOTO_120	0x04    
#define MDT_DATA_80	0x05
#define MDT_AUDIO_80	0x06
#define MDT_COMB_80	0x07 
#define MDT_PHOTO_80	0x08
#define MDT_NO_DISC	0x70
#define MDT_DOOR_OPEN	0x71
#define MDT_FMT_ERROR	0x72
	u_int8_t	reserved1[5];

	/* Capabilities page */
	u_int8_t	page_code;
#define CAP_PAGE	0x2a
	u_int8_t param_len;
	u_int8_t reserved2[2];

	u_int8_t	format_cap;
#define FORMAT_AUDIO_PLAY	0x01	/* audio play supported */
#define FORMAT_COMPOSITE	0x02	/* composite audio/video supported */
#define FORMAT_DPORT1		0x04	/* digital audio on port 1 */
#define FORMAT_DPORT2		0x08	/* digital audio on port 2 */
#define FORMAT_MODE2_FORM1	0x10	/* mode 2 form 1 (XA) read */
#define FORMAT_MODE2_FORM2	0x20	/* mode 2 form 2 format */
#define FORMAT_MULTISESSION	0x40	/* multi-session photo-CD */

	u_int8_t	ops_cap;
#define OPS_CDDA		0x01	/* audio-CD read supported */
#define OPS_CDDA_STREAM		0x02	/* CDDA streaming */
#define OPS_RW			0x04	/* combined R-W subchannels */
#define OPS_RW_CORR		0x08	/* R-W subchannel data corrected */
#define OPS_C2			0x10	/* C2 error pointers supported */
#define OPS_ISRC		0x20	/* can return the ISRC info */
#define OPS_UPC			0x40	/* can return the catalog number UPC */

	u_int8_t	hw_cap;
#define HW_LOCK			0x01	/* could be locked */
#define HW_LOCKED		0x02	/* current lock state */
#define HW_PREVENT		0x04	/* prevent jumper installed */
#define HW_EJECT		0x08	/* can eject */
#define HW_MECH_MASK		0xe0	/* loading mechanism type mask */

#define MECH_CADDY		0x00
#define MECH_TRAY		0x20
#define MECH_POPUP		0x40
#define MECH_CHANGER		0x80
#define MECH_CARTRIDGE		0xa0

	u_int8_t	sep_cap;
#define SEP_VOL			0x01	/* independent volume controls */
#define SEP_MUTE		0x02	/* independent mute controls */

	u_int16_t	max_speed;	/* max raw data rate in bytes/1000 */
	u_int16_t	max_vol_levels;	/* number of discrete volume levels */
	u_int16_t	buf_size;	/* internal buffer size in bytes/1024 */
	u_int16_t	cur_speed;	/* current data rate in bytes/1000  */

	/* Digital drive output format description (optional?) */
	u_int8_t	reserved3;
	u_int8_t	ddofd;
#define DDOFD_BCKF		0x01	/* data valid on failing edge of BCK */
#define DDOFD_RCH		0x02	/* hight LRCK indicated left channel */
#define DDOFD_LSBF		0x04	/* set if LSB first */
#define DDOFD_DLEN_MASK		0x18	/* mask of DLEN values */

#define DLEN_32			0x00	/* 32 BCKs */
#define DLEN_16			0x08	/* 16 BCKs */
#define DLEN_24			0x10	/* 24 BCKs */
#define DLEN_24_I2S		0x18	/* 24 BCKs (I2S) */

	u_int8_t	reserved4[2];
};

/* ATAPI error codes */
#define ATAPI_SK_NO_SENSE		0x0
#define ATAPI_SK_REC_ERROR		0x1	/* recovered error */
#define ATAPI_SK_NOT_READY		0x2
#define ATAPI_SK_MEDIUM_ERROR		0x3
#define ATAPI_SK_HARDWARE_ERROR		0x4
#define ATAPI_SK_ILLEGAL_REQUEST	0x5
#define ATAPI_SK_UNIT_ATTENTION		0x6
#define ATAPI_SK_DATA_PROTECT		0x7
					/* 0x8 reserved */
					/* 0x9-0xa reserved */
#define ATAPI_SK_ABORTED_COMMAND	0xb
					/* 0xc-0xd not referenced */
#define ATAPI_SK_MISCOMPARE		0xe
					/* 0xf reserved */

#define ATAPI_MCR	0x08	/* media change requested */
#define ATAPI_ABRT	0x04	/* aborted command */
#define ATAPI_EOM	0x02	/* end of media */
#define ATAPI_ILI	0x01	/* illegal length indication */


int	atapi_exec_cmd __P((struct at_dev_link *, void *, int,
	    void *, int, long, int));
int	atapi_exec_io __P((struct at_dev_link *, void *, int,
	    struct buf *, int));
int	atapi_test_unit_ready __P((struct at_dev_link *, int));
int	atapi_start_stop __P((struct at_dev_link *, int, int));
int	atapi_prevent __P((struct at_dev_link *, int));
