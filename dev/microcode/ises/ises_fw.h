/*	$OpenBSD$	*/

/* 
 * ------------------------------------------
 * PCC-ISES Basic Functionality firmware v2.0 
 * ------------------------------------------
 */

/* 
 * To regenerate this file from the *.pcc or *.emb file supplied by the
 * vendor, do the following:
 *  - In the *.pcc or *.emb file, pick out the BCMD_IDPLEN and BCMD_IPDCRC
 *    values and put them in ISES_BF_IDPLEN / ISES_BF_IDPCRC respectively.
 *  - Remove everything from the file except the "firmware data", and feed
 *    the result through this filter:
 *       sed -e 's/$//' -e 's/ /, 0x/g' -e 's/./0x&/' -e 's/.$/,/' | \
 *           xargs -n4 | sed 's/./<TAB>&/'
 */

/*
 * Copyright (C) 1999, 2000 Pijnenburg Custom Chips B.V.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms are permitted provided
 * that the following conditions are met:
 * 1. Redistribution of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistribution in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission
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

#define ISES_BF_IDPLEN		0x00000403	/* Total length, 32bit words */
#define ISES_BF_IDPCRC		0x59DE7DEF	/* Firmware CRC */

static const u_int32_t ises_bf_fw[] = {
	0xEA000006, 0xE1A00000, 0xE1A00000, 0xE1A00000,
	0xE1A00000, 0xE1A00000, 0xEA000013, 0xEA000026,
	0xE59FD0C4, 0xE3A000D2, 0xE121F000, 0xE59FD0BC,
	0xE3A000D1, 0xE121F000, 0xE59FD0B0, 0xE3A000D3,
	0xE121F000, 0xEB00029B, 0xEAFFFFFE, 0xE10F1000,
	0xE1811000, 0xE121F001, 0xE12FFF1E, 0xE10F1000,
	0xE1C11000, 0xE121F001, 0xE12FFF1E, 0xE92D0003,
	0xE59F107C, 0xE5910004, 0xE5810000, 0xE3100902,
	0x0A00000B, 0xE59F106C, 0xE5910000, 0xE3500000,
	0x1A000003, 0xE59F1060, 0xE3A00001, 0xE5810000,
	0xEA000003, 0xE3500001, 0x1A000001, 0xE59F1038,
	0xE5810000, 0xE8BD0003, 0xE25EF004, 0xE25EF004,
	0xE3A000D3, 0xE121F000, 0xE59FE030, 0xE3A00000,
	0xE3A01032, 0xE48E0004, 0xE2511001, 0x1AFFFFFC,
	0xE91E3FFE, 0xE3A0E000, 0xE3A0F000, 0x000017B4,
	0x000017DC, 0xFFFFEA00, 0xFFFFE60C, 0xFFFFF000,
	0x00001714, 0xE3A01001, 0xE3A00AFE, 0xE2400601,
	0xE5A01624, 0xE1A0F00E, 0xE3A00AFE, 0xE2400601,
	0xE5901620, 0xE3510000, 0x0AFFFFFC, 0xE5900610,
	0xE1A0F00E, 0xE3A02000, 0xE3510000, 0x91A0F00E,
	0xE3A03AFE, 0xE2433601, 0xE593C620, 0xE35C0000,
	0x0AFFFFFC, 0xE593C610, 0xE2822001, 0xE1520001,
	0xE480C004, 0x3AFFFFF7, 0xE1A0F00E, 0xE3A01000,
	0xE3A00AFE, 0xE2400601, 0xE5A01618, 0xE1A0F00E,
	0xE3A01002, 0xE3A00AFE, 0xE2400601, 0xE5A01624,
	0xE1A0F00E, 0xE3A01AFE, 0xE2411601, 0xE591261C,
	0xE3520000, 0x1A000006, 0xE3A02506, 0xE5812600,
	0xE591261C, 0xE3520000, 0x0AFFFFFC, 0xE3A02401,
	0xE5812600, 0xE5A10610, 0xE1A0F00E, 0xE3A03000,
	0xE3510000, 0x91A0F00E, 0xE92D4010, 0xE3A0E506,
	0xE3A0C401, 0xE3A02AFE, 0xE2422601, 0xE592461C,
	0xE3540000, 0x1A000004, 0xE582E600, 0xE592461C,
	0xE3540000, 0x0AFFFFFC, 0xE582C600, 0xE4904004,
	0xE5824610, 0xE2833001, 0xE1530001, 0x3AFFFFF2,
	0xE8BD8010, 0xE3A01000, 0xE3A00AFE, 0xE2400601,
	0xE5A01614, 0xE3A00CE6, 0xE2400801, 0xE5901000,
	0xE3811302, 0xE5801000, 0xE5901000, 0xE3C11302,
	0xE5801000, 0xE1A0F00E, 0xE59F2038, 0xE59F0038,
	0xE5901000, 0xE0821181, 0xE5D11006, 0xE20120F0,
	0xE3A03BC2, 0xE2433701, 0xE0832402, 0xE5802008,
	0xE2822004, 0xE580200C, 0xE2432B02, 0xE0821401,
	0xE5A01010, 0xE1A0F00E, 0x00000E00, 0x00001000,
	0xE92D4000, 0xEBFFFFB0, 0xE3A01002, 0xE59F0028,
	0xE5801000, 0xE3A01AFE, 0xE2411601, 0xE5912204,
	0xE5802004, 0xE5911208, 0xE5801008, 0xE3A01003,
	0xEB000264, 0xE3A00000, 0xE8BD8000, 0x00001014,
	0xE92D4000, 0xEBFFFFDB, 0xEBFFFF9F, 0xE51F005C,
	0xE51F205C, 0xE5921000, 0xE0800181, 0xE5D01007,
	0xE3510011, 0x1A000004, 0xE51F0074, 0xE5900008,
	0xE5903000, 0xE3130008, 0x0AFFFFFC, 0xE5B2000C,
	0xE5801000, 0xE3A00000, 0xEB000247, 0xE3A00000,
	0xE8BD8000, 0xE92D4000, 0xEBFFFFC6, 0xEBFFFF8A,
	0xE51F10AC, 0xE5910008, 0xE5902000, 0xE3120008,
	0x0AFFFFFC, 0xE5900000, 0xE3100002, 0x0A000002,
	0xE3A00003, 0xEB000238, 0xEA000002, 0xE5B10010,
	0xE3A01040, 0xEB00023B, 0xE3A00000, 0xE8BD8000,
	0xE92D4010, 0xEBFFFFB3, 0xE51F40F4, 0xE5940008,
	0xE5901000, 0xE3110008, 0x0AFFFFFC, 0xE5900000,
	0xE3100002, 0x0A000005, 0xE5B41004, 0xE51F00D8,
	0xEBFFFF5F, 0xEBFFFF6C, 0xE3A00003, 0xEA00000B,
	0xE5940010, 0xE5941004, 0xEBFFFF59, 0xEBFFFF66,
	0xE51F0140, 0xE5941000, 0xE0800181, 0xE5D02007,
	0xE5B41004, 0xE594000C, 0xEB00024E, 0xE3A00000,
	0xEB000215, 0xE3A00000, 0xE8BD8010, 0xE92D4010,
	0xEBFFFF94, 0xEBFFFF58, 0xE51F0174, 0xE5901008,
	0xE5912000, 0xE3120008, 0x0AFFFFFC, 0xE5911000,
	0xE3110002, 0xE3A04000, 0x13A00003, 0x1A00000F,
	0xE51F11A0, 0xE5902000, 0xE0812182, 0xE5D22007,
	0xE3520034, 0x1A000003, 0xE5902010, 0xE5224088,
	0xE5902010, 0xE5224084, 0xE5902000, 0xE0811182,
	0xE5D11007, 0xE590000C, 0xE5801000, 0xE3A00000,
	0xEB0001F5, 0xE1A00004, 0xE8BD8010, 0xE92D4010,
	0xEBFFFF39, 0xE3A04000, 0xE3A00AFE, 0xE2400601,
	0xE5A04400, 0xE3A00000, 0xEB0001EB, 0xE1A00004,
	0xE8BD8010, 0xE92D4010, 0xE51F01D4, 0xE1A04000,
	0xE3A01002, 0xEBFFFF1E, 0xEBFFFF2B, 0xE3A00AFE,
	0xE2400601, 0xE5901474, 0xE3110002, 0x1A00000F,
	0xE5901424, 0xE3110402, 0x13A00005, 0x1A00000E,
	0xE3A01003, 0xE5801470, 0xE5901474, 0xE3110020,
	0x0AFFFFFC, 0xE5941000, 0xE5801478, 0xE5B41004,
	0xE5801478, 0xE5900474, 0xE3100002, 0x0A000001,
	0xE3A00004, 0xEA000000, 0xE3A00000, 0xEB0001CA,
	0xE3A00000, 0xE8BD8010, 0xE92D4000, 0xEBFFFF0E,
	0xE3A00AFE, 0xE2400601, 0xE5901474, 0xE3110002,
	0xE51F126C, 0x1A00000D, 0xE5902424, 0xE3120402,
	0x13A00005, 0x1A00000C, 0xE5902474, 0xE3120040,
	0x0AFFFFFC, 0xE5902478, 0xE5812000, 0xE5902478,
	0xE5812004, 0xE5900474, 0xE3100002, 0x0A000001,
	0xE3A00004, 0xEA000000, 0xE3A00000, 0xE3500000,
	0x1A000003, 0xE1A00001, 0xE3A01002, 0xEB0001B1,
	0xEA000000, 0xEB0001A8, 0xE3A00000, 0xE8BD8000,
	0xE92D40F0, 0xE51F6320, 0xE5961004, 0xE51F02E8,
	0xE1A04000, 0xEBFFFEDA, 0xEBFFFEE7, 0xE3A05AFE,
	0xE2455601, 0xE3A07000, 0xE5960000, 0xE3500023,
	0x1A000004, 0xE5941000, 0xE3110004, 0x13A00004,
	0x15A50470, 0x1A000018, 0xE3500023, 0x13500031,
	0x0A000002, 0xE5951474, 0xE3110001, 0x1AFFFFFC,
	0xE3500025, 0x1A000003, 0xE5940000, 0xE3C0033F,
	0xE5840000, 0xE5857470, 0xE5960004, 0xE1A02100,
	0xE51F03A0, 0xE5961000, 0xE0800181, 0xE5D00006,
	0xE2400B07, 0xE1A01004, 0xEB0001D0, 0xE5B50474,
	0xE3100002, 0x13A00004, 0x1A000000, 0xE3A00000,
	0xEB000179, 0xE1A00007, 0xE8BD80F0, 0xE92D40F0,
	0xEBFFFEBD, 0xE3A06AFE, 0xE2466601, 0xE5960474,
	0xE3100002, 0xE51F73B0, 0xE51F43F4, 0x15940000,
	0x13500020, 0x1A000014, 0xE5940000, 0xE350002E,
	0x13500030, 0x13500034, 0x1A000002, 0xE5961474,
	0xE3110001, 0x1AFFFFFC, 0xE51F1428, 0xE0810180,
	0xE5D05007, 0xE1A02105, 0xE5D00006, 0xE2401B07,
	0xE1A00007, 0xEB0001AD, 0xE5B60474, 0xE3100002,
	0x15940000, 0x13500020, 0x0A000001, 0xE3A00004,
	0xEA000000, 0xE3A00000, 0xE3500000, 0x1A000003,
	0xE1A01005, 0xE1A00007, 0xEB000156, 0xEA000000,
	0xEB00014D, 0xE3A00000, 0xE8BD80F0, 0xE92D41F0,
	0xE3A04000, 0xE3A00003, 0xE3A08AFE, 0xE2488601,
	0xE5880470, 0xE5980474, 0xE3100002, 0xE51F54A8,
	0x13A04004, 0x1A000006, 0xE5950004, 0xE3100001,
	0x13A04006, 0x1A000002, 0xE5980424, 0xE3100402,
	0x13A04005, 0xE51F7490, 0xE3540000, 0x0A000004,
	0xE1A00007, 0xE5951004, 0xEBFFFE6D, 0xEBFFFE7A,
	0xEA00001D, 0xE3A06000, 0xE5950004, 0xE3500000,
	0xDA000015, 0xE5980474, 0xE3100020, 0x0AFFFFFC,
	0xEBFFFE5C, 0xE5880478, 0xEBFFFE5A, 0xE5880478,
	0xE5980474, 0xE3100004, 0x1A000007, 0xE5980474,
	0xE3100040, 0x0AFFFFFC, 0xE5980478, 0xE7870106,
	0xE5980478, 0xE0871106, 0xE5A10004, 0xE2866002,
	0xE5950004, 0xE1560000, 0xBAFFFFE9, 0xEBFFFE5E,
	0xE5980474, 0xE3100002, 0x13A04004, 0xE3540000,
	0x05B80474, 0x03100004, 0x1A000003, 0xE1A00007,
	0xE5B51004, 0xEB000113, 0xEA000001, 0xE1A00004,
	0xEB000109, 0xE3A00000, 0xE8BD81F0, 0xE92D4010,
	0xEBFFFE4D, 0xE3A00AFE, 0xE2400601, 0xE5901474,
	0xE3110002, 0xE3A04000, 0x1A000013, 0xE5901424,
	0xE3110402, 0x0A000003, 0xE5901474, 0xE3110C02,
	0x0AFFFFFC, 0xEA000002, 0xE5901474, 0xE3110020,
	0x0AFFFFFC, 0xE5901470, 0xE3C11001, 0xE5801470,
	0xE5901474, 0xE3110001, 0x1AFFFFFC, 0xE5804470,
	0xE5900474, 0xE3100002, 0x0A000001, 0xE3A00004,
	0xEA000000, 0xE3A00000, 0xEB0000E7, 0xE1A00004,
	0xE8BD8010, 0xE92D4070, 0xE3A00AFE, 0xE2400601,
	0xE5901474, 0xE3110002, 0xE51F55F4, 0xE3A06000,
	0x0A000005, 0xE3A04004, 0xE1A00005, 0xE3A01012,
	0xEBFFFE13, 0xEBFFFE20, 0xEA000027, 0xE5901424,
	0xE3110402, 0x0A000003, 0xE5901474, 0xE3110C02,
	0x0AFFFFFC, 0xEA000002, 0xE5901474, 0xE3110020,
	0x0AFFFFFC, 0xE5901470, 0xE3C11001, 0xE5801470,
	0xE5901474, 0xE3110001, 0x1AFFFFFC, 0xE1A04000,
	0xE5806470, 0xE1A00005, 0xE3A02048, 0xE3A0100C,
	0xE2411B07, 0xEB000111, 0xE3A01006, 0xE3A0000C,
	0xE2400B07, 0xEBFFFDF6, 0xEBFFFDEE, 0xE3C0033F,
	0xE5840424, 0xE3A0100B, 0xE3A00028, 0xE2400B07,
	0xEBFFFDEF, 0xEBFFFDFC, 0xE3A00003, 0xE5840470,
	0xE5B40474, 0xE2104002, 0x13A04004, 0xE3540000,
	0x1A000003, 0xE1A00005, 0xE3A01012, 0xEB0000B1,
	0xEA000001, 0xE1A00004, 0xEB0000A7, 0xE1A00006,
	0xE8BD8070, 0xE92D4000, 0xEBFFFDEB, 0xE51F172C,
	0xE51F072C, 0xE5900000, 0xE0810180, 0xE5D00007,
	0xE3A01AFE, 0xE2411601, 0xE5A10C00, 0xE3A00000,
	0xEB000099, 0xE3A00000, 0xE8BD8000, 0xE92D4000,
	0xEBFFFDC8, 0xE3A01AFE, 0xE2411601, 0xE5A10C04,
	0xEBFFFDD9, 0xE3A00000, 0xEB00008F, 0xE3A00000,
	0xE8BD8000, 0xE92D41F0, 0xEBFFFDBE, 0xE1A04000,
	0xEBFFFDD1, 0xE3A06000, 0xE35400FF, 0x9A000002,
	0xE3A00007, 0xEB000084, 0xEA00001C, 0xE3A00000,
	0xE51F376C, 0xE3540000, 0x9A000015, 0xE3A07064,
	0xE3A0C080, 0xE3A02AFE, 0xE2422601, 0xE3A0E001,
	0xE3A01000, 0xE5827800, 0xE582C808, 0xE5928A00,
	0xE3180001, 0x0AFFFFFC, 0xE582EA00, 0xE5928C04,
	0xE20880FF, 0xE0885405, 0xE2811001, 0xE3510004,
	0x3AFFFFF3, 0xE7835100, 0xE2800001, 0xE1500004,
	0x3AFFFFEE, 0xE1A01004, 0xE1A00003, 0xEB00006D,
	0xE1A00006, 0xE8BD81F0, 0xE92D47F0, 0xEB0000A1,
	0xE3A09004, 0xE3A08AFE, 0xE2488601, 0xE3A07010,
	0xE1A06088, 0xE2485A01, 0xE3500000, 0x0A000001,
	0xEB000084, 0xEA000015, 0xEB000082, 0xE5889470,
	0xE3A0A000, 0xE59F4040, 0xEBFFFD81, 0xEBFFFD9F,
	0xE3A00001, 0xEB000014, 0xEB000019, 0xE3500002,
	0x0A00000A, 0xE5867804, 0xE5857804, 0xE5889470,
	0xE3A00002, 0xEB00000C, 0xE584A000, 0xE5940000,
	0xE3500000, 0x0AFFFFFC, 0xEAFFFFEE, 0x000017B4,
	0xE5A67804, 0xE5A57804, 0xE5A89470, 0xE3A00003,
	0xEB000001, 0xEAFFFFFF, 0xEAFFFFFE, 0xE1A00C00,
	0xE3A01AFE, 0xE2411601, 0xE5A10600, 0xE1A0F00E,
	0xE1A0F00E, 0xE92D43F0, 0xE3A07001, 0xE3A09AFE,
	0xE2499601, 0xE3A080FF, 0xE59F6030, 0xE59F5030,
	0xE59F4030, 0xE5990618, 0xE3500C01, 0x0A000005,
	0xE5990618, 0xE3500000, 0x9A000007, 0xE5990620,
	0xE3500000, 0x1A000004, 0xE3A00002, 0xEA000019,
	0x00001004, 0x00001000, 0x00000E00, 0xE5990618,
	0xE3500000, 0x0AFFFFFC, 0xEBFFFD4E, 0xE20010FF,
	0xE0082420, 0xE5851000, 0xE1A03800, 0xE1A03823,
	0xE1530820, 0xE5862000, 0x1A000007, 0xE351003E,
	0xD0843181, 0xD5D3C004, 0xD15C0002, 0xCA000002,
	0xE5D33005, 0xE1530002, 0xAA000006, 0xE1A00820,
	0xE1A00800, 0xE3800001, 0xEBFFFD59, 0xEBFFFD7C,
	0xE1A00007, 0xE8BD83F0, 0xE1A0E00F, 0xE794F181,
	0xE3500000, 0x0AFFFFD2, 0xE8BD83F0, 0xE92D4000,
	0xE51F1094, 0xE5911000, 0xE1800801, 0xEBFFFD4C,
	0xE8BD4000, 0xEAFFFD6E, 0xE92D4030, 0xE1A05000,
	0xE1A04001, 0xE1A00C01, 0xE51F10BC, 0xE5911000,
	0xE1800801, 0xEBFFFD42, 0xE1A01004, 0xE1A00005,
	0xEBFFFD4D, 0xE8BD4030, 0xEAFFFD61, 0xE59F1020,
	0xE5910000, 0xE59F201C, 0xE1500002, 0x21A0F00E,
	0xE5922000, 0xE5802000, 0xE3E00000, 0xE5810000,
	0xE1A0F00E, 0x000017FC, 0x000017F8, 0xE92D4010,
	0xE1A04000, 0xEB000010, 0xE3540018, 0x03A00080,
	0x0A000002, 0xE354001C, 0x18BD8010, 0xE3A00040,
	0xE8BD4010, 0xEAFFFCD8, 0xE92D4000, 0xE3A01001,
	0xE3A00018, 0xEBFFFFF0, 0xE3A01902, 0xE3A00AFE,
	0xE2400601, 0xE5A01A08, 0xE8BD8000, 0xE3A000C0,
	0xEAFFFCC9, 0xE1A0F00E, 0xE3A03000, 0xE1510002,
	0x21A0F00E, 0xE7803101, 0xE2811001, 0xE1510002,
	0x3AFFFFFB, 0xE1A0F00E, 0xE3A00000, 0xE1A0F00E,
	0xE3A00001, 0xE1A0F00E, 0xE3A00002, 0xE1A0F00E,
	0xE92D4000, 0xE59F001C, 0xE5901000, 0xE59F0018,
	0xEBFFFCEF, 0xEBFFFCFC, 0xE3A00011, 0xEBFFFFB2,
	0xE3A00000, 0xE8BD8000, 0x00001004, 0x00001014,
	0xE1803001, 0xE1833002, 0xE3130003, 0x1A00000B,
	0xE1A0C122, 0xE1A03000, 0xE24C2001, 0xE35C0000,
	0x91A0F00E, 0xE491C004, 0xE483C004, 0xE1A0C002,
	0xE2422001, 0xE35C0000, 0x8AFFFFF9, 0xE1A0F00E,
	0xE1A03000, 0xE1A0C002, 0xE2422001, 0xE35C0000,
	0x91A0F00E, 0xE4D1C001, 0xE4C3C001, 0xE1A0C002,
	0xE2422001, 0xE35C0000, 0x8AFFFFF9, 0xE1A0F00E,
	0x000002A0, 0x00000000, 0x000002E0, 0x10C80000,
	0x000002E0, 0x11C80000, 0x00000334, 0x00C00000,
	0x00000380, 0x40C04000, 0x00000380, 0x40C14000,
	0x00000380, 0x40C24000, 0x00000380, 0x20C52000,
	0x00000380, 0x20C72000, 0x00000380, 0x22C42200,
	0x00000380, 0x22C62200, 0x00000380, 0x22C32200,
	0x000003FC, 0x34C80000, 0x000003FC, 0x35C80000,
	0x000003FC, 0x36C80000, 0x000003FC, 0x37C80000,
	0x000002E0, 0x10D80000, 0x000002E0, 0x11D80000,
	0x00000334, 0x00D00000, 0x00000380, 0x40D04000,
	0x00000380, 0x40D14000, 0x00000380, 0x40D24000,
	0x00000380, 0x20D52000, 0x00000380, 0x20D72000,
	0x00000380, 0x22D42200, 0x00000380, 0x22D62200,
	0x00000380, 0x22D32200, 0x000003FC, 0x34D80000,
	0x000003FC, 0x35D80000, 0x000003FC, 0x36D80000,
	0x000003FC, 0x37D80000, 0x0000047C, 0x00000000,
	0x0000066C, 0x01000000, 0x000004A4, 0x00000202,
	0x00000528, 0x00000000, 0x000005B0, 0x01700101,
	0x0000066C, 0x01700000, 0x000005B0, 0x01240101,
	0x0000066C, 0x01240000, 0x000005B0, 0x021C0202,
	0x0000066C, 0x021C0000, 0x000005B0, 0x02140202,
	0x0000066C, 0x02140000, 0x000005B0, 0x020C0202,
	0x0000066C, 0x020C0000, 0x000005B0, 0x02280202,
	0x0000066C, 0x02280000, 0x000005B0, 0x02440202,
	0x0000066C, 0x02440000, 0x000005B0, 0x024C0202,
	0x0000066C, 0x024C0000, 0x000005B0, 0x05300505,
	0x0000066C, 0x05300000, 0x0000071C, 0x0000FF00,
	0x0000082C, 0x00000000, 0x000008B4, 0x00001212,
	0x000009B4, 0x01000000, 0x000009B4, 0x00000000,
	0x000009EC, 0x00000101, 0x00000A14, 0x00000101,
	0x00000D58, 0x00004040, 0x00000D60, 0x00002020,
	0x00000D60, 0x0000FF00, 0x00000000, 0x00000000,
	0x00000000, 0x00000000, 0x00000000
};
