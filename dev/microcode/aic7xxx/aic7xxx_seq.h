/* $OpenBSD: aic7xxx.seq,v 1.15 2003/12/24 23:27:55 krw Exp $ */
/*
 * DO NOT EDIT - This file is automatically generated
 *		 from the following source files:
 *
 * $NetBSD: aic7xxx.seq,v 1.15 2003/05/03 18:11:31 wiz Exp $
 * $NetBSD: aic7xxx.reg,v 1.2 2003/04/19 19:26:10 fvdl Exp $
 */
static uint8_t seqprog[] = {
	0xb2, 0x00, 0x00, 0x08,
	0xf7, 0x11, 0x22, 0x08,
	0x00, 0x65, 0xee, 0x59,
	0xf7, 0x01, 0x02, 0x08,
	0xff, 0x6a, 0x24, 0x08,
	0x40, 0x00, 0x40, 0x68,
	0x08, 0x1f, 0x3e, 0x10,
	0x40, 0x00, 0x40, 0x68,
	0xff, 0x40, 0x3c, 0x60,
	0x08, 0x1f, 0x3e, 0x10,
	0x60, 0x0b, 0x42, 0x68,
	0x40, 0xfa, 0x12, 0x78,
	0x01, 0x4d, 0xc8, 0x30,
	0x00, 0x4c, 0x12, 0x70,
	0x01, 0x39, 0xa2, 0x30,
	0x00, 0x6a, 0xd6, 0x5e,
	0x01, 0x51, 0x20, 0x31,
	0x01, 0x57, 0xae, 0x00,
	0x0d, 0x6a, 0x76, 0x00,
	0x00, 0x51, 0x28, 0x5e,
	0x01, 0x51, 0xc8, 0x30,
	0x00, 0x39, 0xc8, 0x60,
	0x00, 0xbb, 0x30, 0x70,
	0xc1, 0x6a, 0xee, 0x5e,
	0x01, 0xbf, 0x72, 0x30,
	0x01, 0x40, 0x7e, 0x31,
	0x01, 0x90, 0x80, 0x30,
	0x01, 0xf6, 0xd4, 0x30,
	0x01, 0x4d, 0x9a, 0x18,
	0xfe, 0x57, 0xae, 0x08,
	0x01, 0x40, 0x20, 0x31,
	0x00, 0x65, 0xcc, 0x58,
	0x60, 0x0b, 0x40, 0x78,
	0x08, 0x6a, 0x18, 0x00,
	0x08, 0x11, 0x22, 0x00,
	0x60, 0x0b, 0x00, 0x78,
	0x40, 0x0b, 0xfa, 0x68,
	0x80, 0x0b, 0xb6, 0x78,
	0x20, 0x6a, 0x16, 0x00,
	0xa4, 0x6a, 0x06, 0x00,
	0x08, 0x6a, 0x78, 0x00,
	0x01, 0x50, 0xc8, 0x30,
	0xe0, 0x6a, 0xcc, 0x00,
	0x48, 0x6a, 0x12, 0x5e,
	0x01, 0x6a, 0xdc, 0x01,
	0x88, 0x6a, 0xcc, 0x00,
	0x48, 0x6a, 0x12, 0x5e,
	0x01, 0x6a, 0x26, 0x01,
	0xf0, 0x19, 0x7a, 0x08,
	0x0f, 0x18, 0xc8, 0x08,
	0x0f, 0x0f, 0xc8, 0x08,
	0x0f, 0x05, 0xc8, 0x08,
	0x00, 0x3d, 0x7a, 0x00,
	0x08, 0x1f, 0x6e, 0x78,
	0x80, 0x3d, 0x7a, 0x00,
	0x01, 0x3d, 0xd8, 0x31,
	0x01, 0x3d, 0x32, 0x31,
	0x10, 0x03, 0x4e, 0x79,
	0x00, 0x65, 0xf2, 0x58,
	0x80, 0x66, 0xae, 0x78,
	0x01, 0x66, 0xd8, 0x31,
	0x01, 0x66, 0x32, 0x31,
	0x1f, 0x66, 0x7c, 0x08,
	0x40, 0x66, 0x82, 0x68,
	0x01, 0x3c, 0x78, 0x00,
	0x10, 0x03, 0x9e, 0x78,
	0x00, 0x65, 0xf2, 0x58,
	0xe0, 0x66, 0xc8, 0x18,
	0x00, 0x65, 0xaa, 0x50,
	0xdd, 0x66, 0xc8, 0x18,
	0x00, 0x65, 0xaa, 0x48,
	0x01, 0x66, 0xd8, 0x31,
	0x01, 0x66, 0x32, 0x31,
	0x10, 0x03, 0x4e, 0x79,
	0x00, 0x65, 0xf2, 0x58,
	0x01, 0x66, 0xd8, 0x31,
	0x01, 0x66, 0x32, 0x31,
	0x01, 0x66, 0xac, 0x30,
	0x40, 0x3c, 0x78, 0x00,
	0xff, 0x6a, 0xd8, 0x01,
	0xff, 0x6a, 0x32, 0x01,
	0x10, 0x3c, 0x78, 0x00,
	0x02, 0x57, 0x40, 0x69,
	0x10, 0x03, 0x3e, 0x69,
	0x00, 0x65, 0x20, 0x41,
	0x02, 0x57, 0xae, 0x00,
	0x00, 0x65, 0x9e, 0x40,
	0x61, 0x6a, 0xee, 0x5e,
	0x08, 0x51, 0x20, 0x71,
	0x02, 0x0b, 0xb2, 0x78,
	0x00, 0x65, 0xae, 0x40,
	0x1a, 0x01, 0x02, 0x00,
	0xf0, 0x19, 0x7a, 0x08,
	0x0f, 0x0f, 0xc8, 0x08,
	0x0f, 0x05, 0xc8, 0x08,
	0x00, 0x3d, 0x7a, 0x00,
	0x08, 0x1f, 0xc4, 0x78,
	0x80, 0x3d, 0x7a, 0x00,
	0x20, 0x6a, 0x16, 0x00,
	0x00, 0x65, 0xcc, 0x41,
	0x00, 0x65, 0xc8, 0x5e,
	0x00, 0x65, 0x12, 0x40,
	0x20, 0x11, 0xd2, 0x68,
	0x20, 0x6a, 0x18, 0x00,
	0x20, 0x11, 0x22, 0x00,
	0xf7, 0x1f, 0xca, 0x08,
	0x80, 0xb9, 0xd8, 0x78,
	0x08, 0x65, 0xca, 0x00,
	0x01, 0x65, 0x3e, 0x30,
	0x01, 0xb9, 0x1e, 0x30,
	0x7f, 0xb9, 0x0a, 0x08,
	0x01, 0xb9, 0x0a, 0x30,
	0x01, 0x54, 0xca, 0x30,
	0x80, 0xb8, 0xe6, 0x78,
	0x80, 0x65, 0xca, 0x00,
	0x01, 0x65, 0x00, 0x34,
	0x01, 0x54, 0x00, 0x34,
	0x08, 0xb8, 0xee, 0x78,
	0x20, 0x01, 0x02, 0x00,
	0x02, 0xbd, 0x08, 0x34,
	0x01, 0xbd, 0x08, 0x34,
	0x08, 0x01, 0x02, 0x00,
	0x02, 0x0b, 0xf4, 0x78,
	0xf7, 0x01, 0x02, 0x08,
	0x01, 0x06, 0xcc, 0x34,
	0xb2, 0x00, 0x00, 0x08,
	0x01, 0x40, 0x20, 0x31,
	0x01, 0xbf, 0x80, 0x30,
	0x01, 0xb9, 0x7a, 0x30,
	0x01, 0xba, 0x7c, 0x30,
	0x00, 0x65, 0xea, 0x58,
	0x80, 0x0b, 0xc4, 0x79,
	0x12, 0x01, 0x02, 0x00,
	0x01, 0xab, 0xac, 0x30,
	0xe4, 0x6a, 0x84, 0x5d,
	0x40, 0x6a, 0x16, 0x00,
	0x80, 0xba, 0x9a, 0x5d,
	0x20, 0xb8, 0x18, 0x79,
	0x20, 0x6a, 0x9a, 0x5d,
	0x00, 0xab, 0x9a, 0x5d,
	0x01, 0xa9, 0x78, 0x30,
	0x10, 0xb8, 0x20, 0x79,
	0xe4, 0x6a, 0x84, 0x5d,
	0x00, 0x65, 0xae, 0x40,
	0x10, 0x03, 0x3c, 0x69,
	0x08, 0x3c, 0x5a, 0x69,
	0x04, 0x3c, 0x92, 0x69,
	0x02, 0x3c, 0x98, 0x69,
	0x01, 0x3c, 0x44, 0x79,
	0xff, 0x6a, 0xa6, 0x00,
	0x00, 0x65, 0xa4, 0x59,
	0x00, 0x6a, 0xd6, 0x5e,
	0xff, 0x53, 0x30, 0x71,
	0x0d, 0x6a, 0x76, 0x00,
	0x00, 0x53, 0x28, 0x5e,
	0x00, 0x65, 0xea, 0x58,
	0x12, 0x01, 0x02, 0x00,
	0x00, 0x65, 0x18, 0x41,
	0xa4, 0x6a, 0x06, 0x00,
	0x00, 0x65, 0xf2, 0x58,
	0xfd, 0x57, 0xae, 0x08,
	0x00, 0x65, 0xae, 0x40,
	0xe4, 0x6a, 0x84, 0x5d,
	0x20, 0x3c, 0x4a, 0x79,
	0x02, 0x6a, 0x9a, 0x5d,
	0x04, 0x6a, 0x9a, 0x5d,
	0x01, 0x03, 0x4c, 0x69,
	0xf7, 0x11, 0x22, 0x08,
	0xff, 0x6a, 0x24, 0x08,
	0xff, 0x6a, 0x06, 0x08,
	0x01, 0x6a, 0x7e, 0x00,
	0x00, 0x65, 0xa4, 0x59,
	0x00, 0x65, 0x04, 0x40,
	0x80, 0x86, 0xc8, 0x08,
	0x01, 0x4f, 0xc8, 0x30,
	0x00, 0x50, 0x6c, 0x61,
	0xc4, 0x6a, 0x84, 0x5d,
	0x40, 0x3c, 0x68, 0x79,
	0x28, 0x6a, 0x9a, 0x5d,
	0x00, 0x65, 0x4c, 0x41,
	0x08, 0x6a, 0x9a, 0x5d,
	0x00, 0x65, 0x4c, 0x41,
	0x84, 0x6a, 0x84, 0x5d,
	0x00, 0x65, 0xf2, 0x58,
	0x01, 0x66, 0xc8, 0x30,
	0x01, 0x64, 0xd8, 0x31,
	0x01, 0x64, 0x32, 0x31,
	0x5b, 0x64, 0xc8, 0x28,
	0x30, 0x64, 0xca, 0x18,
	0x01, 0x6c, 0xc8, 0x30,
	0xff, 0x64, 0x8e, 0x79,
	0x08, 0x01, 0x02, 0x00,
	0x02, 0x0b, 0x80, 0x79,
	0x01, 0x64, 0x86, 0x61,
	0xf7, 0x01, 0x02, 0x08,
	0x01, 0x06, 0xd8, 0x31,
	0x01, 0x06, 0x32, 0x31,
	0xff, 0x64, 0xc8, 0x18,
	0xff, 0x64, 0x80, 0x69,
	0xf7, 0x3c, 0x78, 0x08,
	0x00, 0x65, 0x20, 0x41,
	0x40, 0xaa, 0x7e, 0x10,
	0x04, 0xaa, 0x84, 0x5d,
	0x00, 0x65, 0x60, 0x42,
	0xc4, 0x6a, 0x84, 0x5d,
	0xc0, 0x6a, 0x7e, 0x00,
	0x00, 0xa8, 0x9a, 0x5d,
	0xe4, 0x6a, 0x06, 0x00,
	0x00, 0x6a, 0x9a, 0x5d,
	0x00, 0x65, 0x4c, 0x41,
	0x10, 0x3c, 0xa8, 0x69,
	0x00, 0xbb, 0xa0, 0x44,
	0x18, 0x6a, 0xda, 0x01,
	0x01, 0x69, 0xd8, 0x31,
	0x1c, 0x6a, 0xd0, 0x01,
	0x09, 0xee, 0xdc, 0x01,
	0x80, 0xee, 0xb0, 0x79,
	0xff, 0x6a, 0xdc, 0x09,
	0x01, 0x93, 0x26, 0x01,
	0x03, 0x6a, 0x2a, 0x01,
	0x01, 0x69, 0x32, 0x31,
	0x1c, 0x6a, 0xf6, 0x5d,
	0x0a, 0x93, 0x26, 0x01,
	0x00, 0x65, 0xbe, 0x5e,
	0x01, 0x50, 0xa0, 0x18,
	0x02, 0x6a, 0x22, 0x05,
	0x1a, 0x01, 0x02, 0x00,
	0x80, 0x6a, 0x74, 0x00,
	0x40, 0x6a, 0x78, 0x00,
	0x40, 0x6a, 0x16, 0x00,
	0x00, 0x65, 0xee, 0x5d,
	0x01, 0x3f, 0xc8, 0x30,
	0xbf, 0x64, 0x60, 0x7a,
	0x80, 0x64, 0xb4, 0x73,
	0xa0, 0x64, 0x16, 0x74,
	0xc0, 0x64, 0x0a, 0x74,
	0xe0, 0x64, 0x46, 0x74,
	0x01, 0x6a, 0xee, 0x5e,
	0x00, 0x65, 0xcc, 0x41,
	0xf7, 0x11, 0x22, 0x08,
	0x01, 0x06, 0xd4, 0x30,
	0xff, 0x6a, 0x24, 0x08,
	0xf7, 0x01, 0x02, 0x08,
	0xc0, 0x6a, 0x78, 0x00,
	0x09, 0x0c, 0xe8, 0x79,
	0x08, 0x0c, 0x04, 0x68,
	0xb1, 0x6a, 0xee, 0x5e,
	0xff, 0x6a, 0x26, 0x09,
	0x12, 0x01, 0x02, 0x00,
	0x02, 0x6a, 0x08, 0x30,
	0xff, 0x6a, 0x08, 0x08,
	0xdf, 0x01, 0x02, 0x08,
	0x01, 0x6a, 0x7e, 0x00,
	0xc0, 0x6a, 0x78, 0x04,
	0xff, 0x6a, 0xc8, 0x08,
	0x08, 0xa4, 0x48, 0x19,
	0x00, 0xa5, 0x4a, 0x21,
	0x00, 0xa6, 0x4c, 0x21,
	0x00, 0xa7, 0x4e, 0x25,
	0x08, 0xeb, 0xf2, 0x7e,
	0x80, 0xeb, 0x08, 0x7a,
	0xff, 0x6a, 0xd6, 0x09,
	0x08, 0xeb, 0x0c, 0x6a,
	0xff, 0x6a, 0xd4, 0x0c,
	0x80, 0xa3, 0xf2, 0x6e,
	0x88, 0xeb, 0x22, 0x72,
	0x08, 0xeb, 0xf2, 0x6e,
	0x04, 0xea, 0x26, 0xe2,
	0x08, 0xee, 0xf2, 0x6e,
	0x04, 0x6a, 0xd0, 0x81,
	0x05, 0xa4, 0xc0, 0x89,
	0x03, 0xa5, 0xc2, 0x31,
	0x09, 0x6a, 0xd6, 0x05,
	0x00, 0x65, 0x0a, 0x5a,
	0x06, 0xa4, 0xd4, 0x89,
	0x80, 0x94, 0xf2, 0x7e,
	0x07, 0xe9, 0x10, 0x31,
	0x01, 0x8c, 0x2e, 0x7a,
	0x01, 0x55, 0xaa, 0x10,
	0x01, 0xe9, 0x46, 0x31,
	0x00, 0xa3, 0xd0, 0x5e,
	0x00, 0x65, 0xfc, 0x59,
	0x01, 0xa4, 0xca, 0x30,
	0x01, 0x55, 0x3a, 0x7a,
	0x04, 0x65, 0xca, 0x00,
	0x80, 0xa3, 0x3e, 0x7a,
	0x02, 0x65, 0xca, 0x00,
	0x01, 0x65, 0xf8, 0x31,
	0x80, 0x93, 0x26, 0x01,
	0xff, 0x6a, 0xd4, 0x0c,
	0x01, 0x8c, 0xc8, 0x30,
	0x00, 0x88, 0xc8, 0x18,
	0x02, 0x64, 0xc8, 0x88,
	0xff, 0x64, 0xf2, 0x7e,
	0xff, 0x8d, 0x54, 0x6a,
	0xff, 0x8e, 0x54, 0x6a,
	0x03, 0x8c, 0xd4, 0x98,
	0x00, 0x65, 0xf2, 0x56,
	0x01, 0x64, 0x70, 0x30,
	0xff, 0x64, 0xc8, 0x10,
	0x01, 0x64, 0xc8, 0x18,
	0x00, 0x8c, 0x18, 0x19,
	0xff, 0x8d, 0x1a, 0x21,
	0xff, 0x8e, 0x1c, 0x25,
	0xc0, 0x3c, 0x64, 0x7a,
	0x21, 0x6a, 0xee, 0x5e,
	0xa8, 0x6a, 0x76, 0x00,
	0x79, 0x6a, 0x76, 0x00,
	0x40, 0x3f, 0x6c, 0x6a,
	0x04, 0x3b, 0x76, 0x00,
	0x04, 0x6a, 0xd4, 0x81,
	0x20, 0x3c, 0x74, 0x7a,
	0x51, 0x6a, 0xee, 0x5e,
	0x00, 0x65, 0x8e, 0x42,
	0x20, 0x3c, 0x78, 0x00,
	0x00, 0xb3, 0xd0, 0x5e,
	0x07, 0xac, 0x10, 0x31,
	0x05, 0xb3, 0x46, 0x31,
	0x88, 0x6a, 0xcc, 0x00,
	0xac, 0x6a, 0x04, 0x5e,
	0xa3, 0x6a, 0xcc, 0x00,
	0xb3, 0x6a, 0x08, 0x5e,
	0x00, 0x65, 0x44, 0x5a,
	0xfd, 0xa4, 0x48, 0x09,
	0x01, 0x8c, 0xaa, 0x08,
	0x03, 0x8c, 0x10, 0x30,
	0x00, 0x65, 0xfc, 0x5d,
	0x01, 0xa4, 0xa0, 0x7a,
	0x04, 0x3b, 0x76, 0x08,
	0x01, 0x3b, 0x26, 0x31,
	0x80, 0x02, 0x04, 0x00,
	0x10, 0x0c, 0x96, 0x7a,
	0x03, 0x9e, 0x98, 0x6a,
	0x7f, 0x02, 0x04, 0x08,
	0x91, 0x6a, 0xee, 0x5e,
	0x00, 0x65, 0xcc, 0x41,
	0x01, 0xa4, 0xca, 0x30,
	0x80, 0xa3, 0xa6, 0x7a,
	0x02, 0x65, 0xca, 0x00,
	0x01, 0x55, 0xaa, 0x7a,
	0x04, 0x65, 0xca, 0x00,
	0x01, 0x65, 0xf8, 0x31,
	0x01, 0x3b, 0x26, 0x31,
	0x00, 0x65, 0x10, 0x5a,
	0x01, 0xfc, 0xb8, 0x6a,
	0x80, 0x0b, 0xae, 0x6a,
	0x10, 0x0c, 0xae, 0x7a,
	0x20, 0x93, 0xae, 0x6a,
	0x02, 0x93, 0x26, 0x01,
	0x02, 0xfc, 0xc2, 0x7a,
	0x40, 0x0d, 0xdc, 0x6a,
	0x01, 0xa4, 0x48, 0x01,
	0x00, 0x65, 0xdc, 0x42,
	0x40, 0x0d, 0xc8, 0x6a,
	0x00, 0x65, 0x10, 0x5a,
	0x00, 0x65, 0xba, 0x42,
	0x80, 0xfc, 0xd2, 0x7a,
	0x80, 0xa4, 0xd2, 0x6a,
	0xff, 0xa5, 0x4a, 0x19,
	0xff, 0xa6, 0x4c, 0x21,
	0xff, 0xa7, 0x4e, 0x21,
	0xf8, 0xfc, 0x48, 0x09,
	0xff, 0x6a, 0xaa, 0x08,
	0x04, 0xfc, 0xda, 0x7a,
	0x01, 0x55, 0xaa, 0x00,
	0xff, 0x6a, 0x46, 0x09,
	0x04, 0x3b, 0xf4, 0x6a,
	0x02, 0x93, 0x26, 0x01,
	0x01, 0x94, 0xde, 0x7a,
	0x01, 0x94, 0xde, 0x7a,
	0x01, 0x94, 0xde, 0x7a,
	0x01, 0x94, 0xde, 0x7a,
	0x01, 0x94, 0xde, 0x7a,
	0x01, 0xa4, 0xf2, 0x7a,
	0x01, 0xfc, 0xec, 0x7a,
	0x01, 0x94, 0xf4, 0x6a,
	0x00, 0x65, 0x8e, 0x42,
	0x01, 0x94, 0xf2, 0x7a,
	0x10, 0x94, 0xf4, 0x6a,
	0xd7, 0x93, 0x26, 0x09,
	0x28, 0x93, 0xf8, 0x6a,
	0x01, 0x85, 0x0a, 0x01,
	0x02, 0xfc, 0x00, 0x6b,
	0x01, 0x14, 0x46, 0x31,
	0xff, 0x6a, 0x10, 0x09,
	0xfe, 0x85, 0x0a, 0x09,
	0xff, 0x38, 0x0e, 0x6b,
	0x80, 0xa3, 0x0e, 0x7b,
	0x80, 0x0b, 0x0c, 0x7b,
	0x04, 0x3b, 0x0e, 0x7b,
	0xbf, 0x3b, 0x76, 0x08,
	0x01, 0x3b, 0x26, 0x31,
	0x00, 0x65, 0x10, 0x5a,
	0x01, 0x0b, 0x1c, 0x6b,
	0x10, 0x0c, 0x10, 0x7b,
	0x04, 0x93, 0x1a, 0x6b,
	0x01, 0x94, 0x18, 0x7b,
	0x10, 0x94, 0x1a, 0x6b,
	0xc7, 0x93, 0x26, 0x09,
	0x01, 0x99, 0xd4, 0x30,
	0x38, 0x93, 0x1e, 0x6b,
	0xff, 0x08, 0x70, 0x6b,
	0xff, 0x09, 0x70, 0x6b,
	0xff, 0x0a, 0x70, 0x6b,
	0xff, 0x38, 0x3a, 0x7b,
	0x04, 0x14, 0x10, 0x31,
	0x01, 0x38, 0x18, 0x31,
	0x02, 0x6a, 0x1a, 0x31,
	0x88, 0x6a, 0xcc, 0x00,
	0x14, 0x6a, 0x0a, 0x5e,
	0x00, 0x38, 0xf6, 0x5d,
	0xff, 0x6a, 0x70, 0x08,
	0x00, 0x65, 0x66, 0x43,
	0x80, 0xa3, 0x40, 0x7b,
	0x01, 0xa4, 0x48, 0x01,
	0x00, 0x65, 0x70, 0x43,
	0x08, 0xeb, 0x46, 0x7b,
	0x00, 0x65, 0x10, 0x5a,
	0x08, 0xeb, 0x42, 0x6b,
	0x07, 0xe9, 0x10, 0x31,
	0x01, 0xe9, 0xca, 0x30,
	0x01, 0x65, 0x46, 0x31,
	0x00, 0x6a, 0xd0, 0x5e,
	0x88, 0x6a, 0xcc, 0x00,
	0xa4, 0x6a, 0x0a, 0x5e,
	0x08, 0x6a, 0xf6, 0x5d,
	0x0d, 0x93, 0x26, 0x01,
	0x00, 0x65, 0xbe, 0x5e,
	0x88, 0x6a, 0xcc, 0x00,
	0x00, 0x65, 0xa0, 0x5e,
	0x01, 0x99, 0x46, 0x31,
	0x00, 0xa3, 0xd0, 0x5e,
	0x01, 0x88, 0x10, 0x31,
	0x00, 0x65, 0x44, 0x5a,
	0x00, 0x65, 0xfc, 0x59,
	0x03, 0x8c, 0x10, 0x30,
	0x00, 0x65, 0xfc, 0x5d,
	0x01, 0x8c, 0x6e, 0x7b,
	0x01, 0x55, 0xaa, 0x10,
	0x80, 0x0b, 0x8e, 0x6a,
	0x80, 0x0b, 0x78, 0x6b,
	0x01, 0x0c, 0x72, 0x7b,
	0x10, 0x0c, 0x8e, 0x7a,
	0x03, 0x9e, 0x8e, 0x6a,
	0x00, 0x65, 0x06, 0x5a,
	0x00, 0x6a, 0xd0, 0x5e,
	0x01, 0xa4, 0x98, 0x6b,
	0xff, 0x38, 0x8e, 0x7b,
	0x01, 0x38, 0xc8, 0x30,
	0x00, 0x08, 0x40, 0x19,
	0xff, 0x6a, 0xc8, 0x08,
	0x00, 0x09, 0x42, 0x21,
	0x00, 0x0a, 0x44, 0x21,
	0xff, 0x6a, 0x70, 0x08,
	0x00, 0x65, 0x90, 0x43,
	0x03, 0x08, 0x40, 0x31,
	0x03, 0x08, 0x40, 0x31,
	0x01, 0x08, 0x40, 0x31,
	0x01, 0x09, 0x42, 0x31,
	0x01, 0x0a, 0x44, 0x31,
	0xfd, 0xb4, 0x68, 0x09,
	0x12, 0x01, 0x02, 0x00,
	0x12, 0x01, 0x02, 0x00,
	0x04, 0x3c, 0xcc, 0x79,
	0xfb, 0x3c, 0x78, 0x08,
	0x04, 0x93, 0x20, 0x79,
	0x01, 0x0c, 0xa4, 0x6b,
	0x01, 0x55, 0x20, 0x79,
	0x80, 0x04, 0x20, 0x79,
	0xe4, 0x6a, 0x84, 0x5d,
	0x23, 0x6a, 0x9a, 0x5d,
	0x01, 0x6a, 0x9a, 0x5d,
	0x00, 0x65, 0x20, 0x41,
	0x00, 0x65, 0xcc, 0x41,
	0x80, 0x3c, 0xb8, 0x7b,
	0x21, 0x6a, 0xee, 0x5e,
	0x01, 0xbc, 0x18, 0x31,
	0x02, 0x6a, 0x1a, 0x31,
	0x02, 0x6a, 0xf8, 0x01,
	0x01, 0xbc, 0x10, 0x30,
	0x02, 0x6a, 0x12, 0x30,
	0x01, 0xbc, 0x10, 0x30,
	0xff, 0x6a, 0x12, 0x08,
	0xff, 0x6a, 0x14, 0x08,
	0xf3, 0xbc, 0xd4, 0x18,
	0xa0, 0x6a, 0xde, 0x53,
	0x04, 0xa0, 0x10, 0x31,
	0xac, 0x6a, 0x26, 0x01,
	0x04, 0xa0, 0x10, 0x31,
	0x03, 0x08, 0x18, 0x31,
	0x88, 0x6a, 0xcc, 0x00,
	0xa0, 0x6a, 0x0a, 0x5e,
	0x00, 0xbc, 0xf6, 0x5d,
	0x3d, 0x6a, 0x26, 0x01,
	0x00, 0x65, 0xf6, 0x43,
	0xff, 0x6a, 0x10, 0x09,
	0xa4, 0x6a, 0x26, 0x01,
	0x0c, 0xa0, 0x32, 0x31,
	0x05, 0x6a, 0x26, 0x01,
	0x35, 0x6a, 0x26, 0x01,
	0x0c, 0xa0, 0x32, 0x31,
	0x36, 0x6a, 0x26, 0x01,
	0x02, 0x93, 0x26, 0x01,
	0x35, 0x6a, 0x26, 0x01,
	0x00, 0x65, 0xb2, 0x5e,
	0x00, 0x65, 0xb2, 0x5e,
	0x02, 0x93, 0x26, 0x01,
	0xbf, 0x3c, 0x78, 0x08,
	0x04, 0x0b, 0xfc, 0x6b,
	0x10, 0x0c, 0xf8, 0x7b,
	0x01, 0x03, 0xfc, 0x6b,
	0x20, 0x93, 0xfe, 0x6b,
	0x04, 0x0b, 0x04, 0x6c,
	0x40, 0x3c, 0x78, 0x00,
	0xc7, 0x93, 0x26, 0x09,
	0x38, 0x93, 0x06, 0x6c,
	0x00, 0x65, 0xcc, 0x41,
	0x80, 0x3c, 0x6c, 0x6c,
	0x01, 0x06, 0x50, 0x31,
	0x80, 0xb8, 0x70, 0x01,
	0x00, 0x65, 0xcc, 0x41,
	0x10, 0x3f, 0x06, 0x00,
	0x10, 0x6a, 0x06, 0x00,
	0x01, 0x3a, 0xca, 0x30,
	0x80, 0x65, 0x32, 0x64,
	0x10, 0xb8, 0x56, 0x6c,
	0xc0, 0xba, 0xca, 0x00,
	0x40, 0xb8, 0x22, 0x6c,
	0xbf, 0x65, 0xca, 0x08,
	0x20, 0xb8, 0x36, 0x7c,
	0x01, 0x65, 0x0c, 0x30,
	0x00, 0x65, 0xee, 0x5d,
	0xa0, 0x3f, 0x3e, 0x64,
	0x23, 0xb8, 0x0c, 0x08,
	0x00, 0x65, 0xee, 0x5d,
	0xa0, 0x3f, 0x3e, 0x64,
	0x00, 0xbb, 0x36, 0x44,
	0xff, 0x65, 0x36, 0x64,
	0x00, 0x65, 0x56, 0x44,
	0x40, 0x6a, 0x18, 0x00,
	0x01, 0x65, 0x0c, 0x30,
	0x00, 0x65, 0xee, 0x5d,
	0xa0, 0x3f, 0x12, 0x74,
	0x40, 0x6a, 0x18, 0x00,
	0x01, 0x3a, 0xa6, 0x30,
	0x08, 0x6a, 0x74, 0x00,
	0x00, 0x65, 0xcc, 0x41,
	0x64, 0x6a, 0x7e, 0x5d,
	0x80, 0x64, 0xee, 0x6c,
	0x04, 0x64, 0xb0, 0x74,
	0x02, 0x64, 0xc0, 0x74,
	0x00, 0x6a, 0x76, 0x74,
	0x03, 0x64, 0xde, 0x74,
	0x23, 0x64, 0x5e, 0x74,
	0x08, 0x64, 0x72, 0x74,
	0x61, 0x6a, 0xee, 0x5e,
	0x00, 0x65, 0xee, 0x5d,
	0x08, 0x51, 0xce, 0x71,
	0x00, 0x65, 0x56, 0x44,
	0x80, 0x04, 0x70, 0x7c,
	0x51, 0x6a, 0x74, 0x5d,
	0x01, 0x51, 0x70, 0x64,
	0x01, 0xa4, 0x68, 0x7c,
	0x01, 0x55, 0x72, 0x7c,
	0x41, 0x6a, 0xee, 0x5e,
	0x00, 0x65, 0x72, 0x44,
	0x21, 0x6a, 0xee, 0x5e,
	0x00, 0x65, 0x72, 0x44,
	0x07, 0x6a, 0x6a, 0x5d,
	0x01, 0x06, 0xd4, 0x30,
	0x00, 0x65, 0xcc, 0x41,
	0x80, 0xb8, 0x6c, 0x7c,
	0xc0, 0x3c, 0x80, 0x7c,
	0x80, 0x3c, 0x6c, 0x6c,
	0xff, 0xa8, 0x80, 0x6c,
	0x40, 0x3c, 0x6c, 0x6c,
	0x10, 0xb8, 0x84, 0x7c,
	0xa1, 0x6a, 0xee, 0x5e,
	0x01, 0xb4, 0x8a, 0x6c,
	0x02, 0xb4, 0x8c, 0x6c,
	0x01, 0xa4, 0x8c, 0x7c,
	0xff, 0xa8, 0x9c, 0x7c,
	0x04, 0xb4, 0x68, 0x01,
	0x01, 0x6a, 0x76, 0x00,
	0x00, 0xbb, 0x28, 0x5e,
	0xff, 0xa8, 0x9c, 0x7c,
	0x71, 0x6a, 0xee, 0x5e,
	0x40, 0x51, 0x9c, 0x64,
	0x00, 0x65, 0xc8, 0x5e,
	0x00, 0x65, 0xde, 0x41,
	0x00, 0xbb, 0xa0, 0x5c,
	0x00, 0x65, 0xde, 0x41,
	0x00, 0x65, 0xc8, 0x5e,
	0x01, 0x65, 0xa2, 0x30,
	0x01, 0xf8, 0xc8, 0x30,
	0x01, 0x4e, 0xc8, 0x30,
	0x00, 0x6a, 0xcc, 0xdd,
	0x00, 0x51, 0xde, 0x5d,
	0x01, 0x4e, 0x9c, 0x18,
	0x02, 0x6a, 0x22, 0x05,
	0xc0, 0x3c, 0x6c, 0x6c,
	0x04, 0xb8, 0x70, 0x01,
	0x00, 0x65, 0xea, 0x5e,
	0x20, 0xb8, 0xde, 0x69,
	0x01, 0xbb, 0xa2, 0x30,
	0x01, 0xba, 0x7c, 0x30,
	0x00, 0xb9, 0xe4, 0x5c,
	0x00, 0x65, 0xde, 0x41,
	0x01, 0x06, 0xd4, 0x30,
	0x20, 0x3c, 0xcc, 0x79,
	0x20, 0x3c, 0x72, 0x7c,
	0x01, 0xa4, 0xce, 0x7c,
	0x01, 0xb4, 0x68, 0x01,
	0x00, 0x65, 0xcc, 0x41,
	0x00, 0x65, 0x72, 0x44,
	0x04, 0x14, 0x58, 0x31,
	0x01, 0x06, 0xd4, 0x30,
	0x08, 0xa0, 0x60, 0x31,
	0xac, 0x6a, 0xcc, 0x00,
	0x14, 0x6a, 0x0a, 0x5e,
	0x01, 0x06, 0xd4, 0x30,
	0xa0, 0x6a, 0x02, 0x5e,
	0x00, 0x65, 0xcc, 0x41,
	0xdf, 0x3c, 0x78, 0x08,
	0x12, 0x01, 0x02, 0x00,
	0x00, 0x65, 0x72, 0x44,
	0x4c, 0x65, 0xcc, 0x28,
	0x01, 0x3e, 0x20, 0x31,
	0xd0, 0x66, 0xcc, 0x18,
	0x20, 0x66, 0xcc, 0x18,
	0x01, 0x51, 0xda, 0x34,
	0x4c, 0x3d, 0xca, 0x28,
	0x1f, 0x64, 0x7c, 0x08,
	0xd0, 0x65, 0xca, 0x18,
	0x01, 0x3e, 0x20, 0x31,
	0x30, 0x65, 0xd4, 0x18,
	0x00, 0x65, 0xfc, 0x4c,
	0xe1, 0x6a, 0x22, 0x01,
	0xff, 0x6a, 0xd4, 0x08,
	0x20, 0x65, 0xd4, 0x18,
	0x00, 0x65, 0x04, 0x55,
	0xe1, 0x6a, 0x22, 0x01,
	0xff, 0x6a, 0xd4, 0x08,
	0x20, 0x65, 0xca, 0x18,
	0xe0, 0x65, 0xd4, 0x18,
	0x00, 0x65, 0x0e, 0x4d,
	0xe1, 0x6a, 0x22, 0x01,
	0xff, 0x6a, 0xd4, 0x08,
	0xd0, 0x65, 0xd4, 0x18,
	0x00, 0x65, 0x16, 0x55,
	0xe1, 0x6a, 0x22, 0x01,
	0xff, 0x6a, 0xd4, 0x08,
	0x01, 0x6c, 0xa2, 0x30,
	0xff, 0x51, 0x28, 0x75,
	0x00, 0x51, 0xa4, 0x5d,
	0x01, 0x51, 0x20, 0x31,
	0x00, 0x65, 0x4a, 0x45,
	0x01, 0xba, 0xc8, 0x30,
	0x00, 0x3e, 0x4a, 0x75,
	0x00, 0x65, 0xc6, 0x5e,
	0x80, 0x3c, 0x78, 0x00,
	0x01, 0x06, 0xd4, 0x30,
	0x00, 0x65, 0xee, 0x5d,
	0x01, 0x3c, 0x78, 0x00,
	0xe0, 0x3f, 0x66, 0x65,
	0x02, 0x3c, 0x78, 0x00,
	0x20, 0x12, 0x66, 0x65,
	0x51, 0x6a, 0x74, 0x5d,
	0x00, 0x51, 0xa4, 0x5d,
	0x51, 0x6a, 0x74, 0x5d,
	0x01, 0x51, 0x20, 0x31,
	0x04, 0x3c, 0x78, 0x00,
	0x01, 0xb9, 0xc8, 0x30,
	0x00, 0x3d, 0x64, 0x65,
	0x08, 0x3c, 0x78, 0x00,
	0x01, 0xba, 0xc8, 0x30,
	0x00, 0x3e, 0x64, 0x65,
	0x10, 0x3c, 0x78, 0x00,
	0x04, 0xb8, 0x64, 0x7d,
	0xfb, 0xb8, 0x70, 0x09,
	0x20, 0xb8, 0x5a, 0x6d,
	0x01, 0x90, 0xc8, 0x30,
	0xff, 0x6a, 0xa2, 0x00,
	0x00, 0x3d, 0xe4, 0x5c,
	0x01, 0x64, 0x20, 0x31,
	0xff, 0x6a, 0x78, 0x08,
	0x00, 0x65, 0xea, 0x58,
	0x10, 0xb8, 0x72, 0x7c,
	0xff, 0x6a, 0x6a, 0x5d,
	0x00, 0x65, 0x72, 0x44,
	0x00, 0x65, 0xc6, 0x5e,
	0x31, 0x6a, 0xee, 0x5e,
	0x00, 0x65, 0x72, 0x44,
	0x10, 0x3f, 0x06, 0x00,
	0x10, 0x6a, 0x06, 0x00,
	0x01, 0x65, 0x74, 0x34,
	0x81, 0x6a, 0xee, 0x5e,
	0x00, 0x65, 0x76, 0x45,
	0x01, 0x06, 0xd4, 0x30,
	0x01, 0x0c, 0x76, 0x7d,
	0x04, 0x0c, 0x70, 0x6d,
	0xe0, 0x03, 0x7e, 0x08,
	0xe0, 0x3f, 0xcc, 0x61,
	0x01, 0x65, 0xcc, 0x30,
	0x01, 0x12, 0xda, 0x34,
	0x01, 0x06, 0xd4, 0x34,
	0x01, 0x03, 0x84, 0x6d,
	0x40, 0x03, 0xcc, 0x08,
	0x01, 0x65, 0x06, 0x30,
	0x40, 0x65, 0xc8, 0x08,
	0x00, 0x66, 0x92, 0x75,
	0x40, 0x65, 0x92, 0x7d,
	0x00, 0x65, 0x92, 0x5d,
	0xff, 0x6a, 0xd4, 0x08,
	0xff, 0x6a, 0xd4, 0x08,
	0xff, 0x6a, 0xd4, 0x08,
	0xff, 0x6a, 0xd4, 0x0c,
	0x08, 0x01, 0x02, 0x00,
	0x02, 0x0b, 0x9c, 0x7d,
	0x01, 0x65, 0x0c, 0x30,
	0x02, 0x0b, 0xa0, 0x7d,
	0xf7, 0x01, 0x02, 0x0c,
	0x01, 0x65, 0xc8, 0x30,
	0xff, 0x41, 0xc4, 0x75,
	0x01, 0x41, 0x20, 0x31,
	0xff, 0x6a, 0xa4, 0x00,
	0x00, 0x65, 0xb4, 0x45,
	0xff, 0xbf, 0xc4, 0x75,
	0x01, 0x90, 0xa4, 0x30,
	0x01, 0xbf, 0x20, 0x31,
	0x00, 0xbb, 0xae, 0x65,
	0xff, 0x52, 0xc2, 0x75,
	0x01, 0xbf, 0xcc, 0x30,
	0x01, 0x90, 0xca, 0x30,
	0x01, 0x52, 0x20, 0x31,
	0x01, 0x66, 0x7e, 0x31,
	0x01, 0x65, 0x20, 0x35,
	0x01, 0xbf, 0x82, 0x34,
	0x01, 0x64, 0xa2, 0x30,
	0x00, 0x6a, 0xd6, 0x5e,
	0x0d, 0x6a, 0x76, 0x00,
	0x00, 0x51, 0x28, 0x46,
	0x01, 0x65, 0xa4, 0x30,
	0xe0, 0x6a, 0xcc, 0x00,
	0x48, 0x6a, 0x1c, 0x5e,
	0x01, 0x6a, 0xd0, 0x01,
	0x01, 0x6a, 0xdc, 0x05,
	0x88, 0x6a, 0xcc, 0x00,
	0x48, 0x6a, 0x1c, 0x5e,
	0x01, 0x6a, 0xf6, 0x5d,
	0x01, 0x6a, 0x26, 0x05,
	0x01, 0x65, 0xd8, 0x31,
	0x09, 0xee, 0xdc, 0x01,
	0x80, 0xee, 0xe2, 0x7d,
	0xff, 0x6a, 0xdc, 0x0d,
	0x01, 0x65, 0x32, 0x31,
	0x0a, 0x93, 0x26, 0x01,
	0x00, 0x65, 0xbe, 0x46,
	0x81, 0x6a, 0xee, 0x5e,
	0x01, 0x0c, 0xee, 0x7d,
	0x04, 0x0c, 0xec, 0x6d,
	0xe0, 0x03, 0x06, 0x08,
	0xe0, 0x03, 0x7e, 0x0c,
	0x01, 0x65, 0x18, 0x31,
	0xff, 0x6a, 0x1a, 0x09,
	0xff, 0x6a, 0x1c, 0x0d,
	0x01, 0x8c, 0x10, 0x30,
	0x01, 0x8d, 0x12, 0x30,
	0x01, 0x8e, 0x14, 0x34,
	0x01, 0x6c, 0xda, 0x30,
	0x01, 0x6c, 0xda, 0x30,
	0x01, 0x6c, 0xda, 0x30,
	0x01, 0x6c, 0xda, 0x30,
	0x01, 0x6c, 0xda, 0x30,
	0x01, 0x6c, 0xda, 0x30,
	0x01, 0x6c, 0xda, 0x30,
	0x01, 0x6c, 0xda, 0x34,
	0x3d, 0x64, 0xa4, 0x28,
	0x55, 0x64, 0xc8, 0x28,
	0x00, 0x65, 0x1c, 0x46,
	0x2e, 0x64, 0xa4, 0x28,
	0x66, 0x64, 0xc8, 0x28,
	0x00, 0x6c, 0xda, 0x18,
	0x01, 0x52, 0xc8, 0x30,
	0x00, 0x6c, 0xda, 0x20,
	0xff, 0x6a, 0xc8, 0x08,
	0x00, 0x6c, 0xda, 0x20,
	0x00, 0x6c, 0xda, 0x24,
	0x01, 0x65, 0xc8, 0x30,
	0xe0, 0x6a, 0xcc, 0x00,
	0x44, 0x6a, 0x18, 0x5e,
	0x01, 0x90, 0xe2, 0x31,
	0x04, 0x3b, 0x3c, 0x7e,
	0x30, 0x6a, 0xd0, 0x01,
	0x20, 0x6a, 0xd0, 0x01,
	0x1d, 0x6a, 0xdc, 0x01,
	0xdc, 0xee, 0x38, 0x66,
	0x00, 0x65, 0x54, 0x46,
	0x20, 0x6a, 0xd0, 0x01,
	0x01, 0x6a, 0xdc, 0x01,
	0x20, 0xa0, 0xd8, 0x31,
	0x09, 0xee, 0xdc, 0x01,
	0x80, 0xee, 0x44, 0x7e,
	0x11, 0x6a, 0xdc, 0x01,
	0x50, 0xee, 0x48, 0x66,
	0x20, 0x6a, 0xd0, 0x01,
	0x09, 0x6a, 0xdc, 0x01,
	0x88, 0xee, 0x4e, 0x66,
	0x19, 0x6a, 0xdc, 0x01,
	0xd8, 0xee, 0x52, 0x66,
	0xff, 0x6a, 0xdc, 0x09,
	0x18, 0xee, 0x56, 0x6e,
	0xff, 0x6a, 0xd4, 0x0c,
	0x88, 0x6a, 0xcc, 0x00,
	0x44, 0x6a, 0x18, 0x5e,
	0x20, 0x6a, 0xf6, 0x5d,
	0x01, 0x3b, 0x26, 0x31,
	0x04, 0x3b, 0x70, 0x6e,
	0xa0, 0x6a, 0xca, 0x00,
	0x20, 0x65, 0xc8, 0x18,
	0x00, 0x65, 0xae, 0x5e,
	0x00, 0x65, 0x68, 0x66,
	0x0a, 0x93, 0x26, 0x01,
	0x00, 0x65, 0xbe, 0x46,
	0xa0, 0x6a, 0xcc, 0x00,
	0xff, 0x6a, 0xc8, 0x08,
	0x20, 0x94, 0x74, 0x6e,
	0x10, 0x94, 0x76, 0x6e,
	0x08, 0x94, 0x90, 0x6e,
	0x08, 0x94, 0x90, 0x6e,
	0x08, 0x94, 0x90, 0x6e,
	0xff, 0x8c, 0xc8, 0x10,
	0xc1, 0x64, 0xc8, 0x18,
	0xf8, 0x64, 0xc8, 0x08,
	0x01, 0x99, 0xda, 0x30,
	0x00, 0x66, 0x84, 0x66,
	0xc0, 0x66, 0xc0, 0x76,
	0x60, 0x66, 0xc8, 0x18,
	0x3d, 0x64, 0xc8, 0x28,
	0x00, 0x65, 0x74, 0x46,
	0xf7, 0x93, 0x26, 0x09,
	0x08, 0x93, 0x92, 0x6e,
	0x00, 0x62, 0xc4, 0x18,
	0x00, 0x65, 0xbe, 0x5e,
	0x00, 0x65, 0x9e, 0x5e,
	0x00, 0x65, 0x9e, 0x5e,
	0x00, 0x65, 0x9e, 0x5e,
	0x01, 0x99, 0xda, 0x30,
	0x01, 0x99, 0xda, 0x30,
	0x01, 0x99, 0xda, 0x30,
	0x01, 0x99, 0xda, 0x30,
	0x01, 0x99, 0xda, 0x30,
	0x01, 0x99, 0xda, 0x30,
	0x01, 0x99, 0xda, 0x30,
	0x01, 0x99, 0xda, 0x34,
	0x01, 0x6c, 0x32, 0x31,
	0x01, 0x6c, 0x32, 0x31,
	0x01, 0x6c, 0x32, 0x31,
	0x01, 0x6c, 0x32, 0x31,
	0x01, 0x6c, 0x32, 0x31,
	0x01, 0x6c, 0x32, 0x31,
	0x01, 0x6c, 0x32, 0x31,
	0x01, 0x6c, 0x32, 0x35,
	0x08, 0x94, 0xbe, 0x7e,
	0xf7, 0x93, 0x26, 0x09,
	0x08, 0x93, 0xc2, 0x6e,
	0xff, 0x6a, 0xd4, 0x0c,
	0x04, 0xb8, 0xea, 0x6e,
	0x01, 0x42, 0x7e, 0x31,
	0xff, 0x6a, 0x76, 0x01,
	0x01, 0x90, 0x84, 0x34,
	0xff, 0x6a, 0x76, 0x05,
	0x01, 0x85, 0x0a, 0x01,
	0x7f, 0x65, 0x10, 0x09,
	0xfe, 0x85, 0x0a, 0x0d,
	0xff, 0x42, 0xe6, 0x66,
	0xff, 0x41, 0xde, 0x66,
	0xd1, 0x6a, 0xee, 0x5e,
	0xff, 0x6a, 0xca, 0x04,
	0x01, 0x41, 0x20, 0x31,
	0x01, 0xbf, 0x82, 0x30,
	0x01, 0x6a, 0x76, 0x00,
	0x00, 0xbb, 0x28, 0x46,
	0x01, 0x42, 0x20, 0x31,
	0x01, 0xbf, 0x84, 0x34,
	0x01, 0x41, 0x7e, 0x31,
	0x01, 0x90, 0x82, 0x34,
	0x01, 0x65, 0x22, 0x31,
	0xff, 0x6a, 0xd4, 0x08,
	0xff, 0x6a, 0xd4, 0x0c
};

typedef int ahc_patch_func_t (struct ahc_softc *ahc);
static ahc_patch_func_t ahc_patch23_func;

static int
ahc_patch23_func(struct ahc_softc *ahc)
{
	return ((ahc->bugs & AHC_SCBCHAN_UPLOAD_BUG) != 0);
}

static ahc_patch_func_t ahc_patch22_func;

static int
ahc_patch22_func(struct ahc_softc *ahc)
{
	return ((ahc->features & AHC_CMD_CHAN) == 0);
}

static ahc_patch_func_t ahc_patch21_func;

static int
ahc_patch21_func(struct ahc_softc *ahc)
{
	return ((ahc->features & AHC_QUEUE_REGS) == 0);
}

static ahc_patch_func_t ahc_patch20_func;

static int
ahc_patch20_func(struct ahc_softc *ahc)
{
	return ((ahc->features & AHC_WIDE) != 0);
}

static ahc_patch_func_t ahc_patch19_func;

static int
ahc_patch19_func(struct ahc_softc *ahc)
{
	return ((ahc->flags & AHC_SCB_BTT) != 0);
}

static ahc_patch_func_t ahc_patch18_func;

static int
ahc_patch18_func(struct ahc_softc *ahc)
{
	return ((ahc->bugs & AHC_PCI_2_1_RETRY_BUG) != 0);
}

static ahc_patch_func_t ahc_patch17_func;

static int
ahc_patch17_func(struct ahc_softc *ahc)
{
	return ((ahc->bugs & AHC_TMODE_WIDEODD_BUG) != 0);
}

static ahc_patch_func_t ahc_patch16_func;

static int
ahc_patch16_func(struct ahc_softc *ahc)
{
	return ((ahc->bugs & AHC_AUTOFLUSH_BUG) != 0);
}

static ahc_patch_func_t ahc_patch15_func;

static int
ahc_patch15_func(struct ahc_softc *ahc)
{
	return ((ahc->features & AHC_ULTRA2) == 0);
}

static ahc_patch_func_t ahc_patch14_func;

static int
ahc_patch14_func(struct ahc_softc *ahc)
{
	return ((ahc->bugs & AHC_PCI_MWI_BUG) != 0 && ahc->pci_cachesize != 0);
}

static ahc_patch_func_t ahc_patch13_func;

static int
ahc_patch13_func(struct ahc_softc *ahc)
{
	return ((ahc->flags & AHC_39BIT_ADDRESSING) != 0);
}

static ahc_patch_func_t ahc_patch12_func;

static int
ahc_patch12_func(struct ahc_softc *ahc)
{
	return ((ahc->features & AHC_HS_MAILBOX) != 0);
}

static ahc_patch_func_t ahc_patch11_func;

static int
ahc_patch11_func(struct ahc_softc *ahc)
{
	return ((ahc->features & AHC_ULTRA) != 0);
}

static ahc_patch_func_t ahc_patch10_func;

static int
ahc_patch10_func(struct ahc_softc *ahc)
{
	return ((ahc->features & AHC_MULTI_TID) != 0);
}

static ahc_patch_func_t ahc_patch9_func;

static int
ahc_patch9_func(struct ahc_softc *ahc)
{
	return ((ahc->features & AHC_CMD_CHAN) != 0);
}

static ahc_patch_func_t ahc_patch8_func;

static int
ahc_patch8_func(struct ahc_softc *ahc)
{
	return ((ahc->flags & AHC_INITIATORROLE) != 0);
}

static ahc_patch_func_t ahc_patch7_func;

static int
ahc_patch7_func(struct ahc_softc *ahc)
{
	return ((ahc->flags & AHC_TARGETROLE) != 0);
}

static ahc_patch_func_t ahc_patch6_func;

static int
ahc_patch6_func(struct ahc_softc *ahc)
{
	return ((ahc->features & AHC_DT) == 0);
}

static ahc_patch_func_t ahc_patch5_func;

static int
ahc_patch5_func(struct ahc_softc *ahc)
{
	return ((ahc->flags & AHC_SEQUENCER_DEBUG) != 0);
}

static ahc_patch_func_t ahc_patch4_func;

static int
ahc_patch4_func(struct ahc_softc *ahc)
{
	return ((ahc->flags & AHC_PAGESCBS) != 0);
}

static ahc_patch_func_t ahc_patch3_func;

static int
ahc_patch3_func(struct ahc_softc *ahc)
{
	return ((ahc->features & AHC_QUEUE_REGS) != 0);
}

static ahc_patch_func_t ahc_patch2_func;

static int
ahc_patch2_func(struct ahc_softc *ahc)
{
	return ((ahc->features & AHC_TWIN) != 0);
}

static ahc_patch_func_t ahc_patch1_func;

static int
ahc_patch1_func(struct ahc_softc *ahc)
{
	return ((ahc->features & AHC_ULTRA2) != 0);
}

static ahc_patch_func_t ahc_patch0_func;

static int
ahc_patch0_func(struct ahc_softc *ahc)
{
	return (0);
}

static struct patch {
	ahc_patch_func_t		*patch_func;
	uint32_t		 begin		:10,
				 skip_instr	:10,
				 skip_patch	:12;
} patches[] = {
	{ ahc_patch1_func, 4, 1, 1 },
	{ ahc_patch2_func, 6, 2, 1 },
	{ ahc_patch2_func, 9, 1, 1 },
	{ ahc_patch3_func, 11, 1, 2 },
	{ ahc_patch0_func, 12, 2, 1 },
	{ ahc_patch4_func, 15, 1, 2 },
	{ ahc_patch0_func, 16, 1, 1 },
	{ ahc_patch5_func, 22, 2, 1 },
	{ ahc_patch3_func, 27, 1, 2 },
	{ ahc_patch0_func, 28, 1, 1 },
	{ ahc_patch6_func, 34, 1, 1 },
	{ ahc_patch7_func, 37, 54, 19 },
	{ ahc_patch8_func, 37, 1, 1 },
	{ ahc_patch9_func, 42, 3, 2 },
	{ ahc_patch0_func, 45, 3, 1 },
	{ ahc_patch10_func, 49, 1, 2 },
	{ ahc_patch0_func, 50, 2, 3 },
	{ ahc_patch1_func, 50, 1, 2 },
	{ ahc_patch0_func, 51, 1, 1 },
	{ ahc_patch2_func, 53, 2, 1 },
	{ ahc_patch9_func, 55, 1, 2 },
	{ ahc_patch0_func, 56, 1, 1 },
	{ ahc_patch9_func, 60, 1, 2 },
	{ ahc_patch0_func, 61, 1, 1 },
	{ ahc_patch9_func, 71, 1, 2 },
	{ ahc_patch0_func, 72, 1, 1 },
	{ ahc_patch9_func, 75, 1, 2 },
	{ ahc_patch0_func, 76, 1, 1 },
	{ ahc_patch9_func, 79, 1, 2 },
	{ ahc_patch0_func, 80, 1, 1 },
	{ ahc_patch8_func, 91, 9, 4 },
	{ ahc_patch1_func, 93, 1, 2 },
	{ ahc_patch0_func, 94, 1, 1 },
	{ ahc_patch2_func, 96, 2, 1 },
	{ ahc_patch2_func, 105, 4, 1 },
	{ ahc_patch1_func, 109, 1, 2 },
	{ ahc_patch0_func, 110, 2, 3 },
	{ ahc_patch2_func, 110, 1, 2 },
	{ ahc_patch0_func, 111, 1, 1 },
	{ ahc_patch7_func, 112, 4, 2 },
	{ ahc_patch0_func, 116, 1, 1 },
	{ ahc_patch11_func, 117, 2, 1 },
	{ ahc_patch1_func, 119, 1, 2 },
	{ ahc_patch0_func, 120, 1, 1 },
	{ ahc_patch7_func, 121, 4, 1 },
	{ ahc_patch7_func, 131, 95, 11 },
	{ ahc_patch4_func, 151, 1, 1 },
	{ ahc_patch1_func, 168, 1, 1 },
	{ ahc_patch12_func, 173, 1, 2 },
	{ ahc_patch0_func, 174, 1, 1 },
	{ ahc_patch9_func, 185, 1, 2 },
	{ ahc_patch0_func, 186, 1, 1 },
	{ ahc_patch9_func, 195, 1, 2 },
	{ ahc_patch0_func, 196, 1, 1 },
	{ ahc_patch9_func, 212, 6, 2 },
	{ ahc_patch0_func, 218, 6, 1 },
	{ ahc_patch8_func, 226, 21, 2 },
	{ ahc_patch1_func, 241, 1, 1 },
	{ ahc_patch1_func, 249, 1, 2 },
	{ ahc_patch0_func, 250, 2, 2 },
	{ ahc_patch11_func, 251, 1, 1 },
	{ ahc_patch9_func, 259, 31, 3 },
	{ ahc_patch1_func, 275, 14, 2 },
	{ ahc_patch13_func, 280, 1, 1 },
	{ ahc_patch14_func, 290, 14, 1 },
	{ ahc_patch1_func, 306, 1, 2 },
	{ ahc_patch0_func, 307, 1, 1 },
	{ ahc_patch9_func, 310, 1, 1 },
	{ ahc_patch13_func, 315, 1, 1 },
	{ ahc_patch9_func, 316, 2, 2 },
	{ ahc_patch0_func, 318, 4, 1 },
	{ ahc_patch14_func, 322, 1, 1 },
	{ ahc_patch15_func, 325, 2, 3 },
	{ ahc_patch9_func, 325, 1, 2 },
	{ ahc_patch0_func, 326, 1, 1 },
	{ ahc_patch6_func, 331, 1, 2 },
	{ ahc_patch0_func, 332, 1, 1 },
	{ ahc_patch1_func, 336, 50, 11 },
	{ ahc_patch6_func, 345, 2, 4 },
	{ ahc_patch7_func, 345, 1, 1 },
	{ ahc_patch8_func, 346, 1, 1 },
	{ ahc_patch0_func, 347, 1, 1 },
	{ ahc_patch16_func, 348, 1, 1 },
	{ ahc_patch6_func, 367, 6, 3 },
	{ ahc_patch16_func, 367, 5, 1 },
	{ ahc_patch0_func, 373, 5, 1 },
	{ ahc_patch13_func, 381, 5, 1 },
	{ ahc_patch0_func, 386, 54, 17 },
	{ ahc_patch14_func, 386, 1, 1 },
	{ ahc_patch7_func, 388, 2, 2 },
	{ ahc_patch17_func, 389, 1, 1 },
	{ ahc_patch9_func, 392, 1, 1 },
	{ ahc_patch18_func, 399, 1, 1 },
	{ ahc_patch14_func, 404, 9, 3 },
	{ ahc_patch9_func, 405, 3, 2 },
	{ ahc_patch0_func, 408, 3, 1 },
	{ ahc_patch9_func, 416, 6, 2 },
	{ ahc_patch0_func, 422, 9, 2 },
	{ ahc_patch13_func, 422, 1, 1 },
	{ ahc_patch13_func, 431, 2, 1 },
	{ ahc_patch14_func, 433, 1, 1 },
	{ ahc_patch9_func, 435, 1, 2 },
	{ ahc_patch0_func, 436, 1, 1 },
	{ ahc_patch7_func, 439, 1, 1 },
	{ ahc_patch7_func, 440, 1, 1 },
	{ ahc_patch8_func, 441, 3, 3 },
	{ ahc_patch6_func, 442, 1, 2 },
	{ ahc_patch0_func, 443, 1, 1 },
	{ ahc_patch9_func, 444, 1, 1 },
	{ ahc_patch15_func, 445, 1, 2 },
	{ ahc_patch13_func, 445, 1, 1 },
	{ ahc_patch14_func, 447, 9, 4 },
	{ ahc_patch9_func, 447, 1, 1 },
	{ ahc_patch9_func, 454, 2, 1 },
	{ ahc_patch0_func, 456, 4, 3 },
	{ ahc_patch9_func, 456, 1, 2 },
	{ ahc_patch0_func, 457, 3, 1 },
	{ ahc_patch1_func, 461, 2, 1 },
	{ ahc_patch7_func, 463, 10, 2 },
	{ ahc_patch0_func, 473, 1, 1 },
	{ ahc_patch8_func, 474, 118, 22 },
	{ ahc_patch1_func, 476, 3, 2 },
	{ ahc_patch0_func, 479, 5, 3 },
	{ ahc_patch9_func, 479, 2, 2 },
	{ ahc_patch0_func, 481, 3, 1 },
	{ ahc_patch1_func, 486, 2, 2 },
	{ ahc_patch0_func, 488, 6, 3 },
	{ ahc_patch9_func, 488, 2, 2 },
	{ ahc_patch0_func, 490, 3, 1 },
	{ ahc_patch1_func, 496, 2, 2 },
	{ ahc_patch0_func, 498, 9, 7 },
	{ ahc_patch9_func, 498, 5, 6 },
	{ ahc_patch19_func, 498, 1, 2 },
	{ ahc_patch0_func, 499, 1, 1 },
	{ ahc_patch19_func, 501, 1, 2 },
	{ ahc_patch0_func, 502, 1, 1 },
	{ ahc_patch0_func, 503, 4, 1 },
	{ ahc_patch6_func, 508, 3, 2 },
	{ ahc_patch0_func, 511, 1, 1 },
	{ ahc_patch6_func, 521, 1, 2 },
	{ ahc_patch0_func, 522, 1, 1 },
	{ ahc_patch20_func, 559, 7, 1 },
	{ ahc_patch3_func, 594, 1, 2 },
	{ ahc_patch0_func, 595, 1, 1 },
	{ ahc_patch21_func, 598, 1, 1 },
	{ ahc_patch8_func, 600, 106, 33 },
	{ ahc_patch4_func, 602, 1, 1 },
	{ ahc_patch1_func, 608, 2, 2 },
	{ ahc_patch0_func, 610, 1, 1 },
	{ ahc_patch1_func, 613, 1, 2 },
	{ ahc_patch0_func, 614, 1, 1 },
	{ ahc_patch9_func, 615, 3, 3 },
	{ ahc_patch15_func, 616, 1, 1 },
	{ ahc_patch0_func, 618, 4, 1 },
	{ ahc_patch19_func, 627, 2, 2 },
	{ ahc_patch0_func, 629, 1, 1 },
	{ ahc_patch19_func, 633, 10, 3 },
	{ ahc_patch5_func, 635, 8, 1 },
	{ ahc_patch0_func, 643, 9, 2 },
	{ ahc_patch5_func, 644, 8, 1 },
	{ ahc_patch4_func, 654, 1, 2 },
	{ ahc_patch0_func, 655, 1, 1 },
	{ ahc_patch19_func, 656, 1, 2 },
	{ ahc_patch0_func, 657, 3, 2 },
	{ ahc_patch4_func, 659, 1, 1 },
	{ ahc_patch5_func, 660, 1, 1 },
	{ ahc_patch5_func, 663, 1, 1 },
	{ ahc_patch5_func, 665, 1, 1 },
	{ ahc_patch4_func, 667, 2, 2 },
	{ ahc_patch0_func, 669, 2, 1 },
	{ ahc_patch5_func, 671, 1, 1 },
	{ ahc_patch5_func, 674, 1, 1 },
	{ ahc_patch5_func, 677, 1, 1 },
	{ ahc_patch19_func, 681, 1, 1 },
	{ ahc_patch19_func, 684, 1, 1 },
	{ ahc_patch4_func, 690, 1, 1 },
	{ ahc_patch6_func, 693, 1, 2 },
	{ ahc_patch0_func, 694, 1, 1 },
	{ ahc_patch7_func, 706, 16, 1 },
	{ ahc_patch4_func, 722, 20, 1 },
	{ ahc_patch9_func, 743, 4, 2 },
	{ ahc_patch0_func, 747, 4, 1 },
	{ ahc_patch9_func, 751, 4, 2 },
	{ ahc_patch0_func, 755, 3, 1 },
	{ ahc_patch6_func, 761, 1, 1 },
	{ ahc_patch22_func, 763, 14, 1 },
	{ ahc_patch7_func, 777, 3, 1 },
	{ ahc_patch9_func, 789, 24, 8 },
	{ ahc_patch19_func, 793, 1, 2 },
	{ ahc_patch0_func, 794, 1, 1 },
	{ ahc_patch15_func, 799, 4, 2 },
	{ ahc_patch0_func, 803, 7, 3 },
	{ ahc_patch23_func, 803, 5, 2 },
	{ ahc_patch0_func, 808, 2, 1 },
	{ ahc_patch0_func, 813, 42, 3 },
	{ ahc_patch18_func, 825, 18, 2 },
	{ ahc_patch0_func, 843, 1, 1 },
	{ ahc_patch4_func, 867, 1, 1 },
	{ ahc_patch4_func, 868, 3, 2 },
	{ ahc_patch0_func, 871, 1, 1 },
	{ ahc_patch13_func, 872, 3, 1 },
	{ ahc_patch4_func, 875, 12, 1 }
};

static struct cs {
	uint16_t	begin;
	uint16_t	end;
} critical_sections[] = {
	{ 11, 18 },
	{ 21, 30 },
	{ 722, 738 },
	{ 868, 871 },
	{ 875, 881 },
	{ 883, 885 },
	{ 885, 887 }
};

static const int num_critical_sections = sizeof(critical_sections)
				       / sizeof(*critical_sections);
