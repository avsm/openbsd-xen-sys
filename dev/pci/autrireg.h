/*	$OpenBSD$	*/

/*
 * Copyright (c) 2001 SOMEYA Yoshihiko and KUROSAWA Takahiro.
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
 * Trident 4DWAVE registers
 */

#ifndef _DEV_PCI_AUTRIREG_H_
#define	_DEV_PCI_AUTRIREG_H_


#define AUTRI_DEVICE_ID_4DWAVE_DX \
	((PCI_PRODUCT_TRIDENT_4DWAVE_DX << 16) | PCI_VENDOR_TRIDENT)
#define AUTRI_DEVICE_ID_4DWAVE_NX \
	((PCI_PRODUCT_TRIDENT_4DWAVE_NX << 16) | PCI_VENDOR_TRIDENT)
#define AUTRI_DEVICE_ID_SIS_7018 \
	((PCI_PRODUCT_SIS_7018 << 16) | PCI_VENDOR_SIS)
#define AUTRI_DEVICE_ID_ALI_M5451 \
	((PCI_PRODUCT_ALI_M5451 << 16) | PCI_VENDOR_ALI)

/*
 * PCI Config Registers
 */
#define AUTRI_PCI_MEMORY_BASE	0x14
#define AUTRI_PCI_DDMA_CFG	0x40
#define AUTRI_PCI_LEGACY_IOBASE	0x44

/*
 * AC'97 Registers
 */
#define AUTRI_DX_ACR0			0x40
# define AUTRI_DX_ACR0_CMD_WRITE	0x00008000
# define AUTRI_DX_ACR0_BUSY_WRITE	0x00008000
#define AUTRI_DX_ACR1			0x44
# define AUTRI_DX_ACR1_CMD_READ		0x00008000
# define AUTRI_DX_ACR1_BUSY_READ	0x00008000
#define AUTRI_DX_ACR2			0x48
# define AUTRI_DX_ACR2_CODEC_READY	0x00000010

#define AUTRI_NX_ACR0			0x40
# define AUTRI_NX_ACR0_PSB_CAPTURE	0x00000200
# define AUTRI_NX_ACR0_CODEC_READY	0x00000008
#define AUTRI_NX_ACR1			0x44
# define AUTRI_NX_ACR1_CMD_WRITE	0x00000800
# define AUTRI_NX_ACR1_BUSY_WRITE	0x00000800
#define AUTRI_NX_ACR2			0x48
# define AUTRI_NX_ACR2_CMD_READ		0x00000800
# define AUTRI_NX_ACR2_BUSY_READ	0x00000800
# define AUTRI_NX_ACR2_RECV_WAIT	0x00000400
#define AUTRI_NX_ACR3			0x4c

#define AUTRI_SIS_ACWR			0x40
# define AUTRI_SIS_ACWR_CMD_WRITE	0x00008000
# define AUTRI_SIS_ACWR_BUSY_WRITE	0x00008000
# define AUTRI_SIS_ACWR_AUDIO_BUSY	0x00004000
#define AUTRI_SIS_ACRD			0x44
# define AUTRI_SIS_ACRD_CMD_READ	0x00008000
# define AUTRI_SIS_ACRD_BUSY_READ	0x00008000
# define AUTRI_SIS_ACRD_AUDIO_BUSY	0x00004000
#define AUTRI_SIS_SCTRL			0x48
# define AUTRI_SIS_SCTRL_CODEC_READY	0x01000000
#define AUTRI_SIS_ACGPIO		0x4c

#define AUTRI_ALI_ACWR			0x40
# define AUTRI_ALI_ACWR_CMD_WRITE	0x00008000
# define AUTRI_ALI_ACWR_BUSY_WRITE	0x00008000
#define AUTRI_ALI_ACRD			0x44
# define AUTRI_ALI_ACRD_CMD_READ	0x00008000
# define AUTRI_ALI_ACRD_BUSY_READ	0x00008000
#define AUTRI_ALI_SCTRL			0x48
# define AUTRI_ALI_SCTRL_CODEC_READY	0x01000000
#define AUTRI_ALI_ACGPIO		0x4c
/*
# define AUTRI_ALI_AC97_BUSY_READ  0x00008000
# define AUTRI_ALI_AC97_BUSY_WRITE 0x00008000
# define AUTRI_ALI_AC97_CMD_WRITE  0x00008000
*/

/*
 * MPU-401 UART
 */
#define AUTRI_MPUR0			0x20
#define AUTRI_MPUR1			0x21
# define AUTRI_MIDIOUT_READY		0x40
#define AUTRI_MPUR2			0x22
# define AUTRI_MIDIOUT_CONNECT		0x10
# define AUTRI_MIDIIN_ENABLE_INTR	0x08

#define MIDI_BUSY_WAIT			100
#define MIDI_BUSY_DELAY			100

/*
 * Channel Registers
 */
#define AUTRI_START_A			0x80
#define AUTRI_STOP_A			0x84
#define AUTRI_DLY_A			0x88
#define AUTRI_SIGN_CSO_A		0x8c
#define AUTRI_CSPF_A			0x90
#define AUTRI_CEBC_A			0x94
#define AUTRI_AIN_A			0x98
#define AUTRI_EINT_A			0x9c
#define AUTRI_LFO_GC_CIR		0xa0
# define ENDLP_IE			0x00001000
# define MIDLP_IE			0x00002000
# define BANK_B_EN			0x00010000
#define AUTRI_AINTEN_A			0xa4
#define AUTRI_MUSICVOL_WAVEVOL		0xa8
#define AUTRI_MISCINT			0xb0
# define ST_TARGET_REACHED		0x00008000
# define MIXER_OVERFLOW			0x00000800
# define MIXER_UNDERFLOW		0x00000800
# define ADDRESS_IRQ			0x00000020
# define MPU401_IRQ			0x00000008
#define AUTRI_START_B			0xb4
#define AUTRI_STOP_B			0xb8
#define AUTRI_CSPF_B			0xbc
#define AUTRI_AIN_B			0xd8
#define AUTRI_AINTEN_B			0xdc

/*
 * Indexed Channel Registers
 */
#define AUTRI_ARAM_CR			0xe0
# define AUTRI_CTRL_WAVEVOL		0x80000000
# define AUTRI_CTRL_MUTE		0x3fff0000
# define AUTRI_CTRL_16BIT		0x00008000
# define AUTRI_CTRL_STEREO		0x00004000
# define AUTRI_CTRL_SIGNED		0x00002000
# define AUTRI_CTRL_LOOPMODE		0x00001000
#define AUTRI_EBUF1			0xf4
#define AUTRI_EBUF2			0xf8
# define AUTRI_EMOD_STILL		0x30000000

/*
 * Others
 */
#define AUTRI_NX_RCI3			0x73
# define AUTRI_NX_RCI3_ENABLE		0x80

#define AUTRI_ALI_GCONTROL		0xd4
# define AUTRI_ALI_GCONTROL_PCM_IN	0x80000000


#endif /* _DEV_PCI_AUTRIREG_H_ */
