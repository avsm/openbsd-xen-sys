#! /usr/bin/awk -f
#	$OpenBSD: makemap.awk,v 1.5 2005/05/09 05:07:25 miod Exp $
#
# Copyright (c) 2003, 2005, Miodrag Vallat.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
# IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
# OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
# IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
# NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
# THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#

#
# This script attempts to convert, with minimal hacks and losses, the
# regular PS/2 keyboard (pckbd) layout tables into GSC keyboard (gsckbd)
# layout tables, as almost all scancodes are completely different in the
# GSC world.
#

BEGIN {
	rcsid = "$OpenBSD: makemap.awk,v 1.5 2005/05/09 05:07:25 miod Exp $"
	ifdepth = 0
	ignore = 0
	mapnum = 0
	declk = 0

	# PS/2 id -> GSCKBD conversion table, or "sanity lossage 101"
	for (i = 0; i < 256; i++)
		conv[i] = -1

	conv[1] = 118
	conv[2] = 22
	conv[3] = 30
	conv[4] = 38
	conv[5] = 37
	conv[6] = 46
	conv[7] = 54
	conv[8] = 61
	conv[9] = 62
	conv[10] = 70
	conv[11] = 69
	conv[12] = 78
	conv[13] = 85
	conv[14] = 102
	conv[15] = 13
	conv[16] = 21
	conv[17] = 29
	conv[18] = 36
	conv[19] = 45
	conv[20] = 44
	conv[21] = 53
	conv[22] = 60
	conv[23] = 67
	conv[24] = 68
	conv[25] = 77
	conv[26] = 84
	conv[27] = 91
	conv[28] = 90
	conv[29] = 20
	conv[30] = 28
	conv[31] = 27
	conv[32] = 35
	conv[33] = 43
	conv[34] = 52
	conv[35] = 51
	conv[36] = 59
	conv[37] = 66
	conv[38] = 75
	conv[39] = 76
	conv[40] = 82
	conv[41] = 14
	conv[42] = 18
	conv[43] = 93
	conv[44] = 26
	conv[45] = 34
	conv[46] = 33
	conv[47] = 42
	conv[48] = 50
	conv[49] = 49
	conv[50] = 58
	conv[51] = 65
	conv[52] = 73
	conv[53] = 74
	conv[54] = 89
	conv[55] = 124
	conv[56] = 17
	conv[57] = 41
	conv[58] = 88
	conv[59] = 5
	conv[60] = 6
	conv[61] = 4
	conv[62] = 12
	conv[63] = 3
	conv[64] = 11
	conv[65] = 131
	conv[66] = 10
	conv[67] = 1
	conv[68] = 9
	conv[69] = 119
	conv[70] = 126
	conv[71] = 108
	conv[72] = 117
	conv[73] = 125
	conv[74] = 123
	conv[75] = 107
	conv[76] = 115
	conv[77] = 116
	conv[78] = 121
	conv[79] = 105
	conv[80] = 114
	conv[81] = 122
	conv[82] = 112
	conv[83] = 113
	conv[86] = 97
	conv[87] = 120
	conv[88] = 7
	# 112 used by jp
	# 115 used by jp and br
	# 121 used by jp
	# 123 used by jp
	# 125 used by jp
	conv[127] = 127
	conv[156] = 218
	conv[157] = 148
	# Print Screen produces E0 12 E0 7C when pressed, then E0 7C E0 12
	# when released.  Ignore the E0 12 code and match only on E0 7C
	conv[170] = 252
	conv[181] = 202
	conv[184] = 145
	conv[198] = 254
	conv[199] = 236
	conv[200] = 245
	conv[201] = 253
	conv[203] = 235
	conv[205] = 244
	conv[207] = 233
	conv[208] = 242
	conv[209] = 250
	conv[210] = 240
	conv[211] = 113
}
NR == 1 {
	VERSION = $0
	gsub("\\$", "", VERSION)
	gsub("\\$", "", rcsid)

	printf("/*\t\$OpenBSD\$\t*/\n\n")
	printf("/*\n")
	printf(" * THIS FILE IS AUTOMAGICALLY GENERATED.  DO NOT EDIT.\n")
	printf(" *\n")
	printf(" * generated by:\n")
	printf(" *\t%s\n", rcsid)
	printf(" * generated from:\n")
	printf(" */\n")
	print VERSION

	next
}

#
# A very limited #if ... #endif parser. We only want to correctly detect
# ``#if 0'' constructs, so as not to process their contents. This is necessary
# since our output is out-of-order from our input.
#
# Note that this does NOT handle ``#ifdef notyet'' correctly - please only use
# ``#if 0'' constructs in the input.
#

/^#if/ {
	ignores[ifdepth] = ignore
	if ($2 == "0")
		ignore = 1
	else
		ignore = 0
	ifdepth++
	if (ignore)
		next
}
/^#endif/ {
	oldignore = ignore
	ifdepth--
	ignore = ignores[ifdepth]
	ignores[ifdepth] = 0
	if (oldignore)
		next
}

$1 == "#include" {
	if (ignore)
		next
	if ($2 == "<dev/pckbc/wskbdmap_mfii.h>")
		print "#include <hppa/gsc/gsckbdmap.h>"
	else
		printf("#include %s\n", $2)

	next
}
$1 == "#define" || $1 == "#undef" {
	if (ignore)
		next
	print $0
	next
}

# Don't bother converting the DEC LK layout.
/declk\[/ {
	declk = 1
	next
}
/declk/ {
	next
}

/pckbd/ {
	if (ignore)
		next
	gsub("pckbd", "gsckbd", $0)
	print $0
	next
}

/KC/ {
	if (ignore)
		next

	if (declk)
		next

	sidx = substr($1, 4, length($1) - 5)
	orig = int(sidx)
	id = conv[orig]

	# 183 is another Print Screen...
	if (orig == 183)
		next

	if (id == -1) {
		printf("/* initially KC(%d),", orig)
		for (f = 2; f <= NF; f++) {
			if ($f != "/*" && $f != "*/")
				printf("\t%s", $f)
		}
		printf("\t*/\n")
	} else {
		lines[id] = sprintf("    KC(%d),\t", id)
		#
		# This makes sure that the non-comment part of the output
		# ends up with a trailing comma. This is necessary since
		# the last line of an input block might not have a trailing
		# comma, but might not be the last line of an output block
		# due to sorting.
		#
		comma = 0
		for (f = 2; f <= NF; f++) {
			l = length($f)
			if ($f == "/*")
				comma++
			if (comma == 0 && substr($f, l) != ",") {
				lines[id] = sprintf("%s%s,", lines[id], $f)
				l++
			} else {
				lines[id] = sprintf("%s%s", lines[id], $f)
			}
			if (comma == 0 && f != NF) {
				if (l < 2 * 8)
					lines[id] = lines[id] "\t"
				if (l < 8)
					lines[id] = lines[id] "\t"
			}
			if ($f == "*/")
				comma--
		}
	}

	next
}
/};/ {
	if (ignore)
		next

	if (declk) {
		declk = 0
		next
	}

	for (i = 0; i < 256; i++)
		if (lines[i]) {
			print lines[i]
			lines[i] = ""
		}

	if (mapnum == 0) {
		# Add 241 to the US map...
		print "    KC(241),\tKS_Delete"
		print $0
		print "\nstatic const keysym_t gsckbd_keydesc_precisionbook[] = {"
		print "/*  pos      command\t\tnormal		shifted */"
		print "    KC(7),   KS_Cmd_Screen0,\tKS_f1,"
		print "    KC(15),  KS_Cmd_Screen1,\tKS_f2,"
		print "    KC(23),  KS_Cmd_Screen2,\tKS_f3,"
		print "    KC(31),  KS_Cmd_Screen3,\tKS_f4,"
		print "    KC(39),  KS_Cmd_Screen4,\tKS_f5,"
		print "    KC(47),  KS_Cmd_Screen5,\tKS_f6,"
		print "    KC(55),  KS_Cmd_Screen6,\tKS_f7,"
		print "    KC(63),  KS_Cmd_Screen7,\tKS_f8,"
		print "    KC(71),  KS_Cmd_Screen8,\tKS_f9,"
		print "    KC(79),  KS_Cmd_Screen9,\tKS_f10,"
		print "    KC(86),  KS_Cmd_Screen10,\tKS_f11,"
		print "    KC(94),  KS_Cmd_Screen11,\tKS_f12,"
		print "    KC(8),   KS_Cmd_Debugger,\tKS_Escape,"
		print "    KC(87),\t\t\tKS_Print_Screen,"
		print "    KC(92),\t\t\tKS_backslash,\tKS_bar,"
		print "    KC(96),\t\t\tKS_KP_Down,\tKS_KP_2,"
		print "    KC(95),\t\t\tKS_Hold_Screen,"
		print "    KC(97),\t\t\tKS_KP_Left,\tKS_KP_4,"
		print "    KC(98),\t\t\tKS_Pause, /* Break */"
		print "    KC(99),\t\t\tKS_KP_Up,\tKS_KP_8,"
		print "    KC(100),\t\t\tKS_KP_Delete,\tKS_KP_Decimal,"
		print "    KC(101),\t\t\tKS_KP_End,\tKS_KP_1,"
		print "    KC(103),\t\t\tKS_KP_Insert,\tKS_KP_0,"
		print "    KC(106),\t\t\tKS_KP_Right,\tKS_KP_6,"
		print "    KC(109),\t\t\tKS_KP_Next,\tKS_KP_3,"
		print "    KC(110),\t\t\tKS_KP_Home,\tKS_KP_7,"
		print "    KC(111),\t\t\tKS_KP_Prior,\tKS_KP_9,"
		print "    KC(20),\t\t\tKS_Caps_Lock,"
		print "    KC(17),  KS_Cmd1,\t\tKS_Control_L,"
		print "    KC(88),  KS_Cmd1,\t\tKS_Control_R,"
		print "    KC(25),  KS_Cmd2,\t\tKS_Alt_L,"
		print "    KC(57),  KS_Cmd2,\t\tKS_Alt_R,\tKS_Multi_key,"
		print "    KC(139),\t\t\tKS_Meta_L,"
		print "    KC(140),\t\t\tKS_Meta_R,"
	}
	mapnum++
}
/{0, 0, 0, 0}/ {
	printf("\tKBD_MAP(KB_US | KB_MACHDEP,\tKB_US,\tgsckbd_keydesc_precisionbook),\n");
}
{
	if (ignore)
		next
	if (declk)
		next
	print $0
}
