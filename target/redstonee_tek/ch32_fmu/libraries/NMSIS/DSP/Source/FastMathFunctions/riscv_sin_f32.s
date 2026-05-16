	.file	"riscv_sin_f32.c"
	.option nopic
	.attribute arch, "rv32i2p0_m2p0_a2p0_f2p0_c2p0_zba1p0_zbb1p0_zbc1p0_zbs1p0_xw"
	.attribute unaligned_access, 0
	.attribute stack_align, 16
	.text
.Ltext0:
	.cfi_sections	.debug_frame
	.file 1 "libraries\\NMSIS\\DSP\\Source\\FastMathFunctions\\riscv_sin_f32.c"
	.section	.text.riscv_sin_f32,"ax",@progbits
	.align	1
	.globl	riscv_sin_f32
	.type	riscv_sin_f32, @function
riscv_sin_f32:
.LFB135:
	.loc 1 74 1
	.cfi_startproc
.LVL0:
	.loc 1 75 3
	.loc 1 76 3
	.loc 1 77 3
	.loc 1 78 3
	.loc 1 79 3
	.loc 1 83 3
	.loc 1 83 6 is_stmt 0
	flw	fa5,.LC0,a5
	fmul.s	fa0,fa0,fa5
.LVL1:
	.loc 1 86 3 is_stmt 1
	.loc 1 86 5 is_stmt 0
	fcvt.w.s a5,fa0,rtz
.LVL2:
	.loc 1 89 3 is_stmt 1
	.loc 1 89 6 is_stmt 0
	fmv.w.x	fa5,zero
	flt.s	a4,fa0,fa5
	bne	a4,zero,.L6
.L2:
	.loc 1 95 3 is_stmt 1
	.loc 1 95 13 is_stmt 0
	fcvt.s.w	fa5,a5
	.loc 1 95 6
	fsub.s	fa0,fa0,fa5
.LVL3:
	.loc 1 98 3 is_stmt 1
	.loc 1 98 10 is_stmt 0
	flw	fa5,.LC1,a5
.LVL4:
	fmul.s	fa0,fa0,fa5
.LVL5:
	.loc 1 99 3 is_stmt 1
	.loc 1 99 9 is_stmt 0
	fcvt.wu.s a5,fa0,rtz
	zext.h	a5,a5
.LVL6:
	.loc 1 102 3 is_stmt 1
	.loc 1 102 6 is_stmt 0
	li	a4,511
	bleu	a5,a4,.L4
	.loc 1 103 5 is_stmt 1
.LVL7:
	.loc 1 104 5
	.loc 1 104 12 is_stmt 0
	fsub.s	fa0,fa0,fa5
.LVL8:
	.loc 1 103 11
	li	a5,0
.LVL9:
.L4:
	.loc 1 108 3 is_stmt 1
	.loc 1 108 20 is_stmt 0
	fcvt.s.wu	fa5,a5
	.loc 1 108 9
	fsub.s	fa5,fa0,fa5
.LVL10:
	.loc 1 111 3 is_stmt 1
	.loc 1 111 5 is_stmt 0
	lla	a4,sinTable_f32
	sh2add	a3,a5,a4
	flw	fa2,0(a3)
.LVL11:
	.loc 1 112 3 is_stmt 1
	.loc 1 112 25 is_stmt 0
	addi	a5,a5,1
.LVL12:
	.loc 1 112 5
	sh2add	a5,a5,a4
	flw	fa3,0(a5)
.LVL13:
	.loc 1 115 3 is_stmt 1
	.loc 1 115 18 is_stmt 0
	flw	fa4,.LC2,a5
	fsub.s	fa4,fa4,fa5
	.loc 1 115 27
	fmul.s	fa4,fa4,fa2
	.loc 1 115 39
	fmul.s	fa0,fa5,fa3
.LVL14:
	.loc 1 118 3 is_stmt 1
	.loc 1 119 1 is_stmt 0
	fadd.s	fa0,fa4,fa0
.LVL15:
	ret
.LVL16:
.L6:
	.loc 1 91 5 is_stmt 1
	.loc 1 91 6 is_stmt 0
	addi	a5,a5,-1
.LVL17:
	j	.L2
	.cfi_endproc
.LFE135:
	.size	riscv_sin_f32, .-riscv_sin_f32
	.section	.srodata.cst4,"aM",@progbits,4
	.align	2
.LC0:
	.word	1042479491
	.align	2
.LC1:
	.word	1140850688
	.align	2
.LC2:
	.word	1065353216
	.text
.Letext0:
	.file 2 "d:\\mounriver\\mounriver_studio2\\resources\\app\\resources\\win32\\components\\wch\\toolchain\\risc-v embedded gcc12\\riscv-wch-elf\\include\\machine\\_default_types.h"
	.file 3 "d:\\mounriver\\mounriver_studio2\\resources\\app\\resources\\win32\\components\\wch\\toolchain\\risc-v embedded gcc12\\riscv-wch-elf\\include\\sys\\_stdint.h"
	.file 4 "libraries\\NMSIS\\DSP\\Include/riscv_math_types.h"
	.file 5 "libraries\\NMSIS\\DSP\\Include/riscv_common_tables.h"
	.section	.debug_info,"",@progbits
.Ldebug_info0:
	.4byte	0x191
	.2byte	0x2
	.4byte	.Ldebug_abbrev0
	.byte	0x4
	.byte	0x1
	.4byte	.LASF22
	.byte	0xc
	.4byte	.LASF23
	.4byte	.LASF24
	.4byte	.Ldebug_ranges0+0
	.4byte	0
	.4byte	0
	.4byte	.Ldebug_line0
	.byte	0x2
	.byte	0x4
	.byte	0x4
	.4byte	.LASF0
	.byte	0x2
	.byte	0x1
	.byte	0x6
	.4byte	.LASF1
	.byte	0x2
	.byte	0x1
	.byte	0x8
	.4byte	.LASF2
	.byte	0x2
	.byte	0x2
	.byte	0x5
	.4byte	.LASF3
	.byte	0x3
	.4byte	.LASF5
	.byte	0x2
	.byte	0x39
	.byte	0x1c
	.4byte	0x51
	.byte	0x2
	.byte	0x2
	.byte	0x7
	.4byte	.LASF4
	.byte	0x3
	.4byte	.LASF6
	.byte	0x2
	.byte	0x4d
	.byte	0x12
	.4byte	0x64
	.byte	0x2
	.byte	0x4
	.byte	0x5
	.4byte	.LASF7
	.byte	0x2
	.byte	0x4
	.byte	0x7
	.4byte	.LASF8
	.byte	0x2
	.byte	0x8
	.byte	0x5
	.4byte	.LASF9
	.byte	0x2
	.byte	0x8
	.byte	0x7
	.4byte	.LASF10
	.byte	0x4
	.byte	0x4
	.byte	0x5
	.string	"int"
	.byte	0x2
	.byte	0x4
	.byte	0x7
	.4byte	.LASF11
	.byte	0x3
	.4byte	.LASF12
	.byte	0x3
	.byte	0x24
	.byte	0x14
	.4byte	0x45
	.byte	0x3
	.4byte	.LASF13
	.byte	0x3
	.byte	0x2c
	.byte	0x13
	.4byte	0x58
	.byte	0x2
	.byte	0x10
	.byte	0x4
	.4byte	.LASF14
	.byte	0x2
	.byte	0x1
	.byte	0x6
	.4byte	.LASF15
	.byte	0x2
	.byte	0x8
	.byte	0x4
	.4byte	.LASF16
	.byte	0x5
	.4byte	.LASF17
	.byte	0x4
	.2byte	0x101
	.byte	0x11
	.4byte	0x29
	.byte	0x6
	.4byte	0xbb
	.byte	0x7
	.4byte	0xc8
	.4byte	0xde
	.byte	0x8
	.4byte	0x87
	.2byte	0x200
	.byte	0
	.byte	0x6
	.4byte	0xcd
	.byte	0x9
	.4byte	.LASF25
	.byte	0x5
	.2byte	0x126
	.byte	0x1c
	.4byte	0xde
	.byte	0x1
	.byte	0x1
	.byte	0xa
	.byte	0x1
	.4byte	.LASF26
	.byte	0x1
	.byte	0x48
	.byte	0xc
	.byte	0x1
	.4byte	0xbb
	.4byte	.LFB135
	.4byte	.LFE135
	.byte	0x2
	.byte	0x72
	.byte	0
	.byte	0x1
	.byte	0xb
	.string	"x"
	.byte	0x1
	.byte	0x49
	.byte	0xd
	.4byte	0xbb
	.4byte	.LLST0
	.byte	0xc
	.4byte	.LASF18
	.byte	0x1
	.byte	0x4b
	.byte	0xd
	.4byte	0xbb
	.4byte	.LLST1
	.byte	0xc
	.4byte	.LASF19
	.byte	0x1
	.byte	0x4b
	.byte	0x15
	.4byte	0xbb
	.4byte	.LLST2
	.byte	0xd
	.string	"in"
	.byte	0x1
	.byte	0x4b
	.byte	0x1c
	.4byte	0xbb
	.4byte	.LLST3
	.byte	0xc
	.4byte	.LASF20
	.byte	0x1
	.byte	0x4c
	.byte	0xc
	.4byte	0x8e
	.4byte	.LLST4
	.byte	0xd
	.string	"a"
	.byte	0x1
	.byte	0x4d
	.byte	0xd
	.4byte	0xbb
	.4byte	.LLST5
	.byte	0xd
	.string	"b"
	.byte	0x1
	.byte	0x4d
	.byte	0x10
	.4byte	0xbb
	.4byte	.LLST6
	.byte	0xd
	.string	"n"
	.byte	0x1
	.byte	0x4e
	.byte	0xb
	.4byte	0x9a
	.4byte	.LLST7
	.byte	0xc
	.4byte	.LASF21
	.byte	0x1
	.byte	0x4f
	.byte	0xd
	.4byte	0xbb
	.4byte	.LLST8
	.byte	0
	.byte	0
	.section	.debug_abbrev,"",@progbits
.Ldebug_abbrev0:
	.byte	0x1
	.byte	0x11
	.byte	0x1
	.byte	0x25
	.byte	0xe
	.byte	0x13
	.byte	0xb
	.byte	0x3
	.byte	0xe
	.byte	0x1b
	.byte	0xe
	.byte	0x55
	.byte	0x6
	.byte	0x11
	.byte	0x1
	.byte	0x52
	.byte	0x1
	.byte	0x10
	.byte	0x6
	.byte	0
	.byte	0
	.byte	0x2
	.byte	0x24
	.byte	0
	.byte	0xb
	.byte	0xb
	.byte	0x3e
	.byte	0xb
	.byte	0x3
	.byte	0xe
	.byte	0
	.byte	0
	.byte	0x3
	.byte	0x16
	.byte	0
	.byte	0x3
	.byte	0xe
	.byte	0x3a
	.byte	0xb
	.byte	0x3b
	.byte	0xb
	.byte	0x39
	.byte	0xb
	.byte	0x49
	.byte	0x13
	.byte	0
	.byte	0
	.byte	0x4
	.byte	0x24
	.byte	0
	.byte	0xb
	.byte	0xb
	.byte	0x3e
	.byte	0xb
	.byte	0x3
	.byte	0x8
	.byte	0
	.byte	0
	.byte	0x5
	.byte	0x16
	.byte	0
	.byte	0x3
	.byte	0xe
	.byte	0x3a
	.byte	0xb
	.byte	0x3b
	.byte	0x5
	.byte	0x39
	.byte	0xb
	.byte	0x49
	.byte	0x13
	.byte	0
	.byte	0
	.byte	0x6
	.byte	0x26
	.byte	0
	.byte	0x49
	.byte	0x13
	.byte	0
	.byte	0
	.byte	0x7
	.byte	0x1
	.byte	0x1
	.byte	0x49
	.byte	0x13
	.byte	0x1
	.byte	0x13
	.byte	0
	.byte	0
	.byte	0x8
	.byte	0x21
	.byte	0
	.byte	0x49
	.byte	0x13
	.byte	0x2f
	.byte	0x5
	.byte	0
	.byte	0
	.byte	0x9
	.byte	0x34
	.byte	0
	.byte	0x3
	.byte	0xe
	.byte	0x3a
	.byte	0xb
	.byte	0x3b
	.byte	0x5
	.byte	0x39
	.byte	0xb
	.byte	0x49
	.byte	0x13
	.byte	0x3f
	.byte	0xc
	.byte	0x3c
	.byte	0xc
	.byte	0
	.byte	0
	.byte	0xa
	.byte	0x2e
	.byte	0x1
	.byte	0x3f
	.byte	0xc
	.byte	0x3
	.byte	0xe
	.byte	0x3a
	.byte	0xb
	.byte	0x3b
	.byte	0xb
	.byte	0x39
	.byte	0xb
	.byte	0x27
	.byte	0xc
	.byte	0x49
	.byte	0x13
	.byte	0x11
	.byte	0x1
	.byte	0x12
	.byte	0x1
	.byte	0x40
	.byte	0xa
	.byte	0x97,0x42
	.byte	0xc
	.byte	0
	.byte	0
	.byte	0xb
	.byte	0x5
	.byte	0
	.byte	0x3
	.byte	0x8
	.byte	0x3a
	.byte	0xb
	.byte	0x3b
	.byte	0xb
	.byte	0x39
	.byte	0xb
	.byte	0x49
	.byte	0x13
	.byte	0x2
	.byte	0x6
	.byte	0
	.byte	0
	.byte	0xc
	.byte	0x34
	.byte	0
	.byte	0x3
	.byte	0xe
	.byte	0x3a
	.byte	0xb
	.byte	0x3b
	.byte	0xb
	.byte	0x39
	.byte	0xb
	.byte	0x49
	.byte	0x13
	.byte	0x2
	.byte	0x6
	.byte	0
	.byte	0
	.byte	0xd
	.byte	0x34
	.byte	0
	.byte	0x3
	.byte	0x8
	.byte	0x3a
	.byte	0xb
	.byte	0x3b
	.byte	0xb
	.byte	0x39
	.byte	0xb
	.byte	0x49
	.byte	0x13
	.byte	0x2
	.byte	0x6
	.byte	0
	.byte	0
	.byte	0
	.section	.debug_loc,"",@progbits
.Ldebug_loc0:
.LLST0:
	.4byte	.LVL0
	.4byte	.LVL1
	.2byte	0x2
	.byte	0x90
	.byte	0x2a
	.4byte	.LVL1
	.4byte	.LFE135
	.2byte	0x6
	.byte	0xf3
	.byte	0x3
	.byte	0xf5
	.byte	0x2a
	.byte	0x29
	.byte	0x9f
	.4byte	0
	.4byte	0
.LLST1:
	.4byte	.LVL14
	.4byte	.LVL15
	.2byte	0x8
	.byte	0xf5
	.byte	0x2e
	.byte	0x29
	.byte	0xf5
	.byte	0x2a
	.byte	0x29
	.byte	0x22
	.byte	0x9f
	.4byte	.LVL15
	.4byte	.LVL16
	.2byte	0x2
	.byte	0x90
	.byte	0x2a
	.4byte	0
	.4byte	0
.LLST2:
	.4byte	.LVL10
	.4byte	.LVL16
	.2byte	0x2
	.byte	0x90
	.byte	0x2f
	.4byte	0
	.4byte	0
.LLST3:
	.4byte	.LVL1
	.4byte	.LVL5
	.2byte	0x2
	.byte	0x90
	.byte	0x2a
	.4byte	.LVL16
	.4byte	.LFE135
	.2byte	0x2
	.byte	0x90
	.byte	0x2a
	.4byte	0
	.4byte	0
.LLST4:
	.4byte	.LVL6
	.4byte	.LVL7
	.2byte	0x1
	.byte	0x5f
	.4byte	.LVL7
	.4byte	.LVL9
	.2byte	0x2
	.byte	0x30
	.byte	0x9f
	.4byte	.LVL9
	.4byte	.LVL12
	.2byte	0x1
	.byte	0x5f
	.4byte	0
	.4byte	0
.LLST5:
	.4byte	.LVL11
	.4byte	.LVL16
	.2byte	0x2
	.byte	0x90
	.byte	0x2c
	.4byte	0
	.4byte	0
.LLST6:
	.4byte	.LVL13
	.4byte	.LVL16
	.2byte	0x2
	.byte	0x90
	.byte	0x2d
	.4byte	0
	.4byte	0
.LLST7:
	.4byte	.LVL2
	.4byte	.LVL4
	.2byte	0x1
	.byte	0x5f
	.4byte	.LVL16
	.4byte	.LFE135
	.2byte	0x1
	.byte	0x5f
	.4byte	0
	.4byte	0
.LLST8:
	.4byte	.LVL5
	.4byte	.LVL14
	.2byte	0x2
	.byte	0x90
	.byte	0x2a
	.4byte	0
	.4byte	0
	.section	.debug_aranges,"",@progbits
	.4byte	0x1c
	.2byte	0x2
	.4byte	.Ldebug_info0
	.byte	0x4
	.byte	0
	.2byte	0
	.2byte	0
	.4byte	.LFB135
	.4byte	.LFE135-.LFB135
	.4byte	0
	.4byte	0
	.section	.debug_ranges,"",@progbits
.Ldebug_ranges0:
	.4byte	.LFB135
	.4byte	.LFE135
	.4byte	0
	.4byte	0
	.section	.debug_line,"",@progbits
.Ldebug_line0:
	.section	.debug_str,"MS",@progbits,1
.LASF6:
	.string	"__int32_t"
.LASF24:
	.string	"C:\\\\Users\\\\RedStone\\\\src\\\\fmu\\\\FMT-Firmware\\\\target\\\\redstonee_tek\\\\ch32_fmu"
.LASF22:
	.string	"GNU C11 12.2.0 -mabi=ilp32f -msmall-data-limit= 8 -msave-restore -mcmodel=medany -misa-spec=2.2 -march=rv32imafc_zba_zbb_zbc_zbs_xw -gdwarf-2 -Og -std=c11 -fsigned-char -ffunction-sections -fdata-sections -fno-common"
.LASF21:
	.string	"findex"
.LASF0:
	.string	"float"
.LASF18:
	.string	"sinVal"
.LASF2:
	.string	"unsigned char"
.LASF8:
	.string	"long unsigned int"
.LASF4:
	.string	"short unsigned int"
.LASF23:
	.string	"libraries\\NMSIS\\DSP\\Source\\FastMathFunctions\\riscv_sin_f32.c"
.LASF5:
	.string	"__uint16_t"
.LASF16:
	.string	"double"
.LASF11:
	.string	"unsigned int"
.LASF25:
	.string	"sinTable_f32"
.LASF10:
	.string	"long long unsigned int"
.LASF14:
	.string	"long double"
.LASF9:
	.string	"long long int"
.LASF15:
	.string	"char"
.LASF19:
	.string	"fract"
.LASF20:
	.string	"index"
.LASF17:
	.string	"float32_t"
.LASF13:
	.string	"int32_t"
.LASF3:
	.string	"short int"
.LASF12:
	.string	"uint16_t"
.LASF7:
	.string	"long int"
.LASF26:
	.string	"riscv_sin_f32"
.LASF1:
	.string	"signed char"
	.ident	"GCC: (xPack GNU RISC-V Embedded GCC i386) 12.2.0"
