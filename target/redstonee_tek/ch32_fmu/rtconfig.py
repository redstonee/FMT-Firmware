import os

# board options
BOARD = 'ch32-fmu'

# toolchains options
ARCH = 'risc-v'
CPU = 'ch32'
CROSS_TOOL = 'gcc'
# build version: debug or release
BUILD = 'debug'

if os.getenv('RTT_CC'):
    CROSS_TOOL = os.getenv('RTT_CC')

# cross_tool provides the cross compiler
# EXEC_PATH is the compiler execute path, for example, CodeSourcery, Keil MDK, IAR
if CROSS_TOOL == 'gcc':
    PLATFORM = 'gcc'
    EXEC_PATH = 'your-compiler-path'
else:
    print('================ERROR============================')
    print('Not support %s yet!' % CROSS_TOOL)
    print('=================================================')
    exit(0)

if os.getenv('RTT_EXEC_PATH'):
    EXEC_PATH = os.getenv('RTT_EXEC_PATH')

if PLATFORM == 'gcc':
    # toolchains
    PREFIX = 'riscv32-wch-elf-'
    CC = PREFIX + 'gcc'
    AS = PREFIX + 'gcc'
    AR = PREFIX + 'ar'
    CXX = PREFIX + 'g++'
    LINK = PREFIX + 'gcc'
    TARGET_EXT = 'elf'
    SIZE = PREFIX + 'size'
    OBJDUMP = PREFIX + 'objdump'
    OBJCPY = PREFIX + 'objcopy'

    DEVICE = '-march=rv32imac_zicsr -mabi=ilp32 -msmall-data-limit=8 -msave-restore -fsigned-char -ffunction-sections -fdata-sections -fno-common -Wunused -Wuninitialized -DUSE_PLIC -DUSE_M_TIME -DNO_INIT -mcmodel=medany -lc '
   
    CFLAGS = DEVICE + ' -save-temps=obj'
    AFLAGS = '-c'+ DEVICE + ' -x assembler-with-cpp'
    LFLAGS = DEVICE + '-Wl,--gc-sections,--print-memory-usage,-cref,-Map=build/fmt_' + BOARD + '.map -T link.lds'

    CPATH = ''
    LPATH = ''

    if BUILD == 'debug':
        CFLAGS += ' -O0 -gdwarf-2'
        AFLAGS += ' -gdwarf-2'
    else:
        CFLAGS += ' -O2'

    CXXFLAGS = CFLAGS
    CFLAGS += ' -std=c11'
    CXXFLAGS += ' -std=c++23'

    POST_ACTION = OBJCPY + ' -O binary $TARGET build/fmt_' + BOARD + '.bin\n' + SIZE + ' $TARGET \n'
    