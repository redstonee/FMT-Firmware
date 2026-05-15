openocd-wch -f wch-dual-core.cfg -c init -c halt\
    -c 'flash write_image build/fmt_ch32-fmu.elf'\
    -c 'verify_image build/fmt_ch32-fmu.elf' -c reset -c exit