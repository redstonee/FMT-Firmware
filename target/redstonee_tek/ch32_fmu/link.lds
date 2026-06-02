ENTRY( _start_v3f )

__stack_size = 8192;

PROVIDE( _stack_size = __stack_size );

MEMORY
{
	FLASH (rx): ORIGIN = 0x00000000, LENGTH = 960K		/* Flash */
	ITCM (rwx): ORIGIN = 0x200A0000, LENGTH = 128K		/* ITCM for V5F */
	DTCM (rwx): ORIGIN = 0x200C0000, LENGTH = 256K		/* DTCM for V5F */
	RAM (rwx): ORIGIN = 0x20100000, LENGTH = 512K		/* Shared RAM for V5F and V3F */
}

SECTIONS
{
	.init_v3f :
	{
		. = ALIGN(4);
		KEEP(*(SORT_NONE(.init_v3f)))
		*(.init_v3f.bringup)
	} >FLASH AT>FLASH

	.init_v5f :
	{
		. = ALIGN(1024);
		/* Must be 1KiB aligned */
		KEEP(*(SORT_NONE(.init_v5f)))
	} >FLASH AT>FLASH

	.vector_v3f :
	{
		. = ALIGN(1024);
		/* Must be 1KiB aligned */
        *(.vector_v3f);
	} >FLASH AT>FLASH
	
	.fast_code :
	{
		. = ALIGN(1024);
		PROVIDE (__stext = .);
		PROVIDE(_fast_code_vma_start = .);
		/* Must be 1KiB aligned */
		*(.fast_code.vector_v5f); 
		*(.fast_code.vector_handler);
		*(.fast_code);
		/* Force model and some kernel text into ITCM */
		*build*fmt*src*model*.o(.text .text.*);
		*build*kernel*src*.o(.text .text.*);
		*build*kernel*libcpu*.o(.text .text.*);
		. = ALIGN(4); 
		PROVIDE(_fast_code_vma_end = .);
	}>ITCM AT>FLASH
	PROVIDE(_fast_code_lma = LOADADDR(.fast_code));

	.ram_code :
	{
		. = ALIGN(4);
		PROVIDE(_ram_code_vma_start = .);

		*(.text)
		*(.text.*)
		*(.rodata)
		*(.rodata*)
		. = ALIGN(4); 
    	PROVIDE (__etext = .);
		PROVIDE(_ram_code_vma_end = .);
	}>RAM AT>FLASH
	PROVIDE(_ram_code_lma = LOADADDR(.ram_code));

	.slow_shit :
	{
		. = ALIGN(4);

		*(.gnu.linkonce.t.*)
		. = ALIGN(4);  
		   /* section information for finsh shell */
        __fsymtab_start = .;
        KEEP(*(FSymTab))
        __fsymtab_end = .;
        . = ALIGN(4);
        __vsymtab_start = .;
        KEEP(*(VSymTab))
        __vsymtab_end = .;
        . = ALIGN(4);

        /* section information for initial. */
        __rt_init_start = .;
        KEEP(*(SORT(.rti_fn*)))
        __rt_init_end = .;
        . = ALIGN(4);

        PROVIDE(__ctors_start__ = .);
        KEEP(*(SORT(.init_array.*)))
        KEEP(*(.init_array))
        PROVIDE(__ctors_end__ = .);
        . = ALIGN(4);

        /* section information for parameters. */
        . = ALIGN(4);
        __fmt_param_start = .;
        KEEP(*(ParamTab))
        __fmt_param_end = .;
        . = ALIGN(4);  

        /* section information for mlog. */
        . = ALIGN(4);
        __fmt_mlog_start = .;
        KEEP(*(MlogTab))
        __fmt_mlog_end = .;
        . = ALIGN(4);
		
		/* section information for utest */
		. = ALIGN(4);
		__rt_utest_tc_tab_start = .;
		KEEP(*(UtestTcTab))
		__rt_utest_tc_tab_end = .;
        . = ALIGN(4);

        /* section information for task. */
        . = ALIGN(4);
        __fmt_task_start = .;
        KEEP(*(TaskTab))
        __fmt_task_end = .;
		. = ALIGN(4);
	} >FLASH AT>FLASH

	.data :
	{
		. = ALIGN(4);
		PROVIDE(_data_vma = .);
    	*(.gnu.linkonce.r.*)
    	*(.data .data.*)
    	*(.gnu.linkonce.d.*)
		. = ALIGN(8);
    	PROVIDE( __global_pointer$ = . + 0x800 );
    	*(.sdata .sdata.*)
		*(.sdata2.*)
    	*(.gnu.linkonce.s.*)
    	. = ALIGN(8);
    	*(.srodata.cst16)
    	*(.srodata.cst8)
    	*(.srodata.cst4)
    	*(.srodata.cst2)
    	*(.srodata .srodata.*)
		PROVIDE(__dtors_start__ = .);
        KEEP(*(SORT(.dtors.*)))
        KEEP(*(.dtors))
        PROVIDE(__dtors_end__ = .);
		. = ALIGN(4);
		
		PROVIDE(_data_end = .);
		PROVIDE( _edata = .);
	} >DTCM AT>FLASH
	PROVIDE(_data_lma = LOADADDR(.data));

	.bss :
	{
		. = ALIGN(4);
		PROVIDE( _sbss = .);
  	    *(.sbss*)
        *(.gnu.linkonce.sb.*)
		*(.bss*)
     	*(.gnu.linkonce.b.*)		
		*(COMMON*)
		. = ALIGN(4);
		PROVIDE( _ebss = .);
		PROVIDE( _heap_start = .);
	} >DTCM

	PROVIDE( _heap_end = ORIGIN(DTCM) + LENGTH(DTCM) );

    .stack ORIGIN(RAM) + LENGTH(RAM) - __stack_size :
    {
        . = ALIGN(4);
        PROVIDE(_susrstack = . );
        . = . + __stack_size;
        PROVIDE( _eusrstack = .);
        PROVIDE( __rt_rvstack = . );
    } >RAM

}



