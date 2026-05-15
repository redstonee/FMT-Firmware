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
		. = ALIGN(1024); /* Must be 1KiB aligned */
		KEEP(*(SORT_NONE(.init_v5f)))
	} >FLASH AT>FLASH

	.vector_v3f :
	{
		. = ALIGN(1024); /* Must be 1KiB aligned */
        *(.vector_v3f);
	} >FLASH AT>FLASH
	
    .fast_code_lalign : 
    {       
        . = ALIGN(4);
        PROVIDE(_fast_code_lma = .); 
    } >FLASH AT>FLASH

	.fast_code :
	{
		. = ALIGN(1024);
        PROVIDE(_fast_code_vma_start = .);
		*(.fast_code.vector_v5f); /* Must be 1KiB aligned */
		*(.fast_code.vector_handler);
		*(.fast_code);
		. = ALIGN(4); 
		PROVIDE(_fast_code_vma_end = .);
	}>ITCM AT>FLASH

	.text :
	{
		. = ALIGN(4);
		*(.vector);
		*(.text)
		*(.text.*)
		*(.rodata)
		*(.rodata*)
        *(.sdata2.*)
		*(.glue_7)
		*(.glue_7t)
		*(.gnu.linkonce.t.*)

        /* section information for finsh shell */
        . = ALIGN(4);
        __fsymtab_start = .;
        KEEP(*(FSymTab))
        __fsymtab_end = .;
        . = ALIGN(4);
        __vsymtab_start = .;
        KEEP(*(VSymTab))
        __vsymtab_end = .;
        . = ALIGN(4);

        /* section information for initial. */
        . = ALIGN(4);
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

	.fini :
	{
	    . = ALIGN(4);
        PROVIDE(_highcode_vma_start1 = .);
		KEEP(*(SORT_NONE(.fini)))
		. = ALIGN(4);
	} >FLASH AT>FLASH

	PROVIDE( _etext = . );
	PROVIDE( _eitcm = . );	

	.preinit_array  :
	{
	  PROVIDE_HIDDEN (__preinit_array_start = .);
	  KEEP (*(.preinit_array))
	  PROVIDE_HIDDEN (__preinit_array_end = .);
	} >FLASH AT>FLASH
	
	.init_array     :
	{
	  PROVIDE_HIDDEN (__init_array_start = .);
	  KEEP (*(SORT_BY_INIT_PRIORITY(.init_array.*) SORT_BY_INIT_PRIORITY(.ctors.*)))
	  KEEP (*(.init_array EXCLUDE_FILE (*crtbegin.o *crtbegin?.o *crtend.o *crtend?.o ) .ctors))
	  PROVIDE_HIDDEN (__init_array_end = .);
	} >FLASH AT>FLASH 
	
	.fini_array     :
	{
	  PROVIDE_HIDDEN (__fini_array_start = .);
	  KEEP (*(SORT_BY_INIT_PRIORITY(.fini_array.*) SORT_BY_INIT_PRIORITY(.dtors.*)))
	  KEEP (*(.fini_array EXCLUDE_FILE (*crtbegin.o *crtbegin?.o *crtend.o *crtend?.o ) .dtors))
	  PROVIDE_HIDDEN (__fini_array_end = .);
	} >FLASH AT>FLASH
	
	.ctors          :
	{
	  /* gcc uses crtbegin.o to find the start of
	     the constructors, so we make sure it is
	     first.  Because this is a wildcard, it
	     doesn't matter if the user does not
	     actually link against crtbegin.o; the
	     linker won't look for a file to match a
	     wildcard.  The wildcard also means that it
	     doesn't matter which directory crtbegin.o
	     is in.  */
	  KEEP (*crtbegin.o(.ctors))
	  KEEP (*crtbegin?.o(.ctors))
	  /* We don't want to include the .ctor section from
	     the crtend.o file until after the sorted ctors.
	     The .ctor section from the crtend file contains the
	     end of ctors marker and it must be last */
	  KEEP (*(EXCLUDE_FILE (*crtend.o *crtend?.o ) .ctors))
	  KEEP (*(SORT(.ctors.*)))
	  KEEP (*(.ctors))
	} >FLASH AT>FLASH 
	
	.dtors          :
	{
	  KEEP (*crtbegin.o(.dtors))
	  KEEP (*crtbegin?.o(.dtors))
	  KEEP (*(EXCLUDE_FILE (*crtend.o *crtend?.o ) .dtors))
	  KEEP (*(SORT(.dtors.*)))
	  KEEP (*(.dtors))
	} >FLASH AT>FLASH

	.dalign :
	{
		. = ALIGN(4);
		PROVIDE(_data_vma = .);
	} >RAM AT>FLASH

	.dlalign :
	{
		. = ALIGN(4); 
		PROVIDE(_data_lma = .);
	} >FLASH AT>FLASH

	.data :
	{
		. = ALIGN(4);
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
    	. = ALIGN(4);
		PROVIDE( _edata = .);
	} >RAM AT>FLASH

    __bss_start = .;
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
	} >RAM

	PROVIDE( _end = _ebss);
	PROVIDE( end = . );

	PROVIDE( _ssram = ORIGIN(RAM));
    .stack ORIGIN(RAM) + LENGTH(RAM) - __stack_size :
    {
        PROVIDE( _heap_end = . );    
        . = ALIGN(4);
        PROVIDE(_susrstack = . );
        . = . + __stack_size;
        PROVIDE( _eusrstack = .);
    } >RAM

}



