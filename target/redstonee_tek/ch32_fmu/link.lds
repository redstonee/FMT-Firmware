ENTRY( _start_v3f )

__stack_size = 2048;

PROVIDE( _stack_size = __stack_size );

PROVIDE( V5F_StartAddr = 0x1000 ); /* the start address of V5F core, which is defined in the startup code of V5F core, and should be consistent with the actual code location */
MEMORY
{
	/*FLASH, RAM, and RAM-CODE can be freely configured according to project 
	requirements;Please refer to the datasheet manual for specific allocation*/
	CODE_V3F (rx) : ORIGIN = 0x00000000, LENGTH = V5F_StartAddr             /* code for the V3F core, which is to bring up the V5F core */
	FLASH_V5F (rx) : ORIGIN = V5F_StartAddr, LENGTH = 960K - V5F_StartAddr   /* zero_wait flah ,and loaded into RAM_CODE for running */
	RAM_V5F_CODE (xrw) : ORIGIN = 0x200A0000, LENGTH = 128K                  /*ITCM RAM for running code*/
	RAM_DATA (xrw) : ORIGIN = (0x200C0000 + 512 + 256), LENGTH = (256K + 512K - 512 - 256)   /*DTCM and shared SRAM for .data .bss section and other*/ 
	RAM_LOAD (xrw) : ORIGIN = (0x200C0000 + 512), LENGTH = 256             /* RAM for .load section*/
}

SECTIONS
{
	.text_v3f :
	{
		. = ALIGN(4);
		*(.vector_v3f);
		*(.text_v3f)
		*(.text_v3f.*)
		*(.rodata)
		*(.rodata*)
        *(.sdata2.*)
		*(.glue_7)
		*(.glue_7t)
		*(.gnu.linkonce.t.*)
		. = ALIGN(4);
	} >CODE_V3F AT>CODE_V3F

    .loadcodelalign : 
    {       
        . = ALIGN(4);
        PROVIDE(_loadcode_lma = .); 
    } >FLASH_V5F AT>FLASH_V5F 

	.loadcode :
	{
		. = ALIGN(4);
        PROVIDE(_loadcode_vma_start = .);
		*(.load);
		. = ALIGN(4); 
		PROVIDE(_loadcode_vma_end = .);
	}>RAM_LOAD AT>FLASH_V5F

    .highcodelalign : 
    {       
        . = ALIGN(4);
        PROVIDE(_highcode_lma = .); 
    } >FLASH_V5F AT>FLASH_V5F 
    
    .highcode : 
    {
        . = ALIGN(4);
        PROVIDE(_highcode_vma_start = .);
        *(.vector);
		KEEP(*(SORT_NONE(.vector_handler)))
		*(.text)
		*(.text.*)
		*(.rodata)
		*(.rodata*)
        *(.sdata2.*)
		*(.glue_7)
		*(.glue_7t)
		*(.gnu.linkonce.t.*)
		. = ALIGN(4); 

		/* For RT-Thread */
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

        /* section information for modules */
        . = ALIGN(4);
        __rtmsymtab_start = .;
        KEEP(*(RTMSymTab))
        __rtmsymtab_end = .;
		. = ALIGN(4);

        /* section information for initial. */
        . = ALIGN(4);
        __rt_init_start = .;
        KEEP(*(SORT(.rti_fn*)))
        __rt_init_end = .;

        . = ALIGN(4);

        PROVIDE(_highcode_vma_end = .);
    } >RAM_V5F_CODE AT>FLASH_V5F

	.text :
	{
		. = ALIGN(4);
		KEEP(*(SORT_NONE(.handle_reset)))
		. = ALIGN(4);
	} >FLASH_V5F AT>FLASH_V5F

    .highcodelalign1 : 
    {       
        . = ALIGN(4);
        PROVIDE(_highcode_lma1 = .); 
    } >FLASH_V5F AT>FLASH_V5F 

	.fini :
	{
	    . = ALIGN(4);
        PROVIDE(_highcode_vma_start1 = .);
		KEEP(*(SORT_NONE(.fini)))
		. = ALIGN(4);
	} >RAM_V5F_CODE AT>FLASH_V5F

	PROVIDE( _etext = . );
	PROVIDE( _eitcm = . );	

	.preinit_array  :
	{
	  PROVIDE_HIDDEN (__preinit_array_start = .);
	  KEEP (*(.preinit_array))
	  PROVIDE_HIDDEN (__preinit_array_end = .);
	} >RAM_V5F_CODE AT>FLASH_V5F 
	
	.init_array     :
	{
	  PROVIDE_HIDDEN (__init_array_start = .);
	  KEEP (*(SORT_BY_INIT_PRIORITY(.init_array.*) SORT_BY_INIT_PRIORITY(.ctors.*)))
	  KEEP (*(.init_array EXCLUDE_FILE (*crtbegin.o *crtbegin?.o *crtend.o *crtend?.o ) .ctors))
	  PROVIDE_HIDDEN (__init_array_end = .);
	} >RAM_V5F_CODE AT>FLASH_V5F 
	
	.fini_array     :
	{
	  PROVIDE_HIDDEN (__fini_array_start = .);
	  KEEP (*(SORT_BY_INIT_PRIORITY(.fini_array.*) SORT_BY_INIT_PRIORITY(.dtors.*)))
	  KEEP (*(.fini_array EXCLUDE_FILE (*crtbegin.o *crtbegin?.o *crtend.o *crtend?.o ) .dtors))
	  PROVIDE_HIDDEN (__fini_array_end = .);
	} >RAM_V5F_CODE AT>FLASH_V5F 
	
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
	} >RAM_CODE AT>FLASH 
	
	.dtors          :
	{
	  KEEP (*crtbegin.o(.dtors))
	  KEEP (*crtbegin?.o(.dtors))
	  KEEP (*(EXCLUDE_FILE (*crtend.o *crtend?.o ) .dtors))
	  KEEP (*(SORT(.dtors.*)))
	  KEEP (*(.dtors))
	  . = ALIGN(4); 
      PROVIDE(_highcode_vma_end1 = .);
	} >RAM_DATA AT>FLASH_V5F

	.dalign :
	{
		. = ALIGN(4);
		PROVIDE(_data_vma = .);
	} >RAM_DATA AT>FLASH_V5F	

	.dlalign :
	{
		. = ALIGN(4); 
		PROVIDE(_data_lma = .);
	} >FLASH_V5F AT>FLASH_V5F

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
	} >RAM_DATA AT>FLASH_V5F

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
	} >RAM_DATA AT>FLASH_V5F

	PROVIDE( _end = _ebss);
	PROVIDE( end = . );

    .stack ORIGIN(RAM) + LENGTH(RAM) - __stack_size :
    {
        PROVIDE( _heap_end = . );    
        . = ALIGN(4);
        PROVIDE(_susrstack = . );
        . = . + __stack_size;
        PROVIDE( _eusrstack = .);
    } >RAM_DATA 

}



