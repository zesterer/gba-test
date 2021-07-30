ENTRY(__start)

MEMORY {
    ewram (w!x) : ORIGIN = 0x2000000, LENGTH = 256K
    iwram (w!x) : ORIGIN = 0x3000000, LENGTH = 32K
    rom (rx)    : ORIGIN = 0x8000000, LENGTH = 32M
}

SECTIONS {
    .text : {
        KEEP(target/crt0.o(.text));
        *(.text .text.*);
        . = ALIGN(4);
    } >rom = 0xff

    .rodata : {
        *(.rodata .rodata.*);
        . = ALIGN(4);

		/* debugging sections */
		/* Stabs */
		*(.stab)
		*(.stabstr)
		*(.stab.excl)
		*(.stab.exclstr)
		*(.stab.index)
		*(.stab.indexstr)
		*(.comment)
		/* DWARF 1 */
		*(.debug)
		*(.line)
		/* GNU DWARF 1 extensions */
		*(.debug_srcinfo)
		*(.debug_sfnames)
		/* DWARF 1.1 and DWARF 2 */
		*(.debug_aranges)
		*(.debug_pubnames)
		/* DWARF 2 */
		*(.debug_info)
		*(.debug_abbrev)
		*(.debug_line)
		*(.debug_frame)
		*(.debug_str)
		*(.debug_loc)
		*(.debug_macinfo)
		/* SGI/MIPS DWARF 2 extensions */
		*(.debug_weaknames)
		*(.debug_funcnames)
		*(.debug_typenames)
		*(.debug_varnames)
		. = ALIGN(4);
    } >rom = 0xff

    __data_lma = .;
    .data : {
        __data_start = ABSOLUTE(.);
        *(.data .data.*);
        . = ALIGN(4);
		*(.text_fast .text_fast.*);
        . = ALIGN(4);
        __data_end = ABSOLUTE(.);
    } >iwram AT>rom = 0xff

    __heap_lma = .;
    .heap : {
        __heap_start = ABSOLUTE(.);
        *(.heap .heap.*);
        . = ALIGN(4);
        __heap_end = ABSOLUTE(.);
    } >ewram AT>rom = 0xff

    .bss : {
        *(.bss .bss.*);
        . = ALIGN(4);
        __bss_end = ABSOLUTE(.);
    } >iwram

    /* discard anything not already mentioned */
    /DISCARD/ : { *(*) }
}
