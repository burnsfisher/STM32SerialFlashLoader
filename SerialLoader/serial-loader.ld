/*
 * Copyright © 2012 Keith Packard <keithp@keithp.com>
 *             2021 Burns Fisher <wb1fj@amsat.org>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA.
 */

MEMORY {
	rom : ORIGIN = 0x08000000, LENGTH = 5K
	ram : ORIGIN = 0x20000000, LENGTH = 32K
	Save : ORIGIN = 0x2000c000-64, LENGTH = 64
/*	ErrorData : ORIGIN = 0x2000C000-64, LENGTH = 64
	*/
}

INCLUDE ../registers.ld

EXTERN (stm_interrupt_vector)

SECTIONS {
	/*
	 * Rom contents
	 */

	.text : {
		__text_start__ = .;
		*(.interrupt)	/* Interrupt vectors */

		. = ORIGIN(rom) + 0x100;
/*
		src/ao_romconfig.o(.romconfig*)
		src/ao_product.o(.romconfig*)
*/
		*(.text*)	/* Executable code */
		*(.ARM.exidx* .gnu.linkonce.armexidx.*)
		*(.rodata*)	/* Constants */
	} > rom
	__text_end__ = .;
	__ramTextSrc = .;

	/* Boot data which must live at the start of ram so that
	 * the application and bootloader share the same addresses.
	 * This must be all uninitialized data
	 */
	.boot (NOLOAD) : {
		__boot_start__ = .;
		*(.boot)
		__boot_end__ = .;
	} >ram
	
	/* Functions placed in RAM (required for flashing)
	 *
	 * Align to 8 bytes as that's what the ARM likes text
	 * segment alignments to be, and if we don't, then
	 * we end up with a mismatch between the location in
	 * ROM and the desired location in RAM. I don't
	 * entirely understand this, but at least this appears
	 * to work...
	 */
   .textRam BLOCK(8): {
   __ramTextDst = .;
   *(.ramtext)
   __ramTextEnd = .;
   } >ram AT>rom

	/* Data -- relocated to RAM, but written to ROM
	 * Also aligned to 8 bytes to agree with textram
	 */
	.data BLOCK(8): {
		*(.data)	/* initialized data */
	} >ram AT>rom

	.bss : {
		__ramCopyEnd = .;
		__bss_start__ = .;
		*(.bss)
		*(COMMON)
		__bss_end__ = .;
	} >ram

	  .ERROR_RAM (NOLOAD) : {
  	*(.ERROR_RAM)
    . = ALIGN(4);
    } >Save

	PROVIDE(__stack__ = ORIGIN(ram) + LENGTH(ram));
	PROVIDE(end = .);
}

ENTRY(start);


