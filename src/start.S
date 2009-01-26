/*
 *	Oxford Semiconductor Ltd 2005
 *
 *	Created: John Larkworthy 28 July 2005
 *
 *	Title  : start.s
 *
 *	Description:
 *	Entry point for stage1 loader to be copied from disk into SRAM by the ROM
 *	loader boot-from-SATA code
 *
 *	Modifications:
 *		BHC: 24/05/06	Reworked to add stage2 loader environment load from disk
 *      JJL: 21/07/06   Restructured to provide space for boot loader validation
 */	
.section .init
.org 0
	ldr pc, = start

.org 0x30 
image_length: .word code_length
image_crc: .word 0
header_crc: .word 0

.section .text	
/* end of header so start code here
 */
.set GPIO_B_SET,  0x41100014

start:
/* remove PENDING reset triggered in the bootrom when program starts (bootrom coding error!)*/
	ldr 	r0, = 0x4
	ldr	r1, = GPIO_B_SET
	str	r0, [r1]
	ldr	r13,	initial_stack
	ldr	pc,		= main

initial_stack:	.word	0x4c007ff0

