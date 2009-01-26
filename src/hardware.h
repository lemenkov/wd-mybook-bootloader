@ Hardware description file for inclusion in assembler source 
@ 
@Title hardware.h @ Author:John Larkworthy
@ (c) Oxford Semiconductors Ltd 2005.
@ *********************************************
@ JJL: 21/07/06 Remove stack base constant.
@

.equ RPSA, (0x45300000)
.equ _WATCHDOG_REGISTER, (0X0380 + RPSA)
.equ _RESET_STATUS, (0x0330 + RPSA)

.equ _WATCHDOG_MASK, 0X20D15F88	    /* inhibit watchdog value. */
.equ GPIO_BASE, 0X44100000
.equ GPIO_BOOT_REG, (GPIO_BASE + 0X0)
.equ GPIO_BOOT_MASK, 0X3
.equ GPIO_BOOT_SHIFT, 0
.equ START_OF_SRAM, (0X4C000000)
.equ SRAM_SIZE, 0X8000
.equ RAM_SIZE, 0X1000
.equ DTCP_STACK, (START_OF_SRAM + 0x400)
