/*******************************************************************
 *
 * File:            ns16550.c
 *
 * Description:     UART driver
 *
 * Date:            06 December 2005
 *
 * Author:          B.H.Clarke
 *
 * Copyright:       Oxford Semiconductor Ltd, 2005
 *
 *******************************************************************/
#include <ns16550.h>
#include <oxnas.h>

#define LCRVAL LCR_8N1                              /* 8 data, 1 stop, no parity */
#define MCRVAL (MCR_DTR | MCR_RTS)                  /* RTS/DTR */
#define FCRVAL (FCR_FIFO_EN | FCR_RXSR | FCR_TXSR)  /* Clear & enable FIFOs */

static int oxnas_fractional_divider(NS16550_t com_port, int baud_divisor_x16)
{
    // Baud rate is passed around x16
    int real_divisor = baud_divisor_x16 >> 4;
    // Top three bits of 8-bit dlf register hold the number of eigths
    // for the fractional part of the divide ratio
    com_port->dlf = (unsigned char)(((baud_divisor_x16 - (real_divisor << 4)) << 4) & 0xFF);
    // Return the x1 divider for the normal divider register
    return real_divisor;
}

void reinit_NS16550(NS16550_t com_port, int baud_divisor_x16)
{
    int baud_divisor = oxnas_fractional_divider(com_port, baud_divisor_x16);

    com_port->ier = 0x00;
    com_port->lcr = LCR_BKSE | LCRVAL;
    com_port->dll = baud_divisor & 0xff;
    com_port->dlm = (baud_divisor >> 8) & 0xff;
    com_port->lcr = LCRVAL;
    com_port->mcr = MCRVAL;
    com_port->fcr = FCRVAL;
}

void init_NS16550(NS16550_t com_port, int baud_divisor_x16)
{
    if (com_port == (NS16550_t)UART_1_BASE) {
        /* Block reset UART1 */
        *(volatile unsigned long*)SYS_CTRL_RSTEN_SET_CTRL = (1UL << SYS_CTRL_RSTEN_UART1_BIT);
        *(volatile unsigned long*)SYS_CTRL_RSTEN_CLR_CTRL = (1UL << SYS_CTRL_RSTEN_UART1_BIT);

        /* Setup pin mux'ing for UART1 */
        *(volatile unsigned long*)SYS_CTRL_GPIO_PRIMSEL_CTRL0 &= ~0x80000000;
        *(volatile unsigned long*)SYS_CTRL_GPIO_SECSEL_CTRL0  &= ~0x80000000;
        *(volatile unsigned long*)SYS_CTRL_GPIO_TERSEL_CTRL0  |=  0x80000000; // Route UART1 SOUT onto external pins

        *(volatile unsigned long*)SYS_CTRL_GPIO_PRIMSEL_CTRL1 &= ~0x00000001;
        *(volatile unsigned long*)SYS_CTRL_GPIO_SECSEL_CTRL1  &= ~0x00000001;
        *(volatile unsigned long*)SYS_CTRL_GPIO_TERSEL_CTRL1  |=  0x00000001; // Route UART1 SIN onto external pins

        *(volatile unsigned long*)GPIO_A_SET_OE |= 0x80000000;                // Make UART1 SOUT an o/p
        *(volatile unsigned long*)GPIO_B_CLR_OE |= 0x00000001;                // Make UART1 SIN an i/p
    }

    if (com_port == (NS16550_t)UART_2_BASE) {
        /* Block reset UART2 */
        *(volatile unsigned long*)SYS_CTRL_RSTEN_SET_CTRL = (1UL << SYS_CTRL_RSTEN_UART2_BIT);
        *(volatile unsigned long*)SYS_CTRL_RSTEN_CLR_CTRL = (1UL << SYS_CTRL_RSTEN_UART2_BIT);

        /* Setup pin mux'ing for UART2 */
        *(volatile unsigned long*)SYS_CTRL_GPIO_PRIMSEL_CTRL0 &= ~0x00500000;
        *(volatile unsigned long*)SYS_CTRL_GPIO_SECSEL_CTRL0  &= ~0x00500000;
        *(volatile unsigned long*)SYS_CTRL_GPIO_TERSEL_CTRL0  |=  0x00500000; // Route UART2 SOUT and SIN onto external pins

        *(volatile unsigned long*)GPIO_A_SET_OE |= 0x00100000;                // Make UART2 SOUT an o/p
        *(volatile unsigned long*)GPIO_A_CLR_OE |= 0x00400000;                // Make UART2 SIN an i/p
    }

    if (com_port == (NS16550_t)UART_3_BASE) {
        /* Block reset UART3 */
        *(volatile unsigned long*)SYS_CTRL_RSTEN_SET_CTRL = (1UL << SYS_CTRL_RSTEN_UART3_BIT);
        *(volatile unsigned long*)SYS_CTRL_RSTEN_CLR_CTRL = (1UL << SYS_CTRL_RSTEN_UART3_BIT);

        /* Setup pin mux'ing for UART2 */
        *(volatile unsigned long*)SYS_CTRL_GPIO_PRIMSEL_CTRL0 &= ~0x000000C0;
        *(volatile unsigned long*)SYS_CTRL_GPIO_SECSEL_CTRL0  &= ~0x000000C0;
        *(volatile unsigned long*)SYS_CTRL_GPIO_TERSEL_CTRL0  |=  0x000000C0; // Route UART3 SOUT and SIN onto external pins

        *(volatile unsigned long*)GPIO_A_SET_OE |= 0x00000080;                // Make UART3 SOUT an o/p
        *(volatile unsigned long*)GPIO_A_CLR_OE |= 0x00000040;                // Make UART3 SIN an i/p
    }

    reinit_NS16550(com_port, baud_divisor_x16);
}

