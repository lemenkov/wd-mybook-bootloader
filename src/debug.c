/*******************************************************************
 *
 * File:            debug.c
 *
 * Description:     Routines for sending debug messages
 *
 * Date:            03 March 2006
 *
 * Author:          B.H.Clarke
 *
 * Copyright:       Oxford Semiconductor Ltd, 2006
 *
 *******************************************************************/
#include <debug.h>

NS16550_t debug_uart;

void putstr(NS16550_t debug_uart, const char *s)
{
    while (*s) {
        putc_NS16550(debug_uart, *s++);
    }
}

char* ultohex(unsigned long num) {
    static char string[11];
    int i;

    string[0] = 'O';
    string[1] = 'x';
    for (i=9; i>=2; i--) {
        int digit = num & 0xf;
        if (digit <= 9) {
            string[i] = '0' + digit;
        } else {
            string[i] = 'a' + (digit-10);
        }
        num >>= 4;
    }
    string[10] = '\0';
    return string;
}

char* ustohex(unsigned short num) {
    static char string[7];
    int i;

    string[0] = 'O';
    string[1] = 'x';
    for (i=5; i>=2; i--) {
        int digit = num & 0xf;
        if (digit <= 9) {
            string[i] = '0' + digit;
        } else {
            string[i] = 'a' + (digit-10);
        }
        num >>= 4;
    }
    string[6] = '\0';
    return string;
}

char* uctohex(unsigned char num) {
    static char string[5];
    int i;

    string[0] = 'O';
    string[1] = 'x';
    for (i=3; i>=2; i--) {
        int digit = num & 0xf;
        if (digit <= 9) {
            string[i] = '0' + digit;
        } else {
            string[i] = 'a' + (digit-10);
        }
        num >>= 4;
    }
    string[4] = '\0';
    return string;
}

