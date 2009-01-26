#if !defined(__DEBUG_H__)
#define __DEBUG_H__

#include <ns16550.h>

extern NS16550_t debug_uart;
extern void putstr(NS16550_t debug_uart, const char *s);
extern char* ultohex(unsigned long num);
extern char* ustohex(unsigned short num);
extern char* uctohex(unsigned char num);

#endif        //  #if !defined(__DEBUG_H__)

