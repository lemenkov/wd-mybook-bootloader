#ifndef CRC32_H
#define CRC32_H


/* this is crc32 ( old crc, start address, length in bytes) returns new crc */
unsigned long crc32(unsigned long, const unsigned char *, unsigned int);


#endif
