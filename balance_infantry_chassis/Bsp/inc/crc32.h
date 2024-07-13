#ifndef CRC32_H
#define CRC32_H

#include <stdint.h>

uint32_t crc32_core(uint32_t* ptr, uint32_t len);
uint32_t crc32(void* input, int len);

extern void crc32_init(uint32_t poly);
#endif  // CRC32_H
