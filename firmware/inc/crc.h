#ifndef __CRC_H
#define __CRC_H

#include <stddef.h>
#include <stdint.h>

/**
 * Implements CRC with the following parameters:
 *
 * Width:   32
 * Poly:    04C11DB7
 * Init:    FFFFFFFF
 * RefIn:   False
 * RefOut:  False
 * XorOut:  00000000
 *
 * The CRC implementation will pad the buffer with zeroes to the nearest
 * multiple of 4 bytes (if necessary).
 */
uint32_t crc32(const void *buf, size_t size);

#endif /* __CRC_H */
