#ifndef _STM32F4_CRC_H_
#define _STM32F4_CRC_H_

#include <stdint.h>

#define STM32F4_CRC_RESIDUE 0xC704DD7B

uint32_t stm32f4_crc32(uint8_t *buffer, int length);

#endif /* _STM32F4_CRC_H_ */
