#ifndef __HOSTINTF_H
#define __HOSTINTF_H

#include <stdint.h>
#include <atomicBitset.h>

/**
 * System-facing hostIntf API
 */

#define MAX_INTERRUPTS      256

void hostIntfCopyClearInterrupts(struct AtomicBitset *dst, uint32_t numBits);
void hostIntfSetInterrupt(uint32_t bit);
void hostInfClearInterrupt(uint32_t bit);
void hostIntfSetInterruptMask(uint32_t bit);
void hostInfClearInterruptMask(uint32_t bit);
void hostIntfPacketFree(void *ptr);
bool hostIntfPacketDequeue(void **ptr, int *length, int *sensor);

#endif /* __HOSTINTF_H */
