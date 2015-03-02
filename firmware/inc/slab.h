#ifndef _SLAB_H_
#define _SLAB_H_

#include <stdint.h>

struct SlabAllocator;



//thread/interrupt safe. allocations will not fail if space exists. even in interrupts.
struct SlabAllocator* slabAllocatorNew(uint32_t itemSz, uint32_t itemAlign, uint32_t numItems);
void slabAllocatorDestroy(struct SlabAllocator *allocator);
void* slabAllocatorAlloc(struct SlabAllocator *allocator);
void slabAllocatorFree(struct SlabAllocator *allocator, void *ptr);


#endif

