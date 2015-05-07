#ifndef _SLAB_H_
#define _SLAB_H_

#include <stdint.h>

struct SlabAllocator;



//thread/interrupt safe. allocations will not fail if space exists. even in interrupts.
//itemAlign over 4 will not be guaranteed since the heap does not hand out chunks with that kind of alignment
struct SlabAllocator* slabAllocatorNew(uint32_t itemSz, uint32_t itemAlign, uint32_t numItems);
void slabAllocatorDestroy(struct SlabAllocator *allocator);
void* slabAllocatorAlloc(struct SlabAllocator *allocator);
void slabAllocatorFree(struct SlabAllocator *allocator, void *ptr);

void* slabAllocatorGetNth(struct SlabAllocator *allocator, uint32_t idx); // -> pointer or NULL if that slot is empty   may be not int-safe. YMMV

#endif

