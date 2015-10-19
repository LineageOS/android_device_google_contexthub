#ifndef _STM_TAGGED_PTR_H_
#define _STM_TAGGED_PTR_H_

#include <stdbool.h>
#include <stdint.h>


#define TAG	0x80000000UL  //no valid pointers that we care about in STM32F are at 0x80000000 or further

typedef uintptr_t TaggedPtr;

static inline void *taggedPtrToPtr(TaggedPtr tPtr)
{
    return (void*)tPtr;
}

static inline uintptr_t taggedPtrToUint(TaggedPtr tPtr)
{
    return tPtr &~ TAG;
}

static inline bool taggedPtrIsPtr(TaggedPtr tPtr)
{
    return !(tPtr & TAG);
}

static inline bool taggedPtrIsUint(TaggedPtr tPtr)
{
    return !taggedPtrIsPtr(tPtr);
}

static inline TaggedPtr taggedPtrMakeFromPtr(const void* ptr)
{
    return (uintptr_t)ptr;
}

static inline TaggedPtr taggedPtrMakeFromUint(uintptr_t ptr)
{
    return ptr | TAG;
}

#endif

