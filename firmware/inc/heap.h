#ifndef _HEAP_H_
#define _HEAP_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>




bool heapInit(void);
void* heapAlloc(uint32_t sz);
void heapFree(void* ptr);


#ifdef __cplusplus
}
#endif


#endif

