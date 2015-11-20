#ifndef _SIMPLE_Q_H_
#define _SIMPLE_Q_H_


#include <stdbool.h>
#include <stdint.h>


#define SIMPLE_QUEUE_MAX_ELEMENTS 0x0FFFFFFE

typedef bool (*SimpleQueueForciblyDiscardCbkF)(void *data, bool onDelete); //return false to reject

//SINGLE producer, SINGLE consumer queue. data is copied INTO/OUT of the queue by simpleQueueEnqueue/simpleQueueDequeue

struct SimpleQueue* simpleQueueAlloc(uint32_t numEntries, uint32_t entrySz, SimpleQueueForciblyDiscardCbkF forceDiscardCbk);
void simpleQueueDestroy(struct SimpleQueue* sq); //will call discard, but in no particular order!
bool simpleQueueEnqueue(struct SimpleQueue* sq, const void *data, bool possiblyDiscardable);
bool simpleQueueDequeue(struct SimpleQueue* sq, void *dataVal);


#endif

