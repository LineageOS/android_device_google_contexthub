#ifndef _EVENT_Q_H_
#define _EVENT_Q_H_


#include <stdbool.h>
#include <stdint.h>


#define EVENT_TYPE_BIT_DISCARDABLE    0x80000000 /* set for events we can afford to lose */

struct EvtQueue;

typedef void (*EvtQueueForciblyDiscardEvtCbkF)(uint32_t evtType, void *evtData, uintptr_t evtFreeData);

//multi-producer, SINGLE consumer queue

struct EvtQueue* evtQueueAlloc(uint32_t size, EvtQueueForciblyDiscardEvtCbkF forceDiscardCbk);
void evtQueueFree(struct EvtQueue* q);
bool evtQueueEnqueue(struct EvtQueue* q, uint32_t evtType, void *evtData, uintptr_t evtFreeData);
bool evtQueueDequeue(struct EvtQueue* q, uint32_t *evtTypeP, void **evtDataP, uintptr_t *evtFreeDataP, bool sleepIfNone);


#endif

