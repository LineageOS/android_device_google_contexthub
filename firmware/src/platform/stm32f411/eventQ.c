#include <cpu/inc/pendsv.h>
#include <eventQ.h>
#include <stddef.h>
#include <stdio.h>
#include <heap.h>
#include <slab.h>
#include <cpu.h>






struct EvtRecord {
    struct EvtRecord *next;
    struct EvtRecord *prev;
    uint32_t evtType;
    void* evtData;
    EventFreeF evtFreeF;
};

struct EvtQueue {
    struct EvtRecord *head;
    struct EvtRecord *tail;
    struct SlabAllocator *evtsSlab;
};


static uint32_t mEvtDequeueStartPc, mEvtDequeueEndPc, mEvtDequeueCancelPc;


struct EvtQueue* evtQueueAlloc(uint32_t size)
{
    struct EvtQueue *q = heapAlloc(sizeof(struct EvtQueue));
    struct SlabAllocator *slab = slabAllocatorNew(sizeof(struct EvtRecord), 1, size);

    if (q && slab) {
        q->evtsSlab = slab;
        q->head = NULL;
        q->tail = NULL;
        return q;
    }

    if (q)
        heapFree(q);
    if (slab)
        slabAllocatorDestroy(slab);

    return NULL;
}

void evtQueueFree(struct EvtQueue* q)
{
    struct EvtRecord *t;

    while (q->head) {
        t = q->head;
        q->head = q->head->next;
        t->evtFreeF(t->evtData);
        slabAllocatorFree(q->evtsSlab, t);
    }

    slabAllocatorDestroy(q->evtsSlab);
    heapFree(q);
}

bool evtQueueEnqueue(struct EvtQueue* q, uint32_t evtType, void *evtData, EventFreeF freeF)
{
    struct EvtRecord *rec;
    uint64_t intSta;

    rec = slabAllocatorAlloc(q->evtsSlab);
    if (!rec) {
        intSta = cpuIntsOff();

        //find a victim for discarding
        rec = q->tail;
        while (rec && !(rec->evtType & EVENT_TYPE_BIT_DISCARDABLE))
            rec = rec->prev;

        if (rec) {
            rec->evtFreeF(rec->evtData);
            if (rec->prev)
                rec->prev->next = rec->next;
            else
                q->head = rec->next;
            if (rec->next)
                rec->next->prev = rec->prev;
            else
                q->tail = rec->prev;
        }

        cpuIntsRestore (intSta);
        if (!rec)
           return false;
    }

    rec->next = NULL;
    rec->evtType = evtType;
    rec->evtData = evtData;
    rec->evtFreeF = freeF;

    intSta = cpuIntsOff();
    rec->prev = q->tail;
    q->tail = rec;
    if (q->head)
        rec->prev->next = rec;
    else
        q->head = rec;

    cpuIntsRestore(intSta);
    return true;
}

bool evtQueueDequeue(struct EvtQueue* q, uint32_t *evtTypeP, void **evtDataP, EventFreeF *evtFreeFP, bool sleepIfNone)
{
    struct EvtRecord *rec = NULL;
    uint64_t intSta;

    while(1) {
        intSta = cpuIntsOff();

        rec = q->head;
        if (rec) {
            q->head = rec->next;
            if (q->head)
                q->head->prev = NULL;
            else
                q->tail = NULL;
            break;
        } else if (!sleepIfNone)
            break;
        /* we need to restore ints & then atomically check for events, and go to sleep. let's try that now */
        asm volatile(
            "    push {r0-r3, r12, lr} \n"
            "    adr %0, 1f            \n"
            "    str %0, [%1]          \n"
            "    adr %0, 2f            \n"
            "    str %0, [%2]          \n"
            "    adr %0, 3f            \n"
            "    str %0, [%3]          \n"
            "1:                        \n"
            "    msr PRIMASK, %4       \n"
            "    ldr %0, [%5]          \n"
            "    cmp %0, #0            \n"
            "    bne 4f                \n"
            "    bl platformSleep      \n" /* should be interruptible */
            "2:                        \n"
            "3:                        \n"
            "    bl platformWake       \n" /* should handle undoing whatever interrupted platformSleep() did */
            "4:                        \n"
            "    pop {r0-r3, r12, lr}  \n"
            :"=r"(rec)
            :"r"(&mEvtDequeueStartPc), "r"(&mEvtDequeueEndPc), "r"(&mEvtDequeueCancelPc), "r"((uint32_t)intSta), "r"(&q->head)
            :"memory","cc"
        );
    }

    cpuIntsRestore(intSta);
    if (rec) {
        *evtTypeP = rec->evtType;
        *evtDataP = rec->evtData;
        *evtFreeFP = rec->evtFreeF;
        slabAllocatorFree(q->evtsSlab, rec);

        return true;
    }

    return false;
}

static void evtQueuePendsvCbk(struct PendsvRegsLow *loRegs, struct PendsvRegsHi *hiRegs)
{
    if (loRegs->pc >= mEvtDequeueStartPc && loRegs->pc < mEvtDequeueEndPc)
        loRegs->pc = mEvtDequeueCancelPc;
}

bool evtQueueSubsystemInit(void)
{
    return pendsvSubscribe(evtQueuePendsvCbk);
}













