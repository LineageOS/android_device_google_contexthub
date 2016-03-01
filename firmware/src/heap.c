/*
 * Copyright (C) 2016 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <trylock.h>
#include <atomic.h>
#include <stdint.h>
#include <stdio.h>
#include <heap.h>



struct HeapNode {

    struct HeapNode* prev;
    uint32_t size:31;
    uint32_t used: 1;
    uint8_t  data[];
};

#ifdef FORCE_HEAP_IN_DOT_DATA

    static uint8_t __attribute__ ((aligned (8))) gHeap[HEAP_SIZE];

    #define REAL_HEAP_SIZE     ((HEAP_SIZE) &~ 7)
    #define ALIGNED_HEAP_START (&gHeap)

#else

    extern uint8_t __heap_end[], __heap_start[];
    #define ALIGNED_HEAP_START  (uint8_t*)((((uintptr_t)&__heap_start) + 7) &~ 7)
    #define ALIGNED_HEAP_END    (uint8_t*)(((uintptr_t)&__heap_end) &~ 7)

    #define REAL_HEAP_SIZE      (ALIGNED_HEAP_END - ALIGNED_HEAP_START)


#endif

static struct HeapNode* gHeapHead;
static TRYLOCK_DECL_STATIC(gHeapLock) = TRYLOCK_INIT_STATIC();
static volatile uint8_t gNeedFreeMerge = false; /* cannot be bool since its size is ill defined */
static struct HeapNode *gHeapTail;

static inline struct HeapNode* heapPrvGetNext(struct HeapNode* node)
{
    return (gHeapTail == node) ? NULL : (struct HeapNode*)(node->data + node->size);
}

bool heapInit(void)
{
    uint32_t size = REAL_HEAP_SIZE;
    struct HeapNode* node;

    node = gHeapHead = (struct HeapNode*)ALIGNED_HEAP_START;

    if (size < sizeof(struct HeapNode))
        return false;

    gHeapTail = node;

    node->used = 0;
    node->prev = NULL;
    node->size = size - sizeof(struct HeapNode);

    return true;
}

//called to merge free chunks in case free() was unable to last time it tried. only call with lock held please
static void heapMergeFreeChunks(void)
{
    while (atomicXchgByte(&gNeedFreeMerge, false)) {
        struct HeapNode *node = gHeapHead, *next;

        while (node) {
            next = heapPrvGetNext(node);

            if (!node->used && next && !next->used) { /* merged */
                node->size += sizeof(struct HeapNode) + next->size;

                next = heapPrvGetNext(node);
                if (next)
                    next->prev = node;
                else
                    gHeapTail = node;
            }
            else
                node = next;
        }
    }
}

void* heapAlloc(uint32_t sz)
{
    struct HeapNode *node, *best = NULL;
    void* ret = NULL;

    if (!trylockTryTake(&gHeapLock))
        return NULL;

    /* merge free chunks to help better use space */
    heapMergeFreeChunks();

    sz = (sz + 3) &~ 3;
    node = gHeapHead;

    while (node) {
        if (!node->used && node->size >= sz && (!best || best->size > node->size)) {
            best = node;
            if (best->size == sz)
                break;
        }

        node = heapPrvGetNext(node);
    }

    if (!best) //alloc failed
        goto out;

    if (best->size - sz > sizeof(struct HeapNode)) {        //there is a point to split up the chunk

        node = (struct HeapNode*)(best->data + sz);

        node->used = 0;
        node->size = best->size - sz - sizeof(struct HeapNode);
        node->prev = best;

        if (best != gHeapTail)
            heapPrvGetNext(node)->prev = node;
        else
            gHeapTail = node;

        best->size = sz;
    }

    best->used = 1;
    ret = best->data;

out:
    trylockRelease(&gHeapLock);
    return ret;
}

void heapFree(void* ptr)
{
    struct HeapNode *node, *t;
    bool haveLock;

    haveLock = trylockTryTake(&gHeapLock);

    node = ((struct HeapNode*)ptr) - 1;
    node->used = 0;

    if (haveLock) {

        while (node->prev && !node->prev->used)
            node = node->prev;

        while ((t = heapPrvGetNext(node)) && !t->used) {
            node->size += sizeof(struct HeapNode) + t->size;
            if (gHeapTail == t)
                gHeapTail = node;
        }

        if ((t = heapPrvGetNext(node)))
            t->prev = node;

        trylockRelease(&gHeapLock);
    }
    else
        gNeedFreeMerge = true;
}



