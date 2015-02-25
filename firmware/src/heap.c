#include <stdio.h>
#include <heap.h>



struct HeapNode {

    struct HeapNode* prev;
    uint32_t size:31;
    uint32_t used: 1;
    uint8_t  data[];
};

static struct HeapNode *gHeapTail;
static uint8_t __attribute__ ((aligned (8))) gHeap[102400]; /* 100K heap */

static inline struct HeapNode* heapPrvGetNext(struct HeapNode* node)
{
    return (gHeapTail == node) ? NULL : (struct HeapNode*)(node->data + node->size);
}

bool heapInit(void)
{
    uint32_t size = sizeof(gHeap);
    struct HeapNode* node;

    node = (struct HeapNode*)gHeap;

    if (size < sizeof(struct HeapNode))
        return false;

    gHeapTail = node;

    node->used = 0;
    node->prev = NULL;
    node->size = size - sizeof(struct HeapNode);

    return true;
}

void* heapAlloc(uint32_t sz)
{
    struct HeapNode* node = (struct HeapNode*)gHeap;
    struct HeapNode* best = NULL;
    void* ret = NULL;

    sz = (sz + 3) &~ 3;

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
    return ret;
}

void heapFree(void* ptr)
{
    struct HeapNode* node = (struct HeapNode*)gHeap;
    struct HeapNode* t;

    node = ((struct HeapNode*)ptr) - 1;

    node->used = 0;

    while (node->prev && !node->prev->used)
        node = node->prev;

    while ((t = heapPrvGetNext(node)) && !t->used) {
        node->size += sizeof(struct HeapNode) + t->size;
        if (gHeapTail == t)
            gHeapTail = node;
    }

    if ((t = heapPrvGetNext(node)))
        t->prev = node;
}



