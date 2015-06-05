#include <syscall.h>
#include <heap.h>




typedef uintptr_t (*UserspaceCbkF)(uintptr_t, uintptr_t, uintptr_t, uintptr_t);
static uintptr_t doUserspaceCall(uintptr_t p1, uintptr_t p2, uintptr_t p3, uintptr_t p4, UserspaceCbkF cbk);


struct UserspaceCallback
{
    UserspaceCbkF f;
    uintptr_t p1;
    uintptr_t p2;
    uintptr_t p3;
    uintptr_t p4;
};

struct UserspaceCallback* syscallUserspaceCallbackAlloc(void *func, uintptr_t param1, uintptr_t param2, uintptr_t param3, uintptr_t param4)
{
    struct UserspaceCallback *ucbk = heapAlloc(sizeof(struct UserspaceCallback));

    if (ucbk) {
        ucbk->f = (UserspaceCbkF)func;
        ucbk->p1 = param1;
        ucbk->p2 = param2;
        ucbk->p3 = param3;
        ucbk->p4 = param4;
    }

    return ucbk;
}

void syscallUserspaceCallbackFree(struct UserspaceCallback *ucbk)
{
    heapFree(ucbk);
}

uintptr_t syscallUserspaceCallbackCall(struct UserspaceCallback *ucbk, const uintptr_t *overrideParam1P, const uintptr_t *overrideParam2P, const uintptr_t *overrideParam3P, const uintptr_t *overrideParam4P)
{
    uintptr_t p1 = overrideParam1P ? *overrideParam1P : ucbk->p1;
    uintptr_t p2 = overrideParam1P ? *overrideParam2P : ucbk->p2;
    uintptr_t p3 = overrideParam1P ? *overrideParam3P : ucbk->p3;
    uintptr_t p4 = overrideParam1P ? *overrideParam4P : ucbk->p4;

    return doUserspaceCall(p1, p2, p3, p4, ucbk->f);
}


/* soon this will be a behemoth of a function, dealing with context switches and all, for now it is simple */
static uintptr_t __attribute__((naked)) doUserspaceCall(uintptr_t p1, uintptr_t p2, uintptr_t p3, uintptr_t p4, UserspaceCbkF cbk)
{
    asm volatile (
        "ldr pc, [sp]   \n"
    );

    /* not reached, just here to make gcc happy */
    return 0;
}





