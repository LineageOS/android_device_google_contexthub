#include <cpu/inc/trylock.h>


void trylockInit(struct TryLock *lock)
{
    lock->lock = 0;
}

void trylockRelease(struct TryLock *lock)
{
    lock->lock = 0;
}

bool trylockTryTake(struct TryLock *lock)
{
    uint32_t prevVal, storeFailed;

    do {
        asm volatile(
            "ldrexb %0, [%2]     \n"
            "strexb %1, %3, [%2] \n"
            :"=r"(prevVal), "=r"(storeFailed)
            :"r"(&lock->lock), "r"(1)
            :"memory"
        );
    } while (storeFailed);

    return !prevVal;
}

