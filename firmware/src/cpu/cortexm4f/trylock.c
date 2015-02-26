#include <cpu/inc/trylock.h>
#include <cpu/inc/atomicXchg.h>


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
    return !atomicXchgByte(&lock->lock, 1);
}

