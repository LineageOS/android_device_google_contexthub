#include <trylock.h>
#include <atomic.h>


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

