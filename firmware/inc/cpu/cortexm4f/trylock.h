#ifndef _CM4F_TRYLOCK_H_
#define _CM4F_TRYLOCK_H_

#include <stdint.h>
#include <stdbool.h>

struct TryLock {
    volatile uint8_t lock;
};

#define TRYLOCK_DECL_STATIC(name)   struct TryLock name
#define TRYLOCK_INIT_STATIC()       {0}

void trylockInit(struct TryLock *lock);
void trylockRelease(struct TryLock *lock);
bool trylockTryTake(struct TryLock *lock); //true if we took it

/* DON'T YOU EVER DARE TO TRY AND IMPLEMENT A BLOCKING "TAKE" ON THIS TYPE OF LOCK!   -dmitrygr@ */


#endif

