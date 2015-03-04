#ifndef _CM4F_PENDSV_H_
#define _CM4F_PENDSV_H_

#include <stdbool.h>
#include <stdint.h>

#define MAX_PENDSV_SUBSCRIBERS	4

struct PendsvRegsLow {
    uint32_t r0, r1, r2, r3;
    uint32_t r12, lr, pc, cpsr;
};

struct PendsvRegsHi {
    uint32_t r4, r5, r6, r7;
    uint32_t r8, r9, r10, r11;
};

typedef void (*PendsvCallbackF)(struct PendsvRegsLow *loRegs, struct PendsvRegsHi *hiRegs);

bool pendsvSubscribe(PendsvCallbackF cbk);	//may not be interrupt safe if reentered
bool pendsvUnsubscribe(PendsvCallbackF cbk);
void pendsvTrigger(void);
void pendsvClear(void);
bool pendsvIsPending(void);


#endif

