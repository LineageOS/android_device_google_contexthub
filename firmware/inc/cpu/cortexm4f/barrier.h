#ifndef _CM4F_BARRIER_H_
#define _CM4F_BARRIER_H_


static inline void mem_reorder_barrier(void)
{
    asm volatile("":::"memory");
}

#endif

