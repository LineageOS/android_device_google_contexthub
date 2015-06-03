#ifndef _X86_BARRIER_H_
#define _X86_BARRIER_H_


static inline void mem_reorder_barrier(void)
{
    asm volatile("":::"memory");
}

#endif

