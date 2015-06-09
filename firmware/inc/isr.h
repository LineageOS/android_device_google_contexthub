#ifndef __ISR_H
#define __ISR_H

#include <stdbool.h>
#include <stdint.h>

#include <cpu.h>
#include <list.h>
#include <util.h>

struct ChainedInterrupt {
    link_t isrs;

    void (*const enable)(struct ChainedInterrupt *);
    void (*const disable)(struct ChainedInterrupt *);
};

struct ChainedIsr {
    link_t node;
    bool (*func)(struct ChainedIsr *);
};

static inline void chainIsr(struct ChainedInterrupt *interrupt, struct ChainedIsr *isr)
{
    interrupt->disable(interrupt);
    list_add_tail(&interrupt->isrs, &isr->node);
    interrupt->enable(interrupt);
}

static inline void unchainIsr(struct ChainedInterrupt *interrupt, struct ChainedIsr *isr)
{
    interrupt->disable(interrupt);
    list_delete(&isr->node);
    if (!list_is_empty(&interrupt->isrs))
        interrupt->enable(interrupt);
}

static inline bool dispatchIsr(struct ChainedInterrupt *interrupt)
{
    struct link_t *cur, *tmp;
    bool handled = false;

    list_iterate(&interrupt->isrs, cur, tmp) {
        struct ChainedIsr *curIsr = container_of(cur, struct ChainedIsr, node);
        handled = handled || curIsr->func(curIsr);
    }

    return handled;
}

#endif /* __ISR_H */
