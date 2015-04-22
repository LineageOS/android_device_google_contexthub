#include <plat/inc/exti.h>
#include <plat/inc/pwr.h>

struct StmExti
{
    volatile uint32_t IMR;
    volatile uint32_t EMR;
    volatile uint32_t RTSR;
    volatile uint32_t FTSR;
    volatile uint32_t SWIER;
    volatile uint32_t PR;
};

#define EXTI ((struct StmExti*)EXTI_BASE)

void extiEnableIntLine(const enum ExtiLine line, enum ExtiTrigger trigger)
{
    if (trigger == EXTI_TRIGGER_BOTH) {
        EXTI->RTSR |= (1UL << line);
        EXTI->FTSR |= (1UL << line);
    } else if (trigger == EXTI_TRIGGER_RISING) {
        EXTI->RTSR |= (1UL << line);
        EXTI->FTSR &= ~(1UL << line);
    } else if (trigger == EXTI_TRIGGER_FALLING) {
        EXTI->RTSR &= ~(1UL << line);
        EXTI->FTSR |= (1UL << line);
    }

    /* Clear pending interrupt */
    extiClearPendingLine(line);

    /* Enable hardware interrupt */
    EXTI->IMR |= (1UL << line);
}

void extiDisableIntLine(const enum ExtiLine line)
{
    EXTI->IMR &= ~(1UL << line);
}

void extiClearPendingLine(const enum ExtiLine line)
{
    EXTI->PR |= (1UL << line);
}

bool extiIsPendingLine(const enum ExtiLine line)
{
    return (EXTI->PR & (1UL << line)) ? true : false;
}
