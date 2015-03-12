#ifndef _EXTI_H_
#define _EXTI_H_

#include <plat/inc/gpio.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    EXTI_TRIGGER_RISING = 0,
    EXTI_TRIGGER_FALLING,
    EXTI_TRIGGER_BOTH,
} ExtiTrigger;

void extiEnableInt(const struct Gpio *__restrict gpio, ExtiTrigger trigger);
void extiDisableInt(const struct Gpio *__restrict gpio);
void extiClearPending(const struct Gpio *__restrict gpio);

#ifdef __cplusplus
}
#endif

#endif
