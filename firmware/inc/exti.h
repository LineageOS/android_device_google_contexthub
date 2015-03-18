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

void extiEnableIntGpio(const struct Gpio *__restrict gpio, ExtiTrigger trigger);
void extiEnableIntLine(uint32_t line, ExtiTrigger trigger);
void extiDisableIntGpio(const struct Gpio *__restrict gpio);
void extiDisableIntLine(uint32_t line);
void extiClearPendingGpio(const struct Gpio *__restrict gpio);
void extiClearPendingLine(uint32_t line);

#ifdef __cplusplus
}
#endif

#endif
