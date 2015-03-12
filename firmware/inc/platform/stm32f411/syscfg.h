#ifndef _STM32F411_SYSCFG_H_
#define _STM32F411_SYSCFG_H_

#include <plat/inc/gpio.h>

#ifdef __cplusplus
extern "C" {
#endif

void syscfgSetExtiPort(const struct Gpio *__restrict gpio);

#ifdef __cplusplus
}
#endif

#endif
