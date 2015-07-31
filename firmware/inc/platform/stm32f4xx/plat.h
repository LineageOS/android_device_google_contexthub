#ifndef _STM32_PLAT_H_
#define _STM32_PLAT_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>


enum PlatSleepDevID
{
    Stm32sleepDevTim2, /* we use this for short sleeps in WFI mode */
    Stm32sleepDevTim4, /* input capture uses this, potentially */
    Stm32sleepDevTim5, /* input capture uses this, potentially */

    Stm32sleepDevNum,  //must be last always, and must be <= PLAT_MAX_SLEEP_DEVS
};


#ifdef __cplusplus
}
#endif

#endif

