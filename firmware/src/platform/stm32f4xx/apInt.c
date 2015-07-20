#include <apInt.h>
#include <gpio.h>
#include <variant/inc/variant.h>
#include <plat/inc/gpio.h>

void apIntInit()
{
#if defined(AP_INT_WAKEUP) || defined(AP_INT_NONWAKEUP)
    struct Gpio gpio;
#endif

#ifdef AP_INT_WAKEUP
    gpioRequest(&gpio, AP_INT_WAKEUP);
    gpioConfigOutput(&gpio, GPIO_SPEED_LOW, GPIO_PULL_NONE, GPIO_OUT_PUSH_PULL, 1);
#endif

#ifdef AP_INT_NONWAKEUP
    gpioRequest(&gpio, AP_INT_NONWAKEUP);
    gpioConfigOutput(&gpio, GPIO_SPEED_LOW, GPIO_PULL_NONE, GPIO_OUT_PUSH_PULL, 1);
#endif
}

void apIntSet(bool wakeup)
{
    if (wakeup) {
#ifdef AP_INT_WAKEUP
        struct Gpio gpio;

        gpioRequest(&gpio, AP_INT_WAKEUP);
        gpioSet(&gpio, 0);
#endif
    } else {
#ifdef AP_INT_NONWAKEUP
        struct Gpio gpio;

        gpioRequest(&gpio, AP_INT_NONWAKEUP);
        gpioSet(&gpio, 0);
#endif
    }
}

void apIntClear(bool wakeup)
{
    if (wakeup) {
#ifdef AP_INT_WAKEUP
        struct Gpio gpio;

        gpioRequest(&gpio, AP_INT_WAKEUP);
        gpioSet(&gpio, 1);
#endif
    } else {
#ifdef AP_INT_NONWAKEUP
        struct Gpio gpio;

        gpioRequest(&gpio, AP_INT_NONWAKEUP);
        gpioSet(&gpio, 1);
#endif
    }
}
