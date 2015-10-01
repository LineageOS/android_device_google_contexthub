#include <apInt.h>
#include <gpio.h>
#include <variant/inc/variant.h>
#include <plat/inc/gpio.h>

void apIntInit()
{
    struct Gpio gpio;

    gpioRequest(&gpio, AP_INT_WAKEUP);
    gpioConfigOutput(&gpio, GPIO_SPEED_LOW, GPIO_PULL_NONE, GPIO_OUT_PUSH_PULL, 1);

#ifdef AP_INT_NONWAKEUP
    gpioRequest(&gpio, AP_INT_NONWAKEUP);
    gpioConfigOutput(&gpio, GPIO_SPEED_LOW, GPIO_PULL_NONE, GPIO_OUT_PUSH_PULL, 1);
#endif
}

void apIntSet(bool wakeup)
{
    struct Gpio gpio;

    if (wakeup) {
        gpioRequest(&gpio, AP_INT_WAKEUP);
        gpioSet(&gpio, 0);
    } else {
#ifdef AP_INT_NONWAKEUP
        gpioRequest(&gpio, AP_INT_NONWAKEUP);
        gpioSet(&gpio, 0);
#endif
    }
}

void apIntClear(bool wakeup)
{
    struct Gpio gpio;

    if (wakeup) {
        gpioRequest(&gpio, AP_INT_WAKEUP);
        gpioSet(&gpio, 1);
    } else {
#ifdef AP_INT_NONWAKEUP
        gpioRequest(&gpio, AP_INT_NONWAKEUP);
        gpioSet(&gpio, 1);
#endif
    }
}
