#include <apInt.h>
#include <gpio.h>
#include <variant/inc/variant.h>
#include <plat/inc/gpio.h>

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
