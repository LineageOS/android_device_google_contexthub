#include <apInt.h>
#include <gpio.h>
#include <variant/inc/variant.h>
#include <plat/inc/gpio.h>

static struct Gpio *apIntWkup;
#ifdef AP_INT_NONWAKEUP
static struct Gpio *apIntNonWkup;
#endif

void apIntInit()
{
    apIntWkup = gpioRequest(AP_INT_WAKEUP);
    gpioConfigOutput(apIntWkup, GPIO_SPEED_LOW, GPIO_PULL_NONE, GPIO_OUT_PUSH_PULL, 1);

#ifdef AP_INT_NONWAKEUP
    apIntNonWkup = gpioRequest(AP_INT_NONWAKEUP);
    gpioConfigOutput(apIntNonWkup, GPIO_SPEED_LOW, GPIO_PULL_NONE, GPIO_OUT_PUSH_PULL, 1);
#endif
}

void apIntSet(bool wakeup)
{
    if (wakeup)
        gpioSet(apIntWkup, 0);
#ifdef AP_INT_NONWAKEUP
    else
        gpioSet(apIntNonWkup, 0);
#endif
}

void apIntClear(bool wakeup)
{
    if (wakeup)
        gpioSet(apIntWkup, 1);
#ifdef AP_INT_NONWAKEUP
    else
        gpioSet(apIntNonWkup, 1);
#endif
}
