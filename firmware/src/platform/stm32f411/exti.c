#include <exti.h>
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

void extiEnableIntGpio(const struct Gpio *__restrict gpio, ExtiTrigger trigger)
{
    const uint8_t pinNo = gpio->gpio & GPIO_PIN_MASK;
    extiEnableIntLine(pinNo, trigger);
}

void extiEnableIntLine(uint32_t line, ExtiTrigger trigger)
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
    EXTI->PR |= (1UL << line);

    /* Enable hardware interrupt */
    EXTI->IMR |= (1UL << line);
}

void extiDisableIntGpio(const struct Gpio *__restrict gpio)
{
    const uint8_t pinNo = gpio->gpio & GPIO_PIN_MASK;

    extiDisableIntLine(pinNo);
}

void extiDisableIntLine(uint32_t line)
{
    EXTI->IMR &= ~(1UL << line);
}

void extiClearPendingGpio(const struct Gpio *__restrict gpio)
{
    const uint8_t pinNo = gpio->gpio & GPIO_PIN_MASK;

    EXTI->PR |= (1UL << pinNo);
}

void extiClearPendingLine(uint32_t line)
{
    EXTI->PR |= (1UL << line);
}
