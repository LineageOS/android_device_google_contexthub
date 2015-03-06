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

void extiEnableInt(const struct gpio *__restrict gpio, ExtiTrigger trigger)
{
    struct StmExti *block = (struct StmExti *)EXTI_BASE;
    const uint8_t pinNo = gpio->gpio & GPIO_PIN_MASK;

    if (trigger == EXTI_TRIGGER_BOTH) {
        block->RTSR |= (1UL << pinNo);
        block->FTSR |= (1UL << pinNo);
    } else if (trigger == EXTI_TRIGGER_RISING) {
        block->RTSR |= (1UL << pinNo);
        block->FTSR &= ~(1UL << pinNo);
    } else if (trigger == EXTI_TRIGGER_FALLING) {
        block->RTSR &= ~(1UL << pinNo);
        block->FTSR |= (1UL << pinNo);
    }

    extiClearPending(gpio);

    block->IMR |= (1UL << pinNo);
}

void extiDisableInt(const struct gpio *__restrict gpio)
{
    struct StmExti *block = (struct StmExti *)(EXTI_BASE);
    const uint8_t pinNo = gpio->gpio & GPIO_PIN_MASK;

    block->IMR &= ~(1UL << pinNo);
}

void extiClearPending(const struct gpio *__restrict gpio)
{
    struct StmExti *block = (struct StmExti *)(EXTI_BASE);
    const uint8_t pinNo = gpio->gpio & GPIO_PIN_MASK;

    block->PR |= (1UL << pinNo);
}
