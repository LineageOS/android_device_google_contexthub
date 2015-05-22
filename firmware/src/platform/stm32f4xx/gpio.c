#include <plat/inc/gpio.h>
#include <plat/inc/pwr.h>
#include <gpio.h>

struct StmGpio {
    volatile uint32_t MODER;
    volatile uint32_t OTYPER;
    volatile uint32_t OSPEEDR;
    volatile uint32_t PUPDR;
    volatile uint32_t IDR;
    volatile uint32_t ODR;
    volatile uint32_t BSRR;
    volatile uint32_t LCKR;
    volatile uint32_t AFR[2];
};

static const uint32_t mGpioPeriphs[] = {
    PERIPH_AHB1_GPIOA,
    PERIPH_AHB1_GPIOB,
    PERIPH_AHB1_GPIOC,
    PERIPH_AHB1_GPIOD,
    PERIPH_AHB1_GPIOE,
    PERIPH_AHB1_GPIOF,
    PERIPH_AHB1_GPIOG,
    PERIPH_AHB1_GPIOH,
    PERIPH_AHB1_GPIOI,
};

static const uint32_t mGpioBases[] = {
    GPIOA_BASE,
    GPIOB_BASE,
    GPIOC_BASE,
    GPIOD_BASE,
    GPIOE_BASE,
    GPIOF_BASE,
    GPIOG_BASE,
    GPIOH_BASE,
    GPIOI_BASE,
};

void gpioRequest(struct Gpio *gpio, GpioNum number)
{
    gpio->gpio = number;
}

static void gpioConfig(const struct Gpio* __restrict gpio, enum GpioPullMode pull, enum GpioOpenDrainMode output)
{
    struct StmGpio *block = (struct StmGpio*)mGpioBases[gpio->gpio >> GPIO_PORT_SHIFT];
    const uint32_t shift_1b = gpio->gpio & GPIO_PIN_MASK;
    const uint32_t shift_2b = (gpio->gpio & GPIO_PIN_MASK) * 2;
    const uint32_t mask_1b = (1UL << shift_1b);
    const uint32_t mask_2b = (3UL << shift_2b);

    /* unit clock */
    pwrUnitClock(PERIPH_BUS_AHB1, mGpioPeriphs[gpio->gpio >> GPIO_PORT_SHIFT], true);

    /* by default speed registers are configured for 2MHz */
    block->OSPEEDR &= ~mask_2b;

    /* pull ups/downs */
    block->PUPDR = (block->PUPDR& ~mask_2b) | (((uint32_t)pull) << shift_2b);
    /* push/pull or open drain */
    if (output == GPIO_OUT_PUSH_PULL)
        block->OTYPER &= ~mask_1b;
    else
        block->OTYPER |= mask_1b;
}

void gpioConfigInput(const struct Gpio* __restrict gpio, enum GpioPullMode pull)
{
    struct StmGpio *block = (struct StmGpio*)mGpioBases[gpio->gpio >> GPIO_PORT_SHIFT];
    const uint32_t shift_2b = (gpio->gpio & GPIO_PIN_MASK) * 2;
    const uint32_t mask_2b = (3UL << shift_2b);

    gpioConfig(gpio, pull, GPIO_OUT_PUSH_PULL);

    /* direction */
    block->MODER = (block->MODER & ~mask_2b) | (((uint32_t)GPIO_MODE_IN) << shift_2b);
}

void gpioConfigOutput(const struct Gpio* __restrict gpio, enum GpioPullMode pull, enum GpioOpenDrainMode output, bool value)
{
    struct StmGpio *block = (struct StmGpio*)mGpioBases[gpio->gpio >> GPIO_PORT_SHIFT];
    const uint32_t shift_2b = (gpio->gpio & GPIO_PIN_MASK) * 2;
    const uint32_t mask_2b = (3UL << shift_2b);

    gpioConfig(gpio, pull, output);

    /* set the initial output value */
    gpioSet(gpio, value);

    /* direction */
    block->MODER = (block->MODER & ~mask_2b) | (((uint32_t)GPIO_MODE_OUT) << shift_2b);
}

void gpioConfigAlt(const struct Gpio* __restrict gpio, enum GpioPullMode pull, enum GpioOpenDrainMode output, enum GpioAltFunc func)
{
    struct StmGpio *block = (struct StmGpio*)mGpioBases[gpio->gpio >> GPIO_PORT_SHIFT];
    const uint32_t pinNo = gpio->gpio & GPIO_PIN_MASK;
    const uint32_t regNo = pinNo >> (GPIO_PORT_SHIFT - 1);
    const uint32_t nibbleNo = pinNo & (GPIO_PIN_MASK >> 1);
    const uint32_t shift_2b = pinNo * 2;
    const uint32_t shift_4b = nibbleNo * 4;
    const uint32_t mask_2b = (3UL << shift_2b);
    const uint32_t mask_4b = (15UL << shift_4b);

    gpioConfig(gpio, pull, output);

    /* assign function */
    block->AFR[regNo] = (block->AFR[regNo] & ~mask_4b) | (((uint32_t)func) << shift_4b);

    /* direction */
    block->MODER = (block->MODER & ~mask_2b) | (((uint32_t)GPIO_MODE_ALTERNATE) << shift_2b);
}

void gpioSet(const struct Gpio* __restrict gpio, bool value)
{
    struct StmGpio *block = (struct StmGpio*)mGpioBases[gpio->gpio >> GPIO_PORT_SHIFT];
    const uint32_t shift_1b = gpio->gpio & GPIO_PIN_MASK;
    const uint32_t mask_set_1b = (1UL << (0  + shift_1b));
    const uint32_t mask_clr_1b = (1UL << (16 + shift_1b));

    block->BSRR = value ? mask_set_1b : mask_clr_1b;
}

bool gpioGet(const struct Gpio* __restrict gpio)
{
    struct StmGpio *block = (struct StmGpio*)mGpioBases[gpio->gpio >> GPIO_PORT_SHIFT];
    const uint32_t shift_1b = gpio->gpio & GPIO_PIN_MASK;
    const uint32_t mask_1b = (1UL << shift_1b);

    return !!(block->IDR & mask_1b);
}
