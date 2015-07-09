#include <plat/inc/spi.h>
#include <util.h>

static const struct StmSpiPinCfg mStmSpiPinCfgs[] = {
    [0] = {
        .gpioMiso = GPIO_PA(6),
        .gpioMosi = GPIO_PA(7),
        .gpioSclk = GPIO_PA(5),
        .gpioNss = GPIO_PA(4),
        .gpioFunc = GPIO_AF_SPI1,
        .irqNss = EXTI4_IRQn,
    },
    [1] = {
        .gpioMiso = GPIO_PB(14),
        .gpioMosi = GPIO_PB(15),
        .gpioSclk = GPIO_PB(13),
        .gpioNss = GPIO_PB(12),
        .gpioFunc = GPIO_AF_SPI2_A,
        .irqNss = EXTI15_10_IRQn,
    },
};

const struct StmSpiPinCfg *boardStmSpiPinCfg(uint8_t busId)
{
    if (busId >= ARRAY_SIZE(mStmSpiPinCfgs))
        return NULL;

    return &mStmSpiPinCfgs[busId];
}
