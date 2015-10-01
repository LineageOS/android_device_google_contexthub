#include <plat/inc/spi.h>
#include <util.h>

static const struct StmSpiBoardCfg mStmSpiBoardCfgs[] = {
    [0] = {
        .gpioMiso = GPIO_PA(6),
        .gpioMosi = GPIO_PA(7),
        .gpioSclk = GPIO_PA(5),
        .gpioNss = GPIO_PA(4),

        .gpioFunc = GPIO_AF_SPI1,
        .gpioSpeed = GPIO_SPEED_MEDIUM,

        .irqNss = EXTI4_IRQn,

        .dmaRx = SPI1_DMA_RX_CFG_B,
        .dmaTx = SPI1_DMA_TX_CFG_B,

        .sleepDev = -1,
    },
    [1] = {
        .gpioMiso = GPIO_PB(14),
        .gpioMosi = GPIO_PB(15),
        .gpioSclk = GPIO_PB(13),
        .gpioNss = GPIO_PB(12),

        .gpioSpeed = GPIO_SPEED_MEDIUM,
        .gpioFunc = GPIO_AF_SPI2_A,

        .irqNss = EXTI15_10_IRQn,

        .dmaRx = SPI2_DMA_RX_CFG,
        .dmaTx = SPI2_DMA_TX_CFG,

        .sleepDev = Stm32sleepDevSpi2,
    },
};

const struct StmSpiBoardCfg *boardStmSpiCfg(uint8_t busId)
{
    if (busId >= ARRAY_SIZE(mStmSpiBoardCfgs))
        return NULL;

    return &mStmSpiBoardCfgs[busId];
}
