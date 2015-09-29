#include <plat/inc/i2c.h>
#include <util.h>

static const struct StmI2cBoardCfg mStmI2cBoardCfgs[] = {
    [0] = {
        .gpioScl = I2C1_GPIO_SCL_PB8,
        .gpioSda = I2C1_GPIO_SDA_PB9,
        .gpioPull = GPIO_PULL_NONE,
    },
};

const struct StmI2cBoardCfg *boardStmI2cCfg(uint8_t busId)
{
    if (busId >= ARRAY_SIZE(mStmI2cBoardCfgs))
        return NULL;

    return &mStmI2cBoardCfgs[busId];
}
