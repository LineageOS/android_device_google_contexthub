#ifndef __PLAT_I2C_H
#define __PLAT_I2C_H

#include <gpio.h>
#include <plat/inc/cmsis.h>
#include <plat/inc/gpio.h>

struct StmI2cGpioCfg {
    GpioNum num;
    enum GpioAltFunc func;
};

struct StmI2cBoardCfg {
    struct StmI2cGpioCfg gpioScl;
    struct StmI2cGpioCfg gpioSda;

    enum GpioSpeed gpioSpeed;
    enum GpioPullMode gpioPull;
};

#define I2C1_GPIO_SCL_PB6   { .num = GPIO_PB(6), .func = GPIO_AF_I2C1 }
#define I2C1_GPIO_SCL_PB8   { .num = GPIO_PB(8), .func = GPIO_AF_I2C1 }
#define I2C1_GPIO_SDA_PB7   { .num = GPIO_PB(7), .func = GPIO_AF_I2C1 }
#define I2C1_GPIO_SDA_PB9   { .num = GPIO_PB(9), .func = GPIO_AF_I2C1 }

#define I2C2_GPIO_SCL_PB10  { .num = GPIO_PB(10), .func = GPIO_AF_I2C2_A }
#define I2C2_GPIO_SDA_PB3   { .num = GPIO_PB(3), .func = GPIO_AF_I2C2_B }
#define I2C2_GPIO_SDA_PB9   { .num = GPIO_PB(9), .func = GPIO_AF_I2C2_B }
#define I2C2_GPIO_SDA_PB11  { .num = GPIO_PB(11), .func = GPIO_AF_I2C2_A }

#define I2C3_GPIO_SCL_PA8   { .num = GPIO_PA(8), .func = GPIO_AF_I2C3_A }
#define I2C3_GPIO_SDA_PB4   { .num = GPIO_PB(4), .func = GPIO_AF_I2C3_B }
#define I2C3_GPIO_SDA_PC9   { .num = GPIO_PC(9), .func = GPIO_AF_I2C3_A }

extern const struct StmI2cBoardCfg *boardStmI2cCfg(uint8_t busId);

#endif /* __PLAT_I2C_H */
