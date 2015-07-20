#ifndef __PLAT_SPI_H
#define __PLAT_SPI_H

#include <gpio.h>
#include <plat/inc/cmsis.h>
#include <plat/inc/gpio.h>

struct StmSpiBoardCfg {
    GpioNum gpioMiso;
    GpioNum gpioMosi;
    GpioNum gpioSclk;
    GpioNum gpioNss;

    enum GpioAltFunc gpioFunc;
    enum GpioSpeed gpioSpeed;
    enum GpioPullMode gpioPull;

    IRQn_Type irqNss;
};

extern const struct StmSpiBoardCfg *boardStmSpiCfg(uint8_t busId);

#endif /* __PLAT_SPI_H */
