#ifndef __PLAT_SPI_H
#define __PLAT_SPI_H

#include <gpio.h>
#include <plat/inc/cmsis.h>
#include <plat/inc/gpio.h>

struct StmSpiPinCfg {
    GpioNum gpioMiso;
    GpioNum gpioMosi;
    GpioNum gpioSclk;
    GpioNum gpioNss;

    enum GpioAltFunc gpioFunc;
    enum GpioPullMode gpioPull;

    IRQn_Type irqNss;
};

extern const struct StmSpiPinCfg *boardStmSpiPinCfg(uint8_t busId);

#endif /* __PLAT_SPI_H */
