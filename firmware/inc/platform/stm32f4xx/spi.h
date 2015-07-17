#ifndef __PLAT_SPI_H
#define __PLAT_SPI_H

#include <gpio.h>
#include <plat/inc/cmsis.h>
#include <plat/inc/gpio.h>

struct StmSpiDmaCfg {
    uint8_t channel;
    uint8_t stream;
};

struct StmSpiBoardCfg {
    GpioNum gpioMiso;
    GpioNum gpioMosi;
    GpioNum gpioSclk;
    GpioNum gpioNss;

    enum GpioAltFunc gpioFunc;
    enum GpioSpeed gpioSpeed;
    enum GpioPullMode gpioPull;

    IRQn_Type irqNss;

    struct StmSpiDmaCfg dmaRx;
    struct StmSpiDmaCfg dmaTx;
};

#define SPI1_DMA_BUS        1
#define SPI1_DMA_RX_CFG_A   { .channel = 3, .stream = 0 }
#define SPI1_DMA_RX_CFG_B   { .channel = 3, .stream = 2 }
#define SPI1_DMA_TX_CFG_A   { .channel = 3, .stream = 3 }
#define SPI1_DMA_TX_CFG_B   { .channel = 3, .stream = 5 }

#define SPI2_DMA_BUS        0
#define SPI2_DMA_RX_CFG     { .channel = 0, .stream = 3 }
#define SPI2_DMA_TX_CFG     { .channel = 0, .stream = 4 }

#define SPI3_DMA_BUS        0
#define SPI3_DMA_RX_CFG_A   { .channel = 0, .stream = 2 }
#define SPI3_DMA_RX_CFG_B   { .channel = 0, .stream = 3 }
#define SPI3_DMA_TX_CFG_A   { .channel = 0, .stream = 5 }
#define SPI3_DMA_TX_CFG_B   { .channel = 0, .stream = 7 }

#define SPI4_DMA_BUS        1
#define SPI4_DMA_RX_CFG_A   { .channel = 4, .stream = 0 }
#define SPI4_DMA_RX_CFG_B   { .channel = 5, .stream = 3 }
#define SPI4_DMA_TX_CFG_A   { .channel = 4, .stream = 1 }
#define SPI4_DMA_TX_CFG_B   { .channel = 5, .stream = 4 }

#define SPI5_DMA_BUS        1
#define SPI5_DMA_RX_CFG_A   { .channel = 2, .stream = 3 }
#define SPI5_DMA_RX_CFG_B   { .channel = 7, .stream = 5 }
#define SPI5_DMA_TX_CFG_A   { .channel = 2, .stream = 4 }
#define SPI5_DMA_TX_CFG_B   { .channel = 7, .stream = 6 }

#define SPI6_DMA_BUS        1
#define SPI6_DMA_RX_CFG     { .channel = 1, .stream = 6 }
#define SPI6_DMA_TX_CFG     { .channel = 1, .stream = 5 }

extern const struct StmSpiBoardCfg *boardStmSpiCfg(uint8_t busId);

#endif /* __PLAT_SPI_H */
