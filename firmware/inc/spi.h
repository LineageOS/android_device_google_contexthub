#ifndef __SPI_H
#define __SPI_H

#include <inttypes.h>
#include <seos.h>
#include <stdlib.h>

struct SpiDevice;

typedef uint8_t spi_cs_t;
typedef uint32_t SpiSpeed;

typedef void (*SpiCbkF)(void *cookie, int err);

struct SpiMode {
    enum {
        SPI_CPOL_IDLE_LO,
        SPI_CPOL_IDLE_HI,
    } cpol;

    enum {
        SPI_CPHA_LEADING_EDGE,
        SPI_CPHA_TRAILING_EDGE,
    } cpha;

    uint8_t bitsPerWord;
    enum {
        SPI_FORMAT_LSB_FIRST,
        SPI_FORMAT_MSB_FIRST,
    } format;

    uint16_t txWord;

    SpiSpeed speed;
};

int spiMasterRxTx(uint8_t busId, spi_cs_t cs,
        void *rxBuf[], const void *txBuf[], size_t size[], size_t n,
        const struct SpiMode *mode, SpiCbkF callback,
        void *cookie);

static inline int spiMasterRx(uint8_t busId, spi_cs_t cs,
        void *buf, size_t size, const struct SpiMode *mode,
        SpiCbkF callback, void *cookie)
{
    void *rxBuf[1] = {buf};
    const void *txBuf[1] = {NULL};
    size_t sizes[1] = {size};
    return spiMasterRxTx(busId, cs, rxBuf, txBuf, sizes, 1, mode,
            callback, cookie);
}

static inline int spiMasterTx(uint8_t busId, spi_cs_t cs,
        const void *buf, size_t size, const struct SpiMode *mode,
        SpiCbkF callback, void *cookie)
{
    void *rxBuf[1] = {NULL};
    const void *txBuf[1] = {buf};
    size_t sizes[1] = {size};
    return spiMasterRxTx(busId, cs, rxBuf, txBuf, sizes, 1, mode,
            callback, cookie);
}

int spiSlaveRequest(uint8_t busId, const struct SpiMode *mode,
        struct SpiDevice **dev);

int spiSlaveRxTx(struct SpiDevice *dev,
        void *rxBuf[], const void *txBuf[], size_t size[], size_t n,
        SpiCbkF callback, void *cookie);

int spiSlaveRelease(struct SpiDevice *dev);

static inline int spiSlaveRx(struct SpiDevice *dev,
        void *buf, size_t size,
        SpiCbkF callback, void *cookie)
{
    void *rxBuf[1] = {buf};
    const void *txBuf[1] = {NULL};
    size_t sizes[1] = {size};
    return spiSlaveRxTx(dev, rxBuf, txBuf, sizes, 1, callback, cookie);
}

static inline int spiSlaveTx(struct SpiDevice *dev,
        const void *buf, size_t size,
        SpiCbkF callback, void *cookie)
{
    void *rxBuf[1] = {NULL};
    const void *txBuf[1] = {buf};
    size_t sizes[1] = {size};
    return spiSlaveRxTx(dev, rxBuf, txBuf, sizes, 1, callback, cookie);
}

#endif /* __SPI_H */
