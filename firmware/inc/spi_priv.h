#ifndef __SPI_PRIV_H
#define __SPI_PRIV_H

#include <spi.h>
#include <seos.h>

struct SpiDevice {
    const struct SpiDevice_ops *ops;
    void *pdata;
};

struct SpiDevice_ops {
    int (*masterStartSync)(struct SpiDevice *dev, spi_cs_t cs,
            const struct SpiMode *mode);
    int (*masterStartAsync)(struct SpiDevice *dev, spi_cs_t cs,
            const struct SpiMode *mode);

    int (*masterRxTx)(struct SpiDevice *dev, void *rxBuf, const void *txBuf,
            size_t size, const struct SpiMode *mode);

    int (*masterStopSync)(struct SpiDevice *dev);
    int (*masterStopAsync)(struct SpiDevice *dev);

    int (*slaveStartSync)(struct SpiDevice *dev, const struct SpiMode *mode);
    int (*slaveStartAsync)(struct SpiDevice *dev, const struct SpiMode *mode);

    int (*slaveIdle)(struct SpiDevice *dev, const struct SpiMode *mode);
    int (*slaveRxTx)(struct SpiDevice *dev, void *rxBuf, const void *txBuf,
            size_t size, const struct SpiMode *mode);

    int (*slaveStopSync)(struct SpiDevice *dev);
    int (*slaveStopAsync)(struct SpiDevice *dev);

    int (*release)(struct SpiDevice *dev);
};

int spiRequest(struct SpiDevice *dev, uint8_t busId);

void spi_masterStartAsync_done(struct SpiDevice *dev, int err);
void spiMasterRxTxDone(struct SpiDevice *dev, int err);
void spiMasterStopAsyncDone(struct SpiDevice *dev, int err);

void spiSlaveStartAsyncDone(struct SpiDevice *dev, int err);
void spiSlaveRxTxDone(struct SpiDevice *dev, int err);
void spiSlaveStopAsyncDone(struct SpiDevice *dev, int err);

#endif /* __SPI_PRIV_H */
