#ifndef __SPI_PRIV_H
#define __SPI_PRIV_H

#include <spi.h>
#include <seos.h>

struct spi_device {
    const struct spi_device_ops *ops;
    void *pdata;
};

struct spi_device_ops {
    int (*master_start_sync)(struct spi_device *dev, spi_cs_t cs,
            const struct spi_mode *mode);
    int (*master_start_async)(struct spi_device *dev, spi_cs_t cs,
            const struct spi_mode *mode);

    int (*master_rxtx)(struct spi_device *dev, void *rx_buf, const void *tx_buf,
            size_t size, const struct spi_mode *mode);

    int (*master_stop_sync)(struct spi_device *dev);
    int (*master_stop_async)(struct spi_device *dev);

    int (*release)(struct spi_device *dev);
};

int spi_request(struct spi_device *dev, uint8_t bus_id);

void spi_master_start_async_done(struct spi_device *dev, int err);
void spi_master_rxtx_done(struct spi_device *dev, int err);
void spi_master_stop_async_done(struct spi_device *dev, int err);

#endif /* __SPI_PRIV_H */
