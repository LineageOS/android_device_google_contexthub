#ifndef __SPI_H
#define __SPI_H

#include <inttypes.h>
#include <seos.h>
#include <stdlib.h>

struct spi_device;

typedef uint8_t spi_cs_t;
typedef uint32_t spi_speed_t;

typedef void (*spi_master_callback)(void *cookie, int err);

struct spi_mode {
    enum {
        SPI_CPOL_IDLE_LO,
        SPI_CPOL_IDLE_HI,
    } cpol;

    enum {
        SPI_CPHA_LEADING_EDGE,
        SPI_CPHA_TRAILING_EDGE,
    } cpha;

    uint8_t bits_per_word;
    enum {
        SPI_FORMAT_LSB_FIRST,
        SPI_FORMAT_MSB_FIRST,
    } format;

    spi_speed_t speed;
};

int spi_master_rxtx(uint8_t bus_id, spi_cs_t cs,
        void *rx_buf[], const void *tx_buf[], size_t size[], size_t n,
        const struct spi_mode *mode, spi_master_callback callback,
        void *cookie);

static inline int spi_master_rx(uint8_t bus_id, spi_cs_t cs,
        void *buf, size_t size, const struct spi_mode *mode,
        spi_master_callback callback, void *cookie)
{
    void *rx_buf[1] = {buf};
    const void *tx_buf[1] = {NULL};
    size_t sizes[1] = {size};
    return spi_master_rxtx(bus_id, cs, rx_buf, tx_buf, sizes, 1, mode,
            callback, cookie);
}

static inline int spi_master_tx(uint8_t bus_id, spi_cs_t cs,
        const void *buf, size_t size, const struct spi_mode *mode,
        spi_master_callback callback, void *cookie)
{
    void *rx_buf[1] = {NULL};
    const void *tx_buf[1] = {buf};
    size_t sizes[1] = {size};
    return spi_master_rxtx(bus_id, cs, rx_buf, tx_buf, sizes, 1, mode,
            callback, cookie);
}

#endif /* __SPI_H */
