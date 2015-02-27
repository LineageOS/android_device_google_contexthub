#include <errno.h>
#include <heap.h>
#include <string.h>

#include <spi.h>
#include <spi_priv.h>

struct spi_master_state {
    struct spi_device dev;

    void **rx_buf;
    const void **tx_buf;
    size_t *size;
    size_t n;
    size_t current_buf;
    struct spi_mode mode;
    spi_master_callback callback;
    void *cookie;
};
#define SPI_MASTER_TO_STATE(p) ((struct spi_master_state *)p)

static void spi_master_next(struct spi_master_state *state);
static void spi_master_stop(struct spi_master_state *dev);
static void spi_master_done(struct spi_master_state *state, int err);

static int spi_master_start(struct spi_master_state *state,
        spi_cs_t cs, const struct spi_mode *mode)
{
    struct spi_device *dev = &state->dev;

    if (dev->ops->master_start_async)
        return dev->ops->master_start_async(dev, cs, mode);

    if (dev->ops->master_start_sync) {
        int err = dev->ops->master_start_sync(dev, cs, mode);
        if (err < 0)
            return err;
    }

    return dev->ops->master_rxtx(dev, state->rx_buf[0], state->tx_buf[0],
            state->size[0], mode);
}

void spi_master_start_async_done(struct spi_device *dev, int err)
{
    struct spi_master_state *state = SPI_MASTER_TO_STATE(dev);
    if (err)
        spi_master_done(state, err);
    else
        spi_master_next(state);
}

static void spi_master_next(struct spi_master_state *state)
{
    struct spi_device *dev = &state->dev;

    if (state->current_buf == state->n) {
        spi_master_stop(state);
        return;
    }

    size_t i = state->current_buf;
    void *rx_buf = state->rx_buf[i];
    const void *tx_buf = state->tx_buf[i];
    size_t size = state->size[i];
    const struct spi_mode *mode = &state->mode;

    int err = dev->ops->master_rxtx(dev, rx_buf, tx_buf, size, mode);
    if (err)
        spi_master_done(state, err);
}

void spi_master_rxtx_done(struct spi_device *dev, int err)
{
    struct spi_master_state *state = SPI_MASTER_TO_STATE(dev);
    if (err) {
        spi_master_done(state, err);
    } else {
        state->current_buf++;
        spi_master_next(state);
    }
}

static void spi_master_stop(struct spi_master_state *state)
{
    struct spi_device *dev = &state->dev;

    if (dev->ops->master_stop_sync) {
        int err = dev->ops->master_stop_sync(dev);
        spi_master_done(state, err);
    } else if (dev->ops->master_stop_async) {
        int err = dev->ops->master_stop_async(dev);
        if (err < 0)
            spi_master_done(state, err);
    } else {
        spi_master_done(state, 0);
    }
}

void spi_master_stop_async_done(struct spi_device *dev, int err)
{
    struct spi_master_state *state = SPI_MASTER_TO_STATE(dev);
    spi_master_done(state, err);
}

static void spi_master_done(struct spi_master_state *state, int err)
{
    struct spi_device *dev = &state->dev;
    spi_master_callback callback = state->callback;
    void *cookie = state->cookie;

    heapFree(state->rx_buf);
    heapFree(state->tx_buf);
    heapFree(state->size);
    if (dev->ops->release)
        dev->ops->release(dev);
    heapFree(state);

    callback(cookie, err);
}

int spi_master_rxtx(uint8_t bus_id, spi_cs_t cs,
        void *rx_buf[], const void *tx_buf[], size_t size[], size_t n,
        const struct spi_mode *mode, spi_master_callback callback,
        void *cookie)
{
    int ret = 0;

    if (!n)
        return -EINVAL;

    struct spi_master_state *state = heapAlloc(sizeof(*state));
    if (!state)
        return -ENOMEM;
    struct spi_device *dev = &state->dev;

    ret = spi_request(dev, bus_id);
    if (ret < 0)
        goto err_request;

    if (!dev->ops->master_rxtx) {
        ret = -EOPNOTSUPP;
        goto err_opsupp;
    }

    state->rx_buf = heapAlloc(n * sizeof(*rx_buf));
    state->tx_buf = heapAlloc(n * sizeof(*tx_buf));
    state->size = heapAlloc(n * sizeof(*size));

    if (!state->rx_buf || !state->tx_buf || !state->size) {
        ret = -ENOMEM;
        goto err_alloc;
    }

    memcpy(state->rx_buf, rx_buf, n * sizeof(*rx_buf));
    memcpy(state->tx_buf, tx_buf, n * sizeof(*tx_buf));
    memcpy(state->size, size, n * sizeof(*size));
    state->n = n;
    state->current_buf = 0;
    state->mode = *mode;
    state->callback = callback;
    state->cookie = cookie;

    ret = spi_master_start(state, cs, mode);
    if (ret < 0)
        return 0;

err_alloc:
    heapFree(state->rx_buf);
    heapFree(state->tx_buf);
    heapFree(state->size);
err_opsupp:
    if (dev->ops->release)
        dev->ops->release(dev);
err_request:
    heapFree(state);
    return ret;
}
