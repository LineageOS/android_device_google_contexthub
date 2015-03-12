#include <errno.h>
#include <heap.h>
#include <string.h>

#include <spi.h>
#include <spi_priv.h>

struct SpiMasterState {
    struct SpiDevice dev;

    void **rxBuf;
    const void **txBuf;
    size_t *size;
    size_t n;
    size_t currentBuf;
    struct SpiMode mode;
    SpiMasterCbkF callback;
    void *cookie;
};
#define SPI_MASTER_TO_STATE(p) ((struct SpiMasterState *)p)

static void spiMasterNext(struct SpiMasterState *state);
static void spiMasterStop(struct SpiMasterState *dev);
static void spiMasterDone(struct SpiMasterState *state, int err);

static int spiMasterStart(struct SpiMasterState *state,
        spi_cs_t cs, const struct SpiMode *mode)
{
    struct SpiDevice *dev = &state->dev;

    if (dev->ops->masterStartAsync)
        return dev->ops->masterStartAsync(dev, cs, mode);

    if (dev->ops->masterStartSync) {
        int err = dev->ops->masterStartSync(dev, cs, mode);
        if (err < 0)
            return err;
    }

    return dev->ops->masterRxTx(dev, state->rxBuf[0], state->txBuf[0],
            state->size[0], mode);
}

void spi_masterStartAsync_done(struct SpiDevice *dev, int err)
{
    struct SpiMasterState *state = SPI_MASTER_TO_STATE(dev);
    if (err)
        spiMasterDone(state, err);
    else
        spiMasterNext(state);
}

static void spiMasterNext(struct SpiMasterState *state)
{
    struct SpiDevice *dev = &state->dev;

    if (state->currentBuf == state->n) {
        spiMasterStop(state);
        return;
    }

    size_t i = state->currentBuf;
    void *rxBuf = state->rxBuf[i];
    const void *txBuf = state->txBuf[i];
    size_t size = state->size[i];
    const struct SpiMode *mode = &state->mode;

    int err = dev->ops->masterRxTx(dev, rxBuf, txBuf, size, mode);
    if (err)
        spiMasterDone(state, err);
}

void spiMasterRxTxDone(struct SpiDevice *dev, int err)
{
    struct SpiMasterState *state = SPI_MASTER_TO_STATE(dev);
    if (err) {
        spiMasterDone(state, err);
    } else {
        state->currentBuf++;
        spiMasterNext(state);
    }
}

static void spiMasterStop(struct SpiMasterState *state)
{
    struct SpiDevice *dev = &state->dev;

    if (dev->ops->masterStopSync) {
        int err = dev->ops->masterStopSync(dev);
        spiMasterDone(state, err);
    } else if (dev->ops->masterStopAsync) {
        int err = dev->ops->masterStopAsync(dev);
        if (err < 0)
            spiMasterDone(state, err);
    } else {
        spiMasterDone(state, 0);
    }
}

void spiMasterStopAsyncDone(struct SpiDevice *dev, int err)
{
    struct SpiMasterState *state = SPI_MASTER_TO_STATE(dev);
    spiMasterDone(state, err);
}

static void spiMasterDone(struct SpiMasterState *state, int err)
{
    struct SpiDevice *dev = &state->dev;
    SpiMasterCbkF callback = state->callback;
    void *cookie = state->cookie;

    heapFree(state->rxBuf);
    heapFree(state->txBuf);
    heapFree(state->size);
    if (dev->ops->release)
        dev->ops->release(dev);
    heapFree(state);

    callback(cookie, err);
}

int spiMasterRxTx(uint8_t busId, spi_cs_t cs,
        void *rxBuf[], const void *txBuf[], size_t size[], size_t n,
        const struct SpiMode *mode, SpiMasterCbkF callback,
        void *cookie)
{
    int ret = 0;

    if (!n)
        return -EINVAL;

    struct SpiMasterState *state = heapAlloc(sizeof(*state));
    if (!state)
        return -ENOMEM;
    struct SpiDevice *dev = &state->dev;

    ret = spiRequest(dev, busId);
    if (ret < 0)
        goto err_request;

    if (!dev->ops->masterRxTx) {
        ret = -EOPNOTSUPP;
        goto err_opsupp;
    }

    state->rxBuf = heapAlloc(n * sizeof(*rxBuf));
    state->txBuf = heapAlloc(n * sizeof(*txBuf));
    state->size = heapAlloc(n * sizeof(*size));

    if (!state->rxBuf || !state->txBuf || !state->size) {
        ret = -ENOMEM;
        goto err_alloc;
    }

    memcpy(state->rxBuf, rxBuf, n * sizeof(*rxBuf));
    memcpy(state->txBuf, txBuf, n * sizeof(*txBuf));
    memcpy(state->size, size, n * sizeof(*size));
    state->n = n;
    state->currentBuf = 0;
    state->mode = *mode;
    state->callback = callback;
    state->cookie = cookie;

    ret = spiMasterStart(state, cs, mode);
    if (ret < 0)
        return 0;

err_alloc:
    heapFree(state->rxBuf);
    heapFree(state->txBuf);
    heapFree(state->size);
err_opsupp:
    if (dev->ops->release)
        dev->ops->release(dev);
err_request:
    heapFree(state);
    return ret;
}
