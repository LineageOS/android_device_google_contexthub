#include <hostIntf.h>
#include <hostIntf_priv.h>
#include <nanohubPacket.h>
#include <spi.h>

static uint8_t gBusId;
static struct SpiDevice *gSpi;

static void *gRxBuf;
static size_t gRxSize;
static size_t gTxSize;

static const struct SpiMode gSpiMode = {
    .cpol = SPI_CPOL_IDLE_LO,
    .cpha = SPI_CPHA_LEADING_EDGE,
    .bitsPerWord = 8,
    .format = SPI_FORMAT_MSB_FIRST,
    .txWord = NANOHUB_PREAMBLE_BYTE,
};

static void hostIntfSpiRxCallback(void *cookie, int err)
{
    struct NanohubPacket *packet = gRxBuf;
    HostIntfCommCallbackF callback = cookie;
    callback(NANOHUB_PACKET_SIZE(packet->len), err);
}

static void hostIntfSpiTxCallback(void *cookie, int err)
{
    HostIntfCommCallbackF callback = cookie;
    callback(gTxSize, err);
}

static void hostIntfSpiInactiveCallback(void *cookie, int err)
{
    HostIntfCommCallbackF callback = cookie;

    if (err < 0) {
        callback(0, err);
        return;
    }

    err = spiSlaveRx(gSpi, gRxBuf, gRxSize, hostIntfSpiRxCallback, callback);
    if (err < 0)
        callback(0, err);
}

static int hostIntfSpiRequest()
{
    return spiSlaveRequest(gBusId, &gSpiMode, &gSpi);
}

static int hostIntfSpiRxPacket(void *rxBuf, size_t rxSize,
        HostIntfCommCallbackF callback)
{
    gRxBuf = rxBuf;
    gRxSize = rxSize;
    return spiSlaveWaitForInactive(gSpi, hostIntfSpiInactiveCallback, callback);
}

static int hostIntfSpiTxPacket(const void *txBuf, size_t txSize,
        HostIntfCommCallbackF callback)
{
    ((uint8_t *)txBuf)[txSize] = NANOHUB_PREAMBLE_BYTE;
    gTxSize = txSize;
    return spiSlaveTx(gSpi, txBuf, txSize + 1, hostIntfSpiTxCallback,
            callback);
}

static int hostIntfSpiRelease(void)
{
    return spiSlaveRelease(gSpi);
}

static const struct HostIntfComm gSpiComm = {
   .request = hostIntfSpiRequest,
   .rxPacket = hostIntfSpiRxPacket,
   .txPacket = hostIntfSpiTxPacket,
   .release = hostIntfSpiRelease,
};

const struct HostIntfComm *hostIntfSpiInit(uint8_t busId)
{
    gBusId = busId;
    return &gSpiComm;
}
