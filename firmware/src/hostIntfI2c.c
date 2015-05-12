#include <hostIntf.h>
#include <hostIntf_priv.h>
#include <nanohubPacket.h>
#include <i2c.h>

#define NANOHUB_I2C_SLAVE_ADDRESS     0x55
static I2cBus gBusId;

static void hostIntfI2cPreambleCallback(void *cookie, size_t tx, size_t rx, int err)
{
}

static void hostIntfI2cRxCallback(void *cookie, size_t tx, size_t rx, int err)
{
    HostIntfCommCallbackF callback = cookie;
    i2cSlaveTxPreamble(gBusId, NANOHUB_PREAMBLE_BYTE,
            hostIntfI2cPreambleCallback, NULL);
    callback(rx, err);
}

static void hostIntfI2cTxCallback(void *cookie, size_t tx, size_t rx, int err)
{
    HostIntfCommCallbackF callback = cookie;
    i2cSlaveTxPreamble(gBusId, NANOHUB_PREAMBLE_BYTE,
            hostIntfI2cPreambleCallback, NULL);
    callback(tx, err);
}

static int hostIntfI2cRequest()
{
    return i2cSlaveRequest(gBusId, NANOHUB_I2C_SLAVE_ADDRESS);
}

static int hostIntfI2cRxPacket(void *rxBuf, size_t rxSize,
        HostIntfCommCallbackF callback)
{
    i2cSlaveEnableRx(gBusId, rxBuf, rxSize, hostIntfI2cRxCallback,
            callback);
    return 0;
}

static int hostIntfI2cTxPacket(const void *txBuf, size_t txSize,
        HostIntfCommCallbackF callback)
{
    return i2cSlaveTxPacket(gBusId, txBuf, txSize, hostIntfI2cTxCallback,
            callback);
}

static int hostIntfI2cRelease(void)
{
    return i2cSlaveRelease(gBusId);
}

static const struct HostIntfComm gI2cComm = {
   .request = hostIntfI2cRequest,
   .rxPacket = hostIntfI2cRxPacket,
   .txPacket = hostIntfI2cTxPacket,
   .release = hostIntfI2cRelease,
};

const struct HostIntfComm *hostIntfI2cInit(I2cBus busId)
{
    gBusId = busId;
    return &gI2cComm;
}
