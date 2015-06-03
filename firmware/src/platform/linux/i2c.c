#include <errno.h>
#include <stdint.h>
#include <gpio.h>
#include <i2c.h>
#include <seos.h>
#include <util.h>
#include <atomicBitset.h>
#include <atomic.h>




int i2cMasterRequest(I2cBus busId, I2cSpeed speed)
{
    return -EINVAL;
}

int i2cMasterRelease(I2cBus busId)
{
    return -EINVAL;
}

int i2cMasterTxRx(I2cBus busId, I2cAddr addr,
        const void *txBuf, size_t txSize, void *rxBuf, size_t rxSize,
        I2cCallbackF callback, void *cookie)
{
    return -EINVAL;
}

int i2cSlaveRequest(I2cBus busId, I2cAddr addr)
{
    return -EINVAL;
}

int i2cSlaveRelease(I2cBus busId)
{
    return -EINVAL;
}

void i2cSlaveEnableRx(I2cBus busId, void *rxBuf, size_t rxSize,
        I2cCallbackF callback, void *cookie)
{
    //
}

int i2cSlaveTxPreamble(I2cBus busId, uint8_t byte, I2cCallbackF callback, void *cookie)
{
    return -EBUSY;
}

int i2cSlaveTxPacket(I2cBus busId, const void *txBuf, size_t txSize, I2cCallbackF callback, void *cookie)
{
    return -EBUSY;
}
