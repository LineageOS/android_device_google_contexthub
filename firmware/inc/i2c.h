#ifndef _I2C_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

typedef uint8_t I2cBus;
typedef uint8_t I2cAddr;
typedef uint32_t I2cSpeed;

typedef void (*I2cCallbackF)(void *cookie, size_t tx, size_t rx, int err);

int i2cMasterRequest(I2cBus busId, I2cSpeed speed);
int i2cMasterRelease(I2cBus busId);
int i2cMasterTxRx(I2cBus busId, I2cAddr addr, const void *txBuf, size_t txSize,
        void *rxBuf, size_t rxSize, I2cCallbackF callback, void *cookie);
static inline int i2cMasterTx(I2cBus busId, I2cAddr addr,
        const void *txBuf, size_t txSize, I2cCallbackF callback, void *cookie)
{
    return i2cMasterTxRx(busId, addr, txBuf, txSize, NULL, 0, callback, cookie);}
static inline int i2cMasterRx(I2cBus busId, I2cAddr addr,
        void *rxBuf, size_t rxSize, I2cCallbackF callback, void *cookie)
{
    return i2cMasterTxRx(busId, addr, NULL, 0, rxBuf, rxSize, callback, cookie);
}

int i2cSlaveRequest(I2cBus busId, I2cAddr addr);
int i2cSlaveRelease(I2cBus busId);
void i2cSlaveEnableRx(I2cBus busId, void *rxBuf, size_t rxSize,
        I2cCallbackF callback, void *cookie);
int i2cSlaveTx(I2cBus busId, const void *txBuf, size_t txSize,
        I2cCallbackF callback, void *cookie);

#ifdef __cplusplus
}
#endif

#endif
