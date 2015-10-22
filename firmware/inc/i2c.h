#ifndef _I2C_H_
#define _I2C_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

typedef void (*I2cCallbackF)(void *cookie, size_t tx, size_t rx, int err);

int i2cMasterRequest(uint32_t busId, uint32_t speedInHz);
int i2cMasterRelease(uint32_t busId);
int i2cMasterTxRx(uint32_t busId, uint32_t addr, const void *txBuf, size_t txSize,
        void *rxBuf, size_t rxSize, I2cCallbackF callback, void *cookie);
static inline int i2cMasterTx(uint32_t busId, uint32_t addr,
        const void *txBuf, size_t txSize, I2cCallbackF callback, void *cookie)
{
    return i2cMasterTxRx(busId, addr, txBuf, txSize, NULL, 0, callback, cookie);}
static inline int i2cMasterRx(uint32_t busId, uint32_t addr,
        void *rxBuf, size_t rxSize, I2cCallbackF callback, void *cookie)
{
    return i2cMasterTxRx(busId, addr, NULL, 0, rxBuf, rxSize, callback, cookie);
}

int i2cSlaveRequest(uint32_t busId, uint32_t addr);
int i2cSlaveRelease(uint32_t busId);

void i2cSlaveEnableRx(uint32_t busId, void *rxBuf, size_t rxSize,
        I2cCallbackF callback, void *cookie);
int i2cSlaveTxPreamble(uint32_t busId, uint8_t byte,
        I2cCallbackF callback, void *cookie);
int i2cSlaveTxPacket(uint32_t busId, const void *txBuf, size_t txSize,
        I2cCallbackF callback, void *cookie);

#ifdef __cplusplus
}
#endif

#endif /* _I2C_H_ */
