#ifndef _I2C_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

typedef uint8_t I2cAddr;
typedef uint32_t I2cSpeed;

typedef void (*I2cCallbackF)(void *cookie, int tx, int rx);

int i2cMasterRequest(uint8_t busId, I2cSpeed speed);
int i2cMasterRelease(uint8_t busId);
int i2cMasterTxRx(uint8_t busId, uint8_t addr,
        const void *txBuf, size_t tx_size,
        void *rxBuf, size_t rx_len, I2cCallbackF callback, void *cookie);

int i2cSlaveRequest(uint8_t busId, I2cAddr addr);
int i2cSlaveRelease(uint8_t busId);
void i2cSlaveEnableRx(uint8_t busId, void *rxBuf, size_t size,
        I2cCallbackF callback, void *cookie);
void i2cSlaveDisable(uint8_t busId);
int i2cSlaveTx(uint8_t busId, const void *buf, size_t size,
        I2cCallbackF callback, void *cookie);

#ifdef __cplusplus
}
#endif

#endif

