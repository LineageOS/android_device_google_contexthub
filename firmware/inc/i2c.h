#ifndef _I2C_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

typedef uint8_t i2c_addr_t;
typedef uint32_t i2c_speed_t;

#define I2C_BS_ADDR_WAS_NAKED     -1
#define I2C_BS_ARBITRATION_FAILED -2

/* MASTER API (addr is 7-bit, right justified).
 * Returns false on immediate failure, true if callback will be called.
 * Callback may be called before return.
 * Buffer must reamin valid until callback is called.
 * Callback WILL be called.
 */
typedef void (*I2cMasterCbk)(void* userData, int32_t txBytesSent, int32_t rxBytesSent); //I2C_BS_* is also possible to get for "*BytesSent"
int8_t i2cTransRxTx(uint32_t bus, uint8_t addr, const void *txPtr, uint32_t txLen, void *rxPtr, uint32_t rxLen, I2cMasterCbk cbk, void *userData);

static inline int8_t i2cTransTx(uint32_t bus, uint8_t addr, const void *txPtr, uint32_t txLen, I2cMasterCbk cbk, void *userData)
{
    return i2cTransRxTx(bus, addr, txPtr, txLen, NULL, 0, cbk, userData);
}

typedef void (*i2c_callback_t)(void *cookie, int err);

int OS_I2C_slave_request(uint8_t bus_id, i2c_speed_t speed, i2c_addr_t addr);
void OS_I2C_slave_enable_rx(uint8_t bus_id, void *rx_buf, size_t size,
        i2c_callback_t callback, void *cookie);
void OS_I2C_slave_disable(uint8_t bus_id);
int OS_I2C_slave_send(uint8_t bus_id, const void *buf, size_t size,
        i2c_callback_t callback, void *cookie);

#ifdef __cplusplus
}
#endif

#endif

