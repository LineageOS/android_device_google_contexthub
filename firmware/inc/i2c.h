#ifndef _I2C_H_
#ifndef _I2C_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>


#define I2C_BS_ADDR_WAS_NAKED     -1
#define I2C_BS_ARBITRATION_FAILED -2

/* MASTER API (addr is 7-bit, right justified).
 * Returns false on immediate failure, true if callback will be called.
 * Callback may be called before return.
 * Buffer must reamin valid until callback is called.
 * Callback WILL be called.
 */
void (*I2cMasterCbk)(void* userData, int32_t txBytesSent, int32_t rxBytesSent); //I2C_BS_* is also possible to get for "*BytesSent"
int8_t i2cTransRxTx(uint32_t bus, uint8_t addr, const void *txPtr, uint32_t txLen, void *rxPtr, uint32_t rxLen, I2cMasterCbk cbk, void *userData);

static inline int8_t i2cTransTx(uint32_t bus, uint8_t addr, const void *txPtr, uint32_t txLen, I2cMasterCbk cbk, void *userData)
{
    return i2cTransRxTx(bus, addr, txPtr, txLen, NULL, 0, cbk, userData);
}



#ifdef __cplusplus
}
#endif

#endif

