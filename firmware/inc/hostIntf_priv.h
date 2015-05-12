#ifndef __HOSTINTF_PRIV_H
#define __HOSTINTF_PRIV_H

#include <i2c.h>
#include <stdint.h>
#include <stddef.h>


/**
 * hostIntf communication abstraction layer
 */

typedef void (*HostIntfCommCallbackF)(size_t bytesTransferred, int err);
struct HostIntfComm {
    int (*request)(void);

    int (*rxPacket)(void *rxBuf, size_t rxSize, HostIntfCommCallbackF callback);
    int (*txPacket)(const void *txBuf, size_t txSize,
            HostIntfCommCallbackF callback);

    int (*release)(void);
};

/**
 * Returns a HostIntfOps backed by I2C
 */
const struct HostIntfComm *hostIntfI2cInit(I2cBus busId);

/**
 * Returns a HostIntfOps backed by SPI
 */
const struct HostIntfComm *hostIntfSpiInit(uint8_t busId);


/**
 * Platform-internal hostIntf API
 */

/**
 * Returns the platform's communication implementation.  The platform should
 * delegate this to hostIntfI2cInit() or hostIntfSpiInit() as appropriate.
 */
const struct HostIntfComm *platHostIntfInit();

/**
 * Returns the platform's hardware type (16-bit, host byte order)
 */
uint16_t platHwType(void);

/**
 * Returns the platform's hardware version (16-bit, host byte order)
 */
uint16_t platHwVer(void);

/**
 * Returns the platform's bootloader version (16-bit, host byte order)
 */
uint16_t platBlVer(void);

#endif /* __HOSTINTF_PRIV_H */
