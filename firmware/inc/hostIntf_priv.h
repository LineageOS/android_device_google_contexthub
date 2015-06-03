#ifndef __HOSTINTF_PRIV_H
#define __HOSTINTF_PRIV_H

#include <i2c.h>
#include <stdint.h>

/**
 * Platform-internal hostIntf API
 */

/**
 * Returns the I2C bus used by the host interface
 */
I2cBus platHostIntfI2cBus(void);

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
