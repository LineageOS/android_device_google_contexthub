#ifndef _VARIANT_LNX_H_
#define _VARIANT_LNX_H_

#ifdef __cplusplus
extern "C" {
#endif

//we have no LSE in linux
#define HAVE_LSE    false

//i2c bus for comms (dummy)
#define PLATFORM_HOST_INTF_I2C_BUS  12345




#ifdef __cplusplus
}
#endif

#endif

