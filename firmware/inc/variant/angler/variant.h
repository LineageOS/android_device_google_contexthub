#ifndef _VARIANT_ANGLER_H_
#define _VARIANT_ANGLER_H_

#ifdef __cplusplus
extern "C" {
#endif

//we have LSE in angler
#define RTC_CLK                     RTC_CLK_LSE

//spi bus for comms
#define PLATFORM_HOST_INTF_SPI_BUS  0

#define AP_INT_WAKEUP               GPIO_PA(3)
#undef AP_INT_NONWAKEUP

#define DEBUG_LOG_EVT               0x3B474F4C

#ifdef __cplusplus
}
#endif

#endif
