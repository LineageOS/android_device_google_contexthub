#ifndef _VARIANT_LUNCHBOX_H_
#define _VARIANT_LUNCHBOX_H_

#ifdef __cplusplus
extern "C" {
#endif

//we have LSE in lunchbox
#define RTC_CLK                     RTC_CLK_LSE
// fCAL = fRTCCLK x [1 + (256 - CALM) / (2^20 + CALM - 256)]
// 32764.505 * (1 + (256 - 144)/(1048576 + 144 - 256)) = 32768.005
#define RTC_PREDIV_A                0UL
#define RTC_PREDIV_S                32759UL
#define RTC_CALM                    144UL
#define RTC_CALP                    0UL

//spi bus for comms
#define PLATFORM_HOST_INTF_SPI_BUS  0

#define SH_INT_WAKEUP               GPIO_PA(2)
#define SH_EXTI_WAKEUP_IRQ          EXTI2_IRQn
#define AP_INT_WAKEUP               GPIO_PA(3)
#undef AP_INT_NONWAKEUP

#define DEBUG_LOG_EVT               0x3B474F4C

#define BMI160_TO_ANDROID_COORDINATE(x, y, z)   \
    do {                                        \
        int32_t xi = x, yi = y, zi = z;         \
        x = -yi; y = xi; z = zi;                \
    } while (0)

#define BMM150_TO_ANDROID_COORDINATE(x, y, z)   \
    do {                                        \
        int32_t xi = x, yi = y, zi = z;         \
        x = xi; y = -yi; z = -zi;               \
    } while (0)

#define HALL_PIN GPIO_PA(9)
#define HALL_IRQ EXTI9_5_IRQn

#define VSYNC_PIN GPIO_PB(1)
#define VSYNC_IRQ EXTI1_IRQn

//define tap sensor threshould
#define TAP_THRESHOULD  0x01

//define Accelerometer fast compensation config
#define ACC_FOC_CONFIG 0x3d

#ifdef __cplusplus
}
#endif

#endif
