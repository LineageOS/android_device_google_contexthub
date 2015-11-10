#ifndef _VARIANT_NUCLEO_H_
#define _VARIANT_NUCLEO_H_

#ifdef __cplusplus
extern "C" {
#endif

//we have LSE in nucleo
#define RTC_CLK                     RTC_CLK_LSE

//spi bus for comms
#define PLATFORM_HOST_INTF_SPI_BUS  0

#define SH_INT_WAKEUP               GPIO_PA(0)
#define SH_EXTI_WAKEUP_IRQ          EXTI0_IRQn
#define AP_INT_WAKEUP               GPIO_PA(1)
#undef AP_INT_NONWAKEUP

#define DEBUG_UART_UNITNO           2
#define DEBUG_UART_GPIO_TX          GPIO_PA(2)
#define DEBUG_UART_GPIO_RX          GPIO_PA(3)

#define BMI160_TO_ANDROID_COORDINATE(x, y, z)   \
    do {                                        \
        int32_t xi = x, yi = y, zi = z;         \
        x = xi; y = yi; z = zi;                 \
    } while (0)

#define BMM150_TO_ANDROID_COORDINATE(x, y, z)   \
    do {                                        \
        int32_t xi = x, yi = y, zi = z;         \
        x = xi; y = -yi; z = -zi;               \
    } while (0)

#define HALL_PIN GPIO_PB(5)
#define HALL_IRQ EXTI9_5_IRQn

#define VSYNC_PIN GPIO_PB(1)
#define VSYNC_IRQ EXTI1_IRQn

//define tap sensor threshould
#define TAP_THRESHOULD 0x01

#ifdef __cplusplus
}
#endif

#endif
