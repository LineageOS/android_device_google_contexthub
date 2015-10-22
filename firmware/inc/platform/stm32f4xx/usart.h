#ifndef _STM32F4XX_USART_H_
#define _STM32F4XX_USART_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <gpio.h>

struct usart
{
    struct Gpio *tx;
    struct Gpio *rx;
    uint8_t unit;
};

#ifdef __cplusplus
}
#endif

#endif

