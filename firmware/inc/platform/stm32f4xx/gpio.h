#ifndef _STM32F4XX_GPIO_H_
#define _STM32F4XX_GPIO_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define GPIO_PORTA 0
#define GPIO_PORTB 1
#define GPIO_PORTC 2
#define GPIO_PORTD 3
#define GPIO_PORTE 4
#define GPIO_PORTF 5
#define GPIO_PORTG 6
#define GPIO_PORTH 7
#define GPIO_PORTI 8

#define GPIO_PORT_SHIFT 4

/*
 * This is a shorthand to specify a GPIO by port and number.
 * Use GPIO_PA(5) for the Nucleo green LED LD2.
 */
#define GPIO_PA(x) ((GPIO_PORTA << GPIO_PORT_SHIFT) + (x))
#define GPIO_PB(x) ((GPIO_PORTB << GPIO_PORT_SHIFT) + (x))
#define GPIO_PC(x) ((GPIO_PORTC << GPIO_PORT_SHIFT) + (x))
#define GPIO_PD(x) ((GPIO_PORTD << GPIO_PORT_SHIFT) + (x))
#define GPIO_PE(x) ((GPIO_PORTE << GPIO_PORT_SHIFT) + (x))
#define GPIO_PF(x) ((GPIO_PORTF << GPIO_PORT_SHIFT) + (x))
#define GPIO_PG(x) ((GPIO_PORTG << GPIO_PORT_SHIFT) + (x))
#define GPIO_PH(x) ((GPIO_PORTH << GPIO_PORT_SHIFT) + (x))
#define GPIO_PI(x) ((GPIO_PORTI << GPIO_PORT_SHIFT) + (x))

#define GPIO_PIN_MASK 0xf

struct Gpio
{
    uint8_t gpio;
};

enum GpioAltFunc
{
    GPIO_AF00 = 0,
    GPIO_AF01,
    GPIO_AF02,
    GPIO_AF03,
    GPIO_AF04,
    GPIO_AF05,
    GPIO_AF06,
    GPIO_AF07,
    GPIO_AF08,
    GPIO_AF09,
    GPIO_AF10,
    GPIO_AF11,
    GPIO_AF12,
    GPIO_AF13,
    GPIO_AF14,
    GPIO_AF15,
    GPIO_AF_SYS = GPIO_AF00,
    GPIO_AF_TIM1 = GPIO_AF01,
    GPIO_AF_TIM2 = GPIO_AF01,
    GPIO_AF_TIM3 = GPIO_AF02,
    GPIO_AF_TIM4 = GPIO_AF02,
    GPIO_AF_TIM5 = GPIO_AF02,
    GPIO_AF_TIM9 = GPIO_AF03,
    GPIO_AF_TIM10 = GPIO_AF03,
    GPIO_AF_TIM11 = GPIO_AF03,
    GPIO_AF_I2C1 = GPIO_AF04,
    GPIO_AF_I2C2_A = GPIO_AF04,
    GPIO_AF_I2C3_A = GPIO_AF04,
    GPIO_AF_SPI1 = GPIO_AF05,
    GPIO_AF_I2S1 = GPIO_AF05,
    GPIO_AF_SPI2_A = GPIO_AF05,
    GPIO_AF_I2S2_A = GPIO_AF05,
    GPIO_AF_SPI3_A = GPIO_AF05,
    GPIO_AF_I2S3_A = GPIO_AF05,
    GPIO_AF_SPI2_B = GPIO_AF06,
    GPIO_AF_I2S2_B = GPIO_AF06,
    GPIO_AF_SPI3_B = GPIO_AF06,
    GPIO_AF_I2S3_B = GPIO_AF06,
    GPIO_AF_SPI4_B = GPIO_AF06,
    GPIO_AF_I2S4_B = GPIO_AF06,
    GPIO_AF_SPI5_B = GPIO_AF06,
    GPIO_AF_I2S5_B = GPIO_AF06,
    GPIO_AF_SPI3_C = GPIO_AF07,
    GPIO_AF_I2S3_C = GPIO_AF07,
    GPIO_AF_USART1 = GPIO_AF07,
    GPIO_AF_USART2 = GPIO_AF07,
    GPIO_AF_USART6 = GPIO_AF08,
    GPIO_AF_I2C2_B = GPIO_AF09,
    GPIO_AF_I2C3_B = GPIO_AF09,
    GPIO_AF_OTG1 = GPIO_AF10,
    GPIO_AF_SDIO = GPIO_AF12,
    GPIO_AF_EVENT = GPIO_AF15,
};

#ifdef __cplusplus
}
#endif

#endif
