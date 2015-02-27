#ifndef _STM32F411_GPIO_H_
#define _STM32F411_GPIO_H_

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

struct gpio
{
    uint8_t gpio;
};

void gpio_assign_func(const struct gpio* __restrict gpio, uint8_t func);



/* a select few alternate assignment bits */
#define GPIO_A2_AFR_I2C              4
#define GPIO_A2_AFR_SPI123           5
#define GPIO_A2_AFR_SPI345           6
#define GPIO_A2_AFR_USART2           7

#ifdef __cplusplus
}
#endif

#endif

