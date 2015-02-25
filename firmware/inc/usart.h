#ifndef _USART_H_
#define _USART_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <gpio.h>
#include <stdint.h>

struct usart;

typedef uint8_t usart_port_t;

typedef enum
{
    USART_DATA_BITS_8,
    USART_DATA_BITS_9,
} usart_data_t;

typedef enum
{
    USART_STOP_BITS_0_5,
    USART_STOP_BITS_1_0,
    USART_STOP_BITS_1_5,
    USART_STOP_BITS_2_0,
} usart_stop_t;

typedef enum
{
    USART_PARITY_NONE,
    USART_PARITY_EVEN,
    USART_PARITY_ODD,
} usart_parity_t;

typedef enum
{
    USART_FLOW_CONTROL_NONE,
    USART_FLOW_CONTROL_RTS,
    USART_FLOW_CONTROL_CTS,
    USART_FLOW_CONTROL_RTS_CTS,
} usart_flow_control_t;

void usart_open(struct usart* __restrict usart, usart_port_t port, /* port number is 1-based!!!!! */
                gpio_number_t tx, gpio_number_t rx,
                uint32_t baud, usart_data_t data_bits,
                usart_stop_t stop_bits, usart_parity_t parity,
                usart_flow_control_t flow_control);
void usart_close(const struct usart* __restrict usart);
void usart_putchar(const struct usart* __restrict usart, char c);

#ifdef __cplusplus
}
#endif

#endif

