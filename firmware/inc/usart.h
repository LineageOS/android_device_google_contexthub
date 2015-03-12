#ifndef _USART_H_
#define _USART_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <gpio.h>
#include <stdint.h>

struct usart;

typedef uint8_t UsartPort;

typedef enum
{
    USART_DATA_BITS_8,
    USART_DATA_BITS_9,
} UsartDataBitsCfg;

typedef enum
{
    USART_STOP_BITS_0_5,
    USART_STOP_BITS_1_0,
    USART_STOP_BITS_1_5,
    USART_STOP_BITS_2_0,
} UsatStopBitsCfg;

typedef enum
{
    USART_PARITY_NONE,
    USART_PARITY_EVEN,
    USART_PARITY_ODD,
} UsartParityCfg;

typedef enum
{
    USART_FLOW_CONTROL_NONE,
    USART_FLOW_CONTROL_RTS,
    USART_FLOW_CONTROL_CTS,
    USART_FLOW_CONTROL_RTS_CTS,
} UsartFlowControlCfg;

void usartOpen(struct usart* __restrict usart, UsartPort port, /* port number is 1-based!!!!! */
                GpioNum tx, GpioNum rx,
                uint32_t baud, UsartDataBitsCfg data_bits,
                UsatStopBitsCfg stop_bits, UsartParityCfg parity,
                UsartFlowControlCfg flow_control);
void usartClose(const struct usart* __restrict usart);
void usartPutchat(const struct usart* __restrict usart, char c);

#ifdef __cplusplus
}
#endif

#endif

