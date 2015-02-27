#ifndef _I2C_H_
#define _I2C_H_

#include "stm32_bl.h"

typedef struct i2c_handle
{
    handle_t handle;
    int fd;
} i2c_handle_t;

uint8_t write_data(handle_t *handle, uint8_t *buffer, int length);
uint8_t write_cmd(handle_t *handle, uint8_t cmd);
uint8_t read_data(handle_t *handle, uint8_t *data, int length);
uint8_t read_ack(handle_t *handle);

#endif /* _I2C_H_ */
