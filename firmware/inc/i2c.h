#ifndef _I2C_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

typedef uint8_t i2c_addr_t;
typedef uint32_t i2c_speed_t;

typedef void (*i2c_callback_t)(void *cookie, int tx, int rx);

int OS_I2C_master_request(uint8_t bus_id, i2c_speed_t speed);
int OS_I2C_master_release(uint8_t bus_id);
int OS_I2C_master_rxtx(uint8_t bus_id, uint8_t addr,
        const void *tx_buf, size_t tx_size,
        void *rx_buf, size_t rx_len, i2c_callback_t callback, void *cookie);

int OS_I2C_slave_request(uint8_t bus_id, i2c_addr_t addr);
int OS_I2C_slave_release(uint8_t bus_id);
void OS_I2C_slave_enable_rx(uint8_t bus_id, void *rx_buf, size_t size,
        i2c_callback_t callback, void *cookie);
void OS_I2C_slave_disable(uint8_t bus_id);
int OS_I2C_slave_send(uint8_t bus_id, const void *buf, size_t size,
        i2c_callback_t callback, void *cookie);

#ifdef __cplusplus
}
#endif

#endif

