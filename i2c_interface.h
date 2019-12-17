#ifndef _I2C_H
#define _I2C_H
#include <i2c_stm32f411_config.h>
#include <stdint.h>


typedef struct
{
  i2c_channel_t channel;
  uint32_t * p_tx_buffer;
  uint32_t * p_rx_buffer;
  uint32_t tx_len;
  uint32_t rx_len;
  uint32_t state;
  uint8_t slave_address;
} i2c_handle_t;


void i2c_init(i2c_config_t *config_table);
void i2c_deinit(i2c_channel_t);


void i2c_master_transmit(i2c_handle_t *);
void i2c_master_receive(i2c_handle_t *);
void i2c_slave_transmit(i2c_handle_t *);
void i2c_slave_receive(i2c_handle_t *);

void i2c_register_write(uint32_t i2c_register);
uint32_t i2c_register_read(uint32_t i2c_regsister);



#endif /* define _I2C_H */
