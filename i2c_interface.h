#ifndef _I2C_H
#define _I2C_H
#include "i2c_stm32f411_config.h"
#include <stdint.h>


typedef struct
{
  i2c_channel_t channel;
  uint8_t * buffer;
  uint32_t data_length;
  uint8_t slave_address;
} i2c_transfer_t;


void i2c_init(i2c_config_t *config_table);
void i2c_master_transmit(i2c_transfer_t *);
void i2c_master_receive(i2c_transfer_t *);
void i2c_slave_transmit(i2c_transfer_t *);
void i2c_slave_receive(i2c_transfer_t *);
void i2c_ev_irq_handler(i2c_channel_t channel);

void i2c_register_write(uint32_t i2c_register);
uint32_t i2c_register_read(uint32_t i2c_regsister);



#endif /* define _I2C_H */
