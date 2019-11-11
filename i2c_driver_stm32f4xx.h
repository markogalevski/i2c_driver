#ifndef _I2C_H
#define _I2C_H
#include "stm32f411xe.h"
#include <stdint.h>
typedef struct
{
  uint32_t smbus_en;
  uint32_t smbus_type;
  uint32_t arp_en;
  uint32_t gencall_en;
  uint32_t no_stretch_dis;
  uint32_t clk_freq;
  uint32_t dma_en;
} i2c_init_t;

typedef struct
{
  I2C_TypeDef * instance;
  i2c_init_t init;
  uint32_t * p_tx_buffer;
  uint32_t * p_rx_buffer;
  uint32_t tx_len;
  uint32_t rx_len;
  uint32_t state;

} i2c_handle_t;

typedef enum
{
  error_interrupt = 8,
  event_interrupt = 9,
  buffer_interrupt = 10
} i2c_interrupt_t;

void i2c_generate_init(i2c_handle_t *);
void i2c_init(i2c_handle_t *);
void i2c_deinit(i2c_handle_t *);
void i2c_master_transmit(i2c_handle_t *, uint32_t *, uint32_t, uint32_t);
void i2c_master_receive(i2c_handle_t *, uint32_t *, uint32_t, uint32_t);
void i2c_slave_transmit();
void i2c_slave_receive();


#endif /* define _I2C_H */
