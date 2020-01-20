#ifndef _I2C_STM32F411_CFG
#define _I2C_STM32F411_CFG

#include <stdint.h>

#ifndef DISABLED
#define DISABLED 0
#endif

#ifndef ENABLED
#define ENABLED 1
#endif

typedef uint32_t i2c_enabled_t;

typedef uint32_t i2c_ack_en_t;

typedef uint32_t i2c_dma_en_t;

typedef uint32_t i2c_it_err_en_t;

typedef uint32_t i2c_it_buf_en_t;

typedef uint32_t i2c_it_evt_en_t;

typedef enum
{
	I2C_1 = 0x00UL,
	I2C_2 = 0x01UL,
	I2C_3 = 0x02UL,
	NUM_I2C
} i2c_channel_t;


typedef enum
{
	I2C_SM = 0x00UL,
	I2C_FM = 0x01UL
}i2c_fast_slow_t;

typedef enum
{
	FM_MODE_2 = 0x00UL,
	FM_MODE_16_9 = 0x01
}i2c_fm_duty_cycle_t;

typedef struct
{
  i2c_enabled_t en;
  i2c_ack_en_t slave_ack_en;
  uint32_t periph_clk_freq_MHz;
  uint32_t i2c_op_freq_kHz;
  i2c_dma_en_t dma_en;
  i2c_it_err_en_t it_err_en;
  i2c_it_buf_en_t it_buf_en;
  i2c_it_evt_en_t it_evt_en;
  i2c_fast_slow_t fast_or_std;
  i2c_fm_duty_cycle_t duty_cycle;
} i2c_config_t;

const i2c_config_t *i2c_config_table_get(void);

#endif
