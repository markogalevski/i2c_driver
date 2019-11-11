#include "i2c_driver_stm32f4xx.h"

void i2c_enable_peripheral(i2c_handle_t *hi2c);
void i2c_disable_peripheral(i2c_handle_t *hi2c);
static void i2c_start_condition(i2c_handle_t *hi2c);
static void i2c_stop_condition(i2c_handle_t *hi2c);
static void i2c_ack_set(i2c_handle_t *hi2c);
static void i2c_ack_reset(i2c_handle_t *hi2c);
static void i2c_smalert_set(i2c_handle_t *hi2c);
static void i2c_smalert_reset(i2c_handle_t *hi2c);
static void i2c_swreset_set(i2c_handle_t *hi2c);
static void i2c_swreset_reset(i2c_handle_t *hi2c);

void i2c_generate_init(i2c_handle_t *hi2c)
{
	i2c_init_t local_init = {0};
	hi2c->init = local_init;
}

void i2c_init(i2c_handle_t *hi2c)
{
	i2c_disable_peripheral(hi2c);
	i2c_init_t init = hi2c->init;
	hi2c->instance->CR1 = (init.smbus_en << I2C_CR1_SMBUS_Pos)
			              | (init.smbus_type << I2C_CR1_SMBTYPE_Pos)
						  | (init.arp_en << I2C_CR1_ENARP_Pos)
						  | (init.no_stretch_dis << I2C_CR1_NOSTRETCH_Pos)
						  | (init.gencall_en << I2C_CR1_ENGC_Pos);
	hi2c->instance->CR2 = (init.clk_freq & (I2C_CR2_FREQ_Msk))
			              | (init.dma_en << I2C_CR2_DMAEN_Pos);
	i2c_enable_peripheral(hi2c);
}


void i2c_enable_peripheral(i2c_handle_t *hi2c)
{
	hi2c->instance->CR1 |= I2C_CR1_PE_Msk;
}

void i2c_disable_peripheral(i2c_handle_t *hi2c)
{
	hi2c->instance->CR1 &= ~(I2C_CR1_PE_Msk);
}

static void i2c_start_condition(i2c_handle_t *hi2c)
{
	hi2c->instance->CR1 |= I2C_CR1_START_Msk;
}

static void i2c_stop_condition(i2c_handle_t *hi2c)
{
	hi2c->instance->CR1 |= I2C_CR1_STOP_Msk;
}

static void i2c_ack_set(i2c_handle_t *hi2c)
{
	hi2c->instance->CR1 |= I2C_CR1_ACK_Msk;
}

static void i2c_ack_reset(i2c_handle_t *hi2c)
{
	hi2c->instance->CR1 &= ~(I2C_CR1_ACK_Msk);
}

//TODO: Maybe implement the error checking functionalities

static void i2c_smalert_set(i2c_handle_t *hi2c)
{
	hi2c->instance->CR1 |= I2C_CR1_ALERT_Msk;
}

static void i2c_smalert_reset(i2c_handle_t *hi2c)
{
	hi2c->instance->CR1 &= ~(I2C_CR1_ALERT_Msk);
}

static void i2c_swreset_set(i2c_handle_t *hi2c)
{
	hi2c->instance->CR1 |= I2C_CR1_SWRST_Msk;
}

static void i2c_swreset_reset(i2c_handle_t *hi2c)
{
	hi2c->instance->CR1 &= ~(I2C_CR1_SWRST_Msk);
}
