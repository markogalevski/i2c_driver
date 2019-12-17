#include <i2c_interface.h>
#include "stm32f411xe.h"
#include <assert.h>

#define SM_RISE_TIME_MAX 1000
#define FM_RISE_TIME_MAX 300

volatile uint16_t *const I2C_CR1[NUM_I2C] =
{
	(uint16_t *)I2C1_BASE, (uint16_t *)I2C2_BASE, (uint16_t *)I2C3_BASE
};

volatile uint16_t *const I2C_CR2[NUM_I2C] =
{
	(uint16_t *)I2C1_BASE + 0x04UL, (uint16_t *)I2C2_BASE + 0x04UL,
	(uint16_t *)I2C3_BASE + 0x04UL

};

volatile uint16_t *const I2C_OAR1[NUM_I2C] =
{
	(uint16_t *)I2C1_BASE + 0x08UL, (uint16_t *)I2C2_BASE + 0x08UL,
	(uint16_t *)I2C3_BASE + 0x08UL
};

volatile uint16_t *const I2C_OAR2[NUM_I2C] =
{
	(uint16_t *)I2C1_BASE + 0x0CUL, (uint16_t *)I2C2_BASE + 0x0CUL,
	(uint16_t *)I2C3_BASE + 0x0CUL
};

volatile uint16_t *const I2C_DR[NUM_I2C] =
{
	(uint16_t *)I2C1_BASE + 0x10UL, (uint16_t *)I2C2_BASE + 0x10UL,
	(uint16_t *)I2C3_BASE + 0x10UL
};

volatile uint16_t *const I2C_SR1[NUM_I2C] =
{
	(uint16_t *)I2C1_BASE + 0x14UL, (uint16_t *)I2C2_BASE + 0x14UL,
	(uint16_t *)I2C3_BASE + 0x14UL
};

volatile uint16_t *const I2C_SR2[NUM_I2C] =
{
	(uint16_t *)I2C1_BASE + 0x18UL, (uint16_t *)I2C2_BASE + 0x18UL,
	(uint16_t *)I2C3_BASE + 0x18UL
};

volatile uint16_t *const I2C_CCR[NUM_I2C] =
{
	(uint16_t *)I2C1_BASE + 0x1CUL, (uint16_t *)I2C2_BASE + 0x1CUL,
	(uint16_t *)I2C3_BASE + 0x1CUL
};

volatile uint16_t *const I2C_TRISE[NUM_I2C] =
{
	(uint16_t *)I2C1_BASE + 0x20UL, (uint16_t *)I2C2_BASE + 0x20UL,
	(uint16_t *)I2C3_BASE + 0x20UL
};

volatile uint16_t *const I2C_FLTR[NUM_I2C] =
{
	(uint16_t *)I2C1_BASE + 0x24UL, (uint16_t *)I2C2_BASE + 0x24UL,
	(uint16_t *)I2C3_BASE + 0x24UL
};

static uint32_t i2c_calculate_ccr(i2s_channel_t i2c);
static uint32_t i2c_calculate_trise(i2c_channel_t);
void i2c_init(i2c_config_t *config_table)
{
	for (int i2c_channel = 0; i2c_channel < NUM_I2C; i2c_channel++)
	{
		if (config_table[i2c_channel].en == ENABLED)
		{
			*I2C_CR1[i2c_channel] &= ~(0x01UL << I2C_CR1_PE_Pos);

			*I2C_CR1[i2c_channel] = config_table[i2c_channel].slave_ack_en << I2C_CR1_ACK_Pos;

			assert(config_table[i2c_channel].periph_clk_freq <= 0x32
					&& config_table[i2c_channel].periph_clk_freq > 0x01);
			*I2C_CR2[i2c_channel] |= config_table[i2c_channel].periph_clk_freq << I2C_CR2_FREQ_Pos
									| config_table[i2c_channel].it_err_en << I2C_CR2_ITERREN_Pos
									| config_table[i2c_channel].it_buf_en << I2C_CR2_ITBUFEN_Pos
									| config_table[i2c_channel].it_evt_en << I2C_CR2_ITEVTEN_Pos
									| config_table[i2c_channel].dma_en << I2C_CR2_DMAEN_Pos;

			*I2C_CCR[i2c_channel] |= config_table[i2c_channel].fast_or_slow << I2C_CCR_FS_Pos
									| config_table[i2c_channel].duty_cycle << I2C_CCR_DUTY_Pos;

			*I2C_CCR[i2c_channel] = i2c_calculate_ccr(i2c_channel);
			*I2C_TRISE[i2c_channel] = i2c_calculate_trise(i2c_channel);

			*I2C_CR1[i2c_channel] |= 0x01UL << I2C_CR1_PE_Pos;
		}
	}
}

uint32_t i2c_calculate_ccr(i2c_channel_t i2c)
{
	uint32_t fast_slow = *I2C_CCR[i2c] & (I2C_CCR_FS_Msk);
	if (fast_slow == 0)
	{

	}
	else
	{

	}
}

uint32_t i2c_calculate_trise(i2c_channel_t)
{

}


