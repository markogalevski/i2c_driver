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

static uint32_t i2c_calculate_ccr(i2c_config_t *config_entry);
static uint32_t i2c_calculate_trise(i2c_config_t *config_entry);

void i2c_init(i2c_config_t *config_table)
{
	for (int i2c_channel = 0; i2c_channel < NUM_I2C; i2c_channel++)
	{
		if (config_table[i2c_channel].en == ENABLED)
		{
			*I2C_CR1[i2c_channel] &= ~(I2C_CR1_PE_Msk);

			*I2C_CR1[i2c_channel] = config_table[i2c_channel].slave_ack_en << I2C_CR1_ACK_Pos;

			assert(config_table[i2c_channel].periph_clk_freq_MHz <= 0x32
					&& config_table[i2c_channel].periph_clk_freq_MHz > 0x01);
			*I2C_CR2[i2c_channel] |= config_table[i2c_channel].periph_clk_freq_MHz << I2C_CR2_FREQ_Pos
									| config_table[i2c_channel].it_err_en << I2C_CR2_ITERREN_Pos
									| config_table[i2c_channel].it_buf_en << I2C_CR2_ITBUFEN_Pos
									| config_table[i2c_channel].it_evt_en << I2C_CR2_ITEVTEN_Pos
									| config_table[i2c_channel].dma_en << I2C_CR2_DMAEN_Pos;

			*I2C_CCR[i2c_channel] |= config_table[i2c_channel].fast_or_std << I2C_CCR_FS_Pos
									| config_table[i2c_channel].duty_cycle << I2C_CCR_DUTY_Pos;

			*I2C_CCR[i2c_channel] = i2c_calculate_ccr((i2c_config_t *) &config_table[i2c_channel]);
			*I2C_TRISE[i2c_channel] = i2c_calculate_trise((i2c_config_t *) &config_table[i2c_channel]);

			*I2C_CR1[i2c_channel] |= I2C_CR1_PE_Msk;
		}
	}
}

uint32_t i2c_calculate_ccr(i2c_config_t *config_entry)
{
	uint32_t calculated_ccr = 0;
	if (config_entry->fast_or_std == I2C_SM)
	{
		//TODO: Use doxygen to formally present equations
		assert(config_entry->i2c_op_freq_kHz <= 100);
		calculated_ccr = (config_entry->periph_clk_freq_MHz)/(2000UL*config_entry->i2c_op_freq_kHz);
	}
	else if (config_entry->fast_or_std == I2C_FM)
	{
		assert(config_entry->i2c_op_freq_kHz <= 400);
		if (config_entry->duty_cycle == FM_MODE_2)
		{
		calculated_ccr = (config_entry->periph_clk_freq_MHz)/(3000UL*config_entry->i2c_op_freq_kHz);
		}
		else if (config_entry->duty_cycle == FM_MODE_16_9)
		{
		calculated_ccr = (9UL*config_entry->periph_clk_freq_MHz)/(25000UL*config_entry->i2c_op_freq_kHz);
		}
	}
	assert(calculated_ccr < (0x02 << 12)); //Check for proper CCR size.
	return (calculated_ccr);
}

uint32_t i2c_calculate_trise(i2c_config_t *config_entry)
{
	uint32_t calculated_trise = 0;
	if (config_entry->fast_or_std == I2C_SM)
		{
		//for standard mode, max rise time is 1000 nano seconds. Or a single 1MHz pulse.
		calculated_trise = config_entry->periph_clk_freq_MHz + 1;
		}
	else if (config_entry->fast_or_std == I2C_FM)
	{
		//for fast mode, max rise time is 300 nano seconds, or 3.33MHz
		calculated_trise = (uint32_t) (config_entry->periph_clk_freq_MHz * 3.33F) + 1;
	}
	assert(calculated_trise < (0x02 << 6));

	return (calculated_trise);
}



