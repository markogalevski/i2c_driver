#include <i2c_interface.h>
#include "stm32f411xe.h"
#include <assert.h>


#define SM_RISE_TIME_MAX 1000
#define FM_RISE_TIME_MAX 300

static volatile uint16_t *const I2C_CR1[NUM_I2C] =
{
	(uint16_t *)I2C1_BASE, (uint16_t *)I2C2_BASE, (uint16_t *)I2C3_BASE
};

static volatile uint16_t *const I2C_CR2[NUM_I2C] =
{
	(uint16_t *)I2C1_BASE + 0x04UL, (uint16_t *)I2C2_BASE + 0x04UL,
	(uint16_t *)I2C3_BASE + 0x04UL

};

static volatile uint16_t *const I2C_OAR1[NUM_I2C] =
{
	(uint16_t *)I2C1_BASE + 0x08UL, (uint16_t *)I2C2_BASE + 0x08UL,
	(uint16_t *)I2C3_BASE + 0x08UL
};

static volatile uint16_t *const I2C_OAR2[NUM_I2C] =
{
	(uint16_t *)I2C1_BASE + 0x0CUL, (uint16_t *)I2C2_BASE + 0x0CUL,
	(uint16_t *)I2C3_BASE + 0x0CUL
};

static volatile uint16_t *const I2C_DR[NUM_I2C] =
{
	(uint16_t *)I2C1_BASE + 0x10UL, (uint16_t *)I2C2_BASE + 0x10UL,
	(uint16_t *)I2C3_BASE + 0x10UL
};

static volatile uint16_t *const I2C_SR1[NUM_I2C] =
{
	(uint16_t *)I2C1_BASE + 0x14UL, (uint16_t *)I2C2_BASE + 0x14UL,
	(uint16_t *)I2C3_BASE + 0x14UL
};

static volatile uint16_t *const I2C_SR2[NUM_I2C] =
{
	(uint16_t *)I2C1_BASE + 0x18UL, (uint16_t *)I2C2_BASE + 0x18UL,
	(uint16_t *)I2C3_BASE + 0x18UL
};

static volatile uint16_t *const I2C_CCR[NUM_I2C] =
{
	(uint16_t *)I2C1_BASE + 0x1CUL, (uint16_t *)I2C2_BASE + 0x1CUL,
	(uint16_t *)I2C3_BASE + 0x1CUL
};

static volatile uint16_t *const I2C_TRISE[NUM_I2C] =
{
	(uint16_t *)I2C1_BASE + 0x20UL, (uint16_t *)I2C2_BASE + 0x20UL,
	(uint16_t *)I2C3_BASE + 0x20UL
};

static volatile uint16_t *const I2C_FLTR[NUM_I2C] =
{
	(uint16_t *)I2C1_BASE + 0x24UL, (uint16_t *)I2C2_BASE + 0x24UL,
	(uint16_t *)I2C3_BASE + 0x24UL
};

static uint32_t i2c_calculate_ccr(i2c_config_t *config_entry);
static uint32_t i2c_calculate_trise(i2c_config_t *config_entry);
static void i2c_clear_addr_bit(i2c_channel_t channel);
static void i2c_clear_stopf_bit(i2c_channel_t channel);
static void i2c_one_byte_reception(i2c_transfer_t *i2c_transfer);
static void i2c_two_byte_reception(i2c_transfer_t *i2c_transfer);
static void i2c_n_byte_reception(i2c_transfer_t *i2c_transfer);


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
		assert(config_entry->periph_clk_freq_MHz > 0x03);
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
		//for fast mode, max rise time is 300 nanoseconds, or 3.33MHz
		calculated_trise = (uint32_t) (config_entry->periph_clk_freq_MHz * 3.33F) + 1;
	}
	assert(calculated_trise < (0x02 << 6));

	return (calculated_trise);
}

void i2c_master_transmit(i2c_transfer_t *i2c_transfer)
{
	uint16_t status_reg = 0;
	//1. Generate START condition
	*I2C_CR1[i2c_transfer->channel] |= (I2C_CR1_START_Msk);
	//2. Read SR1
	status_reg = *I2C_SR1[i2c_transfer->channel];
	//disable POS
	*I2C_CR1[i2c_transfer->channel] &= ~I2C_CR1_POS_Msk;
	//3. Write Slave Address into DR
	*I2C_DR[i2c_transfer->channel] = i2c_transfer->slave_address;
	//4. Read SR1
	status_reg = *I2C_SR1[i2c_transfer->channel];
	//5. Read SR2
	status_reg = *I2C_SR2[i2c_transfer->channel];

	//loop
	while(i2c_transfer->data_length > 0)
	{
		//6. Wait for TxE
	    do
		{
		  status_reg = *I2C_SR1[i2c_transfer->channel] & I2C_SR1_TXE_Msk;
		}while (status_reg == 0);
	    //6. Write data byte into DR
		*I2C_DR[i2c_transfer->channel] = *(i2c_transfer->buffer);
		i2c_transfer->buffer++;
		i2c_transfer->data_length--;
		status_reg = I2C_SR1[i2c_transfer->channel] & I2C_SR1_BTF_Msk;
		if (status_reg != 0 && i2c_transfer->data_length != 0)
		{
			*I2C_DR[i2c_transfer->channel] = *(i2c_transfer->buffer);
			i2c_transfer->buffer++;
			i2c_transfer->data_length--;
		}

	}
	//8. Send STOP condition
	*I2C_CR1[i2c_transfer->channel] |= (I2C_CR1_STOP_Msk);

}

void i2c_master_receive(i2c_transfer_t *i2c_transfer)
{
	/*
	 * NOTE: Only using 7 bit addressing. Will extend for 10 if necessary
	 * Master reception for i2c has 3 major potential states depending on transfer length (N) bytes:
	 * 1. N = 1; instant ACK disable
	 * 2. N = 2; instant ACK disable followed by a double read
	 * 3. N > 2; disables ACK upon the last 3 bytes.
	 */
	uint16_t status_reg = 0;
	assert(i2c_transfer->data_length != 0 && i2c_transfer->buffer != 0UL);

	*I2C_CR1[i2c_transfer->channel] |= I2C_CR1_ACK_Msk; //assume ack will be used
	//1. Generate START condition
	*I2C_CR1[i2c_transfer->channel] |= (I2C_CR1_START_Msk);
	//2. Read SR1
	status_reg = *I2C_SR1[i2c_transfer->channel];
	//3. Write Slave Address +1 into DR
	*I2C_DR[i2c_transfer->channel] = i2c_transfer->slave_address | (0x01);
	//SB bit is now clear.
	/*
	 * this is where the transfer methods branch.
	 */
	do
	{
		status_reg = I2C_SR1[i2c_transfer->channel] & I2C_SR1_ADDR_Msk;
	} while (status_reg == 0);

	if (i2c_transfer->data_length == 0UL)
	{
		i2c_clear_addr_bit(i2c_transfer->channel);
		*I2C_CR1[i2c_transfer->channel] &= ~I2C_CR1_STOP_Msk;
	}
	else if (i2c_transfer->data_length == 1UL)
	{
		i2c_one_byte_reception(i2c_transfer);
	}
	else if (i2c_transfer->data_length == 2UL)
	{
		i2c_two_byte_reception(i2c_transfer);
	}
	else
	{
		i2c_n_byte_reception(i2c_transfer);
	}
}

void i2c_slave_transmit(i2c_transfer_t *i2c_transfer)
{
  uint32_t status_reg;
  uint32_t af_flag = 0;
  uint32_t txe_flag = 1;
  assert(i2c_transfer->buffer != 0);
  *I2C_CR1[i2c_transfer->channel] |= I2C_CR1_ACK_Msk;
  do
  {
	  status_reg = *I2C_SR1[i2c_transfer->channel] & I2C_SR1_ADDR_Msk;
  } while(status_reg == 0);
  i2c_clear_addr_bit(i2c_transfer->channel);
  while (i2c_transfer->data_length > 0UL)
  {
	  do
	  {
	  status_reg = *I2C_SR1[i2c_transfer->channel] & I2C_SR1_TXE_Msk;
	  }while (status_reg ==  0);

	  *I2C_DR[i2c_transfer->channel] = *i2c_transfer->buffer;
	  i2c_transfer->buffer++;
	  i2c_transfer->data_length--;

	  status_reg = *I2C_SR1[i2c_transfer->channel] & I2C_SR1_BTF_Msk;
	  if (status_reg && i2c_transfer->data_length != 0)
	  {
		  *I2C_DR[i2c_transfer->channel] = *i2c_transfer->buffer;
		  i2c_transfer->buffer++;
		  i2c_transfer->data_length--;
	  }
  }

  	  *I2C_SR1[i2c_transfer->channel] &= ~I2C_SR1_AF_Msk;
  	  *I2C_CR1[i2c_transfer->channel] &= ~I2C_CR1_ACK_Msk;

}

void i2c_slave_receive(i2c_transfer_t *i2c_transfer)
{
	uint32_t status_reg;
	uint32_t stopf_flag = 0;
	assert(i2c_transfer->buffer != 0);
	*I2C_CR1[i2c_transfer->channel] |= I2C_CR1_ACK_Msk;
	*I2C_CR1[i2c_transfer->channel] &= ~I2C_CR1_POS_Msk;
	do
	{
	  status_reg = *I2C_SR1[i2c_transfer->channel] & I2C_SR1_ADDR_Msk;
	} while(status_reg == 0);
	i2c_clear_addr_bit(i2c_transfer->channel);

	while(i2c_transfer->data_length > 0)
	{
		do
		{
			status_reg = *I2C_SR1[i2c_transfer->channel] & I2C_SR1_RXNE_Msk;
		} while(status_reg == 0);

		*i2c_transfer->buffer = (uint8_t) *I2C_DR[i2c_transfer->channel];
		i2c_transfer->buffer++;
		i2c_transfer->data_length--;
		status_reg = *I2C_SR1[i2c_transfer->channel] & I2C_SR1_BTF_Msk;
	    if (status_reg && i2c_transfer->data_length != 0)
	    {
	    	*i2c_transfer->buffer = (uint8_t) *I2C_DR[i2c_transfer->channel];
	    	i2c_transfer->buffer++;
	    	i2c_transfer->data_length--;
	    }
	}

	do
		status_reg = *I2C_SR1[i2c_transfer->channel] & I2C_SR1_STOPF_Msk;
	while (status_reg == 0);
	i2c_clear_stopf_bit(i2c_transfer->channel);
	*I2C_CR1[i2c_transfer->channel] &= ~I2C_CR1_ACK_Msk;
}



static void i2c_clear_addr_bit(i2c_channel_t channel)
{
	uint32_t status_reg = *I2C_SR1[channel];
	status_reg = *I2C_SR2[channel];
}

static void i2c_clear_stopf_bit(i2c_channel_t channel)
{
	uint32_t status_reg = *I2C_SR1[channel];
	*I2C_CR1[channel] |= I2C_CR1_STOP_Msk;
}

static void i2c_one_byte_reception(i2c_transfer_t *i2c_transfer)
{
	uint32_t status_reg;
	*I2C_CR1[i2c_transfer->channel] &= ~I2C_CR1_ACK_Msk;
	i2c_clear_addr_bit(i2c_transfer->channel);
	*I2C_CR1[i2c_transfer->channel] |= I2C_CR1_STOP_Msk;
	do
	{
	  status_reg = *I2C_SR1[i2c_transfer->channel] & I2C_SR1_RXNE_Msk;
	} while(status_reg == 0);
	*i2c_transfer->buffer = (uint8_t) *I2C_DR[i2c_transfer->channel];
	i2c_transfer->buffer++;
	i2c_transfer->data_length--;
}

static void i2c_two_byte_reception(i2c_transfer_t *i2c_transfer)
{
	uint32_t status_reg;
	*I2C_CR1[i2c_transfer->channel] &= ~I2C_CR1_ACK_Msk;
	*I2C_CR1[i2c_transfer->channel] |= I2C_CR1_POS_Msk;
	i2c_clear_addr_bit(i2c_transfer->channel);
	do
	{
		status_reg = *I2C_SR1[i2c_transfer->channel] & I2C_SR1_BTF_Msk;
	} while (status_reg == 0);
	*I2C_CR1[i2c_transfer->channel] |= I2C_CR1_STOP_Msk;
	*i2c_transfer->buffer = (uint8_t) *I2C_DR[i2c_transfer->channel];
	i2c_transfer->buffer++;
	i2c_transfer->data_length--;
	*i2c_transfer->buffer = (uint8_t) *I2C_DR[i2c_transfer->channel];
	i2c_transfer->buffer++;
	i2c_transfer->data_length--;
}

static void i2c_n_byte_reception(i2c_transfer_t *i2c_transfer)
{
  uint32_t status_reg;
  while (i2c_transfer->data_length > 3)
  	{
  	  do
  	  {
  	    status_reg = *I2C_SR1[i2c_transfer->channel] & I2C_SR1_RXNE_Msk;
  	  } while(status_reg ==  0);
	  *i2c_transfer->buffer = *I2C_DR[i2c_transfer->channel];
  	  i2c_transfer->buffer++;
  	  i2c_transfer->data_length--;
  	  status_reg = I2C_SR1[i2c_transfer->channel] & I2C_SR1_BTF_Msk;
  	  if (status_reg &&  i2c_transfer->data_length > 3)
  	  {
  		  *i2c_transfer->buffer = *I2C_DR[i2c_transfer->channel];
  	  	  i2c_transfer->buffer++;
  	  	  i2c_transfer->data_length--;
  	  }
  	}

  	do
  	{
  	  status_reg = *I2C_SR1[i2c_transfer->channel] & I2C_SR1_BTF_Msk;
  	}while(status_reg == 0);

  	*I2C_CR1[i2c_transfer->channel] &= ~I2C_CR1_ACK_Msk;
  	*i2c_transfer->buffer = *I2C_DR[i2c_transfer->channel];
  	i2c_transfer->buffer++;
  	i2c_transfer->data_length--;

 	do
  	{
  	 status_reg = *I2C_SR1[i2c_transfer->channel] & I2C_SR1_BTF_Msk;
  	} while(status_reg ==  0);
  	*I2C_CR1[i2c_transfer->channel] |= I2C_CR1_STOP_Msk;
  	*i2c_transfer->buffer = *I2C_DR[i2c_transfer->channel];
  	i2c_transfer->buffer++;
  	i2c_transfer->data_length--;
  	*i2c_transfer->buffer = *I2C_DR[i2c_transfer->channel];
  	i2c_transfer->buffer++;
  	i2c_transfer->data_length--;
}
