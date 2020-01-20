/*******************************************************************************
* Title                 :   I2C Implementation for STM32F411
* Filename              :   i2c_stm32f411.c
* Author                :   Marko Galevski
* Origin Date           :   20/01/2020
* Version               :   1.0.0
* Compiler              :   GCC
* Target                :   STM32F411 (Arm Cortex M4)
* Notes                 :   None
*
*
*******************************************************************************/
/****************************************************************************
* Doxygen C Template
* Copyright (c) 2013 - Jacob Beningo - All Rights Reserved
*
* Feel free to use this Doxygen Code Template at your own risk for your own
* purposes.  The latest license and updates for this Doxygen C template can be
* found at www.beningo.com or by contacting Jacob at jacob@beningo.com.
*
* For updates, free software, training and to stay up to date on the latest
* embedded software techniques sign-up for Jacobs newsletter at
* http://www.beningo.com/814-2/
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Template.
*
*****************************************************************************/

/** @file i2c_stm32f411.c
 *  @brief Chip specific implementation for i2c communication.
 */
#include "i2c_interface.h"
#include "stm32f411xe.h"
#include <assert.h>

/**
 * Rise times obtained from the phillips i2c spec sheet
 */
#define SM_RISE_TIME_MAX 1000 /**<Maximum rise time for a stanard mode pulse in ns.*/
#define FM_RISE_TIME_MAX 300 /**<Maximum rise time for a fast mode pulse in ns.  */

/**
 * Array of pointers to the Control Register 1 registers
 */
static volatile uint16_t *const I2C_CR1[NUM_I2C] =
{
	(uint16_t *)I2C1_BASE, (uint16_t *)I2C2_BASE, (uint16_t *)I2C3_BASE
};

/**
 * Array of pointers to the Control Register 2 registers
 */
static volatile uint16_t *const I2C_CR2[NUM_I2C] =
{
	(uint16_t *)I2C1_BASE + 0x04UL, (uint16_t *)I2C2_BASE + 0x04UL,
	(uint16_t *)I2C3_BASE + 0x04UL

};

/**
 * Array of pointers to the Own Address 1 registers
 */
static volatile uint16_t *const I2C_OAR1[NUM_I2C] =
{
	(uint16_t *)I2C1_BASE + 0x08UL, (uint16_t *)I2C2_BASE + 0x08UL,
	(uint16_t *)I2C3_BASE + 0x08UL
};

/**
 * Array of pointers to the Own Address 2 registers
 */
static volatile uint16_t *const I2C_OAR2[NUM_I2C] =
{
	(uint16_t *)I2C1_BASE + 0x0CUL, (uint16_t *)I2C2_BASE + 0x0CUL,
	(uint16_t *)I2C3_BASE + 0x0CUL
};

/**
 * Array of pointers to the Data registers
 */
static volatile uint16_t *const I2C_DR[NUM_I2C] =
{
	(uint16_t *)I2C1_BASE + 0x10UL, (uint16_t *)I2C2_BASE + 0x10UL,
	(uint16_t *)I2C3_BASE + 0x10UL
};

/**
 * Array of pointers to the Status Register 1 registers
 */
static volatile uint16_t *const I2C_SR1[NUM_I2C] =
{
	(uint16_t *)I2C1_BASE + 0x14UL, (uint16_t *)I2C2_BASE + 0x14UL,
	(uint16_t *)I2C3_BASE + 0x14UL
};

/**
 * Array of pointers to the Status Register 2 registers
 */
static volatile uint16_t *const I2C_SR2[NUM_I2C] =
{
	(uint16_t *)I2C1_BASE + 0x18UL, (uint16_t *)I2C2_BASE + 0x18UL,
	(uint16_t *)I2C3_BASE + 0x18UL
};

/**
 * Array of pointers to the clock control registers
 */
static volatile uint16_t *const I2C_CCR[NUM_I2C] =
{
	(uint16_t *)I2C1_BASE + 0x1CUL, (uint16_t *)I2C2_BASE + 0x1CUL,
	(uint16_t *)I2C3_BASE + 0x1CUL
};

/**
 * Array of pointers to the rise time registers
 */
static volatile uint16_t *const I2C_TRISE[NUM_I2C] =
{
	(uint16_t *)I2C1_BASE + 0x20UL, (uint16_t *)I2C2_BASE + 0x20UL,
	(uint16_t *)I2C3_BASE + 0x20UL
};

/**
 * Array of pointers to the filter registers
 */
static volatile uint16_t *const I2C_FLTR[NUM_I2C] =
{
	(uint16_t *)I2C1_BASE + 0x24UL, (uint16_t *)I2C2_BASE + 0x24UL,
	(uint16_t *)I2C3_BASE + 0x24UL
};

/**
 * Static array which holds copies of requested interrupt based transfers
 */
static i2c_transfer_t i2c_interrupt_transfers[NUM_I2C];

/**
 * Callback typedef for interrupt callbacks
 */
typedef void (*i2c_interrupt_callback_t)(i2c_transfer_t *);
/**
 * Static array containing interrupt callbacks currently mapped to
 * each i2c channel
 */
static i2c_interrupt_callback_t i2c_interrupt_callbacks[NUM_I2C];

static uint32_t i2c_calculate_ccr(i2c_config_t *config_entry);
static uint32_t i2c_calculate_trise(i2c_config_t *config_entry);
static void i2c_clear_addr_bit(i2c_channel_t channel);
static void i2c_clear_stopf_bit(i2c_channel_t channel);
static void i2c_one_byte_reception(i2c_transfer_t *i2c_transfer);
static void i2c_two_byte_reception(i2c_transfer_t *i2c_transfer);
static void i2c_n_byte_reception(i2c_transfer_t *i2c_transfer);

/******************************************************************************
* Function: i2c_init()
*//**
* \b Description:
*
* 	Carries out the initialisation of the I2C channels as per the information
* 	in the config table
*
*
* PRE-CONDITION: The config table has been obtained and is non-null

* @return 		void
*
* \b Example:
* @code
*	const i2c_config_t *config_table = i2c_config_get();
*	i2c_init(config_table);
* @endcode
*
* @see i2c_config_get
* <br><b> - CHANGE HISTORY - </b>
*
* <table align="left" style="width:800px">
* <tr><td> Date       </td><td> Software Version </td><td> Initials </td><td> Description </td></tr>
* </table><br><br>
* <hr>
*******************************************************************************/
void i2c_init(const i2c_config_t *config_table)
{
	for (int i2c_channel = 0; i2c_channel < NUM_I2C; i2c_channel++)
	{
		if (config_table[i2c_channel].en == ENABLED)
		{
			*I2C_CR1[i2c_channel] &= ~(I2C_CR1_PE_Msk);

			*I2C_CR1[i2c_channel] = config_table[i2c_channel].ack_en << I2C_CR1_ACK_Pos;

			assert(config_table[i2c_channel].periph_clk_freq_MHz <= 50
					&& config_table[i2c_channel].periph_clk_freq_MHz > 1);

			*I2C_CR2[i2c_channel] |= config_table[i2c_channel].periph_clk_freq_MHz << I2C_CR2_FREQ_Pos;
			*I2C_CCR[i2c_channel] |= config_table[i2c_channel].fast_or_std << I2C_CCR_FS_Pos
									| config_table[i2c_channel].duty_cycle << I2C_CCR_DUTY_Pos;

			*I2C_CCR[i2c_channel] = i2c_calculate_ccr((i2c_config_t *) &config_table[i2c_channel]);
			*I2C_TRISE[i2c_channel] = i2c_calculate_trise((i2c_config_t *) &config_table[i2c_channel]);

			*I2C_CR1[i2c_channel] |= I2C_CR1_PE_Msk;
		}
	}
}

/******************************************************************************
* Function: i2c_interrupt_control()
*//**
* \b Description:
*
* 	Enabled or disables the selected interrupt on the selected channel. Caled both by users
* 	and within the driver itself
*
*
* PRE-CONDITION: The i2c_init function has been carried out successfully

* @return 		void
*
* \b Example:
* @code
*	i2c_interrupt_control(I2C_2, IT_BUF, INTERRUPT_ENABLED);
* @endcode
*
* @see i2c_init
* @see i2c_master_transmit_it
* @see i2c_master_receive_it
* @see i2c_slave_transmit_it
* @see i2c_slave_receive_it
* <br><b> - CHANGE HISTORY - </b>
*
* <table align="left" style="width:800px">
* <tr><td> Date       </td><td> Software Version </td><td> Initials </td><td> Description </td></tr>
* </table><br><br>
* <hr>
*******************************************************************************/
void i2c_interrupt_control(i2c_channel_t channel, i2c_interrupt_dma_t interrupt, i2c_interrupt_control_t signal)
{
	if (signal == INTERRUPT_ENABLED)
	{
		*I2C_CR2[channel] |= (0x01UL) << (8 + interrupt);
	}
	else
	{
		*I2C_CR2[channel] &= ~(0x01UL) << (8 + interrupt);
	}
}

/******************************************************************************
* Function: i2c_calculate_ccr()
*//**
* \b Description:
*
* 	Static inline function called from within the driver to carry out the calculation of
* 	the required pulse length for a given frequency.
*
*
* PRE-CONDITION: The config table has been obtained and is non-null

* @return 		uint32_t
*
* \b Example:
* 	Automatically called within i2c_init
*
* @see i2c_init
* @see i2c_calculate_trise
* <br><b> - CHANGE HISTORY - </b>
*
* <table align="left" style="width:800px">
* <tr><td> Date       </td><td> Software Version </td><td> Initials </td><td> Description </td></tr>
* </table><br><br>
* <hr>
*******************************************************************************/
static inline uint32_t i2c_calculate_ccr(i2c_config_t *config_entry)
{
	uint32_t calculated_ccr = 0;
	if (config_entry->fast_or_std == I2C_SM)
	{
		//TODO: Use doxygen to formally present equations
		assert(config_entry->i2c_op_freq_kHz <= 100);
		/**
		 * Equation obtained from RM0383 18.6.8:
		 * \f$ CCR = \frac{T_(high)}{T_pclk}\f$,
		 * where
		 * \f$ T_(high) = \frac{1}{2 \times T_(opfreq)} \f$ (kHz)
		 * and
		 * \f$ T_(op_freq) = \frac{1}{f_(opfreq)} \f$
		 * and \f$ T_(pclk) = \frac{1}{f_(pclk)}
		 * leading to
		 * \f$ CCR = \frac{f_pclk (MHz) }{2 \times f_(opfreq) (khZ)}
		 * 			= \frac{f_pclk}{2000 \times f_(opfreq)}
		 */
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

static inline uint32_t i2c_calculate_trise(i2c_config_t *config_entry)
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
	i2c_clear_addr_bit(i2c_transfer->channel);

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
		status_reg = *I2C_SR1[i2c_transfer->channel] & I2C_SR1_BTF_Msk;
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
		status_reg = *I2C_SR1[i2c_transfer->channel] & I2C_SR1_ADDR_Msk;
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


static void i2c_master_transmit_it_callback(i2c_transfer_t *i2c_transfer)
{
	uint32_t status_reg = *I2C_SR1[i2c_transfer->channel];
	if (status_reg & I2C_SR1_ADDR_Msk)
		i2c_clear_addr_bit(i2c_transfer->channel);
	else if ((status_reg & I2C_SR1_TXE_Msk) && i2c_transfer->data_length > 0)
	{
		*I2C_DR[i2c_transfer->channel] = *i2c_transfer->buffer;
		i2c_transfer->buffer++;
		i2c_transfer->data_length--;
		status_reg = *I2C_SR1[i2c_transfer->channel];
	}
	if ((status_reg & I2C_SR1_BTF_Msk) && i2c_transfer->data_length > 0)
	{
		*I2C_DR[i2c_transfer->channel] = *i2c_transfer->buffer;
		i2c_transfer->buffer++;
		i2c_transfer->data_length--;
	}

	if (i2c_transfer->data_length == 0)
	{
		*I2C_CR2[i2c_transfer->channel] &= ~(I2C_CR2_ITEVTEN_Msk | I2C_CR2_ITBUFEN_Msk);
		*I2C_CR1[i2c_transfer->channel] |= I2C_CR1_STOP_Msk;
	}
}

static void i2c_master_receive_it_callback(i2c_transfer_t *i2c_transfer)
{
	uint32_t status_reg = *I2C_SR1[i2c_transfer->channel];
	if (status_reg & I2C_SR1_ADDR_Msk)
	{
		i2c_clear_addr_bit(i2c_transfer->channel);
	}
	else if (status_reg && I2C_SR1_RXNE_Msk && i2c_transfer->data_length > 3)
	{
		*i2c_transfer->buffer = (uint8_t) *I2C_DR[i2c_transfer->channel];
		i2c_transfer->buffer++;
		i2c_transfer->data_length--;
	}
	else if ((status_reg & I2C_SR1_BTF_Msk) && i2c_transfer->data_length > 3)
	{
		*i2c_transfer->buffer = (uint8_t) *I2C_DR[i2c_transfer->channel];
		i2c_transfer->buffer++;
		i2c_transfer->data_length--;
	}
	else if (i2c_transfer->data_length == 3)
	{
		*I2C_CR2[i2c_transfer->channel] &= ~(I2C_CR2_ITBUFEN_Msk | I2C_CR2_ITEVTEN_Msk);
		do
		{
			status_reg = *I2C_SR1[i2c_transfer->channel] & I2C_SR1_BTF_Msk;
		} while(status_reg == 0);

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
		*I2C_CR2[i2c_transfer->channel] &= ~(I2C_CR2_ITEVTEN_Msk | I2C_CR2_ITBUFEN_Msk);
	}
}

static void i2c_slave_transmit_it_callback(i2c_transfer_t *i2c_transfer)
{
	uint32_t status_reg = *I2C_SR1[i2c_transfer->channel];
	if (status_reg & I2C_SR1_ADDR_Msk)
	{
		i2c_clear_addr_bit(i2c_transfer->channel);
	}
	else if ((status_reg & I2C_SR1_TXE_Msk) && i2c_transfer->data_length > 0)
	{
		*I2C_DR[i2c_transfer->channel] = *i2c_transfer->buffer;
		i2c_transfer->buffer++;
		i2c_transfer->data_length--;
	}
	else if ((status_reg & I2C_SR1_BTF_Msk) && i2c_transfer->data_length < 0)
	{
		*I2C_DR[i2c_transfer->channel] = *i2c_transfer->buffer;
		i2c_transfer->buffer++;
		i2c_transfer->data_length--;
	}
	if (i2c_transfer->data_length == 0)
	{
		*I2C_CR2[i2c_transfer->channel] &= ~(I2C_CR2_ITEVTEN_Msk | I2C_CR2_ITBUFEN_Msk);
		*I2C_SR1[i2c_transfer->channel] &= ~I2C_SR1_AF_Msk;
		*I2C_CR1[i2c_transfer->channel] &= ~I2C_CR1_ACK_Msk;
	}
}

static void i2c_slave_receive_it_callback(i2c_transfer_t *i2c_transfer)
{
  uint32_t status_reg = *I2C_SR1[i2c_transfer->channel];
  if (status_reg & I2C_SR1_ADDR_Msk)
  {
	  i2c_clear_addr_bit(i2c_transfer->channel);
  }
  else if ((status_reg & I2C_SR1_RXNE_Msk) && i2c_transfer->data_length > 0)
  {
	  *i2c_transfer->buffer = (uint8_t) *I2C_DR[i2c_transfer->channel];
	  i2c_transfer->buffer++;
	  i2c_transfer->data_length--;
  }
  else if ((status_reg & I2C_SR1_BTF_Msk) && i2c_transfer->data_length > 0)
  {
	  *i2c_transfer->buffer = (uint8_t) *I2C_DR[i2c_transfer->channel];
	  i2c_transfer->buffer++;
	  i2c_transfer->data_length--;
  }
  else if (status_reg & I2C_SR1_STOPF_Msk)
  {
	  *I2C_CR2[i2c_transfer->channel] &= ~(I2C_CR2_ITEVTEN_Msk | I2C_CR2_ITBUFEN_Msk);
	  i2c_clear_stopf_bit(i2c_transfer->channel);
	  *I2C_CR1[i2c_transfer->channel] &= ~I2C_CR1_ACK_Msk;
  }
}



void i2c_master_transmit_it(i2c_transfer_t *i2c_transfer)
{
	i2c_interrupt_transfers[i2c_transfer->channel] = *i2c_transfer;
	i2c_interrupt_callbacks[i2c_transfer->channel] = i2c_master_transmit_it_callback;
	*I2C_CR2[i2c_transfer->channel] |= (I2C_CR2_ITEVTEN_Msk | I2C_CR2_ITBUFEN_Msk);
	*I2C_CR1[i2c_transfer->channel] |= I2C_CR1_START_Msk;
	volatile uint32_t status_reg = *I2C_SR1[i2c_transfer->channel];
	*I2C_DR[i2c_transfer->channel] = i2c_transfer->slave_address;
}

void i2c_master_receive_it(i2c_transfer_t *i2c_transfer)
{
	i2c_interrupt_transfers[i2c_transfer->channel] = *i2c_transfer;
	i2c_interrupt_callbacks[i2c_transfer->channel] = i2c_master_receive_it_callback;
	*I2C_CR2[i2c_transfer->channel] |= (I2C_CR2_ITEVTEN_Msk | I2C_CR2_ITBUFEN_Msk);
	*I2C_CR1[i2c_transfer->channel] |= I2C_CR1_START_Msk;
	volatile uint32_t status_reg = *I2C_SR1[i2c_transfer->channel];
	*I2C_DR[i2c_transfer->channel] = i2c_transfer->slave_address | 0x01;
}

void i2c_slave_transmit_it(i2c_transfer_t *i2c_transfer)
{
	i2c_interrupt_transfers[i2c_transfer->channel] = *i2c_transfer;
	i2c_interrupt_callbacks[i2c_transfer->channel] = i2c_slave_transmit_it_callback;
	*I2C_CR2[i2c_transfer->channel] |= (I2C_CR2_ITEVTEN_Msk | I2C_CR2_ITBUFEN_Msk);
}

void i2c_slave_receive_it(i2c_transfer_t *i2c_transfer)
{
	i2c_interrupt_transfers[i2c_transfer->channel] = *i2c_transfer;
	i2c_interrupt_callbacks[i2c_transfer->channel] = i2c_slave_receive_it_callback;
	*I2C_CR2[i2c_transfer->channel] |= (I2C_CR2_ITEVTEN_Msk | I2C_CR2_ITBUFEN_Msk);
}

static void i2c_clear_addr_bit(i2c_channel_t channel)
{
	volatile uint32_t status_reg = *I2C_SR1[channel];
	status_reg = *I2C_SR2[channel];
}

static void i2c_clear_stopf_bit(i2c_channel_t channel)
{
	volatile uint32_t status_reg = *I2C_SR1[channel];
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
  	  status_reg = *I2C_SR1[i2c_transfer->channel] & I2C_SR1_BTF_Msk;
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

void i2c_irq_handler(i2c_channel_t channel)
{
	i2c_transfer_t *transfer = &i2c_interrupt_transfers[channel];
	if (i2c_interrupt_callbacks[channel] != 0)
	  i2c_interrupt_callbacks[channel](transfer);

}



