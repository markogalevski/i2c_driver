/*******************************************************************************
* Title                 :   I2C Interface
* Filename              :   i2c_interface.h
* Author                :   Marko Galevski
* Origin Date           :   20/01/2020
* Version               :   1.0.0
* Compiler              :   None
* Target                :   None
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

/** @file i2c_interface.h
 *  @brief General interface covering user accesses to the i2c communication
 *  	bus.
 */

#ifndef _I2C_H
#define _I2C_H
#include "i2c_stm32f411_config.h"
#include <stdint.h>

/**
 * Generic transfer structure, independent of implementation. Passed into transmission
 * 	functions.
 */
typedef struct
{
  i2c_channel_t channel; 	/**<The target I2C peripheral*/
  uint8_t *buffer;			/**<The data buffer */
  uint32_t data_length;		/**<The number of bytes to be receive/sent*/
  uint8_t slave_address;	/**<The 7-bit slave address */
} i2c_transfer_t;

typedef enum
{
	INTERRUPT_DISABLED,
	INTERRUPT_ENABLED
} i2c_interrupt_control_t;

void i2c_init(const i2c_config_t *config_table);
void i2c_irq_handler(i2c_channel_t channel);
void i2c_interrupt_control(i2c_channel_t channel, i2c_interrupt_dma_t interrupt, i2c_interrupt_control_t signal);

void i2c_master_transmit(i2c_transfer_t *transfer);
void i2c_master_receive(i2c_transfer_t *transfer);
void i2c_slave_transmit(i2c_transfer_t *transfer);
void i2c_slave_receive(i2c_transfer_t *transfer);
void i2c_master_transmit_it(i2c_transfer_t *transfer);
void i2c_master_receive_it(i2c_transfer_t *transfer);
void i2c_slave_transmit_it(i2c_transfer_t *transfer);
void i2c_slave_receive_it(i2c_transfer_t *transfer);

void i2c_register_write(uint32_t i2c_register);
uint32_t i2c_register_read(uint32_t i2c_regsister);



#endif /* define _I2C_H */
