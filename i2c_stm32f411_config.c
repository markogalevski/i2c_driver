/*******************************************************************************
* Title                 :   I2C Config Table for STM32F411
* Filename              :   i2c_stm32f411_config.c
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

/** @file i2c_stm32f411_config.c
 *  @brief Contains the configuration information for each I2C channel
 */
#include "i2c_stm32f411_config.h"

/**
 * The configuration table that must be filled out by the user and is used by i2c_init to initialise the
 * separate i2c channels
 */
static const i2c_config_t i2c_config_table[NUM_I2C] =
{	//ENABLED		//SLAVE					//PERIPH				//OPERATIONAL		//FAST OR 			//DUTY
					//ACKNOWLEDGE			//CLOCK FREQ			//FREQUENCY			//STANDARD			//CYCLE
		{},
		{},
		{}
};

/******************************************************************************
* Function: i2c_config_get()
*//**
* \b Description:
*
* 	Returns a pointer to the base of the configuration table for i2c peripherals
*
*
* PRE-CONDITION: The config table has been filled out and is non-null

* @return 		*i2c_config_t
*
* \b Example:
* @code
*	const i2c_config_t *config_table = i2c_config_get();
*	i2c_init(config_table);
* @endcode
*
* @see i2c_init
* <br><b> - CHANGE HISTORY - </b>
*
* <table align="left" style="width:800px">
* <tr><td> Date       </td><td> Software Version </td><td> Initials </td><td> Description </td></tr>
* </table><br><br>
* <hr>
*******************************************************************************/
const i2c_config_t *i2c_config_get(void)
{
	return (i2c_config_table);
}
