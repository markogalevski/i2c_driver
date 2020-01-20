/*******************************************************************************
* Title                 :   I2C Config Header for STM32F411
* Filename              :   i2c_stm32f411_config.h
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

/** @file i2c_stm32f411_config.h
 *  @brief Contains the definitions and structures required to configure the
 *  		i2c peripherals on an stm32f411
 */
#ifndef _I2C_STM32F411_CFG
#define _I2C_STM32F411_CFG

#include <stdint.h>

#ifndef DISABLED
#define DISABLED 0
#endif

#ifndef ENABLED
#define ENABLED 1
#endif

/**
 * Options for enabling or disabling an I2C channel
 */
typedef enum
{
	I2C_DISABLED,
	I2C_ENABLED
}i2c_enabled_t;

/**
 * Options which decided whether the I2C returns an ACK pulse upon data
 * reception or address match
 */
typedef enum
{
	I2C_ACK_DISABLED,
	I2C_ACK_ENABLED
} i2c_ack_en_t;

/**
 * Lists all the possible interrupts (and the dma request mode) available to the I2C channel
 */
typedef enum
{
	IT_ERR, /**<The I2C raises an interrupt upon an error flag being raised*/
	IT_EVT, /**<The I2C raises an interrupt upon events: Start Bit, Address Matching, STOPF, BTF*/
	IT_BUF, /**<The I2C raises an interrupt when TxE or RxNE = 1 (if IT_EVT is also enabled) */
	DMA_REQ	/**<Th2 I2C issues a DMA request upon TxE or TxNE = 1 */
}i2c_interrupt_dma_t;

/**
 * Contains all of the I2C devices on chip
 */
typedef enum
{
	I2C_1 = 0x00UL,
	I2C_2 = 0x01UL,
	I2C_3 = 0x02UL,
	NUM_I2C
} i2c_channel_t;


/**
 * Decides the maximum frequency with which the i2c may work
 */
typedef enum
{
	I2C_SM = 0x00UL, /**< Up to 100kHz */
	I2C_FM = 0x01UL  /**< Up to 400kHz */
}i2c_fast_slow_t;

/**
 * Determines the ratio of low to high periods per I2C pulse
 */
typedef enum
{
	FM_MODE_2 = 0x00UL, 	/**<T_low/T_high = 2 */
	FM_MODE_16_9 = 0x01UL 	/**<T_low/T_high = 16/9 */
}i2c_fm_duty_cycle_t;

/**
 * Struct contains the settings required to configure an i2c device.
 */
typedef struct
{
  i2c_enabled_t en; 				/**<Whether the device is enabled or not*/
  i2c_ack_en_t ack_en;				/**<Whether the device sends ACK upon byte reception*/
  uint32_t periph_clk_freq_MHz;		/**<The frequency of the device in MHz*/
  uint32_t i2c_op_freq_kHz;			/**<The operational frequency of the I2C bus*/
  i2c_fast_slow_t fast_or_std;		/**<Whether the I2C device will be in fast or standard mode*/
  i2c_fm_duty_cycle_t duty_cycle; 	/**<The ratio of the period of low vs high cycles of bit pulses */
} i2c_config_t;

const i2c_config_t *i2c_config_get(void);

#endif
