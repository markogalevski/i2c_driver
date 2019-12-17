#include <i2c_stm32f411_config.h>

static const i2c_config_t i2c_config_table[] =
{	//ENABLED		//SLAVE					//PERIPH			//DMA EN		//ERR Interrupt		//BUF Interrupt		//EVT Interrupt
					//ACKNOWLEDGE			//CLOCK FREQ
		{},
		{},
		{}
};


const i2c_config_t * i2c_config_table_get(void)
{
	return (i2c_config_table);
}
