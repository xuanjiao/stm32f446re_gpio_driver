/*
 * stm32f446xx_gpio.h
 *
 *  Created on: April 13, 2021
 *      Author: Xuanjiao Zhu
 */

#ifndef INC_STM32F446XX_I2C_H_
#define INC_STM32F446XX_I2C_H_

#include "stm32f446xx.h"
/*
 * This is a Pin configuration structure.
 * It is filled by the application, and the driver code extracts the data and applies the settings.
 */
typedef struct
{
	uint32_t I2C_SCLSpeed;
	uint8_t	 I2C_DeviceAddress;
	uint8_t	 I2C_ACKControl;
	uint16_t I2C_FMDutyCycle;
}I2C_Config_t;

/*
 *	Handle structure for I2Cx peripheral
 */

#endif /* INC_STM32F446XX_I2C_H_ */
