/*
 * stm32f446xx_gpio.c
 *
 *  Created on: Feb 25, 2021
 *      Author: Xuanjiao Zhu
 */

#include "stm32f446xx_gpio.h"

/*********************************************************************
 * @fn      		  - GPIO_PeriClockControl
 *
 * @brief             - This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]         - base address of the GPIO peripheral
 * @param[in]         - ENABLE or DISABLE macros
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi){

	if(EnorDi == ENABLE){

		if(pGPIOx == GPIOA){
			GPIOA_PCLK_EN();
		}else if(pGPIOx == GPIOB){
			GPIOB_PCLK_EN();
		}else if(pGPIOx == GPIOC){
			GPIOC_PCLK_EN();
		}else if(pGPIOx == GPIOD){
			GPIOD_PCLK_EN();
		}else if(pGPIOx == GPIOE){
			GPIOE_PCLK_EN();
		}else if(pGPIOx == GPIOF){
			GPIOF_PCLK_EN();
		}else if(pGPIOx == GPIOG){
			GPIOG_PCLK_EN();
		}else if(pGPIOx == GPIOH){
			GPIOH_PCLK_EN();
		}
	}else{

		if(pGPIOx == GPIOA){
			GPIOA_PCLK_DI();
		}else if(pGPIOx == GPIOB){
			GPIOB_PCLK_DI();
		}else if(pGPIOx == GPIOC){
			GPIOC_PCLK_DI();
		}else if(pGPIOx == GPIOD){
			GPIOD_PCLK_DI();
		}else if(pGPIOx == GPIOE){
			GPIOE_PCLK_DI();
		}else if(pGPIOx == GPIOF){
			GPIOF_PCLK_DI();
		}else if(pGPIOx == GPIOG){
			GPIOG_PCLK_DI();
		}else if(pGPIOx == GPIOH){
			GPIOH_PCLK_DI();
		}
	}
}



/*********************************************************************
 * @fn      		  - GPIO_Init
 *
 * @brief             - This function initialize the GPIO driver
 *
 * @param[in]         - pointer to a GPIO handle structure
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle){

	uint32_t temp = 0; // Store the configuration data

	// Configure the pin mode
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode < GPIO_MODE_ANALOG){
		temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		pGPIOHandle->pGPIOx->MODE &= ~( 0x3 << 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // Clear the pin x configuration bit
		pGPIOHandle->pGPIOx->MODE |= temp;	// Setting
	}else{

	}

	// Configure the pin speed
	temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OSPEEDER &= ~( 0x3 << 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed); // Clear the pin x configuration bit
	pGPIOHandle->pGPIOx->OSPEEDER |= temp;	// Setting

	// Configure the pin pull-down/pull-up resistor
	temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->PUPDR &= ~( 0x3 << 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // Clear the pin x configuration bit
	pGPIOHandle->pGPIOx->PUPDR |= temp;		// Setting

	// Configure the pin output type
	temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER &= ~( 0x1 << 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // Clear the pin x configuration bit
	pGPIOHandle->pGPIOx->OTYPER |= temp;	// Setting

	// Configure the pin alternate function selection when the pin mode is alternate function mode.
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		uint8_t temp1, temp2;
		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;		// Select from alternate function low register or alternate function high register
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8; 	// Select the pin x configuration bits
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4 * temp2 ));		// Clear the pin x configuration bit
		temp  = pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << ( 4 * temp2 );
		pGPIOHandle->pGPIOx->PUPDR |= temp;		// Setting
	}

}

/*********************************************************************
 * @fn      		  - GPIO_DeInit
 *
 * @brief             - This function de-initialize the GPIO driver
 *
 * @param[in]         - base address of the GPIO peripheral
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx); 	/* Use reset register to reset the GPIO port */


/*********************************************************************
 * @fn      		  - GPIO_ReadFromInputPin
 *
 * @brief             - This function read the state of a GPIO pin when it is in input mode
 *
 * @param[in]         - base address of the GPIO peripheral
 * @param[in]         - the number of given pin
 *
 * @return            -  the state of given GPIO pin
 *
 * @Note              -  none
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/*********************************************************************
 * @fn      		  - GPIO_ReadFromInputPort
 *
 * @brief             - This function read the state of a GPIO port
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);

/*********************************************************************
 * @fn      		  -
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void GPIO_WriteToInputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber, uint8_t Value);

/*********************************************************************
 * @fn      		  -
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void GPIO_WriteToInputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);

/*********************************************************************
 * @fn      		  -
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/*
 * IRQ (Interrupt Request) Configuration and ISR (interrupt service routine) handling
 */

/*********************************************************************
 * @fn      		  -
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi);

/*********************************************************************
 * @fn      		  -
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void GPIO_IRQHandling(uint8_t PinNumber);


