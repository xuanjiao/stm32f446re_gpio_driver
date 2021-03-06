/*
 * stm32f446xx_spi.c
 *
 *  Created on: März 5,2021
 *      Author: Xuanjiao Zhu
 */

#include "stm32f446xx_spi.h"

/*********************************************************************
 * @fn      		  - GPIO_PeriClockControl
 *
 * @brief             - This function enables or disables peripheral clock for the given SPI peripheral
 *
 * @param[in]         - base address of the SPI peripheral
 * @param[in]         - ENABLE or DISABLE macros
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_EN();

		}else if(pSPIx == SPI2)
		{
			SPI2_PCLK_EN();
		}
		else if(pSPIx == SPI3)
		{
			SPI3_PCLK_EN();
		}
	}else
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_DI();
		}
		else if(pSPIx == SPI2)
		{
			SPI2_PCLK_DI();
		}
		else if(pSPIx == SPI3)
		{
			SPI3_PCLK_DI();
		}
	}
}

/*********************************************************************
 * @fn      		  - SPI_Init
 *
 * @brief             - This function initialize the SPI
 *
 * @param[in]         - pointer to a SPI handle structure
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	// Enable the SPI peripheral Control
	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

	// Set SPI control register
	uint32_t tempreg  = 0;

	// Configure the device mode
	tempreg |= ( pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR);

	// Configure the SPI bus
	if( pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		tempreg &= ~( 1 << SPI_CR1_BIDIMODE ); // Bidirectional data mode disable
	}
	else if( pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		tempreg |= ( 1 << SPI_CR1_BIDIMODE ); // Bidirectional data mode enable
	}
	else if( pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{
		tempreg &= ~( 1 << SPI_CR1_BIDIMODE ); // Bidirectional data mode disable
		tempreg |= ~( 1 << SPI_CR1_RXONLY ); // Receive only mode enable
	}

	// Configure the data frame format
	tempreg |= (pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF);

	// Configure the software slave management
	tempreg |= (pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM);

	// Configure the Serial clock speed (baud rate)
	tempreg |= (pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR);

	// Configure the CPOL
	tempreg |= (pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL);

	// Configure the CPHA
	tempreg |= (pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA);

	pSPIHandle->pSPIx->CR1 = tempreg;
}

/*********************************************************************
 * @fn      		  - SPI_DeInit
 *
 * @brief             - This function use reset register to reset the SPI
 *
 * @param[in]         - base address of the SPI
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
	if(pSPIx == SPI1)
		SPI1_REG_RESET();
	else if (pSPIx == SPI2)
		SPI2_REG_RESET();
	else if (pSPIx == SPI3)
		SPI3_REG_RESET();
	else if (pSPIx == SPI4)
		SPI4_REG_RESET();
}


/*********************************************************************
 * @fn      		  - SPI_ReceiveData
 *
 * @brief             - This function receives data from SPI
 *
 * @param[in]         - base address of the SPI
 * @param[in]         - the pointer to the RX buffer
 * @param[in]         - the length of received data
 *
 * @return            - none
 *
 * @Note              - none
 */
void SPI_ReceiveData(SPI_RegDef_t *pSPIx,uint8_t *pRxffer,uint32_t Len)
{


}

/*********************************************************************
 * @fn      		  - SPI_SendData
 *
 * @brief             - This function send data to SPI
 *
 * @param[in]         - base address of the SPI
 * @param[in]         - the pointer to the TX buffer
 * @param[in]         - the length of received data
 *
 * @return            - none
 *
 * @Note              - This is blocking call
 */
void SPI_SendData(SPI_RegDef_t *pSPIx,uint8_t *pTxffer,uint32_t Len)
{
	while(Len > 0)
	{
		while( SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET ); // Wait until the Tx buffer is empty.

		if (pSPIx->CR1 & (1 << SPI_CR1_DFF))
		{
			// 16 bit
			pSPIx->DR = *((uint16_t*)pTxffer);
			Len -= 2;
			(uint16_t*)pTxffer++;
		}else
		{
			// 8 bit
			pSPIx->DR = *pTxffer;
			Len--;
			pTxffer++;
		}
	}
}


/*********************************************************************
 * @fn      		  - SPI_IRQInterrputConfig
 *
 * @brief             - This function enable the interrupt in processor's perspective
 *
 * @param[in]         -	the IRQ number
 * @param[in]         - ENABLE or DISABLE macros
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void SPI_IRQInterrputConfig(uint8_t IRQNumber, uint8_t EnorDi)
{

}

/*********************************************************************
 * @fn      		  - SPI_IRQPriorityConfig
 *
 * @brief             - This function configure the IRQ Priority
 *
 * @param[in]         - the IRQ number
 * @param[in]         - the IRQ priority
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{

}

/*********************************************************************
 * @fn      		  - SPI_IRQHandling
 *
 * @brief             - This function clear the corresponding bit in EXTI pending register
 *
 * @param[in]         - the pin number
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle)
{

}

/*********************************************************************
 * @fn      		  - SPI_PeriphralControl
 *
 * @brief             - This function enable or disable the SPI peripheral
 *
 * @param[in]         - the pin number
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if( EnorDi == ENABLE)
	{
		pSPIx->CR1 |= ( 1 << SPI_CR1_SPE);
	}else
	{
		pSPIx->CR1 &= ~( 1 << SPI_CR1_SPE);
	}
}

/*********************************************************************
 * @fn      		  - SPI_SSIConfig
 *
 * @brief             - This function enable or disable the SPI internal slave select. The value is forced onto the NSS pin
 *
 * @param[in]         - the pin number
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if ( EnorDi == ENABLE )
	{
		pSPIx->CR1 |= ( 1 << SPI_CR1_SSI);
	}else
	{
		pSPIx->CR1 &= ~( 1 << SPI_CR1_SSI);
	}
}

/*********************************************************************
 * @fn      		  - SPI_SSOEConfig
 *
 * @brief             - This function enable or disable the SSOE
 *
 * @param[in]         - the pin number
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if ( EnorDi == ENABLE )
	{
		pSPIx->CR2 |= ( 1 << SPI_CR2_SSOE);
	}else
	{
		pSPIx->CR2 &= ~( 1 << SPI_CR2_SSOE);
	}
}

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx , uint32_t FlagName)
{
	if(pSPIx->SR & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}

