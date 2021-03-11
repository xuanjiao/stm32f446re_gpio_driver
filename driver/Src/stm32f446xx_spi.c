/*
 * stm32f446xx_spi.c
 *
 *  Created on: März 5,2021
 *      Author: Xuanjiao Zhu
 */

#include "stm32f446xx_spi.h"

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_rene_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle);

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
	// Enable the SPI Clock
	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

	// Set SPI control register
	uint32_t tempreg  = 0;

	// Configure the device mode
	tempreg |= ( pSPIHandle->SPI_Config.SPI_DeviceMode << SPI_CR1_MSTR);

	// Configure the SPI bus
	if( pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		tempreg &= ~( 1 << SPI_CR1_BIDIMODE ); // Bidirectional data mode disable
	}
	else if( pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		tempreg |= ( 1 << SPI_CR1_BIDIMODE ); // Bidirectional data mode enable
	}
	else if( pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{
		tempreg &= ~( 1 << SPI_CR1_BIDIMODE ); // Bidirectional data mode disable
		tempreg |= ~( 1 << SPI_CR1_RXONLY ); // Receive only mode enable
	}

	// Configure the data frame format
	tempreg |= (pSPIHandle->SPI_Config.SPI_DFF << SPI_CR1_DFF);

	// Configure the software slave management
	tempreg |= (pSPIHandle->SPI_Config.SPI_SSM << SPI_CR1_SSM);

	// Configure the Serial clock speed (baud rate)
	tempreg |= (pSPIHandle->SPI_Config.SPI_SclkSpeed << SPI_CR1_BR);

	// Configure the CPOL
	tempreg |= (pSPIHandle->SPI_Config.SPI_CPOL << SPI_CR1_CPOL);

	// Configure the CPHA
	tempreg |= (pSPIHandle->SPI_Config.SPI_CPHA << SPI_CR1_CPHA);

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
void SPI_ReceiveData(SPI_RegDef_t *pSPIx,uint8_t *pRxBuffer,uint32_t Len)
{
	while(Len > 0){
		while (SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET); // Wait until RX buffer not empty

		if ( pSPIx->CR1 & ( 1 << SPI_CR1_DFF))
		{
			// Frame format 16 bit
			*((uint16_t*)pRxBuffer) = pSPIx->DR; // Load the data from DR to RxBuffer
			Len -= 2;
			(uint16_t*)pRxBuffer++;
		}else
		{
			// Frame format 8 bit
			*pRxBuffer = pSPIx->DR;
			Len--;
			pRxBuffer++;
		}
	}

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
void SPI_SendData(SPI_RegDef_t *pSPIx,uint8_t *pTxBuffer,uint32_t Len)
{
	while(Len > 0)
	{
		while( SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET ); // Wait until the Tx buffer is empty.

		if (pSPIx->CR1 & (1 << SPI_CR1_DFF))
		{
			// 16 bit
			pSPIx->DR = *((uint16_t*)pTxBuffer);
			Len -= 2;
			(uint16_t*)pTxBuffer++;
		}else
		{
			// 8 bit
			pSPIx->DR = *pTxBuffer;
			Len--;
			pTxBuffer++;
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
	if( EnorDi == ENABLE)
	{
		if ( IRQNumber < 32 )
		{
			*NVIC_ISER0 |= ( 1 << IRQNumber);

		}else if ( IRQNumber >= 32 && IRQNumber < 64 )
		{
			*NVIC_ISER0 |= ( 1 << IRQNumber % 32 );

		}else if ( IRQNumber >= 64 && IRQNumber < 96 )
		{
			*NVIC_ISER0 |= ( 1 << IRQNumber % 64 );
		}
	}else
	{
		if( IRQNumber < 32)
		{
			*NVIC_ICER0 |= ( 1 << IRQNumber);

		}else if (IRQNumber >= 32 && IRQNumber < 64)
		{
			*NVIC_ICER1 |= ( 1 << IRQNumber % 32);

		}else if (IRQNumber >= 64 && IRQNumber < 96)
		{
			*NVIC_ICER2 |= ( 1 << IRQNumber % 64);

		}else{}
	}
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
	uint8_t temp1, temp2;
	temp1 = IRQNumber / 4;
	temp2 = IRQNumber % 4;
	*(NVIC_IPR_BASEADDR + temp1) =  IRQPriority << (temp2 + 8 - NO_PR_BITS_IMPLEMENTED);
}

/*********************************************************************
 * @fn      		  - SPI_SendDataIT
 *
 * @brief             - This function enable the interrupt for TXE event
 *
 * @param[in]         - base address of the SPI
 * @param[in]         - the pointer to the TX buffer
 * @param[in]         - the length of data to send
 *
 * @return            - the TX state
 *
 * @Note              - none
 */
uint8_t SPI_SendDataIT(SPI_Handle_t* pSPIHandle, uint8_t *pTxBuffer,uint32_t Len)
{
	// Make sure the TX is not busy
	uint8_t txState = pSPIHandle->TX_State;
	if(txState != SPI_TX_STATE_BUSY)
	{
		// Store TX buffer address in global variable
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxBufferLen = Len;

		// Set state to TX busy
		pSPIHandle->TX_State = SPI_TX_STATE_BUSY;

		// Enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR register
		pSPIHandle->pSPIx->CR2 |= ( 1 << SPI_CR2_TXEIE );
	}

	return txState;
}

/*********************************************************************
 * @fn      		  - SPI_ReceiveDataIT
 *
 * @brief             - This function enable the interrupt for RXNE event
 *
 * @param[in]         - base address of the SPI
 * @param[in]         - the pointer to the RX buffer
 * @param[in]         - the length of data to receive
 *
 * @return            - the RX state
 *
 * @Note              - none
 */
uint8_t SPI_ReceiveDataIT(SPI_Handle_t* pSPIHandle, uint8_t *pRxBuffer,uint32_t Len)
{
	uint8_t rxState = pSPIHandle->TX_State;

	if(rxState != SPI_RX_STATE_BUSY)
	{
		// Store RX buffer address in global variable
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxBufferLen = Len;

		// Set state to RX busy
		pSPIHandle->TX_State = SPI_RX_STATE_BUSY;

		// Enable the RXNE control bit to get interrupt whenever RXNE flag is set in SR register
		pSPIHandle->pSPIx->CR2 |= ( 1 << SPI_CR2_RXNEIE);
	}

	return rxState;
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
	uint8_t temp1, temp2;

	// Check whether transmit Tx buffer ready to be loaded
	temp1 = pSPIHandle->pSPIx->CR2 | ( 1 << SPI_CR2_TXEIE);
	temp2 = pSPIHandle->pSPIx->SR | ( 1 << SPI_SR_TXE);

	if (temp1 && temp2)
	{
		spi_txe_interrupt_handle(pSPIHandle);
	}

	// Check whether data received in Rx buffer
	temp1 = pSPIHandle->pSPIx->CR2 | ( 1 << SPI_CR2_RXNEIE);
	temp2 = pSPIHandle->pSPIx->SR | ( 1 << SPI_SR_RXNE);

	if (temp1 && temp2)
	{
		spi_rene_interrupt_handle(pSPIHandle);
	}

	// Check Overrun error
	temp1 = pSPIHandle->pSPIx->CR2 | ( 1 << SPI_CR2_ERRIE);
	temp2 = pSPIHandle->pSPIx->SR | ( 1 << SPI_SR_OVR);

	if (temp1 && temp2)
	{
		spi_ovr_err_interrupt_handle(pSPIHandle);
	}
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
 * @fn      		  - SPI_CloseTransmission
 *
 * @brief             - This function clear the TX buffer and disable the interrupt for TXE event
 *
 * @param[in]         - the pointer to the SPI handle
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->TxBufferLen = 0;
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TX_State = SPI_READY;

	// Disable the interrupt for TXE event
	pSPIHandle->pSPIx->CR2 &= ~( 1 << SPI_CR2_TXEIE );
}

/*********************************************************************
 * @fn      		  - SPI_CloseTransmission
 *
 * @brief             - This function clear the RX buffer and disable the interrupt for RXNE event
 *
 * @param[in]         - the pointer to the SPI handle
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void SPI_CloseReception(SPI_Handle_t *pSPIHandle){
	pSPIHandle->RxBufferLen = 0;
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RX_State = SPI_READY;

	pSPIHandle->pSPIx->CR2 &= ~( 1 << SPI_CR2_RXNEIE );
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

/*********************************************************************
 * @fn      		  - SPI_GetFlagStatus
 *
 * @brief             - This function get the SPI status in SR register
 *
 * @param[in]         - the pin number
 *
 * @return            -  none
 *
 * @Note              -  none
 */
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx , uint32_t FlagName)
{
	if(pSPIx->SR & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}

/*
 * Some helper function
 */
void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	if (pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF))
	{
		// 16 bit
		pSPIHandle->pSPIx->DR = *((uint16_t*)pSPIHandle->pTxBuffer);
		pSPIHandle->TxBufferLen -= 2;
		(uint16_t*)pSPIHandle->pTxBuffer++;
	}else
	{
		// 8 bit
		pSPIHandle->pSPIx->DR = *(pSPIHandle->pTxBuffer);
		pSPIHandle->TxBufferLen--;
		pSPIHandle->pTxBuffer++;
	}

	// When all data is sent, end TX transmission
	if(pSPIHandle->TxBufferLen <= 0)
	{
		SPI_CloseTransmission(pSPIHandle);

		// Inform the application
		SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_TX_COMPLETE);
	}
}

void spi_rene_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	if ( pSPIHandle->pSPIx->CR1 & ( 1 << SPI_CR1_DFF))
	{
		// Frame format 16 bit
		*((uint16_t*)pSPIHandle->pRxBuffer) = pSPIHandle->pSPIx->DR; // Load the data from DR to RxBuffer
		pSPIHandle->RxBufferLen -= 2;
		(uint16_t*)pSPIHandle->pRxBuffer++;
	}else
	{
		// Frame format 8 bit
		*(pSPIHandle->pRxBuffer) = pSPIHandle->pSPIx->DR;
		pSPIHandle->RxBufferLen--;
		pSPIHandle->pRxBuffer++;
	}

	// When all data is received, end RX transmission
	if(pSPIHandle->RxBufferLen <= 0)
	{
		SPI_CloseReception(pSPIHandle);

		// Inform the application
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_COMPLETE);
	}
}

static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	uint8_t temp;

	if(pSPIHandle->TX_State != SPI_TX_STATE_BUSY)
	{
		// Clearing the OVR bit is done by a read access to the SPI_DR register followed by a read access to the SPI_SR register.
		temp = pSPIHandle->pSPIx->DR;
		temp = pSPIHandle->pSPIx->SR;
	}
	(void)temp;

	// Inform the application
	SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_OVR_ERR);
}

__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle,uint8_t AppEv){
	// User should overwrite this function
}
