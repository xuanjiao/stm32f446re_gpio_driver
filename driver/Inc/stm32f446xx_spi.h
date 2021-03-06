/*
 * stm32f446xx_spi.h
 *
 *  Created on: März 5,2021
 *      Author: Xuanjiao Zhu
 */

#ifndef INC_STM32F446XX_SPI_H_
#define INC_STM32F446XX_SPI_H_

#include "stm32f446xx.h"
/*
 * Configuration structure for SPIx peripheral
 */
typedef struct
{
	uint8_t SPI_DeviceMode; // Possible device modes are shown in @SPI_DeviceMode
	uint8_t	SPI_BusConfig; 	// Possible bus configurations are shown in @SPI_BusConfig
	uint8_t SPI_SclkSpeed; 	// Possible SCLK Speeds are shown in @SPI_SclkSpeed
	uint8_t SPI_DFF;		// Possible data frame formats are shown in @SPI_DFF
	uint8_t SPI_CPOL;		// Possible clock polarities are shown in  @SPI_CPOL
	uint8_t SPI_CPHA;		// Possible clock phases are shown in  @SPI_CPHA
	uint8_t SPI_SSM;		// Possible Software Slave Select settings are shown in @SPI_SSM
}SPI_Config_t;

/*
 * Handle structure for SPIx peripheral
 */
typedef struct
{
	SPI_RegDef_t *pSPIx;
	SPI_Config_t SPIConfig;
}SPI_Handle_t;

/*
 * @SPI_DeviceMode
 */
#define SPI_DEV_MODE_SLAVE		0
#define SPI_DEV_MODE_MASTER		1

/*
 * @SPI_BusConfig
 */
#define SPI_BUS_CONFIG_FD				0	// Full-duplex synchronous transfers on three lines
#define SPI_BUS_CONFIG_HD				1	// Half-duplex synchronous transfer on two lines
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY 	2	// Simplex synchronous transfers on two lines, Receive only

/*
 * @SPI_SclkSpeed
 */
#define SPI_SCLK_SPEED_DIV2				0	// Frequency = fPCLK/2
#define SPI_SCLK_SPEED_DIV4				1	// Frequency = fPCLK/4
#define SPI_SCLK_SPEED_DIV8				2	// Frequency = fPCLK/8
#define SPI_SCLK_SPEED_DIV16			3	// Frequency = fPCLK/16
#define SPI_SCLK_SPEED_DIV32			4	// Frequency = fPCLK/32
#define SPI_SCLK_SPEED_DIV64			5	// Frequency = fPCLK/64
#define SPI_SCLK_SPEED_DIV128			6	// Frequency = fPCLK/128
#define SPI_SCLK_SPEED_DIV256			7	// Frequency = fPCLK/256

/*
 * @SPI_DFF
 */
#define SPI_DFF_8BITS		0		// 8-bit data frame format is selected for transmission/reception
#define SPI_DFF_16BITS		1		// 16-bit data frame format is selected for transmission/reception

/*
 * @SPI_CPOL
 */
#define SPI_CPOL_LOW			0	// CK to 0 when idle
#define SPI_SPOL_HIGH			1	// CK to 1 when idle

/*
 * @SPI_CPHA
 */
#define SPI_CPHA_LOW			0	// The first clock transition is the first data capture edge
#define SPI_CPHA_HIGH			1	// The second clock transition is the first data capture edge

/*
 * @SPI_SSM
 */
#define SPI_SSM_DI		0
#define SPI_SSM_EN		1

/*
 * SPI related status flags definitions
 */
#define SPI_TXE_FLAG    ( 1 << SPI_SR_TXE)
#define SPI_RXNE_FLAG   ( 1 << SPI_SR_RXNE)
#define SPI_BUSY_FLAG   ( 1 << SPI_SR_BSY)

/**********************************************************************************************
 *								 APIs supported by this driver
 **********************************************************************************************/

/*
 * Peripheral clock configuration
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

/*
 * Init and De-Init
 */
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

/*
 * Data read and write
 */
void SPI_ReceiveData(SPI_RegDef_t *pSPI,uint8_t *pRxffer,uint32_t Len);
void SPI_SendData(SPI_RegDef_t *pSPI,uint8_t *pTxffer,uint32_t Len);

/*
 * IRQ (Interrupt Request) Configuration and ISR (interrupt service routine) handling
 */
void SPI_IRQInterrputConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle);

/*
 * Enable or disable SPI peripheral
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

/*
 * Configure internal slave select bit (SSI)
 */
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

/*
 * Configure the SSOE bit
 */
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

/*
 * Get SPI status
 */
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx , uint32_t FlagName);

#endif /* INC_STM32F446XX_SPI_H_ */
