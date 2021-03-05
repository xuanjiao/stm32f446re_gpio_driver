/*
 * stm32f446xx_spi.c
 *
 *  Created on: März 5,2021
 *      Author: Xuanjiao Zhu
 */


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
void SPI_SendData(SPI_RegDef_t *pSPI,uint8_t *pRxffer,uint32_t Len);


/*
 * IRQ (Interrupt Request) Configuration and ISR (interrupt service routine) handling
 */
void SPI_IRQInterrputConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle);

