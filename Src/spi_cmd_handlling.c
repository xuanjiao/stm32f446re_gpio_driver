/*
 * spi_cmd_handlling.c
 *
 *  Created on: Mar 7, 2021
 *      Author: Xuanjiao Zhu
 */

#include "stm32f446xx.h"
#include <string.h>

#define HIGH		1
#define LOW			0
#define	BUTTON_SET	LOW

/*
 * Software delay
 */
void delay()
{
	for(int i = 0; i < 100000; i++);
}

/*
 *
 * PB13 --> SPI1_SCLK
 * PB14 --> SPI1_MISO
 * PB15 --> SPI1_MOSI
 * Alternate function mode: 5
 */
void SPI2_GPIO_Init()
{
	GPIO_Handle_t SPIPins;

	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OUT_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_NO_PU_PD;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;

	// SCLK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(&SPIPins);

	// MISO
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	GPIO_Init(&SPIPins);

	// MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(&SPIPins);

	//NSS
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIO_Init(&SPIPins);
}

void SPI2_Init(void)
{
	SPI_Handle_t SPIHandle;
	SPIHandle.pSPIx = SPI2;
	SPIHandle.SPI_Config.SPI_DeviceMode = SPI_DEV_MODE_MASTER;
	SPIHandle.SPI_Config.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPIHandle.SPI_Config.SPI_DFF = SPI_DFF_8BITS;
	SPIHandle.SPI_Config.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV8;
	SPIHandle.SPI_Config.SPI_CPHA = SPI_CPHA_LOW;
	SPIHandle.SPI_Config.SPI_CPOL = SPI_CPOL_LOW;
	SPIHandle.SPI_Config.SPI_SSM = SPI_SSM_DI;

	SPI_Init(&SPIHandle);

}
void Btn_GPIO_Init()
{
	// Button1 is connected to PC13

	GPIO_Handle_t GPIOBtn;
	GPIOBtn.pGPIOx = GPIOC;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GPIOBtn.GPIO_PinConfig.GPIO_PinOPType = GPIO_OUT_TYPE_PP;
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_NO_PU_PD;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;

	GPIO_PeriClockControl(GPIOC, ENABLE);

	GPIO_Init(&GPIOBtn);

}
int main(){

	char TxBuffer[] = "Hello World";
	uint8_t dataLen = strlen(TxBuffer);
	// Initialize the button
	Btn_GPIO_Init();

	// Configure GPIO port as SPI mode
	SPI2_GPIO_Init();

	// Initialize SPI2 parameters
	SPI2_Init();


	// When SPE = 1, NSS will be pulled to low, and NSS pin will be high when SPE = 0
	SPI_SSOEConfig(SPI2, ENABLE);

	while(1)
	{

		// Wait until button is pressed
		while (GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13) != BUTTON_SET);

		// Avoid button de-bouncing
		delay();

		// After configuration, Enable SPI2
		SPI_PeripheralControl(SPI2,ENABLE);

		SPI_SendData(SPI2,&dataLen,1);

		// Wait until not busy
		while( SPI_GetFlagStatus(SPI2,SPI_BUSY_FLAG) == FLAG_SET);
		// Disable SPI2
		SPI_PeripheralControl(SPI2,DISABLE);
	}




}





