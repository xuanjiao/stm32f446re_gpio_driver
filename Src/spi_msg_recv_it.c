/*
 * spi_msg_recv_it.c
 *
 *  Created on: Mar 8, 2021
 *      Author: Xuanjiao Zhu
 */

#include "stm32f446xx.h"

SPI_Handle_t SPIHandle;

volatile uint8_t rcvStop = 0;

volatile uint8_t dataAvailable = 0;

#define MAX_LEN 500
char RcvBuff[MAX_LEN];

volatile char rcvByte;




/*
 * The slave notify the master when the data is available
 * PD6 --> Arduino P8
 */
void Slave_Notify_PIN_GPIO_Init()
{
	GPIO_Handle_t ITPin;

	ITPin.pGPIOx = GPIOD;
	ITPin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6; // PD6
	ITPin.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT; // Interrupt generate at falling edge
	ITPin.GPIO_PinConfig.GPIO_PinOPType = GPIO_OUT_TYPE_PP;
	ITPin.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_NO_PU_PD;
	ITPin.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	GPIO_Init(&ITPin);
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

void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle,uint8_t AppEv)
{
	if (AppEv == SPI_EVENT_RX_COMPLETE)
	{
		rcvStop = 1;
	}
}


int main()
{
	uint8_t dummySent;

	// Initialize GPIO pin to SPI mode
	SPI2_GPIO_Init();

	// Initialize SPI parameters
	SPI2_Init();

	// Initialize the notification pin
	Slave_Notify_PIN_GPIO_Init();

	/*
	 * The NSS signal is driven low as soon as the SPI is enabled in master mode (SPE=1),
	 * and is kept
	 * low until the SPI is disabled (SPE=0).
	 */
	SPI_SSOEConfig(SPI2,ENABLE);

	while(1)
	{
		// Enable interrupt from slave
		GPIO_IRQInterrputConfig(IRQ_NO_EXTI9_5, ENABLE);

		// Wait for Slave interrupt
		while(!dataAvailable);

		// Disable interrupt from slave
		GPIO_IRQInterrputConfig(IRQ_NO_EXTI9_5, DISABLE);

		// Enable SPI peripheral
		SPI_PeripheralControl(SPI2, ENABLE);

		// Receive one message
		while(!rcvStop)
		{
			// Fetch data from SPI peripheral in interrupt mode
			while (SPI_SendDataIT(&SPIHandle, &dummySent, 1) == SPI_TX_STATE_BUSY);
			while (SPI_ReceiveDataIT(&SPIHandle, (uint8_t*)&rcvByte, 1) == SPI_RX_STATE_BUSY);
		}

		// Disable SPI peripheral
		SPI_PeripheralControl(SPI2, DISABLE);

	}



}
