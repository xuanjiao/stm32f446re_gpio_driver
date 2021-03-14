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

void Slave_Notify_PIN_GPIO_Init(void);
void SPI2_GPIO_Init(void);
void SPI2_Init(void);
void SPI2_IRQHandler (void);
void EXTI9_5_IRQHandler(void);
void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle,uint8_t AppEv);

int main(void)
{
	printf("Main function start..\n");
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
		rcvStop = 0;

		// Enable interrupt on processor perspective
		GPIO_IRQInterrputConfig(IRQ_NO_EXTI9_5, ENABLE);

		// Wait for Slave interrupt
		while(!dataAvailable);

		// Disable interrupt from slave
		GPIO_IRQInterrputConfig(IRQ_NO_EXTI9_5, DISABLE);

		// Enable SPI peripheral
		SPI_PeripheralControl(SPI2, ENABLE);
		SPI_IRQInterrputConfig(IRQ_NO_SPI2, ENABLE);

		// Receive data byte by byte until reach the string termination
		while(!rcvStop)
		{
			/*
			 * Send one byte in interrupt mode
			 * When some transmission is going on,  SPI_TX_STATE_BUSY will be return,  try again
			 */
			while (SPI_SendDataIT(&SPIHandle, &dummySent, 1) == SPI_TX_STATE_BUSY);

			// Receive data in interrupt mode
			while (SPI_ReceiveDataIT(&SPIHandle, (uint8_t*)&rcvByte, 1) == SPI_RX_STATE_BUSY);
		}

		// Disable SPI peripheral
		SPI_PeripheralControl(SPI2, DISABLE);

		printf("Received data: %s\n",RcvBuff);
		dataAvailable = 0;
	}
}

/*
 * The slave notify the master when the data is available
 * PC6 --> Arduino P8
 */
void Slave_Notify_PIN_GPIO_Init(void)
{
	GPIO_Handle_t ITPin;

	ITPin.pGPIOx = GPIOC;
	ITPin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6; // PC6
	ITPin.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT; // Interrupt generate at falling edge
	ITPin.GPIO_PinConfig.GPIO_PinOPType = GPIO_OUT_TYPE_PP;
	ITPin.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_NO_PU_PD;
	ITPin.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	GPIO_Init(&ITPin);

	GPIO_IRQPriorityConfig(IRQ_NO_EXTI9_5,NVIC_IRQ_PRI15);
	GPIO_IRQInterrputConfig(IRQ_NO_EXTI9_5,ENABLE);


}


/*
 * Initialize pin to SPI mode
 * PB13 --> SPI2_SCLK
 * PB14 --> SPI2_MISO
 * PB15 --> SPI2_MOSI
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

/*
 * Initialize SPI2 peripheral
 */
void SPI2_Init(void)
{
	SPIHandle.pSPIx = SPI2;
	SPIHandle.SPI_Config.SPI_DeviceMode = SPI_DEV_MODE_MASTER;
	SPIHandle.SPI_Config.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPIHandle.SPI_Config.SPI_DFF = SPI_DFF_8BITS;
	SPIHandle.SPI_Config.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV16;
	SPIHandle.SPI_Config.SPI_CPHA = SPI_CPHA_LOW;
	SPIHandle.SPI_Config.SPI_CPOL = SPI_CPOL_LOW;
	SPIHandle.SPI_Config.SPI_SSM = SPI_SSM_DI;

	SPI_Init(&SPIHandle);
	SPI_IRQInterrputConfig(IRQ_NO_SPI2, ENABLE);
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI9_5,NVIC_IRQ_PRI14);

	// SPI_PeripheralControl(SPI2, ENABLE);
}

/*
 * Runs when receive interrupt from the slave
 */
void EXTI9_5_IRQHandler(void)
{
	dataAvailable = 1;
	GPIO_IRQHandling(GPIO_PIN_NO_6);
}


/*
 *  SPI2 global interrupt.
 */
void SPI2_IRQHandler (void)
{
	SPI_IRQHandling(&SPIHandle);
}

/*
 * Runs when receive a event from driver
 */
void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle,uint8_t AppEv)
{
	static uint32_t i = 0;

	// One byte is received
	if (AppEv == SPI_EVENT_RX_COMPLETE)
	{
		RcvBuff[i]=rcvByte;
		i++;
		if (rcvByte == '\0' || i > MAX_LEN ) // Check whether it is the string termination or reach the maximal length
		{
			rcvStop = 1;
			i = 0;
		}
	}
}


