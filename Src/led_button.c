/*
 * led_button.c
 *
 *  Created on: Feb 28, 2021
 *      Author: Xuanjiao Zhu
 */

#include "stm32f446xx.h"

#define HIGH		1
#define LOW			0
#define	BUTTON_SET	LOW
#define LED_SET		HIGH
/*
 * Software delay
 */
void delay()
{
	for(int i = 0; i < 100000; i++);
}

int main()
{
	GPIO_Handle_t gpio_led, gpio_btn;

	// LED2 is connected to PA5
	gpio_led.pGPIOx = GPIOA;
	gpio_led.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	gpio_led.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	gpio_led.GPIO_PinConfig.GPIO_PinOPType = GPIO_OUT_TYPE_PP;
	gpio_led.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_NO_PU_PD;
	gpio_led.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;

	GPIO_PeriClockControl(GPIOA, ENABLE);

	GPIO_Init(&gpio_led);

	// Button1 is connected to PC13
	gpio_btn.pGPIOx = GPIOC;
	gpio_btn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	gpio_btn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	gpio_btn.GPIO_PinConfig.GPIO_PinOPType = GPIO_OUT_TYPE_PP;
	gpio_btn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_NO_PU_PD;
	gpio_btn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;

	GPIO_PeriClockControl(GPIOC, ENABLE);

	GPIO_Init(&gpio_btn);


	while(1){

			// LED2 on when button1 is pressed
		if( GPIO_ReadFromInputPin(gpio_btn.pGPIOx, gpio_btn.GPIO_PinConfig.GPIO_PinNumber) == BUTTON_SET)
		{
			delay();
			GPIO_WriteToOutputPin(gpio_led.pGPIOx, gpio_led.GPIO_PinConfig.GPIO_PinNumber, GPIO_PIN_SET);
		}else
		{	// LED2 off when button1 is released
			delay();
			GPIO_WriteToOutputPin(gpio_led.pGPIOx, gpio_led.GPIO_PinConfig.GPIO_PinNumber, GPIO_PIN_RESET);
		}

	}
}

