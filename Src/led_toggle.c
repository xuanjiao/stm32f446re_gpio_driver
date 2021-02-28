/*
 * led_toggle.c
 *
 *  Created on: Feb 28, 2021
 *      Author: Xuanjiao Zhu
 */

#include "stm32f446xx.h"

/*
 * Software delay
 */
void delay()
{
	for(int i = 0; i < 5000000; i++);
}
int main()
{
	GPIO_Handle_t gpio_handle;

	// LED2 is connected to PA5
	gpio_handle.pGPIOx = GPIOA;
	gpio_handle.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	gpio_handle.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	gpio_handle.GPIO_PinConfig.GPIO_PinOPType = GPIO_OUT_TYPE_PP;
	gpio_handle.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_NO_PU_PD;
	gpio_handle.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;

	GPIO_PeriClockControl(GPIOA, ENABLE);

	GPIO_Init(&gpio_handle);

	while(1){
		GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_5);
		delay();
	}
	return 0;
}


