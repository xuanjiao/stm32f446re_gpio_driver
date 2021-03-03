/*
 * button_interrupt.c
 *
 *  Created on: Mar 3, 2021
 *      Author: Xuanjiao Zhu
 */



#include "stm32f446xx.h"
#include <string.h>

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
	memset(&gpio_led,0,sizeof(gpio_led));
	memset(&gpio_btn,0,sizeof(gpio_btn));

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
	gpio_btn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT; // problem here gpio c mode13 0
	gpio_btn.GPIO_PinConfig.GPIO_PinOPType = GPIO_OUT_TYPE_PP;
	gpio_btn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_NO_PU_PD;
	gpio_btn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;

	GPIO_PeriClockControl(GPIOC, ENABLE);

	GPIO_Init(&gpio_btn);

	// Configure the IRQ
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI15_10, NVIC_IRQ_PRI15);
	GPIO_IRQInterrputConfig(IRQ_NO_EXTI15_10,ENABLE);


	while(1);
}

/*
 * Handle the button interrupt
 */
void EXTI15_10_IRQHandler(void)
{
	delay();
	GPIO_IRQHandling(GPIO_PIN_NO_13);			// The driver handle the interrupt
	GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_5);	// Toggle the LED
}

