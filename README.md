# stm32f446re_gpio_driver

Drivers for GPIO peripherals of STM32F446RE Microcontroller and some demo.

## Requirements

[STM32 Nucleo-64 development board](https://www.st.com/en/evaluation-tools/nucleo-f446re.html)

# Installation

`git clone https://github.com/xuanjiao/stm32f446re_gpio_driver.git`

# Usage

## Use drivers in a customer project
Copy the entire `driver` folder to your project directory. In `driver` folder,

- `stm32f446xx.h` is MCU specific header file which contains microcontroller specific data
-  `stm32f446xx_gpio.h` are driver header files which contains driver specific data 
-  `stm32f446xx_gpio.c` are driver source files

## Run example
1. Open project in IDE (e.g. STM32Cube), build project with `button_interrupt.c` file
2. Press the B1 button on the board
3. Observe the LED state (ON -> OFF or OFF-> ON) 


