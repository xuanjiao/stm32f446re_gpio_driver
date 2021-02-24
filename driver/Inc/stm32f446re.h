/*
 * stm32f446re.h
 *
 *  Created on: Feb 24, 2021
 *      Author: Xuanjiao
 */

#ifndef INC_STM32F446RE_H_
#define INC_STM32F446RE_H_

/*
 * Base addresses of MCU's embedded memories
 */
#define FLASH_BASEADDR						(uint32_t)0x08000000	/* Base address of flash */
#define SRAM1_BASEADDR						(uint32_t)0x20000000	/* Base address of SRAM1 */
#define	SRAM2_BASEADDR						(uint32_t)0x2001C000	/* Base address of SRAM2 */
#define ROM									(uint32_t)0x1FFF0000    /* Base address of System memory */
#define SRAM								SRAM1_BASEADDR			/* SRAM consists of SRAM1 and SRAM2*/

/*
 * Base addresses of different bus domains
 * AHB bus is used for peripherals which need high speed data communication.
 * APB bus is used for peripherals capable of low speed data communication.
 */
#define APB1PERIPH_BASE						(uint32_t)0x40000000	/* Base address of APB1 bus */
#define APB2PERIPH_BASE						(uint32_t)0x40010000	/* Base address of APB2 bus */
#define AHB1PERIPH_BASE						(uint32_t)0x40020000	/* Base address of AHB1 bus */
#define AHB2PERIPH_BASE						(uint32_t)0x50000000	/* Base address of AHB2 bus */

/*
 * Base addresses of peripherals which are hanging on AHB1 bus
 */
#define GPIOA_BASEADDR						(AHB1PERIPH_BASE + 0x0000)
#define GPIOB_BASEADDR						(AHB1PERIPH_BASE + 0x0400)
#define GPIOC_BASEADDR						(AHB1PERIPH_BASE + 0x0800)
#define GPIOD_BASEADDR						(AHB1PERIPH_BASE + 0x0C00)
#define GPIOE_BASEADDR						(AHB1PERIPH_BASE + 0x1000)
#define GPIOF_BASEADDR						(AHB1PERIPH_BASE + 0x1400)
#define GPIOG_BASEADDR						(AHB1PERIPH_BASE + 0x1800)
#define GPIOH_BASEADDR						(AHB1PERIPH_BASE + 0x1C00)

/*
 * Base addresses of peripherals which are hanging on APB1 bus
 */
#define I2C1_BASEADDR						(APB1PERIPH_BASE + 0x5400)
#define I2C2_BASEADDR						(APB1PERIPH_BASE + 0x5800)
#define USART2_BASEADDR						(APB1PERIPH_BASE + 0x4400)
#define USART3_BASEADDR						(APB1PERIPH_BASE + 0x4800)
#define UART4_BASEADDR						(APB1PERIPH_BASE + 0x4C00)
#define UART5_BASEADDR						(APB1PERIPH_BASE + 0x5000)
#define SPI2_BASEADDR						(APB1PERIPH_BASE + 0x3800)
#define SPI3_BASEADDR						(APB1PERIPH_BASE + 0x3C00)

/*
 * Base addresses of peripherals which are hanging on APB2 bus
 */
#define SPI1_BASEADDR						(APB2PERIPH_BASE + 0x3000)
#define USART1_BASEADDR						(APB2PERIPH_BASE + 0x1000)
#define USART6_BASEADDR						(APB2PERIPH_BASE + 0x1400)
#define EXTI_BASEADDR						(APB2PERIPH_BASE + 0x3C00)
#define SYSCFG_BASEADDR						(APB2PERIPH_BASE + 0x3800)


#endif /* INC_STM32F446RE_H_ */
