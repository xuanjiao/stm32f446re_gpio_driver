/*
 * stm32f446xx.h
 *
 *  Created on: Feb 24, 2021
 *      Author: Xuanjiao Zhu
 */

#ifndef INC_STM32F446XX_H_
#define INC_STM32F446XX_H_

#include<stdint.h>

#define __vo volatile

/**********************************START:Processor Specific Details **********************************/
/*
 * ARM Cortex M4 Processor NVIC ISERx register (Interrupt Set-pending Registers) Addresses
 */
#define NVIC_ISER0							(uint32_t*)0xE000E100
#define NVIC_ISER1							(uint32_t*)0xE000E104
#define NVIC_ISER2							(uint32_t*)0xE000E108
#define NVIC_ISER3							(uint32_t*)0xE000E10C
#define NVIC_ISER4							(uint32_t*)0xE000E110
#define NVIC_ISER5							(uint32_t*)0xE000E114
#define NVIC_ISER6							(uint32_t*)0xE000E118
#define NVIC_ISER7							(uint32_t*)0xE000E11C

/*
 * ARM Cortex M4 Processor NVIC ICERx register (Interrupt Clear-pending Registers) Addresses
 */
#define NVIC_ICER0							(uint32_t*)0xE000E180
#define NVIC_ICER1							(uint32_t*)0xE000E184
#define NVIC_ICER2							(uint32_t*)0xE000E188
#define NVIC_ICER3							(uint32_t*)0xE000E18C
#define NVIC_ICER4							(uint32_t*)0xE000E190
#define NVIC_ICER5							(uint32_t*)0xE000E194
#define NVIC_ICER6							(uint32_t*)0xE000E198
#define NVIC_ICER7							(uint32_t*)0xE000E19C

/*
 * ARM Cortex M4 Processor NVIC IPRx register (Interrupt Priority Registers) Base Addresses
 */
#define NVIC_IPR_BASEADDR					uint32_t*)0xE000E400

/*
 * ARM Cortex M4 Processor number of priority bits implemented in Priority Register
 * (non-implemented low-order bits read as zero and ignore writes)
 */
#define NO_PR_BITS_IMPLEMENTED  			4

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
#define RCC_BASEADDR						(AHB1PERIPH_BASE + 0x3800)

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




/*
 * Peripheral register structure definition
 */
typedef struct
{
	__vo uint32_t MODE;						/* GPIO port mode register 							Address offset: 0x00 */
	__vo uint32_t OTYPER;					/* GPIO port output type register					Address offset: 0x04 */
	__vo uint32_t OSPEEDER;					/* GPIO port output speed register 					Address offset: 0x08 */
	__vo uint32_t PUPDR;					/* GPIO port pull-up/pull-down register 			Address offset: 0x0C */
	__vo uint32_t IDR;						/* GPIO port input data register 					Address offset: 0x10 */
	__vo uint32_t ODR;						/* GPIO port output data register 					Address offset: 0x14 */
	__vo uint32_t BSRR;						/* GPIO port bit set/reset register					Address offset: 0x18 */
	__vo uint32_t LCKR;						/* GPIO port port configuration lock register 		Address offset: 0x1C */
	__vo uint32_t AFR[2];					/* AFR[0]: GPIO port alternate function low register, AFR[1]: GPIO port alternate function high register Address offset: 0x20-0x24*/
}GPIO_RegDef_t;

typedef struct
{
	__vo uint32_t CR;						/* RCC clock control register						Address offset: 0x00 */
	__vo uint32_t PLLCFGR;					/* RCC PLL configuration register					Address offset: 0x04 */
	__vo uint32_t CFGR;						/* RCC clock configuration register					Address offset: 0x08 */
	__vo uint32_t CIR;						/* RCC clock interrupt register						Address offset: 0x0C */
	__vo uint32_t AHB1RSTR;					/* RCC AHB1 peripheral reset register				Address offset: 0x10 */
	__vo uint32_t AHB2RSTR;					/* RCC AHB2 peripheral reset register				Address offset: 0x14 */
	__vo uint32_t AHB3RSTR;					/* AHB3 peripheral reset register					Address offset: 0x18 */
	uint32_t RESERVED0;
	__vo uint32_t APB1RSTR;					/* RCC APB1 peripheral reset register				Address offset: 0x20 */
	__vo uint32_t APB2RSTR;					/* RCC APB2 peripheral reset register				Address offset: 0x24 */
	uint32_t RESERVED1[2];
	__vo uint32_t AHB1ENR;					/* RCC AHB1 peripheral clock enable register		Address offset: 0x30 */
	__vo uint32_t AHB2ENR;					/* AHB2 peripheral clock enable register			Address offset: 0x34 */
	__vo uint32_t AHB3ENR;					/* RCC AHB3 peripheral clock enable register		Address offset: 0x38 */
	uint32_t RESERVED2;
	__vo uint32_t APB1ENR;					/* RCC APB1 peripheral clock enable register		Address offset: 0x40 */
	__vo uint32_t APB2ENR;					/* RCC APB2 peripheral clock enable register		Address offset: 0x44 */
	uint32_t RESERVED3[2];
	__vo uint32_t AHB1LPENR;				/* RCC AHB1 peripheral clock enable in low power mode register  Address offset: 0x50 */
	__vo uint32_t AHB2LPENR;				/* RCC AHB2 peripheral clock enable in low power mode register  Address offset: 0x54 */
	__vo uint32_t AHB3LPENR;				/* RCC AHB3 peripheral clock enable in low power mode register  Address offset: 0x58 */
	__vo uint32_t RESERVED4;
	__vo uint32_t APB1LPENR;				/* RCC APB1 peripheral clock enable in low power mode register  Address offset: 0x60 */
	__vo uint32_t APB2LPENR;				/* RCC APB2 peripheral clock enable in low power mode register Address offset: 0x64 */
	uint32_t RESERVED5[2];
	__vo uint32_t BDCR;						/* RCC Backup domain control register				Address offset: 0x70 */
	__vo uint32_t CSR;						/* RCC clock control & status register				Address offset: 0x74 */
	uint32_t RESERVED6[2];
	__vo uint32_t SSCGR;					/* RCC spread spectrum clock generation register	Address offset: 0x80 */
	__vo uint32_t PLLI2SCFGR;				/* RCC PLLI2S configuration register				Address offset: 0x84 */
	__vo uint32_t PLLSAICFGR;				/* RCC PLL configuration register					Address offset: 0x88 */
	__vo uint32_t DCKCFGR;					/* RCC dedicated clock configuration register		Address offset: 0x8C */
	__vo uint32_t CKGATENR;					/* RCC clocks gated enable register					Address offset: 0x90 */
	__vo uint32_t DCKCFGR2;					/* RCC dedicated clocks configuration register 2	Address offset: 0x94 */
}RCC_RegDef_t;


typedef struct{
	__vo uint32_t EXTI_IMR;					/* Interrupt mask register							Address offset: 0x00 */
	__vo uint32_t EXTI_EMR;					/* Event mask register								Address offset: 0x04 */
	__vo uint32_t EXTI_RTSR;				/* Rising trigger selection register				Address offset: 0x08 */
	__vo uint32_t EXTI_FTSR;				/* Falling trigger selection register				Address offset: 0x0C */
	__vo uint32_t EXTI_SWIER;				/* Software interrupt event register 				Address offset: 0x10 */
	__vo uint32_t EXTI_PR;					/* Pending register									Address offset: 0x14 */
}EXTI_RegDef_t;

typedef struct{
	__vo uint32_t SYSCFG_MEMRMP;		/* SYSCFG memory remap register							Address offset: 0x00 */
	__vo uint32_t SYSCFG_PMC;			/* SYSCFG peripheral mode configuration register		Address offset: 0x04 */
	__vo uint32_t SYSCFG_EXTICR[4];		/* SYSCFG external interrupt configuration register 1-4	Address offset: 0x08-0x14 */
	uint32_t RESERVED0[2];
	__vo uint32_t SYSCFG_CMPCR;			/* Compensation cell control register					Address offset: 0x20 */
	uint32_t RESERVED1[2];
	__vo uint32_t SYSCFG_CFGR;			/* SYSCFG configuration register						Address offset: 0x2C */
}SYSCFG_RegDef_t;

/*
 * Peripheral definition
 */
#define GPIOA								((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB								((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC								((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD								((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE								((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF								((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG								((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH								((GPIO_RegDef_t*)GPIOH_BASEADDR)

#define RCC									((RCC_RegDef_t*)RCC_BASEADDR)

#define EXTI								((EXTI_RegDef_t*)EXTI_BASEADDR)
#define SYSCFG								((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

/*
 * Peripheral enable and disable Macros
 */

/*
 * Clock Enable Macros for GPIOx peripherals
 */
#define GPIOA_PCLK_EN()						( RCC->AHB1ENR |= ( 1<<0) )
#define GPIOB_PCLK_EN()						( RCC->AHB1ENR |= ( 1<<1) )
#define GPIOC_PCLK_EN()						( RCC->AHB1ENR |= ( 1<<2) )
#define GPIOD_PCLK_EN()						( RCC->AHB1ENR |= ( 1<<3) )
#define GPIOE_PCLK_EN()						( RCC->AHB1ENR |= ( 1<<4) )
#define GPIOF_PCLK_EN()						( RCC->AHB1ENR |= ( 1<<5) )
#define GPIOG_PCLK_EN()						( RCC->AHB1ENR |= ( 1<<6) )
#define GPIOH_PCLK_EN()						( RCC->AHB1ENR |= ( 1<<7) )

/*
 * Clock Enable Macros for I2Cx peripherals
 */
#define I2C1_PCLK_EN()						( RCC->APB1ENR |= ( 1<<21) )
#define I2C2_PCLK_EN()						( RCC->APB1ENR |= ( 1<<22) )
#define I2C3_PCLK_EN()						( RCC->APB1ENR |= ( 1<<23) )

/*
 * Clock Enable Macros for SPIx peripherals
 */
#define SPI1_PCLK_EN()						( RCC->APB2ENR |= ( 1<<12) )
#define SPI2_PCLK_EN()						( RCC->APB1ENR |= ( 1<<14) )
#define SPI3_PCLK_EN()						( RCC->APB1ENR |= ( 1<<15) )
#define SPI4_PCLK_EN()						( RCC->APB2ENR |= ( 1<<13) )

/*
 * Clock Enable Macros for USARTx peripherals
 */
#define USART1_PCLK_EN()					( RCC->APB2ENR |= ( 1<<4 ) )
#define USART2_PCLK_EN()					( RCC->APB1ENR |= ( 1<<17) )
#define USART3_PCLK_EN()					( RCC->APB1ENR |= ( 1<<18) )
#define UART4_PCLK_EN()						( RCC->APB1ENR |= ( 1<<19) )
#define UART5_PCLK_EN()						( RCC->APB1ENR |= ( 1<<20) )
#define USART6_PCLK_EN()					( RCC->APB2ENR |= ( 1<<6 ) )

/*
 * Clock Enable Macros for SYSCFG peripherals
 */
#define SYSCFG_PCLK_EN()					( RCC->APB2ENR |= ( 1<<14) )

/*
 * Clock Disable Macros for GPIOx peripherals
 */

#define GPIOA_PCLK_DI()						( RCC->AHB1ENR &= ~( 1<<0) )
#define GPIOB_PCLK_DI()						( RCC->AHB1ENR &= ~( 1<<1) )
#define GPIOC_PCLK_DI()						( RCC->AHB1ENR &= ~( 1<<2) )
#define GPIOD_PCLK_DI()						( RCC->AHB1ENR &= ~( 1<<3) )
#define GPIOE_PCLK_DI()						( RCC->AHB1ENR &= ~( 1<<4) )
#define GPIOF_PCLK_DI()						( RCC->AHB1ENR &= ~( 1<<5) )
#define GPIOG_PCLK_DI()						( RCC->AHB1ENR &= ~( 1<<6) )
#define GPIOH_PCLK_DI()						( RCC->AHB1ENR &= ~( 1<<7) )

/*
 * Clock Disable Macros for I2Cx peripherals
 */
#define I2C1_PCLK_DI()						( RCC->APB1ENR &= ~( 1<<21) )
#define I2C2_PCLK_DI()						( RCC->APB1ENR &= ~( 1<<22) )
#define I2C3_PCLK_DI()						( RCC->APB1ENR &= ~( 1<<23) )

/*
 * Clock Disable Macros for SPIx peripherals
 */
#define SPI1_PCLK_DI()						( RCC->APB2ENR &= ~( 1<<12) )
#define SPI2_PCLK_DI()						( RCC->APB1ENR &= ~( 1<<14) )
#define SPI3_PCLK_DI()						( RCC->APB1ENR &= ~( 1<<15) )
#define SPI4_PCLK_DI()						( RCC->APB2ENR &= ~( 1<<13) )

/*
 * Clock Disable Macros for USARTx peripherals
 */
#define USART1_PCLK_DI()					( RCC->APB2ENR &= ~( 1<<4 ) )
#define USART2_PCLK_DI()					( RCC->APB1ENR &= ~( 1<<17) )
#define USART3_PCLK_DI()					( RCC->APB1ENR &= ~( 1<<18) )
#define UART4_PCLK_DI()						( RCC->APB1ENR &= ~( 1<<19) )
#define UART5_PCLK_DI()						( RCC->APB1ENR &= ~( 1<<20) )
#define USART6_PCLK_DI()					( RCC->APB2ENR &= ~( 1<<6 ) )

/*
 * Clock Disable Macros for SYSCFG peripherals
 */
#define SYSCFG_PCLK_DI()					( RCC->APB2ENR &= ~( 1<<14) )

/*
 * Reset Macros for GPIOx peripherals
 */
#define GPIOA_REG_RESET()					do{ RCC->AHB1RSTR |= ( 1<<0); RCC->AHB1RSTR &= ~( 1<<0);} while(0)
#define GPIOB_REG_RESET()					do{ RCC->AHB1RSTR |= ( 1<<1); RCC->AHB1RSTR &= ~( 1<<1);} while(0)
#define GPIOC_REG_RESET()					do{ RCC->AHB1RSTR |= ( 1<<2); RCC->AHB1RSTR &= ~( 1<<2);} while(0)
#define GPIOD_REG_RESET()					do{ RCC->AHB1RSTR |= ( 1<<3); RCC->AHB1RSTR &= ~( 1<<3);} while(0)
#define GPIOE_REG_RESET()					do{ RCC->AHB1RSTR |= ( 1<<4); RCC->AHB1RSTR &= ~( 1<<4);} while(0)
#define GPIOF_REG_RESET()					do{ RCC->AHB1RSTR |= ( 1<<5); RCC->AHB1RSTR &= ~( 1<<5);} while(0)
#define GPIOG_REG_RESET()					do{ RCC->AHB1RSTR |= ( 1<<6); RCC->AHB1RSTR &= ~( 1<<6);} while(0)
#define GPIOH_REG_RESET()					do{ RCC->AHB1RSTR |= ( 1<<7); RCC->AHB1RSTR &= ~( 1<<7);} while(0)

/*
 * Convert GPIO Port base address to code
 */
#define GPIO_BASEADDR_TO_CODE(x)			( ( x == GPIOA )? 0: \
											  ( x == GPIOB )? 1: \
											  ( x == GPIOC )? 2: \
											  ( x == GPIOD )? 3: \
											  ( x == GPIOE )? 4: \
											  ( x == GPIOF )? 5: \
											  ( x == GPIOG )? 6: \
											  ( x == GPIOH )? 7:0 )


/*
 * Some general Macros
 */
#define ENABLE					1
#define DISABLE					0
#define SET						ENABLE
#define RESET					DISABLE
#define GPIO_PIN_SET        	SET
#define GPIO_PIN_RESET      	RESET
#define FLAG_RESET         		RESET
#define FLAG_SET 				SET

/*
 * Include driver header files
 */
#include "stm32f446xx_gpio.h"

#endif /* INC_STM32F446XX_H_ */
