/*
 * stm32f446xx.h
 *
 *  Created on: Feb 24, 2021
 *      Author: Xuanjiao Zhu
 */

#ifndef INC_STM32F446XX_H_
#define INC_STM32F446XX_H_

#include<stdint.h>
#include <stdio.h>
#include <stdlib.h>

#define __vo volatile
#define __weak  __attribute__((weak))

/**********************************START:Processor Specific Details **********************************/
/*
 * ARM Cortex M4 Processor NVIC ISERx register (Interrupt Set-pending Registers) Addresses
 */
#define NVIC_ISER0							((__vo uint32_t*)0xE000E100)
#define NVIC_ISER1							((__vo uint32_t*)0xE000E104)
#define NVIC_ISER2							((__vo uint32_t*)0xE000E108)
#define NVIC_ISER3							((__vo uint32_t*)0xE000E10C)


/*
 * ARM Cortex M4 Processor NVIC ICERx register (Interrupt Clear-pending Registers) Addresses
 */
#define NVIC_ICER0							((__vo uint32_t*)0xE000E180)
#define NVIC_ICER1							((__vo uint32_t*)0xE000E184)
#define NVIC_ICER2							((__vo uint32_t*)0xE000E188)
#define NVIC_ICER3							((__vo uint32_t*)0xE000E18C)

/*
 * ARM Cortex M4 Processor NVIC IPRx register (Interrupt Priority Registers) Base Addresses
 */
#define NVIC_IPR_BASEADDR					((__vo uint32_t*)0xE000E400)

/*
 * ARM Cortex M4 Processor number of priority bits implemented in Priority Register
 * (non-implemented low-order bits read as zero and ignore writes)
 */
#define NO_PR_BITS_IMPLEMENTED  			4

/*
 * Base addresses of MCU's embedded memories
 */
#define FLASH_BASEADDR						0x08000000U	/* Base address of flash */
#define SRAM1_BASEADDR						0x20000000U	/* Base address of SRAM1 */
#define	SRAM2_BASEADDR						0x2001C000U	/* Base address of SRAM2 */
#define ROM									0x1FFF0000U    /* Base address of System memory */
#define SRAM								SRAM1_BASEADDR			/* SRAM consists of SRAM1 and SRAM2*/

/*
 * Base addresses of different bus domains
 * AHB bus is used for peripherals which need high speed data communication.
 * APB bus is used for peripherals capable of low speed data communication.
 */
#define PERIPH_BASEADDR 					0x40000000U
#define APB1PERIPH_BASE						PERIPH_BASEADDR	/* Base address of APB1 bus */
#define APB2PERIPH_BASE						0x40010000U	/* Base address of APB2 bus */
#define AHB1PERIPH_BASE						0x40020000U	/* Base address of AHB1 bus */
#define AHB2PERIPH_BASE						0x50000000U	/* Base address of AHB2 bus */

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
#define SPI4_BASEADDR						(APB2PERIPH_BASE + 0x3400)


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
	__vo uint32_t AFR[2];					/* AFR[0]: GPIO port alternate function low register, AFR[1]: GPIO port alternate function high register Address offset: 0x20 - 0x24 */
}GPIO_RegDef_t;

typedef struct
{
	__vo uint32_t CR1;						/* SPI control register 1 							Address offset: 0x00 */
	__vo uint32_t CR2;						/* SPI control register 2							Address offset: 0x04 */
	__vo uint32_t SR;						/* SPI status register								Address offset: 0x08 */
	__vo uint32_t DR;						/* SPI data register								Address offset: 0x0C */
	__vo uint32_t CRCPR;					/* SPI CRC polynomial register						Address offset: 0x10 */
	__vo uint32_t RXCRCR;					/* SPI RX CRC register								Address offset: 0x14 */
	__vo uint32_t TXCRCR;					/* SPI TX CRC register								Address offset: 0x18 */
	__vo uint32_t I2SCFGR;					/* SPI_I2S configuration register					Address offset: 0x1C */
	__vo uint32_t I2SPR;					/* SPI_I2S prescaler register						Address offset: 0x20 */
}SPI_RegDef_t;

typedef struct
{
	__vo uint32_t I2C_CR1;					/* I2C control register 1 							Address offset: 0x00 */
	__vo uint32_t I2C_CR2;					/* I2C control register 2 							Address offset: 0x04 */
	__vo uint32_t I2C_OAR1;					/* I2C own address register 1 						Address offset: 0x08 */
	__vo uint32_t I2C_OAR2;					/* I2C own address register 2 						Address offset: 0x0C */
	__vo uint32_t I2C_DR;					/* I2C data register 								Address offset: 0x10 */
	__vo uint32_t I2C_SR1;					/* I2C status register 1 							Address offset: 0x14 */
	__vo uint32_t I2C_SR2;					/* I2C status register 2							Address offset: 0x18 */
	__vo uint32_t I2C_CCR;					/* I2C clock control register 						Address offset: 0x1C */
	__vo uint32_t I2C_TRISE;				/* I2C TRISE register								Address offset: 0x20 */
	__vo uint32_t I2C_FLTR;					/* I2C FLTR register 								Address offset: 0x24 */
}I2C_RegDef_t;

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

#define SPI1								((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2								((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3								((SPI_RegDef_t*)SPI3_BASEADDR)
#define SPI4								((SPI_RegDef_t*)SPI4_BASEADDR)

#define I2C1								((I2C_RegDef_t*)I2C1_BASEADDR)
#define I2C2								((I2C_RegDef_t*)I2C2_BASEADDR)

#define RCC									((RCC_RegDef_t*)RCC_BASEADDR)

#define EXTI								((EXTI_RegDef_t*)EXTI_BASEADDR)
#define SYSCFG								((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

/*
 * Peripheral enable and disable Macros
 */

/*
 * Clock Enable Macros for GPIOx peripherals
 */
#define GPIOA_PCLK_EN()						( RCC->AHB1ENR |= ( 1 << 0 ) )
#define GPIOB_PCLK_EN()						( RCC->AHB1ENR |= ( 1 << 1 ) )
#define GPIOC_PCLK_EN()						( RCC->AHB1ENR |= ( 1 << 2 ) )
#define GPIOD_PCLK_EN()						( RCC->AHB1ENR |= ( 1 << 3 ) )
#define GPIOE_PCLK_EN()						( RCC->AHB1ENR |= ( 1 << 4 ) )
#define GPIOF_PCLK_EN()						( RCC->AHB1ENR |= ( 1 << 5 ) )
#define GPIOG_PCLK_EN()						( RCC->AHB1ENR |= ( 1 << 6 ) )
#define GPIOH_PCLK_EN()						( RCC->AHB1ENR |= ( 1 << 7 ) )

/*
 * Clock Enable Macros for I2Cx peripherals
 */
#define I2C1_PCLK_EN()						( RCC->APB1ENR |= ( 1 << 21 ) )
#define I2C2_PCLK_EN()						( RCC->APB1ENR |= ( 1 << 22 ) )
#define I2C3_PCLK_EN()						( RCC->APB1ENR |= ( 1 << 23 ) )

/*
 * Clock Enable Macros for SPIx peripherals
 */
#define SPI1_PCLK_EN()						( RCC->APB2ENR |= ( 1 << 12) )
#define SPI2_PCLK_EN()						( RCC->APB1ENR |= ( 1 << 14) )
#define SPI3_PCLK_EN()						( RCC->APB1ENR |= ( 1 << 15) )
#define SPI4_PCLK_EN()						( RCC->APB2ENR |= ( 1 << 13) )

/*
 * Clock Enable Macros for USARTx peripherals
 */
#define USART1_PCLK_EN()					( RCC->APB2ENR |= ( 1 << 4 ) )
#define USART2_PCLK_EN()					( RCC->APB1ENR |= ( 1 << 17 ) )
#define USART3_PCLK_EN()					( RCC->APB1ENR |= ( 1 << 18 ) )
#define UART4_PCLK_EN()						( RCC->APB1ENR |= ( 1 << 19 ) )
#define UART5_PCLK_EN()						( RCC->APB1ENR |= ( 1 << 20 ) )
#define USART6_PCLK_EN()					( RCC->APB2ENR |= ( 1 << 6  ) )

/*
 * Clock Enable Macros for SYSCFG peripherals
 */
#define SYSCFG_PCLK_EN()					( RCC->APB2ENR |= ( 1 << 14) )

/*
 * Clock Disable Macros for GPIOx peripherals
 */

#define GPIOA_PCLK_DI()						( RCC->AHB1ENR &= ~( 1 << 0 ) )
#define GPIOB_PCLK_DI()						( RCC->AHB1ENR &= ~( 1 << 1 ) )
#define GPIOC_PCLK_DI()						( RCC->AHB1ENR &= ~( 1 << 2 ) )
#define GPIOD_PCLK_DI()						( RCC->AHB1ENR &= ~( 1 << 3 ) )
#define GPIOE_PCLK_DI()						( RCC->AHB1ENR &= ~( 1 << 4 ) )
#define GPIOF_PCLK_DI()						( RCC->AHB1ENR &= ~( 1 << 5 ) )
#define GPIOG_PCLK_DI()						( RCC->AHB1ENR &= ~( 1 << 6 ) )
#define GPIOH_PCLK_DI()						( RCC->AHB1ENR &= ~( 1 << 7 ) )

/*
 * Clock Disable Macros for I2Cx peripherals
 */
#define I2C1_PCLK_DI()						( RCC->APB1ENR &= ~( 1 << 21) )
#define I2C2_PCLK_DI()						( RCC->APB1ENR &= ~( 1 << 22) )
#define I2C3_PCLK_DI()						( RCC->APB1ENR &= ~( 1 << 23) )

/*
 * Clock Disable Macros for SPIx peripherals
 */
#define SPI1_PCLK_DI()						( RCC->APB2ENR &= ~( 1 << 12 ) )
#define SPI2_PCLK_DI()						( RCC->APB1ENR &= ~( 1 << 14 ) )
#define SPI3_PCLK_DI()						( RCC->APB1ENR &= ~( 1 << 15 ) )
#define SPI4_PCLK_DI()						( RCC->APB2ENR &= ~( 1 << 13 ) )

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
#define GPIOA_REG_RESET()					do{ RCC->AHB1RSTR |= ( 1 << 0 ); RCC->AHB1RSTR &= ~( 1 << 0 ); } while(0)
#define GPIOB_REG_RESET()					do{ RCC->AHB1RSTR |= ( 1 << 1 ); RCC->AHB1RSTR &= ~( 1 << 1 ); } while(0)
#define GPIOC_REG_RESET()					do{ RCC->AHB1RSTR |= ( 1 << 2 ); RCC->AHB1RSTR &= ~( 1 << 2 ); } while(0)
#define GPIOD_REG_RESET()					do{ RCC->AHB1RSTR |= ( 1 << 3 ); RCC->AHB1RSTR &= ~( 1 << 3 ); } while(0)
#define GPIOE_REG_RESET()					do{ RCC->AHB1RSTR |= ( 1 << 4 ); RCC->AHB1RSTR &= ~( 1 << 4 ); } while(0)
#define GPIOF_REG_RESET()					do{ RCC->AHB1RSTR |= ( 1 << 5 ); RCC->AHB1RSTR &= ~( 1 << 5 ); } while(0)
#define GPIOG_REG_RESET()					do{ RCC->AHB1RSTR |= ( 1 << 6 ); RCC->AHB1RSTR &= ~( 1 << 6 ); } while(0)
#define GPIOH_REG_RESET()					do{ RCC->AHB1RSTR |= ( 1 << 7 ); RCC->AHB1RSTR &= ~( 1 << 7 ); } while(0)

/*
 * Reset Macros for SPIx peripherals
 */
#define SPI1_REG_RESET()						do{ RCC->APB2RSTR |= ( 1 << 12 ); RCC->APB2RSTR |= ~( 1 << 12 );} while(0)
#define SPI2_REG_RESET()						do{ RCC->APB1RSTR |= ( 1 << 14 ); RCC->APB1RSTR |= ~( 1 << 14 );} while(0)
#define SPI3_REG_RESET()						do{ RCC->APB1RSTR |= ( 1 << 15 ); RCC->APB1RSTR |= ~( 1 << 15 );} while(0)
#define SPI4_REG_RESET()						do{ RCC->APB2RSTR |= ( 1 << 13 ); RCC->APB1RSTR |= ~( 1 << 13 );} while(0)

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
 * IRQ(Interrupt Request) Numbers of STM32F446x MCU
 */

#define IRQ_NO_EXTI0 		6
#define IRQ_NO_EXTI1 		7
#define IRQ_NO_EXTI2 		8
#define IRQ_NO_EXTI3 		9
#define IRQ_NO_EXTI4 		10
#define IRQ_NO_EXTI9_5 		23
#define IRQ_NO_EXTI15_10 	40
#define IRQ_NO_SPI1			35
#define IRQ_NO_SPI2         36
#define IRQ_NO_SPI3         51
#define IRQ_NO_SPI4			84
#define IRQ_NO_I2C1_EV     31
#define IRQ_NO_I2C1_ER     32
#define IRQ_NO_USART1	    37
#define IRQ_NO_USART2	    38
#define IRQ_NO_USART3	    39
#define IRQ_NO_UART4	    52
#define IRQ_NO_UART5	    53
#define IRQ_NO_USART6	    71


/*
 * 　Macros for all the possible priority levels
 */
#define NVIC_IRQ_PRI0    	0
#define NVIC_IRQ_PRI1    	1
#define NVIC_IRQ_PRI2    	2
#define NVIC_IRQ_PRI3   	3
#define NVIC_IRQ_PRI4    	4
#define NVIC_IRQ_PRI5    	5
#define NVIC_IRQ_PRI6    	6
#define NVIC_IRQ_PRI7    	7
#define NVIC_IRQ_PRI8    	8
#define NVIC_IRQ_PRI9    	9
#define NVIC_IRQ_PRI10    	10
#define NVIC_IRQ_PRI11    	11
#define NVIC_IRQ_PRI12    	12
#define NVIC_IRQ_PRI13    	13
#define NVIC_IRQ_PRI14    	14
#define NVIC_IRQ_PRI15    	15


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

/********************************************
 * Bit position definitions for SPI peripheral
 * ******************************************/

/*
  * Bit position definitions for SPI_CR1
  */
#define SPI_CR1_CPHA			0
#define SPI_CR1_CPOL			1
#define SPI_CR1_MSTR			2
#define SPI_CR1_BR				3
#define SPI_CR1_SPE				6
#define SPI_CR1_LSBFIRST		7
#define SPI_CR1_SSI				8
#define SPI_CR1_SSM				9
#define SPI_CR1_RXONLY			10
#define SPI_CR1_DFF				11
#define SPI_CR1_CRCNEXT			12
#define SPI_CR1_CRCEN			13
#define SPI_CR1_BIDIOE			14
#define SPI_CR1_BIDIMODE		15

/*
  * Bit position definitions for SPI_CR2
  */
#define SPI_CR2_RXDMAEN			0
#define SPI_CR2_TXDMAEN			1
#define SPI_CR2_SSOE			2
#define SPI_CR2_FRF				4
#define SPI_CR2_ERRIE			5
#define SPI_CR2_RXNEIE			6
#define SPI_CR2_TXEIE			7

/*
  * Bit position definitions for SPI_SR
  */
#define SPI_SR_RXNE				0
#define SPI_SR_TXE				1
#define SPI_SR_CHSIDE			2
#define SPI_SR_UDR				3
#define SPI_SR_CRCERR			4
#define SPI_SR_MODF				5
#define SPI_SR_OVR				6
#define SPI_SR_BSY				7
#define SPI_SR_FRE				8

/*
 * Bit position definitions for I2C_CR1
 */
#define I2C_CR1_PE				1
#define I2C_CR1_SMBUS			2
#define I2C_CR1_SMBTYPE			3
#define I2C_CR1_ENARP			4
#define I2C_CR1_ENPEC			5
#define I2C_CR1_ENGC			6
#define I2C_CR1_NOSTRETCH		7
#define I2C_CR1_START			8
#define I2C_CR1_STOP			9
#define I2C_CR1_ACK				10
#define I2C_CR1_POS				11
#define I2C_CR1_PEC				12
#define I2C_CR1_ALERT			13
#define I2C_CR1_SWRST			15

/*
 * Bit position definitions for I2C_CR2
 */
#define I2C_CR2_FREQ			0
#define I2C_CR2_ITERREN			8
#define I2C_CR2_ITEVTEN			9
#define I2C_CR2_ITBUFEN			10
#define I2C_CR2_LAST			12
/*
 * Bit position definitions for I2C_SR1
 */
#define I2C_SR1_SB				0
#define I2C_SR1_ADDR			1
#define I2C_SR1_BTF				2
#define I2C_SR1_ADD10			3
#define I2C_SR1_STOPF			4
#define I2C_SR1_RxNE			6
#define I2C_SR1_TxE				7
#define I2C_SR1_BERR			8
#define I2C_SR1_ARLO			9
#define I2C_SR1_AF				10
#define I2C_SR1_OVR				11
#define I2C_SR1_PECERR			12
#define I2C_SR1_TIMEOUT			14
#define I2C_SR1_SMBALERT		15

/*
 * Bit position definitions for I2C_SR2
 */
#define I2C_SR2_MSL				0
#define I2C_SR2_BUSY			1
#define I2C_SR2_TRA				2
#define I2C_SR2_GENCALL			4
#define I2C_SR2_SMBDEFAULT		5
#define I2C_SR2_SMBHOST			6
#define I2C_SR2_DUALF			7
#define I2C_SR2_PEC				8

/*
 * Bit position definitions for I2C_CCR
 */
#define I2C_CCR_CCR				0
#define I2C_CCR_DUTY			14
#define I2C_CCR_FS				15

/*
 * Include driver header files
 */
#include "stm32f446xx_gpio.h"

#endif /* INC_STM32F446XX_H_ */
