#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#define __vo volatile
#define __weak __attribute__((weak))

#include<stdint.h>
#include<stddef.h>



/**********************************************************************************************************************/
/********************************************* How to use this library ************************************************/
/* (*) Reset and Clock Control                                                                                        */
/* At the start of any application, the RCC Handle Structure must be delared and have its configuration structure     */
/* programmed with appropriate configuration settings as per need. It is recommended to do this in a seperate         */
/* function.                                                                                                          */
/* If the PLL clock is to be used, then the pll configuration sturcture within the rcc configuration structure must   */
/* also be programmed with appropriate settings as per need. Details can be found in the rcc driver.c file.           */
/* Once the configurations have been set, call the RCC_Enable function and pass a reference to the rcc peripheral     */
/* register and the rcc configuration structure. The RCC is now configured and ready to be used for enabling other    */
/* peripherals.                                                                                                       */
/*                                                                                                                    */
/* (*) Embedded Flash Memory Interface (Flash)                                                                        */
/* */
/* (*) Power Controller (PWR)                                                                                         */
/* */
/* (*) General Port Input Output (GPIO)                                                                               */
/* */
/* (*) Direct Memory Access (DMA)                                                                                     */
/* */
/* (*) Interrupts and Events (NVIC) (EXTI)                                                                            */
/* */
/* (*) Analog to Digital Conversion (ADC)                                                                             */
/* */
/* (*) Digital to Analog Conversion (DAC)                                                                             */
/* */
/* (*) Serial Peripheral Interface (SPI)                                                                              */
/* */
/* (*) Inter-Intergrated Circuit Interface (I2C)                                                                      */
/* */
/* (*) Universal Synchronous Asynchronous Receiver Transmitter (USART)                                                */
/* */

/**********************************************************************************************************************/



/************************************* START: Processor Specific Details **********************************************/

//ARM Cortex Mx Processor NVIC ISERx Register Address
#define NVIC_ISER0					((__vo uint32_t*)0xE000E100)
#define NVIC_ISER1					((__vo uint32_t*)0xE000E104)
#define NVIC_ISER2					((__vo uint32_t*)0xE000E108)
#define NVIC_ISER3					((__vo uint32_t*)0xE000E10C)
#define NVIC_ISER4					((__vo uint32_t*)0xE000E110)
#define NVIC_ISER5					((__vo uint32_t*)0xE000E114)
#define NVIC_ISER6					((__vo uint32_t*)0xE000E118)
#define NVIC_ISER7					((__vo uint32_t*)0xE000E11C)

//ARM Cortex Mx Processor NVIC ICERx Register Address
#define NVIC_ICER0					((__vo uint32_t*)0xE000E180)
#define NVIC_ICER1					((__vo uint32_t*)0xE000E184)
#define NVIC_ICER2					((__vo uint32_t*)0xE000E188)
#define NVIC_ICER3					((__vo uint32_t*)0xE000E18C)
#define NVIC_ICER4					((__vo uint32_t*)0xE000E190)
#define NVIC_ICER5					((__vo uint32_t*)0xE000E194)
#define NVIC_ICER6					((__vo uint32_t*)0xE000E198)
#define NVIC_ICER7					((__vo uint32_t*)0xE000E19C)

//ARM Cortex Mx Processor Priority Register Address Calculation
#define NVIC_PR_BASEADDR			((__vo uint32_t*)0xE000E400)

//ARM Cortex Mx Processor number of priority bits implemented in Priority Register
#define NO_PR_BITS_IMPLEMENTED		4

/*************************************  END: Processor Specific Details  **********************************************/



//Base addresses of flash and SRAM memories.
#define FLASH_BASEADDR						0x08000000U			/*Base Address of Flash memory*/
#define SRAM1_BASEADDR						0x20000000U			/*Base Address of SRAM1*/
#define SRAM2_BASEADDR						0x2001C000U			/*Base Address of SRAM2*/
#define ROM_BASEADDR						0x1FFF0000U			/*Base Address of ROM*/
#define OTP_BASEADDR						0x1FFF7800U			/*Base Address of OTP*/
#define SRAM								SRAM1_BASEADDR		/*Base Address of SRAM*/

//Base addresses of AHBx and APBx bus peripherals.
#define PERIPHERAL_BASEADDR					0x40000000U			/*Base Address of where Peripheral buses start*/
#define APB1PERIPHERAL_BASEADDR				0x40000000U			/*Base Address of Peripheral bus APB1*/
#define APB2PERIPHERAL_BASEADDR				0x40010000U			/*Base Address of Peripheral bus APB2*/
#define AHB1PERIPHERAL_BASEADDR				0x40020000U			/*Base Address of Peripheral bus AHB1*/
#define AHB2PERIPHERAL_BASEADDR				0x50000000U			/*Base Address of Peripheral bus AHB2*/

//Base addresses of peripherals on the AHB1 bus.
#define GPIOA_BASEADDR						0x40020000U			/*Base Address of Peripheral Register GPIOA*/
#define GPIOB_BASEADDR						0x40020400U			/*Base Address of Peripheral Register GPIOB*/
#define GPIOC_BASEADDR						0x40020800U			/*Base Address of Peripheral Register GPIOC*/
#define GPIOD_BASEADDR						0x40020C00U			/*Base Address of Peripheral Register GPIOD*/
#define GPIOE_BASEADDR						0x40021000U			/*Base Address of Peripheral Register GPIOE*/
#define GPIOF_BASEADDR						0x40021400U			/*Base Address of Peripheral Register GPIOF*/
#define GPIOG_BASEADDR						0x40021800U			/*Base Address of Peripheral Register GPIOG*/
#define GPIOH_BASEADDR						0x40021C00U			/*Base Address of Peripheral Register GPIOH*/
#define GPIOI_BASEADDR						0x40022000U			/*Base Address of Peripheral Register GPIOI*/
#define RCC_BASEADDR						0x40023800U			/*Base Address of Peripheral Register RCC*/
#define DMA1_BASEADDR						0x40026000U			/*Base Address of Peripheral Register DMA1*/
#define DMA2_BASEADDR						0x40026400U			/*Base Address of Peripheral Register DMA2*/

//Base addresses of peripherals on the APB1 bus.
#define SPI2_I2S2_BASEADDR					0x40003800U			/*Base Address of SPI2/I2S2 Registers*/
#define SPI3_I2S3_BASEADDR					0x40003C00U			/*Base Address of SPI3/I2S3 Register*/
#define USART2_BASEADDR						0x40004400U			/*Base Address of USART2 Register*/
#define USART3_BASEADDR						0x40004800U			/*Base Address of USART3 Register*/
#define UART4_BASEADDR						0x40004C00U			/*Base Address of UART4 Register*/
#define UART5_BASEADDR						0x40005000U			/*Base Address of UART5 Register*/
#define I2C1_BASEADDR						0x40005400U			/*Base Address of I2C1 Register*/
#define I2C2_BASEADDR						0x40005800U			/*Base Address of I2C2 Register*/
#define I2C3_BASEADDR						0x40005C00U			/*Base Address of I2C3 Register*/

//Base addresses of peripherals on the APB2 bus.
#define SPI1_BASEADDR						0x40013000U			/*Base Address of SPI1 Register*/
#define USART1_BASEADDR						0x40011000U			/*Base Address of USART1 Register*/
#define USART6_BASEADDR						0x40011400U			/*Base Address of USART6 Register*/
#define EXTI_BASEADDR						0x40013C00U			/*Base Address of EXTI Register*/
#define SYSCFG_BASEADDR						0x40013800U			/*Base Address of System Configuration Register*/
#define SPI4_BASEADDR						0x40013400U			/*Base Address of SPI4 Register*/
#define SPI5_BASEADDR 						0x40015000U			/*Base Address of SPI5 Register*/
#define SPI6_BASEADDR 						0x40015400U			/*Base Address of SPI6 Register*/
#define ADC1_BASEADDR						0x40012000U			/*Base Address of ADC1 Register*/
#define ADC2_BASEADDR						0x40012100U			/*Base Address of ADC2 Register*/
#define ADC3_BASEADDR						0x40012200U			/*Base Address of ADC3 Register*/


/*********************************************************************************************************************/
/******************************************* Peripheral Structure Definitions ****************************************/
/*********************************************************************************************************************/

typedef struct
{
	__vo uint32_t MODER;		/*GPIO port mode register						Address offset: 0x00*/
	__vo uint32_t OTYPER;		/*GPIO port output type register				Address offset: 0x04*/
	__vo uint32_t OSPEEDR;		/*GPIO port output speed register				Address offset: 0x08*/
	__vo uint32_t PUPDR;		/*GPIO port pull-up/pull-down register			Address offset: 0x0C*/
	__vo uint32_t IDR;			/*GPIO port input data register					Address offset: 0x10*/
	__vo uint32_t ODR;			/*GPIO port output data register				Address offset: 0x14*/
	__vo uint32_t BSRR;			/*GPIO port bit set/reset register				Address offset: 0x18*/
	__vo uint32_t LCKR;			/*GPIO port configuration lock register			Address offset: 0x1C*/
	__vo uint32_t AFR[2];		/*GPIO alternate function registers(high/low)	Address offset: 0x20-0x24*/
}GPIO_RegDef_t;

typedef struct
{
	__vo uint32_t CR;			/*RCC clock control register					Address offset: 0x00*/
	__vo uint32_t PLLCFGR;			/*RCC PLL configuration register				Address offset: 0x04*/
	__vo uint32_t CFGR;			/*RCC clock configuration register				Address offset: 0x08*/
	__vo uint32_t CIR;			/*RCC clock interrupt register					Address offset: 0x0C*/
	__vo uint32_t AHB1RSTR;		/*RCC AHB1 peripheral reset register			Address offset: 0x10*/
	__vo uint32_t AHB2RSTR;		/*RCC AHB2 peripheral reset register			Address offset: 0x14*/
	__vo uint32_t AHB3RSTR;		/*RCC AHB3 peripheral reset register			Address offset: 0x18*/
	uint32_t      RESERVED1;	/*Reserved space: 0x1C*/
	__vo uint32_t APB1RSTR; 	/*RCC APB1 peripheral reset register			Address offset: 0x20*/
	__vo uint32_t APB2RSTR; 	/*RCC APB2 peripheral reset register			Address offset: 0x24*/
	uint32_t      RESERVED2;	/*Reserved space: 0x28*/
	uint32_t      RESERVED3;	/*Reserved space: 0x2C*/
	__vo uint32_t AHB1ENR;		/*RCC AHB1 peripheral clock enable register		Address offset: 0x30*/
	__vo uint32_t AHB2ENR;		/*RCC AHB2 peripheral clock enable register		Address offset: 0x34*/
	__vo uint32_t AHB3ENR;		/*RCC AHB3 peripheral clock enable register		Address offset: 0x38*/
	uint32_t      RESERVED4;	/*Reserved space: 0x3C*/
	__vo uint32_t APB1ENR;		/*RCC APB1 peripheral clock enable register		Address offset: 0x40*/
	__vo uint32_t APB2ENR;		/*RCC APB2 peripheral clock enable register		Address offset: 0x44*/
	uint32_t      RESERVED5;	/*Reserved space: 0x48*/
	uint32_t      RESERVED6;	/*Reserved space: 0x4C*/
	__vo uint32_t AHB1LPENR;	/*RCC AHB1 peripheral clock enable in 			Address offset: 0x50
								 *low power mode register*/
	__vo uint32_t AHB2LPENR;	/*RCC AHB2 peripheral clock enable in 			Address offset: 0x54
								 *low power mode register*/
	__vo uint32_t AHB3LPENR;	/*RCC AHB3 peripheral clock enable in 			Address offset: 0x58
								 *low power mode register*/
	uint32_t      RESERVED7;	/*Reserved space: 0x5C*/
	__vo uint32_t APB1LPENR;	/*RCC APB1 peripheral clock enable in 			Address offset: 0x60
								 *low power mode register*/
	__vo uint32_t APB2LPENR;	/*RCC APB2 peripheral clock enabled in			Address offset: 0x64
 	 	 	 	 	 	 	 	 *low power mode register*/
	uint32_t      RESERVED8;	/*Reserved space: 0x68*/
	uint32_t      RESERVED9;	/*Reserved space: 0x6C*/
	__vo uint32_t BDCR;			/*RCC Backup domain control register			Address offset: 0x70*/
	__vo uint32_t CSR;			/*RCC clock control & status register			Address offset: 0x74*/
	uint32_t      RESERVED10;	/*Reserved space: 0x78*/
	uint32_t      RESERVED11;	/*Reserved space: 0x7C*/
	__vo uint32_t SSCGR;		/*RCC spread spectrum clock generation register	Address offset: 0x80*/
	__vo uint32_t PLLI2SCFGR;	/*RCC PLLI2S configuration register				Address offset: 0x84*/
}RCC_RegDef_t;

//PLL Configuration Structure Definition
typedef struct
{
	uint8_t PLL_M;		/* Division factor for the main PLL (PLL) and
						 * audio PLL (PLLI2S) input clock.
						 * 2 <= PLLM <= 63								*/

	uint16_t PLL_N;		/* Main PLL (PLL) multiplication factor for VCO.
						 * 50 <= PLLN <= 432							*/

	uint8_t PLL_P;		/* Main PLL (PLL) division factor for main
						 * system clock.
						 * Possible values from @PLL_P					*/

	uint8_t PLL_SRC;	/* Main PLL(PLL) and audio PLL (PLLI2S) entry
						 * clock source.
						 * Possible values from @PLL_SRC				*/

	uint8_t PLL_Q;		/* Main PLL (PLL) division factor for USB OTG FS,
						 * SDIO and random number generator.
						 * 2 <= PLLQ <= 15								*/
}RCC_PLL_Config_t;

//RCC Configuration Structure Definition
typedef struct
{
	uint8_t RCC_ClockSource;			//@RCC_ClockSource
	uint32_t RCC_HSE_Frequency;			//4 - 26 MHz
	uint8_t RCC_AHB_Prescaler;			//@AHB_Prescaler
	uint8_t RCC_APB_LSPrescaler;		//@APB_LowSpeedPrescaler
	uint8_t RCC_APB_HSPrescaler;		//@APB_HighSpeedPrescaler
	uint8_t RCC_HSE_DivRTC;				//@HSE_DivisionFactorForRTC_Clock
	uint8_t RCC_MCO1_ClkOut;			//@MicrocontrollerClockOutput1
	uint8_t RCC_MCO2_ClkOut;			//@MicrocontrollerClockOutput2
	uint8_t RCC_MCO1_Prescaler;			//@MicrocontrollerPrescaler
	uint8_t RCC_MCO2_Prescaler;			//@MicrocontrollerPrescaler
	uint8_t RCC_I2S_ClkSel;				//@I2S_ClockSelection
	RCC_PLL_Config_t RCC_PLL_Config;
}RCC_Config_t;

typedef struct
{
	__vo uint32_t IMR;			/*Interrupt mask register						Address offset: 0x00*/
	__vo uint32_t EMR;			/*Event mask register							Address offset: 0x04*/
	__vo uint32_t RTSR;			/*Rising trigger selection register				Address offset: 0x08*/
	__vo uint32_t FTSR;			/*Falling trigger selection register			Address offset: 0x0C*/
	__vo uint32_t SWIER;		/*Software interrupt event register				Address offset: 0x10*/
	__vo uint32_t PR;			/*Pending register								Address offset: 0x14*/
}EXTI_RegDef_t;

typedef struct
{
	__vo uint32_t MEMRMP;		/*Memory re-map register						Address offset: 0x00*/
	__vo uint32_t PMC;			/*Peripheral mode configuration register		Address offset: 0x04*/
	__vo uint32_t EXTICR[4];	/*External interrupt configuration registers	Address offset: 0x08 - 0x14*/
	uint32_t	  RESERVED[2];	/*Reserved registers							Reserved: 0x18 - 0x1C*/
	__vo uint32_t CMPCR;		/*Compensation cell control register			Address offset: 0x20*/
}SYSCFG_RegDef_t;

typedef struct
{
	__vo uint32_t CR1;			/*SPI control register 1						Address offset: 0x00*/
	__vo uint32_t CR2;			/*SPI control register 2						Address offset: 0x04*/
	__vo uint32_t SR;			/*SPI status register							Address offset: 0x08*/
	__vo uint32_t DR;			/*SPI data register								Address offset: 0x0C*/
	__vo uint32_t CRCPR;		/*SPI CRC polynomial register					Address offset: 0x10*/
	__vo uint32_t RXCRCR;		/*SPI RX CRC register							Address offset: 0x14*/
	__vo uint32_t TXCRCR;		/*SPI TX CRC register							Address offset: 0x18*/
	__vo uint32_t I2SCFGR;		/*SPI_I2S configuration register				Address offset: 0x1C*/
	__vo uint32_t I2SPR;		/*SPI_I2S prescaler register					Address offset: 0x20*/
}SPI_RegDef_t;

typedef struct
{
	__vo uint32_t CR1;			/*I2C Control register 1						Address offset: 0x00*/
	__vo uint32_t CR2;			/*I2C Control register 2						Address offset: 0x04*/
	__vo uint32_t OAR1;			/*I2C Own address register 1					Address offset: 0x08*/
	__vo uint32_t OAR2;			/*I2C Own address register 2					Address offset: 0x0C*/
	__vo uint32_t DR;			/*I2C Data register								Address offset: 0x10*/
	__vo uint32_t SR1;			/*I2C Status register 1							Address offset: 0x14*/
	__vo uint32_t SR2;			/*I2C Status register 2							Address offset: 0x18*/
	__vo uint32_t CCR;			/*I2C Clock control register					Address offset: 0x1C*/
	__vo uint32_t TRISE;		/*I2C TRISE register							Address offset: 0x20*/
	__vo uint32_t FLTR;			/*I2C FLTR register								Address offset: 0x24*/
}I2C_RegDef_t;

typedef struct
{
	__vo uint32_t SR;			/*USART Status register							Address offset: 0x00*/
	__vo uint32_t DR;			/*USART Data register							Address offset: 0x04*/
	__vo uint32_t BRR;			/*USART Baud rate register						Address offset: 0x08*/
	__vo uint32_t CR1;			/*USART Control register 1						Address offset: 0x0C*/
	__vo uint32_t CR2;			/*USART Control register 2						Address offset: 0x10*/
	__vo uint32_t CR3;			/*USART Control register 3						Address offset: 0x14*/
	__vo uint32_t GTPR;			/*USART Guard time and prescaler register		Address offset: 0x18*/
}USART_RegDef_t;

typedef struct
{
	__vo uint32_t SR;			/*ADC status register							Address offset: 0x00*/
	__vo uint32_t CR1;			/*ADC control register 1						Address offset: 0x04*/
	__vo uint32_t CR2;			/*ADC control register 2						Address offset: 0x08*/
	__vo uint32_t SMPR1;		/*ADC sample time register 1					Address offset: 0x0C*/
	__vo uint32_t SMPR2;		/*ADC sample time register 2					Address offset: 0x10*/
	__vo uint32_t JOFR1;		/*ADC injected channel data offset register 1	Address offset: 0x14*/
	__vo uint32_t JOFR2;		/*ADC injected channel data offset register 2	Address offset: 0x18*/
	__vo uint32_t JOFR3;		/*ADC injected channel data offset register 3	Address offset: 0x1C*/
	__vo uint32_t JOFR4;		/*ADC injected channel data offset register 4	Address offset: 0x20*/
	__vo uint32_t HTR;			/*ADC watchdog higher threshold register		Address offset: 0x24*/
	__vo uint32_t LTR;			/*ADC watchdog lower threshold register			Address offset: 0x28*/
	__vo uint32_t SQR1;			/*ADC regular sequence register 1				Address offset: 0x2C*/
	__vo uint32_t SQR2;			/*ADC regular sequence register 2				Address offset: 0x30*/
	__vo uint32_t SQR3;			/*ADC regular sequence register 3				Address offset: 0x34*/
	__vo uint32_t JSQR;			/*ADC injected sequence register				Address offset: 0x38*/
	__vo uint32_t JDR1;			/*ADC injected data register 1					Address offset: 0x3C*/
	__vo uint32_t JDR2;			/*ADC injected data register 2					Address offset: 0x40*/
	__vo uint32_t JDR3;			/*ADC injected data register 3					Address offset: 0x44*/
	__vo uint32_t JDR4;			/*ADC injected data register 4					Address offset: 0x48*/
	__vo uint32_t DR;			/*ADC regular data register						Address offset: 0x4C*/
	__vo uint32_t CSR;			/*ADC Common status register					Address offset: 0x00
										(this offset address is relative to ADC1 base address + 0x300)*/
	__vo uint32_t CCR;			/*ADC common control register					Address offset: 0x04
										(this offset address is relative to ADC1 base address + 0x300)*/
	__vo uint32_t CDR;			/*ADC common regular data register for dual and triple modes
	 	 	 	 	 	 	Address offset: 0x08 (this offset address is relative to ADC1 base address + 0x300)*/
}ADC_RegDef_t;

typedef struct
{
	__vo uint32_t SxCR;			/*DMA stream x configuration register
									Address offset: 0x10 + 0x18 × stream number*/
	__vo uint32_t SxNDTR;		/*DMA stream x number of data register
									Address offset: 0x14 + 0x18 × stream number*/
	__vo uint32_t SxPAR;		/*DMA stream x peripheral address register
									Address offset: 0x18 + 0x18 × stream number*/
	__vo uint32_t SxM0AR;		/*DMA stream x memory 0 address register
									Address offset: 0x1C + 0x18 × stream number*/
	__vo uint32_t SxM1AR;		/*DMA stream x memory 1 address register
									Address offset: 0x20 + 0x18 × stream number*/
	__vo uint32_t SxFCR;		/*DMA stream x FIFO control register
									Address offset: 0x24 + 0x24 × stream number*/
}DMA_StreamX_RegDef_t;

typedef struct
{
	__vo uint32_t LISR;			/*DMA low interrupt status register				Address offset: 0x00*/
	__vo uint32_t HISR;			/*DMA high interrupt status register			Address offset: 0x04*/
	__vo uint32_t LIFCR;		/*DMA low interrupt flag clear register			Address offset: 0x08*/
	__vo uint32_t HIFCR;		/*DMA high interrupt flag clear register		Address offset: 0x0C*/
	__vo DMA_StreamX_RegDef_t DMA_Sx[8];	/*DMA Stream Number X				Address offset: 0x10*/
}DMA_RegDef_t;


//Peripheral Definitions (Peripheral base addresses type-casted to xxxx_RegDef_t).
#define GPIOA		((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB		((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC		((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD		((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE		((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF		((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG		((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH		((GPIO_RegDef_t*)GPIOH_BASEADDR)
#define GPIOI		((GPIO_RegDef_t*)GPIOI_BASEADDR)

#define RCC			((RCC_RegDef_t*)RCC_BASEADDR)

#define EXTI		((EXTI_RegDef_t*)EXTI_BASEADDR)

#define SYSCFG		((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

#define SPI1		((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2		((SPI_RegDef_t*)SPI2_I2S2_BASEADDR)
#define SPI3		((SPI_RegDef_t*)SPI3_I2S3_BASEADDR)
#define SPI4		((SPI_RegDef_t*)SPI4_BASEADDR)

#define I2C1		((I2C_RegDef_t*)I2C1_BASEADDR)
#define I2C2		((I2C_RegDef_t*)I2C2_BASEADDR)
#define I2C3		((I2C_RegDef_t*)I2C3_BASEADDR)

#define USART1		((USART_RegDef_t*)USART1_BASEADDR)
#define USART2		((USART_RegDef_t*)USART2_BASEADDR)
#define USART3		((USART_RegDef_t*)USART3_BASEADDR)
#define USART6		((USART_RegDef_t*)USART6_BASEADDR)

#define ADC1		((ADC_RegDef_t*)ADC1_BASEADDR)
#define ADC2		((ADC_RegDef_t*)ADC2_BASEADDR)
#define ADC3		((ADC_RegDef_t*)ADC3_BASEADDR)

#define DMA1		((DMA_RegDef_t*)DMA1_BASEADDR)
#define DMA2		((DMA_RegDef_t*)DMA2_BASEADDR)


//Clock Enable Macro for SYSCFG peripherals.
#define SYSCFG_PCLK_EN()	(RCC->APB2ENR |= (1 << 14))

//Clock Disable Macro for SYSCFG peripheral.
#define SYSCFG_PCLK_DI()	(RCC->APB2ENR &= ~(1 << 14))


//Returns port code for given GPIOx base address.
#define GPIO_BASEADDR_TO_CODE(x)	   ((x == GPIOA) ? 0 :\
										(x == GPIOB) ? 1 :\
										(x == GPIOC) ? 2 :\
										(x == GPIOD) ? 3 :\
										(x == GPIOE) ? 4 :\
										(x == GPIOF) ? 5 :\
										(x == GPIOG) ? 6 :\
										(x == GPIOH) ? 7 :\
										(x == GPIOI) ? 8 :0)


//IRQ(Interrupt Request) Numbers of STM32F407xx MCU.
#define IRQ_NO_EXTI0				6		/*!<EXTI Line0 interrupt>*/
#define IRQ_NO_EXTI1				7		/*!<EXTI Line1 interrupt>*/
#define IRQ_NO_EXTI2				8		/*!<EXTI Line2 interrupt>*/
#define IRQ_NO_EXTI3				9		/*!<EXTI Line3 interrupt>*/
#define IRQ_NO_EXTI4				10		/*!<EXTI Line4 interrupt>*/
#define IRQ_NO_EXTI9_5				23		/*!<EXTI Line[9:5] interrupts>*/
#define IRQ_NO_EXTI15_10			40		/*!<EXTI Line[15:10] interrupts>*/
#define IRQ_NO_TIM1_BRK_TIM9		24		/*!<TIM1 Break interrupt and TIM9 global
												interrupt>*/
#define IRQ_NO_TIM1_UP_TIM10		25		/*!<TIM1 Update interrupt and TIM10
												global interrupt>*/
#define IRQ_NO_TIM1_TRG_COM_TIM11	26		/*!<TIM1 Trigger and Commutation
												interrupts and TIM11 global interrupt>*/
#define IRQ_NO_TIM1_CC				27		/*!<TIM1 Capture Compare interrupt>*/
#define IRQ_NO_TIM2					28		/*!<TIM2 global interrupt>*/
#define IRQ_NO_TIM3					29		/*!<TIM3 global interrupt>*/
#define IRQ_NO_TIM4					30		/*!<TIM4 global interrupt>*/
#define IRQ_NO_TIM5					50		/*!<TIM5 global interrupt>*/
#define IRQ_NO_TIM6_DAC				54		/*!<TIM6 global interrupt,
												DAC1 and DAC2 underrun error interrupts>*/
#define IRQ_NO_TIM7					55		/*!<TIM7 global interrupt>*/
#define IRQ_NO_TIM8_BRK_TIM12		43		/*!<TIM8 Break interrupt and TIM12
												global interrupt>*/
#define IRQ_NO_TIM8_UP_TIM13		44		/*!<TIM8 Update interrupt and TIM13
												global interrupt>*/
#define IRQ_NO_TIM8_TRG_COM_TIM14	45		/*!<TIM8 Trigger and Commutation
												interrupts and TIM14 global interrupt>*/
#define IRQ_NO_TIM8_CC				46		/*!<TIM8 Capture Compare interrupt>*/
#define IRQ_NO_RCC					5		/*!<RCC global interrupt>*/
#define IRQ_NO_SPI1					35		/*!<SPI1 global interrupt>*/
#define IRQ_NO_SPI2					36		/*!<SPI2 global interrupt>*/
#define IRQ_NO_SPI3					51		/*!<SPI3 global interrupt>*/
#define IRQ_NO_I2C1_EV				31		/*!<I2C1 event interrupt>*/
#define IRQ_NO_I2C1_ER				32		/*!<I2C1 error interrupt>*/
#define IRQ_NO_I2C2_EV				33		/*!<I2C2 event interrupt>*/
#define IRQ_NO_I2C2_ER				34		/*!<I2C2 error interrupt>*/
#define IRQ_NO_I2C3_EV				72		/*!<I2C3 event interrupt>*/
#define IRQ_NO_I2C3_ER				73		/*!<I2C3 error interrupt>*/
#define IRQ_NO_USART1				37		/*!<USART1 global interrupt>*/
#define IRQ_NO_USART2				38		/*!<USART2 global interrupt>*/
#define IRQ_NO_USART3				39		/*!<USART3 global interrupt>*/
#define IRQ_NO_UART4				52		/*!<UART4 global interrupt>*/
#define IRQ_NO_UART5				53		/*!<UART5 global interrupt>*/
#define IRQ_NO_USART6				71		/*!<USART6 global interrupt>*/
#define IRQ_NO_ADC					18		/*!<ADC1, ADC2 and ADC3 global interrupts>*/
#define IRQ_NO_DMA1_STREAM0			11		/*!<DMA1 Stream0 global interrupt>*/
#define IRQ_NO_DMA1_STREAM1			12		/*!<DMA1 Stream1 global interrupt>*/
#define IRQ_NO_DMA1_STREAM2			13		/*!<DMA1 Stream2 global interrupt>*/
#define IRQ_NO_DMA1_STREAM3			14		/*!<DMA1 Stream3 global interrupt>*/
#define IRQ_NO_DMA1_STREAM4			15		/*!<DMA1 Stream4 global interrupt>*/
#define IRQ_NO_DMA1_STREAM5			16		/*!<DMA1 Stream5 global interrupt>*/
#define IRQ_NO_DMA1_STREAM6			17		/*!<DMA1 Stream6 global interrupt>*/
#define IRQ_NO_DMA1_STREAM7			47		/*!<DMA1 Stream7 global interrupt>*/
#define IRQ_NO_DMA2_STREAM0			56		/*!<DMA2 Stream0 global interrupt>*/
#define IRQ_NO_DMA2_STREAM1			57		/*!<DMA2 Stream1 global interrupt>*/
#define IRQ_NO_DMA2_STREAM2			58		/*!<DMA2 Stream2 global interrupt>*/
#define IRQ_NO_DMA2_STREAM3			59		/*!<DMA2 Stream3 global interrupt>*/
#define IRQ_NO_DMA2_STREAM4			60		/*!<DMA2 Stream4 global interrupt>*/
#define IRQ_NO_DMA2_STREAM5			68		/*!<DMA2 Stream5 global interrupt>*/
#define IRQ_NO_DMA2_STREAM6			69		/*!<DMA2 Stream6 global interrupt>*/
#define IRQ_NO_DMA2_STREAM7			70		/*!<DMA2 Stream7 global interrupt>*/

//IRQ Priority Levels of STM32F407xx MCU.
#define NVIC_IRQ_PRIORITY0		0
#define NVIC_IRQ_PRIORITY1		1
#define NVIC_IRQ_PRIORITY2		2
#define NVIC_IRQ_PRIORITY3		3
#define NVIC_IRQ_PRIORITY4		4
#define NVIC_IRQ_PRIORITY5		5
#define NVIC_IRQ_PRIORITY6		6
#define NVIC_IRQ_PRIORITY7		7
#define NVIC_IRQ_PRIORITY8		8
#define NVIC_IRQ_PRIORITY9		9
#define NVIC_IRQ_PRIORITY10		10
#define NVIC_IRQ_PRIORITY11		11
#define NVIC_IRQ_PRIORITY12		12
#define NVIC_IRQ_PRIORITY13		13
#define NVIC_IRQ_PRIORITY14		14
#define NVIC_IRQ_PRIORITY15		15


//Generic Macros
#define ENABLE				1
#define DISABLE				0
#define SET					ENABLE
#define RESET				DISABLE
#define GPIO_PIN_SET		SET
#define GPIO_PIN_RESET		RESET
#define FLAG_SET			SET
#define FLAG_RESET			RESET
#define INTERRUPT_ENABLE	ENABLE
#define INTERRUPT_DISABLE	DISABLE

#define _32MHZ		32000000U
#define _16MHZ		16000000U
#define  _8MHZ		 8000000U
#define  _4MHZ		 4000000U
#define  _2MHZ		 2000000U
#define  _1MHZ		 1000000U

#define _1NANO		1000000000U


/************************************************************************************************************************
*               Bit Position Definitions of SPI Peripherals                                                             *
************************************************************************************************************************/

//Bit position definitions for SPI_CR1
#define SPI_CR1_CPHA		0
#define SPI_CR1_CPOL		1
#define SPI_CR1_MSTR		2
#define SPI_CR1_BR			3
#define SPI_CR1_SPE 		6
#define SPI_CR1_LSBFIRST	7
#define SPI_CR1_SSI			8
#define SPI_CR1_SSM			9
#define SPI_CR1_RXONLY		10
#define SPI_CR1_DFF			11
#define SPI_CR1_CRCNEXT		12
#define SPI_CR1_CRCEN		13
#define SPI_CR1_BIDIOE		14
#define SPI_CR1_BIDIMODE	15

//Bit position definitions for SPI_CR2
#define SPI_CR2_RXDMAEN		0
#define SPI_CR2_TXDMAEN		1
#define SPI_CR2_SSOE		2
#define SPI_CR2_FRF			4
#define SPI_CR2_ERRIE		5
#define SPI_CR2_RXNEIE		6
#define SPI_CR2_TXEIE		7

//Bit position definitions for SPI_SR
#define SPI_SR_RXNE			0
#define SPI_SR_TXE			1
#define SPI_SR_CHSIDE		2
#define SPI_SR_UDR			3
#define SPI_SR_CRCERR		4
#define SPI_SR_MODF			5
#define SPI_SR_OVR			6
#define SPI_SR_BSY			7
#define SPI_SR_FRE			8


/************************************************************************************************************************
*               Bit Position Definitions of RCC Registers                                                               *
************************************************************************************************************************/

//Bit position definitions for RCC_CR
#define RCC_CR_HSION		0
#define RCC_CR_HSIRDY		1
#define RCC_CR_HSITRIM		3
#define RCC_CR_HSICAL		8
#define RCC_CR_HSEON		16
#define RCC_CR_HSERDY		17
#define RCC_CR_HSEBYP		18
#define RCC_CR_CSSON		19
#define RCC_CR_PLLON		24
#define RCC_CR_PLLRDY		25
#define RCC_CR_PLLI2SON		26
#define RCC_CR_PLLI2SRDY	27

//Bit position definitions for RCC_PLLCFGR
#define RCC_PLLCFGR_PLLM0	0
#define RCC_PLLCFGR_PLLM1	1
#define RCC_PLLCFGR_PLLM2	2
#define RCC_PLLCFGR_PLLM3	3
#define RCC_PLLCFGR_PLLM4	4
#define RCC_PLLCFGR_PLLM5	5
#define RCC_PLLCFGR_PLLN	6
#define RCC_PLLCFGR_PLLP0	16
#define RCC_PLLCFGR_PLLP1	17
#define RCC_PLLCFGR_PLLSRC	22
#define RCC_PLLCFGR_PLLQ0	24
#define RCC_PLLCFGR_PLLQ1	25
#define RCC_PLLCFGR_PLLQ2	26
#define RCC_PLLCFGR_PLLQ3	27

//Bit position definitions for RCC_CFGR
#define RCC_CFGR_SW0		0
#define RCC_CFGR_SW1		1
#define RCC_CFGR_SWS0		2
#define RCC_CFGR_SWS1		3
#define RCC_CFGR_HPRE		4
#define RCC_CFGR_PPRE1		10
#define RCC_CFGR_PPRE2		13
#define RCC_CFGR_RTCPRE		16
#define RCC_CFGR_MCO1		21
#define RCC_CFGR_I2SSCR		23
#define RCC_CFGR_MCO1PRE	24
#define RCC_CFGR_MCO2PRE	27
#define RCC_CFGR_MCO2		30

//Bit position definitions for RCC_CIR
#define RCC_CIR_LSIRDYF		0
#define RCC_CIR_LSERDYF		1
#define RCC_CIR_HSIRDYF		2
#define RCC_CIR_HSERDYF		3
#define RCC_CIR_PLLRDYF		4
#define RCC_CIR_PLLI2SRDYF	5
#define RCC_CIR_CSSF		7
#define RCC_CIR_LSIRDYIE	8
#define RCC_CIR_LSERDYIE	9
#define RCC_CIR_HSIRDYIE	10
#define RCC_CIR_HSERDYIE	11
#define RCC_CIR_PLLRDYIE	12
#define RCC_CIR_PLLI2SRDYIE 13
#define RCC_CIR_LSIRDYC		16
#define RCC_CIR_LSERDYC		17
#define RCC_CIR_HSIRDYC		18
#define RCC_CIR_HSERDYC		19
#define RCC_CIR_PLLRDYC		20
#define RCC_CIR_PLLI2SRDYC	21
#define RCC_CIR_CSSC		23

//Bit position definitions for RCC_AHB1RSTR
#define RCC_AHB1RSTR_GPIOARST		0
#define RCC_AHB1RSTR_GPIOBRST		1
#define RCC_AHB1RSTR_GPIOCRST		2
#define RCC_AHB1RSTR_GPIODRST		3
#define RCC_AHB1RSTR_GPIOERST		4
#define RCC_AHB1RSTR_GPIOFRST		5
#define RCC_AHB1RSTR_GPIOGRST		6
#define RCC_AHB1RSTR_GPIOHRST		7
#define RCC_AHB1RSTR_GPIOIRST		8
#define RCC_AHB1RSTR_CRCRST			12
#define RCC_AHB1RSTR_DMA1RST		21
#define RCC_AHB1RSTR_DMA2RST		22
#define RCC_AHB1RSTR_ETHMACRST		25
#define RCC_AHB1RSTR_OTGHSRST		29

//Bit position definitions for RCC_AHB2RSTR
#define RCC_AHB2RSTR_DCMIRST		0
#define RCC_AHB2RSTR_CRYPRST		4
#define RCC_AHB2RSTR_HASHRST		5
#define RCC_AHB2RSTR_RNGRST			6
#define RCC_AHB2RSTR_OTGFSRST		7

//Bit position definitions for RCC_AHB3RSTR
#define RCC_AHB3RSTR_FSMCRST		0

//Bit position definitions for RCC_APB1RSTR
#define RCC_APB1RSTR_TIM2RST		0
#define RCC_APB1RSTR_TIM3RST		1
#define RCC_APB1RSTR_TIM4RST		2
#define RCC_APB1RSTR_TIM5RST		3
#define RCC_APB1RSTR_TIM6RST		4
#define RCC_APB1RSTR_TIM7RST		5
#define RCC_APB1RSTR_TIM12RST		6
#define RCC_APB1RSTR_TIM13RST		7
#define RCC_APB1RSTR_TIM14RST		8
#define RCC_APB1RSTR_WWDGRST		11
#define RCC_APB1RSTR_SPI2RST		14
#define RCC_APB1RSTR_SPI3RST		15
#define RCC_APB1RSTR_UART2RST		17
#define RCC_APB1RSTR_UART3RST		18
#define RCC_APB1RSTR_UART4RST		19
#define RCC_APB1RSTR_UART5RST		20
#define RCC_APB1RSTR_I2C1RST		21
#define RCC_APB1RSTR_I2C2RST		22
#define RCC_APB1RSTR_I2C3RST		23
#define RCC_APB1RSTR_CAN1RST		25
#define RCC_APB1RSTR_CAN2RST		26
#define RCC_APB1RSTR_PWRRST			28
#define RCC_APB1RSTR_DACRST			29

//Bit position definitions for RCC_APB2RSTR
#define RCC_APB2RSTR_TIM1RST		0
#define RCC_APB2RSTR_TIM8RST		1
#define RCC_APB2RSTR_USART1RST		4
#define RCC_APB2RSTR_USART6RST		5
#define RCC_APB2RSTR_ADCRST			8
#define RCC_APB2RSTR_SDIORST		11
#define RCC_APB2RSTR_SPI1RST		12
#define RCC_APB2RSTR_SYSCFGRST		14
#define RCC_APB2RSTR_TIM9RST		16
#define RCC_APB2RSTR_TIM10RST		17
#define RCC_APB2RSTR_TIM11RST		18

//Bit position definitions for RCC_AHB1ENR
#define RCC_AHB1ENR_GPIOAEN			0
#define RCC_AHB1ENR_GPIOBEN			1
#define RCC_AHB1ENR_GPIOCEN			2
#define RCC_AHB1ENR_GPIODEN			3
#define RCC_AHB1ENR_GPIOEEN			4
#define RCC_AHB1ENR_GPIOFEN			5
#define RCC_AHB1ENR_GPIOGEN			6
#define RCC_AHB1ENR_GPIOHEN			7
#define RCC_AHB1ENR_GPIOIEN			8
#define RCC_AHB1ENR_CRCEN			12
#define RCC_AHB1ENR_BKPSRAMEN		18
#define RCC_AHB1ENR_CCMDATARAMEN	20
#define RCC_AHB1ENR_DMA1EN			21
#define RCC_AHB1ENR_DMA2EN			22
#define RCC_AHB1ENR_ETHMACEN		25
#define RCC_AHB1ENR_ETHMACTXEN		26
#define RCC_AHB1ENR_ETHMACRXEN		27
#define RCC_AHB1ENR_ETHMACPTPEN		28
#define RCC_AHB1ENR_OTGHSEN			29
#define RCC_AHB1ENR_OTGHSULPIEN		30

//Bit position definitions for RCC_AHB2ENR
#define RCC_AHB2ENR_DCMIEN			0
#define RCC_AHB2ENR_CRPYEN			4
#define RCC_AHB2ENR_HASHEN			5
#define RCC_AHB2ENR_RNGEN			6
#define RCC_AHB2ENR_OTGFSEN			7

//Bit position definitions for RCC_AHB3ENR
#define RCC_AHB2ENR_FSMCEN			0

//Bit position definitions for RCC_APB1ENR
#define RCC_APB1ENR_TIM2EN			0
#define RCC_APB1ENR_TIM3EN			1
#define RCC_APB1ENR_TIM4EN			2
#define RCC_APB1ENR_TIM5EN			3
#define RCC_APB1ENR_TIM6EN			4
#define RCC_APB1ENR_TIM7EN			5
#define RCC_APB1ENR_TIM12EN			6
#define RCC_APB1ENR_TIM13EN			7
#define RCC_APB1ENR_TIM14EN			8
#define RCC_APB1ENR_WWDGEN			11
#define RCC_APB1ENR_SPI2EN			14
#define RCC_APB1ENR_SPI3EN			15
#define RCC_APB1ENR_USART2EN		17
#define RCC_APB1ENR_USART3EN		18
#define RCC_APB1ENR_UART4EN			19
#define RCC_APB1ENR_UART5EN			20
#define RCC_APB1ENR_I2C1EN			21
#define RCC_APB1ENR_I2C2EN			22
#define RCC_APB1ENR_I2C3EN			23
#define RCC_APB1ENR_CAN1EN			25
#define RCC_APB1ENR_CAN2EN			26
#define RCC_APB1ENR_PWREN			28
#define RCC_APB1ENR_DACEN			29

//Bit position definitions for RCC_APB2ENR
#define RCC_APB2ENR_TIM1EN			0
#define RCC_APB2ENR_TIM8EN			1
#define RCC_APB2ENR_USART1EN		4
#define RCC_APB2ENR_USART6EN		5
#define RCC_APB2ENR_ADC1EN			8
#define RCC_APB2ENR_ADC2EN			9
#define RCC_APB2ENR_ADC3EN			10
#define RCC_APB2ENR_SDIOEN			11
#define RCC_APB2ENR_SPI1EN			12
#define RCC_APB2ENR_SYSCFGEN		14
#define RCC_APB2ENR_TIM9EN			16
#define RCC_APB2ENR_TIM10EN			17
#define RCC_APB2ENR_TIM11EN			18

//Bit position definitions for RCC_AHB1LPENR
#define RCC_AHB1LPENR_GPIOALPEN		0
#define RCC_AHB1LPENR_GPIOBLPEN		1
#define RCC_AHB1LPENR_GPIOCLPEN		2
#define RCC_AHB1LPENR_GPIODLPEN		3
#define RCC_AHB1LPENR_GPIOELPEN		4
#define RCC_AHB1LPENR_GPIOFLPEN		5
#define RCC_AHB1LPENR_GPIOGLPEN		6
#define RCC_AHB1LPENR_GPIOHLPEN		7
#define RCC_AHB1LPENR_GPIOILPEN		8
#define RCC_AHB1LPENR_CRCLPEN		12
#define RCC_AHB1LPENR_FLITFLPEN		15
#define RCC_AHB1LPENR_SRAM1LPEN		16
#define RCC_AHB1LPENR_SRAM2LPEN		17
#define RCC_AHB1LPENR_BKPSRAMLPEN	18
#define RCC_AHB1LPENR_DMA1LPEN		21
#define RCC_AHB1LPENR_DMA2LPEN		22
#define RCC_AHB1LPENR_ETHMACLPEN	25
#define RCC_AHB1LPENR_ETHTXLPEN		26
#define RCC_AHB1LPENR_ETHRXLPEN		27
#define RCC_AHB1LPENR_ETHPTPLPEN	28
#define RCC_AHB1LPENR_OTGHSLPEN		29
#define RCC_AHB1LPENR_OTGHSULPILPEN	30

//Bit position definitions for RCC_AHB2LPENR
#define RCC_AHB2LPENR_DCMILPEN		0
#define RCC_AHB2LPENR_CRYPLPEN		4
#define RCC_AHB2LPENR_HASHLPEN		5
#define RCC_AHB2LPENR_RNGPLEN		6
#define RCC_AHB2LPENR_OTGFSLPEN		7

//Bit position definitions for RCC_AHB3LPENR
#define RCC_AHB3LPENR_FSMCLPEN		0

//Bit position definitions for RCC_APB1LPENR
#define RCC_APB1LPENR_TIM2LPEN		0
#define RCC_APB1LPENR_TIM3LPEN		1
#define RCC_APB1LPENR_TIM4LPEN		2
#define RCC_APB1LPENR_TIM5LPEN		3
#define RCC_APB1LPENR_TIM6LPEN		4
#define RCC_APB1LPENR_TIM7LPEN		5
#define RCC_APB1LPENR_TIM12LPEN		6
#define RCC_APB1LPENR_TIM13LPEN		7
#define RCC_APB1LPENR_TIM14LPEN		8
#define RCC_APB1LPENR_WWDGLPEN		11
#define RCC_APB1LPENR_SPI2LPEN		14
#define RCC_APB1LPENR_SPI3LPEN		15
#define RCC_APB1LPENR_USART2LPEN	17
#define RCC_APB1LPENR_USART3LPEN	18
#define RCC_APB1LPENR_UART4LPEN		19
#define RCC_APB1LPENR_UART5LPEN		20
#define RCC_APB1LPENR_I2C1LPEN		21
#define RCC_APB1LPENR_I2C2LPEN		22
#define RCC_APB1LPENR_I2C3LPEN		23
#define RCC_APB1LPENR_CAN1LPEN		25
#define RCC_APB1LPENR_CAN2LPEN		26
#define RCC_APB1LPENR_PWRLPEN		28
#define RCC_APB1LPENR_DACLPEN		29

//Bit position definitions for RCC_APB2LPENR
#define RCC_APB2LPENR_TIM1LPEN		0
#define RCC_APB2LPENR_TIM8LPEN		1
#define RCC_APB2LPENR_USART1LPEN	4
#define RCC_APB2LPENR_USART6LPEN	5
#define RCC_APB2LPENR_ADC1LPEN		8
#define RCC_APB2LPENR_ADC2LPEN		9
#define RCC_APB2LPENR_ADC3LPEN		10
#define RCC_APB2LPENR_SDIOLPEN		11
#define RCC_APB2LPENR_SPI1LPEN		12
#define RCC_APB2LPENR_SYSCFGLPEN	14
#define RCC_APB2LPENR_TIM9LPEN		16
#define RCC_APB2LPENR_TIM10LPEN		17
#define RCC_APB2LPENR_TIM11LPEN		18

//Bit position definitions for RCC_BDCR
#define RCC_BDCR_LSEON				0
#define RCC_BDCR_LSERDY				1
#define RCC_BDCR_LSEBYP				2
#define RCC_BDCR_RTCSEL				8
#define RCC_BDCR_RTCEN				15
#define RCC_BDCR_BDRST				16

//Bit position definitions for RCC_CSR
#define RCC_CSR_LSION				0
#define RCC_CSR_LSIRDY				1
#define RCC_CSR_RMVF				24
#define RCC_CSR_BORRSTF				25
#define RCC_CSR_PINRSTF				26
#define RCC_CSR_PORRSTF				27
#define RCC_CSR_SFTRSTF				28
#define RCC_CSR_IWDGRSTF			29
#define RCC_CSR_WWDGRSTF			30
#define RCC_CSR_LPWRRSTF			31

//Bit position definitions for RCC_SSCGR
#define RCC_SSCGR_MODPER			0
#define RCC_SSCGR_INCSTEP			13
#define RCC_SSCGR_SPREADSEL			30
#define RCC_SSCGR_SSCGEN			31

//Bit position definitions for RCC_PLLI2SCFGR
#define RCC_PLLI2SCFGR_PLLI2SN0		6
#define RCC_PLLI2SCFGR_PLLI2SN1		7
#define RCC_PLLI2SCFGR_PLLI2SN2		8
#define RCC_PLLI2SCFGR_PLLI2SN3		9
#define RCC_PLLI2SCFGR_PLLI2SN4		10
#define RCC_PLLI2SCFGR_PLLI2SN5		11
#define RCC_PLLI2SCFGR_PLLI2SN6		12
#define RCC_PLLI2SCFGR_PLLI2SN7		13
#define RCC_PLLI2SCFGR_PLLI2SN8		14
#define RCC_PLLI2SCFGR_PLLI2SR0		28
#define RCC_PLLI2SCFGR_PLLI2SR1		29
#define RCC_PLLI2SCFGR_PLLI2SR2		30


/************************************************************************************************************************
*               Bit Position Definitions of I2Cx Peripherals                                                            *
************************************************************************************************************************/

//Bit position definitions for I2C_CR1
#define I2C_CR1_PE			0
#define I2C_CR1_SMBUS		1
#define I2C_CR1_SMBTYPE		3
#define I2C_CR1_ENARP		4
#define I2C_CR1_ENPEC		5
#define I2C_CR1_ENGC		6
#define I2C_CR1_NOSTRETCH	7
#define I2C_CR1_START		8
#define I2C_CR1_STOP		9
#define I2C_CR1_ACK			10
#define I2C_CR1_POS			11
#define I2C_CR1_PEC			12
#define I2C_CR1_ALERT		13
#define I2C_CR1_SWRST		15

//Bit position definitions for I2C_CR2
#define I2C_CR2_FREQ		0
#define I2C_CR2_ITERREN		8
#define I2C_CR2_ITEVTEN		9
#define I2C_CR2_ITBUFEN		10
#define I2C_CR2_DMAEN		11
#define I2C_CR2_LAST		12

//Bit position definitions for I2C_OAR1
#define I2C_OAR1_ADD_0		0
#define I2C_OAR1_ADD_7_1	1
#define I2C_OAR1_ADD_9_8	8
#define I2C_OAR1_RES_BIT14	14
#define I2C_OAR1_ADDMODE	15

//Bit position definitions for I2C_OAR2
#define I2C_OAR2_ENDUAL		0
#define I2C_OAR2_ADD2		1

//Bit position definitions for I2C_DR
#define I2C_DR_DR			0

//Bit position definitions for I2C_SR1
#define I2C_SR1_SB			0
#define I2C_SR1_ADDR		1
#define I2C_SR1_BTF			2
#define I2C_SR1_ADD10		3
#define I2C_SR1_STOPF		4
#define I2C_SR1_RXNE		6
#define I2C_SR1_TXE			7
#define I2C_SR1_BERR		8
#define I2C_SR1_ARLO		9
#define I2C_SR1_AF			10
#define I2C_SR1_OVR			11
#define I2C_SR1_PECERR		12
#define I2C_SR1_TIMEOUT		14
#define I2C_SR1_SMBALERT	15

//Bit position definitions for I2C_SR2
#define I2C_SR2_MSL			0
#define I2C_SR2_BUSY		1
#define I2C_SR2_TRA			2
#define I2C_SR2_GENCALL		4
#define I2C_SR2_SMBDEFAULT	5
#define I2C_SR2_SMBHOST		6
#define I2C_SR2_DUALF		7
#define I2C_SR2_PEC			8

//Bit position definitions for I2C_CCR
#define I2C_CCR_CCR			0
#define I2C_CCR_DUTY		14
#define I2C_CCR_FS			15

//Bit position definitions for I2C_TRISE
#define I2C_TRISE_TRISE		0

//Bit position definitions for I2C_FLTR
#define I2C_FLTR_DNF		0
#define I2C_FLTR_ANOFF		4


/************************************************************************************************************************
*               Bit Position Definitions of USARTx Peripherals                                                          *
************************************************************************************************************************/

//Bit position definitions for USART_SR
#define USART_SR_PE			0
#define USART_SR_FE			1
#define USART_SR_NF			2
#define USART_SR_ORE		3
#define USART_SR_IDLE		4
#define USART_SR_RXNE		5
#define USART_SR_TC			6
#define USART_SR_TXE		7
#define USART_SR_LBD		8
#define USART_SR_CTS		9

//Bit position definitions for USART_DR
#define USART_DR_DR			0

//Bit position definitions for USART_BRR
#define USART_BRR_DIV_F		0
#define USART_BRR_DIV_M		4

//Bit position definitions for USART_CR1
#define USART_CR1_SBK		0
#define USART_CR1_RWU		1
#define USART_CR1_RE		2
#define USART_CR1_TE		3
#define USART_CR1_IDLEIE	4
#define USART_CR1_RXNEIE	5
#define USART_CR1_TCIE		6
#define USART_CR1_TXEIE		7
#define USART_CR1_PEIE		8
#define USART_CR1_PS		9
#define USART_CR1_PCE		10
#define USART_CR1_WAKE		11
#define USART_CR1_M			12
#define USART_CR1_UE		13
#define USART_CR1_OVER8		15

//Bit position definitions for USART_CR2
#define USART_CR2_ADD		0
#define USART_CR2_LBDL		5
#define USART_CR2_LBDIE		6
#define USART_CR2_LBCL		8
#define USART_CR2_CPHA		9
#define USART_CR2_CPOL		10
#define USART_CR2_CLKEN		11
#define USART_CR2_STOP		12
#define USART_CR2_LINEN		14

//Bit position definitions for USART_CR3
#define USART_CR3_EIE		0
#define USART_CR3_IREN		1
#define USART_CR3_IRLP		2
#define USART_CR3_HDSEL		3
#define USART_CR3_NACK		4
#define USART_CR3_SCEN		5
#define USART_CR3_DMAR		6
#define USART_CR3_DMAT		7
#define USART_CR3_RTSE		8
#define USART_CR3_CTSE		9
#define USART_CR3_CTSIE		10
#define USART_CR3_ONEBIT	11

//Bit position definitions for USART_GTPR
#define USART_GTPR_PSC		0
#define USART_GTPR_GT		8


/************************************************************************************************************************
*               Bit Position Definitions of ADCx Peripherals                                                            *
************************************************************************************************************************/

//Bit position definitions for ADC_SR
#define ADC_SR_AWD			0
#define ADC_SR_EOC			1
#define ADC_SR_JEOC			2
#define ADC_SR_JSTRT		3
#define ADC_SR_STRT			4
#define ADC_SR_OVR			5

//Bit position definitions for ADC_CR1
#define ADC_CR1_AWDCH		0
#define ADC_CR1_EOCIE		5
#define ADC_CR1_AWDIE		6
#define ADC_CR1_JEOCIE		7
#define ADC_CR1_SCAN		8
#define ADC_CR1_AWDSGL		9
#define ADC_CR1_JAUTO		10
#define ADC_CR1_DISCEN		11
#define ADC_CR1_JDISCEN		12
#define ADC_CR1_DISCNUM		13
#define ADC_CR1_JAWDEN		22
#define ADC_CR1_AWDEN		23
#define ADC_CR1_RES			24
#define ADC_CR1_OVRIE		26

//Bit position definitions for ADC_CR2
#define ADC_CR2_ADON		0
#define ADC_CR2_CONT		1
#define ADC_CR2_DMA			8
#define ADC_CR2_DDS			9
#define ADC_CR2_EOCS		10
#define ADC_CR2_ALIGN		11
#define ADC_CR2_JEXTSEL		16
#define ADC_CR2_JEXTEN		20
#define ADC_CR2_JSWSTART	22
#define ADC_CR2_EXTSEL		24
#define ADC_CR2_EXTEN		28
#define ADC_CR2_SWSTART		30

//Bit position definitions for ADC_SMPR1
#define ADC_SMPR1_SMP10		0
#define ADC_SMPR1_SMP11		3
#define ADC_SMPR1_SMP12		6
#define ADC_SMPR1_SMP13		9
#define ADC_SMPR1_SMP14		12
#define ADC_SMPR1_SMP15		15
#define ADC_SMPR1_SMP16		18
#define ADC_SMPR1_SMP17		21
#define ADC_SMPR1_SMP18		24

//Bit position definitions for ADC_SMPR2
#define ADC_SMPR2_SMP0		0
#define ADC_SMPR2_SMP1		3
#define ADC_SMPR2_SMP2		6
#define ADC_SMPR2_SMP3		9
#define ADC_SMPR2_SMP4		12
#define ADC_SMPR2_SMP5		15
#define ADC_SMPR2_SMP6		18
#define ADC_SMPR2_SMP7		21
#define ADC_SMPR2_SMP8		24
#define ADC_SMPR2_SMP9		27

//Bit position definitions for ADC_JOFRx (x = 1 ... 4)
#define ADC_JOFRX_JOFFSET0	0
#define ADC_JOFRX_JOFFSET1	1
#define ADC_JOFRX_JOFFSET2	2
#define ADC_JOFRX_JOFFSET3	3
#define ADC_JOFRX_JOFFSET4	4
#define ADC_JOFRX_JOFFSET5	5
#define ADC_JOFRX_JOFFSET6	6
#define ADC_JOFRX_JOFFSET7	7
#define ADC_JOFRX_JOFFSET8	8
#define ADC_JOFRX_JOFFSET9	9
#define ADC_JOFRX_JOFFSET10	10
#define ADC_JOFRX_JOFFSET11	11

//Bit position definitions for ADC_HTR
#define ADC_HTR_HT0			0
#define ADC_HTR_HT1			1
#define ADC_HTR_HT2			2
#define ADC_HTR_HT3			3
#define ADC_HTR_HT4			4
#define ADC_HTR_HT5			5
#define ADC_HTR_HT6			6
#define ADC_HTR_HT7			7
#define ADC_HTR_HT8			8
#define ADC_HTR_HT9			9
#define ADC_HTR_HT10		10
#define ADC_HTR_HT11		11

//Bit position definitions for ADC_LTR
#define ADC_LTR_LT0			0
#define ADC_LTR_LT1			1
#define ADC_LTR_LT2			2
#define ADC_LTR_LT3			3
#define ADC_LTR_LT4			4
#define ADC_LTR_LT5			5
#define ADC_LTR_LT6			6
#define ADC_LTR_LT7			7
#define ADC_LTR_LT8			8
#define ADC_LTR_LT9			9
#define ADC_LTR_LT10		10
#define ADC_LTR_LT11		11

//Bit position definitions for ADC_SQR1
#define ADC_SQR1_SQ13		0
#define ADC_SQR1_SQ14		5
#define ADC_SQR1_SQ15		10
#define ADC_SQR1_SQ16		15
#define ADC_SQR1_L			20

//Bit position definitions for ADC_SQR2
#define ADC_SQR2_SQ7		0
#define ADC_SQR2_SQ8		5
#define ADC_SQR2_SQ9		10
#define ADC_SQR2_SQ10		15
#define ADC_SQR2_SQ11		20
#define ADC_SQR2_SQ12		25

//Bit position definitions for ADC_SQR3
#define ADC_SQR3_SQ1		0
#define ADC_SQR3_SQ2		5
#define ADC_SQR3_SQ3		10
#define ADC_SQR3_SQ4		15
#define ADC_SQR3_SQ5		20
#define ADC_SQR3_SQ6		25

//Bit position definitions for ADC_JSQR
#define ADC_JSQR_JSQ1		0
#define ADC_JSQR_JSQ2		5
#define ADC_JSQR_JSQ3		10
#define ADC_JSQR_JSQ4		15
#define ADC_JSQR_JL			20

//Bit position definitions for ADC_CCR
#define ADC_CCR_MULTI		0
#define ADC_CCR_DELAY		8
#define ADC_CCR_DDS			13
#define ADC_CCR_DMA			14
#define ADC_CCR_ADCPRE		16
#define ADC_CCR_VBATE		22
#define ADC_CCR_TSVREFE		23


/************************************************************************************************************************
*               Bit Position Definitions of DMAx Peripherals                                                            *
************************************************************************************************************************/

//Bit position definitions for DMA_LISR
#define DMA_LISR_FEIF0		0
#define DMA_LISR_DMEIF0		2
#define DMA_LISR_TEIF0		3
#define DMA_LISR_HTIF0		4
#define DMA_LISR_TCIF0		5
#define DMA_LISR_FEIF1		6
#define DMA_LISR_DMEIF1		8
#define DMA_LISR_TEIF1		9
#define DMA_LISR_HTIF1		10
#define DMA_LISR_TCIF1		11
#define DMA_LISR_FEIF2		16
#define DMA_LISR_DMEIF2		18
#define DMA_LISR_TEIF2		19
#define DMA_LISR_HTIF2		20
#define DMA_LISR_TCIF2		21
#define DMA_LISR_FEIF3		22
#define DMA_LISR_DMEIF3		24
#define DMA_LISR_TEIF3		25
#define DMA_LISR_HTIF3		26
#define DMA_LISR_TCIF3		27

//Bit position definitions for DMA_HISR
#define DMA_HISR_FEIF4		0
#define DMA_HISR_DMEIF4		2
#define DMA_HISR_TEIF4		3
#define DMA_HISR_HTIF4		4
#define DMA_HISR_TCIF4		5
#define DMA_HISR_FEIF5		6
#define DMA_HISR_DMEIF5		8
#define DMA_HISR_TEIF5		9
#define DMA_HISR_HTIF5		10
#define DMA_HISR_TCIF5		11
#define DMA_HISR_FEIF6		16
#define DMA_HISR_DMEIF6		18
#define DMA_HISR_TEIF6		19
#define DMA_HISR_HTIF6		20
#define DMA_HISR_TCIF6		21
#define DMA_HISR_FEIF7		22
#define DMA_HISR_DMEIF7		24
#define DMA_HISR_TEIF7		25
#define DMA_HISR_HTIF7		26
#define DMA_HISR_TCIF7		27

//Bit position definitions for DMA_LIFCR
#define DMA_LIFCR_CFEIF0		0
#define DMA_LIFCR_CDMEIF0		2
#define DMA_LIFCR_CTEIF0		3
#define DMA_LIFCR_CHTIF0		4
#define DMA_LIFCR_CTCIF0		5
#define DMA_LIFCR_CFEIF1		6
#define DMA_LIFCR_CDMEIF1		8
#define DMA_LIFCR_CTEIF1		9
#define DMA_LIFCR_CHTIF1		10
#define DMA_LIFCR_CTCIF1		11
#define DMA_LIFCR_CFEIF2		16
#define DMA_LIFCR_CDMEIF2		18
#define DMA_LIFCR_CTEIF2		19
#define DMA_LIFCR_CHTIF2		20
#define DMA_LIFCR_CTCIF2		21
#define DMA_LIFCR_CFEIF3		22
#define DMA_LIFCR_CDMEIF3		24
#define DMA_LIFCR_CTEIF3		25
#define DMA_LIFCR_CHTIF3		26
#define DMA_LIFCR_CTCIF3		27

//Bit position definitions for DMA_HIFCR
#define DMA_HIFCR_CFEIF4		0
#define DMA_HIFCR_CDMEIF4		2
#define DMA_HIFCR_CTEIF4		3
#define DMA_HIFCR_CHTIF4		4
#define DMA_HIFCR_CTCIF4		5
#define DMA_HIFCR_CFEIF5		6
#define DMA_HIFCR_CDMEIF5		8
#define DMA_HIFCR_CTEIF5		9
#define DMA_HIFCR_CHTIF5		10
#define DMA_HIFCR_CTCIF5		11
#define DMA_HIFCR_CFEIF6		16
#define DMA_HIFCR_CDMEIF6		18
#define DMA_HIFCR_CTEIF6		19
#define DMA_HIFCR_CHTIF6		20
#define DMA_HIFCR_CTCIF6		21
#define DMA_HIFCR_CFEIF7		22
#define DMA_HIFCR_CDMEIF7		24
#define DMA_HIFCR_CTEIF7		25
#define DMA_HIFCR_CHTIF7		26
#define DMA_HIFCR_CTCIF7		27

//Bit position definitions for DMA_SXCR
#define DMA_SXCR_EN				0
#define DMA_SXCR_DMEIE			1
#define DMA_SXCR_TEIE			2
#define DMA_SXCR_HTIE			3
#define DMA_SXCR_TCIE			4
#define DMA_SXCR_PFCTRL			5
#define DMA_SXCR_DIR			6
#define DMA_SXCR_CIRC			8
#define DMA_SXCR_PINC			9
#define DMA_SXCR_MINC			10
#define DMA_SXCR_PSIZE			11
#define DMA_SXCR_MSIZE			13
#define DMA_SXCR_PINCOS			15
#define DMA_SXCR_PL				16
#define DMA_SXCR_DBM			18
#define DMA_SXCR_CT				19
#define DMA_SXCR_PBURST			21
#define DMA_SXCR_MBURST			23
#define DMA_SXCR_CHSEL			25

//Bit position definitions for DMA_SXNDTR
#define DMA_SXNDTR_NDT			0

//Bit position definitions for DMA_SXPAR
#define DMA_SXPAR_PAR			0

//Bit position definitions for DMA_SXM0AR
#define DMA_SXM0AR_M0A			0

//Bit position definitions for DMA_SXM1AR
#define DMA_SXM1AR_M1A			0

//Bit position definitions for DMA_SXFCR
#define DMA_SXFCR_FTH			0
#define DMA_SXFCR_DMDIS			2
#define DMA_SXFCR_FS			3
#define DMA_SXFCR_FEIE			7


//Driver includes
#include"stm32f407xx_gpio_driver.h"
#include"stm32f407xx_spi_driver.h"
#include"stm32f407xx_i2c_driver.h"
#include"stm32f407xx_usart_driver.h"
#include"stm32f407xx_rcc_driver.h"
#include"stm32f407xx_adc_driver.h"
#include"stm32f407xx_dma_driver.h"

#endif /* INC_STM32F407XX_H_ */
